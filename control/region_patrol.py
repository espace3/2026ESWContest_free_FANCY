"""
control/region_patrol.py

부위 인식 모드(0x03)의 순수 계산 모듈 — GPIO/BlueZ/카메라를 import하지 않는다.
입력은 프레임별 관측 dict(장부 각도·부위 좌표), 출력은 팬/틸트 목표 각도(degree).
실제 모터 구동은 hardware/stepper.py 몫이다. main.py의 부위 러너가 쓴다.

  - RegionPatrolScenario: 전신 추적 상태기계 (매핑 → 순찰, 이탈 시 재조준·탐색)
  - region_wind_level:    풍속 중재 — 지금 겨누는 부위의 저장 세기
  - MotionGate:         순찰 ↔ 추적 폴백 전환 판정

전제: 카메라가 팬·틸트 헤드에 함께 실려 움직인다 (2026-07-10 확정). 그래서 틸트도
팬과 같은 피드백(상대 보정) 루프다 — 새 목표각 = 현재 장부각 + gain × 오차각,
오차각은 FOV만으로 정해지고 거리에 의존하지 않는다
(docs/tracking_feedback.md 참고).

이력: 원래 FullBodyScenario(control/fullbody_scenario.py) + 그것을 상속한
RegionPatrolScenario 두 클래스였다. 구체 클래스가 하나뿐이라 상속이 다형성을 주지
못했고, 부모의 반복 수렴 스캔·도착 판정 순찰(91줄)은 자식이 완전히 대체해 한 번도
실행되지 않았다. 어느 쪽이 도는지 매번 확인해야 하는 비용만 남아 한 클래스로 합쳤다
(2026-09-02). 삭제된 경로는 git 이력에 있다 — `git show 5adb773:control/fullbody_scenario.py`.
"""

from __future__ import annotations

from collections import deque

from control.control_signal import (clamp_angle, compute_pan_angle,
                                              compute_tilt_angle)


class RegionPatrolScenario:
    """관측 dict → (팬, 틸트) 목표각. 관측 형식:

    {
      "t": float,             # time.time()
      "pos": (pan, tilt),     # 모터 장부 각도 (°)
      "idle": bool,           # 두 축 모두 목표 도달 여부
      "target_idx": int|None, # 선정된 대상 인덱스 (없으면 None)
      "chest": {cx, cy, visible, paired},   # 어깨 중점 (이번 프레임 원시 관측)
      "regions": {head/upper/lower: {cx, cy, visible}},  # 스무딩 좌표
      "fresh": {head/upper/lower: bool},    # 이번 프레임 실관측 여부
    }

    chest["paired"]는 "cx를 몸의 좌우 중심으로 믿어도 되는가"다 — 한쪽 어깨만
    잡힌 프레임은 cx가 어깨폭 절반만큼 치우쳐 팬을 잘못 돌린다. 팬을 만드는 곳은
    모두 이 플래그를 확인한다 (app/tracking.py의 chest_point 참고).

    ── 상태기계 ─────────────────────────────────────────────────────────────
      scan     한 프레임 매핑. 보이는 부위의 틸트각을 즉시 웨이포인트로 기록하고
               순찰로 넘어간다. map_timeout_s 안에 못 본 부위는 아는 부위에서
               gap만큼 밀어 추정 기록한다(estimated=True). 헤드는 가슴을 중앙으로.
      patrol   웨이포인트 경로를 시간 슬롯으로 순회. 조준은 매 프레임 연속
               피드백이고, 정지 구간에서만 웨이포인트를 관측으로 갱신한다.
      recenter 가슴 수평 오차 > move_thr_deg (사용자 이동) → 가슴을 다시 중앙으로.
               완료 후 이동량이 rescan_thr_deg 이하면 중단 지점 재개, 초과면
               거리·자세가 변한 것으로 보고 전신 재매핑.
      search   대상 상실 lost_frames 연속 → 틸트를 어깨가 보일 각도(upper
               웨이포인트, 없으면 0°)로 올리고 팬을 body_pan ± search_step_deg
               확장 스윕. 어깨 재검출 시 recenter로 넘어가 중단 지점을 재개한다.
               하체 조준 중에는 사람이 프레임에 원래 안 담기므로 상실 카운트는
               머리/상체 조준 중에만 센다.

    ── 왜 매핑이 한 프레임이고 순찰이 시간 슬롯인가 ──────────────────────────
    (1) 부위를 하나씩 화면 중앙에 반복 수렴시켜 각도를 찾던 방식은 수직 FOV
        41도(구세대 렌즈) 전제였다. 지금은 Wide(67도)라 전신이 한 프레임에 담기고,
        부위 각도는
            웨이포인트 = 현재 틸트 + (부위 cy - 0.5) x FOV_V
        로 한 번에 나온다 (거리 무관·순수 각도).
    (2) "도착 판정(|현재각 − 웨이포인트| <= converge_deg) 후 체류"는 잡음 억제용
        데드존과 충돌한다. 데드존은 '명령 변화량'을 자르는데 명령이 gain x 오차라,
        오차가 데드존/gain 아래로 내려가면 모터가 **덜 간 채로** 멈춘다. 그 정지
        오차가 converge_deg보다 크면 도착이 영영 성립하지 않아 그 부위에 갇힌다
        (실기 2026-08-28 "조작해도 바로 안 먹는다"). 시간 슬롯은 도착 판정이
        없으니 갇힐 수가 없고, 데드존을 원래 목적대로 쓸 수 있다.

    ── 거리 적응 ────────────────────────────────────────────────────────────
    관측에서 나온 각도는 이미 거리 보정이 되어 있다(순수 각도). 거리에 따라
    어긋나던 것은 우리가 **덧붙인 고정 각도**(머리 위로 11도, 부위 간 분리 3도)
    뿐이었다 — 부위 간 각도 간격은 거리에 반비례하므로(2m에서 머리↔상체 약 10도,
    4m면 약 5도) 고정 각도는 멀어질수록 과해진다. 그래서 보정값을 전부 **측정된
    머리↔상체 간격(gap)의 비율**로 둔다. 거리를 따로 추정할 필요가 없다.

        조준각(부위) = 웨이포인트 - aim_ratio[부위] x gap      (+ 는 위로)
        부위 간 최소 간격 = spread_ratio x gap

    ── 그 밖 ────────────────────────────────────────────────────────────────
    좌표 기억 구조: 수평은 body_pan 단일값(사람은 좌우로 한 덩어리로 움직이므로
    팬 보정 시 경로 전체가 함께 이동), 수직은 waypoints[부위]["tilt"]. 그래서
    부위 구분은 틸트 전용이고, 누운 자세처럼 부위가 가로로 늘어서면 세 조준각이
    붙어버린다 — 현재 설계의 한계다.

    allowed는 러너가 매 프레임 "세기 1 이상인 부위"로 갱신한다. 웨이포인트가 아직
    없는 부위를 새로 켜면 매핑으로 돌아가 그 부위를 확보한 뒤 순찰한다.

    모든 틸트 목표는 [tilt_min, tilt_max]로 clamp한다 (config.py "limits" —
    상한 +15°는 실측된 기구 파손 한계다).

    step()은 상태 전이 로그를 self.events(문자열 리스트)에 채운다 — 출력은
    호출부 몫. 파라미터 기본값은 실기 검증 전 시작값이라 호출부(CLI)가 덮어쓴다.
    """

    SCAN_ORDER = ("head", "upper", "lower")
    PATROL_ORDER = ("head", "upper", "lower", "upper")

    def __init__(
        self,
        fov_h_deg: float,
        fov_v_deg: float,
        pan_min: float, pan_max: float,
        tilt_min: float, tilt_max: float,
        gain_pan: float,
        gain_tilt: float,
        target_cx: float,
        target_cy: float,
        invert_pan: bool = False,
        invert_tilt: bool = False,
        converge_deg: float = 1.5,
        dwell_s: float = 2.0,
        move_thr_deg: float = 8.0,
        rescan_thr_deg: float = 12.0,
        lost_frames: int = 15,
        search_step_deg: float = 15.0,
        search_limit_deg: float = 60.0,
        search_dwell: int = 8,
        aim_ratio: dict[str, float] | None = None,
        spread_ratio: float = 0.3,
        default_gap_deg: float = 10.0,
        map_timeout_s: float = 1.5,
        remap_alpha: float = 0.3,
        gap_alpha: float = 0.2,
        tilt_rate_dps: float = 11.0,
    ) -> None:
        self.fov_h, self.fov_v = fov_h_deg, fov_v_deg
        self.pan_min, self.pan_max = pan_min, pan_max
        self.tilt_min, self.tilt_max = tilt_min, tilt_max
        self.gain_pan, self.gain_tilt = gain_pan, gain_tilt
        self.sign_pan = -1.0 if invert_pan else 1.0
        self.sign_tilt = -1.0 if invert_tilt else 1.0
        self.target_cx, self.target_cy = target_cx, target_cy
        self.converge_deg = converge_deg
        self.dwell_s = dwell_s
        self.move_thr_deg = move_thr_deg
        self.rescan_thr_deg = rescan_thr_deg
        self.lost_frames = lost_frames
        self.search_step_deg = search_step_deg
        self.search_limit_deg = search_limit_deg
        self.search_dwell = search_dwell
        # 조준 보정 — 측정 간격(gap) 대비 배수. + 는 위로(틸트 음수 방향).
        self.aim_ratio = aim_ratio or {}
        self.spread_ratio = spread_ratio
        self.default_gap_deg = default_gap_deg   # 머리를 아직 못 봤을 때 쓸 값
        self.gap_alpha = gap_alpha
        # 매핑에서 부위가 안 보일 때 추정으로 넘어가기까지의 시간 (s).
        self.map_timeout_s = map_timeout_s
        self.remap_alpha = remap_alpha
        # 슬롯 길이 = 체류 + 이동 시간(추정)에 쓰는 틸트 순항 속도 (도/s, 실측).
        self.tilt_rate_dps = tilt_rate_dps

        self.state = "scan"
        self.body_pan = 0.0
        self.waypoints: dict[str, dict] = {}   # 부위 → {"tilt": °, "estimated": bool}
        self.events: list[str] = []
        # 러너가 매 프레임 갱신하는 값들.
        self.allowed: set[str] = {"head", "upper", "lower"}
        self.levels: dict[str, int] = {}

        self._gap_deg: float | None = None
        self._aim_logged: str | None = None   # [aim] 로그를 부위당 한 번만
        self._lost = 0
        self._patrol_i = 0
        self._resume: str | None = None         # 중단 지점 "scan" | "patrol"
        self._recenter_from = 0.0
        self._recenter_miss = 0
        self._search_seq: list[float] = []
        self._search_i = 0
        self._search_hold = 0
        self._search_tilt = 0.0
        self._map_since: float | None = None
        self._slot_until: float | None = None
        self._last_route: list[str] | None = None

    # ── 헬퍼 ─────────────────────────────────────────────────────────────────

    def _pan_err(self, cx: float) -> float:
        return self.sign_pan * compute_pan_angle(cx - (self.target_cx - 0.5), self.fov_h)

    def _tilt_err(self, cy: float) -> float:
        return self.sign_tilt * compute_tilt_angle(cy - (self.target_cy - 0.5), self.fov_v)

    def _cp(self, deg: float) -> float:
        return clamp_angle(deg, self.pan_min, self.pan_max)

    def _ct(self, deg: float) -> float:
        return clamp_angle(deg, self.tilt_min, self.tilt_max)

    def _tilt_of(self, region_obs: dict, cur_tilt: float) -> float:
        """관측 좌표 → 그 부위의 절대 틸트각 (한 프레임 매핑의 핵심)."""
        return self._ct(cur_tilt + self._tilt_err(region_obs["cy"]))

    # ── 경로 ────────────────────────────────────────────────────────────────

    def _route(self) -> list[str]:
        """켜진 부위에 따른 최소 순회 경로.

        0개(전부 정지)는 한 곳에 머물고, 1개는 그 부위에 고정한다. 인접한 두
        부위는 둘만 왕복하고, 머리+하체(양 끝)일 때만 상체를 지나는 전체 스윕을
        쓴다. PATROL_ORDER(head→upper→lower→upper)는 상체가 두 번 들어 있어
        연속 중복이 생기므로 접는다.
        """
        on = [r for r in self.SCAN_ORDER
              if r in self.allowed and r in self.waypoints]
        if len(on) == 1:
            return on
        if len(on) >= 2:
            selected = set(on)
            if {"head", "lower"} <= selected:
                route = [r for r in self.PATROL_ORDER if r in self.waypoints]
            else:
                route = [r for r in self.PATROL_ORDER if r in selected]
            cleaned = [r for i, r in enumerate(route) if i == 0 or r != route[i - 1]]
            if len(cleaned) > 1 and cleaned[0] == cleaned[-1]:
                cleaned.pop()
            return cleaned
        for region in ("upper", "head", "lower"):
            if region in self.waypoints:
                return [region]
        return []

    def active_region(self) -> str:
        """지금 겨누는 부위. 매핑 중에는 가슴 중심이라 조준 부위가 없다('-')."""
        if self.state == "patrol":
            route = self._route()
            if route:
                return route[self._patrol_i % len(route)]
        return "-"

    # ── 거리 척도와 조준각 ───────────────────────────────────────────────────

    @property
    def gap_deg(self) -> float:
        """머리↔상체 각도 간격 = 거리 척도. 아직 못 쟀으면 기본값."""
        return self.default_gap_deg if self._gap_deg is None else self._gap_deg

    def _update_gap(self, obs: dict) -> None:
        """머리↔상체 간격을 **화면 관측**으로 잰다 (EMA).

        웨이포인트로 재면 안 된다 — 웨이포인트는 틸트 리밋으로 잘린 값이라,
        근거리처럼 둘 다 리밋에 붙는 상황에서 차이가 0이 되어 거리 척도가
        붕괴한다(실측 1m에서 18도가 2도로). 화면상 세로 간격은 리밋과 무관하다.
        """
        h = obs["regions"].get("head")
        u = obs["regions"].get("upper")
        if not (h and u and h["visible"] and u["visible"]):
            return
        gap = min(max(abs(u["cy"] - h["cy"]) * self.fov_v, 2.0), 40.0)  # 오검출 방어
        self._gap_deg = (gap if self._gap_deg is None
                         else self._gap_deg + self.gap_alpha * (gap - self._gap_deg))

    def aims(self) -> dict[str, float]:
        """부위별 최종 조준각 — 비율 보정 + 최소 간격 + 리밋.

        웨이포인트는 순수 관측각이고 보정은 여기서만 얹는다. 관측을 건드리지
        않으므로 "보정된 값으로 다시 보정하는" 순환이 없다.
        """
        gap = self.gap_deg
        aims = {r: self._ct(wp["tilt"] - self.aim_ratio.get(r, 0.0) * gap)
                for r, wp in self.waypoints.items()}
        order = [r for r in ("head", "upper", "lower") if r in aims]
        min_gap = self.spread_ratio * gap
        for i in range(1, len(order)):
            prev, cur = order[i - 1], order[i]
            if aims[cur] - aims[prev] < min_gap:
                aims[cur] = aims[prev] + min_gap
        # 아래로 밀다 리밋을 넘으면 전체를 위로 옮겨 간격을 지킨다.
        if order:
            over = aims[order[-1]] - self.tilt_max
            if over > 0:
                for r in order:
                    aims[r] -= over
            aims = {r: self._ct(a) for r, a in aims.items()}
        return aims

    # ── 상태 전이 ────────────────────────────────────────────────────────────

    def _restart_scan(self) -> None:
        self.state = "scan"
        self._map_since = None
        self._slot_until = None

    def _enter_patrol(self) -> None:
        self.state = "patrol"
        self._patrol_i = 0
        self._slot_until = None
        self._map_since = None
        wps = "  ".join(f"{k}:{v['tilt']:+.1f}{'(추정)' if v['estimated'] else ''}"
                        for k, v in self.waypoints.items())
        self.events.append(f"매핑 완료 → 순찰 시작: {wps} (간격 {self.gap_deg:.1f}도)")

    def _enter_search(self, reason: str) -> None:
        if self.state in ("scan", "patrol"):
            self._resume = self.state
        self.state = "search"
        self._lost = 0
        wu = self.waypoints.get("upper")
        self._search_tilt = self._ct(wu["tilt"]) if wu else 0.0
        seq = [self.body_pan]
        k = 1
        while k * self.search_step_deg <= self.search_limit_deg:
            seq += [self._cp(self.body_pan + k * self.search_step_deg),
                    self._cp(self.body_pan - k * self.search_step_deg)]
            k += 1
        self._search_seq = list(dict.fromkeys(round(s, 3) for s in seq))
        self._search_i = self._search_hold = 0
        self.events.append(f"{reason} → 어깨 탐색 (tilt {self._search_tilt:+.1f}°, "
                           f"pan {self.body_pan:+.1f}°±{self.search_limit_deg:g}°)")

    # ── 메인 스텝 ────────────────────────────────────────────────────────────

    def step(self, obs: dict) -> tuple[float, float]:
        self.events = []
        cur_pan, cur_tilt = obs["pos"]

        # 켜진 부위가 바뀌었나 — 순찰 중에만 본다.
        if self.state == "patrol":
            if self.allowed - self.waypoints.keys():
                # 웨이포인트가 없는 부위를 새로 켰다 — 매핑으로 확보하고 온다.
                # 그냥 두면 _route가 그 부위를 세지 못해 "켜도 반응이 없다".
                self._restart_scan()
                self._last_route = None
            else:
                new_route = self._route()
                if new_route != self._last_route:
                    # 경로가 바뀌어도 지금 겨누던 부위가 새 경로에 있으면 이어서
                    # 간다. 슬롯은 새로 센다.
                    current = (self._last_route[self._patrol_i % len(self._last_route)]
                               if self._last_route else None)
                    if current in new_route:
                        self._patrol_i = new_route.index(current)
                    elif new_route:
                        self._patrol_i %= len(new_route)
                    self._slot_until = None
                    self._last_route = list(new_route)

        # 대상 상실 카운트 — 머리/상체 조준 중에만 (하체 조준 중엔 원래 안 보임)
        if obs["target_idx"] is None:
            if (self.state in ("scan", "patrol")
                    and self.active_region() in ("head", "upper")):
                self._lost += 1
        else:
            self._lost = 0
        if self.state in ("scan", "patrol") and self._lost >= self.lost_frames:
            self._enter_search("대상 상실")

        # 사용자 이동 감지 — 가슴 수평 오차가 크면 재조준
        chest = obs["chest"]
        if self.state in ("scan", "patrol") and chest["visible"] and chest["paired"]:
            err = self._pan_err(chest["cx"])
            if abs(err) > self.move_thr_deg:
                self._resume = self.state
                self._recenter_from = cur_pan
                self._recenter_miss = 0
                self.state = "recenter"
                self.events.append(f"이동 감지 (가슴 오차 {err:+.1f}°) → 재조준")

        if self.state == "scan":
            return self._step_scan(obs, cur_pan, cur_tilt)
        if self.state == "patrol":
            return self._step_patrol(obs, cur_pan, cur_tilt)
        if self.state == "recenter":
            return self._step_recenter(obs, cur_pan, cur_tilt)
        return self._step_search(obs, cur_pan, cur_tilt)

    # ── 매핑 ─────────────────────────────────────────────────────────────────

    def _remap(self, obs: dict, cur_tilt: float) -> None:
        """보이는 부위의 웨이포인트를 관측각으로 갱신(EMA).

        도착 판정이 없으므로 조준 중인 부위도 함께 갱신한다 — 사용자가 움직이거나
        거리가 변해도 재매핑 없이 조준이 따라간다.

        ⚠ 이동 중에 부르면 웨이포인트가 진행 방향으로 밀린다. _tilt_of 는
        "장부각 + 화면 오차각"으로 절대각을 만드는데, 장부는 큐잉 시점에 기록돼
        이동 중에는 실제 로터보다 앞서 있고(_LOOKAHEAD_S 분량) 화면은 실제
        로터가 본 것이라 기준이 다르다. 웨이포인트는 관측과 달리 **저장**되므로
        그 편향이 다음 프레임에 스스로 교정되지 않는다.
        그래서 순찰에서는 obs["idle"] 일 때만 부른다 (_step_patrol 참고).
        매핑은 끝내야 진행되므로 이동 중에도 부르고, 그때 들어간 편향은 순찰의
        정지 구간 갱신이 걷어낸다.
        """
        for region in self.SCAN_ORDER:
            if not obs["fresh"][region]:
                continue
            tilt = self._tilt_of(obs["regions"][region], cur_tilt)
            wp = self.waypoints.get(region)
            if wp is None:
                self.waypoints[region] = {"tilt": tilt, "estimated": False}
            else:
                wp["tilt"] += self.remap_alpha * (tilt - wp["tilt"])
                wp["estimated"] = False
        self._update_gap(obs)

    def _step_scan(self, obs: dict, cur_pan: float, cur_tilt: float) -> tuple[float, float]:
        """한 프레임 매핑 — 보이는 부위를 즉시 기록하고 순찰로 넘어간다."""
        if self._map_since is None:
            self._map_since = obs["t"]
        chest = obs["chest"]
        self._remap(obs, cur_tilt)

        # 헤드는 가슴을 중앙으로 (부위 조준은 순찰이 한다).
        pan_t, tilt_t = self.body_pan, cur_tilt
        if chest["visible"]:
            if chest["paired"]:   # 한쪽 어깨뿐이면 cx 가 치우쳐 있다 (chest_point 참고)
                pan_t = self._cp(cur_pan + self.gain_pan * self._pan_err(chest["cx"]))
                self.body_pan = pan_t
            tilt_t = self._ct(cur_tilt + self.gain_tilt * self._tilt_err(chest["cy"]))

        need = (self.allowed & set(self.SCAN_ORDER)) or {"upper"}
        if need <= self.waypoints.keys():
            self._enter_patrol()
            return pan_t, tilt_t

        # 타임아웃 — 못 본 부위를 추정으로 채우고 진행한다(정체 차단).
        if (obs["t"] - self._map_since >= self.map_timeout_s
                and obs["target_idx"] is not None):
            for region in sorted(need - self.waypoints.keys()):
                self.waypoints[region] = {
                    "tilt": self._ct(self._fallback_tilt(region, chest, cur_tilt)),
                    "estimated": True}
                self.events.append(f"[map] {region} 미검출 — 추정 조준")
            if self.waypoints:
                self._enter_patrol()
        return pan_t, tilt_t

    def _fallback_tilt(self, region: str, chest: dict, cur_tilt: float) -> float:
        """관측 못 한 부위의 임시 각도 — 아는 부위에서 간격만큼 민다."""
        gap = self.gap_deg
        if region == "upper":
            if chest["visible"]:
                return self._tilt_of(chest, cur_tilt)      # 가슴 = 상체
            head = self.waypoints.get("head")
            return head["tilt"] + gap if head else cur_tilt
        upper = self.waypoints.get("upper")
        if region == "head":
            return upper["tilt"] - gap if upper else self.tilt_min
        return upper["tilt"] + gap * 2.0 if upper else self.tilt_max

    # ── 시간 슬롯 순찰 ───────────────────────────────────────────────────────

    def _slot_len(self, region: str, cur_tilt: float) -> float:
        """이 부위에 머물 시간 = 체류 + 이동 시간(추정).

        도착을 판정하지 않는 대신 이동에 걸릴 시간을 더해 준다 — 멀리 있는
        부위로 갈 때 도착하자마자 넘어가 버리는 것을 막는다.
        """
        if self.levels.get(region, 1) == 0:
            return 0.0        # 바람이 없는 부위는 지나가기만 한다
        travel = abs(self.aims().get(region, cur_tilt) - cur_tilt) / self.tilt_rate_dps
        return self.dwell_s + travel

    def _step_patrol(self, obs: dict, cur_pan: float, cur_tilt: float) -> tuple[float, float]:
        route = self._route()
        if not route:
            self.events.append("웨이포인트 없음 — 매핑 재시작")
            self._restart_scan()
            return self.body_pan, cur_tilt
        # 이동 중 갱신은 장부 지연만큼 웨이포인트를 밀어 "내려가다 멈추고 다시
        # 올라가는" 진동을 만든다 (실기 2026-09-02). 정지 구간에서만 갱신한다 —
        # 부위마다 dwell_s 씩 머무르므로 기회는 충분하다.
        if obs["idle"]:
            self._remap(obs, cur_tilt)

        region = route[self._patrol_i % len(route)]
        if self._slot_until is None:
            self._slot_until = obs["t"] + self._slot_len(region, cur_tilt)
        elif obs["t"] >= self._slot_until:
            self._patrol_i = (self._patrol_i + 1) % len(route)
            region = route[self._patrol_i % len(route)]
            self._aim_logged = None    # 새 부위 — 로그 한 줄 다시
            self._slot_until = obs["t"] + self._slot_len(region, cur_tilt)

        pan_t = self.body_pan
        chest = obs["chest"]
        if chest["visible"] and chest["paired"]:
            pan_t = self._cp(cur_pan + self.gain_pan * self._pan_err(chest["cx"]))
            self.body_pan = pan_t
        return pan_t, self._aim_tilt(region, obs, cur_tilt)

    def _aim_tilt(self, region: str, obs: dict, cur_tilt: float) -> float:
        """조준 부위의 틸트 목표각 — 매 프레임 연속 피드백.

            목표각 = 현재각 + gain_tilt x (화면오차각 - 조준편향)
            수렴점 = 부위의 실제 각도 - 조준편향

        [왜 gain 을 곱하나] 한 번에 오차 전부를 명령하면(gain=1) 모델 오차가
        그대로 과잉 이동이 된다 — 화각·장부 지연·카메라 지연이 조금만 어긋나도
        지나쳤다 되돌아오는 진동이 되고, 얼굴처럼 화면 가장자리에 있는 부위는
        그 한 번에 프레임 밖으로 나간다 (실기 2026-09-02). gain<1 이면 남은
        오차를 여러 프레임에 나눠 갚아 그 오차가 감쇠된다.

        [왜 매 프레임인가] 장부 지연이 상쇄된다. 목표각 = 장부 + gain x 오차 이고
        모터는 장부가 목표에 닿으면 서므로, 정지 조건이 gain x 오차 = 0 이 되어
        장부가 실제보다 앞서 있든 말든 수렴점은 오차 0 인 자리다. 웨이포인트처럼
        절대각을 저장하면 그 편향이 남지만, 매 프레임 다시 재면 남지 않는다.

        ⚠ 데드존과 짝이다. 명령 변화량이 gain x 오차 라, 데드존이 넓으면
        오차 < 데드존/gain 구간에서 명령이 아예 안 나가 그만큼 못 미친 채 선다
        (틸트 기본값이면 1.0/0.2 = 5°). 그래서 순찰 중에는 main.py 가 틸트
        데드존을 수렴용(conv_dz)으로 좁혀 사각지대를 1.25° 로 줄인다.

        부위가 안 보이면 웨이포인트 기반 aims() 로 떨어진다 — 부위 간 최소 간격과
        리밋 보정이 거기 들어 있어, 지금 못 보는 부위도 겹치지 않게 겨눈다.
        """
        r = obs["regions"].get(region)
        if obs["fresh"].get(region) and r and r["visible"]:
            bias = self.aim_ratio.get(region, 0.0) * self.gap_deg
            err = self._tilt_err(r["cy"]) - bias
            aim = self._ct(cur_tilt + self.gain_tilt * err)
            if self._aim_logged != region:
                self.events.append(
                    f"[aim] {region} cy={r['cy']:.2f} 오차{err:+.1f}° "
                    f"장부{cur_tilt:+.1f}° → 조준{aim:+.1f}°")
                self._aim_logged = region
            return aim
        if self._aim_logged != region:
            self.events.append(f"[aim] {region} 미관측 → 웨이포인트 조준")
            self._aim_logged = region
        return self.aims().get(region, cur_tilt)

    # ── 재조준 · 탐색 ────────────────────────────────────────────────────────

    def _step_recenter(self, obs: dict, cur_pan: float, cur_tilt: float) -> tuple[float, float]:
        chest = obs["chest"]
        if not chest["visible"]:
            self._recenter_miss += 1
            if self._recenter_miss >= self.lost_frames:
                self._enter_search("재조준 중 가슴 상실")
            return self.body_pan, cur_tilt
        self._recenter_miss = 0
        tilt_t = self._ct(cur_tilt + self.gain_tilt * self._tilt_err(chest["cy"]))
        if not chest["paired"]:
            # cx 가 한쪽 어깨로 치우쳐 있다 — 그 값으로 수렴을 판정하면 어깨폭
            # 절반만큼 틀어진 자리를 "중앙"으로 확정한다. 팬은 그대로 두고
            # 두 어깨(또는 코)가 다시 잡힐 때까지 판정을 미룬다.
            return self.body_pan, tilt_t
        ep = self._pan_err(chest["cx"])
        pan_t = self._cp(cur_pan + self.gain_pan * ep)
        self.body_pan = pan_t
        if abs(ep) < self.converge_deg:
            moved = abs(pan_t - self._recenter_from)
            if self._resume != "patrol" or moved > self.rescan_thr_deg:
                # 매핑 중이었거나 많이 움직였다 — 거리·자세가 변했을 수 있으니
                # 다시 잰다. 매핑은 한 프레임이라 "중간부터 재개"가 없다.
                self.events.append(f"재조준 완료 (이동 {moved:.1f}°) — 전신 재매핑")
                self._restart_scan()
            else:
                self.state = "patrol"
                self._slot_until = None   # 재조준에 걸린 시간만큼 슬롯이 밀렸다
                self.events.append(f"재조준 완료 (이동 {moved:.1f}°) — 순찰 재개")
            self._resume = None
        return pan_t, tilt_t

    def _step_search(self, obs: dict, cur_pan: float, cur_tilt: float) -> tuple[float, float]:
        if obs["chest"]["visible"]:
            self._recenter_from = self.body_pan   # 상실 지점 기준으로 이동량 판정
            self._recenter_miss = 0
            self.state = "recenter"
            self.events.append("어깨 재검출 → 재조준")
            return cur_pan, cur_tilt
        pan_t = self._search_seq[self._search_i]
        if obs["idle"] and abs(cur_pan - pan_t) <= self.converge_deg:
            self._search_hold += 1
            if self._search_hold >= self.search_dwell:
                self._search_hold = 0
                self._search_i += 1
                if self._search_i >= len(self._search_seq):
                    self._search_i = 0
                    self.events.append("탐색 스윕 미발견 — 반복")
        return pan_t, self._search_tilt


def region_wind_level(scenario: RegionPatrolScenario, levels: dict[str, int],
                    common_level: int = 0) -> int:
    """지금 겨누는 부위의 세기, 겨누는 부위가 없으면 common_level.

    scenario.active_region()은 순찰 중이면 체류 중인 부위를 준다. 매핑·탐색·
    재조준은 겨누는 부위가 없어('-') 공용 세기 = 추적 모드와 같은 풍속을
    유지한다 (실기 2026-08-27: 단계마다 다른 세기를 쓰면 진입 직후 풍속이 튄다).
    """
    region = scenario.active_region()
    if region in levels:
        return levels[region]
    return common_level


class MotionGate:
    """순찰 ↔ 추적 폴백 전환 판정 (순수 계산, 프레임 단위 호출).

    양방향 모두 "최근 시간창의 팬 각 범위(최대-최소)"로 판정한다 — 순찰/폴백
    중 팬은 (가슴 피드백을 통해) 사용자를 따라갈 때만 움직이고 부위 간 이동은
    틸트이므로, 팬 이동량이 곧 사용자 이동량이다.

    exit (순찰→폴백): 시나리오가 patrol 상태로 안정된 동안 최근 exit_window_s
      의 팬 범위 > exit_err_deg. 가슴 오차 기반이던 이전 판정은 헤드 자신이
      스윙하는 동안(부위 전환·재조준·탐색)의 일시적 오차를 사용자 이동으로
      오판했다 (실기 2026-08-27). scan/recenter/search 중에는 창을 비우고
      판정을 쉰다 — 탐색의 팬 스윕 오판 방지. 창보다 느린 완만한 이동은
      시나리오의 recenter가 자체 처리하므로 폴백까지 갈 필요가 없다.
    enter (폴백→순찰): 최근 enter_still_s 동안 팬 범위 < enter_span_deg이고
      이번 프레임에 가슴이 보이면. 전환 직후에는 창이 찰 때까지 판정을 보류한다
      (exit 쪽은 보류가 없어도 된다 — 샘플이 적으면 범위가 작아져 오탐 방향이
      아니다).
    """

    def __init__(self, exit_err_deg: float = 12.0, exit_window_s: float = 3.0,
                 enter_still_s: float = 5.0, enter_span_deg: float = 3.0) -> None:
        self.exit_err_deg = exit_err_deg
        self.exit_window_s = exit_window_s
        self.enter_still_s = enter_still_s
        self.enter_span_deg = enter_span_deg
        self._exit_hist: deque[tuple[float, float]] = deque()  # (t, pan°)
        self._pan_hist: deque[tuple[float, float]] = deque()   # (t, pan°)

    def reset_patrol(self) -> None:
        self._exit_hist.clear()

    def reset_fallback(self) -> None:
        self._pan_hist.clear()

    def update_patrol(self, t: float, pan_deg: float, in_patrol: bool) -> bool:
        """순찰 상 프레임마다 호출 — True면 추적 폴백으로 나가라.
        in_patrol = 시나리오가 patrol 상태인지 (scan/recenter/search면 쉼)."""
        if not in_patrol:
            self._exit_hist.clear()
            return False
        self._exit_hist.append((t, pan_deg))
        cutoff = t - self.exit_window_s
        while self._exit_hist and self._exit_hist[0][0] < cutoff:
            self._exit_hist.popleft()
        pans = [p for _, p in self._exit_hist]
        if max(pans) - min(pans) > self.exit_err_deg:
            self.reset_patrol()
            return True
        return False

    def update_fallback(self, t: float, pan_deg: float,
                        chest_visible: bool) -> bool:
        """폴백 프레임마다 호출 — True면 순찰로 재진입하라."""
        self._pan_hist.append((t, pan_deg))
        cutoff = t - self.enter_still_s
        while self._pan_hist and self._pan_hist[0][0] < cutoff:
            self._pan_hist.popleft()
        if not chest_visible:
            return False
        if self._pan_hist[0][0] > cutoff + 0.5:
            return False  # 폴백 직후 — 정지 시간 창이 아직 안 찼다
        pans = [p for _, p in self._pan_hist]
        if max(pans) - min(pans) >= self.enter_span_deg:
            return False
        self.reset_fallback()
        return True
