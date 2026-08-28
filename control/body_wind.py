"""
control/body_wind.py

부위 인식 모드(0x03)의 순수 계산 보조 — GPIO/BlueZ/카메라를 import하지 않는다.
scripts/verify_E2E_v3.py의 부위 러너가 사용한다.

  - BodyPatrolScenario: FullBodyScenario를 부위 모드용으로 바꾼 것 —
    반복 수렴 스캔을 **한 프레임 직접 매핑**으로 대체하고, 순찰 경로를
    세기 ≥1인 부위로 제한하며, 부위 간 조준각을 벌린다.
  - body_wind_level: 풍속 중재 — 지금 겨누는 부위(스캔·순찰)의 저장 세기,
    겨누는 부위가 없는 동안(탐색·재조준)은 공용 세기(추적 모드와 동일 풍속).
  - MotionGate: 순찰 ↔ 추적 폴백 전환 판정.
"""

from __future__ import annotations

from collections import deque

from control.fullbody_scenario import FullBodyScenario


class BodyPatrolScenario(FullBodyScenario):
    """부위 모드용 전신 시나리오 — 스캔을 '직접 매핑'으로 대체한 파생 클래스.

    ── 직접 매핑 (2026-08-27, 실기 "스캔이 너무 오래 걸림"에 대한 답) ────────
    부모의 스캔은 부위를 하나씩 화면 중앙에 **반복 수렴**시켜 각도를 기록한다.
    수직 FOV 41°(구세대 표준 렌즈) 전제의 설계였다 — 그때는 사람이 가까우면
    머리와 발이 한 프레임에 같이 안 담겨서 틸트를 옮겨가며 찾을 수밖에 없었다.
    지금 카메라는 Camera Module 3 Wide(수직 67°)라 부위 각도가
        웨이포인트 = 현재 틸트 + (부위 cy에서 계산한 각도)
    로 **한 프레임에** 나온다(거리 무관·순수 각도). 그래서 스캔 상태는 보이는
    부위를 즉시 기록하고 곧바로 순찰로 넘어가는 매핑 단계가 되었다.

    탐색 스윕을 두지 않은 근거(기하로 확인): 겨눌 수 있는 각도는 틸트 리밋
    이내(|θ| ≤ 15°)이고 카메라는 현재 틸트 ±33.5°를 보므로, 틸트가 어디에
    있든 겨눌 수 있는 부위는 **항상 이미 프레임 안**이다(최대 이격 30° <
    33.5°). 프레임 밖으로 나가는 부위(근접 시의 발 등)는 어차피 리밋 밖이라
    스윕으로 찾아내도 웨이포인트가 리밋으로 클램프된다 — 찾으나 추정하나
    결과가 같다. 부위가 안 보이는 진짜 원인은 가림·저신뢰도인데 스윕은 그걸
    해결하지 못한다. (틸트 리밋을 33.5° 너머로 넓히면 이 근거가 깨지므로
    그때 스윕 도입을 재검토할 것.)

    안 보이는 부위 처리:
      - 관측되지 않은 부위는 **웨이포인트를 만들지 않는다** → _route에서 자동
        제외되어 눈먼 조준을 하지 않고, 보이는 순간 _remap_others가 넣는다.
      - 하체만 예외로, map_timeout_s가 지나도 못 보면 비율 추정
        (_est_lower_tilt)으로 기록한다 — 사용자가 하체 세기를 지정했다면
        어딘가는 겨눠야 하기 때문.
      - 사람은 잡혔는데 세 부위 모두 신뢰도가 낮으면 매핑이 안 끝나므로,
        타임아웃 후 가슴 좌표를 상체 웨이포인트로 삼아 진행한다(정체 차단).
    부모(FullBodyScenario)는 건드리지 않는다 — verify_fulltrack.py는 그대로
    반복 수렴 스캔을 쓴다.

    ── 그 밖의 차이 ────────────────────────────────────────────────────────
    allowed는 러너가 매 프레임 "세기 ≥1인 부위"로 갱신한다 — 순찰 중 앱이
    세기를 0으로 내리면 다음 부위 선택부터 빠지고, 다시 올리면 (웨이포인트가
    남아 있으므로) 즉시 경로에 복귀한다. 경로 길이가 변하면 순찰 인덱스가 한
    부위를 건너뛰거나 반복할 수 있지만 한 사이클 안에서 정리된다.

    조준각을 벌리는 장치가 둘인데 역할이 다르다:
      aim_bias_norm  — 부위별 조준점을 옮긴다 (관측 쪽, __init__ 주석 참고).
      tilt_spread_deg — 부위별 틸트 허용 구간을 어긋나게 자른다 (출력 쪽,
        _spread_clamp 참고). 리밋에 눌린 부위들이 한 각도로 뭉치는 것을 막는다.
    """

    def __init__(self, *args, aim_bias_norm: dict[str, float] | None = None,
                 tilt_spread_deg: float = 0.0, map_timeout_s: float = 1.5,
                 remap_alpha: float = 0.3, **kwargs):
        super().__init__(*args, **kwargs)
        self.allowed: set[str] = {"head", "upper", "lower"}
        # 매핑에서 부위가 안 보일 때 추정/진행으로 넘어가기까지의 시간 (s).
        self.map_timeout_s = map_timeout_s
        # 순찰 중 재매핑 EMA 계수 (_remap_others) — 좌표 잡음이 경로를 흔들지
        # 않게 섞어 넣는다.
        self.remap_alpha = remap_alpha
        self._map_since: float | None = None
        # 부위별 틸트 구간 분리 폭 (°) — 아래 _spread_clamp 참고.
        self.tilt_spread_deg = tilt_spread_deg
        # 부위별 조준 편향 (정규화 화면 단위 ≈ 편향각/수직FOV, +는 위로 조준).
        # 해당 부위의 관측 cy를 그만큼 옮겨 보고하면 피드백이 "실제 부위에서
        # 편향각만큼 벗어난 곳"에 수렴해 부위 간 틸트 간격이 벌어진다
        # (예: 머리 +, 상체 − — 실기에서 머리/상체 구분이 안 보임, 2026-08-27).
        # 모터 각도에 직접 더하면 피드백이 도로 당겨와 편향각/게인 배로
        # 증폭되므로(발진 위험) 반드시 관측 쪽에 넣는다. 스캔 수렴 좌표에도
        # 같은 편향이 걸려 웨이포인트가 자동으로 편향을 포함한다.
        self.aim_bias_norm = aim_bias_norm or {}

    def _route(self):
        route = [r for r in self.PATROL_ORDER
                 if r in self.waypoints and r in self.allowed]
        if route:
            return route
        # 세기가 전부 0이면(사용자가 바람만 끈 상태) 경로가 빈다. 그대로 두면
        # 부모의 patrol이 "웨이포인트 없음 → 스캔 재시작"을 매 프레임 반복해
        # 스캔↔순찰 루프에 빠지므로, allowed를 무시하고 조준만 이어간다 —
        # 풍속은 body_wind_level이 각 부위 세기(0)를 그대로 읽어 정지가 된다.
        return [r for r in self.PATROL_ORDER if r in self.waypoints]

    # 부위 순서 (틸트 값 오름차순 = 화면 위→아래). _spread_clamp가 쓴다.
    _SPREAD_ORDER = {"head": 0, "upper": 1, "lower": 2}

    def _spread_clamp(self, region: str, tilt_deg: float) -> float:
        """부위별로 틸트 허용 구간을 어긋나게 잘라 조준각이 겹치지 않게 한다.

        틸트 리밋이 좁으면(config limits.tilt는 실측 전 임시 ±15°) 상체가
        하한(아래쪽 리밋)에 눌리고, 그 아래인 하체도 같은 각도로 클램프돼
        두 부위 조준이 같은 자리가 된다 — 순찰이 멈춘 것처럼 보인다
        (실기 2026-08-27). 그래서 아래쪽 리밋은 하체에만 내주고 상체는
        spread만큼, 머리는 2×spread만큼 위에서 멈추게 한다(위쪽도 대칭).
        피드백이 리밋까지 밀어붙이는 것을 출력에서 막는 방식이라, 관측이
        어떻든 부위 간 간격이 보장된다.
        """
        g = self.tilt_spread_deg
        idx = self._SPREAD_ORDER.get(region)
        # 구간을 다 넣을 만큼 넓지 않으면(리밋이 아주 좁으면) 분리를 포기한다 —
        # 억지로 자르면 모든 부위가 한 점에 몰려 오히려 나빠진다.
        if not g or idx is None or self.tilt_max - self.tilt_min < 2 * g:
            return tilt_deg
        lo = self.tilt_min + idx * g
        hi = self.tilt_max - (2 - idx) * g
        return min(max(tilt_deg, lo), hi)

    # ── 직접 매핑 (부모의 반복 수렴 스캔 대체) ───────────────────────────────

    def active_region(self) -> str:
        # 매핑 중에는 특정 부위를 겨누지 않는다(가슴 중심) → 조준 부위 없음.
        # 풍속도 이 동안은 공용 세기가 된다 (body_wind_level).
        return "-" if self.state == "scan" else super().active_region()

    def _restart_scan(self) -> None:
        super()._restart_scan()
        self._map_since = None

    def _tilt_of(self, region_obs: dict, cur_tilt: float) -> float:
        """관측 좌표 → 그 부위를 겨누는 절대 틸트각 (한 프레임 매핑의 핵심)."""
        return self._ct(cur_tilt + self._tilt_err(region_obs["cy"]))

    def _enter_patrol(self) -> None:
        self.state = "patrol"
        self.scan_i = len(self.SCAN_ORDER)
        self._patrol_i = 0
        self._dwell_until = None
        self._cvg = self._occl = 0
        self._map_since = None
        wps = "  ".join(f"{k}:{v['tilt']:+.1f}°{'(추정)' if v['estimated'] else ''}"
                        for k, v in self.waypoints.items())
        self.events.append(f"매핑 완료 → 순찰 시작: {wps} @ pan {self.body_pan:+.1f}°")

    def _step_scan(self, obs: dict, cur_pan: float, cur_tilt: float):
        """부모의 반복 수렴 스캔을 대체하는 한 프레임 매핑 (클래스 docstring)."""
        if self._map_since is None:
            self._map_since = obs["t"]
        chest = obs["chest"]

        # 1) 이번 프레임에 실제로 보이는 부위를 즉시 기록한다.
        for region in self.SCAN_ORDER:
            if obs["fresh"][region]:
                self.waypoints[region] = {
                    "tilt": self._tilt_of(obs["regions"][region], cur_tilt),
                    "estimated": False}

        # 2) 헤드는 가슴을 중앙으로 (부위 조준은 순찰이 한다).
        pan_t, tilt_t = self.body_pan, cur_tilt
        if chest["visible"]:
            pan_t = self._cp(cur_pan + self.gain * self._pan_err(chest["cx"]))
            tilt_t = self._ct(cur_tilt + self.gain_tilt * self._tilt_err(chest["cy"]))
            self.body_pan = pan_t

        # 3) 세기 ≥1인 부위가 다 모였으면 곧바로 순찰 (보통 첫 프레임).
        need = (self.allowed & set(self.SCAN_ORDER)) or {"upper"}
        if need <= self.waypoints.keys():
            self._enter_patrol()
            return pan_t, tilt_t

        # 4) 타임아웃 — 필요한데 못 본 부위를 추정으로 채우고 진행한다.
        #    비우고 넘어가면 그 부위가 순찰 경로에서 빠져 "다음 부위로 안 넘어감"
        #    으로 보인다(진단 S4). 보이는 순간 _remap_others가 실제 값으로 고친다.
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
        """관측 못 한 부위의 임시 조준각 (매핑 타임아웃 시)."""
        if region == "upper" and chest["visible"]:
            return self._tilt_of(chest, cur_tilt)   # 가슴 ≈ 상체
        if region == "lower":
            est = self._est_lower_tilt()
            head = self.waypoints.get("head")
            # 머리가 리밋에 눌렸으면 머리↔상체 간격이 가짜라 비율 추정도 무의미
            # 하다 (verify_fulltrack 알려진 문제 1). 그럴 땐 아래 끝을 겨눈다.
            if est is not None and not (head and head["tilt"] <= self.tilt_min + 1e-6):
                return est
        # 부위가 있을 법한 극단 — 머리는 위, 나머지는 아래.
        return self.tilt_min if region == "head" else self.tilt_max

    def _remap_others(self, obs: dict, cur_tilt: float, aimed: str) -> None:
        """순찰 중, 조준 중이 아닌 부위의 웨이포인트를 보이는 대로 갱신한다.

        매핑이 한 프레임짜리라 갱신도 상시 가능하다 — 사용자가 움직이거나
        거리가 변해도 재스캔 없이 경로가 최신으로 유지되고, 처음에 추정으로
        때운 부위도 보이는 순간 실제 값으로 교정된다. 조준 중인 부위는
        제외한다 — 부모의 도착 판정(현재각 vs 웨이포인트)이 흔들리기 때문.
        """
        for region in self.SCAN_ORDER:
            if region == aimed or not obs["fresh"][region]:
                continue
            tilt = self._tilt_of(obs["regions"][region], cur_tilt)
            wp = self.waypoints.get(region)
            if wp is None:
                self.waypoints[region] = {"tilt": tilt, "estimated": False}
            else:
                wp["tilt"] += self.remap_alpha * (tilt - wp["tilt"])
                wp["estimated"] = False

    def _normalize_waypoints(self) -> None:
        """웨이포인트를 '실제로 겨눌 수 있는 각도'로 유지한다.

        부모의 순찰은 다음 부위로 넘어가기 전에 `|현재 틸트 − 웨이포인트| ≤
        converge_deg`(도착 판정)를 요구한다. 그런데 _spread_clamp는 **명령만**
        자르므로, 웨이포인트가 그 구간 밖이면 모터가 도착할 수 없어 체류가
        시작조차 안 되고 그 부위에 영원히 머문다 (진단 S5 — 근접 시 상체가
        리밋에 눌리면 재현). 그래서 기록 주체(매핑·재매핑·부모의 EMA 보정)와
        무관하게 매 스텝 한 번 구간 안으로 normalize한다.

        더불어 순서(머리 ≤ 상체 ≤ 하체)가 뒤집히면 **추정 웨이포인트만** 밀어
        바로잡는다 — 실관측은 건드리지 않는다(관측이 진실).
        """
        g = self.tilt_spread_deg
        for region, wp in self.waypoints.items():
            wp["tilt"] = self._spread_clamp(region, self._ct(wp["tilt"]))
        upper = self.waypoints.get("upper")
        if not upper:
            return
        head, lower = self.waypoints.get("head"), self.waypoints.get("lower")
        if head and head["estimated"] and head["tilt"] > upper["tilt"] - g:
            head["tilt"] = self._spread_clamp("head", upper["tilt"] - g)
        if lower and lower["estimated"] and lower["tilt"] < upper["tilt"] + g:
            lower["tilt"] = self._spread_clamp("lower", upper["tilt"] + g)

    def step(self, obs: dict):
        biased = {r: b for r, b in self.aim_bias_norm.items()
                  if b and obs["regions"].get(r, {}).get("visible")}
        if biased:
            # 원본 dict는 오버레이 표시 등에 그대로 쓰이므로 복사본만 조작한다.
            obs = dict(obs)
            obs["regions"] = dict(obs["regions"])
            for r, b in biased.items():
                region = obs["regions"][r]
                obs["regions"][r] = dict(region, cy=region["cy"] - b)
        # 조준 부위는 step 전에 읽는다 — step이 다음 부위로 넘어갈 수 있어서,
        # 뒤에 읽으면 이번 목표각에 다음 부위의 구간이 걸린다.
        region = self.active_region()
        pan_t, tilt_t = super().step(obs)
        if self.state == "patrol":
            # 제외 대상은 step 이후의 조준 부위 — 다음 프레임 도착 판정의 기준.
            self._remap_others(obs, obs["pos"][1], self.active_region())
        self._normalize_waypoints()
        return pan_t, self._spread_clamp(region, tilt_t)


def body_wind_level(scenario: FullBodyScenario, levels: dict[str, int],
                    common_level: int = 0) -> int:
    """지금 겨누는 부위의 세기, 겨누는 부위가 없으면 common_level.

    scenario.active_region()은 스캔 중이면 그때 수렴시키는 부위를, 순찰 중이면
    체류 중인 부위를 준다 — 두 경우 모두 "이 부위를 겨누는 중"이 성립하므로
    부위별 세기를 그대로 적용한다 (실기 2026-08-27: 스캔 동안만 다른 세기를
    쓰면 부위 모드 진입 직후 풍속이 튄다). 탐색(search)·재조준(recenter)은
    사람을 다시 찾는 중이라 겨누는 부위가 없어('-') 공용 세기 = 추적 모드와
    같은 풍속을 유지한다.
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
      이번 프레임에 가슴이 보이면. 각 방향 모두 전환 직후에는 창이 찰 때까지
      판정을 보류한다.
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
