"""
control/body_wind.py

부위 인식 모드(0x03)의 순수 계산 보조 — GPIO/BlueZ/카메라를 import하지 않는다.
main.py의 부위 러너가 사용한다.

  - BodyPatrolScenario: FullBodyScenario를 부위 모드용으로 바꾼 것 —
    반복 수렴 스캔을 한 프레임 직접 매핑으로, 도착 판정 순찰을 시간 슬롯
    순찰로 대체한다. 조준 보정값은 각도가 아니라 **측정된 몸 간격의 비율**이라
    거리가 변해도 따라간다.
  - body_wind_level: 풍속 중재 — 지금 겨누는 부위(매핑·순찰)의 저장 세기,
    겨누는 부위가 없는 동안(탐색·재조준)은 공용 세기(추적 모드와 동일 풍속).
  - MotionGate: 순찰 ↔ 추적 폴백 전환 판정.
"""

from __future__ import annotations

from collections import deque

from control.fullbody_scenario import FullBodyScenario


class BodyPatrolScenario(FullBodyScenario):
    """부위 모드용 전신 시나리오 — 매핑 + 시간 슬롯 순찰.

    ── 왜 부모를 그대로 쓰지 않는가 ─────────────────────────────────────────
    부모(FullBodyScenario)는 (1) 부위를 하나씩 화면 중앙에 반복 수렴시켜 각도를
    찾고, (2) "도착 판정(|현재각 − 웨이포인트| <= converge_deg) 후 체류"로 다음
    부위에 넘어간다. 둘 다 이 모드에서는 문제가 됐다:

      (1) 반복 수렴은 수직 FOV 41도(구세대 렌즈) 전제였다. 지금은 Wide(67도)라
          전신이 한 프레임에 담기고, 부위 각도는
              웨이포인트 = 현재 틸트 + (부위 cy - 0.5) x FOV_V
          로 한 번에 나온다(거리 무관·순수 각도). → _step_scan을 한 프레임
          매핑으로 대체.
      (2) 도착 판정은 잡음 억제용 데드존과 충돌한다. 데드존은 '명령 변화량'을
          자르는데 명령이 gain x 오차라, 오차가 데드존/gain 아래로 내려가면
          모터가 **덜 간 채로** 멈춘다. 그 정지 오차가 converge_deg보다 크면
          도착이 영영 성립하지 않아 그 부위에 갇힌다 (실기 2026-08-28
          "조작해도 바로 안 먹는다"). → _step_patrol을 **시간 슬롯**으로 대체.
          도착 판정이 없으니 갇힐 수가 없고, 데드존을 원래 목적(잡음 억제)
          대로 쓸 수 있다.

    ── 거리 적응 ────────────────────────────────────────────────────────────
    관측에서 나온 각도는 이미 거리 보정이 되어 있다(순수 각도). 거리에 따라
    어긋나던 것은 우리가 **덧붙인 고정 각도**(머리 위로 11도, 부위 간 분리 3도)
    뿐이었다 — 부위 간 각도 간격은 거리에 반비례하므로(2m에서 머리↔상체 약 10도,
    4m면 약 5도) 고정 각도는 멀어질수록 과해진다. 그래서 보정값을 전부 **측정된
    머리↔상체 간격(gap)의 비율**로 둔다. 거리를 따로 추정할 필요가 없다.

        조준각(부위) = 웨이포인트 - aim_ratio[부위] x gap      (+ 는 위로)
        부위 간 최소 간격 = spread_ratio x gap

    ── 그 밖 ────────────────────────────────────────────────────────────────
    allowed는 러너가 매 프레임 "세기 1 이상인 부위"로 갱신한다. 웨이포인트가
    아직 없는 부위를 새로 켜면 매핑 단계로 돌아가 그 부위를 확보한 뒤 순찰한다.
    부모(FullBodyScenario)는 건드리지 않는다 — 반복 수렴 스캔 경로가 그대로 남아 있다.
    """

    def __init__(self, *args, aim_ratio: dict[str, float] | None = None,
                 spread_ratio: float = 0.3, default_gap_deg: float = 10.0,
                 map_timeout_s: float = 1.5, remap_alpha: float = 0.3,
                 gap_alpha: float = 0.2, tilt_rate_dps: float = 11.0, **kwargs):
        super().__init__(*args, **kwargs)
        # 조준 보정 — 측정 간격(gap) 대비 배수. + 는 위로(틸트 음수 방향).
        self.aim_ratio = aim_ratio or {}
        self.spread_ratio = spread_ratio
        self.default_gap_deg = default_gap_deg   # 머리를 아직 못 봤을 때 쓸 값
        self.gap_alpha = gap_alpha
        self._gap_deg: float | None = None
        # 러너가 매 프레임 갱신하는 값들.
        self.allowed: set[str] = {"head", "upper", "lower"}
        self.levels: dict[str, int] = {}
        # 매핑에서 부위가 안 보일 때 추정으로 넘어가기까지의 시간 (s).
        self.map_timeout_s = map_timeout_s
        self.remap_alpha = remap_alpha
        # 슬롯 길이 = 체류 + 이동 시간(추정)에 쓰는 틸트 순항 속도 (도/s, 실측).
        self.tilt_rate_dps = tilt_rate_dps
        self._map_since: float | None = None
        self._slot_until: float | None = None
        self._last_route: list[str] | None = None

    # ── 경로 ────────────────────────────────────────────────────────────────

    def _route(self):
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

    # ── 매핑 (부모의 반복 수렴 스캔 대체) ────────────────────────────────────

    def active_region(self) -> str:
        # 매핑 중에는 특정 부위를 겨누지 않는다(가슴 중심) → 조준 부위 없음.
        return "-" if self.state == "scan" else super().active_region()

    def _restart_scan(self) -> None:
        super()._restart_scan()
        self._map_since = None
        self._slot_until = None

    def _tilt_of(self, region_obs: dict, cur_tilt: float) -> float:
        """관측 좌표 → 그 부위의 절대 틸트각 (한 프레임 매핑의 핵심)."""
        return self._ct(cur_tilt + self._tilt_err(region_obs["cy"]))

    def _enter_patrol(self) -> None:
        self.state = "patrol"
        self.scan_i = len(self.SCAN_ORDER)
        self._patrol_i = 0
        self._slot_until = None
        self._map_since = None
        self._cvg = self._occl = 0
        wps = "  ".join(f"{k}:{v['tilt']:+.1f}{'(추정)' if v['estimated'] else ''}"
                        for k, v in self.waypoints.items())
        self.events.append(f"매핑 완료 → 순찰 시작: {wps} (간격 {self.gap_deg:.1f}도)")

    def _remap(self, obs: dict, cur_tilt: float) -> None:
        """보이는 부위의 웨이포인트를 관측각으로 갱신(EMA).

        도착 판정이 없어졌으므로 조준 중인 부위도 함께 갱신한다 — 사용자가
        움직이거나 거리가 변해도 재매핑 없이 조준이 따라간다.
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

    def _step_scan(self, obs: dict, cur_pan: float, cur_tilt: float):
        """한 프레임 매핑 — 보이는 부위를 즉시 기록하고 순찰로 넘어간다."""
        if self._map_since is None:
            self._map_since = obs["t"]
        chest = obs["chest"]
        self._remap(obs, cur_tilt)

        # 헤드는 가슴을 중앙으로 (부위 조준은 순찰이 한다).
        pan_t, tilt_t = self.body_pan, cur_tilt
        if chest["visible"]:
            pan_t = self._cp(cur_pan + self.gain_pan * self._pan_err(chest["cx"]))
            tilt_t = self._ct(cur_tilt + self.gain_tilt * self._tilt_err(chest["cy"]))
            self.body_pan = pan_t

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

    # ── 시간 슬롯 순찰 (부모의 도착 판정 순찰 대체) ──────────────────────────

    def _slot_len(self, region: str, cur_tilt: float) -> float:
        """이 부위에 머물 시간 = 체류 + 이동 시간(추정).

        도착을 판정하지 않는 대신 이동에 걸릴 시간을 더해 준다 — 멀리 있는
        부위로 갈 때 도착하자마자 넘어가 버리는 것을 막는다.
        """
        if self.levels.get(region, 1) == 0:
            return 0.0        # 바람이 없는 부위는 지나가기만 한다
        travel = abs(self.aims().get(region, cur_tilt) - cur_tilt) / self.tilt_rate_dps
        return self.dwell_s + travel

    def _step_patrol(self, obs: dict, cur_pan: float, cur_tilt: float):
        route = self._route()
        if not route:
            self.events.append("웨이포인트 없음 — 매핑 재시작")
            self._restart_scan()
            return self.body_pan, cur_tilt
        self._remap(obs, cur_tilt)

        region = route[self._patrol_i % len(route)]
        if self._slot_until is None:
            self._slot_until = obs["t"] + self._slot_len(region, cur_tilt)
        elif obs["t"] >= self._slot_until:
            self._patrol_i = (self._patrol_i + 1) % len(route)
            region = route[self._patrol_i % len(route)]
            self._slot_until = obs["t"] + self._slot_len(region, cur_tilt)

        pan_t = self.body_pan
        if obs["chest"]["visible"]:
            pan_t = self._cp(cur_pan + self.gain_pan * self._pan_err(obs["chest"]["cx"]))
            self.body_pan = pan_t
        return pan_t, self.aims().get(region, cur_tilt)

    def step(self, obs: dict):
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
        return super().step(obs)


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
