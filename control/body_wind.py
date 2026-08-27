"""
control/body_wind.py

부위 인식 모드(0x03)의 순수 계산 보조 — GPIO/BlueZ/카메라를 import하지 않는다.
scripts/verify_E2E_v3.py의 부위 러너가 사용한다.

  - BodyPatrolScenario: FullBodyScenario의 순찰 경로만 필터 — 세기 0(정지)인
    부위는 돌지 않는다. 스캔은 전 부위 그대로 한다(하체 틸트 추정이 head·upper
    웨이포인트를 쓰므로 — fullbody_scenario._est_lower_tilt 참고). 상체는
    프로토콜상 세기 0이 불가라(ble_protocol.md §3.3) 웨이포인트만 잡히면
    경로가 비는 일은 없다.
  - body_wind_level: 풍속 중재 — 지금 겨누는 부위(스캔·순찰)의 저장 세기,
    겨누는 부위가 없는 동안(탐색·재조준)은 공용 세기(추적 모드와 동일 풍속).
  - MotionGate: 순찰 ↔ 추적 폴백 전환 판정.
"""

from __future__ import annotations

from collections import deque

from control.fullbody_scenario import FullBodyScenario


class BodyPatrolScenario(FullBodyScenario):
    """순찰 경로를 allowed 부위로 제한하고, 부위 간 조준각을 벌린 전신 시나리오.

    allowed는 러너가 매 프레임 "세기 ≥1인 부위"로 갱신한다 — 순찰 중 앱이
    세기를 0으로 내리면 다음 부위 선택부터 빠지고, 다시 올리면 (스캔 때 기록된
    웨이포인트가 있으므로) 즉시 경로에 복귀한다. 경로 길이가 변하면 순찰
    인덱스가 한 부위를 건너뛰거나 반복할 수 있지만 한 사이클 안에서 정리된다.

    조준각을 벌리는 장치가 둘인데 역할이 다르다:
      aim_bias_norm  — 부위별 조준점을 옮긴다 (관측 쪽, __init__ 주석 참고).
      tilt_spread_deg — 부위별 틸트 허용 구간을 어긋나게 자른다 (출력 쪽,
        _spread_clamp 참고). 리밋에 눌린 부위들이 한 각도로 뭉치는 것을 막는다.
    """

    def __init__(self, *args, aim_bias_norm: dict[str, float] | None = None,
                 tilt_spread_deg: float = 0.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.allowed: set[str] = {"head", "upper", "lower"}
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
        return [r for r in self.PATROL_ORDER
                if r in self.waypoints and r in self.allowed]

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
