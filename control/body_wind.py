"""
control/body_wind.py

부위 인식 모드(0x03)의 순수 계산 보조 — GPIO/BlueZ/카메라를 import하지 않는다.
scripts/verify_E2E_v3.py의 부위 러너가 사용한다.

  - BodyPatrolScenario: FullBodyScenario의 순찰 경로만 필터 — 세기 0(정지)인
    부위는 돌지 않는다. 스캔은 전 부위 그대로 한다(하체 틸트 추정이 head·upper
    웨이포인트를 쓰므로 — fullbody_scenario._est_lower_tilt 참고). 상체는
    프로토콜상 세기 0이 불가라(ble_protocol.md §3.3) 웨이포인트만 잡히면
    경로가 비는 일은 없다.
  - patrol_wind_level: 풍속 중재 — 순찰(patrol) 중 조준 부위의 저장 세기,
    그 외 상태(scan/recenter/search)는 0. 조준이 확정되지 않은 동안 엉뚱한
    부위에 바람이 가는 것을 막는다.
  - MotionGate: 순찰 ↔ 추적 폴백 전환 판정.
"""

from __future__ import annotations

from collections import deque

from control.fullbody_scenario import FullBodyScenario


class BodyPatrolScenario(FullBodyScenario):
    """순찰 경로를 allowed 부위로 제한한 전신 시나리오.

    allowed는 러너가 매 프레임 "세기 ≥1인 부위"로 갱신한다 — 순찰 중 앱이
    세기를 0으로 내리면 다음 부위 선택부터 빠지고, 다시 올리면 (스캔 때 기록된
    웨이포인트가 있으므로) 즉시 경로에 복귀한다. 경로 길이가 변하면 순찰
    인덱스가 한 부위를 건너뛰거나 반복할 수 있지만 한 사이클 안에서 정리된다.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.allowed: set[str] = {"head", "upper", "lower"}

    def _route(self):
        return [r for r in self.PATROL_ORDER
                if r in self.waypoints and r in self.allowed]


def patrol_wind_level(scenario: FullBodyScenario, levels: dict[str, int]) -> int:
    """순찰 중 조준 부위의 세기, 그 외 상태는 0 (모듈 docstring 참고)."""
    if scenario.state != "patrol":
        return 0
    return levels.get(scenario.active_region(), 0)


class MotionGate:
    """순찰 ↔ 추적 폴백 전환 판정 (순수 계산, 프레임 단위 호출).

    exit (순찰→폴백): |가슴 수평 오차| > exit_err_deg 가 exit_frames 연속.
      가슴 미관측 프레임은 카운트를 유지만 한다(증가도 리셋도 안 함) —
      하체 조준 중 가슴이 화면 밖인 것은 정상이고, 진짜 상실은 시나리오의
      search 상태가 다룬다.
    enter (폴백→순찰): 최근 enter_still_s 동안 팬 각의 범위(최대-최소)가
      enter_span_deg 미만이고 이번 프레임에 가슴이 보이면. 폴백 추적은 가슴을
      화면 중앙에 붙잡아 두므로(사용자가 움직이면 팬이 따라 돎) 팬 각이
      잠잠하다 = 사용자가 정지했다. 폴백 진입 직후에는 창이 찰 때까지
      판정을 보류한다.
    """

    def __init__(self, exit_err_deg: float = 12.0, exit_frames: int = 5,
                 enter_still_s: float = 5.0, enter_span_deg: float = 3.0) -> None:
        self.exit_err_deg = exit_err_deg
        self.exit_frames = exit_frames
        self.enter_still_s = enter_still_s
        self.enter_span_deg = enter_span_deg
        self._exit_cnt = 0
        self._pan_hist: deque[tuple[float, float]] = deque()  # (t, pan°)

    def reset_patrol(self) -> None:
        self._exit_cnt = 0

    def reset_fallback(self) -> None:
        self._pan_hist.clear()

    def update_patrol(self, chest_visible: bool, pan_err_deg: float) -> bool:
        """순찰 프레임마다 호출 — True면 추적 폴백으로 나가라."""
        if not chest_visible:
            return False
        if abs(pan_err_deg) > self.exit_err_deg:
            self._exit_cnt += 1
        else:
            self._exit_cnt = 0
        if self._exit_cnt >= self.exit_frames:
            self._exit_cnt = 0
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
