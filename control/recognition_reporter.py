"""
control/recognition_reporter.py

객체 인식 Status(BLE notify 0x02)를 **언제 보낼지** 정하는 순수 판정 모듈 —
GPIO/BlueZ/카메라를 import하지 않는다. 부위 모드 러너와 일반 타겟 모드
추적 러너가 함께 쓴다(scripts/verify_E2E_v3.py) — 어느 한 모드 전용이 아니라
공통 notify 정책이라 별도 모듈로 둔다.

핵심 문제: 프레임 단위 검출은 경계(먼 거리·부분 가림·역광)에서 빠르게
깜빡인다. "값이 바뀌면 보고"는 그 깜빡임을 그대로 프레임률(≈20Hz)로 BLE에
내보내고(대역·전력 낭비 + 앱 표시 깜빡임), "N프레임 연속 유지"는 교대로
깜빡이면 조건이 영영 안 차 상태가 굳는다. 그래서 시간 창 안의 **검출 비율**로
판정한다 (아래 RecognitionReporter).
"""

from __future__ import annotations

from collections import deque


class RecognitionReporter:
    """최근 창의 검출 비율에 상·하 임계를 따로 둔 슈미트 트리거.

        비율 ≥ on_ratio  → 인식 중
        비율 ≤ off_ratio → 미인식
        그 사이          → 현재 상태 유지 (보고 없음)

    깜빡임은 비율이 중간에 머물러 애매 구간에 갇히므로 notify가 나가지 않고,
    사람이 실제로 들어오거나(비율 → 1) 나가야(→ 0) 한 번 바뀐다.
    min_interval_s는 그래도 남는 경계 진동을 막는 최종 안전장치 —
    이 둘을 합치면 보고는 아무리 나빠도 창당 한 번꼴로 제한된다.
    """

    def __init__(self, window_s: float = 2.0, on_ratio: float = 0.6,
                 off_ratio: float = 0.2, min_interval_s: float = 2.0) -> None:
        self.window_s = window_s
        self.on_ratio = on_ratio
        self.off_ratio = off_ratio
        self.min_interval_s = min_interval_s
        self._hits: deque[tuple[float, bool]] = deque()
        self._state: bool | None = None
        self._last_sent = 0.0

    @property
    def state(self) -> bool | None:
        """마지막으로 보고한 값 (아직 없으면 None)."""
        return self._state

    def reset(self) -> None:
        """세션 경계 — 창을 비우고 다음 확정 값을 처음부터 다시 보고한다."""
        self._hits.clear()
        self._state = None
        self._last_sent = 0.0

    def update(self, t: float, detected: bool) -> bool | None:
        """프레임마다 호출 — 보고할 값이면 그 값을, 아니면 None."""
        self._hits.append((t, detected))
        cutoff = t - self.window_s
        while self._hits and self._hits[0][0] < cutoff:
            self._hits.popleft()
        # 창이 절반도 안 찼으면 판정 보류 — 세션 시작 직후 성급한 보고 방지.
        if self._hits[-1][0] - self._hits[0][0] < self.window_s * 0.5:
            return None
        ratio = sum(1 for _, d in self._hits if d) / len(self._hits)
        if ratio >= self.on_ratio:
            new = True
        elif ratio <= self.off_ratio:
            new = False
        else:
            return None                       # 애매 구간 — 현 상태 유지
        if new == self._state:
            return None
        if self._state is not None and t - self._last_sent < self.min_interval_s:
            return None
        self._state, self._last_sent = new, t
        return new
