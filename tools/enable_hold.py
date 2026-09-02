"""
tools/enable_hold.py - EN 유지 (백래시 손측정용) — 모터를 잠근 채 대기한다.

계산 로직 없음. EN을 켜고 엔터를 누를 때까지 대기하기만 한다.

로터가 잠겨 있어야 손으로 헤드를 흔들었을 때 움직이는 양이 "모터가 헛도는 양"이
아니라 순수한 기어 유격(백래시)이 된다. tools/drive_motor.py는 종료 시 자동으로
disable하므로 그걸로는 잴 수 없다.

측정 절차 (docs/angle_calibration.md 실험 B):
    1. 이 스크립트 실행 → 모터가 잠긴다
    2. 축 중심에서 거리 r인 지점에 표시, r을 자로 재둔다
    3. 회전 방향으로 한쪽 끝까지 밀어붙인 상태에서 연필로 점, 반대쪽도 점
    4. 두 점 사이 거리 d 측정 → 유격 = 57.3 × d / r (도)
    5. 엔터로 종료

실행:
    python tools/enable_hold.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from config import CFG
from hardware.stepper import MotorController


def main() -> None:
    # 이동하지 않으므로 restore_origin()을 부르지 않는다 — 장부 기준이 없는 채로
    # 두면 close()가 상태 파일을 덮지 않아 저장된 위치가 보존된다.
    with MotorController(CFG) as mc:
        mc.enable()
        print("[hold] 두 축 enable — 로터가 잠겼습니다. 헤드를 회전 방향으로 흔들어 보세요.")
        print("[hold] 유격 = 57.3 × d / r  (d: 양 끝점 사이 거리, r: 축 중심에서의 거리)")
        input("[hold] 측정이 끝나면 엔터를 누르세요 > ")


if __name__ == "__main__":
    main()
