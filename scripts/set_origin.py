"""
scripts/set_origin.py - 지금 헤드가 향한 곳을 영점(0°)으로 잡는다.

손으로 헤드를 중앙(정면)에 맞춘 뒤 한 번 실행하면 끝이다. 모터는 움직이지 않고,
현재 위치가 0°라는 사실만 상태 파일(config "motor_state".file)에 기록한다.

이후 모든 스크립트가 이 영점을 기준으로 동작한다:
  - 모터가 움직일 때마다 이동량이 상태 파일에 더해지고
  - 재시작하면 기록된 만큼 되돌아와 다시 0°를 본다

실행 (레포 루트에서):
    python scripts/set_origin.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from config import CFG
from hardware.motor_controller import MotorController


def main() -> None:
    # home()이 "지금 위치 = 0°"를 선언하고, close()가 그 장부를 파일에 기록한다.
    # enable()은 하지 않는다 — 움직일 일이 없다.
    with MotorController(CFG) as mc:
        mc.home()
    print(f"영점을 설정했습니다 (0°, 0°) → {CFG['motor_state']['file']}")
    print("헤드와 베이스에 마커를 표시해두면 나중에 복원 결과를 눈으로 대조할 수 있습니다.")


if __name__ == "__main__":
    main()
