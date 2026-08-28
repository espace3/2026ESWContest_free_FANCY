"""
hardware/position_store.py

팬틸트 장부 위치(정수 스텝)를 파일에 남기고 읽는다 — 파일 I/O 전용.
GPIO도, 각도 계산도 여기 없다.

[왜 필요한가]
스테퍼는 오픈루프라 "지금 내가 몇 도에 있는지"를 스스로 알 방법이 없다.
MotorController.home()은 원점을 찾는 게 아니라 "지금 이 위치를 0°라고 치자"는
선언일 뿐이라, 헤드가 30° 돌아간 채 꺼지면 다음 실행은 그 자리를 0°로 삼는다.
리밋 스위치/엔코더를 달기 전까지는 "보낸 펄스 수(장부)를 꺼지기 전에 파일로
남겨두고, 다시 켰을 때 그만큼 되돌아와 중앙을 보게 하는" 방법뿐이다.

파일에는 정수 스텝을 적는다(장부의 원본 단위). pan_deg/tilt_deg도 함께 적지만
사람이 cat으로 확인하라고 넣은 참고값이고, 복원은 스텝으로만 한다.

[원자적 쓰기를 하는 이유]
이 파일을 읽어야 하는 상황 자체가 "전원이 갑자기 끊긴 다음"이다. 그냥
덮어쓰면 쓰는 도중 끊겼을 때 반쪽짜리 JSON이 남아 위치를 통째로 잃는다.
임시 파일에 다 쓰고 fsync로 확정한 뒤 os.replace()로 갈아끼우면(같은 디렉터리
내 rename은 원자적) 어느 시점에 끊기든 항상 직전의 완전한 값이 남는다.

[이 방식으로 알 수 없는 것 — 한계]
  - 탈조: 장부는 "보낸 펄스"라서 모터가 놓친 스텝은 반영되지 않는다.
  - 마지막 저장 이후의 이동분: 주기 저장이라 전원이 뚝 끊기면 그만큼 유실된다.
헤드와 베이스에 0° 물리 마커를 표시해두고 복원 후 눈으로 대조할 것
(hardware/TODO.md 호밍 항목).
"""

from __future__ import annotations

import json
import os
import threading
from pathlib import Path


class PositionStore:
    """장부 위치(정수 스텝) 한 쌍을 파일 하나에 읽고 쓴다.

    한 대의 팬틸트 헤드 = 파일 하나다. 스크립트마다 다른 경로를 쓰면 A로
    돌려놓고 B로 실행했을 때 B가 낡은 값으로 엉뚱하게 움직이므로, 모든
    스크립트가 같은 기본 경로(config "motor_state".file)를 쓴다.
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        # save()는 저장 스레드와 메인 스레드(restore_origin/close) 양쪽에서 불린다.
        # 둘이 겹치면 같은 .tmp 파일을 서로 truncate해서 깨진 내용이 그대로
        # os.replace될 수 있으므로 직렬화한다.
        self._lock = threading.Lock()

    def load(self) -> tuple[int, int] | None:
        """저장된 (pan_steps, tilt_steps). 파일이 없거나 읽을 수 없으면 None."""
        try:
            st = json.loads(self.path.read_text())
            return int(st["pan_steps"]), int(st["tilt_steps"])
        except FileNotFoundError:
            return None
        except (OSError, ValueError, TypeError, KeyError) as e:
            print(f"[state] {self.path} 를 읽을 수 없습니다({e}) — 복원 생략")
            return None

    def save(self, pan_steps: int, tilt_steps: int,
             pan_deg: float, tilt_deg: float) -> None:
        """원자적으로 덮어쓴다 (임시 파일 → fsync → os.replace → 디렉터리 fsync)."""
        payload = {
            "pan_steps": int(pan_steps),
            "tilt_steps": int(tilt_steps),
            # 아래 둘은 사람이 읽으라고 넣는 참고값 — 복원은 steps로만 한다
            "pan_deg": round(pan_deg, 4),
            "tilt_deg": round(tilt_deg, 4),
        }
        directory = self.path.parent
        tmp = self.path.with_name(self.path.name + ".tmp")
        with self._lock:
            directory.mkdir(parents=True, exist_ok=True)
            with open(tmp, "w") as f:
                json.dump(payload, f)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.path)
            # rename 자체를 디스크에 확정 (안 하면 전원 차단 시 교체가 날아갈 수 있다)
            dir_fd = os.open(directory, os.O_RDONLY)
            try:
                os.fsync(dir_fd)
            finally:
                os.close(dir_fd)
