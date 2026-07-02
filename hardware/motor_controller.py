"""
hardware/motor_controller.py

팬틸트 스테퍼 모터 구동 — 하드웨어 호출 전용 모듈 (STUB).
control/control_signal_generator.py가 계산한 각도(degree)를 받아 실제로
GPIO STEP/DIR 펄스를 내보내는 역할만 한다. 여기에 계산 로직을 넣지 말 것.

TODO — 실제 하드웨어가 정해지면 채울 것:
  - 스테퍼 드라이버 모델 (A4988 / DRV8825 / TMC2209 등) 및 마이크로스테핑 설정
  - GPIO 핀 배정 (STEP/DIR/ENABLE, pan/tilt 각각) — config.py의 "pins"에 추가
  - 스텝당 각도 (모터 step/rev, 감속비 반영: pan=유성기어, tilt=웜기어)
  - 호밍(원점) 방식: 리밋 스위치 유무, 없다면 전원 인가 시 위치를 0으로 가정할지 여부
  - 회전 한계(min/max angle) 초과 시 동작 (하드 스톱 보호)
  - 펄스 타이밍 루프 구현체 (time.sleep 기반 busy-wait vs. 별도 스레드/타이머)

지금은 인터페이스만 정의되어 있고 구현은 비어 있다 (NotImplementedError).
"""

from __future__ import annotations


class MotorController:
    def __init__(self) -> None:
        # TODO: GPIO 초기화 (핀 모드 설정 등)
        self._pan_angle = 0.0
        self._tilt_angle = 0.0

    def home(self) -> None:
        """원점(호밍) 시퀀스. 리밋 스위치가 있다면 그쪽으로 이동 후 0점 설정."""
        raise NotImplementedError

    def move_to(self, pan_angle_deg: float, tilt_angle_deg: float) -> None:
        """목표 각도로 이동. control_signal_generator에서 이미 회전 한계 내로
        clamp된 각도가 들어온다고 가정한다 (여기서는 재계산하지 않음)."""
        raise NotImplementedError

    def current_position(self) -> tuple[float, float]:
        """마지막으로 이동한 (pan_angle_deg, tilt_angle_deg)."""
        return self._pan_angle, self._tilt_angle
