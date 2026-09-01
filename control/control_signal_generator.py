"""
control/control_signal_generator.py

카메라 좌표계 → 모터 제어값 변환, 거리 추정 — 순수 계산 모듈.
GPIO/UART/BlueZ 등 하드웨어 라이브러리를 import하지 않습니다.
입력은 vision/ 모듈이 만든 키포인트·정규화 좌표, 출력은 각도(degree)·거리(m)
같은 순수 값입니다. 실제로 모터를 움직이는 코드는
hardware/motor_controller.py 쪽 몫입니다.

주의: 선풍기 풍속(바람 세기)은 이 모듈이 다루는 대상이 아닙니다 — 풍속은
부위(머리/상체/하체)별로 사용자가 앱에서 BLE로 직접 지정하는 값이라 카메라
거리 추정과 무관합니다 (그쪽은 hardware/relay_controller.py + BLE 서버가
담당).
"""

from __future__ import annotations


def compute_pan_angle(cx_norm: float, fov_h_deg: float) -> float:
    """정규화된 x좌표(0~1, 0.5=화면 중앙)를 pan 각도(degree)로 변환.

    pan_angle = (pixel_x - frame_center_x) / frame_width * FOV_h_degree
    를 정규화 좌표 기준으로 표현한 것 (cx_norm - 0.5) * fov_h_deg.
    화면 중앙보다 오른쪽(x가 큼)이면 양수를 반환한다.
    """
    return (cx_norm - 0.5) * fov_h_deg


def compute_tilt_angle(cy_norm: float, fov_v_deg: float) -> float:
    """정규화된 y좌표(0~1, 0.5=화면 중앙)를 tilt 각도(degree)로 변환.

    화면 중앙보다 아래(y가 큼)면 양수를 반환한다.
    """
    return (cy_norm - 0.5) * fov_v_deg


def clamp_angle(angle_deg: float, min_deg: float, max_deg: float) -> float:
    """모터 회전 한계 내로 각도를 자른다. (실제 GPIO 호출 전, 순수 계산 단계에서 처리)"""
    return max(min_deg, min(max_deg, angle_deg))


def apply_deadzone(new_angle_deg: float, last_sent_angle_deg: float, deadzone_deg: float) -> float:
    """새로 계산된 각도와 '마지막으로 모터에 실제 전송한' 각도의 차이가
    deadzone_deg 이하면 이전 값을 그대로 반환해 모터를 움직이지 않게 한다.

    포즈 추정 잡음으로 목표 지점이 미세하게 계속 흔들려도(사람은 정지 상태),
    이 함수를 거친 뒤의 값을 모터에 보내면 오차가 임계값을 넘을 때만 실제로
    움직인다. PoseTracker의 EMA는 좌표 추정 자체를 부드럽게 만드는 역할이고,
    이 데드존은 '그 부드러운 값도 여전히 미세하게 흔들릴 때 모터까지 그 흔들림이
    전달되지 않게' 마지막 단계에서 한 번 더 걸러주는 역할이다.

    주의: 상태(마지막 전송 각도)는 호출하는 쪽(예: 메인 루프)이 들고 있다가
    매번 넘겨줘야 한다 — 이 모듈은 순수 함수만 두고 상태를 갖지 않는다.
    """
    if abs(new_angle_deg - last_sent_angle_deg) <= deadzone_deg:
        return last_sent_angle_deg
    return new_angle_deg


def estimate_distance_m(
    shoulder_px_dist: float,
    ref_shoulder_cm: float,
    focal_px: float,
    min_m: float | None = None,
    max_m: float | None = None,
) -> float | None:
    """양 어깨 키포인트 간 픽셀 거리로 카메라-사람 간 거리(m)를 추정한다 (핀홀 카메라 모델).

    distance_cm = ref_shoulder_cm * focal_px / shoulder_px_dist

    shoulder_px_dist가 0 이하면 (어깨가 감지되지 않은 경우) None을 반환한다.

    min_m/max_m을 주면 결과를 그 범위로 클램프한다 — 키포인트 잡음으로 어깨
    픽셀 거리가 순간적으로 튀면 물리적으로 말이 안 되는 거리(수십 m / 수 cm)가
    나올 수 있어서, 유효 사용 범위(config.py distance.min_m/max_m)로 잘라낸다.
    """
    if shoulder_px_dist <= 0:
        return None
    distance_m = ref_shoulder_cm * focal_px / shoulder_px_dist / 100.0
    if min_m is not None and distance_m < min_m:
        return min_m
    if max_m is not None and distance_m > max_m:
        return max_m
    return distance_m
