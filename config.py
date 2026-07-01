"""
target_fan / config.py
전체 프로젝트 설정을 한 곳에서 관리합니다.
하드웨어 핀, 제어 파라미터, 서버 설정 등 모든 상수를 여기서 수정하세요.
"""

CFG: dict = {
    # ── 카메라 ──────────────────────────────────────────────────────────────
    "camera": {
        "width":  640,
        "height": 480,
        # Pi 4B 기준으로 기본 FPS를 낮춰 CPU 여유를 확보합니다.
        "fps":    20,
        # "picamera2" | "opencv"  (Pi에서는 picamera2, 개발PC에서는 opencv)
        "backend": "picamera2",
    },

    # ── GPIO 핀 (BCM 번호) ──────────────────────────────────────────────────
    "pins": {
        "pan_servo":  12,   # Pan  서보 PWM (하드웨어 PWM 핀)
        "tilt_servo": 13,   # Tilt 서보 PWM (하드웨어 PWM 핀)
        "ir_led":     18,   # IR LED (pigpio 소프트 PWM)
        "ir_rx":      17,   # IR 수신 (TSOP38238)
    },

    # ── 서보모터 ─────────────────────────────────────────────────────────────
    "servo": {
        "pan": {
            "min_angle": -60,   # 좌 최대 (degree)
            "max_angle":  60,   # 우 최대 (degree)
            "kp":          0.4, # P 게인 (추적 응답 속도)
            "kd":          0.1, # D 게인 (진동 억제)
            "deadzone":    2.0, # 이 각도 이하 오차는 무시 (degree)
            "max_step":    5.0, # 한 틱 최대 이동량 (degree, 부드러운 추적)
        },
        "tilt": {
            "min_angle": -30,
            "max_angle":  30,
            "kp":          0.3,
            "kd":          0.08,
            "deadzone":    2.0,
            "max_step":    4.0,
        },
        # 서보 PWM 파라미터 (50Hz, pulse width μs)
        "pwm_freq":      50,
        "pulse_min_us":  500,   # 최소 펄스폭 → 최소 각도
        "pulse_max_us": 2500,   # 최대 펄스폭 → 최대 각도
    },

    # ── 카메라 FOV (화각) ────────────────────────────────────────────────────
    "fov": {
        "h": 60,  # 수평 화각 (degree) — Pi Camera Module 3 기준
        "v": 40,  # 수직 화각 (degree)
    },

    # ── 거리 추정 ─────────────────────────────────────────────────────────────
    "distance": {
        "ref_shoulder_cm": 40,   # 성인 평균 어깨 너비 (cm)
        "focal_px":        640,  # 추정 초점거리 (픽셀). FOV 60° → width 정도
        "min_m":           0.3,  # 최소 유효 거리
        "max_m":           5.0,  # 최대 유효 거리
    },

    # ── IR 풍속 거리 구간 (m) ────────────────────────────────────────────────
    "speed_zones": {
        "near": 1.0,   # ~1m  → 1단 (약풍)
        "mid":  2.0,   # ~2m  → 2단 (중풍)
        # 2m 초과 → 3단 (강풍)
    },

    # ── IR 리모컨 ────────────────────────────────────────────────────────────
    "ir": {
        "codes_path":    "ir_codes.json",  # 학습된 IR 코드 저장 파일
        "emit_repeat":   3,                # 신호 재전송 횟수 (안정성)
        "emit_gap_ms":   100,              # 재전송 간격 (ms)
    },

    # ── MediaPipe Pose ───────────────────────────────────────────────────────
    "pose": {
        "min_detection_confidence": 0.6,
        "min_tracking_confidence":  0.5,
        "model_complexity":         0,    # 0=Lite(빠름), 1=Full, 2=Heavy
    },

    # ── 회피 제어 파라미터 ────────────────────────────────────────────────────
    "avoid": {
        "head_cy_threshold": 0.40,   # cy < 이값 이면 머리가 중앙 위 → tilt 아래로
        "tilt_offset_deg":   8.0,    # 회피 시 tilt 보정량 (degree)
    },

    # ── sweep 모드 ───────────────────────────────────────────────────────────
    "sweep": {
        "amplitude_deg": 30.0,   # 좌우 스윙 폭 (degree)
        "period_sec":     4.0,   # 왕복 주기 (초)
    },

    # ── WebSocket 서버 ────────────────────────────────────────────────────────
    "server": {
        "host": "0.0.0.0",
        "port": 8765,
        "status_interval_sec": 1.0,   # 상태 브로드캐스트 주기
    },

    # ── LLM 명령 해석 ─────────────────────────────────────────────────────────
    "llm": {
        "enabled":  True,
        "provider": "gemini",              # "gemini" | "openai"
        "model":    "gemini-1.5-flash",    # 또는 "gpt-4o-mini"
        "api_key_env": "GEMINI_API_KEY",   # 환경변수 이름
        "timeout_sec": 5.0,
    },

    # ── 메인 루프 ─────────────────────────────────────────────────────────────
    "loop_sleep":    0.10,   # 100ms → 최대 10fps (Pi 4B 기본)

    # ── 로깅 ──────────────────────────────────────────────────────────────────
    "log_interval_sec": 2.0,
}
