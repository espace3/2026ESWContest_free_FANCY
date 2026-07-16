"""
scripts/verify_track.py - 카메라 기반 팬 추적(닫힌 루프) 3단계 검증 스크립트

목적: 카메라를 팬 헤드에 얹은 상태에서 "선정된 사용자의 가슴(=어깨 중점)이 화면
중앙으로 오도록" 팬 모터를 피드백 제어하는 최소 루프를 검증한다. 틸트는 이번
단계에서 다루지 않는다(팬만 — hardware/TODO.md 및 아래 [피드백 원리] 참고).

이 파일도 다른 verify 스크립트처럼 계산 로직을 담지 않는다:
  - 추정/선정/스무딩:  vision/pose_estimator, target_selector, pose_tracker
  - 좌표→각도/데드존:  control/control_signal_generator
  - 실제 모터 구동:     hardware/motor_controller
여기서는 이들을 엮어 "프레임 → 오차각 → 모터 목표각"을 매 프레임 돌리고,
수렴 상태를 로그로 남기기만 한다. 카메라 캡처는 verify_movenet의 백엔드를 재사용한다.

추가로, 어깨가 한쪽만 잡히면 그쪽만, 둘 다 없으면 코(머리)로 폴백한다.

────────────────────────────────────────────────────────────────────────────
[피드백 원리 — 왜 이렇게 각도를 더하는가]

카메라가 팬 헤드 위에 얹혀 모터와 "함께" 돈다. 그래서 이건 절대 조준이 아니라
피드백(상대 보정) 루프다:

    새 팬 목표각 = 현재 모터각(current_position) + GAIN × 오차각
    오차각 = compute_pan_angle(가슴 cx, fov_h) = (cx − 0.5) × fov_h

핵심: 화면 픽셀 위치 → 광축 기준 각도 변환은 초점거리(FOV)만으로 정해지고
"거리에 의존하지 않는다"(핀홀 기하). 그래서 사물까지의 거리를 몰라도(단일
카메라라 몰라도) 팬 중앙 정렬은 정확히 계산된다 — 거리 가정(1.5m 등)이 필요 없다.
거리가 필요해지는 건 거리별 슬루 속도 스케줄링(speed_zones)이나 나중에 렌즈가
아니라 '바람'을 겨냥할 때뿐이다.

────────────────────────────────────────────────────────────────────────────

실행 (RPi 5, 레포 루트에서):
    python scripts/verify_track.py                      # 팬 추적 (창 표시)
    python scripts/verify_track.py --no-window          # 헤드리스 (Pi 실전)
    python scripts/verify_track.py --gain 0.3 --invert  # 방향/게인 초기 튜닝
    python scripts/verify_track.py --dry-run --opencv    # 모터/lgpio 없이 개발 PC에서
                                                         # 오차→목표각 파이프라인만 확인
    python scripts/verify_track.py --rpicam             # picamera2 미설치 시
"""

from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path

import cv2

# 레포 루트 + scripts 디렉터리를 path에 추가 (config/vision/control + 카메라 백엔드 재사용)
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

from config import CFG
from vision.pose_estimator import MoveNetMultiPoseDetector
from vision.pose_tracker import PoseTracker
# 루프 본문/헬퍼는 tracking_core로 추출됨 (verify_track_tilt/pantilt와 공유).
from tracking_core import _open_motor, run_pan_tracking
# 카메라 캡처 백엔드는 verify_movenet 것을 그대로 재사용한다 (중복 구현 방지).
# verify_movenet은 hardware(lgpio)를 import하지 않으므로 --dry-run 개발 PC에서도 안전.
from verify_movenet import (_open_camera, _release_camera,
                            _WebStreamState, _make_handler, _ThreadedHTTP)


def main() -> None:
    p = argparse.ArgumentParser(description="카메라 기반 팬 추적(닫힌 루프) 검증")
    p.add_argument("--model", default="multipose_lightning.tflite")
    p.add_argument("--conf", type=float, default=0.25, help="키포인트 신뢰도 임계값")
    p.add_argument("--threads", type=int, default=3, help="TFLite 스레드 수")
    # ── 제어 파라미터 (실험이라 시작값은 임의 — 실기에서 튜닝) ────────────────
    p.add_argument("--gain", type=float, default=0.2,
                   help="비례 게인. 오차각의 몇 배를 현재각에 더할지 (기본 0.3). "
                        "방향 미검증 초기엔 0.3 권장, 진동하면 낮출 것")
    p.add_argument("--deadzone", type=float, default=1.0,
                   help="목표각 변화가 이 각도(°) 이하면 모터에 안 보냄 (떨림 억제)")
    p.add_argument("--target-cx", type=float, default=0.5,
                   help="가슴을 맞출 화면상 목표 x (0~1, 기본 0.5=중앙). 카메라축과 "
                        "바람축 오프셋 보정 시 이 값으로 흡수 (지금은 추적 확인용)")
    p.add_argument("--limit", type=float, default=100.0,
                   help="팬 목표각 소프트 클램프 ±°. 부호 미검증 단계의 안전 레일 "
                        "(실제 회전 금지 구역 아님)")
    p.add_argument("--invert", action="store_true",
                   help="오차 부호 반전 (dir_for_positive/FOV 부호 실기 검증용, TODO.md)")
    # ── 카메라 백엔드 (verify_movenet과 동일) ─────────────────────────────────
    p.add_argument("--opencv", action="store_true")
    p.add_argument("--wide", action="store_true", help="Camera Module 3 Wide 렌즈 화각 사용")
    p.add_argument("--rpicam", action="store_true", help="rpicam-vid 서브프로세스 캡처")
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--no-window", action="store_true")
    p.add_argument("--dry-run", action="store_true",
                   help="모터/lgpio 없이 각도 계산만 (개발 PC 검증용)")
    # ── 웹 스트림 (SSH 등 디스플레이 없을 때 브라우저로 확인) ──────────────────
    p.add_argument("--web", action="store_true",
                   help="MJPEG 웹 스트림 송출. SSH 환경에선 보통 --web --no-window 로 실행")
    p.add_argument("--web-host", default="0.0.0.0")
    p.add_argument("--web-port", type=int, default=8090)
    p.add_argument("--web-quality", type=int, default=75)
    p.add_argument("--web-fps", type=float, default=20.0)
    args = p.parse_args()

    if not Path(args.model).exists():
        print(f"[ERROR] 모델 없음: {args.model}")
        sys.exit(1)

    fov_h = CFG["fov_wide" if args.wide else "fov"]["h"]
    sign = -1.0 if args.invert else 1.0

    detector = MoveNetMultiPoseDetector(args.model, conf_thr=args.conf, num_threads=args.threads)
    cam, backend = _open_camera(args.opencv, args.cam, use_rpicam=args.rpicam)
    tracker = PoseTracker()

    # 웹 스트림 (verify_movenet의 인프라 재사용) — SSH에서 브라우저로 추적 상태 확인
    web_srv = web_state = None
    if args.web:
        web_state = _WebStreamState(args.web_quality, args.web_fps)
        web_srv = _ThreadedHTTP((args.web_host, args.web_port), _make_handler(web_state))
        threading.Thread(target=web_srv.serve_forever, daemon=True).start()
        print(f"[web] http://{args.web_host}:{args.web_port}/  (브라우저에서 열기)")

    print("\n[verify_track] 팬 추적 시작 — 카메라를 팬 헤드에 얹은 상태를 가정합니다.")
    print(f"  gain={args.gain}  deadzone={args.deadzone}°  target_cx={args.target_cx}  "
          f"limit=±{args.limit}°  invert={args.invert}")

    # 단독 실행 시에는 아무도 set하지 않는 stop_event — while True와 동일하게 동작.
    stop_event = threading.Event()
    motor_cm = _open_motor(args.dry_run)
    try:
        with motor_cm as mc:
            mc.enable()
            mc.home()  # 현재 물리 위치 = 팬 0°. 시작 위치에 마커를 붙여두면 대조에 편함
            print("[motor] 현재 위치를 팬 0°로 설정 (임시 호밍).")

            run_pan_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                             fov_h, sign, web_state=web_state)

    except KeyboardInterrupt:
        print("\n[verify_track] Ctrl+C 중단")
    finally:
        if web_srv:
            web_srv.shutdown(); web_srv.server_close()
        _release_camera(cam, backend)
        cv2.destroyAllWindows()
        print("\n[verify_track] 종료")


if __name__ == "__main__":
    main()
