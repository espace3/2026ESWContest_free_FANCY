"""
scripts/verify_track_tilt.py - 카메라 기반 틸트 추적(닫힌 루프) 3단계 검증 스크립트

verify_track_pan.py의 틸트판. 팬은 0°에 고정하고(사용자가 좌우 중앙에 서면 됨)
조준점이 화면 세로 중앙(--target-cy)에 오도록 틸트만 피드백 제어한다.
verify_fulltrack.py(전신 시나리오) 전에 틸트의 미검증 요소를 단독으로 확인하는 용도:

  1. 방향: dir_for_positive 기준 미기록 (config.py) — "화면 아래 = +틸트"가 맞는지
     확인하고, 반대면 --invert로 뒤집어 보며 실기 방향을 확정한다.
  2. 수렴/속도: 실측 순항 ≈11°/s (hardware/TODO.md) — 닫힌 루프에서 체감 지연과
     진동 여부 확인 (진동하면 --gain을 낮출 것).
  3. 소프트 리밋: 모든 목표각을 config "limits".tilt로 clamp — 리밋에 눌린 채
     오차가 남는 상황(LIMIT 표시)이 데드라인 문제의 신호다.

--region head/upper/lower로 가슴 대신 부위 중심을 조준하면 fulltrack 스캔의
부위별 수렴을 미리 시험할 수 있다. 피드백 원리(거리 무관)는 verify_track_pan.py
docstring 참고 — 수직도 동일하다.

주의: 틸트는 웜기어라 수동 복귀가 어렵다 — 종료 시 자동으로 0°에 복귀한다.

실행 (RPi 5, 레포 루트에서): (--rpicam 필수)
    python scripts/verify_track_tilt.py                     # 가슴 수직 추적 (창 표시)
    python scripts/verify_track_tilt.py --gain 0.1          # 방향 확인용 저게인
    python scripts/verify_track_tilt.py --invert            # 방향 반전
    python scripts/verify_track_tilt.py --region lower      # 하체 조준 (스캔 예행)
    python scripts/verify_track_tilt.py --web --no-window   # SSH: 브라우저 확인
    python scripts/verify_track_tilt.py --dry-run --opencv  # 개발 PC, 모터 없이
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
# 루프 본문/헬퍼는 tracking_core로 추출됨 (verify_track_pan/pantilt와 공유).
from tracking_core import add_state_args, open_motor_from_args, run_tilt_tracking
from verify_movenet import (_open_camera, _release_camera,
                            _WebStreamState, _make_handler, _ThreadedHTTP)


def main() -> None:
    lim = CFG["limits"]["tilt"]
    p = argparse.ArgumentParser(description="카메라 기반 틸트 추적(닫힌 루프) 검증")
    p.add_argument("--model", default="multipose_lightning.tflite")
    p.add_argument("--conf", type=float, default=0.25, help="키포인트 신뢰도 임계값")
    p.add_argument("--threads", type=int, default=3, help="TFLite 스레드 수")
    # ── 제어 파라미터 (시작값은 임의 — 실기에서 튜닝) ─────────────────────────
    p.add_argument("--gain", type=float, default=0.3,
                   help="틸트 비례 게인. 방향 미검증 초기엔 0.1 권장, 진동하면 낮출 것")
    p.add_argument("--deadzone", type=float, default=0.5,
                   help="목표각 변화가 이 각도(°) 이하면 모터에 안 보냄 (떨림 억제)")
    p.add_argument("--target-cy", type=float, default=0.5,
                   help="조준점을 맞출 화면상 목표 y (0~1, 기본 0.5=중앙)")
    p.add_argument("--tilt-min", type=float, default=lim["min"],
                   help="틸트 소프트 리밋 하한 ° (기본 config limits — 데드라인 실측 전 임시값)")
    p.add_argument("--tilt-max", type=float, default=lim["max"])
    p.add_argument("--invert", action="store_true",
                   help="오차 부호 반전 (tilt dir_for_positive 방향 실기 확정용)")
    p.add_argument("--region", choices=("chest", "head", "upper", "lower"), default="chest",
                   help="조준 부위 (기본 chest=어깨 중점, 나머지는 부위 중심 — 스캔 예행)")
    # ── 카메라 백엔드 (verify_movenet과 동일) ────────────────────────────────
    p.add_argument("--opencv", action="store_true")
    p.add_argument("--rpicam", action="store_true", help="rpicam-vid 서브프로세스 캡처")
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--no-window", action="store_true")
    p.add_argument("--dry-run", action="store_true",
                   help="모터/lgpio 없이 각도 계산만 (개발 PC 검증용)")
    add_state_args(p)
    # ── 웹 스트림 ────────────────────────────────────────────────────────────
    p.add_argument("--web", action="store_true", help="MJPEG 웹 스트림 송출")
    p.add_argument("--web-host", default="0.0.0.0")
    p.add_argument("--web-port", type=int, default=8090)
    p.add_argument("--web-quality", type=int, default=75)
    p.add_argument("--web-fps", type=float, default=20.0)
    args = p.parse_args()

    if not Path(args.model).exists():
        print(f"[ERROR] 모델 없음: {args.model}")
        sys.exit(1)
    if args.tilt_min >= args.tilt_max:
        print("[ERROR] --tilt-min은 --tilt-max보다 작아야 합니다")
        sys.exit(1)

    fov_v = CFG["fov"]["v"]
    sign = -1.0 if args.invert else 1.0
    aim_key = "upper" if args.region == "chest" else args.region

    detector = MoveNetMultiPoseDetector(args.model, conf_thr=args.conf, num_threads=args.threads)
    cam, backend = _open_camera(args.opencv, args.cam, use_rpicam=args.rpicam)
    tracker = PoseTracker()

    web_srv = web_state = None
    if args.web:
        web_state = _WebStreamState(args.web_quality, args.web_fps)
        web_srv = _ThreadedHTTP((args.web_host, args.web_port), _make_handler(web_state))
        threading.Thread(target=web_srv.serve_forever, daemon=True).start()
        print(f"[web] http://{args.web_host}:{args.web_port}/  (브라우저에서 열기)")

    print("\n[verify_track_tilt] 틸트 추적 시작 — 카메라가 팬·틸트 헤드에 함께 실린 상태를 가정합니다.")
    print(f"  gain={args.gain}  deadzone={args.deadzone}°  target_cy={args.target_cy}  "
          f"region={args.region}  tilt=[{args.tilt_min:g},{args.tilt_max:g}]°  invert={args.invert}")

    # 단독 실행 시에는 아무도 set하지 않는 stop_event — while True와 동일하게 동작.
    stop_event = threading.Event()
    motor_cm = open_motor_from_args(args)
    try:
        with motor_cm as mc:
            mc.enable()
            # 이전 실행이 돌아간 채 꺼졌으면 저장 위치만큼 되돌아와 중앙을 본다.
            # 첫 실행이면 현재 위치가 틸트 0°가 된다 — 시작 위치 마커 권장.
            mc.restore_origin()
            try:
                run_tilt_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                                  fov_v, sign, aim_key, web_state=web_state)
            finally:
                # 웜기어는 수동 복귀가 어려우므로 종료 시 0°로 되돌린다
                print("\n[verify_track_tilt] 틸트 0° 복귀 중...")
                mc.move_to(0.0, 0.0)
                if not mc.wait_until_idle(timeout=30):
                    print("[verify_track_tilt] 복귀 미완료 — 물리 위치를 확인하세요.")

    except KeyboardInterrupt:
        print("\n[verify_track_tilt] Ctrl+C 중단")
    finally:
        if web_srv:
            web_srv.shutdown(); web_srv.server_close()
        _release_camera(cam, backend)
        cv2.destroyAllWindows()
        print("\n[verify_track_tilt] 종료")


if __name__ == "__main__":
    main()
