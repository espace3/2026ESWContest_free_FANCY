"""
scripts/verify_track_pantilt.py - 카메라 기반 팬+틸트 동시 추적(닫힌 루프) 3단계 검증

verify_track_pan.py(팬 단독)와 verify_track_tilt.py(틸트 단독)를 합친 스텝. 두 축을
동시에 피드백 제어해서 사용자의 어깨 중심(가슴)을 화면 조준점(--target-cx/cy)에
지속적으로 붙잡는다. fulltrack(전신 시나리오) 전에 "두 축 동시 추적"만 떼어내
검증하는 단계다. 이 파일도 계산 로직을 담지 않는다(각도/데드존/리밋은 control,
추정/선정/스무딩은 vision, 구동은 hardware).

[두 축을 어긋나지 않게 합치는 원칙 — 한 프레임 = 한 관측 = 한 명령]
  팬은 가로오차(cx), 틸트는 세로오차(cy)를 보정하며 서로 직교한다. 하지만 두 축이
  서로 다른 순간의 관측이나 위치로 움직이면 어긋날 수 있으므로, 한 루프에서
    (1) chest를 한 번만 관측하고
    (2) current_position()을 한 번만 스냅샷해서 (cur_pan, cur_tilt)를 얻고
    (3) 두 목표각을 그 하나의 관측·스냅샷에서 함께 계산한 뒤
    (4) move_to(pan, tilt)로 한 번에 내보낸다.
  이렇게 하면 두 축이 항상 같은 정보 위에서 움직인다(최단경로 = 두 축 동시 진행).
  단, 팬은 직결이라 빠르고 틸트는 92.6:1 웜기어라 ≈11°/s로 느려서, 대각선 이동 시
  팬이 먼저 도달하고 틸트가 뒤따른다(경로가 완전한 직선은 아님) — 정상이다.

주의: 틸트는 웜기어라 수동 복귀가 어렵다 — 종료 시 (0°,0°)로 자동 복귀한다.

실행 (RPi 5, 레포 루트에서, --rpicam 권장):
    python scripts/verify_track_pantilt.py                     # 어깨 중심 추적 (창 표시)
    python scripts/verify_track_pantilt.py --gain 0.3 --gain-tilt 0.2
    python scripts/verify_track_pantilt.py --web --no-window   # SSH: 브라우저 확인
    python scripts/verify_track_pantilt.py --dry-run --opencv  # 개발 PC, 모터 없이
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
# 루프 본문/헬퍼는 tracking_core로 추출됨 (verify_track_pan/tilt와 공유).
from tracking_core import _open_motor, run_pantilt_tracking
from verify_movenet import (_open_camera, _release_camera,
                            _WebStreamState, _make_handler, _ThreadedHTTP)


def main() -> None:
    lim = CFG["limits"]
    p = argparse.ArgumentParser(description="카메라 기반 팬+틸트 동시 추적(닫힌 루프) 검증")
    p.add_argument("--model", default="multipose_lightning.tflite")
    p.add_argument("--conf", type=float, default=0.25, help="키포인트 신뢰도 임계값")
    p.add_argument("--threads", type=int, default=3, help="TFLite 스레드 수")
    # ── 제어 파라미터 (시작값은 임의 — 실기에서 튜닝) ─────────────────────────
    p.add_argument("--gain", type=float, default=0.3, help="팬 비례 게인")
    p.add_argument("--gain-tilt", type=float, default=0.2,
                   help="틸트 비례 게인 (웜기어라 느려서 팬보다 낮게, 진동하면 더 낮출 것)")
    p.add_argument("--deadzone", type=float, default=1.0, help="팬 데드존 (°)")
    p.add_argument("--deadzone-tilt", type=float, default=0.5, help="틸트 데드존 (°)")
    p.add_argument("--target-cx", type=float, default=0.5,
                   help="가슴을 맞출 화면상 목표 x (0~1, 기본 0.5=중앙)")
    p.add_argument("--target-cy", type=float, default=0.5,
                   help="가슴을 맞출 화면상 목표 y (0~1, 기본 0.5=중앙)")
    p.add_argument("--pan-min", type=float, default=lim["pan"]["min"])
    p.add_argument("--pan-max", type=float, default=lim["pan"]["max"])
    p.add_argument("--tilt-min", type=float, default=lim["tilt"]["min"],
                   help="틸트 소프트 리밋 하한 ° (기본 config limits)")
    p.add_argument("--tilt-max", type=float, default=lim["tilt"]["max"])
    p.add_argument("--invert", action="store_true", help="팬 오차 부호 반전 (dir 확정됨, 안전용)")
    p.add_argument("--invert-tilt", action="store_true", help="틸트 오차 부호 반전 (dir 확정됨, 안전용)")
    # ── 카메라 백엔드 (verify_movenet과 동일) ────────────────────────────────
    p.add_argument("--opencv", action="store_true")
    p.add_argument("--rpicam", action="store_true", help="rpicam-vid 서브프로세스 캡처")
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--no-window", action="store_true")
    p.add_argument("--dry-run", action="store_true",
                   help="모터/lgpio 없이 각도 계산만 (개발 PC 검증용)")
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
    if args.tilt_min >= args.tilt_max or args.pan_min >= args.pan_max:
        print("[ERROR] 리밋 범위가 비어 있습니다 (min < max 필요)")
        sys.exit(1)

    fov_h, fov_v = CFG["fov"]["h"], CFG["fov"]["v"]
    sign_pan = -1.0 if args.invert else 1.0
    sign_tilt = -1.0 if args.invert_tilt else 1.0

    detector = MoveNetMultiPoseDetector(args.model, conf_thr=args.conf, num_threads=args.threads)
    cam, backend = _open_camera(args.opencv, args.cam, use_rpicam=args.rpicam)
    tracker = PoseTracker()

    web_srv = web_state = None
    if args.web:
        web_state = _WebStreamState(args.web_quality, args.web_fps)
        web_srv = _ThreadedHTTP((args.web_host, args.web_port), _make_handler(web_state))
        threading.Thread(target=web_srv.serve_forever, daemon=True).start()
        print(f"[web] http://{args.web_host}:{args.web_port}/  (브라우저에서 열기)")

    print("\n[verify_track_pantilt] 팬+틸트 동시 추적 시작 — 카메라가 팬·틸트 헤드에 함께 실린 상태를 가정합니다.")
    print(f"  gain={args.gain}/{args.gain_tilt}  deadzone={args.deadzone}/{args.deadzone_tilt}°  "
          f"target=({args.target_cx},{args.target_cy})  tilt=[{args.tilt_min:g},{args.tilt_max:g}]°  "
          f"invert={args.invert}/{args.invert_tilt}")

    # 단독 실행 시에는 아무도 set하지 않는 stop_event — while True와 동일하게 동작.
    stop_event = threading.Event()
    motor_cm = _open_motor(args.dry_run)
    try:
        with motor_cm as mc:
            mc.enable()
            mc.home()
            print("[motor] 현재 위치를 (0°, 0°)로 설정 (임시 호밍).")
            try:
                run_pantilt_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                                     fov_h, fov_v, sign_pan, sign_tilt, web_state=web_state)
            finally:
                # 웜기어는 수동 복귀가 어려우므로 종료 시 (0°,0°)로 되돌린다
                print("\n[verify_track_pantilt] (0°,0°) 복귀 중...")
                mc.move_to(0.0, 0.0)
                if not mc.wait_until_idle(timeout=30):
                    print("[verify_track_pantilt] 복귀 미완료 — 물리 위치를 확인하세요.")

    except KeyboardInterrupt:
        print("\n[verify_track_pantilt] Ctrl+C 중단")
    finally:
        if web_srv:
            web_srv.shutdown(); web_srv.server_close()
        _release_camera(cam, backend)
        cv2.destroyAllWindows()
        print("\n[verify_track_pantilt] 종료")


if __name__ == "__main__":
    main()
