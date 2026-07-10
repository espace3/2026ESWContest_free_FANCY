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

실행 (RPi 5, 레포 루트에서):
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
import time
from pathlib import Path

import cv2

# 레포 루트 + scripts 디렉터리를 path에 추가 (config/vision/control + 카메라 백엔드 재사용)
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

from config import CFG
from vision.pose_estimator import MoveNetMultiPoseDetector
from vision.pose_tracker import PoseTracker
from vision.target_selector import select_target, person_center, DEFAULT_MATCH_RADIUS
from control.control_signal_generator import (apply_deadzone, clamp_angle,
                                              compute_tilt_angle)
from verify_movenet import (_open_camera, _read_frame, _release_camera, draw_pose,
                            _WebStreamState, _make_handler, _ThreadedHTTP)

# 어깨 키포인트 인덱스 (COCO): 5=l_shoulder, 6=r_shoulder, 0=nose
_L_SHOULDER, _R_SHOULDER, _NOSE = 5, 6, 0

_INVISIBLE = {"cx": 0.5, "cy": 0.5, "visible": False}


def chest_point(keypoints: list[dict], conf_thr: float) -> dict:
    """가슴 조준점(정규화 cx/cy) = 어깨 중점. 한쪽 어깨만 잡히면 그쪽, 둘 다
    없으면 코 폴백, 그것도 없으면 visible=False."""
    pts = [(keypoints[i]["x"], keypoints[i]["y"])
           for i in (_L_SHOULDER, _R_SHOULDER) if keypoints[i]["conf"] >= conf_thr]
    if pts:
        return {"cx": sum(p[0] for p in pts) / len(pts),
                "cy": sum(p[1] for p in pts) / len(pts), "visible": True}
    nose = keypoints[_NOSE]
    if nose["conf"] >= conf_thr:
        return {"cx": nose["x"], "cy": nose["y"], "visible": True}
    return {"cx": 0.5, "cy": 0.5, "visible": False}


class _DryMotor:
    """--dry-run용 모터 스텁 — move_to를 '즉시 도달'로 가정 (개발 PC 전용)."""

    def __init__(self) -> None:
        self._pan = 0.0
        self._tilt = 0.0

    def enable(self) -> None: ...
    def home(self) -> None: self._pan = self._tilt = 0.0
    def move_to(self, pan_deg: float, tilt_deg: float) -> None:
        self._pan, self._tilt = pan_deg, tilt_deg
    def current_position(self) -> tuple[float, float]: return (self._pan, self._tilt)
    def wait_until_idle(self, timeout: float | None = None) -> bool: return True
    def close(self) -> None: ...
    def __enter__(self) -> "_DryMotor": return self
    def __exit__(self, *exc) -> None: ...


def _open_motor(dry_run: bool):
    """실모터 또는 드라이런 스텁. lgpio 없는 개발 PC에서 --dry-run으로 뜰 수 있게
    MotorController import를 지연한다."""
    if dry_run:
        print("[motor] DRY-RUN — 실제 모터를 구동하지 않습니다 (각도 계산만).")
        return _DryMotor()
    from hardware.motor_controller import MotorController
    return MotorController(CFG)


def main() -> None:
    lim = CFG["limits"]["tilt"]
    p = argparse.ArgumentParser(description="카메라 기반 틸트 추적(닫힌 루프) 검증")
    p.add_argument("--model", default="multipose_lightning.tflite")
    p.add_argument("--conf", type=float, default=0.25, help="키포인트 신뢰도 임계값")
    p.add_argument("--threads", type=int, default=3, help="TFLite 스레드 수")
    # ── 제어 파라미터 (시작값은 임의 — 실기에서 튜닝) ─────────────────────────
    p.add_argument("--gain", type=float, default=0.25,
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

    prev_center: tuple[float, float] | None = None
    last_sent = 0.0        # 마지막으로 모터에 보낸 틸트 목표각 (데드존 기준)
    lost = True
    fps_hist: list[float] = []
    t_prev = time.time()
    last_log = time.time()

    print("\n[verify_track_tilt] 틸트 추적 시작 — 카메라가 팬·틸트 헤드에 함께 실린 상태를 가정합니다.")
    print(f"  gain={args.gain}  deadzone={args.deadzone}°  target_cy={args.target_cy}  "
          f"region={args.region}  tilt=[{args.tilt_min:g},{args.tilt_max:g}]°  invert={args.invert}")

    motor_cm = _open_motor(args.dry_run)
    try:
        with motor_cm as mc:
            mc.enable()
            mc.home()  # 현재 물리 위치 = 틸트 0°. 시작 위치에 마커를 붙여두면 대조에 편함
            print("[motor] 현재 위치를 틸트 0°로 설정 (임시 호밍).")
            try:
                while True:
                    t0 = time.time()
                    frame = _read_frame(cam, backend)
                    if frame is None:
                        if args.web and web_state:
                            web_state.update_stall(["no frames from the camera.",
                                                    "try --rpicam, or check camera wiring."])
                        time.sleep(0.03)
                        continue

                    people = detector.infer(frame)["people"]
                    target_idx = (
                        select_target([pp["keypoints"] for pp in people],
                                      conf_thr=args.conf, prev_center=prev_center)
                        if people else None
                    )

                    if target_idx is not None:
                        kps = people[target_idx]["keypoints"]
                        new_center = person_center(kps, conf_thr=args.conf)
                        if (prev_center is not None and new_center is not None
                                and ((new_center[0] - prev_center[0]) ** 2
                                     + (new_center[1] - prev_center[1]) ** 2) ** 0.5
                                > DEFAULT_MATCH_RADIUS):
                            tracker.reset()  # 대상 교체 → 스무딩 리셋
                        prev_center = new_center
                        if args.region == "chest":
                            regions = {"head": _INVISIBLE, "upper": chest_point(kps, args.conf),
                                       "lower": _INVISIBLE}
                        else:
                            regions = people[target_idx]["regions"]
                        tracker_input = {"detected": True, "regions": regions}
                    else:
                        tracker_input = {"detected": False,
                                         "regions": {k: _INVISIBLE for k in PoseTracker.REGIONS}}

                    aim = tracker.update(tracker_input)[aim_key]

                    # ── 피드백: 오차각 → 현재각에 누적 → 리밋 → 데드존 → 모터 ────
                    cur_tilt = mc.current_position()[1]
                    err_deg = None
                    at_limit = False
                    if aim["visible"]:
                        if lost:
                            print(f"\n[track] 목표 재획득 — 추적 재개 (idx={target_idx})")
                            lost = False
                        err_cy = aim["cy"] - (args.target_cy - 0.5)
                        err_deg = sign * compute_tilt_angle(err_cy, fov_v)
                        raw_target = clamp_angle(cur_tilt + args.gain * err_deg,
                                                 args.tilt_min, args.tilt_max)
                        at_limit = raw_target in (args.tilt_min, args.tilt_max)
                        gated = apply_deadzone(raw_target, last_sent, args.deadzone)
                        if gated != last_sent:
                            mc.move_to(0.0, gated)  # 틸트만 — 팬은 홈(0°)에 고정
                            last_sent = gated
                    else:
                        if not lost:
                            print("\n[track] 목표 상실 → 대기 (카메라에 사람이 잡히면 재개)")
                            lost = True

                    # ── FPS/수렴 로그 ────────────────────────────────────────
                    dt = time.time() - t_prev
                    t_prev = time.time()
                    fps_hist.append(1.0 / dt if dt > 0 else 0.0)
                    if len(fps_hist) > 30:
                        fps_hist.pop(0)
                    fps = sum(fps_hist) / len(fps_hist)

                    if time.time() - last_log >= 1.0:
                        err_s = f"{err_deg:+6.2f}" if err_deg is not None else "  --  "
                        print(f"\r[{time.strftime('%H:%M:%S')}] "
                              f"people={len(people)} target={target_idx if target_idx is not None else '-':<3} "
                              f"{'LOST ' if lost else 'LIMIT' if at_limit else 'TRACK'}  "
                              f"err={err_s}°  tilt_cur={cur_tilt:+6.2f}° tilt_cmd={last_sent:+6.2f}°  "
                              f"fps={fps:4.1f}  ", end="", flush=True)
                        last_log = time.time()

                    if not args.no_window or args.web:
                        vis = draw_pose(frame, people, target_idx)
                        h, w = vis.shape[:2]
                        ty = int(args.target_cy * h)
                        cv2.line(vis, (0, ty), (w, ty), (0, 210, 230), 1)  # 목표 y 가로선
                        if aim["visible"]:
                            cx, cy = int(aim["cx"] * w), int(aim["cy"] * h)
                            cv2.circle(vis, (cx, cy), 14, (0, 200, 60), 3)  # 조준점
                            cv2.line(vis, (cx, cy), (cx, ty), (0, 200, 60), 2)  # 오차 벡터
                        status = ("LOST (standby)" if lost
                                  else f"{'LIMIT ' if at_limit else ''}TRACK err={err_deg:+.1f}deg")
                        cv2.putText(vis, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 210, 230), 2)
                        cv2.putText(vis, f"tilt_cmd {last_sent:+.1f}deg  fps {fps:.1f}",
                                    (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 200, 0), 2)
                        if args.web and web_state:
                            web_state.update(vis)
                        if not args.no_window:
                            cv2.imshow("Target Fan | Tilt Tracking", vis)
                            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                                break
                    if args.no_window:
                        time.sleep(max(0.0, 0.02 - (time.time() - t0)))
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
