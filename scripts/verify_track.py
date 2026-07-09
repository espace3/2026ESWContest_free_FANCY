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

[게인을 1보다 낮게 두는 이유]
오차각은 기하학적으로 정확하고 이미 모터 단위(도)라, 이론상 정확한 게인은
1.0(deadbeat, 오차만큼 딱 돌리면 한 프레임에 중앙). 그런데도 1보다 낮추는 건
잡음 때문이 아니라: (1) current_position()은 큐 확정 위치라 이동 중 실제 로터보다
앞서므로 전량 반영 시 오버슈트, (2) 캡처+추론 지연으로 오차가 1~2프레임 과거값,
(3) 카메라가 회전축에서 떨어져 있으면 시차, (4) EMA/데드존이 이미 약간 지연.
→ 일반 서보 관례("아주 작게 시작")와 달리 여기선 0.6 근처에서 시작하고 진동하면
내리는 게 맞다. --gain으로 조정.

[⚠ 방향(부호) 미검증 — hardware/TODO.md]
compute_pan_angle은 목표가 중앙보다 오른쪽이면 +를 준다. 이를 현재각에 GAIN(+)로
더하는 건 "+각도 명령 → 카메라가 오른쪽을 더 보게 됨"을 전제한다. 이는
config stepper.pan.dir_for_positive 와 FOV 부호 매핑에 달려 있고 아직 실기
미검증이다(TODO.md). 부호가 반대면 음의 피드백이 양의 피드백이 되어 목표에서
멀어지는 쪽으로 폭주한다. 그래서:
  - 처음엔 --gain 을 낮게(예 0.3) 두고 확인할 것
  - --invert 로 부호를 즉석에서 뒤집어 볼 수 있게 해둠
  - --limit(기본 100°) 소프트 클램프로 폭주 시 물리 스토퍼 이전에 멈추게 함
    (이건 실제 회전 금지 구역이 아니라 부호 검증 단계의 안전 레일이다. 실제
     no-go 구역이 config에 확정되면 그쪽 clamp를 쓴다 — 요구사항상 지금은 생략)
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
import time
from pathlib import Path

import cv2

# 레포 루트 + scripts 디렉터리를 path에 추가 (config/vision/control + 카메라 백엔드 재사용)
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

from config import CFG
from vision.pose_estimator import MoveNetMultiPoseDetector
from vision.target_selector import select_target, person_center, DEFAULT_MATCH_RADIUS
from vision.pose_tracker import PoseTracker
from control.control_signal_generator import compute_pan_angle, apply_deadzone, clamp_angle
# 카메라 캡처 백엔드는 verify_movenet 것을 그대로 재사용한다 (중복 구현 방지).
# verify_movenet은 hardware(lgpio)를 import하지 않으므로 --dry-run 개발 PC에서도 안전.
from verify_movenet import (_open_camera, _read_frame, _release_camera, draw_pose,
                            _WebStreamState, _make_handler, _ThreadedHTTP)

# 어깨 키포인트 인덱스 (COCO): 5=l_shoulder, 6=r_shoulder, 0=nose
_L_SHOULDER, _R_SHOULDER, _NOSE = 5, 6, 0


def chest_point(keypoints: list[dict], conf_thr: float) -> dict:
    """선정된 사람의 '가슴' 조준점(정규화 cx/cy)을 어깨 중점으로 계산한다.

    가슴을 어깨 중점(5,6)으로 잡는 이유: 부위 모듈의 'upper'(어깨+엉덩이 중점)는
    실제로는 배/허리라 가슴이 아니다. 어깨 중점을 쓰면 조준점이 가슴 높이에
    오고, 그 위로 얼굴이 자연히 프레임에 들어와 추적 안정성에도 유리하다.

    어깨가 한쪽만 잡히면 그쪽만, 둘 다 없으면 코(머리)로 폴백한다 — 사람은
    보이는데 어깨만 잠깐 가린 경우까지 대상을 놓치지 않기 위해서다. 아무것도
    없으면 visible=False.
    """
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
    """--dry-run용 모터 스텁. lgpio/실모터 없이 개발 PC에서 오차→목표각 파이프라인만
    확인하기 위한 것. move_to를 '즉시 도달'로 가정해 current_position이 마지막
    목표각을 그대로 돌려준다 (실모터의 가감속/지연은 재현하지 않음 — 어디까지나
    좌표→각도 계산과 데드존/선정 로직을 PC에서 돌려보기 위한 용도)."""

    def __init__(self) -> None:
        self._pan = 0.0

    def enable(self) -> None: ...
    def home(self) -> None: self._pan = 0.0
    def move_to(self, pan_deg: float, tilt_deg: float) -> None: self._pan = pan_deg
    def current_position(self) -> tuple[float, float]: return (self._pan, 0.0)
    def close(self) -> None: ...
    def __enter__(self) -> "_DryMotor": return self
    def __exit__(self, *exc) -> None: ...


def _open_motor(dry_run: bool):
    """실모터(MotorController) 또는 드라이런 스텁을 연다. MotorController import를
    이 함수 안으로 미룬 이유: 그 모듈은 최상단에서 lgpio를 import하는데, 개발
    PC(lgpio 없음)에서 --dry-run으로 돌릴 때 최상단 import면 스크립트가 아예
    뜨질 못하기 때문이다."""
    if dry_run:
        print("[motor] DRY-RUN — 실제 모터를 구동하지 않습니다 (각도 계산만).")
        return _DryMotor()
    from hardware.motor_controller import MotorController
    return MotorController(CFG)


def main() -> None:
    p = argparse.ArgumentParser(description="카메라 기반 팬 추적(닫힌 루프) 검증")
    p.add_argument("--model", default="multipose_lightning.tflite")
    p.add_argument("--conf", type=float, default=0.25, help="키포인트 신뢰도 임계값")
    p.add_argument("--threads", type=int, default=3, help="TFLite 스레드 수")
    # ── 제어 파라미터 (실험이라 시작값은 임의 — 실기에서 튜닝) ────────────────
    p.add_argument("--gain", type=float, default=0.3,
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

    fov_h = CFG["fov"]["h"]
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

    # PoseTracker의 3부위 슬롯 중 'upper'에 우리가 계산한 가슴점을 실어 재사용한다
    # (miss 유예 + 재획득 즉시 점프 로직을 그대로 활용 — 요구사항 5의 "몇 프레임은
    #  마지막 위치로 버티다, 계속 안 보이면 상실" 동작이 이걸로 그냥 나온다).
    _invisible = {"cx": 0.5, "cy": 0.5, "visible": False}

    prev_center: tuple[float, float] | None = None  # 대상 교체 판정용 (verify_movenet 패턴)
    last_sent = 0.0                                  # 마지막으로 모터에 보낸 팬 목표각 (데드존 기준)
    lost = True                                      # 현재 목표 상실(대기) 상태인가
    fps_hist: list[float] = []
    t_prev = time.time()
    last_log = time.time()

    print("\n[verify_track] 팬 추적 시작 — 카메라를 팬 헤드에 얹은 상태를 가정합니다.")
    print(f"  gain={args.gain}  deadzone={args.deadzone}°  target_cx={args.target_cx}  "
          f"limit=±{args.limit}°  invert={args.invert}")
    print("  ⚠ 방향(부호) 미검증: 폭주하면 즉시 Ctrl+C, --invert 또는 --gain 낮춰 재시도\n")

    motor_cm = _open_motor(args.dry_run)
    try:
        with motor_cm as mc:
            mc.enable()
            mc.home()  # 현재 물리 위치 = 팬 0°. 시작 위치에 마커를 붙여두면 대조에 편함
            print("[motor] 현재 위치를 팬 0°로 설정 (임시 호밍).")

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

                # 다중 인원 중 1인 선정 (최대 bbox + 히스테리시스). 대상이 다른
                # 사람으로 교체되면 스무딩 리셋해 조준점이 허공을 미끄러지지 않게 함.
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
                        tracker.reset()
                    chest = chest_point(kps, args.conf)
                    regions = {"head": _invisible, "upper": chest, "lower": _invisible}
                    prev_center = new_center
                    tracker_input = {"detected": True, "regions": regions}
                else:
                    tracker_input = {"detected": False,
                                     "regions": {"head": _invisible, "upper": _invisible,
                                                 "lower": _invisible}}

                chest_s = tracker.update(tracker_input)["upper"]

                # ── 피드백: 오차각 → 현재각에 누적 → 데드존 → 모터 ────────────────
                cur_pan = mc.current_position()[0]
                err_deg = None
                if chest_s["visible"]:
                    # 목표 재획득 → 대기 상태 해제
                    if lost:
                        print(f"\n[track] 목표 재획득 — 추적 재개 (idx={target_idx})")
                        lost = False
                    # (cx − target_cx)로 오차를 잡아 목표점을 화면 중앙이 아닌
                    # target_cx로 맞출 수 있게 한다 (카메라/바람축 오프셋 흡수용).
                    err_cx = chest_s["cx"] - (args.target_cx - 0.5)
                    err_deg = sign * compute_pan_angle(err_cx, fov_h)
                    raw_target = cur_pan + args.gain * err_deg
                    raw_target = clamp_angle(raw_target, -args.limit, args.limit)
                    gated = apply_deadzone(raw_target, last_sent, args.deadzone)
                    if gated != last_sent:
                        mc.move_to(gated, 0.0)  # 팬만 — 틸트는 홈(0°)에 고정
                        last_sent = gated
                else:
                    # 요구사항 5: PoseTracker가 miss_thr 프레임 동안은 마지막 위치를
                    # 유지(visible=True)하며 그쪽으로 계속 조준하고, 그 뒤에도 안
                    # 보이면 여기로 떨어져 '목표 상실'을 1회 출력하고 대기한다.
                    # 대기 중엔 모터를 움직이지 않고 제자리를 지킨다. 다시 사람이
                    # 잡히면 위 분기에서 자동으로 추적을 재개한다.
                    if not lost:
                        print("\n[track] 목표 상실 → 대기 (카메라에 사람이 잡히면 재개)")
                        lost = True

                # ── FPS/수렴 로그 (README: 파이프라인 단계 추가 시 측정 코드 동봉) ──
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
                          f"{'LOST' if lost else 'TRACK'}  "
                          f"err={err_s}°  pan_cur={cur_pan:+7.2f}° pan_cmd={last_sent:+7.2f}°  "
                          f"fps={fps:4.1f}  ", end="", flush=True)
                    last_log = time.time()

                if not args.no_window or args.web:
                    vis = draw_pose(frame, people, target_idx)
                    h, w = vis.shape[:2]
                    tx = int(args.target_cx * w)
                    cv2.line(vis, (tx, 0), (tx, h), (0, 210, 230), 1)  # 목표 x 세로선
                    if chest_s["visible"]:
                        cx, cy = int(chest_s["cx"] * w), int(chest_s["cy"] * h)
                        cv2.circle(vis, (cx, cy), 14, (0, 200, 60), 3)  # 가슴 조준점
                        cv2.line(vis, (cx, cy), (tx, cy), (0, 200, 60), 2)  # 오차 벡터
                    status = "LOST (standby)" if lost else f"TRACK err={err_deg:+.1f}deg"
                    cv2.putText(vis, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 210, 230), 2)
                    cv2.putText(vis, f"pan_cmd {last_sent:+.1f}deg  fps {fps:.1f}",
                                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 200, 0), 2)
                    if args.web and web_state:
                        web_state.update(vis)
                    if not args.no_window:
                        cv2.imshow("Target Fan | Pan Tracking", vis)
                        if (cv2.waitKey(1) & 0xFF) == ord("q"):
                            break
                if args.no_window:
                    time.sleep(max(0.0, 0.02 - (time.time() - t0)))

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
