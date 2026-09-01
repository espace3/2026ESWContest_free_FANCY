"""
app/tracking.py - 팬/틸트 닫힌 루프 공유 모듈
(개발 이력상 tracking_core.py)

팬 단독 / 틸트 단독 / 두 축 동시, 세 개의 축별 검증 스크립트가 각각 독립적으로
갖고 있던 while-루프 본문과 헬퍼(chest_point/_DryMotor/_open_motor)를 모아둔 것이다. 계산/제어 로직은 원본에서 한 글자도 바꾸지 않았고, `while True`만
`while not stop_event.is_set()`으로 바뀌었다 — 각 verify_track_*.py는 이 함수들을
호출하도록 리팩터되어 있으며 `stop_event`를 아무도 set하지 않으므로 단독 실행 시
동작은 기존과 100% 동일하다.

app/ble_protocol.py는 BLE "타겟 모드" 명령에 맞춰 이 함수 중 하나를 백그라운드
스레드로 시작/정지시키는 용도로 재사용한다 (stop_event.set() 후 join).
"""

from __future__ import annotations

import time

import cv2

from config import CFG

# 어깨 키포인트 인덱스 (COCO): 5=l_shoulder, 6=r_shoulder, 0=nose
_L_SHOULDER, _R_SHOULDER, _NOSE = 5, 6, 0

_INVISIBLE = {"cx": 0.5, "cy": 0.5, "visible": False}


def chest_point(keypoints: list[dict], conf_thr: float) -> dict:
    """선정된 사람의 '가슴' 조준점(정규화 cx/cy) = 어깨 중점. 한쪽 어깨만 잡히면
    그쪽, 둘 다 없으면 코 폴백, 그것도 없으면 visible=False."""
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
    목표각을 그대로 돌려준다 (실모터의 가감속/지연은 재현하지 않음).

    위치 상태 파일은 건드리지 않는다 — 실제로 움직인 게 없는데 장부를 덮어쓰면
    다음 실기 실행이 잘못된 원점으로 복원하기 때문이다."""

    def __init__(self) -> None:
        self._pan = 0.0
        self._tilt = 0.0

    def enable(self) -> None: ...
    def home(self) -> None: self._pan = self._tilt = 0.0
    def restore_origin(self, timeout: float = 120.0) -> bool:
        self._pan = self._tilt = 0.0
        return True
    def move_to(self, pan_deg: float, tilt_deg: float) -> None:
        self._pan, self._tilt = pan_deg, tilt_deg
    def stop(self) -> None:
        # 실모터는 감속 정지 후 멈춘 자리가 새 위치가 되지만, 이 스텁은 move_to를
        # '즉시 도달'로 가정하므로 이미 목표각에 있다 — 바꿀 상태가 없다.
        ...
    def current_position(self) -> tuple[float, float]: return (self._pan, self._tilt)
    def wait_until_idle(self, timeout: float | None = None) -> bool: return True
    def close(self) -> None: ...
    def __enter__(self) -> "_DryMotor": return self
    def __exit__(self, *exc) -> None: ...


def _open_motor(dry_run: bool, state_path=None, *, timing: bool = False,
                pin_pulse_core: bool = True, rt: bool = True):
    """실모터(MotorController) 또는 드라이런 스텁을 연다. MotorController import를
    이 함수 안으로 미룬 이유: 그 모듈은 최상단에서 lgpio를 import하는데, 개발
    PC(lgpio 없음)에서 --dry-run으로 돌릴 때 최상단 import면 스크립트가 아예
    뜨질 못하기 때문이다.

    state_path=None이면 config "motor_state".file을 쓴다 (모든 스크립트 공용)."""
    if dry_run:
        print("[motor] DRY-RUN — 실제 모터를 구동하지 않습니다 (각도 계산만).")
        return _DryMotor()
    from hardware.motor_controller import MotorController
    return MotorController(CFG, state_path=state_path, timing=timing,
                           pin_pulse_core=pin_pulse_core, rt=rt)


def add_state_args(parser) -> None:
    """위치 상태 파일 + 펄스 타이밍 인자 (모터를 구동하는 스크립트 공통)."""
    ms = CFG.get("motor_state", {})
    parser.add_argument("--state-file", default=ms.get("file", "motor_state.json"),
                        help="모터 위치 상태 파일 (재시작 시 원점 복원용, 모든 스크립트 공용)")
    # 아래 셋은 추적 중 달그락 소음(lgpio 펄스 스레드 선점) 관련 — docs/lgpio_patch.md
    parser.add_argument("--timing", action="store_true",
                        help="이동 구간마다 펄스 타이밍 실측 출력 (드리프트·언더런)")
    parser.add_argument("--no-pin", action="store_true",
                        help="펄스 스레드 전용 코어 격리를 끈다 (효과 비교용)")
    parser.add_argument("--no-rt", action="store_true",
                        help="펄스 스레드 RT 승격을 끈다 (효과 비교용)")


def open_motor_from_args(args):
    """add_state_args로 받은 인자대로 모터를 연다."""
    return _open_motor(args.dry_run, state_path=getattr(args, "state_file", None),
                       timing=getattr(args, "timing", False),
                       pin_pulse_core=not getattr(args, "no_pin", False),
                       rt=not getattr(args, "no_rt", False))


def run_pan_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                      fov_h, sign, web_state=None) -> None:
    """팬 닫힌 루프 (팬만 제어, 틸트는 0° 고정) — docs/tracking_feedback.md."""
    from vision.target_selector import select_target, person_center, DEFAULT_MATCH_RADIUS
    from control.control_signal_generator import compute_pan_angle, apply_deadzone, clamp_angle
    from app.camera import _read_frame, draw_pose

    prev_center: tuple[float, float] | None = None
    last_sent = 0.0
    lost = True
    fps_hist: list[float] = []
    t_prev = time.time()
    last_log = time.time()

    while not stop_event.is_set():
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
                tracker.reset()
            chest = chest_point(kps, args.conf)
            regions = {"head": _INVISIBLE, "upper": chest, "lower": _INVISIBLE}
            prev_center = new_center
            tracker_input = {"detected": True, "regions": regions}
        else:
            tracker_input = {"detected": False,
                             "regions": {"head": _INVISIBLE, "upper": _INVISIBLE,
                                         "lower": _INVISIBLE}}

        chest_s = tracker.update(tracker_input)["upper"]

        cur_pan = mc.current_position()[0]
        err_deg = None
        if chest_s["visible"]:
            if lost:
                print(f"\n[track] 목표 재획득 — 추적 재개 (idx={target_idx})")
                lost = False
            err_cx = chest_s["cx"] - (args.target_cx - 0.5)
            err_deg = sign * compute_pan_angle(err_cx, fov_h)
            raw_target = cur_pan + args.gain * err_deg
            raw_target = clamp_angle(raw_target, -args.limit, args.limit)
            gated = apply_deadzone(raw_target, last_sent, args.deadzone)
            if gated != last_sent:
                mc.move_to(gated, 0.0)
                last_sent = gated
        else:
            if not lost:
                print("\n[track] 목표 상실 → 대기 (카메라에 사람이 잡히면 재개)")
                lost = True

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
            cv2.line(vis, (tx, 0), (tx, h), (0, 210, 230), 1)
            if chest_s["visible"]:
                cx, cy = int(chest_s["cx"] * w), int(chest_s["cy"] * h)
                cv2.circle(vis, (cx, cy), 14, (0, 200, 60), 3)
                cv2.line(vis, (cx, cy), (tx, cy), (0, 200, 60), 2)
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


def run_tilt_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                       fov_v, sign, aim_key, web_state=None) -> None:
    """틸트 닫힌 루프 (틸트만 제어, 팬은 0° 고정) — docs/tracking_feedback.md."""
    from vision.target_selector import select_target, person_center, DEFAULT_MATCH_RADIUS
    from vision.pose_tracker import PoseTracker
    from control.control_signal_generator import apply_deadzone, clamp_angle, compute_tilt_angle
    from app.camera import _read_frame, draw_pose

    prev_center: tuple[float, float] | None = None
    last_sent = 0.0
    lost = True
    fps_hist: list[float] = []
    t_prev = time.time()
    last_log = time.time()

    while not stop_event.is_set():
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
                tracker.reset()
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
                mc.move_to(0.0, gated)
                last_sent = gated
        else:
            if not lost:
                print("\n[track] 목표 상실 → 대기 (카메라에 사람이 잡히면 재개)")
                lost = True

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
            cv2.line(vis, (0, ty), (w, ty), (0, 210, 230), 1)
            if aim["visible"]:
                cx, cy = int(aim["cx"] * w), int(aim["cy"] * h)
                cv2.circle(vis, (cx, cy), 14, (0, 200, 60), 3)
                cv2.line(vis, (cx, cy), (cx, ty), (0, 200, 60), 2)
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


def run_pantilt_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                          fov_h, fov_v, sign_pan, sign_tilt, web_state=None) -> None:
    """팬+틸트 동시 닫힌 루프 (한 관측 = 한 명령) — docs/tracking_feedback.md."""
    from vision.target_selector import select_target, person_center, DEFAULT_MATCH_RADIUS
    from vision.pose_tracker import PoseTracker
    from control.control_signal_generator import (apply_deadzone, clamp_angle,
                                                  compute_pan_angle, compute_tilt_angle)
    from app.camera import _read_frame, draw_pose

    prev_center: tuple[float, float] | None = None
    last_pan = last_tilt = 0.0
    lost = True
    fps_hist: list[float] = []
    t_prev = time.time()
    last_log = time.time()

    while not stop_event.is_set():
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
                tracker.reset()
            prev_center = new_center
            regions = {"head": _INVISIBLE, "upper": chest_point(kps, args.conf),
                      "lower": _INVISIBLE}
            tracker_input = {"detected": True, "regions": regions}
        else:
            tracker_input = {"detected": False,
                             "regions": {k: _INVISIBLE for k in PoseTracker.REGIONS}}

        chest = tracker.update(tracker_input)["upper"]

        cur_pan, cur_tilt = mc.current_position()
        ep = et = None
        if chest["visible"]:
            if lost:
                print(f"\n[track] 목표 재획득 — 추적 재개 (idx={target_idx})")
                lost = False
            ep = sign_pan * compute_pan_angle(chest["cx"] - (args.target_cx - 0.5), fov_h)
            et = sign_tilt * compute_tilt_angle(chest["cy"] - (args.target_cy - 0.5), fov_v)
            pan_t = clamp_angle(cur_pan + args.gain * ep, args.pan_min, args.pan_max)
            tilt_t = clamp_angle(cur_tilt + args.gain_tilt * et, args.tilt_min, args.tilt_max)
            pan_g = apply_deadzone(pan_t, last_pan, args.deadzone)
            tilt_g = apply_deadzone(tilt_t, last_tilt, args.deadzone_tilt)
            if (pan_g, tilt_g) != (last_pan, last_tilt):
                mc.move_to(pan_g, tilt_g)
                last_pan, last_tilt = pan_g, tilt_g
        else:
            if not lost:
                print("\n[track] 목표 상실 → 대기 (두 축 정지, 사람이 잡히면 재개)")
                lost = True

        dt = time.time() - t_prev
        t_prev = time.time()
        fps_hist.append(1.0 / dt if dt > 0 else 0.0)
        if len(fps_hist) > 30:
            fps_hist.pop(0)
        fps = sum(fps_hist) / len(fps_hist)

        if time.time() - last_log >= 1.0:
            es = (f"{ep:+5.1f}/{et:+5.1f}" if ep is not None else "  --/--  ")
            print(f"\r[{time.strftime('%H:%M:%S')}] "
                  f"people={len(people)} target={target_idx if target_idx is not None else '-':<3} "
                  f"{'LOST ' if lost else 'TRACK'}  err(p/t)={es}°  "
                  f"pan={cur_pan:+7.2f}°>{last_pan:+7.2f}° tilt={cur_tilt:+6.2f}°>{last_tilt:+6.2f}°  "
                  f"fps={fps:4.1f}  ", end="", flush=True)
            last_log = time.time()

        if not args.no_window or args.web:
            vis = draw_pose(frame, people, target_idx)
            h, w = vis.shape[:2]
            tx, ty = int(args.target_cx * w), int(args.target_cy * h)
            cv2.drawMarker(vis, (tx, ty), (0, 210, 230), cv2.MARKER_CROSS, 22, 1)
            if chest["visible"]:
                cx, cy = int(chest["cx"] * w), int(chest["cy"] * h)
                cv2.circle(vis, (cx, cy), 14, (0, 200, 60), 3)
                cv2.line(vis, (cx, cy), (tx, ty), (0, 200, 60), 2)
            status = ("LOST (standby)" if lost
                      else f"TRACK err={ep:+.1f},{et:+.1f}deg")
            cv2.putText(vis, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 210, 230), 2)
            cv2.putText(vis, f"pan {last_pan:+.1f}  tilt {last_tilt:+.1f}  fps {fps:.1f}",
                        (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 200, 0), 2)
            if args.web and web_state:
                web_state.update(vis)
            if not args.no_window:
                cv2.imshow("Target Fan | Pan+Tilt Tracking", vis)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
        if args.no_window:
            time.sleep(max(0.0, 0.02 - (time.time() - t0)))
