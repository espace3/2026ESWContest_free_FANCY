"""
app/tracking.py - 팬/틸트 닫힌 루프 공유 모듈

팬 단독 / 틸트 단독 / 두 축 동시를 한 함수(run_tracking)로 다룬다 — 축마다 따로
검증하던 시절 세 스크립트가 각자 갖고 있던 while-루프 본문과 헬퍼를 모은 것이라,
계산/제어 로직은 그때 실기로 확인한 것과 같다. 루프 조건만 `while True` 에서
`while not stop_event.is_set()` 으로 바뀌었다.

app/runners.py 의 _make_runner 가 BLE "타겟 모드" 명령에 맞춰 이 함수를 백그라운드
스레드로 시작/정지시킨다 (stop_event.set() 후 join).

여기 있는 것: run_tracking(닫힌 루프 본문), chest_point/_axes_idle/_draw_overlay
(부위 러너와 공용 헬퍼), _DryMotor/_DryRelay(--dry-run 스텁), 모터 핸들 열기와
상태파일 CLI 인자.
"""

from __future__ import annotations

import time

import cv2

from config import CFG

# 어깨 키포인트 인덱스 (COCO): 5=l_shoulder, 6=r_shoulder, 0=nose
_L_SHOULDER, _R_SHOULDER, _NOSE = 5, 6, 0

_INVISIBLE = {"cx": 0.5, "cy": 0.5, "visible": False}
_REGION_COLORS = {"head": (60, 180, 255), "upper": (0, 200, 60), "lower": (0, 120, 230)}


def chest_point(keypoints: list[dict], conf_thr: float) -> dict:
    """선정된 사람의 '가슴' 조준점(정규화 cx/cy) = 어깨 중점.

    paired 는 "cx 를 몸의 좌우 중심으로 믿어도 되는가"다. 한쪽 어깨만 잡히면
    cx 가 어깨폭의 절반만큼 그쪽으로 치우친다 — 1m·Wide(102°) 기준 약 8°라,
    사람이 가만히 있어도 팬 명령이 나가고 이동 감지 문턱(move_thr_deg 8°)까지
    넘겨 재조준 상태로 빠진다 (2026-09-02, 하체→상체 전환 때 팬이 움직이는
    증상). 어깨는 카메라가 아래를 볼 때 프레임 가장자리에서 한쪽만 살아남기
    쉬워서 드문 상황이 아니다.

    그래서 한쪽 어깨뿐일 때는 코가 보이면 cx 만 코에서 가져온다 — 코의 좌우
    편차는 머리폭 절반이라 어깨폭 절반의 절반 수준이다. 코도 없으면
    paired=False 로 알려 팬 피드백에서 빼게 한다.

    cy 는 두 어깨 높이가 거의 같으므로 한쪽만 잡혀도 그대로 쓴다 — paired 는
    팬(수평)에만 걸리는 플래그고 틸트는 영향받지 않는다.
    """
    sh = [(keypoints[i]["x"], keypoints[i]["y"])
          for i in (_L_SHOULDER, _R_SHOULDER) if keypoints[i]["conf"] >= conf_thr]
    nose = keypoints[_NOSE]
    has_nose = nose["conf"] >= conf_thr
    if len(sh) == 2:
        return {"cx": (sh[0][0] + sh[1][0]) / 2, "cy": (sh[0][1] + sh[1][1]) / 2,
                "visible": True, "paired": True}
    if len(sh) == 1:
        cx, cy = sh[0]
        if has_nose:
            return {"cx": nose["x"], "cy": cy, "visible": True, "paired": True}
        return {"cx": cx, "cy": cy, "visible": True, "paired": False}
    if has_nose:
        return {"cx": nose["x"], "cy": nose["y"], "visible": True, "paired": True}
    return {"cx": 0.5, "cy": 0.5, "visible": False, "paired": False}


def _axes_idle(mc) -> bool:
    """두 축 모두 목표 도달 + 큐 배출 상태인지 논블로킹 확인 (_DryMotor는 항상 True)."""
    if not hasattr(mc, "pan"):
        return True
    for ax in (mc.pan, mc.tilt):
        with ax.cond:
            if not (ax.idle and ax.target_steps == ax.pos_steps):
                return False
    return True


# ── 시각화 ───────────────────────────────────────────────────────────────────

def _draw_overlay(frame, people, target_idx, smoothed, fresh, scenario,
                  pan_cmd: float, tilt_cmd: float, fps: float,
                  target_cx: float = 0.5, target_cy: float = 0.5,
                  aim_bias_deg: float = 0.0, fov_v: float = 67.0):
    """부위 모드 화면. 조준 관련 표시가 셋이다.

      시안 십자 + 파선   조준점 (target_cx/cy). 이 자리에 부위를 놓는 것이 목표다.
      노란 짧은 선       조준 편향(aim_ratio x gap)까지 더한 최종 수렴 위치.
                        편향이 0이면 시안 십자와 겹친다.
      굵은 원            지금 겨누는 부위. 가는 원은 나머지 부위.
      초록 선            그 부위에서 최종 수렴 위치까지 = 남은 오차.
    """
    from app.camera import draw_pose   # 순환 import 회피 (run_tracking 과 같은 이유)
    vis = draw_pose(frame, people, target_idx)
    h, w = vis.shape[:2]
    tx, ty = int(target_cx * w), int(target_cy * h)
    # 조준점 — 시안 십자 + 전체 폭 파선
    cv2.drawMarker(vis, (tx, ty), (0, 210, 230), cv2.MARKER_CROSS, 24, 2)
    for x in range(0, w, 16):
        cv2.line(vis, (x, ty), (min(x + 8, w), ty), (0, 210, 230), 1)
    # 편향까지 더한 실제 수렴 위치 (부위가 여기 오면 멈춘다)
    ay = int((target_cy + aim_bias_deg / fov_v) * h)
    if abs(ay - ty) > 2:
        cv2.line(vis, (tx - 40, ay), (tx + 40, ay), (220, 200, 0), 2)
        cv2.putText(vis, f"aim{aim_bias_deg:+.1f}deg", (tx + 46, ay + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 200, 0), 1)
    active = scenario.active_region()
    for name, col in _REGION_COLORS.items():
        if fresh[name]:
            c = (int(smoothed[name]["cx"] * w), int(smoothed[name]["cy"] * h))
            if name == active:
                cv2.circle(vis, c, 14, col, 3)
                cv2.line(vis, c, (c[0], ay), (0, 200, 60), 2)   # 남은 오차
            else:
                cv2.circle(vis, c, 6, col, 1)
    cv2.putText(vis, f"{scenario.state.upper()} region={active}", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 210, 230), 2)
    cv2.putText(vis, f"pan {pan_cmd:+.1f}deg  tilt {tilt_cmd:+.1f}deg  fps {fps:.1f}",
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 200, 0), 2)
    return vis



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


class _DryRelay:
    """--dry-run용 릴레이 스텁. lgpio/실릴레이 없이 BLE→풍속 게이팅 로직만
    확인하기 위한 것. FanRelay처럼 같은 목표 반복은 무시한다(로그 소음 방지)."""

    def __init__(self) -> None:
        self._level = 0

    def set_speed(self, level: int) -> None:
        if level == self._level:
            return
        self._level = level
        print(f"[relay] DRY-RUN — set_speed({level}) (실제 릴레이 미구동)")

    def __enter__(self) -> "_DryRelay":
        return self

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


def run_tracking(cam, backend, detector, tracker, mc, args, stop_event, *,
                 axis, fov_h, fov_v, sign_pan, sign_tilt, aim_key="upper",
                 web_state=None) -> None:
    """카메라 기반 닫힌 루프 추적 — docs/tracking_feedback.md.

    axis 로 제어할 축을 고른다. 안 쓰는 축은 0° 로 고정한다.
        "pan"     팬만 (틸트 0° 고정)
        "tilt"    틸트만 (팬 0° 고정) — aim_key 로 겨눌 부위를 고를 수 있다
        "pantilt" 두 축 동시. 한 프레임 = 한 관측 = 한 명령으로 내보내
                  두 축이 서로 다른 순간의 관측으로 어긋나지 않게 한다.

    부위 선택(aim_key)이 틸트에서만 의미 있는 이유: 머리·상체·하체는 화면에서
    세로로만 갈리고 좌우로는 같은 자리라, 팬은 어느 부위를 겨눠도 같은 각도가
    나온다. 그래서 팬·팬틸트는 가슴(어깨 중점) 고정이다.
    """
    from vision.target_selector import select_target, person_center, DEFAULT_MATCH_RADIUS
    from vision.pose_tracker import PoseTracker
    from control.control_signal_generator import (apply_deadzone, clamp_angle,
                                                  compute_pan_angle, compute_tilt_angle)
    from app.camera import _read_frame, draw_pose

    use_pan = axis in ("pan", "pantilt")
    use_tilt = axis in ("tilt", "pantilt")
    title = {"pan": "Pan", "tilt": "Tilt", "pantilt": "Pan+Tilt"}[axis]

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
                                        "try --no-rpicam / --opencv, or check camera wiring."])
            time.sleep(0.03)
            continue

        people = detector.infer(frame)["people"]
        target_idx = (
            select_target(people, prev_center=prev_center)
            if people else None
        )

        if target_idx is not None:
            kps = people[target_idx]["keypoints"]
            new_center = person_center(people[target_idx])
            if (prev_center is not None and new_center is not None
                    and ((new_center[0] - prev_center[0]) ** 2
                         + (new_center[1] - prev_center[1]) ** 2) ** 0.5
                    > DEFAULT_MATCH_RADIUS):
                tracker.reset()   # 대상 교체 → 스무딩 리셋 (허공 미끄러짐 방지)
            prev_center = new_center
            if aim_key == "upper":
                regions = {"head": _INVISIBLE, "upper": chest_point(kps, args.conf),
                           "lower": _INVISIBLE}
            else:
                regions = people[target_idx]["regions"]
            tracker_input = {"detected": True, "regions": regions}
        else:
            tracker_input = {"detected": False,
                             "regions": {k: _INVISIBLE for k in PoseTracker.REGIONS}}

        aim = tracker.update(tracker_input)[aim_key]

        cur_pan, cur_tilt = mc.current_position()
        ep = et = None
        at_limit = False
        if aim["visible"]:
            if lost:
                print(f"\n[track] 목표 재획득 — 추적 재개 (idx={target_idx})")
                lost = False
            pan_t, tilt_t = last_pan, last_tilt
            if use_pan:
                ep = sign_pan * compute_pan_angle(aim["cx"] - (args.target_cx - 0.5), fov_h)
                pan_t = clamp_angle(cur_pan + args.gain_pan * ep,
                                    args.pan_min, args.pan_max)
            if use_tilt:
                et = sign_tilt * compute_tilt_angle(aim["cy"] - (args.target_cy - 0.5), fov_v)
                tilt_t = clamp_angle(cur_tilt + args.gain_tilt * et,
                                     args.tilt_min, args.tilt_max)
                at_limit = tilt_t in (args.tilt_min, args.tilt_max)
            pan_g = apply_deadzone(pan_t, last_pan, args.deadzone_pan) if use_pan else 0.0
            tilt_g = apply_deadzone(tilt_t, last_tilt, args.deadzone_tilt) if use_tilt else 0.0
            if (pan_g, tilt_g) != (last_pan, last_tilt):
                mc.move_to(pan_g, tilt_g)
                last_pan, last_tilt = pan_g, tilt_g
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
            es = "/".join(f"{e:+5.1f}" if e is not None else "  -- "
                          for e in ((ep,) if not use_tilt else (et,) if not use_pan else (ep, et)))
            pos = []
            if use_pan:
                pos.append(f"pan={cur_pan:+7.2f}°>{last_pan:+7.2f}°")
            if use_tilt:
                pos.append(f"tilt={cur_tilt:+6.2f}°>{last_tilt:+6.2f}°")
            print(f"\r[{time.strftime('%H:%M:%S')}] "
                  f"people={len(people)} target={target_idx if target_idx is not None else '-':<3} "
                  f"{'LOST ' if lost else 'LIMIT' if at_limit else 'TRACK'}  "
                  f"err={es}°  {' '.join(pos)}  "
                  f"fps={fps:4.1f}  ", end="", flush=True)
            last_log = time.time()

        if not args.no_window or args.web:
            vis = draw_pose(frame, people, target_idx)
            h, w = vis.shape[:2]
            tx, ty = int(args.target_cx * w), int(args.target_cy * h)
            # 조준 기준선 — 제어하는 축 방향으로만 긋는다.
            if use_pan and use_tilt:
                cv2.drawMarker(vis, (tx, ty), (0, 210, 230), cv2.MARKER_CROSS, 24, 2)
                # 십자만으로는 조준점이 화면 중앙에서 얼마나 벗어났는지 안 보인다.
                for x in range(0, w, 16):
                    cv2.line(vis, (x, ty), (min(x + 8, w), ty), (0, 210, 230), 1)
                for y in range(0, h, 16):
                    cv2.line(vis, (tx, y), (tx, min(y + 8, h)), (0, 210, 230), 1)
            elif use_pan:
                cv2.line(vis, (tx, 0), (tx, h), (0, 210, 230), 1)
            else:
                cv2.line(vis, (0, ty), (w, ty), (0, 210, 230), 1)
            if aim["visible"]:
                cx, cy = int(aim["cx"] * w), int(aim["cy"] * h)
                cv2.circle(vis, (cx, cy), 14, (0, 200, 60), 3)
                end = (tx if use_pan else cx, ty if use_tilt else cy)
                cv2.line(vis, (cx, cy), end, (0, 200, 60), 2)
            if lost:
                status = "LOST (standby)"
            else:
                errs = ",".join(f"{e:+.1f}" for e in (ep, et) if e is not None)
                status = f"{'LIMIT ' if at_limit else ''}TRACK err={errs}deg"
            cv2.putText(vis, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 210, 230), 2)
            cmd = "  ".join(([f"pan {last_pan:+.1f}"] if use_pan else [])
                            + ([f"tilt {last_tilt:+.1f}"] if use_tilt else []))
            cv2.putText(vis, f"{cmd}  fps {fps:.1f}",
                        (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 200, 0), 2)
            if args.web and web_state:
                web_state.update(vis)
            if not args.no_window:
                cv2.imshow(f"Target Fan | {title} Tracking", vis)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
        if args.no_window:
            time.sleep(max(0.0, 0.02 - (time.time() - t0)))
