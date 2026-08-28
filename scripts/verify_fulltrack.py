"""
scripts/verify_fulltrack.py - 팬+틸트 전신(머리→발) 추적 4단계 검증 스크립트

verify_track_pan.py/verify_track_tilt.py(축 단독)의 다음 단계. 카메라가 팬·틸트 헤드에 함께 실려 있다는
전제로 전신 시나리오를 검증한다. 이 파일도 계산 로직을 담지 않는다:
  - 시나리오 상태기계(스캔→순찰→재획득):  control/fullbody_scenario.py
  - 추정/선정/스무딩:  vision/pose_estimator, target_selector, pose_tracker
  - 좌표→각도/데드존/리밋:  control/control_signal_generator (+ config "limits")
  - 실제 모터 구동:  hardware/motor_controller
여기서는 이들을 엮어 매 프레임 돌리고, 상태 전이·수렴을 로그로 남기고, 원점
복귀/복원(요구 5)을 처리한다. 카메라 캡처는 verify_movenet 백엔드를 재사용한다.

────────────────────────────────────────────────────────────────────────────
⚠ 알려진 미해결 문제 (편집·확장 전 반드시 고려할 것 — 2026-07-10 합성 검증)

이 시나리오는 "적절한 마운트 높이(~0.8m)에서 ~1.5m 거리의 서 있는 사람"에는
잘 동작하지만(head/upper/lower 모두 수렴), 아래 조건에서 무너진다:

1. [하체 추정 붕괴] _est_lower_tilt()는 upper+(upper-head)×lower_ratio로 하체를
   추정하는데, head가 틸트 상한(config limits.tilt)에 눌려 진짜 위치에 못 가면
   head↔upper 간격이 가짜가 되어 추정이 upper 옆으로 붕괴한다. 게다가 스캔은
   하체를 "블라인드 스윕" 대신 이 추정 지점에 세워두고 기다려서, 실제 하체가
   화면에 잡히는데도 못 찾고 틀린 값을 기록한다(근접·저마운트에서 20°+ 오차).
   → 추정은 블라인드 스윕이 실패한 뒤의 폴백이어야지 스윕을 대체하면 안 된다.
2. [블라인드 스윕 범위 부족] 블라인드 램프 = gain_tilt×blind_deg (기본 0.5°/프레임).
   occl_frames(기본 50) 동안 25°만 스윕하는데 틸트 범위는 45°(-30~+15)라 전
   범위를 못 덮는다. 부위가 화면에 들어오면 피드백이 이어받아 대개 구제되지만,
   시작각에서 먼 부위는 발견 전에 타임아웃될 수 있다.
3. [저마운트/근접 커버리지] 카메라가 낮거나 사람이 가까우면 머리·상체가 틸트
   상한을 초과해 스캔에서 통째로 빠진다. limits.tilt{-30,+15}는 ~0.8m 마운트
   기준이다. 이게 실제 기구 한계인지 소프트값인지에 따라 대응이 갈린다.
4. [틸트 속도] 실측 순항 ≈11°/s (92.6:1 웜기어). head(-30)↔lower(+15) 45° 스캔이
   편도 ~4s. recenter/search에서 어깨 뷰로 큰 틸트 이동 중엔 추적 공백이 생긴다.
5. [오픈루프 탈조·거리 변화] 스텝 탈조나 사용자 전후 이동은 팬 오차를 안 만들어
   재스캔을 못 트리거하는데, 원근이 변하면 틸트 웨이포인트 전부가 무효화된다.
   estimate_distance_m()로 거리 변화를 감시해 재스캔 트리거를 붙이는 것이 후보.
6. [하체 조준 중 이동 사각] 수직 FOV 41°라 발을 조준하는 dwell 동안 어깨가 화면
   밖 → 그 시간만큼 이동 감지 공백. dwell 단축 또는 광각 렌즈 검토.
7. [대상 재식별 불가] MoveNet은 사람 ID가 없어 search 후 재획득한 사람이 원래
   사용자라는 보장이 없다(1인 환경 전제).
────────────────────────────────────────────────────────────────────────────

원점 복귀·복원 (요구 5):
  - 장부 저장/복원은 MotorController가 담당한다 (hardware/position_store.py).
    실행 중 장부가 바뀔 때마다 상태 파일에 기록되고, 시작 시 restore_origin()이
    기록된 만큼 되돌아와 0°에서 출발한다. 종료 시에도 (0°,0°)로 복귀한다.
  - 영점 자체를 새로 잡으려면 scripts/set_origin.py를 한 번 실행한다.
  - 두 축 모두 기어 자기잠금이라 전원이 꺼져도 위치가 유지된다 → 복원 신뢰 가능.
    남는 오차원은 탈조뿐이다 (hardware/TODO.md 호밍 항목).

실기 검증 절차 (RPi 5, 레포 루트에서):
    # 0) 틸트 방향/속도는 verify_track_tilt.py로 먼저 확정할 것 (--invert 필요 여부)
    python scripts/verify_fulltrack.py                        # 1) 스캔 수렴 → 순찰
    python scripts/verify_fulltrack.py --web --no-window      # SSH: 브라우저 확인
    python scripts/verify_fulltrack.py --dry-run --opencv     # 개발 PC, 모터 없이
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
from control.control_signal_generator import apply_deadzone
from control.fullbody_scenario import FullBodyScenario
# 모터 열기/원점 복원 헬퍼는 tracking_core 공용 (verify_track_*와 같은 상태 파일 사용)
from tracking_core import add_state_args, open_motor_from_args
from verify_movenet import (_open_camera, _read_frame, _release_camera, draw_pose,
                            _WebStreamState, _make_handler, _ThreadedHTTP)

# 어깨 키포인트 인덱스 (COCO): 5=l_shoulder, 6=r_shoulder, 0=nose
_L_SHOULDER, _R_SHOULDER, _NOSE = 5, 6, 0

_INVISIBLE = {"cx": 0.5, "cy": 0.5, "visible": False}

_REGION_COLORS = {"head": (60, 180, 255), "upper": (0, 200, 60), "lower": (0, 120, 230)}


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


def _axes_idle(mc) -> bool:
    """두 축 모두 목표 도달 + 큐 배출 상태인지 논블로킹 확인 (_DryMotor는 항상 True)."""
    if not hasattr(mc, "pan"):
        return True
    for ax in (mc.pan, mc.tilt):
        with ax.cond:
            if not (ax.idle and ax.target_steps == ax.pos_steps):
                return False
    return True


# ── 원점 복귀 (요구 5) ────────────────────────────────────────────────────────
# 장부 저장/복원 자체는 MotorController + hardware/position_store.py가 담당한다
# (예전에는 이 파일에만 있었지만, 같은 헤드를 다른 스크립트로 돌리면 낡은 값이
# 남아 엉뚱하게 복원되는 문제가 있어 공용 모듈로 올렸다). 여기 남은 건 종료 시
# 물리적으로 원점까지 되돌리는 절차뿐이다 — 최종 장부 기록은 close()가 한다.

def _return_home(mc) -> None:
    """종료 시 (0°,0°) 복귀. 완료하지 못하면 다음 실행이 장부로 복원을 시도한다."""
    print("\n[fulltrack] 원점(0°,0°) 복귀 중...")
    done = False
    try:
        mc.move_to(0.0, 0.0)
        done = mc.wait_until_idle(timeout=60)
    except Exception as e:
        print(f"[fulltrack] 복귀 이동 실패: {e}")
    pan, tilt = mc.current_position()
    print("[fulltrack] 원점 복귀 완료" if done else
          f"[fulltrack] 복귀 미완료 (pan={pan:+.2f}° tilt={tilt:+.2f}°) — 다음 실행에서 복원 시도")


# ── 시각화 ───────────────────────────────────────────────────────────────────

def _draw_overlay(frame, people, target_idx, smoothed, fresh, scenario,
                  pan_cmd: float, tilt_cmd: float, fps: float):
    vis = draw_pose(frame, people, target_idx)
    h, w = vis.shape[:2]
    cv2.drawMarker(vis, (w // 2, h // 2), (0, 210, 230), cv2.MARKER_CROSS, 20, 1)
    active = scenario.active_region()
    for name, col in _REGION_COLORS.items():
        if fresh[name]:
            c = (int(smoothed[name]["cx"] * w), int(smoothed[name]["cy"] * h))
            if name == active:
                cv2.circle(vis, c, 14, col, 3)
            else:
                cv2.circle(vis, c, 6, col, 1)
    cv2.putText(vis, f"{scenario.state.upper()} region={active}", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 210, 230), 2)
    cv2.putText(vis, f"pan {pan_cmd:+.1f}deg  tilt {tilt_cmd:+.1f}deg  fps {fps:.1f}",
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 200, 0), 2)
    return vis


# ── 메인 루프 ────────────────────────────────────────────────────────────────

def _track_loop(args, detector, cam, backend, tracker, scenario, mc,
                web_state) -> None:
    prev_center: tuple[float, float] | None = None
    last_pan = last_tilt = 0.0
    fps_hist: list[float] = []
    t_prev = time.time()
    last_log = time.time()

    while True:
        t0 = time.time()
        frame = _read_frame(cam, backend)
        if frame is None:
            if web_state:
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
            chest = chest_point(kps, args.conf)
            tracker_input = {"detected": True, "regions": people[target_idx]["regions"]}
        else:
            chest = dict(_INVISIBLE)
            tracker_input = {"detected": False,
                             "regions": {k: _INVISIBLE for k in PoseTracker.REGIONS}}

        smoothed = tracker.update(tracker_input)
        fresh = {k: tracker.miss[k] == 0 for k in PoseTracker.REGIONS}

        pan_t, tilt_t = scenario.step({
            "t": t0,
            "pos": mc.current_position(),
            "idle": _axes_idle(mc),
            "target_idx": target_idx,
            "chest": chest,
            "regions": smoothed,
            "fresh": fresh,
        })
        for ev in scenario.events:
            print(f"\n[track] {ev}")

        pan_g = apply_deadzone(pan_t, last_pan, args.deadzone)
        tilt_g = apply_deadzone(tilt_t, last_tilt, args.deadzone_tilt)
        if (pan_g, tilt_g) != (last_pan, last_tilt):
            mc.move_to(pan_g, tilt_g)
            last_pan, last_tilt = pan_g, tilt_g

        # ── FPS/상태 로그 (README: 파이프라인 단계 추가 시 측정 코드 동봉) ──
        dt = time.time() - t_prev
        t_prev = time.time()
        fps_hist.append(1.0 / dt if dt > 0 else 0.0)
        if len(fps_hist) > 30:
            fps_hist.pop(0)
        fps = sum(fps_hist) / len(fps_hist)

        if time.time() - last_log >= 1.0:
            cur_pan, cur_tilt = mc.current_position()
            wp_s = " ".join(f"{k[0].upper()}{'~' if v['estimated'] else ''}{v['tilt']:+.1f}"
                            for k, v in scenario.waypoints.items())
            print(f"\r[{time.strftime('%H:%M:%S')}] {scenario.state:<8} "
                  f"rg={scenario.active_region():<5} pan={cur_pan:+7.2f}° "
                  f"tilt={cur_tilt:+6.2f}° wp[{wp_s}] people={len(people)} "
                  f"fps={fps:4.1f}  ", end="", flush=True)
            last_log = time.time()

        if not args.no_window or args.web:
            vis = _draw_overlay(frame, people, target_idx, smoothed, fresh,
                                scenario, last_pan, last_tilt, fps)
            if web_state:
                web_state.update(vis)
            if not args.no_window:
                cv2.imshow("Target Fan | Full-body Tracking", vis)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
        if args.no_window:
            time.sleep(max(0.0, 0.02 - (time.time() - t0)))


def main() -> None:
    lim = CFG["limits"]
    p = argparse.ArgumentParser(description="팬+틸트 전신(머리→발) 추적 시나리오 검증")
    p.add_argument("--model", default="multipose_lightning.tflite")
    p.add_argument("--conf", type=float, default=0.25, help="키포인트 신뢰도 임계값")
    p.add_argument("--threads", type=int, default=3, help="TFLite 스레드 수")
    # ── 제어 (시작값은 임의 — 실기 튜닝) ─────────────────────────────────────
    p.add_argument("--gain", type=float, default=0.3, help="팬 비례 게인")
    p.add_argument("--gain-tilt", type=float, default=0.25, help="틸트 비례 게인")
    p.add_argument("--deadzone", type=float, default=1.0, help="팬 데드존 (°)")
    p.add_argument("--deadzone-tilt", type=float, default=0.5, help="틸트 데드존 (°)")
    p.add_argument("--pan-min", type=float, default=lim["pan"]["min"])
    p.add_argument("--pan-max", type=float, default=lim["pan"]["max"])
    p.add_argument("--tilt-min", type=float, default=lim["tilt"]["min"],
                   help="틸트 소프트 리밋 하한 ° (기본 config limits — 데드라인 실측 전 임시값)")
    p.add_argument("--tilt-max", type=float, default=lim["tilt"]["max"])
    p.add_argument("--invert", action="store_true", help="팬 오차 부호 반전")
    p.add_argument("--invert-tilt", action="store_true",
                   help="틸트 오차 부호 반전 (dir_for_positive 미검증 — 첫 실기에서 확인)")
    # ── 시나리오 (control/fullbody_scenario.py 파라미터) ─────────────────────
    p.add_argument("--converge", type=float, default=1.5, help="수렴 판정 오차 (°)")
    p.add_argument("--converge-frames", type=int, default=4, help="수렴 판정 연속 프레임")
    p.add_argument("--dwell", type=float, default=2.0, help="순찰 부위별 체류 시간 (s)")
    p.add_argument("--trim-alpha", type=float, default=0.2, help="순찰 중 웨이포인트 EMA 갱신율")
    p.add_argument("--move-thr", type=float, default=8.0, help="이동 감지 가슴 수평 오차 임계 (°)")
    p.add_argument("--rescan-thr", type=float, default=12.0, help="전신 재스캔 판정 이동량 (°)")
    p.add_argument("--lost-frames", type=int, default=15, help="대상 상실 판정 연속 프레임")
    p.add_argument("--occl-frames", type=int, default=50, help="부위 가림 판정 연속 프레임")
    p.add_argument("--blind-deg", type=float, default=2.0,
                   help="미관측 부위 탐색 이동량 (화면각 °/프레임)")
    p.add_argument("--lower-ratio", type=float, default=1.1,
                   help="하체 틸트 추정 비율 (머리↔상체 간격 대비)")
    p.add_argument("--search-step", type=float, default=15.0, help="어깨 탐색 팬 스윕 간격 (°)")
    p.add_argument("--search-limit", type=float, default=60.0, help="어깨 탐색 팬 스윕 범위 ±°")
    p.add_argument("--search-dwell", type=int, default=8, help="탐색 지점당 관찰 프레임")
    # ── 원점 상태 파일 (요구 5) — 모든 스크립트 공용 ─────────────────────────
    add_state_args(p)
    p.add_argument("--no-restore", action="store_true",
                   help="시작 시 원점 복원 생략 (헤드를 손으로 0°에 맞춰뒀을 때)")
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

    detector = MoveNetMultiPoseDetector(args.model, conf_thr=args.conf, num_threads=args.threads)
    cam, backend = _open_camera(args.opencv, args.cam, use_rpicam=args.rpicam)
    tracker = PoseTracker()
    _fov = CFG["fov"]
    scenario = FullBodyScenario(
        _fov["h"], _fov["v"],
        args.pan_min, args.pan_max, args.tilt_min, args.tilt_max,
        gain=args.gain, gain_tilt=args.gain_tilt,
        invert_pan=args.invert, invert_tilt=args.invert_tilt,
        converge_deg=args.converge, converge_frames=args.converge_frames,
        dwell_s=args.dwell, trim_alpha=args.trim_alpha,
        move_thr_deg=args.move_thr, rescan_thr_deg=args.rescan_thr,
        lost_frames=args.lost_frames, occl_frames=args.occl_frames,
        blind_deg=args.blind_deg, lower_ratio=args.lower_ratio,
        search_step_deg=args.search_step, search_limit_deg=args.search_limit,
        search_dwell=args.search_dwell,
    )
    web_srv = web_state = None
    if args.web:
        web_state = _WebStreamState(args.web_quality, args.web_fps)
        web_srv = _ThreadedHTTP((args.web_host, args.web_port), _make_handler(web_state))
        threading.Thread(target=web_srv.serve_forever, daemon=True).start()
        print(f"[web] http://{args.web_host}:{args.web_port}/  (브라우저에서 열기)")

    print("\n[verify_fulltrack] 전신 추적 시작 — 카메라가 팬·틸트 헤드에 함께 실린 상태를 가정합니다.")
    print(f"  gain={args.gain}/{args.gain_tilt}  deadzone={args.deadzone}/{args.deadzone_tilt}°  "
          f"tilt=[{args.tilt_min:g},{args.tilt_max:g}]°  invert={args.invert}/{args.invert_tilt}")

    motor_cm = open_motor_from_args(args)
    try:
        with motor_cm as mc:
            mc.enable()
            if args.no_restore:
                mc.home()
                print("[motor] --no-restore — 현재 위치를 (0°, 0°)로 선언합니다.")
            else:
                mc.restore_origin()
            try:
                _track_loop(args, detector, cam, backend, tracker, scenario, mc,
                            web_state)
            finally:
                _return_home(mc)
    except KeyboardInterrupt:
        print("\n[verify_fulltrack] Ctrl+C 중단")
    finally:
        if web_srv:
            web_srv.shutdown(); web_srv.server_close()
        _release_camera(cam, backend)
        cv2.destroyAllWindows()
        print("\n[verify_fulltrack] 종료")


if __name__ == "__main__":
    main()
