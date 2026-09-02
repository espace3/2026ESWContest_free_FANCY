# -*- coding: utf-8 -*-
"""
app/runners.py - 모드별 러너 팩토리 + 모드 감독

앱이 모드를 바꾸면 이 파일이 정한 러너 하나가 스레드에서 돈다.

  러너(runner)   stop_event 하나만 받는 콜러블. 무한 루프로 한 가지 동작을 한다.
  팩토리         프로세스 시작 시 1회 호출해 러너를 만든다 (mc/args 를 클로저에
                 가둬 두므로 supervisor 는 러너의 종류를 몰라도 갈아끼울 수 있다).
  ModeSupervisor 전원·모드·풍속 → 러너 선택, 스레드 교대(stop → 감속 → join → start).

  _make_runner       타겟 모드(0x02) 추적       카메라 O
  _make_body_runner  부위 모드(0x03) 순찰·폴백   카메라 O
  _make_sweeper      기본-회전 모드(0x01) 스윕    카메라 X
  _make_homer        복귀 후 대기 (고정·파킹)     카메라 X

이력: 원래 이 코드는 기능을 하나씩 얹으며 만든 세 개의 통합 스크립트에 개발
순서대로 흩어져 있었고, supervisor 도 그 순서대로 3단 상속이었다(인스턴스화되는
구체 클래스는 마지막 하나뿐). "어느 파일의 어느 클래스가 실제로 도는가"를 매번
확인해야 해서 책임별로 모으고 한 클래스로 합쳤다 (2026-09-02). 그 단계별 코드는
git 이력에 있다.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path

# 레포 루트를 path에 추가 (config / vision / control / hardware / app 해결용)
sys.path.insert(0, str(Path(__file__).parent.parent))

import cv2

from dbus_fast.constants import MessageType
from dbus_fast.message import Message

from config import CFG, TARGET_MODES
from control.body_wind import BodyPatrolScenario, MotionGate, body_wind_level
from control.control_signal_generator import (apply_deadzone, clamp_angle,
                                              compute_pan_angle,
                                              compute_tilt_angle)
from control.recognition_reporter import RecognitionReporter
from vision.pose_tracker import PoseTracker
from vision.target_selector import (DEFAULT_MATCH_RADIUS, person_center,
                                    select_target)
from app.tracking import (_INVISIBLE, _axes_idle, _draw_overlay, chest_point,
                          run_tracking)
from app.camera import _open_cam_retry, _read_frame, _release_cam_pause


class _ReportingDetector:
    """추적 러너용 디텍터 래퍼 — "사람이 보이는가"를 서비스에 보고한다.

    일반 타겟 모드(0x02)의 추적 루프는 app/tracking.py 안에 있어 콜백 자리가 없다.
    공유 모듈(_make_runner · app/tracking.py — 추적 모드 공용)을 고치지 않고
    인식 상태를 얻으려고 디텍터를 감쌌다: infer 결과를 들여다보기만 하고 그대로
    돌려준다. 서비스는 나중에 만들어지므로(supervisor → service 순서) attach로
    꽂는다. 보고 시점은 RecognitionReporter가 정한다(깜빡임 억제).
    부위 러너는 선정된 대상(target_idx) 기준으로 따로 보고하므로 여기 기준
    ("사람이 한 명이라도 검출")과 미세하게 다를 수 있다.
    """

    def __init__(self, detector) -> None:
        self._d = detector
        self._service = None
        self._reporter = RecognitionReporter()

    def attach(self, service) -> None:
        self._service = service

    def infer(self, frame):
        result = self._d.infer(frame)
        seen = self._reporter.update(time.time(), bool(result.get("people")))
        if seen is not None and self._service is not None:
            self._service.report_recognized(seen)
        return result

    def __getattr__(self, name):
        return getattr(self._d, name)   # 나머지 속성/메서드는 원본에 위임



def fan_level_txt(levels, scenario, phase, common_level) -> str:
    """상태 로그용 — 지금 릴레이에 적용 중인 세기 표기."""
    lv = (common_level if phase == "fallback"
          else body_wind_level(scenario, levels, common_level))
    return f"{lv}단" if lv else "정지"



def _make_runner(axis, detector, tracker, mc, args, web_state):
    """axis에 맞는 app/tracking.py 루프를 stop_event 하나만 받는 콜러블로 감싼다.

    카메라는 세션(스레드)마다 새로 열고 끝나면 반드시 해제한다 — 특히
    rpicam-vid(기본) 백엔드는 아무도 읽지 않는 동안 파이프 버퍼가 차서 캡처
    자체가 멎어버리는 게 실기로 확인됐다. "기본 모드"로 쉬는 동안 카메라를
    계속 열어두면 재진입 시 그 멎은 상태(화면 정지) 그대로 남는다 — 매번
    새로 열면 이 문제를 피한다. 오픈·해제 자체는 app/camera.py 의
    _open_cam_retry / _release_cam_pause 가 한다 (부위 러너와 공용).
    """
    fov_cfg = CFG["fov"]
    fov_h, fov_v = fov_cfg["h"], fov_cfg["v"]
    sign_pan = -1.0 if args.invert_pan else 1.0
    sign_tilt = -1.0 if args.invert_tilt else 1.0
    # 부위 선택은 틸트에서만 의미가 있다 (팬은 어느 부위를 겨눠도 같은 각도).
    aim_key = args.region if (axis == "tilt" and args.region != "chest") else "upper"

    def _run(stop_event):
        cam, backend = _open_cam_retry(args)
        try:
            run_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                         axis=axis, fov_h=fov_h, fov_v=fov_v,
                         sign_pan=sign_pan, sign_tilt=sign_tilt,
                         aim_key=aim_key, web_state=web_state)
        finally:
            _release_cam_pause(cam, backend)

    return _run


def _make_sweeper(mc, args):
    """기본-회전 모드(0x01, 풍속 ≥1)용 러너 — pan을 원점(0°) 기준 ±span° 왕복,
    tilt는 0° 유지. 카메라/디텍터는 쓰지 않는다.

    [왜 목표를 앞세우고 반전은 실제 위치로 보는가]
    MotorController 는 목표를 따라잡으면 감속 → 큐 배출 → 정지한다. 그래서 목표가
    모터 순항 속도보다 느리게 전진하면 매 틱 "램프 상승 → 따라잡음 → 감속 → 정지 →
    재기동"을 반복해 눈에 띄게 덜컹거린다 (실기 2026-09-02).
    연속으로 돌리려면 **목표가 항상 모터보다 앞서 있어야** 한다.

    그런데 목표만 앞세우면 ±span 반전 판정이 실제 헤드 위치가 아니라 달아난 목표로
    일어나 스윕 폭이 설정값보다 좁아지고 좌우가 비대칭이 된다. 그래서 **전진은
    lead 만큼 앞세우고, 반전은 실제 장부 위치로 판정**한다.

    결과적으로 스윕 속도는 모터 순항 속도로 고정된다 — 그보다 느리게 "부드럽게"
    돌 방법이 이 드라이버 구조에는 없다 (f_max 를 낮추지 않는 한).
    팬 순항 = f_max × 360 / (steps_per_rev × microstep × gear_ratio) ≈ 7.9°/s.

    stop_event 에는 한 틱(50ms) 안에 반응한다 — 다음 목표를 안 주면 모터가 lead
    만큼 더 가고 그 자리에 감속 정지한다.
    """
    span, lead, period = args.rotate_span, args.rotate_lead, 0.05

    def _run(stop_event):
        pos = mc.current_position()[0]
        direction = -1.0 if pos > 0 else 1.0
        print(f"[E2E] 회전 스윕 시작 — pan ±{span:g}° (0° 기준, 모터 순항 속도), tilt 0°")
        while not stop_event.is_set():
            pos = mc.current_position()[0]
            # 반전은 실제 위치로 판정한다 (span 밖에서 진입해도 범위 쪽으로 걸어온다).
            if pos >= span:
                direction = -1.0
            elif pos <= -span:
                direction = 1.0
            # 목표는 항상 lead 만큼 앞 — 모터가 큐를 비우지 않아 연속으로 돈다.
            mc.move_to(pos + direction * lead, 0.0)
            stop_event.wait(period)

    return _run


def _make_homer(mc, home_pan: bool):
    """복귀 러너 (docstring 8) — tilt는 0°로, pan은 home_pan이면 0°(전원 OFF
    파킹), 아니면 그 자리 유지(기본-고정·회전 정지). 러너가 시작되는 시점은
    supervisor가 이전 스레드를 join한 뒤라 current_position()이 확정 위치다."""
    def _run(stop_event):
        pan, tilt = mc.current_position()
        pan_t = 0.0 if home_pan else pan
        if abs(tilt) > 0.01 or abs(pan_t - pan) > 0.01:
            what = "0°,0° 복귀 (파킹)" if home_pan else f"틸트 0° 복귀 (pan {pan:+.1f}° 유지)"
            print(f"[E2E] {what}")
            mc.move_to(pan_t, 0.0)
        stop_event.wait()

    return _run


def _make_body_runner(detector, tracker, mc, fan, service, gains, args, web_state):
    """부위 모드(0x03) 러너 — 내부 2상(순찰↔추적 폴백) 단일 세션 (docstring 5).

    전신 추적 루프를 세션형(stop_event)으로 옮기고 풍속 중재
    (body_wind_level)·MotionGate 전환·유효 모드/인식 보고를 더했다.
    시나리오는 세션마다 새로 만든다 — 모드를 떠났다 돌아오면 사용자 위치·거리가
    달라졌을 수 있어서인데, 직접 매핑이라 다시 잡는 비용이 한 프레임이다.
    gains=(pan, tilt)는 main에서 미리 캡처한 값을 그대로 받는다.
    """
    fov = CFG["fov"]
    fov_h, fov_v = fov["h"], fov["v"]
    gain_pan, gain_tilt = gains
    # 재조준·탐색은 부모가 수렴 판정으로 진행을 막으므로, 그 구간에서만
    # 데드존을 gain x converge_deg 아래로 좁힌다 (순찰은 시간 슬롯이라
    # 도착 판정이 없어 사용자 데드존을 그대로 써도 갇히지 않는다).
    conv_dz_pan = min(args.body_deadzone_pan, 0.5 * gain_pan * args.body_converge)
    conv_dz_tilt = min(args.body_deadzone_tilt, 0.5 * gain_tilt * args.body_converge)
    sign_pan = -1.0 if args.invert_pan else 1.0
    sign_tilt = -1.0 if args.invert_tilt else 1.0

    def _region_levels():
        b = service.body_levels  # BLE 스레드가 갱신하는 dict — int 읽기는 원자적
        return {"head": b[0x01], "upper": b[0x02], "lower": b[0x03]}

    def _run(stop_event):
        cam, backend = _open_cam_retry(args)
        scenario = BodyPatrolScenario(
            fov_h, fov_v, args.pan_min, args.pan_max, args.tilt_min, args.tilt_max,
            gain_pan=gain_pan, gain_tilt=gain_tilt,
            invert_pan=args.invert_pan, invert_tilt=args.invert_tilt,
            target_cx=args.target_cx, target_cy=args.target_cy,
            dwell_s=args.body_dwell,
            converge_deg=args.body_converge,
            map_timeout_s=args.body_map_timeout,
            # 시나리오 자체 이동 감지(recenter)는 게이트보다 둔하게 — 게이트가
            # 1차 판정자다 (docstring 5의 "이동 감지 우선순위" 참고).
            move_thr_deg=args.body_move_thr,
            rescan_thr_deg=args.body_rescan_thr,
            # 조준 편향: 머리는 위로, 상체는 아래로 — 부위 간 틸트 간격 확대
            # 조준 보정은 각도가 아니라 측정 간격의 배수 — 거리 자동 대응.
            aim_ratio={"head": args.body_head_ratio,
                       "upper": args.body_upper_ratio},
            spread_ratio=args.body_spread_ratio,
            tilt_rate_dps=args.body_tilt_rate)
        gate = MotionGate(args.body_exit_deg, args.body_exit_window,
                          args.body_still_s, args.body_still_deg)
        tracker.reset()  # 이전 세션의 스무딩 잔재 제거
        phase = "patrol"
        service.report_effective(0x03)
        prev_center = None
        last_pan, last_tilt = mc.current_position()
        recognition = RecognitionReporter()   # 인식 Status 보고 시점 (깜빡임 억제)
        fps_hist: list[float] = []
        t_prev = time.time()
        last_log = 0.0
        print(f"[E2E] 부위 순찰 세션 시작 (매핑 + 시간 슬롯) — 조준 배수 "
              f"머리 {args.body_head_ratio:g} 상체 {args.body_upper_ratio:g} "
              f"최소간격 {args.body_spread_ratio:g} (측정 간격 대비)")
        try:
            while not stop_event.is_set():
                t0 = time.time()
                frame = _read_frame(cam, backend)
                if frame is None:
                    if web_state:
                        web_state.update_stall(["no frames from the camera."])
                    stop_event.wait(0.03)
                    continue

                # ── 추론/대상 선정/스무딩 (run_tracking 과 같은 순서) ──
                people = detector.infer(frame)["people"]
                target_idx = (
                    select_target(people, prev_center=prev_center)
                    if people else None
                )
                if target_idx is not None:
                    kps = people[target_idx]["keypoints"]
                    new_center = person_center(people[target_idx])
                    if (prev_center is not None and new_center is not None
                            and math.dist(new_center, prev_center) > DEFAULT_MATCH_RADIUS):
                        tracker.reset()  # 대상 교체 → 스무딩 리셋
                    prev_center = new_center
                    chest = chest_point(kps, args.conf)
                    tracker_input = {"detected": True,
                                     "regions": people[target_idx]["regions"]}
                else:
                    chest = dict(_INVISIBLE, paired=False)
                    tracker_input = {"detected": False,
                                     "regions": {k: _INVISIBLE for k in PoseTracker.REGIONS}}
                smoothed = tracker.update(tracker_input)
                fresh = {k: tracker.miss[k] == 0 for k in PoseTracker.REGIONS}

                seen = recognition.update(t0, target_idx is not None)
                if seen is not None:
                    service.report_recognized(seen)

                cur_pan, cur_tilt = mc.current_position()
                levels = _region_levels()
                scenario.allowed = {r for r, lv in levels.items() if lv > 0}
                scenario.levels = levels   # 세기 0인 부위는 체류 없이 지나간다
                # 조준점은 화면 중앙이 아니라 --target-cx/cy 다. 카메라 렌즈와
                # 송풍구가 같은 높이가 아니라 렌즈를 정확히 맞추면 바람은 어긋난다
                # — cy 를 0.5 에서 옮겨 그 차이를 보정한다.
                # paired=False(한쪽 어깨만)면 cx 가 어깨폭 절반만큼 치우쳐 있어
                # 팬 기준으로 못 쓴다 — 오차 0으로 두어 팬을 유지한다 (chest_point 참고).
                chest_err = (sign_pan * compute_pan_angle(chest["cx"] - (args.target_cx - 0.5), fov_h)
                             if chest["visible"] and chest["paired"] else 0.0)

                # ── 상별 목표각 + 풍속 중재 + 전환 판정 (docstring 5) ─────────
                if phase == "patrol":
                    pan_t, tilt_t = scenario.step({
                        "t": t0, "pos": (cur_pan, cur_tilt), "idle": _axes_idle(mc),
                        "target_idx": target_idx, "chest": chest,
                        "regions": smoothed, "fresh": fresh,
                    })
                    for ev in scenario.events:
                        print(f"\n[E2E] {ev}")
                    fan.set_speed(body_wind_level(scenario, levels,
                                                  service.common_level))
                    if gate.update_patrol(t0, cur_pan, scenario.state == "patrol"):
                        phase = "fallback"
                        gate.reset_fallback()
                        service.report_effective(0x02)
                        print("\n[E2E] 이동 감지 → 추적 폴백 (정지하면 순찰 재개)")
                else:
                    if chest["visible"]:
                        et = sign_tilt * compute_tilt_angle(chest["cy"] - (args.target_cy - 0.5), fov_v)
                        pan_t = clamp_angle(cur_pan + gain_pan * chest_err,
                                            args.pan_min, args.pan_max)
                        tilt_t = clamp_angle(cur_tilt + gain_tilt * et,
                                             args.tilt_min, args.tilt_max)
                    else:
                        pan_t, tilt_t = last_pan, last_tilt  # 미관측 — 조준 유지
                    # 유효 모드가 추적(0x02)이므로 풍속도 추적 모드와 동일한
                    # 공용 세기(앱 타겟 화면 표시값)를 쓴다 — 리모컨 주체 일관.
                    fan.set_speed(service.common_level)
                    if gate.update_fallback(t0, cur_pan, chest["visible"]):
                        phase = "patrol"
                        gate.reset_patrol()
                        service.report_effective(0x03)
                        print("\n[E2E] 정지 감지 → 부위 순찰 재개")

                # 데드존은 원래 목적(포즈 잡음이 모터로 새는 것 차단)대로
                # 쓴다. 순찰이 시간 슬롯이라 도착 판정이 없어져, 데드존이
                # 남기는 정지 오차가 진행을 막지 않는다. 다만 재조준·탐색은
                # 부모가 여전히 수렴으로 판정하므로 그때만 좁힌다.
                converging = scenario.state in ("recenter", "search")
                # 팬은 순찰에서도 사용자 데드존을 쓴다(1.0° — 사각지대 3.3°).
                # 한때 2.0°였던 건 하체→상체 전환에서 팬이 따라 움직이는 것을
                # 막기 위해서였는데, 그 원인은 잡음이 아니라 한쪽 어깨만 잡힌
                # 프레임의 cx 도약이었다 — chest_point 의 paired 로 원인을 막은
                # 뒤 되돌렸다 (a6a41b4). 넓은 데드존은 도약을 근거리에서 막지도
                # 못하면서(1.3m 이내) 상시 조준 오차만 6.7° 로 키웠다.
                pan_g = apply_deadzone(pan_t, last_pan,
                                       conv_dz_pan if converging else args.body_deadzone_pan)
                # 틸트는 순찰도 좁힌다 — 조준이 연속 피드백(현재각 + gain x 오차)
                # 이라 넓은 데드존은 오차 < 데드존/gain 에서 명령을 통째로 막아
                # 그만큼 못 미친 채 세운다 (1.0/0.2 = 5°). 좁히면 1.25°.
                tilt_g = apply_deadzone(
                    tilt_t, last_tilt,
                    conv_dz_tilt if (converging or scenario.state == "patrol")
                    else args.body_deadzone_tilt)
                if not stop_event.is_set() and (pan_g, tilt_g) != (last_pan, last_tilt):
                    mc.move_to(pan_g, tilt_g)
                    last_pan, last_tilt = pan_g, tilt_g

                # ── FPS/로그/프레임 송출 ─────────────────────────────────────
                dt = time.time() - t_prev
                t_prev = time.time()
                fps_hist.append(1.0 / dt if dt > 0 else 0.0)
                if len(fps_hist) > 30:
                    fps_hist.pop(0)
                fps = sum(fps_hist) / len(fps_hist)
                if time.time() - last_log >= 1.0:
                    tag = scenario.state if phase == "patrol" else "fallback"
                    print(f"\r[{time.strftime('%H:%M:%S')}] body:{tag:<8} "
                          f"rg={scenario.active_region():<5} pan={cur_pan:+7.2f}° "
                          f"tilt={cur_tilt:+6.2f}° "
                          f"wind={fan_level_txt(levels, scenario, phase, service.common_level)} "
                          f"fps={fps:4.1f}  ", end="", flush=True)
                    last_log = time.time()
                if web_state:
                    vis = _draw_overlay(frame, people, target_idx, smoothed, fresh,
                                        scenario, last_pan, last_tilt, fps,
                                        target_cx=args.target_cx,
                                        target_cy=args.target_cy,
                                        aim_bias_deg=scenario.aim_ratio.get(
                                            scenario.active_region(), 0.0)
                                        * scenario.gap_deg,
                                        fov_v=fov_v)
                    if phase == "fallback":
                        cv2.putText(vis, "FALLBACK (tracking)", (10, 72),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 80, 255), 2)
                    web_state.update(vis)
                stop_event.wait(max(0.0, 0.02 - (time.time() - t0)))
        finally:
            # 러너 소유 종료 — 릴레이를 0으로 놓고 나가면, 서비스가 join 후
            # 다음 상태를 재적용한다 (docstring 7).
            fan.set_speed(0)
            _release_cam_pause(cam, backend)

    return _run


class ModeSupervisor:
    """전원·모드·풍속 → 러너 하나. 상태가 바뀌면 스레드를 갈아끼운다.

    ── 왜 "아무것도 안 함"도 러너인가 ────────────────────────────────────────
    기본-고정(0x00)이나 전원 OFF에서도 _make_homer 러너가 돈다(한 번 움직이고
    stop_event.wait()로 대기). 그래야 모든 전환이 stop → 모터 감속 → join →
    start 라는 같은 경로를 타고, 죽어가는 이전 러너가 던지는 마지막 move_to 와
    새 러너의 move_to 가 섞이지 않는다.

    ── 러너 선택 ────────────────────────────────────────────────────────────
      전원 OFF          → park   (0°,0° 파킹)
      0x03 + body_fn    → body   (부위 순찰)
      0x02 / 0x03       → track  (가슴 추적)
      0x01 + 풍속 ≥1    → sweep  (좌우 왕복)
      그 외             → home   (틸트만 0° 복귀 후 대기)

    회전만 풍속과 연동한다 — 타겟 모드는 바람이 꺼져도 조준을 유지해야 재개 시
    즉시 맞는 상태가 되기 때문. 0x03 은 body_fn 이 주입되기 전(서비스보다 늦게
    만들어진다)에는 track 으로 떨어진다.
    """

    def __init__(self, track_fn, sweep_fn, home_fn, park_fn, stop_fn=None,
                 web_state=None) -> None:
        self._track_fn = track_fn
        self._sweep_fn = sweep_fn
        self._home_fn = home_fn
        self._park_fn = park_fn
        self._body_fn = None       # set_body_runner 로 나중에 주입
        self._stop_fn = stop_fn    # 모터 감속 정지 (러너 교체 직전 호출)
        self._web_state = web_state
        self._run_fn = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._power_on = False
        self._mode = 0x00
        self._wind_level = 0

    def set_body_runner(self, body_fn) -> None:
        """부위 러너 주입 — 서비스의 report_* 를 쓰므로 서비스보다 늦게 만들어진다."""
        self._body_fn = body_fn

    def _select_runner(self):
        if not self._power_on:
            return self._park_fn
        if self._mode == 0x03 and self._body_fn is not None:
            return self._body_fn
        if self._mode in TARGET_MODES:
            return self._track_fn
        if self._mode == 0x01 and self._wind_level > 0:
            return self._sweep_fn
        return self._home_fn

    def _thread_main(self, stop_event) -> None:
        """예외로 죽으면 원인이 조용히 사라지지 않게 traceback을 남긴다."""
        try:
            self._run_fn(stop_event)
        except Exception:
            import traceback
            print("\n[E2E] 러너 스레드가 예외로 종료됨:")
            traceback.print_exc()
        print("[E2E] 러너 스레드 종료")
        if self._web_state:
            # 창/웹 화면이 마지막 프레임에 얼어 보이지 않게 대기 안내로 교체.
            self._web_state.update_stall(["tracking stopped",
                                          "switch to target mode to resume"])

    def _apply(self) -> None:
        run_fn = self._select_runner()
        # 같은 러너가 정상 구동 중(정지 요청 없음)이면 재시작하지 않는다 —
        # 앱이 같은 모드를 재전송하거나 풍속을 1→2 로 바꿀 때 카메라/스윕
        # 세션이 불필요하게 끊기는 것 방지.
        if (run_fn is self._run_fn and self._thread is not None
                and self._thread.is_alive() and not self._stop_event.is_set()):
            return
        if self._thread is not None and self._thread.is_alive():
            # 이전 러너가 다음 틱까지 새 목표를 던지지 못하도록 먼저 중단
            # 신호와 모터 감속 정지를 전달한 뒤 join 한다.
            self._stop_event.set()
            if self._stop_fn is not None:
                self._stop_fn()
            print("[E2E] 이전 러너 스레드 정리 중...")
            self._thread.join(timeout=15)
            if self._thread.is_alive():
                # 여기서 그냥 return 하면 이후 시작 요청이 영영 씹힌다 —
                # 예전에 is_alive() 만 보고 리턴해 추적이 재개 안 되던 버그가
                # 있었으므로, 살아 있으면 시작을 건너뛰되 상태는 남긴다.
                print("[E2E] 이전 스레드가 아직 안 끝남 — 이번 시작은 건너뜁니다 "
                      "(모드를 다시 토글해 보세요)")
                return
        self._run_fn = run_fn
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._thread_main,
                                        args=(self._stop_event,), daemon=True)
        self._thread.start()

    def set_state(self, power_on: bool, mode: int, wind_level: int) -> None:
        """power/mode/wind를 한 상태로 갱신한 뒤 러너를 재선택한다."""
        self._power_on = power_on
        self._mode = mode
        self._wind_level = wind_level
        self._apply()

    def stop_and_join(self, timeout: float = 10) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=timeout)


async def _watch_disconnects(bus, service) -> None:
    """BlueZ Device1.Connected 프로퍼티 변경을 구독해 연결 끊김을 감지한다
    (docstring 9 — bluez_peripheral엔 연결 이벤트 API가 없어 D-Bus 직접 구독).

    service 는 handle_disconnect() 를 가진 GATT 서비스면 된다 (main.py 의
    EswFanService)."""
    await bus.call(Message(
        destination="org.freedesktop.DBus", path="/org/freedesktop/DBus",
        interface="org.freedesktop.DBus", member="AddMatch", signature="s",
        body=["type='signal',interface='org.freedesktop.DBus.Properties',"
              "member='PropertiesChanged',arg0='org.bluez.Device1'"]))

    def _handler(msg):
        if (msg.message_type != MessageType.SIGNAL
                or msg.interface != "org.freedesktop.DBus.Properties"
                or msg.member != "PropertiesChanged"):
            return
        iface, changed, _ = msg.body
        if iface != "org.bluez.Device1" or "Connected" not in changed:
            return
        dev = msg.path.rsplit("/", 1)[-1]  # dev_XX_XX_... (MAC)
        if changed["Connected"].value:
            print(f"[BLE] 중앙 연결됨 ({dev})")
        else:
            print(f"[BLE] 중앙 연결 끊김 ({dev})")
            service.handle_disconnect()

    bus.add_message_handler(_handler)
