#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/verify_E2E_v3.py - STATUS 실구현 + 리모컨 주체 동기화 (v3, 1단계)

verify_E2E_v2.py와의 차이만 기록합니다 — 카메라 세션/창 스레드/축별 튜닝/
회전 스윕/연결 끊김 처리 등은 v1·v2 docstring과 동일하므로 반복하지 않습니다:

  1. 리모컨(앱) = 설정의 주체 — v2의 모드별 풍량 preset(_basic/_target_level)과
     "모드 전환 시 저장 세기 재적용"(v2 docstring 2)을 폐기한다. 세기는 오직
     풍량 write로만 바뀌고, 모드/전원 전환은 게이팅(전원 OFF·부위 모드 → 0)만
     한다. 모드 전환 직후의 세기는 앱이 화면 표시값을 재전송해 맞춘다
     (계약 — ble_protocol.md §3.3). 앱 미갱신 과도기에는 전환 시 직전 세기가
     유지된다 (v2처럼 저장값으로 "자동으로 바뀌는" 현상 제거).
  2. STATUS(0x0005) 실구현 — read | notify (ble_protocol.md §3.4).
     read: 0x04 스냅샷 [전원, 요청 모드, 유효 모드, 공용 세기, 머리, 상체, 하체]
       — 앱이 재전송 직후 표시값과 대조하는 불일치 검증용 (주체는 앱이므로
       앱이 이 값을 따라가지 않는다. 다르면 앱이 재전송으로 교정).
     notify: 적용된 write마다 에코백 [0x01, char#, 원본...]. 거부된 write는
       에코 없음 — 앱이 타임아웃으로 감지한다. 유효 모드 push(0x03)와 객체
       인식 Status(0x02)는 부위 러너 단계(다음)에서 송신 시작 — 요청/유효
       모드가 갈라지는 첫 지점이 그때라서.
  3. 풍량 [0x02(상체), 0x00] 거부 — 상체는 1~3단만 (부위 순찰 경로가 비지
     않게 하는 보장). 부위별 세기 기본값도 상체만 1단.
  4. 유효 모드(_effective_mode) 도입 — 요청 모드(앱이 write한 값)와 별개로
     RPi가 실제로 돌리는 모드. 부위 러너의 추적 폴백 중에만 갈라진다(아래 5).
  5. 부위 모드(0x03) 실동작 — 단일 세션 러너(_make_body_runner)가 내부 2상으로
     돈다. v2는 0x03을 0x02와 동일 취급 + 풍속 0이었다.
       순찰(patrol): BodyPatrolScenario(control/body_wind.py)가 head→upper→
         lower 순회(세기 0 부위 제외). 부위 각도는 반복 수렴 스캔이 아니라
         **한 프레임 직접 매핑**으로 잡고 곧바로 순찰에 들어간다(수직 FOV 67°
         광각이라 전신이 한 프레임에 담긴다 — 근거는 그쪽 클래스 docstring).
         풍속은 지금 겨누는 부위의 저장 세기를 쓰고, 겨누는 부위가 없는
         동안(매핑·탐색·재조준)은 공용 세기 — 아래 폴백과 같은 규칙.
       추적 폴백(fallback): 순찰 안정 상태에서 최근 --body-exit-window 동안
         팬 이동량 > --body-exit-deg면 전환 (팬은 사용자를 따라갈 때만 움직여
         헤드 자체 스윙에 면역 — control/body_wind.py MotionGate 참고).
         가슴 중심 조준(타겟 모드와 같은 피드백)에 **공용 세기** 적용
         (유효 모드가 추적이므로 풍속도 추적 모드와 동일 — 리모컨 주체 일관).
         팬 각이 --body-still-s 동안 잠잠하면 순찰 재개.
       이동 감지 우선순위: 시나리오 자체 이동 감지(recenter, --body-move-thr)는
         게이트(--body-exit-deg)보다 둔하게 둔다 — recenter가 먼저 걸리면 게이트
         창이 비워져(비 patrol) 폴백이 안 나오고, 작은 흔들림에도 순찰이 자주
         끊긴다 (실기 2026-08-27). 미세 흔들림은 --body-deadzone으로 억제한다.
       조준각 벌리기: 부위 간 틸트 간격이 실기에서 작아(거리 2m에 머리↔상체
         ~10° + 틸트 리밋 ±15°) 두 장치를 쓴다 —
         --body-head-bias/--body-upper-bias는 조준점을 옮기고(관측 쪽),
         --body-tilt-spread는 부위별 틸트 구간을 어긋나게 잘라(출력 쪽)
         아래쪽 리밋을 하체 전용으로 남긴다. 후자가 없으면 상체가 리밋에
         눌릴 때 하체도 같은 각도가 되어 순찰이 멈춘 것처럼 보인다
         (control/body_wind.py aim_bias_norm / _spread_clamp 참고).
     러너 교체가 아니라 내부 상 전환이라 카메라 세션이 안 끊기고, 전환마다
     유효 모드 push [0x03, 모드]가 나간다. 폴백 중 가슴을 잃으면 마지막 조준을
     유지한다(재탐색 스윕은 순찰 상의 search 몫 — 알려진 한계).
     부위 러너는 --axis와 무관하게 팬+틸트를 모두 쓴다.
     0x02↔0x03 전환은 이제 러너가 달라 세션이 재시작된다(v2 docstring 8의
     "0x02↔0x03 세션 유지"는 v3에서 성립하지 않음).
  6. notify 스레드 규칙 — 러너 스레드는 report_effective/report_recognized로
     보고하고, 서비스가 loop.call_soon_threadsafe로 BLE asyncio 루프에 넘긴다
     (dbus_fast는 스레드 안전하지 않음). 객체 인식 Status [0x02, x]는 인식
     여부가 바뀔 때만 보낸다.
  7. _apply_state 순서 — supervisor(이전 러너 join 완료) 먼저, 릴레이 적용을
     나중에. 반대면 죽어가는 부위 러너의 마지막 프레임이 방금 적용한 릴레이
     값을 덮는 레이스가 있다. 부위 모드 중에는 서비스가 릴레이를 아예 건드리지
     않는다(러너 소유 — 러너 종료 시 finally에서 0으로 놓고 나온다).

실행 (RPi 5, 레포 루트에서):
    python3 scripts/verify_E2E_v3.py --axis pan
    python3 scripts/verify_E2E_v3.py --axis pantilt --rpicam --no-window
    python scripts/verify_E2E_v3.py --axis pan --dry-run --opencv   # 개발 PC
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
import threading
import time
from pathlib import Path

# 레포 루트 + scripts 디렉터리를 path에 추가 (config/vision/control + v1·v2 재사용)
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

import cv2

from bluez_peripheral.gatt import CharacteristicFlags as CharFlags
from bluez_peripheral.gatt import Service, characteristic

from config import CFG
from control.body_wind import BodyPatrolScenario, MotionGate, body_wind_level
from control.control_signal_generator import (apply_deadzone, clamp_angle,
                                              compute_pan_angle,
                                              compute_tilt_angle)
from vision.pose_estimator import MoveNetMultiPoseDetector
from vision.pose_tracker import PoseTracker
from vision.target_selector import (DEFAULT_MATCH_RADIUS, person_center,
                                    select_target)
from tracking_core import add_state_args, open_motor_from_args
from verify_movenet import (_open_camera, _read_frame, _release_camera,
                            _WebStreamState, _make_handler, _ThreadedHTTP)
from verify_fulltrack import _INVISIBLE, _axes_idle, _draw_overlay, chest_point
from verify_E2E_v1 import (SERVICE_UUID, POWER_UUID, MODE_UUID, WIND_UUID,
                           STATUS_UUID, MODE_NAMES, WIND_TARGETS,
                           _hex, _make_runner, _window_viewer)
from verify_E2E_v2 import (_DryRelay, _make_sweeper, _make_homer,
                           _ModeSupervisor, _ble_main)


def _open_cam_retry(args):
    """카메라 오픈 + 첫 프레임 확인 — v1 _make_runner._open_cam과 동일 로직
    (그쪽은 클로저 안이라 import 불가). 실패 경로 설명은 v1 참고."""
    for attempt in range(1, 4):
        try:
            cam, backend = _open_camera(args.opencv, args.cam, use_rpicam=args.rpicam)
        except SystemExit:
            print(f"[E2E] 카메라 오픈 실패 ({attempt}/3) — 1s 후 재시도")
            time.sleep(1.0)
            continue
        deadline = time.time() + 4.0
        while time.time() < deadline:
            if _read_frame(cam, backend) is not None:
                print(f"[E2E] 카메라 준비 완료 (backend={backend})")
                return cam, backend
            time.sleep(0.1)
        print(f"[E2E] 카메라가 프레임을 못 줌 ({attempt}/3) — 닫고 재오픈")
        _release_camera(cam, backend)
        time.sleep(1.0)
    raise RuntimeError("[E2E] 카메라를 열 수 없습니다 (재시도 소진)")


def _make_body_runner(detector, tracker, mc, fan, service, gains, args, web_state):
    """부위 모드(0x03) 러너 — 내부 2상(순찰↔추적 폴백) 단일 세션 (docstring 5).

    verify_fulltrack._track_loop를 세션형(stop_event)으로 옮기고 풍속 중재
    (body_wind_level)·MotionGate 전환·유효 모드/인식 보고를 더했다.
    시나리오는 세션마다 새로 만든다 — 모드를 떠났다 돌아오면 사용자 위치·거리가
    달라졌을 수 있어서인데, 직접 매핑이라 다시 잡는 비용이 한 프레임이다.
    gains=(pan, tilt)는 main에서 미리 캡처한 값 — v1 _make_runner가 --axis tilt
    에서 args.gain을 덮어쓰기 때문에 args를 런타임에 읽으면 안 된다.
    """
    fov = CFG["fov"]
    fov_h, fov_v = fov["h"], fov["v"]
    gain_pan, gain_tilt = gains
    # 수렴 구간용 축소 데드존 — 아래 apply_deadzone 주석 참고.
    conv_dz_pan = min(args.body_deadzone, 0.5 * gain_pan * args.body_converge)
    conv_dz_tilt = min(args.body_deadzone_tilt, 0.5 * gain_tilt * args.body_converge)
    sign_pan = -1.0 if args.invert else 1.0
    sign_tilt = -1.0 if args.invert_tilt else 1.0

    def _region_levels():
        b = service.body_levels  # BLE 스레드가 갱신하는 dict — int 읽기는 원자적
        return {"head": b[0x01], "upper": b[0x02], "lower": b[0x03]}

    def _run(stop_event):
        cam, backend = _open_cam_retry(args)
        scenario = BodyPatrolScenario(
            fov_h, fov_v, args.pan_min, args.pan_max, args.tilt_min, args.tilt_max,
            gain=gain_pan, gain_tilt=gain_tilt,
            invert_pan=args.invert, invert_tilt=args.invert_tilt,
            dwell_s=args.body_dwell,
            converge_deg=args.body_converge,
            # 스캔은 한 프레임 직접 매핑으로 대체됐다 — 부모의 반복 수렴
            # 인자(converge_frames/occl_frames/blind_deg)는 쓰이지 않는다.
            map_timeout_s=args.body_map_timeout,
            # 시나리오 자체 이동 감지(recenter)는 게이트보다 둔하게 — 게이트가
            # 1차 판정자다 (docstring 5의 "이동 감지 우선순위" 참고).
            move_thr_deg=args.body_move_thr,
            rescan_thr_deg=args.body_rescan_thr,
            # 조준 편향: 머리는 위로, 상체는 아래로 — 부위 간 틸트 간격 확대
            aim_bias_norm={"head": args.body_head_bias / fov_v,
                           "upper": -args.body_upper_bias / fov_v},
            tilt_spread_deg=args.body_tilt_spread)
        gate = MotionGate(args.body_exit_deg, args.body_exit_window,
                          args.body_still_s, args.body_still_deg)
        tracker.reset()  # 이전 세션의 스무딩 잔재 제거
        phase = "patrol"
        service.report_effective(0x03)
        prev_center = None
        last_pan, last_tilt = mc.current_position()
        recognized = None
        fps_hist: list[float] = []
        t_prev = time.time()
        last_log = 0.0
        spread = " ".join(
            f"{r}[{scenario._spread_clamp(r, args.tilt_min):+.0f},"
            f"{scenario._spread_clamp(r, args.tilt_max):+.0f}]"
            for r in ("head", "upper", "lower"))
        print(f"[E2E] 부위 순찰 세션 시작 (직접 매핑) — 틸트 구간 {spread}")
        try:
            while not stop_event.is_set():
                t0 = time.time()
                frame = _read_frame(cam, backend)
                if frame is None:
                    if web_state:
                        web_state.update_stall(["no frames from the camera."])
                    stop_event.wait(0.03)
                    continue

                # ── 추론/대상 선정/스무딩 (verify_fulltrack._track_loop와 동일) ──
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
                            and math.dist(new_center, prev_center) > DEFAULT_MATCH_RADIUS):
                        tracker.reset()  # 대상 교체 → 스무딩 리셋
                    prev_center = new_center
                    chest = chest_point(kps, args.conf)
                    tracker_input = {"detected": True,
                                     "regions": people[target_idx]["regions"]}
                else:
                    chest = dict(_INVISIBLE)
                    tracker_input = {"detected": False,
                                     "regions": {k: _INVISIBLE for k in PoseTracker.REGIONS}}
                smoothed = tracker.update(tracker_input)
                fresh = {k: tracker.miss[k] == 0 for k in PoseTracker.REGIONS}

                if (target_idx is not None) != recognized:
                    recognized = target_idx is not None
                    service.report_recognized(recognized)

                cur_pan, cur_tilt = mc.current_position()
                levels = _region_levels()
                scenario.allowed = {r for r, lv in levels.items() if lv > 0}
                chest_err = (sign_pan * compute_pan_angle(chest["cx"], fov_h)
                             if chest["visible"] else 0.0)

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
                        et = sign_tilt * compute_tilt_angle(chest["cy"], fov_v)
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

                # 데드존은 부위 모드 전용 인자를 쓴다 — args.deadzone은 --axis
                # tilt일 때 _make_runner가 틸트 값으로 덮어쓰기 때문(게인과 동일).
                # 수렴 판정이 걸린 구간(재조준·탐색)에서는 좁힌 값을 쓴다:
                # 명령 스텝이 gain×오차라, 데드존이 gain×converge_deg보다 크면
                # 오차가 판정선 근처일 때 명령이 통째로 억제돼 그 상태에서
                # 영영 못 벗어난다 (실기 2026-08-27 스캔 정체의 원인).
                converging = scenario.state in ("recenter", "search")
                pan_g = apply_deadzone(pan_t, last_pan,
                                       conv_dz_pan if converging else args.body_deadzone)
                tilt_g = apply_deadzone(tilt_t, last_tilt,
                                        conv_dz_tilt if converging else args.body_deadzone_tilt)
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
                                        scenario, last_pan, last_tilt, fps)
                    if phase == "fallback":
                        cv2.putText(vis, "FALLBACK (tracking)", (10, 72),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 80, 255), 2)
                    web_state.update(vis)
                stop_event.wait(max(0.0, 0.02 - (time.time() - t0)))
        finally:
            # 러너 소유 종료 — 릴레이를 0으로 놓고 나가면, 서비스가 join 후
            # 다음 상태를 재적용한다 (docstring 7).
            fan.set_speed(0)
            _release_camera(cam, backend)
            time.sleep(0.3)  # 다음 세션 재오픈 전 libcamera 자원 해제 여유

    return _run


def fan_level_txt(levels, scenario, phase, common_level) -> str:
    """상태 로그용 — 지금 릴레이에 적용 중인 세기 표기."""
    lv = (common_level if phase == "fallback"
          else body_wind_level(scenario, levels, common_level))
    return f"{lv}단" if lv else "정지"


class _ModeSupervisorV3(_ModeSupervisor):
    """v2 supervisor + 부위 모드(0x03) 전용 러너 분리 (v2는 0x02와 동일 취급).

    body_fn은 서비스보다 늦게 만들어져(러너가 서비스의 report_*를 쓰므로)
    main에서 생성 후 set_body_runner로 주입한다.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._body_fn = None

    def set_body_runner(self, body_fn) -> None:
        self._body_fn = body_fn

    def _select_runner(self):
        if self._power_on and self._mode == 0x03 and self._body_fn is not None:
            return self._body_fn
        return super()._select_runner()


class EswFanServiceV3(Service):
    """리모컨 주체 상태기 + STATUS 에코백/스냅샷 (docstring 1~4).

    v2 서비스를 상속하지 않고 새로 정의한다 — bluez_peripheral의 characteristic
    데코레이터는 클래스 정의에 묶여 있어 setter만 부분 교체할 수 없기 때문
    (v2가 v1 서비스를 새로 쓴 것과 같은 이유).
    """

    _CHAR_NO = {"power": 1, "mode": 2, "wind": 3}   # 에코백의 Characteristic 번호

    def __init__(self, supervisor: _ModeSupervisor, fan):
        super().__init__(SERVICE_UUID, True)
        self._supervisor = supervisor
        self._fan = fan
        self._loop: asyncio.AbstractEventLoop | None = None  # attach_loop가 채움
        self._power_on = False   # 시작 시 전원 OFF 가정 (앱이 POWER ON을 먼저 보냄)
        self._mode = 0x00
        self._effective_mode = 0x00
        self._level = 0                                  # 마지막 수신 공용 세기
        self._body_levels = {0x01: 0, 0x02: 1, 0x03: 0}  # 부위 러너용 — 상체 ≥1

    # ── 상태 적용/통지 ───────────────────────────────────────────────────────

    def _gated_level(self) -> int:
        """게이팅만 한다 — 전원 OFF·부위 모드(러너 소유, 그 전까진 정지)면 0.
        세기 자체는 바꾸지 않는다 (docstring 1)."""
        return self._level if (self._power_on and self._mode != 0x03) else 0

    def _apply_state(self) -> int:
        """현재 상태를 supervisor와 릴레이에 적용한다 — supervisor 먼저
        (이전 러너 join 완료 후 릴레이, docstring 7)."""
        self._supervisor.set_state(self._power_on, self._mode, self._level)
        if self._power_on and self._mode == 0x03:
            return 0  # 부위 모드 릴레이는 러너 소유 — 서비스는 안 건드린다
        # 부위 러너가 아니면 유효 모드 = 요청 모드. join 뒤에 재확정하므로
        # 죽어가는 러너의 마지막 report_effective와 레이스하지 않는다.
        self._set_effective(self._mode)
        level = self._gated_level()
        self._fan.set_speed(level)
        return level

    def _snapshot(self) -> bytes:
        return bytes([0x04, int(self._power_on), self._mode, self._effective_mode,
                      self._level, self._body_levels[0x01],
                      self._body_levels[0x02], self._body_levels[0x03]])

    def _notify(self, payload: bytes) -> None:
        """status notify 송신 — 구독자가 없으면 BlueZ가 버린다.
        본 동작(모터/릴레이)에 영향을 주지 않게 실패는 로그만 남긴다."""
        try:
            self.status.changed(payload)
        except Exception as e:
            print(f"[BLE] status notify 실패: {e}")

    def _echo(self, name: str, value) -> None:
        self._notify(bytes([0x01, self._CHAR_NO[name], *value]))

    # ── 러너 보고 통로 (docstring 6) ─────────────────────────────────────────

    def attach_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """BLE asyncio 루프 연결 — _ble_main_v3가 시작 시 1회 호출."""
        self._loop = loop

    def _notify_threadsafe(self, payload: bytes) -> None:
        """어느 스레드에서든 안전한 notify — dbus_fast는 BLE 루프 전용이라
        call_soon_threadsafe로 넘긴다 (BLE 루프 자신이 불러도 안전)."""
        loop = self._loop
        if loop is None or loop.is_closed():
            return
        loop.call_soon_threadsafe(self._notify, payload)

    @property
    def body_levels(self) -> dict:
        """부위별 저장 세기 — 부위 러너가 매 프레임 읽는다 (int 읽기 원자적)."""
        return self._body_levels

    @property
    def common_level(self) -> int:
        """공용 세기 — 부위 러너의 추적 폴백이 읽는다 (추적 모드와 동일 풍속)."""
        return self._level

    def _set_effective(self, mode: int) -> None:
        if mode == self._effective_mode:
            return
        self._effective_mode = mode
        self._notify_threadsafe(bytes([0x03, mode]))

    def report_effective(self, mode: int) -> None:
        """부위 러너의 상 전환 보고 → 유효 모드 push [0x03, 모드]."""
        self._set_effective(mode)

    def report_recognized(self, recognized: bool) -> None:
        """부위 러너의 객체 인식 변화 보고 → [0x02, 0/1] (변화 시에만 호출됨)."""
        self._notify_threadsafe(bytes([0x02, int(recognized)]))

    # ── Characteristics ──────────────────────────────────────────────────────
    # write-only 특성도 getter 자리(placeholder)가 필요 — 읽기 시도 시에만 쓰인다.

    @characteristic(POWER_UUID, CharFlags.WRITE)
    def power(self, options):
        raise NotImplementedError()

    @power.setter
    def power(self, value, options):
        if len(value) != 1 or value[0] not in (0x00, 0x01):
            print(f"[RX] 전원: 잘못된 값 ({_hex(value)}) — 거부, 에코 없음")
            return
        self._power_on = bool(value[0])
        # 에코는 적용 전에 보낸다(수신·수락 확인) — _apply_state의 러너 전환은
        # 이전 스레드 join으로 수 초 걸릴 수 있어, 뒤에 보내면 앱의 에코
        # 타임아웃이 모드/전원 전환마다 오탐한다. mode/wind setter도 동일.
        self._echo("power", value)
        level = self._apply_state()
        print(f"[RX] 전원: {'ON' if self._power_on else 'OFF'} → 풍속 {level}단")

    @characteristic(MODE_UUID, CharFlags.WRITE)
    def mode(self, options):
        raise NotImplementedError()

    @mode.setter
    def mode(self, value, options):
        if len(value) != 1 or value[0] not in MODE_NAMES:
            print(f"[RX] 모드: 잘못된 값 ({_hex(value)}) — 거부, 에코 없음")
            return
        self._mode = value[0]         # 유효 모드는 _apply_state/부위 러너가 갱신
        self._echo("mode", value)     # 적용 전 송신 (power setter 주석 참고)
        level = self._apply_state()   # 세기 복원 없음 — 게이팅 재평가만 (docstring 1)
        print(f"[RX] 모드: {MODE_NAMES[value[0]]} → 풍속 {level}단 유지")

    @characteristic(WIND_UUID, CharFlags.WRITE)
    def wind(self, options):
        raise NotImplementedError()

    @wind.setter
    def wind(self, value, options):
        if len(value) != 2 or value[0] not in WIND_TARGETS or not 0 <= value[1] <= 3:
            print(f"[RX] 풍량: 잘못된 값 ({_hex(value)}) — 거부, 에코 없음")
            return
        if value[0] == 0x02 and value[1] == 0x00:
            print("[RX] 풍량: 상체는 정지 불가 (1~3단만) — 거부, 에코 없음")
            return
        target, level = value[0], value[1]
        if target == 0x00:
            self._level = level
        else:
            self._body_levels[target] = level
        self._echo("wind", value)     # 적용 전 송신 (power setter 주석 참고)
        applied = self._apply_state()
        level_txt = "정지" if level == 0 else f"{level}단"
        print(f"[RX] 풍량: {WIND_TARGETS[target]} {level_txt} → 현재 적용 {applied}단")

    @characteristic(STATUS_UUID, CharFlags.READ | CharFlags.NOTIFY)
    def status(self, options):
        return self._snapshot()   # read = 스냅샷 (앱의 불일치 검증용 — docstring 2)

    def handle_disconnect(self) -> None:
        """앱(중앙) 연결 끊김 = 전원 OFF 처리 (v2 docstring 9와 동일 계약 —
        끊기면 양쪽 다 OFF로 수렴하므로 끊긴 동안의 불일치가 성립하지 않는다)."""
        if not self._power_on:
            return
        print("[BLE] 연결 끊김 → 전원 OFF 처리 (풍속 정지 + 0°,0° 파킹)")
        self._power_on = False
        self._apply_state()


async def _ble_main_v3(service: EswFanServiceV3) -> None:
    """v2 _ble_main 앞에 BLE 루프 연결만 추가 (러너 → notify 통로, docstring 6)."""
    service.attach_loop(asyncio.get_running_loop())
    await _ble_main(service)


def main() -> None:
    p = argparse.ArgumentParser(description="BLE 추적/풍속 + STATUS 동기화 (v3)")
    p.add_argument("--axis", choices=("pan", "tilt", "pantilt"), required=True,
                   help="타겟 모드일 때 돌릴 추적 축")
    p.add_argument("--model", default="multipose_lightning.tflite")
    p.add_argument("--conf", type=float, default=0.25, help="키포인트 신뢰도 임계값")
    p.add_argument("--threads", type=int, default=3, help="TFLite 스레드 수")
    # ── 축별 튜닝 (axis에 따라 일부만 실제로 쓰임 — verify_track_*.py 참고) ────
    p.add_argument("--gain", type=float, default=0.3)
    p.add_argument("--gain-tilt", type=float, default=0.2)
    p.add_argument("--deadzone", type=float, default=1.0)
    p.add_argument("--deadzone-tilt", type=float, default=0.5)
    p.add_argument("--target-cx", type=float, default=0.5)
    p.add_argument("--target-cy", type=float, default=0.5)
    p.add_argument("--limit", type=float, default=100.0, help="--axis pan 전용 소프트 클램프 ±°")
    lim = CFG["limits"]
    p.add_argument("--pan-min", type=float, default=lim["pan"]["min"])
    p.add_argument("--pan-max", type=float, default=lim["pan"]["max"])
    p.add_argument("--tilt-min", type=float, default=lim["tilt"]["min"])
    p.add_argument("--tilt-max", type=float, default=lim["tilt"]["max"])
    p.add_argument("--invert", action="store_true")
    p.add_argument("--invert-tilt", action="store_true")
    p.add_argument("--region", choices=("chest", "head", "upper", "lower"), default="chest",
                   help="--axis tilt 전용 조준 부위")
    # ── 기본-회전 모드 (0x01) 스윕 (v2 docstring 7) ──────────────────────────
    p.add_argument("--rotate-span", type=float, default=60.0,
                   help="회전 모드 pan 스윕 반각 — 0° 기준 ±°")
    p.add_argument("--rotate-speed", type=float, default=20.0,
                   help="회전 모드 스윕 속도 (°/s)")
    # ── 부위 모드 (0x03) — docstring 5. 세부 시나리오 파라미터(수렴/탐색 등)는
    #    verify_fulltrack과 같은 기본값을 쓴다 (control/fullbody_scenario.py) ──
    p.add_argument("--body-dwell", type=float, default=2.0,
                   help="부위 순찰 체류 시간 (s)")
    p.add_argument("--body-exit-deg", type=float, default=12.0,
                   help="순찰 중 이동 판정 팬 이동량 (° — 추적 폴백 전환)")
    p.add_argument("--body-exit-window", type=float, default=3.0,
                   help="이동 판정 시간창 (s)")
    p.add_argument("--body-still-s", type=float, default=5.0,
                   help="폴백 중 순찰 재진입 정지 시간 (s)")
    p.add_argument("--body-still-deg", type=float, default=3.0,
                   help="정지 판정 팬 각 범위 (°)")
    p.add_argument("--body-head-bias", type=float, default=11.0,
                   help="머리 조준 상향 편향 (° — 부위 간 틸트 간격 확대)")
    p.add_argument("--body-upper-bias", type=float, default=0.5,
                   help="상체 조준 하향 편향 (° — 머리/상체 구분 확대)")
    p.add_argument("--body-tilt-spread", type=float, default=3.0,
                   help="부위별 틸트 구간 분리 폭 (° — 아래쪽 리밋은 하체 전용, "
                        "상체는 그보다 이만큼 위에서 멈춘다. 0이면 분리 없음)")
    # 미세 움직임 둔감화 (실기 2026-08-27: 작은 흔들림에 순찰이 자주 끊김).
    # 시나리오 자체 이동 감지는 게이트(--body-exit-deg)보다 위에 둬야 게이트가
    # 1차 판정자가 된다 — 낮으면 recenter가 먼저 걸려 폴백이 안 나온다.
    p.add_argument("--body-move-thr", type=float, default=20.0,
                   help="시나리오 재조준 트리거 가슴 오차 (° — 게이트보다 크게)")
    p.add_argument("--body-rescan-thr", type=float, default=30.0,
                   help="재조준 후 전신 재스캔 판정 이동량 (°)")
    p.add_argument("--body-deadzone", type=float, default=2.0,
                   help="부위 모드 팬 데드존 (° — 미세 흔들림 억제)")
    p.add_argument("--body-deadzone-tilt", type=float, default=1.0,
                   help="부위 모드 틸트 데드존 (°)")
    p.add_argument("--body-converge", type=float, default=2.5,
                   help="수렴 판정 오차 (° — 순찰 도착·재조준·탐색 공용)")
    p.add_argument("--body-map-timeout", type=float, default=1.5,
                   help="매핑에서 부위가 안 보일 때 추정으로 넘어가는 시간 (s)")
    # ── 카메라 백엔드 (verify_movenet과 동일) ────────────────────────────────
    p.add_argument("--opencv", action="store_true")
    p.add_argument("--rpicam", action="store_true", help="rpicam-vid 서브프로세스 캡처")
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--no-window", action="store_true")
    p.add_argument("--dry-run", action="store_true",
                   help="모터/릴레이/lgpio 없이 각도 계산·풍속 게이팅만 (개발 PC 검증용)")
    add_state_args(p)
    # ── 웹 스트림 ────────────────────────────────────────────────────────────
    p.add_argument("--web", action="store_true", help="MJPEG 웹 스트림 송출")
    p.add_argument("--web-host", default="0.0.0.0")
    p.add_argument("--web-port", type=int, default=8090)
    p.add_argument("--web-quality", type=int, default=75)
    p.add_argument("--web-fps", type=float, default=20.0)
    args = p.parse_args()

    # 창을 원했는지 기억해두고, 추적 세션 스레드에서는 imshow가 절대 안 불리게
    # no_window를 강제한다. 창은 _window_viewer 전용 스레드가 대신 그린다
    # (v1 docstring 참고 — 프레임 통로로 web_state를 재사용).
    local_window = not args.no_window
    serve_http = args.web
    args.no_window = True
    if local_window:
        args.web = True

    if not Path(args.model).exists():
        print(f"[ERROR] 모델 없음: {args.model}")
        sys.exit(1)
    if args.tilt_min >= args.tilt_max:
        print("[ERROR] --tilt-min은 --tilt-max보다 작아야 합니다")
        sys.exit(1)
    if args.axis == "pantilt" and args.pan_min >= args.pan_max:
        print("[ERROR] --pan-min은 --pan-max보다 작아야 합니다")
        sys.exit(1)
    if not 0 < args.rotate_span <= min(lim["pan"]["max"], -lim["pan"]["min"]):
        print("[ERROR] --rotate-span은 0보다 크고 pan 회전 한계 안이어야 합니다")
        sys.exit(1)
    if args.rotate_speed <= 0:
        print("[ERROR] --rotate-speed는 0보다 커야 합니다")
        sys.exit(1)
    if (args.body_dwell <= 0 or args.body_exit_deg <= 0 or args.body_exit_window <= 0
            or args.body_still_s <= 0 or args.body_still_deg <= 0
            or args.body_head_bias < 0 or args.body_upper_bias < 0
            or args.body_tilt_spread < 0
            or args.body_move_thr <= 0 or args.body_rescan_thr <= 0
            or args.body_deadzone <= 0 or args.body_deadzone_tilt <= 0
            or args.body_converge <= 0 or args.body_map_timeout <= 0):
        print("[ERROR] --body-* 인자 범위가 잘못됐습니다 (help 참고)")
        sys.exit(1)
    if args.body_move_thr <= args.body_exit_deg:
        # 시나리오 recenter가 먼저 걸리면 게이트 창이 비워져 폴백이 안 나온다.
        print(f"[WARN] --body-move-thr({args.body_move_thr:g}°)가 "
              f"--body-exit-deg({args.body_exit_deg:g}°) 이하 — 재조준이 먼저 걸려 "
              "추적 폴백이 잘 안 나올 수 있습니다.")

    detector = MoveNetMultiPoseDetector(args.model, conf_thr=args.conf, num_threads=args.threads)
    tracker = PoseTracker()

    web_srv = web_state = viewer_thread = None
    viewer_stop = threading.Event()
    if args.web:
        web_state = _WebStreamState(args.web_quality, args.web_fps)
        if serve_http:
            web_srv = _ThreadedHTTP((args.web_host, args.web_port), _make_handler(web_state))
            threading.Thread(target=web_srv.serve_forever, daemon=True).start()
            print(f"[web] http://{args.web_host}:{args.web_port}/  (브라우저에서 열기)")
        if local_window:
            viewer_thread = threading.Thread(target=_window_viewer,
                                             args=(web_state, viewer_stop), daemon=True)
            viewer_thread.start()
            print("[E2E] 로컬 창 뷰어 시작 (전용 GUI 스레드)")

    motor_cm = open_motor_from_args(args)
    try:
        with motor_cm as mc:
            mc.enable()
            # 이전 실행이 돌아간 채 꺼졌으면 저장 위치만큼 되돌아와 중앙을 본다.
            mc.restore_origin()

            if args.dry_run:
                fan_cm = _DryRelay()
            else:
                # lgpio를 최상단에서 import하는 모듈이라 지연 import (v2 docstring 5)
                from hardware.relay_controller import FanRelay
                fan_cm = FanRelay(CFG, handle=mc.h)  # gpiochip 핸들 공유 (v2 docstring 4)

            with fan_cm as fan:  # mc보다 먼저 닫힘 — 공유 핸들이 살아있을 때 전부 오픈
                # v1 _make_runner가 --axis tilt에서 args.gain을 덮어쓰므로
                # 부위 러너용 게인은 그 전에 캡처한다 (_make_body_runner 참고).
                body_gains = (args.gain, args.gain_tilt)
                track_fn = _make_runner(args.axis, detector, tracker, mc, args, web_state)
                supervisor = _ModeSupervisorV3(track_fn, _make_sweeper(mc, args),
                                               _make_homer(mc, home_pan=False),
                                               _make_homer(mc, home_pan=True),
                                               stop_fn=mc.stop,
                                               web_state=web_state)
                service = EswFanServiceV3(supervisor, fan)
                # 부위 러너는 서비스의 report_*를 쓰므로 서비스 다음에 만든다.
                supervisor.set_body_runner(_make_body_runner(
                    detector, tracker, mc, fan, service, body_gains, args, web_state))

                print(f"[E2E] axis={args.axis} — BLE 전원/모드/풍량 명령 대기 중 (v3).")
                try:
                    asyncio.run(_ble_main_v3(service))
                except KeyboardInterrupt:
                    print("\n[E2E] Ctrl+C 종료")
                finally:
                    fan.set_speed(0)  # 0° 복귀(최대 30s) 동안 팬이 계속 돌지 않게 먼저 정지
                    supervisor.stop_and_join()
                    if args.axis in ("tilt", "pantilt"):
                        # 웜기어(틸트)는 수동 복귀가 어려우므로 종료 시 0°로 되돌린다.
                        print("[E2E] 0° 복귀 중...")
                        mc.move_to(0.0, 0.0)
                        if not mc.wait_until_idle(timeout=30):
                            print("[E2E] 복귀 미완료 — 물리 위치를 확인하세요.")
    finally:
        if web_srv:
            web_srv.shutdown(); web_srv.server_close()
        viewer_stop.set()
        if viewer_thread:
            viewer_thread.join(timeout=3)  # destroyAllWindows는 뷰어 스레드 몫
        print("\n[E2E] 종료")


if __name__ == "__main__":
    main()
