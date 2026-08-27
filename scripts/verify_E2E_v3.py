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
  4. 유효 모드(_effective_mode) 도입 — 이 단계에서는 항상 요청 모드와 같다.
     부위 러너가 이동 감지로 0x03→0x02로 내리면 그때부터 갈라진다.

실행 (RPi 5, 레포 루트에서):
    python3 scripts/verify_E2E_v3.py --axis pan
    python3 scripts/verify_E2E_v3.py --axis pantilt --rpicam --no-window
    python scripts/verify_E2E_v3.py --axis pan --dry-run --opencv   # 개발 PC
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import threading
from pathlib import Path

# 레포 루트 + scripts 디렉터리를 path에 추가 (config/vision/control + v1·v2 재사용)
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

from bluez_peripheral.gatt import CharacteristicFlags as CharFlags
from bluez_peripheral.gatt import Service, characteristic

from config import CFG
from vision.pose_estimator import MoveNetMultiPoseDetector
from vision.pose_tracker import PoseTracker
from tracking_core import add_state_args, open_motor_from_args
from verify_movenet import _WebStreamState, _make_handler, _ThreadedHTTP
from verify_E2E_v1 import (SERVICE_UUID, POWER_UUID, MODE_UUID, WIND_UUID,
                           STATUS_UUID, MODE_NAMES, WIND_TARGETS,
                           _hex, _make_runner, _window_viewer)
from verify_E2E_v2 import (_DryRelay, _make_sweeper, _make_homer,
                           _ModeSupervisor, _ble_main)


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
        """현재 상태를 릴레이와 모터 supervisor에 함께 적용한다."""
        level = self._gated_level()
        self._fan.set_speed(level)
        self._supervisor.set_state(self._power_on, self._mode, self._level)
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
        self._mode = self._effective_mode = value[0]
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
                track_fn = _make_runner(args.axis, detector, tracker, mc, args, web_state)
                supervisor = _ModeSupervisor(track_fn, _make_sweeper(mc, args),
                                             _make_homer(mc, home_pan=False),
                                             _make_homer(mc, home_pan=True),
                                             stop_fn=mc.stop,
                                             web_state=web_state)
                service = EswFanServiceV3(supervisor, fan)

                print(f"[E2E] axis={args.axis} — BLE 전원/모드/풍량 명령 대기 중 (v3).")
                try:
                    asyncio.run(_ble_main(service))
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
