#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/verify_E2E_v1.py - BLE 타겟 모드 → 팬/틸트 추적 루프 연동 (v1)

앱에서 "타겟 모드"(모드 write 0x02/0x03)를 켜면 --axis로 지정한 축의
추적 루프(tracking_core.run_*_tracking, verify_track_pan/tilt/pantilt.py와
동일 로직)를 백그라운드 스레드로 시작하고, 전원을 끄거나 기본 모드로 돌아가면
정지한다. 하드웨어가 아직 완성 전이라 만든 최소 통합(v1) — 부위 인식 모드
(0x03)의 머리/상체/하체 개별 조준은 다루지 않는다(그건 verify_fulltrack.py의
전신 시나리오 몫). 풍량 write도 print만 한다(릴레이/팬속도 하드웨어 미완성).

카메라는 모터/디텍터와 달리 프로세스 전체가 아니라 **추적 세션(스레드)마다
새로 열고 끝나면 반드시 해제**한다 — 특히 `--rpicam`(rpicam-vid 서브프로세스)
백엔드는 아무도 안 읽는 동안 파이프가 막혀 캡처 자체가 멎어버리는 게 실기로
확인됐다(기본 모드로 쉬다가 타겟 모드로 돌아오면 화면이 멈춘 채 그대로).

설치: verify_ble.py, verify_track_pan/tilt/pantilt.py의 설치 안내를 합친 것과 동일
    (bluez_peripheral --pre, tflite-runtime, opencv-python 등).

실행 (RPi 5, 레포 루트에서):
    python3 scripts/verify_E2E_v1.py --axis pan
    python3 scripts/verify_E2E_v1.py --axis tilt --rpicam
    python3 scripts/verify_E2E_v1.py --axis pantilt --rpicam --no-window
    python scripts/verify_E2E_v1.py --axis pan --dry-run --opencv   # 개발 PC, 모터/BLE 없이 파이프라인만

축별 튜닝 인자는 verify_track_pan/tilt/pantilt.py와 이름이 같다
(--gain/--deadzone/--target-cx는 pan·pantilt, --gain-tilt/--deadzone-tilt/
--target-cy/--tilt-min/--tilt-max는 tilt·pantilt, --region은 tilt 전용,
--limit은 pan 전용, --pan-min/--pan-max는 pantilt 전용). 고르지 않은 축의
인자는 무시된다 — 정확한 의미는 각 verify_track_*.py 참고.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import threading
import time
from pathlib import Path

import cv2

# 레포 루트 + scripts 디렉터리를 path에 추가 (config/vision/control + 카메라 백엔드 재사용)
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

from bluez_peripheral.adapter import Adapter
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.gatt import CharacteristicFlags as CharFlags
from bluez_peripheral.gatt import Service, characteristic
from bluez_peripheral.util import get_message_bus

from config import CFG
from vision.pose_estimator import MoveNetMultiPoseDetector
from vision.pose_tracker import PoseTracker
from tracking_core import (_open_motor, run_pan_tracking, run_tilt_tracking,
                           run_pantilt_tracking)
from verify_movenet import (_open_camera, _release_camera,
                            _WebStreamState, _make_handler, _ThreadedHTTP)

# ── 프로토콜 (ble_protocol.md / verify_ble.py와 일치해야 함) ──
UUID_BASE = "14d7{:04x}-7197-49e5-a017-0b2f308120f0"
SERVICE_UUID = UUID_BASE.format(0x0001)
POWER_UUID = UUID_BASE.format(0x0002)
MODE_UUID = UUID_BASE.format(0x0003)
WIND_UUID = UUID_BASE.format(0x0004)
STATUS_UUID = UUID_BASE.format(0x0005)

LOCAL_NAME = "ESW-FAN"

MODE_NAMES = {0x00: "기본-고정", 0x01: "기본-회전", 0x02: "타겟", 0x03: "타겟-부위"}
WIND_TARGETS = {0x00: "공용", 0x01: "머리", 0x02: "상체", 0x03: "하체"}
_TARGET_MODES = (0x02, 0x03)  # v1은 타겟/타겟-부위를 동일하게 취급


def _hex(value):
    return " ".join(f"0x{b:02X}" for b in value)


def _make_runner(axis, detector, tracker, mc, args, web_state):
    """axis에 맞는 tracking_core 루프를 stop_event 하나만 받는 콜러블로 감싼다.

    카메라는 세션(스레드)마다 새로 열고 끝나면 반드시 해제한다 — 특히
    `--rpicam`(rpicam-vid 서브프로세스) 백엔드는 아무도 읽지 않는 동안 파이프
    버퍼가 차서 캡처 자체가 멎어버리는 게 실기로 확인됨. "기본 모드"로 쉬는
    동안 카메라를 계속 열어두면 재진입 시 그 멎은 상태(화면 정지) 그대로
    남는다 — 매번 새로 열면 이 문제를 피한다.
    """
    def _open_cam():
        """카메라를 연다. 방금 release한 직후 재오픈하면 커널/libcamera가 이전
        세션의 자원을 아직 다 안 놓아준 상태라 실패할 수 있어 잠깐 재시도한다.

        `_open_camera`는 rpicam→picamera2→OpenCV까지 전부 실패하면
        `sys.exit(1)`을 부르는데, 지금은 메인 스레드가 아니라 이 추적
        스레드 안에서 실행되므로 그 SystemExit는 프로세스 전체가 아니라
        이 스레드만 조용히 죽인다 — 화면도 안 뜨고 에러도 안 보이는 것처럼
        보이는 원인이었다. 여기서 명시적으로 잡아서 재시도/로그를 남긴다.
        """
        for attempt in range(1, 4):
            try:
                return _open_camera(args.opencv, args.cam, use_rpicam=args.rpicam)
            except SystemExit:
                print(f"[E2E] 카메라 오픈 실패 ({attempt}/3) — 0.5s 후 재시도")
                time.sleep(0.5)
        raise RuntimeError("[E2E] 카메라를 열 수 없습니다 (재시도 소진)")

    def _release_cam(cam, backend):
        _release_camera(cam, backend)
        if not args.no_window:
            cv2.destroyAllWindows()
        # 다음 세션이 바로 재오픈해도 커널/libcamera가 자원을 다 놓을 시간을 준다.
        time.sleep(0.3)

    if axis == "pan":
        fov_h = CFG["fov"]["h"]
        sign = -1.0 if args.invert else 1.0

        def _run(stop_event):
            cam, backend = _open_cam()
            try:
                run_pan_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                                 fov_h, sign, web_state=web_state)
            finally:
                _release_cam(cam, backend)

        return _run

    if axis == "tilt":
        fov_v = CFG["fov"]["v"]
        sign = -1.0 if args.invert else 1.0
        aim_key = "upper" if args.region == "chest" else args.region

        def _run(stop_event):
            cam, backend = _open_cam()
            try:
                run_tilt_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                                  fov_v, sign, aim_key, web_state=web_state)
            finally:
                _release_cam(cam, backend)

        return _run

    fov_h, fov_v = CFG["fov"]["h"], CFG["fov"]["v"]
    sign_pan = -1.0 if args.invert else 1.0
    sign_tilt = -1.0 if args.invert_tilt else 1.0

    def _run(stop_event):
        cam, backend = _open_cam()
        try:
            run_pantilt_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                                 fov_h, fov_v, sign_pan, sign_tilt, web_state=web_state)
        finally:
            _release_cam(cam, backend)

    return _run


class _TrackingSupervisor:
    """BLE 전원/모드 write에 맞춰 추적 루프를 백그라운드 스레드로 시작/정지한다.

    전원 OFF든 기본 모드 복귀든 즉시 정지 신호(stop_event.set())를 보내고,
    새로 시작할 때만 이전 스레드가 완전히 끝났는지 join으로 확인한다
    (정지 자체는 non-blocking — BLE 콜백을 오래 막지 않음).
    """

    def __init__(self, run_fn) -> None:
        self._run_fn = run_fn
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._power_on = False
        self._mode = 0x00

    def _target_active(self) -> bool:
        return self._power_on and self._mode in _TARGET_MODES

    def _apply(self) -> None:
        if self._target_active():
            # 직전 스레드가 (정지 요청을 받았지만 루프 맨 위 체크 지점까지 아직
            # 못 돌아와) 살아있을 수 있다 — "이미 실행 중"으로 오인해 조용히
            # 리턴하면 이후 시작 요청이 영영 씹히므로, 반드시 정지시키고 join한
            # 뒤 새로 시작한다 (기존엔 여기서 is_alive()면 그냥 return 해버려서
            # 레이스가 나면 추적이 재개 안 되는 버그가 있었다).
            if self._thread and self._thread.is_alive():
                print("[E2E] 이전 추적 스레드 정리 중...")
                self._stop_event.set()
                self._thread.join(timeout=5)
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run_fn,
                                            args=(self._stop_event,), daemon=True)
            self._thread.start()
            print("[E2E] 추적 시작")
        else:
            self._stop_event.set()
            print("[E2E] 추적 정지 요청 (백그라운드에서 곧 멈춤)")

    def set_power(self, on: bool) -> None:
        self._power_on = on
        self._apply()

    def set_mode(self, mode: int) -> None:
        self._mode = mode
        self._apply()

    def stop_and_join(self, timeout: float = 10) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=timeout)


class EswFanService(Service):
    def __init__(self, supervisor: _TrackingSupervisor):
        super().__init__(SERVICE_UUID, True)
        self._supervisor = supervisor

    @characteristic(POWER_UUID, CharFlags.WRITE)
    def power(self, options):
        raise NotImplementedError()

    @power.setter
    def power(self, value, options):
        if len(value) != 1 or value[0] not in (0x00, 0x01):
            print(f"[RX] 전원: 잘못된 값 ({_hex(value)})")
            return
        on = bool(value[0])
        print(f"[RX] 전원: {'ON' if on else 'OFF'}")
        self._supervisor.set_power(on)

    @characteristic(MODE_UUID, CharFlags.WRITE)
    def mode(self, options):
        raise NotImplementedError()

    @mode.setter
    def mode(self, value, options):
        if len(value) != 1 or value[0] not in MODE_NAMES:
            print(f"[RX] 모드: 잘못된 값 ({_hex(value)})")
            return
        print(f"[RX] 모드: {MODE_NAMES[value[0]]}")
        self._supervisor.set_mode(value[0])

    @characteristic(WIND_UUID, CharFlags.WRITE)
    def wind(self, options):
        raise NotImplementedError()

    @wind.setter
    def wind(self, value, options):
        if len(value) != 2 or value[0] not in WIND_TARGETS or not 1 <= value[1] <= 3:
            print(f"[RX] 풍량: 잘못된 값 ({_hex(value)})")
            return
        print(f"[RX] 풍량: {WIND_TARGETS[value[0]]} {value[1]}단")
        # v1 범위 밖 — 릴레이/팬속도 하드웨어 미완성이라 실제 제어는 없음.

    # 상태(notify)는 4단계에서 에코백 구현 — 지금은 서비스 발견용으로만 등록.
    @characteristic(STATUS_UUID, CharFlags.NOTIFY)
    def status(self, options):
        raise NotImplementedError()


async def _ble_main(supervisor: _TrackingSupervisor) -> None:
    bus = await get_message_bus()

    service = EswFanService(supervisor)
    await service.register(bus)

    try:
        adapter = await Adapter.get_first(bus)
    except ValueError:
        sys.exit("BLE 어댑터 없음 — 'bluetoothctl power on' 확인")

    advert = Advertisement(LOCAL_NAME, [SERVICE_UUID], appearance=0x0000, timeout=0)
    await advert.register(bus, adapter=adapter)

    print(f"[BLE] Advertising 시작 — {LOCAL_NAME} (Ctrl+C 종료)")
    await bus.wait_for_disconnect()


def main() -> None:
    p = argparse.ArgumentParser(description="BLE 타겟 모드 → 팬/틸트 추적 루프 연동 (v1)")
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
    if args.axis == "pantilt" and args.pan_min >= args.pan_max:
        print("[ERROR] --pan-min은 --pan-max보다 작아야 합니다")
        sys.exit(1)

    detector = MoveNetMultiPoseDetector(args.model, conf_thr=args.conf, num_threads=args.threads)
    tracker = PoseTracker()

    web_srv = web_state = None
    if args.web:
        web_state = _WebStreamState(args.web_quality, args.web_fps)
        web_srv = _ThreadedHTTP((args.web_host, args.web_port), _make_handler(web_state))
        threading.Thread(target=web_srv.serve_forever, daemon=True).start()
        print(f"[web] http://{args.web_host}:{args.web_port}/  (브라우저에서 열기)")

    motor_cm = _open_motor(args.dry_run)
    try:
        with motor_cm as mc:
            mc.enable()
            mc.home()  # 현재 물리 위치 = 0°(각 축). 시작 위치에 마커를 붙여두면 대조에 편함
            print("[motor] 현재 위치를 0°로 설정 (임시 호밍).")

            run_fn = _make_runner(args.axis, detector, tracker, mc, args, web_state)
            supervisor = _TrackingSupervisor(run_fn)

            print(f"[E2E] axis={args.axis} — BLE 타겟 모드(0x02/0x03) 명령 대기 중.")
            try:
                asyncio.run(_ble_main(supervisor))
            except KeyboardInterrupt:
                print("\n[E2E] Ctrl+C 종료")
            finally:
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
        cv2.destroyAllWindows()
        print("\n[E2E] 종료")


if __name__ == "__main__":
    main()
