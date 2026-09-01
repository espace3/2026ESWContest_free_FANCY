#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
app/ble_service.py - 모드 감독 · 풍속 릴레이 연동 · 전원 게이팅
(개발 이력상 verify_E2E_v2.py)

app/ble_protocol.py(추적 연동 원형)와의 차이점만 기록합니다 — 카메라 세션 관리,
cv2 창 전용 스레드, 축별 튜닝 인자, 설치/실행 방법은 원형 상단 docstring과
동일하므로 반복하지 않습니다:

  1. FanRelay(hardware/relay_controller.py) 연결 — v1에서 print에 그치던
     WIND write가 실제 릴레이(TS0011)를 구동한다 (verify_ble_v2.py와 동일).
     릴레이 간 전환 안전(break-before-make: 전부 오픈 → guard_s 대기 →
     하나만 닫기)은 FanRelay 워커가 전담하므로 여기서는 set_speed()만 부른다.
  2. 전원 게이팅 (verify_ble_v2와 동일) — 시작 시 전원 OFF 가정. POWER ON이면
     마지막 수신 세기를 재적용, OFF면 즉시 정지(전부 오픈). 전원 OFF 중
     WIND write는 저장만 하고 돌리지 않는다. 전원/모드에 따른 추적 시작·정지는
     v1의 _TrackingSupervisor 그대로.
     부위 모드(0x03)도 같은 규칙으로 게이팅한다 — 진입 시 풍속 0(정지),
     부위 모드 중 WIND write는 저장만, 이탈 시(전원 ON이면) 해당 모드의
     저장 세기를 재적용한다. 기본/타겟 풍량도 서로 분리해 보관한다.
     부위별 풍속 중재가 미구현이라(아래 6) 부위 모드에서는 아예 돌리지 않는
     게 안전하기 때문.
  3. WIND 세기 0x00 = 풍량 정지 허용 — 프로토콜 원문(ble_protocol.md)은
     세기 1~3만 정의하지만, "전원은 켠 채 바람만 끄기"를 대비해 미리 받는다
     (앱이 아직 안 보내는 값 — 앱에서 쓰기 시작하면 ble_protocol.md도 갱신).
  4. gpiochip 핸들 공유 — MotorController가 연 핸들(mc.h)을 FanRelay에 넘긴다
     (relay_controller.py의 "상위 통합부가 한 번만 열어 나눠 준다" 규약).
     따라서 FanRelay는 mc보다 먼저 닫혀야 한다 (with 중첩 순서가 그 보장).
  5. --dry-run이면 릴레이도 스텁(_DryRelay, print만)으로 대체 — 개발 PC에서
     lgpio 없이 파이프라인 확인 가능 (FanRelay import도 그래서 지연).
     6. WIND [대상, 세기]의 대상은 모드별 preset 저장에 사용한다 —
     부위별 풍향(머리/상체/하체 개별 조준)과 대상별 풍속 중재(프로토콜
     §3.3의 실제 적용)는 미구현으로 4단계 상위 통합부 몫.
  7. 기본-회전 모드(0x01) 구현 — v1은 타겟 모드에서만 스레드를 돌렸지만,
     v2는 회전 모드에서 pan을 원점(0°) 기준 ±(--rotate-span)° 왕복 스윕한다
     (tilt는 0° 유지, 카메라/디텍터 미사용). 각도는 move_to()의 절대각 =
     위치 장부(상태 파일로 영속) 기준이므로 "0,0 기준"이 자동으로 보장된다.
     move_to는 전체 이동을 한 번에 던지면 모터 프로파일 전속으로 돌므로,
     20Hz로 목표를 --rotate-speed(°/s)만큼씩 밀어 팬다운 속도로 돌리고
     정지 신호(모드 전환/전원 OFF)에 한 틱 안에 반응하게 했다.
     회전은 풍속과 연동 — 유효 풍속 0(WIND 세기 0 write)이면 스윕도 정지,
     1~3이 다시 오면 재개한다 (바람 없이 헤드만 도는 동작 방지). 경계
     0↔1~3에서만 시작/정지하고 1→2 같은 세기 변경으로는 재시작하지 않는다.
     타겟 모드는 연동하지 않는다 — 바람이 꺼져도 조준을 유지해야 재개 시
     즉시 맞는 상태가 되기 때문.
  8. 헤드 복귀 정책 — 기본-고정(0x00)과 회전 정지(0x01 + 풍속 0)에서는
     pan을 그 자리에 두고(복귀 없음) tilt만 0°로 되돌린 뒤 대기한다
     (웜기어 틸트만 중립 복귀, 좌우 방향은 멈춘 자리 존중). 전원 OFF는
     pan/tilt 모두 0°(원점)로 복귀(파킹)한 뒤 대기한다. 이 복귀들도 러너로
     돌린다 — supervisor의 stop→join→start 순서를 타므로 죽어가는 이전 추적
     스레드가 던지는 마지막 move_to와 레이스하지 않는다. 같은 러너가 정상
     구동 중이면 재시작하지 않는다(_apply 데드업) — 앱이 같은 모드를
     재전송하거나 타겟(0x02)↔타겟-부위(0x03)를 오가도 카메라 세션이 끊기지
     않는다 (v1은 매번 세션을 재시작했다).
  9. 연결 끊김 = 전원 OFF 처리 — 앱(중앙)의 BLE 연결이 끊기면 전원 OFF
     write와 동일하게 풍속 정지 + 0°,0° 파킹한다 (마지막 상태로 계속 도는
     것 방지). bluez_peripheral에는 연결 이벤트 API가 없어서 하부 dbus_fast
     버스로 BlueZ Device1.Connected PropertiesChanged를 직접 구독한다.
     어댑터에 연결된 아무 Device1의 끊김이든 트리거된다 — 이 기기는 앱 전용
     Peripheral이라 문제없지만, 다른 BLE 기기를 같이 물리면 오탐될 수 있다.
     재연결한 앱은 프로토콜 가정대로 POWER ON부터 다시 보내면 된다
     (Advertising은 timeout=0이라 BlueZ가 끊김 후 자동 재개).

실행 (RPi 5, 레포 루트에서):
    python3 app/ble_service.py --axis pan
    python3 app/ble_service.py --axis pantilt --rpicam --no-window
    python app/ble_service.py --axis pan --dry-run --opencv   # 개발 PC
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import threading
from pathlib import Path

# 레포 루트를 path에 추가 (config / vision / control / hardware / app 해결용)
sys.path.insert(0, str(Path(__file__).parent.parent))

from bluez_peripheral.adapter import Adapter
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.gatt import CharacteristicFlags as CharFlags
from bluez_peripheral.gatt import Service, characteristic
from bluez_peripheral.util import get_message_bus
from dbus_fast.constants import MessageType
from dbus_fast.message import Message

from config import CFG
from vision.pose_estimator import MoveNetMultiPoseDetector
from vision.pose_tracker import PoseTracker
from app.tracking import add_state_args, open_motor_from_args
from app.camera import _WebStreamState, _make_handler, _ThreadedHTTP
from app.ble_protocol import (SERVICE_UUID, POWER_UUID, MODE_UUID, WIND_UUID,
                           STATUS_UUID, LOCAL_NAME, MODE_NAMES, WIND_TARGETS,
                           _TARGET_MODES, _hex, _make_runner, _window_viewer,
                           _TrackingSupervisor)


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


def _make_sweeper(mc, args):
    """기본-회전 모드(0x01, 풍속 ≥1)용 러너 — pan을 원점(0°) 기준 ±span° 왕복,
    tilt는 0° 유지 (docstring 7).

    move_to는 '최신 목표만 유지' 논블로킹이라, 목표를 20Hz로 speed(°/s)만큼씩
    밀면 모터 프로파일 전속이 아니라 팬다운 속도로 돌고, stop_event에도 한 틱
    (50ms) 안에 반응한다 — 마지막 목표가 현 위치에서 한 틱 이내라 그 자리에
    감속 정지한다. 추적하다 span 밖(예: pan 80°)에서 진입하면 한 번에 span으로
    클램프하지 않고(전속 점프 방지) 같은 속도로 범위 안까지 걸어 들어온다.
    카메라/디텍터는 쓰지 않는다.
    """
    span, speed, period = args.rotate_span, args.rotate_speed, 0.05

    def _run(stop_event):
        target = mc.current_position()[0]
        direction = -1.0 if target > 0 else 1.0
        print(f"[E2E] 회전 스윕 시작 — pan ±{span:g}° (0° 기준, {speed:g}°/s), tilt 0°")
        while not stop_event.is_set():
            # span 밖에서 시작했으면 우선 범위 쪽으로 걷는다.
            if target > span:
                direction = -1.0
            elif target < -span:
                direction = 1.0
            target += direction * speed * period
            # 끝단 도달 시 반전 (범위 안에서 넘어선 경우에만).
            if direction > 0 and target >= span:
                target, direction = span, -1.0
            elif direction < 0 and target <= -span:
                target, direction = -span, 1.0
            mc.move_to(target, 0.0)
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


class _ModeSupervisor(_TrackingSupervisor):
    """v1 supervisor 확장 — 첫 BLE write 이후에는 상태마다 항상 러너 하나가 돈다.

    전원 ON: 타겟(0x02/0x03) → 추적, 기본-회전(0x01, 풍속 ≥1) → 스윕,
    기본-고정(0x00)·회전 정지(0x01 + 풍속 0) → 틸트만 복귀 후 대기.
    전원 OFF: pan/tilt 모두 0°로 복귀(파킹) 후 대기 (docstring 8).
    회전만 풍속과 연동한다 — 타겟 모드는 바람이 꺼져도 조준을 유지해야
    재개 시 즉시 맞는 상태가 되기 때문 (docstring 7).
    """

    def __init__(self, track_fn, sweep_fn, home_fn, park_fn, stop_fn=None,
                 web_state=None) -> None:
        super().__init__(track_fn, web_state=web_state)
        self._track_fn = track_fn
        self._sweep_fn = sweep_fn
        self._home_fn = home_fn
        self._park_fn = park_fn
        self._stop_fn = stop_fn
        self._wind_level = 0

    def _select_runner(self):
        if not self._power_on:
            return self._park_fn
        if self._mode in _TARGET_MODES:
            return self._track_fn
        if self._mode == 0x01 and self._wind_level > 0:
            return self._sweep_fn
        return self._home_fn

    def _target_active(self) -> bool:
        return True   # 어느 상태든 러너가 하나 돈다 (_select_runner가 고른다)

    def _apply(self) -> None:
        run_fn = self._select_runner()
        # 같은 러너가 정상 구동 중(정지 요청 없음)이면 재시작하지 않는다 —
        # 앱이 같은 모드를 재전송하거나 0x02↔0x03을 오갈 때, 풍속 1→2 변경 때
        # 카메라/스윕 세션이 불필요하게 끊기는 것 방지 (docstring 8).
        if (run_fn is self._run_fn and self._thread is not None
                and self._thread.is_alive() and not self._stop_event.is_set()):
            return
        if run_fn is not self._run_fn and self._thread is not None \
                and self._thread.is_alive():
            # 이전 runner가 다음 50ms tick까지 새 목표를 던지지 못하도록
            # 먼저 중단 신호와 모터 감속 정지를 전달한 뒤 join한다.
            self._stop_event.set()
            if self._stop_fn is not None:
                self._stop_fn()
        self._run_fn = run_fn
        super()._apply()

    def set_state(self, power_on: bool, mode: int, wind_level: int) -> None:
        """power/mode/wind를 한 상태로 갱신한 뒤 runner를 재선택한다."""
        self._power_on = power_on
        self._mode = mode
        self._wind_level = wind_level
        self._apply()

    def set_wind(self, level: int) -> None:
        """풍속 변화 통지 — 러너가 안 바뀌면 _apply 데드업이 무시한다."""
        self.set_state(self._power_on, self._mode, level)


class EswFanServiceV2(Service):
    """v1 서비스 + verify_ble_v2의 풍속 릴레이/전원 게이팅 결합.

    전원/모드 write는 두 갈래로 갈라진다: 모터 러너(supervisor)와 풍속 게이팅.
    풍량 write는 릴레이에 가고, 세기는 supervisor에도 통지한다(회전·풍속
    연동 — docstring 7). 릴레이가 실제로 도는 조건은 "전원 ON 그리고
    부위 모드 아님" 하나로 모았다(_apply_wind).
    """

    def __init__(self, supervisor: _ModeSupervisor, fan):
        super().__init__(SERVICE_UUID, True)
        self._supervisor = supervisor
        self._fan = fan
        self._power_on = False   # 시작 시 전원 OFF 가정 (앱이 POWER ON을 먼저 보냄)
        self._mode = 0x00
        self._basic_level = 0
        self._target_level = 0
        self._body_levels = {target: 0 for target in (0x01, 0x02, 0x03)}

    def _active_level(self) -> int:
        """현재 모드에서 사용할 저장 풍량을 반환한다."""
        if self._mode in (0x00, 0x01):
            return self._basic_level
        if self._mode == 0x02:
            return self._target_level
        return self._body_levels.get(0x01, 0)

    def _store_wind(self, target: int, level: int) -> None:
        """현재 모드와 대상에 맞는 풍량 preset을 갱신한다."""
        if self._mode in (0x00, 0x01):
            self._basic_level = level
        elif self._mode == 0x02 or target == 0x00:
            self._target_level = level
        else:
            self._body_levels[target] = level

    def _apply_state(self) -> int:
        """현재 BLE 상태를 릴레이와 모터 supervisor에 함께 적용한다."""
        stored_level = self._active_level()
        level = stored_level if (self._power_on and self._mode != 0x03) else 0
        self._fan.set_speed(level)
        self._supervisor.set_state(self._power_on, self._mode, stored_level)
        return level

    def _apply_wind(self) -> int:
        """게이트 상태대로 릴레이에 반영하고, 적용된 세기를 돌려준다.
        전원 OFF 또는 부위 모드(0x03)면 0(정지) — docstring 2."""
        return self._apply_state()

    # write-only 특성도 getter 자리(placeholder)가 필요 — 읽기 시도 시에만 쓰인다.
    @characteristic(POWER_UUID, CharFlags.WRITE)
    def power(self, options):
        raise NotImplementedError()

    @power.setter
    def power(self, value, options):
        if len(value) != 1 or value[0] not in (0x00, 0x01):
            print(f"[RX] 전원: 잘못된 값 ({_hex(value)})")
            return
        self._power_on = bool(value[0])
        level = self._apply_wind()
        print(f"[RX] 전원: {'ON' if self._power_on else 'OFF'} → 풍속 {level}단 적용")

    @characteristic(MODE_UUID, CharFlags.WRITE)
    def mode(self, options):
        raise NotImplementedError()

    @mode.setter
    def mode(self, value, options):
        if len(value) != 1 or value[0] not in MODE_NAMES:
            print(f"[RX] 모드: 잘못된 값 ({_hex(value)})")
            return
        self._mode = value[0]
        level = self._apply_wind()  # 부위 모드 진입 시 0, 이탈 시 저장 세기 재적용
        print(f"[RX] 모드: {MODE_NAMES[value[0]]} → 풍속 {level}단 적용")

    @characteristic(WIND_UUID, CharFlags.WRITE)
    def wind(self, options):
        raise NotImplementedError()

    @wind.setter
    def wind(self, value, options):
        # 세기 0x00(정지)은 프로토콜 원문(1~3)보다 앞서 받는 확장 — docstring 3.
        if len(value) != 2 or value[0] not in WIND_TARGETS or not 0 <= value[1] <= 3:
            print(f"[RX] 풍량: 잘못된 값 ({_hex(value)})")
            return
        self._store_wind(value[0], value[1])
        if not self._power_on:
            note = "전원 OFF — 저장만 (ON 시 적용)"
        elif self._mode == 0x03:
            note = "부위 모드 — 저장만 (모드 이탈 시 적용)"
        else:
            self._apply_wind()
            note = "릴레이 적용"
        applied = self._active_level() if self._mode != 0x03 else 0
        level_txt = "정지" if value[1] == 0 else f"{value[1]}단"
        # 대상별 preset은 저장하지만, 실제 부위별 풍향/풍속 중재는 아직
        # 4단계 상위 통합부 몫이다 (docstring 6).
        print(f"[RX] 풍량: {WIND_TARGETS[value[0]]} {level_txt} → {note} "
              f"(현재 적용 {applied}단)")

    # 상태(notify)는 4단계에서 에코백 구현 — 지금은 서비스 발견용으로만 등록.
    @characteristic(STATUS_UUID, CharFlags.NOTIFY)
    def status(self, options):
        raise NotImplementedError()

    def handle_disconnect(self) -> None:
        """앱(중앙) 연결 끊김 — 전원 OFF write와 동일 처리 (docstring 9).
        write 콜백과 같은 asyncio 루프에서 불리므로 동시성 문제 없음."""
        if not self._power_on:
            return
        print("[BLE] 연결 끊김 → 전원 OFF 처리 (풍속 정지 + 0°,0° 파킹)")
        self._power_on = False
        self._apply_wind()


async def _watch_disconnects(bus, service: EswFanServiceV2) -> None:
    """BlueZ Device1.Connected 프로퍼티 변경을 구독해 연결 끊김을 감지한다
    (docstring 9 — bluez_peripheral엔 연결 이벤트 API가 없어 D-Bus 직접 구독)."""
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


async def _ble_main(service: EswFanServiceV2) -> None:
    bus = await get_message_bus()

    await service.register(bus)
    await _watch_disconnects(bus, service)

    try:
        adapter = await Adapter.get_first(bus)
    except ValueError:
        sys.exit("BLE 어댑터 없음 — 'bluetoothctl power on' 확인")

    advert = Advertisement(LOCAL_NAME, [SERVICE_UUID], appearance=0x0000, timeout=0)
    await advert.register(bus, adapter=adapter)

    print(f"[BLE] Advertising 시작 — {LOCAL_NAME} (Ctrl+C 종료)")
    await bus.wait_for_disconnect()


def main() -> None:
    p = argparse.ArgumentParser(description="BLE 타겟 모드 → 추적 + 풍속 릴레이 (v2)")
    p.add_argument("--axis", choices=("pan", "tilt", "pantilt"), required=True,
                   help="타겟 모드일 때 돌릴 추적 축")
    p.add_argument("--model", default="multipose_lightning.tflite")
    p.add_argument("--conf", type=float, default=0.25, help="키포인트 신뢰도 임계값")
    p.add_argument("--threads", type=int, default=3, help="TFLite 스레드 수")
    # ── 축별 튜닝 (axis에 따라 일부만 실제로 쓰임 — verify_track_*.py 참고) ────
    p.add_argument("--gain-pan", type=float, default=0.3)
    p.add_argument("--gain-tilt", type=float, default=0.2)
    p.add_argument("--deadzone-pan", type=float, default=1.0)
    p.add_argument("--deadzone-tilt", type=float, default=0.5)
    p.add_argument("--target-cx", type=float, default=0.5)
    p.add_argument("--target-cy", type=float, default=0.5)
    p.add_argument("--limit", type=float, default=100.0, help="--axis pan 전용 소프트 클램프 ±°")
    lim = CFG["limits"]
    p.add_argument("--pan-min", type=float, default=lim["pan"]["min"])
    p.add_argument("--pan-max", type=float, default=lim["pan"]["max"])
    p.add_argument("--tilt-min", type=float, default=lim["tilt"]["min"])
    p.add_argument("--tilt-max", type=float, default=lim["tilt"]["max"])
    p.add_argument("--invert-pan", action="store_true")
    p.add_argument("--invert-tilt", action="store_true")
    p.add_argument("--region", choices=("chest", "head", "upper", "lower"), default="chest",
                   help="--axis tilt 전용 조준 부위")
    # ── 기본-회전 모드 (0x01) 스윕 ───────────────────────────────────────────
    p.add_argument("--rotate-span", type=float, default=60.0,
                   help="회전 모드 pan 스윕 반각 — 0° 기준 ±° (docstring 7)")
    p.add_argument("--rotate-speed", type=float, default=20.0,
                   help="회전 모드 스윕 속도 (°/s)")
    # ── 카메라 백엔드 (app/camera.py와 동일) ────────────────────────────────
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
    # (프레임 전달 통로로 web_state를 재사용하므로 args.web을 켠다 —
    #  HTTP 서버는 사용자가 --web을 명시한 경우에만 띄운다).
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
            # 첫 실행이면 현재 위치가 0°(각 축)가 된다 — 시작 위치 마커 권장.
            mc.restore_origin()

            if args.dry_run:
                fan_cm = _DryRelay()
            else:
                # lgpio를 최상단에서 import하는 모듈이라 여기서 지연 import
                # (개발 PC --dry-run에서 스크립트가 아예 못 뜨는 것 방지).
                from hardware.relay_controller import FanRelay
                fan_cm = FanRelay(CFG, handle=mc.h)  # gpiochip 핸들 공유 (docstring 4)

            with fan_cm as fan:  # mc보다 먼저 닫힘 — 공유 핸들이 살아있을 때 전부 오픈
                track_fn = _make_runner(args.axis, detector, tracker, mc, args, web_state)
                supervisor = _ModeSupervisor(track_fn, _make_sweeper(mc, args),
                                             _make_homer(mc, home_pan=False),
                                             _make_homer(mc, home_pan=True),
                                             stop_fn=mc.stop,
                                             web_state=web_state)
                service = EswFanServiceV2(supervisor, fan)

                print(f"[E2E] axis={args.axis} — BLE 전원/모드/풍량 명령 대기 중.")
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
