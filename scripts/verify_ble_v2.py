#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/verify_ble_v2.py - BLE 수신 → 풍속 릴레이 실구동 (verify_ble.py의 v2)

verify_ble.py(print 검증 원형)와의 차이점만 기록합니다 — 설치/실행/커널
회귀 버그 주의사항·프로토콜 명세는 원형 상단 docstring과 동일하므로 반복하지
않습니다:

  1. FanRelay(hardware/relay_controller.py) 연결 — WIND write가 print에
     그치지 않고 실제 릴레이(TS0011)를 구동한다.
  2. 전원 게이팅 — 시작 시 전원 OFF 가정. POWER ON이면 마지막 수신 세기를
     재적용, OFF면 즉시 정지(전부 오픈). 전원 OFF 중 WIND write는 저장만
     하고 돌리지 않는다 (원형은 전원/풍량이 서로 무관하게 print만 했음).
  3. WIND [대상, 세기]에서 대상(공용/머리/상체/하체)은 여전히 로그만 —
     대상별 풍속 중재(프로토콜 §3.3 저장/적용)는 4단계 상위 통합부 몫.
  4. main()을 with FanRelay(CFG)로 감싸 종료(Ctrl+C 포함) 시 정지 보장.
     프로젝트 모듈 import를 위해 sys.path에 레포 루트 추가.
"""

import asyncio
import sys
from pathlib import Path

from bluez_peripheral.adapter import Adapter
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.gatt import CharacteristicFlags as CharFlags
from bluez_peripheral.gatt import Service, characteristic
from bluez_peripheral.util import get_message_bus

sys.path.insert(0, str(Path(__file__).parent.parent))
from config import CFG
from hardware.relay_controller import FanRelay

# ── 프로토콜 (ble_protocol.md와 일치해야 함 — verify_ble.py 원형과 동일) ──
UUID_BASE = "14d7{:04x}-7197-49e5-a017-0b2f308120f0"
SERVICE_UUID = UUID_BASE.format(0x0001)
POWER_UUID = UUID_BASE.format(0x0002)   # write, 1B: 0x00 OFF / 0x01 ON
MODE_UUID = UUID_BASE.format(0x0003)    # write, 1B: 고정/회전/타겟/타겟-부위
WIND_UUID = UUID_BASE.format(0x0004)    # write, 2B: [대상, 세기]
STATUS_UUID = UUID_BASE.format(0x0005)  # notify: 에코백/인식 Status (4단계)

LOCAL_NAME = "ESW-FAN"

MODE_NAMES = {0x00: "기본-고정", 0x01: "기본-회전", 0x02: "타겟", 0x03: "타겟-부위"}
WIND_TARGETS = {0x00: "공용", 0x01: "머리", 0x02: "상체", 0x03: "하체"}


def _hex(value):
    return " ".join(f"0x{b:02X}" for b in value)


class EswFanService(Service):
    def __init__(self, fan: FanRelay):
        super().__init__(SERVICE_UUID, True)
        self._fan = fan
        self._power_on = False   # 시작 시 전원 OFF 가정 (앱이 POWER ON을 먼저 보냄)
        self._last_level = 0     # 마지막 수신 세기 — ON 시 재적용

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
        level = self._last_level if self._power_on else 0
        self._fan.set_speed(level)
        print(f"[RX] 전원: {'ON' if self._power_on else 'OFF'} → 풍속 {level}단 적용")

    @characteristic(MODE_UUID, CharFlags.WRITE)
    def mode(self, options):
        raise NotImplementedError()

    @mode.setter
    def mode(self, value, options):
        if len(value) != 1 or value[0] not in MODE_NAMES:
            print(f"[RX] 모드: 잘못된 값 ({_hex(value)})")
            return
        print(f"[RX] 모드: {MODE_NAMES[value[0]]}")

    @characteristic(WIND_UUID, CharFlags.WRITE)
    def wind(self, options):
        raise NotImplementedError()

    @wind.setter
    def wind(self, value, options):
        if len(value) != 2 or value[0] not in WIND_TARGETS or not 1 <= value[1] <= 3:
            print(f"[RX] 풍량: 잘못된 값 ({_hex(value)})")
            return
        self._last_level = value[1]
        if self._power_on:
            self._fan.set_speed(value[1])
            note = "릴레이 적용"
        else:
            note = "전원 OFF — 저장만 (ON 시 적용)"
        print(f"[RX] 풍량: {WIND_TARGETS[value[0]]} {value[1]}단 → {note}")

    # 상태(notify)는 4단계에서 에코백 구현 — 지금은 서비스 발견용으로만 등록.
    @characteristic(STATUS_UUID, CharFlags.NOTIFY)
    def status(self, options):
        raise NotImplementedError()


async def main(fan: FanRelay):
    bus = await get_message_bus()

    service = EswFanService(fan)
    await service.register(bus)

    try:
        adapter = await Adapter.get_first(bus)
    except ValueError:
        sys.exit("BLE 어댑터 없음 — 'bluetoothctl power on' 확인")

    advert = Advertisement(LOCAL_NAME, [SERVICE_UUID], appearance=0x0000, timeout=0)
    await advert.register(bus, adapter=adapter)

    print(f"[BLE] Advertising 시작 — {LOCAL_NAME} (Ctrl+C 종료)")
    await bus.wait_for_disconnect()


if __name__ == "__main__":
    with FanRelay(CFG) as fan:   # 종료 시 전부 오픈(정지) 보장
        try:
            asyncio.run(main(fan))
        except KeyboardInterrupt:
            print("\n[BLE] 종료 — 풍속 정지")
