#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/verify_ble.py - BLE GATT 서버 print 검증 (앱 연동 2단계)

RPi 5를 Peripheral(`ESW-FAN`)로 Advertising 하고, 앱이 쓴 명령 바이트를
사람이 읽을 수 있게 print 합니다. 프로토콜 명세는
apps/ESW_BLE_app/docs/ble_protocol.md 를 따릅니다 (변경 시 함께 갱신).

설치:
    sudo apt install -y bluez          # BlueZ 데몬 (Pi OS 기본 포함)
    pip install bluez_peripheral       # 순수 파이썬(dbus_fast) — 컴파일/시스템
                                        # 파이썬 바인딩 불필요, pyenv venv에서도 그대로 설치됨
    sudo usermod -aG bluetooth $USER   # system dbus 접근 권한 (재로그인 필요)

실행:
    bluetoothctl power on            # 어댑터 켜기 (기본 on)
    python3 scripts/verify_ble.py    # SSH 포그라운드 실행, Ctrl+C 종료

검증:
    Windows 앱(또는 nRF Connect)에서 ESW-FAN 스캔 → 연결 → write
    → 이 터미널에 [RX] 로그가 찍히면 성공. Advertising은 timeout=0(무제한)으로
    등록하므로 연결 해제 후에도 계속 광고되어 재연결 가능.
"""

import asyncio
import sys

from bluez_peripheral.adapter import Adapter
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.gatt import CharacteristicFlags as CharFlags
from bluez_peripheral.gatt import Service, characteristic
from bluez_peripheral.util import get_message_bus

# ── 프로토콜 (ble_protocol.md와 일치해야 함) ──
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
    def __init__(self):
        super().__init__(SERVICE_UUID, True)

    # write-only 특성도 getter 자리(placeholder)가 필요 — 읽기 시도 시에만 쓰인다.
    @characteristic(POWER_UUID, CharFlags.WRITE)
    def power(self, options):
        raise NotImplementedError()

    @power.setter
    def power(self, value, options):
        if len(value) != 1 or value[0] not in (0x00, 0x01):
            print(f"[RX] 전원: 잘못된 값 ({_hex(value)})")
            return
        print(f"[RX] 전원: {'ON' if value[0] else 'OFF'}")

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
        print(f"[RX] 풍량: {WIND_TARGETS[value[0]]} {value[1]}단")

    # 상태(notify)는 4단계에서 에코백 구현 — 지금은 서비스 발견용으로만 등록.
    @characteristic(STATUS_UUID, CharFlags.NOTIFY)
    def status(self, options):
        raise NotImplementedError()


async def main():
    bus = await get_message_bus()

    service = EswFanService()
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
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[BLE] 종료")
