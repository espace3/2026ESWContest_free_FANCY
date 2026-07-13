#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/verify_ble.py - BLE GATT 서버 print 검증 (앱 연동 2단계)

RPi 5를 Peripheral(`ESW-FAN`)로 Advertising 하고, 앱이 쓴 명령 바이트를
사람이 읽을 수 있게 print 합니다. 프로토콜 명세는
apps/ESW_BLE_app/docs/ble_protocol.md 를 따릅니다 (변경 시 함께 갱신).

설치 (Pi OS Lite):
    sudo apt install -y python3-dbus python3-gi
    pip install bluezero
    # venv 사용 시 dbus/gi가 보이도록: python3 -m venv --system-site-packages .venv

실행:
    bluetoothctl power on            # 어댑터 켜기 (기본 on)
    python3 scripts/verify_ble.py    # SSH 포그라운드 실행, Ctrl+C 종료

검증:
    Windows 앱(또는 nRF Connect)에서 ESW-FAN 스캔 → 연결 → write
    → 이 터미널에 [RX] 로그가 찍히면 성공. 연결 해제 시 자동 re-advertising.
"""

import sys

from bluezero import adapter, peripheral

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


def on_power_write(value, options):
    if len(value) != 1 or value[0] not in (0x00, 0x01):
        print(f"[RX] 전원: 잘못된 값 ({_hex(value)})")
        return
    print(f"[RX] 전원: {'ON' if value[0] else 'OFF'}")


def on_mode_write(value, options):
    if len(value) != 1 or value[0] not in MODE_NAMES:
        print(f"[RX] 모드: 잘못된 값 ({_hex(value)})")
        return
    print(f"[RX] 모드: {MODE_NAMES[value[0]]}")


def on_wind_write(value, options):
    if len(value) != 2 or value[0] not in WIND_TARGETS or not 1 <= value[1] <= 3:
        print(f"[RX] 풍량: 잘못된 값 ({_hex(value)})")
        return
    print(f"[RX] 풍량: {WIND_TARGETS[value[0]]} {value[1]}단")


def on_connect(device=None):
    print(f"[BLE] 연결됨: {getattr(device, 'address', device)}")


def on_disconnect(adapter_address=None, device_address=None):
    print(f"[BLE] 연결 해제: {device_address} (re-advertising 대기)")


def main():
    adapters = list(adapter.Adapter.available())
    if not adapters:
        sys.exit("BLE 어댑터 없음 — 'bluetoothctl power on' 확인")
    dongle = adapters[0]
    print(f"[BLE] 어댑터: {dongle.address} / 광고 이름: {LOCAL_NAME}")

    ble = peripheral.Peripheral(dongle.address, local_name=LOCAL_NAME)
    ble.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)
    ble.add_characteristic(srv_id=1, chr_id=1, uuid=POWER_UUID,
                           value=[], notifying=False,
                           flags=["write"], write_callback=on_power_write)
    ble.add_characteristic(srv_id=1, chr_id=2, uuid=MODE_UUID,
                           value=[], notifying=False,
                           flags=["write"], write_callback=on_mode_write)
    ble.add_characteristic(srv_id=1, chr_id=3, uuid=WIND_UUID,
                           value=[], notifying=False,
                           flags=["write"], write_callback=on_wind_write)
    # 상태(notify)는 4단계에서 에코백 구현 — 지금은 서비스 발견용으로만 등록.
    ble.add_characteristic(srv_id=1, chr_id=4, uuid=STATUS_UUID,
                           value=[0x00], notifying=False,
                           flags=["notify"])
    ble.on_connect = on_connect
    ble.on_disconnect = on_disconnect

    print("[BLE] Advertising 시작 — 앱에서 연결 후 버튼을 눌러보세요 (Ctrl+C 종료)")
    try:
        ble.publish()  # GLib 메인루프 블로킹
    except KeyboardInterrupt:
        print("\n[BLE] 종료")


if __name__ == "__main__":
    main()
