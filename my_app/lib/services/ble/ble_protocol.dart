/// BLE GATT 프로토콜 상수 (docs/ble_protocol.md와 일치해야 함).
///
/// RPi 쪽 구현(ESW/scripts/verify_ble.py)과 공유하는 계약이므로
/// 변경 시 문서·RPi 스크립트를 함께 갱신한다.
library;

/// Advertising 로컬 네임 (표시용 — 기기 식별은 [serviceUuid]로 한다).
const String eswLocalName = 'ESW-FAN';

String _uuid(int index16) =>
    '14d7${index16.toRadixString(16).padLeft(4, '0')}'
    '-7197-49e5-a017-0b2f308120f0';

final String serviceUuid = _uuid(0x0001);

/// write, 1바이트: 0x00 OFF / 0x01 ON
final String powerCharUuid = _uuid(0x0002);

/// write, 1바이트: 0x00 기본-고정 / 0x01 기본-회전 / 0x02 타겟 / 0x03 타겟-부위
final String modeCharUuid = _uuid(0x0003);

/// write, 2바이트: [대상(0 공용/1 머리/2 상체/3 하체), 세기(1~3)]
final String windCharUuid = _uuid(0x0004);

/// notify: [타입, 페이로드...] — 에코백·객체 인식 Status (4단계에서 구현)
final String statusCharUuid = _uuid(0x0005);
