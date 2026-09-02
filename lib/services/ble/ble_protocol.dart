/// BLE GATT 프로토콜 상수 (docs/ble_protocol.md와 일치해야 함).
///
/// RPi 쪽 구현(submission 브랜치의 app/ble_protocol.py, app/ble_service.py)과
/// 공유하는 계약이므로 변경 시 문서·RPi 코드를 함께 갱신한다.
library;

import 'dart:typed_data';

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

/// write, 2바이트: [대상(0 공용/1 머리/2 상체/3 하체), 세기(0 정지/1~3단)]
final String windCharUuid = _uuid(0x0004);

/// read: 상태 스냅샷(아래 [EswSnapshot]) / notify: [타입, 페이로드...]
final String statusCharUuid = _uuid(0x0005);

// ── 상태 notify 타입 (ble_protocol.md 3.4) ──────────────────────────────────

/// 명령 에코백: [0x01, char#(1~3), 원본...] — 수신·수락된 write에만 온다.
const int statusTypeEcho = 0x01;

/// 객체 인식 Status: [0x02, 0x00 미인식 / 0x01 인식 중]
const int statusTypeRecognition = 0x02;

/// 유효 모드 push: [0x03, 유효 모드] — RPi 판단으로 실제 동작 모드가 바뀔 때.
const int statusTypeEffectiveMode = 0x03;

/// 상태 스냅샷: [0x04, 전원, 요청 모드, 유효 모드, 공용 세기, 머리, 상체, 하체]
const int statusTypeSnapshot = 0x04;

/// 에코백의 Characteristic 번호 (프로토콜 §2 표의 # 열).
const int charNoPower = 1, charNoMode = 2, charNoWind = 3;

/// 상태 스냅샷(read 응답·0x04 notify 공용) 파싱 결과.
class EswSnapshot {
  const EswSnapshot({
    required this.powerOn,
    required this.mode,
    required this.effectiveMode,
    required this.level,
    required this.head,
    required this.upper,
    required this.lower,
  });

  final bool powerOn;
  final int mode; // 요청 모드 (앱이 마지막으로 write한 모드)
  final int effectiveMode; // RPi가 실제로 돌리는 모드
  final int level; // 공용 세기
  final int head, upper, lower; // 부위별 세기

  static EswSnapshot? parse(Uint8List data) {
    if (data.length != 8 || data[0] != statusTypeSnapshot) return null;
    return EswSnapshot(
      powerOn: data[1] != 0,
      mode: data[2],
      effectiveMode: data[3],
      level: data[4],
      head: data[5],
      upper: data[6],
      lower: data[7],
    );
  }

  @override
  String toString() =>
      'EswSnapshot(power=$powerOn mode=$mode eff=$effectiveMode '
      'level=$level body=$head/$upper/$lower)';
}
