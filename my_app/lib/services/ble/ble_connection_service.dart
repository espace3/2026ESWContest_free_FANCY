import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:universal_ble/universal_ble.dart';

import 'ble_protocol.dart' as proto;

/// 앱 전역 BLE 연결 상태 — 화면을 넘나들어도 연결이 유지되도록 싱글턴으로 둔다.
class BleConnectionService extends ChangeNotifier {
  BleConnectionService._();

  static final instance = BleConnectionService._();

  String? _deviceId;
  String? _deviceName;
  StreamSubscription<bool>? _connectionSub;
  bool _debugFake = false;

  bool get isConnected => _deviceId != null;

  String? get deviceId => _deviceId;

  /// 연결 시점의 기기 이름 (스캔 페이지의 "연결된 기기" 고정 표시용).
  String? get deviceName => _deviceName;

  Future<void> connect(BleDevice device) async {
    await UniversalBle.connect(device.deviceId);
    // Windows(WinRT)는 서비스 발견 전 write가 실패할 수 있어 연결 절차에 포함.
    await UniversalBle.discoverServices(device.deviceId);
    _deviceId = device.deviceId;
    _deviceName = device.name;
    unawaited(_connectionSub?.cancel());
    _connectionSub = UniversalBle.connectionStream(device.deviceId).listen((
      connected,
    ) {
      if (!connected) {
        _deviceId = null;
        _deviceName = null;
        notifyListeners();
      }
    });
    notifyListeners();
  }

  Future<void> disconnect() async {
    final id = _deviceId;
    if (id == null) return;
    await UniversalBle.disconnect(id);
    await _connectionSub?.cancel();
    _connectionSub = null;
    _deviceId = null;
    _deviceName = null;
    notifyListeners();
  }

  /// 각 write는 성공 여부를 돌려준다 — 호출부(홈 화면 등)가 실패를 사용자에게
  /// 알릴 수 있게 하기 위함 (기존엔 조용히 삼켜서 "눌러도 무반응"으로 보였음).
  Future<bool> writePower(bool on) => _write(proto.powerCharUuid, [on ? 1 : 0]);

  Future<bool> writeMode(int mode) => _write(proto.modeCharUuid, [mode]);

  Future<bool> writeWind(int target, int strength) =>
      _write(proto.windCharUuid, [target, strength]);

  Future<bool> _write(String charUuid, List<int> value) async {
    final id = _deviceId;
    if (id == null) return false;
    if (_debugFake) return true;
    try {
      await _writeRaw(id, charUuid, value);
      return true;
    } catch (e) {
      // RPi 서버를 재시작하면 GATT handle 배치가 바뀔 수 있는데 Windows는
      // 기기별 GATT 캐시를 재사용해서, 재시작 후 첫 연결의 write가 낡은
      // handle로 나가 실패하는 경우가 있다 (실기에서 "실험마다 1회 무반응"
      // 으로 재현됨). 서비스를 재발견해 캐시를 갱신하고 1회 재시도한다.
      debugPrint('BLE write 실패: $e — 서비스 재발견 후 재시도');
      try {
        await UniversalBle.discoverServices(id);
        await _writeRaw(id, charUuid, value);
        return true;
      } catch (e2) {
        debugPrint('BLE write 재시도 실패: $e2 — 연결 강제 해제');
        await _forceDisconnect(id);
        return false;
      }
    }
  }

  Future<void> _writeRaw(String id, String charUuid, List<int> value) =>
      UniversalBle.write(
        id,
        proto.serviceUuid,
        charUuid,
        Uint8List.fromList(value),
      );

  /// write가 재시도까지 실패하면 연결이 죽은 것으로 간주하고 강제 해제한다.
  ///
  /// RPi 프로세스가 죽어도 Windows는 connectionStream을 안 울리는 경우가
  /// 있고, getConnectionState조차 낡은 "연결됨"을 돌려줄 수 있어(실기 확인)
  /// 상태 조회에 의존하지 않는다 — 앱 상태를 먼저 비워 UI(전원 비활성화,
  /// 칩/고정 카드 해제)를 즉시 바로잡고, OS 쪽 잔여 연결은 best effort로 정리.
  Future<void> _forceDisconnect(String id) async {
    if (_deviceId != id) return;
    _deviceId = null;
    _deviceName = null;
    notifyListeners();
    await _connectionSub?.cancel();
    _connectionSub = null;
    try {
      await UniversalBle.disconnect(id);
    } catch (_) {
      // 이미 끊긴 연결의 해제 실패는 무시해도 안전.
    }
  }

  /// 위젯 테스트에서 실제 BLE 없이 "연결됨" 상태를 만들기 위한 훅.
  /// 이후 write 호출은 실제 플랫폼 채널을 타지 않고 성공으로 간주된다.
  @visibleForTesting
  void debugMarkConnected([String deviceId = 'debug']) {
    _deviceId = deviceId;
    _debugFake = true;
    notifyListeners();
  }

  @override
  void dispose() {
    unawaited(_connectionSub?.cancel());
    super.dispose();
  }
}
