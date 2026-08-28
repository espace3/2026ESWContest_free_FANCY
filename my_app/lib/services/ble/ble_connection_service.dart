import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:universal_ble/universal_ble.dart';

import 'ble_protocol.dart' as proto;

/// 연결 확립 전(서비스 발견 등 절차 중)에 상대가 연결을 닫은 경우.
/// 이번 시도의 실패로 돌려 재시도/안내 경로를 타게 하려고 따로 둔다.
class BleDroppedDuringConnect implements Exception {
  const BleDroppedDuringConnect();

  @override
  String toString() => '연결 절차 중 끊김 (기기가 연결을 닫음)';
}

/// write 후 에코백(Status 0x01) 대기 항목 — 타임아웃이면 미확인 write로 본다.
class _PendingEcho {
  _PendingEcho(this.charNo, this.value, this.timer);

  final int charNo;
  final List<int> value;
  final Timer timer;
}

/// 앱 전역 BLE 연결 상태 — 화면을 넘나들어도 연결이 유지되도록 싱글턴으로 둔다.
///
/// v3(상태 동기화)부터 전송 계층의 책임이 늘었다 (docs/ble_protocol.md §3.4):
///  - Status(0x0005) 구독 — 에코백(0x01)은 여기서 소비해 미확인 write를
///    감지하고, 그 외 타입(인식/유효 모드/스냅샷)은 [onStatusNotify]로
///    FanStateService에 전달한다.
///  - 에코 타임아웃: RPi는 write 수락 직후(모터 적용 전) 에코를 보내므로
///    3초 안에 안 오면 유실로 보고 [onEchoTimeout]을 부른다 — 즉시 재전송하지
///    않고 스냅샷 검증에 맡긴다 (중복 전송 방지).
///  - 예기치 못한 끊김이면 30초간 자동 재연결을 시도하고 성공 시
///    [onAutoReconnected]를 부른다. 전원 복원 여부 판단은 FanStateService 몫.
class BleConnectionService extends ChangeNotifier {
  BleConnectionService._();

  static final instance = BleConnectionService._();

  /// 에코백 미수신 판정 시간 (ble_protocol.md §3.4).
  static const echoTimeout = Duration(seconds: 3);

  /// 연결 시도 하나의 제한 시간. universal_ble 기본값은 **60초**라, 실패할
  /// 연결도 1분을 매달린 뒤에야 실패가 드러난다(재시도까지 하면 2분 —
  /// 실기 2026-08-27 "연결 버튼을 누르고 한참 뒤에 실패"의 정체).
  /// 짧게 끊고 여러 번 시도하는 편이 성공률도 체감 속도도 낫다.
  static const _connectTimeout = Duration(seconds: 15);

  /// 시도 사이 대기 (총 시도 횟수 = 항목 수 + 1). 안드로이드는 그 기기와의
  /// 첫 GATT 연결이 133으로 튕기는 일이 흔해, 빠른 재시도로 흡수한다.
  static const _retryDelays = [
    Duration(milliseconds: 500),
    Duration(seconds: 2),
  ];

  /// 재시도 전 낡은 연결을 정리할 때의 제한 시간 — disconnect도 기본이 60초라
  /// 그대로 두면 정리 단계에서 다시 오래 매달린다.
  static const _cleanupTimeout = Duration(seconds: 5);

  /// 예기치 못한 끊김 후 자동 재연결을 시도하는 시간창 — 이 안에 성공하면
  /// FanStateService가 전원까지 복원한다 (순단 자가 치유).
  static const reconnectWindow = Duration(seconds: 30);
  static const _reconnectInterval = Duration(seconds: 3);

  String? _deviceId;
  String? _deviceName;
  StreamSubscription<bool>? _connectionSub;
  StreamSubscription<Uint8List>? _statusSub;
  final _pendingEchoes = <_PendingEcho>[];
  Object? _reconnectToken;
  bool _debugFake = false;
  List<BleService> _lastServices = const [];

  /// 진행 중인 연결 시도가 끊김을 만나면 여기로 알려 시도를 즉시 실패시킨다
  /// (확립 후의 끊김은 [_handleDrop] 경로).
  Completer<void>? _pendingDrop;

  /// Status notify 중 에코백을 제외한 타입 — FanStateService가 해석한다.
  void Function(Uint8List value)? onStatusNotify;

  /// write 에코백이 [echoTimeout] 안에 안 왔다 — 스냅샷 검증 트리거용.
  void Function()? onEchoTimeout;

  /// 예기치 못한 끊김 후 자동 재연결 성공 (창 안).
  void Function()? onAutoReconnected;

  bool get isConnected => _deviceId != null;

  String? get deviceId => _deviceId;

  /// 연결 시점의 기기 이름 (스캔 페이지의 "연결된 기기" 고정 표시용).
  String? get deviceName => _deviceName;

  /// 테스트 훅 여부 — 타이머/실 BLE를 만들면 안 되는 경로의 분기용.
  bool get isDebugFake => _debugFake;

  /// 마지막 연결 절차에서 발견한 서비스 목록.
  /// 호출부(스캔 페이지 로그 등)가 이 값을 읽어 쓰게 해서 연결 직후
  /// discoverServices를 또 던지는 일을 없앤다 — 안드로이드에서 연결 직후
  /// GATT 작업을 연달아 던지면 133 실패를 유발한다.
  List<BleService> get lastServices => List.unmodifiable(_lastServices);

  Future<void> connect(BleDevice device) async {
    _reconnectToken = null; // 진행 중인 자동 재연결 취소 — 수동 연결 우선
    final id = device.deviceId;
    _listenConnection(id, device.name);
    for (var attempt = 0; ; attempt++) {
      try {
        await _establish(id);
        break;
      } catch (e) {
        if (attempt >= _retryDelays.length) {
          // 마지막 시도까지 실패 — 걸어둔 구독을 정리하고 예외는 그대로 던져
          // 호출부(스캔 페이지)가 사용자에게 원인을 보여준다.
          await _detachAttempt();
          rethrow;
        }
        debugPrint('BLE 연결 시도 ${attempt + 1} 실패($e) — 정리 후 재시도');
        // 이전 실행이 정상 종료 못 해(앱이 백그라운드에서 OS에 강제 종료되는 등)
        // RPi가 낡은 연결을 붙들고 있으면 첫 시도가 실패한다. 반쯤 열린 연결을
        // 정리하고 잠깐 뒤 다시 시도해 흡수한다.
        try {
          await UniversalBle.disconnect(id, timeout: _cleanupTimeout);
        } catch (_) {}
        await Future.delayed(_retryDelays[attempt]);
      }
    }
    _attach(id, device.name);
  }

  Future<void> _establish(String id) async {
    // 절차 도중의 끊김(아래 [_listenConnection])을 이번 시도의 실패로 받는다.
    final drop = _pendingDrop = Completer<void>();
    try {
      await Future.any([_runConnectSteps(id), drop.future]);
    } finally {
      if (identical(_pendingDrop, drop)) _pendingDrop = null;
    }
  }

  Future<void> _runConnectSteps(String id) async {
    await UniversalBle.connect(id, timeout: _connectTimeout);
    // Windows(WinRT)는 서비스 발견 전 write가 실패할 수 있어 연결 절차에 포함.
    _lastServices = await UniversalBle.discoverServices(id);
    await _subscribeStatus(id);
  }

  /// 연결 이벤트 구독 — 반드시 [UniversalBle.connect] **전에** 건다.
  /// connectionStream은 과거 이벤트를 재생하지 않는 broadcast 스트림이라,
  /// 확립 후에 구독하면 수 초 걸리는 서비스 발견 도중의 끊김을 통째로 놓쳐
  /// "연결됨"으로 오표시되고 첫 write가 실패할 때까지 모른다 (실기 재현).
  void _listenConnection(String id, String? name) {
    unawaited(_connectionSub?.cancel());
    _connectionSub = UniversalBle.connectionStream(id).listen((connected) {
      if (connected) return;
      final pending = _pendingDrop;
      if (pending != null && !pending.isCompleted) {
        pending.completeError(const BleDroppedDuringConnect()); // 확립 전 → 시도 실패
      } else if (_deviceId == id) {
        _handleDrop(id, name, unexpected: true); // 확립 후 → 자동 재연결
      }
    });
  }

  /// 실패한 연결 시도의 잔여물 정리 — 확립 전이라 앱 상태는 건드릴 게 없다.
  Future<void> _detachAttempt() async {
    await _connectionSub?.cancel();
    _connectionSub = null;
    await _statusSub?.cancel();
    _statusSub = null;
    _lastServices = const [];
  }

  /// 연결 확립 후 공통 마무리 — 필드 반영 + 통지.
  /// 연결 스트림은 시도 전에 [_listenConnection]이 이미 걸어 뒀다.
  void _attach(String id, String? name) {
    _deviceId = id;
    _deviceName = name;
    notifyListeners();
  }

  Future<void> _subscribeStatus(String id) async {
    await _statusSub?.cancel();
    _statusSub = UniversalBle.characteristicValueStream(id, proto.statusCharUuid)
        .listen(_onStatusValue);
    try {
      await UniversalBle.subscribeNotifications(
          id, proto.serviceUuid, proto.statusCharUuid);
    } catch (e) {
      // 구형 RPi 스크립트(E2E v1/v2)는 구독이 실패할 수 있다 — 동기화 기능만
      // 빠질 뿐 제어(write)는 되므로 연결은 유지한다.
      debugPrint('Status 구독 실패 (RPi v3 필요): $e');
    }
  }

  void _onStatusValue(Uint8List value) {
    if (value.isEmpty) return;
    if (value[0] == proto.statusTypeEcho) {
      _resolveEcho(value);
    } else {
      onStatusNotify?.call(value);
    }
  }

  void _resolveEcho(Uint8List value) {
    if (value.length < 2) return;
    final original = value.sublist(2);
    final i = _pendingEchoes.indexWhere(
        (p) => p.charNo == value[1] && listEquals(p.value, original));
    // 짝 없는 에코(등록 전 도착 등)는 무시 — 남은 항목은 타임아웃이
    // 스냅샷 검증으로 흡수한다.
    if (i < 0) return;
    _pendingEchoes.removeAt(i).timer.cancel();
  }

  Future<void> disconnect() async {
    _reconnectToken = null; // 사용자 의도의 해제 — 자동 재연결 안 함
    final id = _deviceId;
    if (id == null) return;
    _handleDrop(id, null, unexpected: false);
    try {
      await UniversalBle.disconnect(id);
    } catch (_) {}
  }

  /// 연결 상실 공통 처리 — 앱 상태를 먼저 비워 UI를 즉시 바로잡는다.
  /// [unexpected]면 자동 재연결을 시작한다.
  void _handleDrop(String id, String? name, {required bool unexpected}) {
    if (_deviceId != id) return; // 확립 전(pending) 끊김은 _listenConnection이 처리
    _deviceId = null;
    _deviceName = null;
    _lastServices = const [];
    for (final p in _pendingEchoes) {
      p.timer.cancel();
    }
    _pendingEchoes.clear();
    unawaited(_connectionSub?.cancel());
    _connectionSub = null;
    unawaited(_statusSub?.cancel());
    _statusSub = null;
    notifyListeners();
    if (unexpected) unawaited(_autoReconnect(id, name));
  }

  Future<void> _autoReconnect(String id, String? name) async {
    final token = Object();
    _reconnectToken = token;
    final deadline = DateTime.now().add(reconnectWindow);
    debugPrint('BLE 자동 재연결 시도 (최대 ${reconnectWindow.inSeconds}s)');
    while (DateTime.now().isBefore(deadline)) {
      await Future.delayed(_reconnectInterval);
      if (_reconnectToken != token || _deviceId != null) return; // 취소/수동 연결
      _listenConnection(id, name); // 여기서도 절차 중 끊김을 놓치면 안 된다
      try {
        await _establish(id);
      } catch (_) {
        continue;
      }
      if (_reconnectToken != token || _deviceId != null) return;
      _attach(id, name);
      debugPrint('BLE 자동 재연결 성공');
      onAutoReconnected?.call();
      return;
    }
    if (_reconnectToken == token) await _detachAttempt(); // 마지막 시도의 구독 정리
    debugPrint('BLE 자동 재연결 포기 — 수동 연결 필요');
  }

  /// 각 write는 성공 여부를 돌려준다 — 호출부(홈 화면 등)가 실패를 사용자에게
  /// 알릴 수 있게 하기 위함 (기존엔 조용히 삼켜서 "눌러도 무반응"으로 보였음).
  Future<bool> writePower(bool on) => _write(proto.powerCharUuid, [on ? 1 : 0]);

  Future<bool> writeMode(int mode) => _write(proto.modeCharUuid, [mode]);

  Future<bool> writeWind(int target, int strength) =>
      _write(proto.windCharUuid, [target, strength]);

  int _charNo(String charUuid) => charUuid == proto.powerCharUuid
      ? proto.charNoPower
      : charUuid == proto.modeCharUuid
          ? proto.charNoMode
          : proto.charNoWind;

  Future<bool> _write(String charUuid, List<int> value) async {
    final id = _deviceId;
    if (id == null) return false;
    if (_debugFake) {
      debugWrites.add([_charNo(charUuid), ...value]);
      return true;
    }
    final pending = _registerEcho(charUuid, value);
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
        _lastServices = await UniversalBle.discoverServices(id);
        await _writeRaw(id, charUuid, value);
        return true;
      } catch (e2) {
        debugPrint('BLE write 재시도 실패: $e2 — 연결 강제 해제');
        pending.timer.cancel();
        _pendingEchoes.remove(pending);
        await _forceDisconnect(id);
        return false;
      }
    }
  }

  _PendingEcho _registerEcho(String charUuid, List<int> value) {
    late final _PendingEcho pending;
    pending = _PendingEcho(_charNo(charUuid), List.of(value),
        Timer(echoTimeout, () {
      _pendingEchoes.remove(pending);
      debugPrint('에코 미수신: char#${pending.charNo} ${pending.value} '
          '— 스냅샷 검증 요청');
      onEchoTimeout?.call();
    }));
    _pendingEchoes.add(pending);
    return pending;
  }

  Future<void> _writeRaw(String id, String charUuid, List<int> value) =>
      UniversalBle.write(
        id,
        proto.serviceUuid,
        charUuid,
        Uint8List.fromList(value),
      );

  /// 상태 스냅샷 read (ble_protocol.md §3.4) — 실패/미연결이면 null.
  Future<proto.EswSnapshot?> readSnapshot() async {
    final id = _deviceId;
    if (id == null || _debugFake) return null;
    try {
      final data =
          await UniversalBle.read(id, proto.serviceUuid, proto.statusCharUuid);
      return proto.EswSnapshot.parse(data);
    } catch (e) {
      debugPrint('스냅샷 read 실패: $e');
      return null;
    }
  }

  /// write가 재시도까지 실패하면 연결이 죽은 것으로 간주하고 강제 해제한다.
  ///
  /// RPi 프로세스가 죽어도 Windows는 connectionStream을 안 울리는 경우가
  /// 있고, getConnectionState조차 낡은 "연결됨"을 돌려줄 수 있어(실기 확인)
  /// 상태 조회에 의존하지 않는다. 이후 자동 재연결이 재발견까지 다시 하므로
  /// RPi 재시작으로 handle이 바뀐 경우도 회복된다.
  Future<void> _forceDisconnect(String id) async {
    final name = _deviceName;
    _handleDrop(id, name, unexpected: true);
    try {
      await UniversalBle.disconnect(id);
    } catch (_) {
      // 이미 끊긴 연결의 해제 실패는 무시해도 안전.
    }
  }

  /// 위젯 테스트에서 실제 BLE 없이 "연결됨" 상태를 만들기 위한 훅.
  /// 이후 write는 플랫폼 채널 대신 [debugWrites]에 기록되고 성공으로 간주된다.
  @visibleForTesting
  void debugMarkConnected([String deviceId = 'debug']) {
    _deviceId = deviceId;
    _debugFake = true;
    notifyListeners();
  }

  /// debugFake 모드에서 기록된 write 목록: `[char#, ...원본 바이트]`.
  /// 재전송 계약(리모컨 주체) 테스트용.
  @visibleForTesting
  final debugWrites = <List<int>>[];

  @override
  void dispose() {
    unawaited(_connectionSub?.cancel());
    unawaited(_statusSub?.cancel());
    for (final p in _pendingEchoes) {
      p.timer.cancel();
    }
    super.dispose();
  }
}
