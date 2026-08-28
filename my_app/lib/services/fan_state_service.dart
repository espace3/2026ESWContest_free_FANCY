import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'ble/ble_connection_service.dart';
import 'ble/ble_protocol.dart' as proto;

/// 앱 전역 선풍기 설정 상태 — **진실의 원천 (리모컨 주체)**.
///
/// docs/ble_protocol.md §3.3의 계약을 앱 쪽에서 이행한다:
///  - 설정(전원/모드/풍량)의 주체는 앱. 화면 표시값이 곧 진실이고 이 서비스가
///    보관(영속화)·전송한다. RPi는 받은 대로 실행할 뿐 스스로 바꾸지 않는다.
///  - 모드가 바뀌는 모든 전환(페이지 이동, 회전/부위 토글, 전원 ON)마다 해당
///    모드의 표시 세기를 **재전송**해 "리모컨 = 선풍기"를 보장한다.
///  - 전송 burst 후 스냅샷 read로 검증하고, 다르면 1회 재전송으로 교정한다.
///  - RPi가 주체인 관측 정보(객체 인식, 유효 모드)는 notify로 받아 보관만 한다.
///
/// 영속화: 설정값은 shared_preferences에 저장돼 앱 재시작에도 남는다.
/// 전원은 저장하지 않는다 — 앱 재시작은 항상 OFF에서 출발 (안전).
/// 예기치 못한 끊김 후 30초 창 안에 자동 재연결이 되면 전원까지 복원한다
/// (BLE 순단 자가 치유). 수동 재연결은 설정만 남고 전원은 사용자 몫 —
/// 자리를 비웠다 돌아왔을 때 선풍기가 멋대로 켜지는 놀람 방지.
class FanStateService extends ChangeNotifier {
  FanStateService._();

  static final instance = FanStateService._();

  final BleConnectionService _ble = BleConnectionService.instance;

  bool _powerOn = false; // 비영속
  int _page = 0; // 0 기본 / 1 타겟 (비영속 — 전원 ON은 항상 기본 모드부터)
  bool _rotating = false;
  bool _bodyMode = false;
  int _basicStrength = 0;
  int _targetStrength = 0;
  final Map<int, int> _bodyStrengths = {0x01: 0, 0x02: 1, 0x03: 0}; // 상체 ≥1

  /// RPi 관측 보고 — 객체 인식 여부 (Status 0x02).
  /// null = 아직 보고 없음(추적이 안 도는 모드이거나 첫 보고 전). RPi는
  /// 깜빡임을 억제해 보내므로(control/recognition_reporter.py) 값이 자주
  /// 바뀌지 않는다.
  bool? recognized;

  /// RPi가 실제로 돌리는 모드 (Status 0x03/0x04). 부위 모드 중 이동 감지로
  /// 추적에 내려가 있는 동안만 [modeByte]와 다르다.
  int? effectiveMode;

  /// 추적이 도는 모드(타겟/타겟-부위)가 아니면 인식 보고가 오지 않으므로,
  /// 남아 있던 값을 지워 화면이 낡은 상태를 보여주지 않게 한다.
  void _clearRecognitionIfIdle() {
    if (!_powerOn || _page != 1) {
      recognized = null;
      effectiveMode = null;
    }
  }

  bool _wasPoweredOn = false; // 끊김 시점의 전원 — 자동 재연결 복원 판단
  bool _wasConnected = false;
  Timer? _verifyDebounce;
  bool _verifying = false;

  bool get powerOn => _powerOn;
  int get page => _page;
  bool get rotating => _rotating;
  bool get bodyMode => _bodyMode;
  int get basicStrength => _basicStrength;
  int get targetStrength => _targetStrength;
  int bodyStrength(int target) => _bodyStrengths[target]!;

  /// 현재 표시 상태가 뜻하는 모드 바이트 (ble_protocol.md §3.2).
  int get modeByte =>
      _page == 0 ? (_rotating ? 0x01 : 0x00) : (_bodyMode ? 0x03 : 0x02);

  /// main()에서 1회 — BLE 이벤트 배선. 위젯 테스트는 부르지 않아도 동작한다.
  void init() {
    _ble.onStatusNotify = _onStatusNotify;
    _ble.onEchoTimeout = _scheduleVerify;
    _ble.onAutoReconnected = _onAutoReconnected;
    _wasConnected = _ble.isConnected;
    _ble.addListener(_onConnectionChanged);
  }

  /// 영속 설정 로드 — 실패(테스트 등 플러그인 없는 환경)하면 기본값 유지.
  Future<void> load() async {
    try {
      final p = await SharedPreferences.getInstance();
      _rotating = p.getBool('rotating') ?? false;
      _bodyMode = p.getBool('bodyMode') ?? false;
      _basicStrength = p.getInt('basicStrength') ?? 0;
      _targetStrength = p.getInt('targetStrength') ?? 0;
      _bodyStrengths[0x01] = p.getInt('bodyHead') ?? 0;
      _bodyStrengths[0x02] = p.getInt('bodyUpper') ?? 1;
      _bodyStrengths[0x03] = p.getInt('bodyLower') ?? 0;
      if (_bodyStrengths[0x02] == 0) _bodyStrengths[0x02] = 1; // 상체 최소 1
      notifyListeners();
    } catch (e) {
      debugPrint('설정 로드 실패 — 기본값 사용: $e');
    }
  }

  Future<void> _save() async {
    try {
      final p = await SharedPreferences.getInstance();
      await p.setBool('rotating', _rotating);
      await p.setBool('bodyMode', _bodyMode);
      await p.setInt('basicStrength', _basicStrength);
      await p.setInt('targetStrength', _targetStrength);
      await p.setInt('bodyHead', _bodyStrengths[0x01]!);
      await p.setInt('bodyUpper', _bodyStrengths[0x02]!);
      await p.setInt('bodyLower', _bodyStrengths[0x03]!);
    } catch (_) {
      // 저장 실패는 다음 변경 때 재시도된다 — 조용히 넘어간다.
    }
  }

  // ── 설정 변경 (앱 = 주체 — 변경 즉시 전송) ─────────────────────────────────

  /// 전원 토글. ON이면 기본 모드 화면에서 시작하고 전체 상태를 밀어넣는다.
  Future<bool> setPower(bool on) async {
    if (on) {
      _page = 0; // 기존 UX 유지 — 전원 ON은 항상 기본 모드부터
      final ok = await _pushAll();
      notifyListeners();
      return ok;
    }
    final ok = await _ble.writePower(false);
    if (ok) {
      _powerOn = false;
      _clearRecognitionIfIdle();
      notifyListeners();
    }
    return ok;
  }

  /// BLE 없이 화면만 확인하는 오프라인 미리보기용 (home_page 참고).
  void setPowerLocal(bool on) {
    _powerOn = on;
    if (on) _page = 0;
    notifyListeners();
  }

  /// 기본(0) ↔ 타겟(1) 페이지 전환 — 모드가 바뀌므로 표시 세기도 재전송.
  Future<void> setPage(int page) async {
    if (page == _page) return;
    _page = page;
    _clearRecognitionIfIdle();
    notifyListeners();
    if (_powerOn) await _pushModeAndWind();
  }

  Future<void> setRotating(bool rotating) async {
    _rotating = rotating;
    notifyListeners();
    unawaited(_save());
    if (_powerOn) await _pushModeAndWind();
  }

  Future<void> setBodyMode(bool bodyMode) async {
    _bodyMode = bodyMode;
    notifyListeners();
    unawaited(_save());
    if (_powerOn) await _pushModeAndWind();
  }

  Future<void> setBasicStrength(int level) async {
    _basicStrength = level;
    notifyListeners();
    unawaited(_save());
    await _ble.writeWind(0x00, level);
  }

  Future<void> setTargetStrength(int level) async {
    _targetStrength = level;
    notifyListeners();
    unawaited(_save());
    await _ble.writeWind(0x00, level);
  }

  Future<void> setBodyStrength(int target, int level) async {
    if (target == 0x02 && level == 0) return; // 상체 정지 불가 (프로토콜 §3.3)
    _bodyStrengths[target] = level;
    notifyListeners();
    unawaited(_save());
    await _ble.writeWind(target, level);
  }

  // ── 재전송 계약 ──────────────────────────────────────────────────────────

  /// 전원 ON + 모드 + 표시 세기 전체 전송 (전원 ON, 재연결 복원, 교정 공용).
  Future<bool> _pushAll() async {
    if (!await _ble.writePower(true)) return false;
    _powerOn = true;
    return _pushModeAndWind();
  }

  /// 모드 write + 그 모드의 표시 세기 재전송 — "리모컨 = 선풍기" 보장의 핵심.
  Future<bool> _pushModeAndWind() async {
    if (!await _ble.writeMode(modeByte)) return false;
    if (_page == 1 && _bodyMode) {
      for (final e in _bodyStrengths.entries) {
        await _ble.writeWind(e.key, e.value);
      }
    } else {
      await _ble.writeWind(0x00, _page == 0 ? _basicStrength : _targetStrength);
    }
    _scheduleVerify();
    return true;
  }

  // ── 스냅샷 검증/교정 ─────────────────────────────────────────────────────

  /// 전송 burst가 가라앉은 뒤(0.7s) 스냅샷을 read해 표시값과 대조한다.
  void _scheduleVerify() {
    if (_ble.isDebugFake || !_ble.isConnected) return;
    _verifyDebounce?.cancel();
    _verifyDebounce = Timer(const Duration(milliseconds: 700), () {
      unawaited(_verifyAndRepair());
    });
  }

  Future<void> _verifyAndRepair() async {
    if (_verifying) return;
    _verifying = true;
    try {
      for (var attempt = 0;; attempt++) {
        final snap = await _ble.readSnapshot();
        if (snap == null) return;
        effectiveMode = snap.effectiveMode;
        notifyListeners();
        if (_matches(snap)) return;
        if (!_powerOn || attempt >= 1) {
          // 교정 1회로 제한 — 재전송 루프 방지. 남은 불일치는 로그로만.
          debugPrint('RPi 상태 교정 실패: $snap — 수동 확인 필요');
          return;
        }
        debugPrint('RPi 상태 불일치: $snap '
            '(기대 power=$_powerOn mode=$modeByte) → 전체 재전송');
        await _pushAll();
      }
    } finally {
      _verifying = false;
    }
  }

  /// 스냅샷이 표시값과 일치하는가. 비교는 현재 모드에 유효한 필드만 —
  /// 예: 부위 모드에선 공용 세기를 안 보내므로 RPi의 공용 세기와 앱 타겟
  /// 세기가 달라도 정상이다 (비교하면 오탐 재전송 루프가 생긴다).
  bool _matches(proto.EswSnapshot s) {
    if (s.powerOn != _powerOn) return false;
    if (!_powerOn) return true; // OFF면 나머지 설정은 다음 ON 때 밀어넣는다
    if (s.mode != modeByte) return false;
    if (_page == 1 && _bodyMode) {
      return s.head == _bodyStrengths[0x01] &&
          s.upper == _bodyStrengths[0x02] &&
          s.lower == _bodyStrengths[0x03];
    }
    return s.level == (_page == 0 ? _basicStrength : _targetStrength);
  }

  // ── RPi 발신 이벤트 ──────────────────────────────────────────────────────

  void _onStatusNotify(Uint8List value) {
    switch (value[0]) {
      case proto.statusTypeRecognition:
        if (value.length >= 2) {
          recognized = value[1] != 0;
          notifyListeners();
        }
      case proto.statusTypeEffectiveMode:
        if (value.length >= 2) {
          effectiveMode = value[1];
          notifyListeners();
        }
      case proto.statusTypeSnapshot:
        // RPi가 먼저 보낸 스냅샷 — 검증 경로로 흡수 (read 후 동일 처리).
        _scheduleVerify();
    }
  }

  void _onConnectionChanged() {
    final connected = _ble.isConnected;
    if (connected == _wasConnected) return;
    _wasConnected = connected;
    if (!connected) {
      _wasPoweredOn = _powerOn;
      // 끊김 = 양쪽 OFF 수렴 계약 (RPi도 풍속 정지 + 파킹).
      _powerOn = false;
      _clearRecognitionIfIdle();
      notifyListeners();
    }
  }

  /// 예기치 못한 끊김 후 자동 재연결 성공(30초 창) — 전원까지 완전 복원.
  Future<void> _onAutoReconnected() async {
    if (!_wasPoweredOn) return;
    debugPrint('자동 재연결 — 이전 상태 복원 (전원 ON 포함)');
    await _pushAll();
    notifyListeners();
  }

  /// 위젯 테스트가 싱글턴 상태를 초기화하기 위한 훅.
  @visibleForTesting
  void debugReset() {
    _powerOn = false;
    _page = 0;
    _rotating = false;
    _bodyMode = false;
    _basicStrength = 0;
    _targetStrength = 0;
    _bodyStrengths[0x01] = 0;
    _bodyStrengths[0x02] = 1;
    _bodyStrengths[0x03] = 0;
    recognized = null;
    effectiveMode = null;
    _wasPoweredOn = false;
  }

  /// 위젯 테스트에서 RPi Status notify를 흉내내기 위한 훅 —
  /// 실제 수신 경로(파싱 + notifyListeners)를 그대로 탄다.
  @visibleForTesting
  void debugStatusNotify(List<int> value) =>
      _onStatusNotify(Uint8List.fromList(value));

  @override
  void dispose() {
    _verifyDebounce?.cancel();
    _ble.removeListener(_onConnectionChanged);
    super.dispose();
  }
}
