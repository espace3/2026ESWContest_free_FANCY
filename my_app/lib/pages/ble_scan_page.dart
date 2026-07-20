import 'dart:async';

import 'package:flutter/material.dart';
import 'package:universal_ble/universal_ble.dart';

import '../services/ble/ble_connection_service.dart';
import '../services/ble/ble_protocol.dart' as proto;

/// BLE 스캔/연결 디버그 페이지 (ble_todo 1단계 스파이크).
///
/// - 스캔 결과는 전부 리스트에 표시하되, **연결은 우리 서비스 UUID를
///   광고하는 기기(ESW-FAN)에만 허용**한다 — 타 기기는 표시 전용.
/// - 연결 후 서비스 발견 결과와 이벤트를 하단 로그 패널에 출력하고,
///   전원/모드/풍량 write 테스트 버튼을 제공한다.
/// - 연결 상태 자체는 [BleConnectionService](앱 전역 싱글턴)가 들고 있어서,
///   이 페이지를 나가도(뒤로가기) 연결이 끊기지 않는다.
class BleScanPage extends StatefulWidget {
  const BleScanPage({super.key});

  @override
  State<BleScanPage> createState() => _BleScanPageState();
}

class _BleScanPageState extends State<BleScanPage> {
  final _devices = <String, BleDevice>{}; // deviceId → 최신 스캔 결과
  final _logs = <String>[];
  final _logScroll = ScrollController();
  final _ble = BleConnectionService.instance;

  StreamSubscription<BleDevice>? _scanSub;
  bool _scanning = false;
  bool _busy = false; // 연결/전송 중 중복 조작 방지

  @override
  void initState() {
    super.initState();
    _ble.addListener(_onConnectionChanged);
    _logAvailability();
  }

  @override
  void dispose() {
    _ble.removeListener(_onConnectionChanged);
    _scanSub?.cancel();
    if (_scanning) UniversalBle.stopScan();
    // 연결 자체는 앱 전역 상태이므로 페이지를 나가도 끊지 않는다.
    _logScroll.dispose();
    super.dispose();
  }

  void _onConnectionChanged() {
    if (mounted) setState(() {});
  }

  void _log(String message) {
    if (!mounted) return;
    setState(() => _logs.add(message));
    // 프레임 반영 후 로그 맨 아래로.
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_logScroll.hasClients) {
        _logScroll.jumpTo(_logScroll.position.maxScrollExtent);
      }
    });
  }

  Future<void> _logAvailability() async {
    try {
      final state = await UniversalBle.getBluetoothAvailabilityState();
      _log('블루투스 상태: ${state.name}');
    } catch (e) {
      _log('블루투스 상태 확인 실패: $e');
    }
  }

  /// 우리 기기 여부 — 광고에 포함된 서비스 UUID로 판별 (이름은 보조).
  bool _isEswDevice(BleDevice device) =>
      device.services.any((s) => s.toLowerCase() == proto.serviceUuid) ||
      device.name == proto.eswLocalName;

  Future<void> _toggleScan() async {
    if (_scanning) {
      await UniversalBle.stopScan();
      await _scanSub?.cancel();
      _scanSub = null;
      setState(() => _scanning = false);
      _log('스캔 중지 (발견 ${_devices.length}대)');
      return;
    }
    // Android 12+는 BLUETOOTH_SCAN/CONNECT를 런타임에도 받아야 스캔이 된다.
    // Windows/Linux/Web에서는 두 API 모두 항상 성공하므로 분기 없이 호출한다.
    if (!await _ensurePermissions()) return;
    setState(() {
      _devices.clear();
      _scanning = true;
    });
    _log('스캔 시작...');
    _scanSub = UniversalBle.scanStream.listen(
      (device) => setState(() => _devices[device.deviceId] = device),
      onError: (Object e) {
        _log('스캔 오류: $e');
        setState(() => _scanning = false);
      },
    );
    try {
      await UniversalBle.startScan();
    } catch (e) {
      _log('스캔 시작 실패: $e');
      setState(() => _scanning = false);
    }
  }

  /// 권한이 없으면 요청한다. 사용자가 거부하면 예외가 나므로 안내 후 false.
  Future<bool> _ensurePermissions() async {
    try {
      if (!await UniversalBle.hasPermissions()) {
        _log('블루투스 권한 요청...');
        await UniversalBle.requestPermissions();
      }
      return true;
    } catch (e) {
      _log('블루투스 권한 거부됨: $e');
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('블루투스 권한이 필요합니다 — 설정에서 허용해 주세요')),
        );
      }
      return false;
    }
  }

  Future<void> _connect(BleDevice device) async {
    if (_busy) return;
    setState(() => _busy = true);
    try {
      if (_scanning) await _toggleScan();
      _log('연결 시도: ${device.name ?? device.deviceId}');
      await _ble.connect(device);
      _log('연결 성공 — 서비스 발견 중...');
      final services = await UniversalBle.discoverServices(device.deviceId);
      for (final s in services) {
        final mark = s.uuid.toLowerCase() == proto.serviceUuid ? ' ← ESW' : '';
        _log('서비스 ${s.uuid}$mark');
        for (final c in s.characteristics) {
          _log('  └ ${c.uuid} [${c.properties.map((p) => p.name).join(',')}]');
        }
      }
    } catch (e) {
      _log('연결 실패: $e');
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  Future<void> _disconnect() async {
    try {
      await _ble.disconnect();
      _log('연결 해제');
    } catch (e) {
      _log('연결 해제 실패: $e');
    }
  }

  Widget _buildDeviceTile(BleDevice device) {
    final isEsw = _isEswDevice(device);
    final connected = device.deviceId == _ble.deviceId;
    return ListTile(
      dense: true,
      leading: Icon(
        isEsw ? Icons.air : Icons.bluetooth,
        color: isEsw ? Theme.of(context).colorScheme.primary : null,
      ),
      title: Text(device.name?.isNotEmpty == true ? device.name! : '(이름 없음)'),
      subtitle: Text('${device.deviceId}  RSSI ${device.rssi ?? '-'}'),
      // 연결은 ESW 기기에만 허용 — 타 기기는 표시 전용.
      trailing: !isEsw
          ? null
          : connected
              ? FilledButton.tonal(
                  onPressed: _busy ? null : _disconnect,
                  child: const Text('해제'),
                )
              : FilledButton(
                  onPressed: _busy ? null : () => _connect(device),
                  child: const Text('연결'),
                ),
    );
  }

  Widget _buildTestButtons() {
    Widget button(String label, Future<void> Function() action) =>
        OutlinedButton(
          onPressed: _busy
              ? null
              : () async {
                  await action();
                  _log('[TX] $label');
                },
          child: Text(label),
        );
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 12),
      child: Wrap(
        spacing: 8,
        runSpacing: 4,
        children: [
          button('전원 ON', () => _ble.writePower(true)),
          button('전원 OFF', () => _ble.writePower(false)),
          button('모드: 회전', () => _ble.writeMode(0x01)),
          button('모드: 타겟', () => _ble.writeMode(0x02)),
          button('풍량: 공용 2단', () => _ble.writeWind(0x00, 0x02)),
          button('풍량: 머리 3단', () => _ble.writeWind(0x01, 0x03)),
        ],
      ),
    );
  }

  /// 현재 연결된 기기 고정 카드 — 연결 중인 기기는 광고를 멈춰 스캔 목록에
  /// 안 잡히므로, 목록과 별개로 항상 최상단에 표시해 해제 진입점을 보장한다.
  Widget _buildConnectedCard() {
    final colors = Theme.of(context).colorScheme;
    return Card(
      margin: const EdgeInsets.fromLTRB(8, 8, 8, 0),
      color: colors.primaryContainer,
      child: ListTile(
        dense: true,
        leading: Icon(Icons.bluetooth_connected, color: colors.onPrimaryContainer),
        title: Text('연결됨: ${_ble.deviceName ?? proto.eswLocalName}'),
        subtitle: Text(_ble.deviceId ?? ''),
        trailing: FilledButton.tonal(
          onPressed: _busy ? null : _disconnect,
          child: const Text('해제'),
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final devices = _devices.values
        // 연결된 기기는 위 고정 카드가 담당 — 목록과 중복 표시 방지.
        .where((d) => d.deviceId != _ble.deviceId)
        .toList()
      ..sort((a, b) {
        // ESW 기기 우선, 그다음 신호 세기순.
        final esw = (_isEswDevice(b) ? 1 : 0) - (_isEswDevice(a) ? 1 : 0);
        if (esw != 0) return esw;
        return (b.rssi ?? -999).compareTo(a.rssi ?? -999);
      });
    return Scaffold(
      appBar: AppBar(
        title: const Text('BLE 연결 (디버그)'),
        actions: [
          TextButton.icon(
            onPressed: _busy ? null : _toggleScan,
            icon: Icon(_scanning ? Icons.stop : Icons.search),
            label: Text(_scanning ? '중지' : '스캔'),
          ),
        ],
      ),
      body: Column(
        children: [
          if (_scanning) const LinearProgressIndicator(),
          if (_ble.isConnected) _buildConnectedCard(),
          Expanded(
            flex: 3,
            child: devices.isEmpty
                ? Center(
                    child: Text(
                      _scanning ? '기기 검색 중...' : '스캔을 시작하세요',
                      style: Theme.of(context).textTheme.bodyMedium,
                    ),
                  )
                : ListView.builder(
                    itemCount: devices.length,
                    itemBuilder: (context, i) => _buildDeviceTile(devices[i]),
                  ),
          ),
          if (_ble.isConnected) _buildTestButtons(),
          const Divider(height: 1),
          Expanded(
            flex: 2,
            child: Container(
              width: double.infinity,
              color: Theme.of(context).colorScheme.surfaceContainerHighest,
              child: ListView.builder(
                controller: _logScroll,
                padding: const EdgeInsets.all(8),
                itemCount: _logs.length,
                itemBuilder: (context, i) => Text(
                  _logs[i],
                  style: Theme.of(context).textTheme.bodySmall?.copyWith(
                        fontFamily: 'monospace',
                      ),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}
