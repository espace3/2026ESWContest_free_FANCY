import 'package:flutter/material.dart';

import '../services/ble/ble_connection_service.dart';
import '../widgets/wind_strength_selector.dart';

/// 기본 모드 화면 (플레이스홀더).
///
/// 제안서 3-1: 고정 모드(회전 없음) / 회전 모드(120° 좌우 회전) 선택과
/// 바람 세기 선택을 제공한다. 값은 아직 로컬 상태이며 BLE 전송은 추후 구현.
class BasicModePage extends StatefulWidget {
  const BasicModePage({super.key});

  @override
  State<BasicModePage> createState() => _BasicModePageState();
}

class _BasicModePageState extends State<BasicModePage> {
  bool _rotating = false;
  int _windStrength = 1;

  @override
  Widget build(BuildContext context) {
    return ListView(
      padding: const EdgeInsets.all(24),
      children: [
        Text('회전', style: Theme.of(context).textTheme.titleMedium),
        const SizedBox(height: 8),
        SegmentedButton<bool>(
          segments: const [
            ButtonSegment(value: false, icon: Icon(Icons.gps_fixed), label: Text('고정')),
            ButtonSegment(value: true, icon: Icon(Icons.threesixty), label: Text('회전')),
          ],
          selected: {_rotating},
          onSelectionChanged: (selection) {
            setState(() => _rotating = selection.first);
            BleConnectionService.instance.writeMode(_rotating ? 0x01 : 0x00);
          },
        ),
        const SizedBox(height: 32),
        Text('바람 세기', style: Theme.of(context).textTheme.titleMedium),
        const SizedBox(height: 8),
        WindStrengthSelector(
          value: _windStrength,
          onChanged: (value) {
            setState(() => _windStrength = value);
            BleConnectionService.instance.writeWind(0x00, value);
          },
        ),
      ],
    );
  }
}
