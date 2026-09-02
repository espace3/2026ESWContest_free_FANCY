import 'package:flutter/material.dart';

import '../services/fan_state_service.dart';
import '../widgets/wind_strength_selector.dart';

/// 기본 모드 화면.
///
/// 제안서 3-1: 고정 모드(회전 없음) / 회전 모드(120° 좌우 회전) 선택과
/// 바람 세기 선택을 제공한다. 값은 [FanStateService]가 소유한다 — 화면을
/// 떠났다 돌아와도, 앱을 재시작해도 유지되고(영속화), 변경 즉시 BLE로
/// 전송된다 (리모컨 주체).
class BasicModePage extends StatelessWidget {
  const BasicModePage({super.key});

  @override
  Widget build(BuildContext context) {
    final fan = FanStateService.instance;
    return ListenableBuilder(
      listenable: fan,
      builder: (context, _) => ListView(
        padding: const EdgeInsets.all(24),
        children: [
          Text('회전', style: Theme.of(context).textTheme.titleMedium),
          const SizedBox(height: 8),
          SegmentedButton<bool>(
            segments: const [
              ButtonSegment(
                  value: false, icon: Icon(Icons.gps_fixed), label: Text('고정')),
              ButtonSegment(
                  value: true, icon: Icon(Icons.threesixty), label: Text('회전')),
            ],
            selected: {fan.rotating},
            onSelectionChanged: (selection) => fan.setRotating(selection.first),
          ),
          const SizedBox(height: 32),
          Text('바람 세기', style: Theme.of(context).textTheme.titleMedium),
          const SizedBox(height: 8),
          WindStrengthSelector(
            value: fan.basicStrength,
            onChanged: (value) => fan.setBasicStrength(value),
          ),
        ],
      ),
    );
  }
}
