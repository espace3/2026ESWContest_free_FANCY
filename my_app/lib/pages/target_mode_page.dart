import 'package:flutter/material.dart';

import '../services/ble/ble_connection_service.dart';
import '../widgets/wind_strength_selector.dart';

/// 부위 이름 → 풍량 Characteristic의 "대상" 바이트 (ble_protocol.md 3.3).
const _bodyPartTargetByte = {'머리': 0x01, '상체': 0x02, '하체': 0x03};

/// 객체 추적(타겟) 모드 화면 (플레이스홀더).
///
/// 제안서 3-2 / 4: 객체 인식 성공 여부(Status) 표시, 풍량 선택,
/// 부위 인식 모드 ON 시 머리·상체·하체 부위별 세기 설정을 제공한다.
/// Status와 명령 전송은 BLE 연동 시 구현하며 지금은 로컬 상태만 가진다.
class TargetModePage extends StatefulWidget {
  const TargetModePage({super.key});

  @override
  State<TargetModePage> createState() => _TargetModePageState();
}

class _TargetModePageState extends State<TargetModePage> {
  int _windStrength = 0;
  bool _bodyMode = false;
  final _bodyStrengths = <String, int>{'머리': 0, '상체': 0, '하체': 0};

  @override
  Widget build(BuildContext context) {
    final colors = Theme.of(context).colorScheme;
    return ListView(
      padding: const EdgeInsets.all(24),
      children: [
        Text('객체 인식 상태', style: Theme.of(context).textTheme.titleMedium),
        const SizedBox(height: 8),
        // TODO(ble): RPi에서 Status Characteristic 알림을 받아 갱신.
        Card(
          child: ListTile(
            leading: Icon(Icons.person_search, color: colors.outline),
            title: const Text('인식 대기 중'),
            subtitle: const Text('BLE 연동 후 표시됩니다'),
          ),
        ),
        const SizedBox(height: 24),
        Row(
          crossAxisAlignment: CrossAxisAlignment.baseline,
          textBaseline: TextBaseline.alphabetic,
          children: [
            Text('바람 세기', style: Theme.of(context).textTheme.titleMedium),
            if (_bodyMode) ...[
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  '부위별 세기가 적용됩니다',
                  textAlign: TextAlign.end,
                  style: Theme.of(context)
                      .textTheme
                      .bodySmall
                      ?.copyWith(color: colors.outline),
                ),
              ),
            ],
          ],
        ),
        const SizedBox(height: 8),
        WindStrengthSelector(
          value: _windStrength,
          onChanged: _bodyMode
              ? null
              : (value) {
                  setState(() => _windStrength = value);
                  BleConnectionService.instance.writeWind(0x00, value);
                },
        ),
        const SizedBox(height: 24),
        SwitchListTile(
          title: const Text('부위 인식 모드'),
          subtitle: const Text('부위별 바람 세기 설정'),
          value: _bodyMode,
          onChanged: (value) {
            setState(() => _bodyMode = value);
            BleConnectionService.instance.writeMode(value ? 0x03 : 0x02);
          },
        ),
        if (_bodyMode)
          for (final part in _bodyStrengths.keys)
            Padding(
              padding: const EdgeInsets.only(top: 12),
              child: Row(
                children: [
                  SizedBox(
                    width: 36,
                    child: Text(
                      part,
                      style: Theme.of(context).textTheme.titleSmall,
                    ),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: WindStrengthSelector(
                      value: _bodyStrengths[part]!,
                      disabledLevels: part == '상체' ? {0} : const {},
                      onChanged: (value) {
                        setState(() => _bodyStrengths[part] = value);
                        BleConnectionService.instance
                            .writeWind(_bodyPartTargetByte[part]!, value);
                      },
                    ),
                  ),
                ],
              ),
            ),
      ],
    );
  }
}
