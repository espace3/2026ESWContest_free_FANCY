import 'package:flutter/material.dart';

import '../services/fan_state_service.dart';
import '../widgets/wind_strength_selector.dart';

/// 부위 이름 → 풍량 Characteristic의 "대상" 바이트 (ble_protocol.md 3.3).
const _bodyPartTargetByte = {'머리': 0x01, '상체': 0x02, '하체': 0x03};

/// 객체 추적(타겟) 모드 화면.
///
/// 제안서 3-2 / 4: 객체 인식 성공 여부(Status) 표시, 풍량 선택,
/// 부위 인식 모드 ON 시 머리·상체·하체 부위별 세기 설정을 제공한다.
/// 값은 [FanStateService]가 소유한다 (영속화 + 변경 즉시 전송 — 리모컨 주체).
/// 상체는 프로토콜상 정지(0)가 불가라 세기 0 버튼이 비활성화된다.
class TargetModePage extends StatelessWidget {
  const TargetModePage({super.key});

  @override
  Widget build(BuildContext context) {
    final fan = FanStateService.instance;
    final colors = Theme.of(context).colorScheme;
    return ListenableBuilder(
      listenable: fan,
      builder: (context, _) => ListView(
        padding: const EdgeInsets.all(24),
        children: [
          Text('객체 인식 상태', style: Theme.of(context).textTheme.titleMedium),
          const SizedBox(height: 8),
          // TODO(ble): 부위 러너 단계에서 fan.recognized/effectiveMode로 갱신.
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
              if (fan.bodyMode) ...[
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
            value: fan.targetStrength,
            onChanged:
                fan.bodyMode ? null : (value) => fan.setTargetStrength(value),
          ),
          const SizedBox(height: 24),
          SwitchListTile(
            title: const Text('부위 인식 모드'),
            subtitle: const Text('부위별 바람 세기 설정'),
            value: fan.bodyMode,
            onChanged: (value) => fan.setBodyMode(value),
          ),
          if (fan.bodyMode)
            for (final entry in _bodyPartTargetByte.entries)
              Padding(
                padding: const EdgeInsets.only(top: 12),
                child: Row(
                  children: [
                    SizedBox(
                      width: 36,
                      child: Text(
                        entry.key,
                        style: Theme.of(context).textTheme.titleSmall,
                      ),
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: WindStrengthSelector(
                        value: fan.bodyStrength(entry.value),
                        // 상체는 정지 불가 (ble_protocol.md 3.3)
                        disabledLevels:
                            entry.value == 0x02 ? const {0} : const {},
                        onChanged: (value) =>
                            fan.setBodyStrength(entry.value, value),
                      ),
                    ),
                  ],
                ),
              ),
        ],
      ),
    );
  }
}
