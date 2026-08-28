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
///
/// RPi가 보고하는 두 값(ble_protocol.md §3.4)을 여기서 보여준다:
///  - 객체 인식(0x02) → 상단 카드.
///  - 유효 모드(0x03) → 부위 모드인데 RPi가 추적으로 내려가 있으면(사용자
///    이동 감지) 안내 배너. 스위치는 켠 채로 두고 상태만 알린다 — 사용자가
///    끈 적이 없고, 다시 정지하면 RPi가 알아서 순찰로 돌아오기 때문.
class TargetModePage extends StatelessWidget {
  const TargetModePage({super.key});

  /// 인식 상태 카드 — recognized가 null이면 "아직 보고 없음"이다
  /// (연결 직후나 추적이 돌기 전). 미인식(false)과 구분해서 보여준다.
  Widget _buildStatusCard(BuildContext context, FanStateService fan) {
    final colors = Theme.of(context).colorScheme;
    final (icon, title, subtitle, color) = switch (fan.recognized) {
      true => (
          Icons.person,
          '인식 중',
          '대상을 추적하고 있습니다',
          colors.primary,
        ),
      false => (
          Icons.person_off_outlined,
          '대상 없음',
          '카메라에 사람이 보이지 않습니다',
          colors.outline,
        ),
      null => (
          Icons.person_search,
          '인식 대기 중',
          '카메라가 준비되면 표시됩니다',
          colors.outline,
        ),
    };
    return Card(
      color: fan.recognized == true ? colors.primaryContainer : null,
      child: ListTile(
        leading: Icon(icon, color: color),
        title: Text(title),
        subtitle: Text(subtitle),
      ),
    );
  }

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
          _buildStatusCard(context, fan),
          // 부위 모드인데 RPi가 추적으로 내려가 있는 동안만 표시.
          if (fan.bodyMode && fan.effectiveMode == 0x02) ...[
            const SizedBox(height: 8),
            Card(
              color: colors.secondaryContainer,
              child: ListTile(
                leading: Icon(Icons.directions_walk,
                    color: colors.onSecondaryContainer),
                title: const Text('이동 감지 — 추적 중'),
                subtitle: const Text('잠시 멈추면 부위별 바람으로 돌아갑니다'),
              ),
            ),
          ],
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
          // 부위별 세기는 부위 모드가 꺼져 있어도 **숨기지 않고 비활성**으로 둔다
          // — 어떤 설정이 있는지 미리 보이고, 켜면 그 값이 그대로 적용된다.
          for (final entry in _bodyPartTargetByte.entries)
            Padding(
              padding: const EdgeInsets.only(top: 12),
              child: Row(
                children: [
                  SizedBox(
                    width: 36,
                    child: Text(
                      entry.key,
                      style: Theme.of(context).textTheme.titleSmall?.copyWith(
                            color: fan.bodyMode ? null : colors.outline,
                          ),
                    ),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: WindStrengthSelector(
                      value: fan.bodyStrength(entry.value),
                      // 상체는 정지 불가 (ble_protocol.md 3.3)
                      disabledLevels:
                          entry.value == 0x02 ? const {0} : const {},
                      onChanged: fan.bodyMode
                          ? (value) => fan.setBodyStrength(entry.value, value)
                          : null,
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
