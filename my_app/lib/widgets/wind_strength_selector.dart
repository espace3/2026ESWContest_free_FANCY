import 'package:flutter/material.dart';

/// 바람 세기(0: 정지, 1~3단) 선택 위젯. 기본 모드/타겟 모드/부위별 설정에서 공용.
///
/// [onChanged]가 null이면 비활성화 상태로 표시된다.
class WindStrengthSelector extends StatelessWidget {
  const WindStrengthSelector({
    super.key,
    required this.value,
    required this.onChanged,
    this.disabledLevels = const {},
  });

  static const levels = [0, 1, 2, 3];

  final int value;
  final ValueChanged<int>? onChanged;
  final Set<int> disabledLevels;

  @override
  Widget build(BuildContext context) {
    final onChanged = this.onChanged;
    final colors = Theme.of(context).colorScheme;
    return SegmentedButton<int>(
      // 선택 아이콘이 나타날 때 항목 폭이 달라지지 않도록 색으로만 상태를 표시한다.
      showSelectedIcon: false,
      segments: [
        for (final level in levels)
          ButtonSegment(
            value: level,
            enabled: !disabledLevels.contains(level),
            label: Text(level == 0 ? '정지' : '$level단'),
          ),
      ],
      selected: {value},
      style: ButtonStyle(
        backgroundColor: WidgetStateProperty.resolveWith((states) {
          if (!states.contains(WidgetState.selected)) return null;
          return states.contains(WidgetState.disabled)
              ? colors.surfaceContainerLow
              : colors.primaryContainer;
        }),
        foregroundColor: WidgetStateProperty.resolveWith((states) {
          if (!states.contains(WidgetState.selected)) return null;
          return states.contains(WidgetState.disabled)
              ? colors.onSurface.withAlpha(0x61)
              : colors.onPrimaryContainer;
        }),
      ),
      onSelectionChanged:
          onChanged == null ? null : (selection) => onChanged(selection.first),
    );
  }
}
