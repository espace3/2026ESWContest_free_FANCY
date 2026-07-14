import 'package:flutter/material.dart';

/// 바람 세기(1~3단) 선택 위젯. 기본 모드/타겟 모드/부위별 설정에서 공용.
///
/// [onChanged]가 null이면 비활성화 상태로 표시된다.
class WindStrengthSelector extends StatelessWidget {
  const WindStrengthSelector({
    super.key,
    required this.value,
    required this.onChanged,
  });

  static const levels = [1, 2, 3];

  final int value;
  final ValueChanged<int>? onChanged;

  @override
  Widget build(BuildContext context) {
    final onChanged = this.onChanged;
    return SegmentedButton<int>(
      segments: [
        for (final level in levels)
          ButtonSegment(value: level, label: Text('$level단')),
      ],
      selected: {value},
      onSelectionChanged:
          onChanged == null ? null : (selection) => onChanged(selection.first),
    );
  }
}
