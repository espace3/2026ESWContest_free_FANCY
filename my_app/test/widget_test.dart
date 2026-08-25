import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:my_app/app.dart';
import 'package:my_app/pages/basic_mode_page.dart';
import 'package:my_app/pages/target_mode_page.dart';
import 'package:my_app/services/ble/ble_connection_service.dart';

void main() {
  // 전원 버튼은 BLE 연결 전까지 비활성화되므로, 테스트에서는 실제 BLE 없이
  // "연결됨" 상태를 흉내낸다.
  BleConnectionService.instance.debugMarkConnected();

  Future<void> turnOn(WidgetTester tester) async {
    await tester.pumpWidget(const EswFanApp());
    await tester.tap(find.byIcon(Icons.power_settings_new));
    await tester.pumpAndSettle();
  }

  testWidgets('전원을 켜면 모드 화면이 나타나고, 다시 누르면 꺼진다', (tester) async {
    await tester.pumpWidget(const EswFanApp());

    expect(find.text('전원 버튼을 눌러 시작하세요'), findsOneWidget);

    await tester.tap(find.byIcon(Icons.power_settings_new));
    await tester.pumpAndSettle();

    expect(find.text('기본 모드'), findsOneWidget);
    expect(find.text('전원 버튼을 눌러 시작하세요'), findsNothing);

    // 좌상단으로 이동한 같은 버튼으로 전원 종료.
    await tester.tap(find.byIcon(Icons.power_settings_new));
    await tester.pumpAndSettle();

    expect(find.text('전원 버튼을 눌러 시작하세요'), findsOneWidget);
  });

  testWidgets('모드 화면에서 스와이프하면 타겟 모드로 전환된다', (tester) async {
    await turnOn(tester);

    await tester.fling(find.byType(PageView), const Offset(-400, 0), 1000);
    await tester.pumpAndSettle();

    expect(find.text('객체 인식 상태'), findsOneWidget);
    expect(find.text('타겟 모드'), findsOneWidget);
  });

  testWidgets('부위 인식 모드를 켜면 타겟 모드의 바람 세기가 비활성화된다', (tester) async {
    await turnOn(tester);

    await tester.fling(find.byType(PageView), const Offset(-400, 0), 1000);
    await tester.pumpAndSettle();

    await tester.tap(find.byType(Switch));
    await tester.pumpAndSettle();

    // 첫 번째 세기 선택기(타겟 모드 공용)는 비활성화, 부위별 선택기는 활성화.
    // (ListView 특성상 화면 밖 항목은 아직 빌드되지 않을 수 있다.)
    // TargetModePage 내부로 범위를 좁혀, 홈 화면 상단의 기본/타겟 전환
    // SegmentedButton과 섞이지 않게 한다.
    final selectors = tester
        .widgetList<SegmentedButton<int>>(find.descendant(
          of: find.byType(TargetModePage),
          matching: find.byType(SegmentedButton<int>),
        ))
        .toList();
    expect(selectors.first.onSelectionChanged, isNull);
    expect(selectors.length, greaterThanOrEqualTo(2));
    for (final selector in selectors.skip(1)) {
      expect(selector.onSelectionChanged, isNotNull);
    }
    expect(find.text('부위별 세기가 적용됩니다'), findsOneWidget);
  });

  testWidgets('풍량 선택기에 정지 상태가 표시된다', (tester) async {
    await turnOn(tester);

    expect(find.text('정지'), findsOneWidget);
    await tester.tap(find.text('정지'));
    await tester.pump();

    final selector = tester
        .widget<SegmentedButton<int>>(find.descendant(
          of: find.byType(BasicModePage),
          matching: find.byType(SegmentedButton<int>),
        ));
    expect(selector.selected, {0});
  });
}
