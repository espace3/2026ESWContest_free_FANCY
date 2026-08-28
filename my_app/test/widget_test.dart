import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:my_app/app.dart';
import 'package:my_app/pages/basic_mode_page.dart';
import 'package:my_app/pages/target_mode_page.dart';
import 'package:my_app/services/ble/ble_connection_service.dart';
import 'package:my_app/services/ble/ble_protocol.dart' as proto;
import 'package:my_app/services/fan_state_service.dart';

void main() {
  // 전원 버튼은 BLE 연결 전까지 비활성화되므로, 테스트에서는 실제 BLE 없이
  // "연결됨" 상태를 흉내낸다.
  BleConnectionService.instance.debugMarkConnected();

  // FanStateService는 싱글턴이라 테스트 간 상태가 새지 않게 매번 초기화한다.
  setUp(() {
    FanStateService.instance.debugReset();
    BleConnectionService.instance.debugWrites.clear();
  });

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

  testWidgets('전원 ON과 모드 전환 시 표시값을 재전송한다 (리모컨 주체)', (tester) async {
    final writes = BleConnectionService.instance.debugWrites;
    await turnOn(tester);

    // 전원 ON: power → mode(기본-고정) → 표시 세기 재전송 (ble_protocol.md 3.3).
    expect(writes, [
      [proto.charNoPower, 0x01],
      [proto.charNoMode, 0x00],
      [proto.charNoWind, 0x00, 0x00],
    ]);

    writes.clear();
    await tester.fling(find.byType(PageView), const Offset(-400, 0), 1000);
    await tester.pumpAndSettle();

    // 타겟 모드 전환: mode(0x02) + 타겟 화면의 표시 세기 재전송.
    expect(writes, [
      [proto.charNoMode, 0x02],
      [proto.charNoWind, 0x00, 0x00],
    ]);

    writes.clear();
    await tester.tap(find.byType(Switch));
    await tester.pumpAndSettle();

    // 부위 모드 진입: mode(0x03) + 부위별 세기 3건 (상체 기본 1단).
    expect(writes, [
      [proto.charNoMode, 0x03],
      [proto.charNoWind, 0x01, 0x00],
      [proto.charNoWind, 0x02, 0x01],
      [proto.charNoWind, 0x03, 0x00],
    ]);
  });

  /// 타겟 모드 화면까지 이동 (전원 ON → 오른쪽 페이지로 스와이프).
  Future<void> goToTarget(WidgetTester tester) async {
    await turnOn(tester);
    await tester.fling(find.byType(PageView), const Offset(-400, 0), 1000);
    await tester.pumpAndSettle();
  }

  List<SegmentedButton<int>> targetSelectors(WidgetTester tester) => tester
      .widgetList<SegmentedButton<int>>(find.descendant(
        of: find.byType(TargetModePage),
        matching: find.byType(SegmentedButton<int>),
      ))
      .toList();

  testWidgets('부위별 세기는 부위 모드가 꺼져 있어도 보이고 비활성이다', (tester) async {
    await goToTarget(tester);

    expect(find.text('머리'), findsOneWidget);
    expect(find.text('상체'), findsOneWidget);
    expect(find.text('하체'), findsOneWidget);

    final selectors = targetSelectors(tester);
    expect(selectors.length, 4); // 공용 1 + 부위 3
    expect(selectors.first.onSelectionChanged, isNotNull); // 공용은 활성
    for (final selector in selectors.skip(1)) {
      expect(selector.onSelectionChanged, isNull); // 부위 행은 비활성
    }
  });

  testWidgets('인식 상태 카드가 RPi 보고를 반영한다', (tester) async {
    await goToTarget(tester);
    expect(find.text('인식 대기 중'), findsOneWidget); // 보고 전(null)

    // 실제 notify 경로로 주입 (ble_protocol.md §3.4 타입 0x02).
    FanStateService.instance
        .debugStatusNotify([proto.statusTypeRecognition, 0x01]);
    await tester.pump();
    expect(find.text('인식 중'), findsOneWidget);

    FanStateService.instance
        .debugStatusNotify([proto.statusTypeRecognition, 0x00]);
    await tester.pump();
    expect(find.text('대상 없음'), findsOneWidget);
  });

  testWidgets('부위 모드 중 유효 모드가 추적이면 이동 감지 배너가 뜬다', (tester) async {
    await goToTarget(tester);
    await tester.tap(find.byType(Switch));
    await tester.pumpAndSettle();
    expect(find.text('이동 감지 — 추적 중'), findsNothing);

    // RPi가 이동을 감지해 추적으로 내려감 (유효 모드 push 0x03).
    FanStateService.instance
        .debugStatusNotify([proto.statusTypeEffectiveMode, 0x02]);
    await tester.pump();
    expect(find.text('이동 감지 — 추적 중'), findsOneWidget);

    // 다시 정지해 순찰로 복귀하면 배너가 사라진다.
    FanStateService.instance
        .debugStatusNotify([proto.statusTypeEffectiveMode, 0x03]);
    await tester.pump();
    expect(find.text('이동 감지 — 추적 중'), findsNothing);
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
