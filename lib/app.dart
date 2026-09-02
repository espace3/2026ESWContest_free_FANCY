import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';

import 'pages/home_page.dart';

/// 데스크톱(Windows)에서도 마우스 드래그로 스와이프가 가능하도록
/// 드래그 입력 장치에 마우스/트랙패드를 추가한다.
class _AppScrollBehavior extends MaterialScrollBehavior {
  const _AppScrollBehavior();

  @override
  Set<PointerDeviceKind> get dragDevices => {
        PointerDeviceKind.touch,
        PointerDeviceKind.mouse,
        PointerDeviceKind.trackpad,
        PointerDeviceKind.stylus,
      };
}

/// ESW 스마트 선풍기 제어 앱 루트.
///
/// 화면 흐름 (1차 마일스톤: 플레이스홀더 + 네비게이션만):
///   HomePage 단일 화면 — 전원 OFF 시 중앙 전원 버튼,
///   ON 시 버튼이 좌상단으로 이동하고 기본 모드 ↔ 타겟 모드 스와이프 전환
class EswFanApp extends StatelessWidget {
  const EswFanApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'ESW 스마트 선풍기',
      scrollBehavior: const _AppScrollBehavior(),
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.teal),
        useMaterial3: true,
      ),
      home: const HomePage(),
    );
  }
}
