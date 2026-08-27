import 'package:flutter/material.dart';

import 'app.dart';
import 'services/fan_state_service.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  FanStateService.instance.init();
  await FanStateService.instance.load(); // 실패 시 기본값 (내부 try/catch)
  runApp(const EswFanApp());
}
