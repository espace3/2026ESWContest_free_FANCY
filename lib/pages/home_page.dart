import 'package:flutter/material.dart';

import '../services/ble/ble_connection_service.dart';
import '../services/fan_state_service.dart';
import 'basic_mode_page.dart';
import 'ble_scan_page.dart';
import 'target_mode_page.dart';

/// 단일 홈 화면.
///
/// - 전원 OFF: 중앙에 큰 전원 버튼과 안내 문구.
/// - 전원 ON: 전원 버튼이 좌상단으로 이동·축소되고, 같은 버튼으로 전원을 끈다.
///   본문에는 기본 모드 ↔ 타겟 모드 화면이 표시되며 좌우 스와이프
///   또는 하단 인디케이터 클릭으로 전환한다.
///
/// 전원/페이지 상태는 [FanStateService]가 소유한다 (리모컨 주체 + 재연결
/// 복원을 위해 위젯 밖에 있어야 함) — 이 위젯은 표시와 입력 전달만 한다.
class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> with WidgetsBindingObserver {
  static const _modeNames = ['기본 모드', '타겟 모드'];
  static const _switchDuration = Duration(milliseconds: 350);
  // TODO(temporary): BLE 없이 조작 화면을 확인할 때만 true로 둔다.
  static const _allowOfflinePreview = false;

  final _pageController = PageController();
  final _ble = BleConnectionService.instance;
  final _fan = FanStateService.instance;
  bool _wasConnected = false;

  @override
  void initState() {
    super.initState();
    _wasConnected = _ble.isConnected;
    _ble.addListener(_onConnectionChanged);
    _fan.addListener(_onFanChanged);
    WidgetsBinding.instance.addObserver(this);
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _ble.removeListener(_onConnectionChanged);
    _fan.removeListener(_onFanChanged);
    _pageController.dispose();
    super.dispose();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    // 앱을 최소화(back/home)하거나 종료할 때, **전원이 꺼져 있을 때만** BLE를
    // 끊는다. 선풍기가 켜진 채 최소화한 경우는 연결을 유지해, 돌아왔을 때
    // 매번 재연결하는 불편을 없앤다. 전원 OFF 상태에서 정리해두면 RPi가
    // 죽은 연결을 붙들지 않아 다음 연결의 10초 타임아웃도 예방된다.
    // (켜진 채 OS가 앱을 강제 종료하는 경우는 connect()의 자동 재시도가 흡수.)
    // 화면 간 이동(스캔↔홈)은 lifecycle 이벤트가 아니라 영향 없다.
    if ((state == AppLifecycleState.paused ||
            state == AppLifecycleState.detached) &&
        !_fan.powerOn) {
      _ble.disconnect();
    }
  }

  void _onConnectionChanged() {
    if (!mounted) return;
    final connected = _ble.isConnected;
    if (connected != _wasConnected) {
      _wasConnected = connected;
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text(connected ? 'BLE 연결 완료' : 'BLE 연결이 해제되었습니다')),
      );
    }
    // 끊김 시 전원 OFF 복귀는 FanStateService가 처리 — _onFanChanged로 반영됨.
    setState(() {});
  }

  void _onFanChanged() {
    if (!mounted) return;
    setState(() {});
    // 재연결 복원 등 외부에서 페이지가 바뀐 경우 PageView를 따라가게 한다.
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted || !_pageController.hasClients) return;
      final visible = _pageController.page?.round() ?? 0;
      if (visible != _fan.page) _pageController.jumpToPage(_fan.page);
    });
  }

  Future<void> _togglePower() async {
    final next = !_fan.powerOn;
    if (!_ble.isConnected && _allowOfflinePreview) {
      _fan.setPowerLocal(next);
      return;
    }
    // 전송이 실제로 성공했을 때만 상태가 바뀐다(FanStateService) — 실패를
    // 조용히 넘기면 "화면은 켜졌는데 선풍기는 꺼져 있는" 불일치가 생긴다.
    final ok = await _fan.setPower(next);
    if (!mounted || ok) return;
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('전원 명령 전송 실패 — BLE 연결을 확인하세요')),
    );
  }

  /// 스와이프든 상단 토글이든, 페이지가 실제로 바뀌면 여기로 모여
  /// 기본/타겟 모드 write + 표시 세기 재전송이 나간다 (FanStateService).
  void _onPageChanged(int index) {
    _fan.setPage(index);
  }

  void _goToPage(int index) {
    _pageController.animateToPage(
      index,
      duration: const Duration(milliseconds: 250),
      curve: Curves.easeInOut,
    );
  }

  Widget _buildPowerButton(BuildContext context) {
    final colors = Theme.of(context).colorScheme;
    return AnimatedAlign(
      duration: _switchDuration,
      curve: Curves.easeInOutCubic,
      alignment: _fan.powerOn ? Alignment.topLeft : Alignment.center,
      child: Padding(
        // 좌상단에 붙었을 때 창 모서리와의 여백 (스케일 영향을 받지 않도록 바깥에 둠).
        padding: const EdgeInsets.all(12),
        child: AnimatedScale(
          duration: _switchDuration,
          curve: Curves.easeInOutCubic,
          scale: _fan.powerOn ? 0.3 : 1,
          alignment: Alignment.topLeft,
          child: IconButton.filled(
            tooltip: _ble.isConnected
                ? (_fan.powerOn ? '전원 끄기' : '전원 켜기')
                : (_allowOfflinePreview ? '오프라인 미리보기' : 'BLE 연결 필요'),
            onPressed: _ble.isConnected || _allowOfflinePreview
                ? _togglePower
                : null,
            iconSize: 96,
            padding: const EdgeInsets.all(32),
            icon: const Icon(Icons.power_settings_new),
            style: IconButton.styleFrom(
              backgroundColor: colors.primaryContainer,
              foregroundColor: colors.onPrimaryContainer,
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildOffContent(BuildContext context) {
    return Column(
      key: const ValueKey('off'),
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        Text('ESW 스마트 선풍기', style: Theme.of(context).textTheme.headlineSmall),
        const SizedBox(height: 8),
        Text(
          '전원 버튼을 눌러 시작하세요',
          style: Theme.of(context).textTheme.bodyMedium,
        ),
        // 중앙의 전원 버튼(별도 레이어)이 차지할 공간.
        const SizedBox(height: 240),
        ActionChip(
          avatar: Icon(
            _ble.isConnected ? Icons.bluetooth_connected : Icons.bluetooth_disabled,
            size: 18,
          ),
          label: Text(_ble.isConnected ? 'BLE 연결됨' : 'BLE 연결 설정'),
          onPressed: () => Navigator.of(context).push(
            MaterialPageRoute(builder: (_) => const BleScanPage()),
          ),
        ),
      ],
    );
  }

  Widget _buildOnContent(BuildContext context) {
    return Column(
      key: const ValueKey('on'),
      children: [
        // 좌상단 전원 버튼(약 48px + 여백 12px)과 같은 줄의 모드 전환 토글.
        // 스와이프로도 전환되지만, 이 토글이 "지금 화면"과 BLE 모드 write를
        // 명시적으로 일치시키는 주된 진입점이다 (스와이프만으로는 명령이
        // 안 나가던 문제를 여기서 해소).
        SizedBox(
          height: 72,
          child: Center(
            child: SegmentedButton<int>(
              segments: [
                for (var i = 0; i < _modeNames.length; i++)
                  ButtonSegment(value: i, label: Text(_modeNames[i])),
              ],
              selected: {_fan.page},
              onSelectionChanged: (selection) => _goToPage(selection.first),
            ),
          ),
        ),
        Expanded(
          child: PageView(
            controller: _pageController,
            onPageChanged: _onPageChanged,
            children: const [
              BasicModePage(),
              TargetModePage(),
            ],
          ),
        ),
        Padding(
          padding: const EdgeInsets.only(bottom: 16),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              for (var i = 0; i < _modeNames.length; i++)
                InkWell(
                  borderRadius: BorderRadius.circular(8),
                  onTap: () => _goToPage(i),
                  child: Padding(
                    padding: const EdgeInsets.all(8),
                    child: AnimatedContainer(
                      duration: const Duration(milliseconds: 200),
                      width: _fan.page == i ? 24 : 8,
                      height: 8,
                      decoration: BoxDecoration(
                        color: _fan.page == i
                            ? Theme.of(context).colorScheme.primary
                            : Theme.of(context).colorScheme.outlineVariant,
                        borderRadius: BorderRadius.circular(4),
                      ),
                    ),
                  ),
                ),
            ],
          ),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Stack(
          children: [
            // Positioned.fill로 화면 전체를 채워야 콘텐츠가 가로 중앙 정렬된다
            // (Stack의 기본 정렬은 좌상단).
            Positioned.fill(
              child: AnimatedSwitcher(
                duration: _switchDuration,
                child: _fan.powerOn
                    ? _buildOnContent(context)
                    : _buildOffContent(context),
              ),
            ),
            _buildPowerButton(context),
          ],
        ),
      ),
    );
  }
}
