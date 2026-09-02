# -*- coding: utf-8 -*-
"""
app/ble_service.py - 모드 러너 · 모드 감독 · 연결 끊김 감지

라이브러리 전용 — 진입점은 main.py 하나다. 여기에는 모드마다 도는 러너 팩토리와
그것을 갈아끼우는 supervisor, 그리고 BLE 연결 끊김 감지가 들어 있다.

  _make_sweeper / _make_homer   기본 모드용 러너 팩토리 (카메라·디텍터 미사용)
  _ModeSupervisor               전원·모드·풍속 → 러너 선택 및 스레드 교대
  _watch_disconnects            BlueZ 연결 끊김 → 전원 OFF 처리
  _DryRelay                     --dry-run용 릴레이 스텁

아래 설계 근거는 이 파일이 독립 진입점이던 시절(verify_E2E_v2.py) 기록이다.
1~6은 지금 main.py의 EswFanServiceV3가 이어받았고, 7~9가 이 파일에 남은 부분의
근거다:

  1. FanRelay(hardware/relay_controller.py) 연결 — v1에서 print에 그치던
     WIND write가 실제 릴레이(TS0011)를 구동한다 (verify_ble_v2.py와 동일).
     릴레이 간 전환 안전(break-before-make: 전부 오픈 → guard_s 대기 →
     하나만 닫기)은 FanRelay 워커가 전담하므로 여기서는 set_speed()만 부른다.
  2. 전원 게이팅 (verify_ble_v2와 동일) — 시작 시 전원 OFF 가정. POWER ON이면
     마지막 수신 세기를 재적용, OFF면 즉시 정지(전부 오픈). 전원 OFF 중
     WIND write는 저장만 하고 돌리지 않는다. 전원/모드에 따른 추적 시작·정지는
     v1의 _TrackingSupervisor 그대로.
     부위 모드(0x03)도 같은 규칙으로 게이팅한다 — 진입 시 풍속 0(정지),
     부위 모드 중 WIND write는 저장만, 이탈 시(전원 ON이면) 해당 모드의
     저장 세기를 재적용한다. 기본/타겟 풍량도 서로 분리해 보관한다.
     부위별 풍속 중재가 미구현이라(아래 6) 부위 모드에서는 아예 돌리지 않는
     게 안전하기 때문.
  3. WIND 세기 0x00 = 풍량 정지 허용 — 프로토콜 원문(ble_protocol.md)은
     세기 1~3만 정의하지만, "전원은 켠 채 바람만 끄기"를 대비해 미리 받는다
     (앱이 아직 안 보내는 값 — 앱에서 쓰기 시작하면 ble_protocol.md도 갱신).
  4. gpiochip 핸들 공유 — MotorController가 연 핸들(mc.h)을 FanRelay에 넘긴다
     (relay_controller.py의 "상위 통합부가 한 번만 열어 나눠 준다" 규약).
     따라서 FanRelay는 mc보다 먼저 닫혀야 한다 (with 중첩 순서가 그 보장).
  5. --dry-run이면 릴레이도 스텁(_DryRelay, print만)으로 대체 — 개발 PC에서
     lgpio 없이 파이프라인 확인 가능 (FanRelay import도 그래서 지연).
     6. WIND [대상, 세기]의 대상은 모드별 preset 저장에 사용한다 —
     부위별 풍향(머리/상체/하체 개별 조준)과 대상별 풍속 중재(프로토콜
     §3.3의 실제 적용)는 미구현으로 4단계 상위 통합부 몫.
  7. 기본-회전 모드(0x01) 구현 — v1은 타겟 모드에서만 스레드를 돌렸지만,
     v2는 회전 모드에서 pan을 원점(0°) 기준 ±(--rotate-span)° 왕복 스윕한다
     (tilt는 0° 유지, 카메라/디텍터 미사용). 각도는 move_to()의 절대각 =
     위치 장부(상태 파일로 영속) 기준이므로 "0,0 기준"이 자동으로 보장된다.
     move_to는 전체 이동을 한 번에 던지면 모터 프로파일 전속으로 돌므로,
     20Hz로 목표를 실제 위치보다 --rotate-lead° 앞세워 연속으로 돌리고
     정지 신호(모드 전환/전원 OFF)에 한 틱 안에 반응하게 했다.
     회전은 풍속과 연동 — 유효 풍속 0(WIND 세기 0 write)이면 스윕도 정지,
     1~3이 다시 오면 재개한다 (바람 없이 헤드만 도는 동작 방지). 경계
     0↔1~3에서만 시작/정지하고 1→2 같은 세기 변경으로는 재시작하지 않는다.
     타겟 모드는 연동하지 않는다 — 바람이 꺼져도 조준을 유지해야 재개 시
     즉시 맞는 상태가 되기 때문.
  8. 헤드 복귀 정책 — 기본-고정(0x00)과 회전 정지(0x01 + 풍속 0)에서는
     pan을 그 자리에 두고(복귀 없음) tilt만 0°로 되돌린 뒤 대기한다
     (웜기어 틸트만 중립 복귀, 좌우 방향은 멈춘 자리 존중). 전원 OFF는
     pan/tilt 모두 0°(원점)로 복귀(파킹)한 뒤 대기한다. 이 복귀들도 러너로
     돌린다 — supervisor의 stop→join→start 순서를 타므로 죽어가는 이전 추적
     스레드가 던지는 마지막 move_to와 레이스하지 않는다. 같은 러너가 정상
     구동 중이면 재시작하지 않는다(_apply 데드업) — 앱이 같은 모드를
     재전송하거나 타겟(0x02)↔타겟-부위(0x03)를 오가도 카메라 세션이 끊기지
     않는다 (v1은 매번 세션을 재시작했다).
  9. 연결 끊김 = 전원 OFF 처리 — 앱(중앙)의 BLE 연결이 끊기면 전원 OFF
     write와 동일하게 풍속 정지 + 0°,0° 파킹한다 (마지막 상태로 계속 도는
     것 방지). bluez_peripheral에는 연결 이벤트 API가 없어서 하부 dbus_fast
     버스로 BlueZ Device1.Connected PropertiesChanged를 직접 구독한다.
     어댑터에 연결된 아무 Device1의 끊김이든 트리거된다 — 이 기기는 앱 전용
     Peripheral이라 문제없지만, 다른 BLE 기기를 같이 물리면 오탐될 수 있다.
     재연결한 앱은 프로토콜 가정대로 POWER ON부터 다시 보내면 된다
     (Advertising은 timeout=0이라 BlueZ가 끊김 후 자동 재개).

"""

from __future__ import annotations

import sys
from pathlib import Path

# 레포 루트를 path에 추가 (config / vision / control / hardware / app 해결용)
sys.path.insert(0, str(Path(__file__).parent.parent))

from dbus_fast.constants import MessageType
from dbus_fast.message import Message

from app.ble_protocol import _TARGET_MODES, _TrackingSupervisor


class _DryRelay:
    """--dry-run용 릴레이 스텁. lgpio/실릴레이 없이 BLE→풍속 게이팅 로직만
    확인하기 위한 것. FanRelay처럼 같은 목표 반복은 무시한다(로그 소음 방지)."""

    def __init__(self) -> None:
        self._level = 0

    def set_speed(self, level: int) -> None:
        if level == self._level:
            return
        self._level = level
        print(f"[relay] DRY-RUN — set_speed({level}) (실제 릴레이 미구동)")

    def __enter__(self) -> "_DryRelay":
        return self

    def __exit__(self, *exc) -> None: ...


def _make_sweeper(mc, args):
    """기본-회전 모드(0x01, 풍속 ≥1)용 러너 — pan을 원점(0°) 기준 ±span° 왕복,
    tilt는 0° 유지. 카메라/디텍터는 쓰지 않는다.

    [왜 목표를 앞세우고 반전은 실제 위치로 보는가]
    MotorController 는 목표를 따라잡으면 감속 → 큐 배출 → 정지한다. 그래서 목표가
    모터 순항 속도보다 느리게 전진하면 매 틱 "램프 상승 → 따라잡음 → 감속 → 정지 →
    재기동"을 반복해 눈에 띄게 덜컹거린다 (실기 2026-09-02).
    연속으로 돌리려면 **목표가 항상 모터보다 앞서 있어야** 한다.

    그런데 목표만 앞세우면 ±span 반전 판정이 실제 헤드 위치가 아니라 달아난 목표로
    일어나 스윕 폭이 설정값보다 좁아지고 좌우가 비대칭이 된다. 그래서 **전진은
    lead 만큼 앞세우고, 반전은 실제 장부 위치로 판정**한다.

    결과적으로 스윕 속도는 모터 순항 속도로 고정된다 — 그보다 느리게 "부드럽게"
    돌 방법이 이 드라이버 구조에는 없다 (f_max 를 낮추지 않는 한).
    팬 순항 = f_max × 360 / (steps_per_rev × microstep × gear_ratio) ≈ 7.9°/s.

    stop_event 에는 한 틱(50ms) 안에 반응한다 — 다음 목표를 안 주면 모터가 lead
    만큼 더 가고 그 자리에 감속 정지한다.
    """
    span, lead, period = args.rotate_span, args.rotate_lead, 0.05

    def _run(stop_event):
        pos = mc.current_position()[0]
        direction = -1.0 if pos > 0 else 1.0
        print(f"[E2E] 회전 스윕 시작 — pan ±{span:g}° (0° 기준, 모터 순항 속도), tilt 0°")
        while not stop_event.is_set():
            pos = mc.current_position()[0]
            # 반전은 실제 위치로 판정한다 (span 밖에서 진입해도 범위 쪽으로 걸어온다).
            if pos >= span:
                direction = -1.0
            elif pos <= -span:
                direction = 1.0
            # 목표는 항상 lead 만큼 앞 — 모터가 큐를 비우지 않아 연속으로 돈다.
            mc.move_to(pos + direction * lead, 0.0)
            stop_event.wait(period)

    return _run


def _make_homer(mc, home_pan: bool):
    """복귀 러너 (docstring 8) — tilt는 0°로, pan은 home_pan이면 0°(전원 OFF
    파킹), 아니면 그 자리 유지(기본-고정·회전 정지). 러너가 시작되는 시점은
    supervisor가 이전 스레드를 join한 뒤라 current_position()이 확정 위치다."""
    def _run(stop_event):
        pan, tilt = mc.current_position()
        pan_t = 0.0 if home_pan else pan
        if abs(tilt) > 0.01 or abs(pan_t - pan) > 0.01:
            what = "0°,0° 복귀 (파킹)" if home_pan else f"틸트 0° 복귀 (pan {pan:+.1f}° 유지)"
            print(f"[E2E] {what}")
            mc.move_to(pan_t, 0.0)
        stop_event.wait()

    return _run


class _ModeSupervisor(_TrackingSupervisor):
    """v1 supervisor 확장 — 첫 BLE write 이후에는 상태마다 항상 러너 하나가 돈다.

    전원 ON: 타겟(0x02/0x03) → 추적, 기본-회전(0x01, 풍속 ≥1) → 스윕,
    기본-고정(0x00)·회전 정지(0x01 + 풍속 0) → 틸트만 복귀 후 대기.
    전원 OFF: pan/tilt 모두 0°로 복귀(파킹) 후 대기 (docstring 8).
    회전만 풍속과 연동한다 — 타겟 모드는 바람이 꺼져도 조준을 유지해야
    재개 시 즉시 맞는 상태가 되기 때문 (docstring 7).
    """

    def __init__(self, track_fn, sweep_fn, home_fn, park_fn, stop_fn=None,
                 web_state=None) -> None:
        super().__init__(track_fn, web_state=web_state)
        self._track_fn = track_fn
        self._sweep_fn = sweep_fn
        self._home_fn = home_fn
        self._park_fn = park_fn
        self._stop_fn = stop_fn
        self._wind_level = 0

    def _select_runner(self):
        if not self._power_on:
            return self._park_fn
        if self._mode in _TARGET_MODES:
            return self._track_fn
        if self._mode == 0x01 and self._wind_level > 0:
            return self._sweep_fn
        return self._home_fn

    def _target_active(self) -> bool:
        return True   # 어느 상태든 러너가 하나 돈다 (_select_runner가 고른다)

    def _apply(self) -> None:
        run_fn = self._select_runner()
        # 같은 러너가 정상 구동 중(정지 요청 없음)이면 재시작하지 않는다 —
        # 앱이 같은 모드를 재전송하거나 0x02↔0x03을 오갈 때, 풍속 1→2 변경 때
        # 카메라/스윕 세션이 불필요하게 끊기는 것 방지 (docstring 8).
        if (run_fn is self._run_fn and self._thread is not None
                and self._thread.is_alive() and not self._stop_event.is_set()):
            return
        if run_fn is not self._run_fn and self._thread is not None \
                and self._thread.is_alive():
            # 이전 runner가 다음 50ms tick까지 새 목표를 던지지 못하도록
            # 먼저 중단 신호와 모터 감속 정지를 전달한 뒤 join한다.
            self._stop_event.set()
            if self._stop_fn is not None:
                self._stop_fn()
        self._run_fn = run_fn
        super()._apply()

    def set_state(self, power_on: bool, mode: int, wind_level: int) -> None:
        """power/mode/wind를 한 상태로 갱신한 뒤 runner를 재선택한다."""
        self._power_on = power_on
        self._mode = mode
        self._wind_level = wind_level
        self._apply()

    def set_wind(self, level: int) -> None:
        """풍속 변화 통지 — 러너가 안 바뀌면 _apply 데드업이 무시한다."""
        self.set_state(self._power_on, self._mode, level)



async def _watch_disconnects(bus, service) -> None:
    """BlueZ Device1.Connected 프로퍼티 변경을 구독해 연결 끊김을 감지한다
    (docstring 9 — bluez_peripheral엔 연결 이벤트 API가 없어 D-Bus 직접 구독).

    service 는 handle_disconnect() 를 가진 GATT 서비스면 된다 (main.py 의
    EswFanServiceV3)."""
    await bus.call(Message(
        destination="org.freedesktop.DBus", path="/org/freedesktop/DBus",
        interface="org.freedesktop.DBus", member="AddMatch", signature="s",
        body=["type='signal',interface='org.freedesktop.DBus.Properties',"
              "member='PropertiesChanged',arg0='org.bluez.Device1'"]))

    def _handler(msg):
        if (msg.message_type != MessageType.SIGNAL
                or msg.interface != "org.freedesktop.DBus.Properties"
                or msg.member != "PropertiesChanged"):
            return
        iface, changed, _ = msg.body
        if iface != "org.bluez.Device1" or "Connected" not in changed:
            return
        dev = msg.path.rsplit("/", 1)[-1]  # dev_XX_XX_... (MAC)
        if changed["Connected"].value:
            print(f"[BLE] 중앙 연결됨 ({dev})")
        else:
            print(f"[BLE] 중앙 연결 끊김 ({dev})")
            service.handle_disconnect()

    bus.add_message_handler(_handler)
