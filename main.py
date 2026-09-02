#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py - 진입점: BLE GATT 서비스 + 전체 조립

  EswFanService  앱의 write 를 받는 GATT 서비스 (전원/모드/풍량/STATUS)
  _ble_main      BLE 부팅 — 페어링 에이전트, 서비스 등록, Advertising
  main()            CLI 인자 → 모터·릴레이·카메라·러너·supervisor 조립

모드마다 도는 러너와 그 교대는 app/runners.py, 카메라·시각화는 app/camera.py,
추적 루프는 app/tracking.py, 부위 시나리오는 control/body_wind.py 에 있다.

설계 근거 (전부 실기 검증에서 나온 것이다):

  1. 리모컨(앱) = 설정의 주체 — 세기는 오직 풍량 write로만 바뀐다. 모드/전원
     전환은 게이팅(전원 OFF·부위 모드 → 0)만 하고, 저장해둔 세기를 되살리지
     않는다. 모드 전환 직후의 세기는 앱이 화면 표시값을 재전송해 맞춘다
     (계약 — docs/ble_protocol.md §3.3). RPi가 모드별 세기를 기억했다 자동
     재적용하던 방식은 앱 화면과 실제 동작이 어긋나는 원인이라 폐기했다.
     앱 미갱신 과도기에는 전환 시 직전 세기가 그대로 유지된다.
  2. STATUS(0x0005) — read | notify (docs/ble_protocol.md §3.4).
     read: 0x04 스냅샷 [전원, 요청 모드, 유효 모드, 공용 세기, 머리, 상체, 하체]
       — 앱이 재전송 직후 표시값과 대조하는 불일치 검증용 (주체는 앱이므로
       앱이 이 값을 따라가지 않는다. 다르면 앱이 재전송으로 교정).
     notify: 적용된 write마다 에코백 [0x01, char#, 원본...]. 거부된 write는
       에코 없음 — 앱이 타임아웃으로 감지한다. 유효 모드 push(0x03)와 객체
       인식 Status(0x02)는 부위 러너가 보낸다 — 요청/유효 모드가 갈라지는
       지점이 거기라서.
  3. 풍량 세기 0(정지)은 공용·부위 어느 대상에도 허용한다. 부위 세 개가 모두
     0이어도 유효한 설정이며(사용자가 바람만 끈 상태), 그때 부위 러너는 조준을
     계속하되 릴레이를 돌리지 않는다 — control/body_wind.py _route 참고.
  4. 유효 모드(_effective_mode) — 요청 모드(앱이 write한 값)와 별개로 RPi가
     실제로 돌리는 모드. 부위 러너의 추적 폴백 중에만 갈라진다(아래 5).
  5. 부위 모드(0x03) — 단일 세션 러너(app/runners.py _make_body_runner)가
     내부 2상으로 돈다.
       순찰(patrol): BodyPatrolScenario(control/body_wind.py)가 head→upper→
         lower 순회(세기 0 부위 제외). 부위 각도는 반복 수렴 스캔이 아니라
         **한 프레임 직접 매핑**으로 잡고 곧바로 순찰에 들어간다(수직 FOV 67°
         광각이라 전신이 한 프레임에 담긴다 — 근거는 그쪽 클래스 docstring).
         풍속은 지금 겨누는 부위의 저장 세기를 쓰고, 겨누는 부위가 없는
         동안(매핑·탐색·재조준)은 공용 세기 — 아래 폴백과 같은 규칙.
       추적 폴백(fallback): 순찰 안정 상태에서 최근 --body-exit-window 동안
         팬 이동량 > --body-exit-deg면 전환 (팬은 사용자를 따라갈 때만 움직여
         헤드 자체 스윙에 면역 — control/body_wind.py MotionGate 참고).
         가슴 중심 조준(타겟 모드와 같은 피드백)에 **공용 세기** 적용
         (유효 모드가 추적이므로 풍속도 추적 모드와 동일 — 리모컨 주체 일관).
         팬 각이 --body-still-s 동안 잠잠하면 순찰 재개.
       이동 감지 우선순위: 시나리오 자체 이동 감지(recenter, --body-move-thr)는
         게이트(--body-exit-deg)보다 둔하게 둔다 — recenter가 먼저 걸리면 게이트
         창이 비워져(비 patrol) 폴백이 안 나오고, 작은 흔들림에도 순찰이 자주
         끊긴다 (실기 2026-08-27). 미세 흔들림은 --body-deadzone-pan/-tilt로 억제한다.
       조준각과 거리: 부위 조준각은 관측에서 바로 나오므로(순수 각도)
         거리 보정이 이미 되어 있다. 우리가 더하는 보정(머리를 위로,
         부위 간 최소 간격)만 고정 각도로 두면 거리에 따라 어긋나므로
         **측정된 머리↔상체 간격의 배수**로 준다 (--body-head-ratio 등).
       순찰은 도착 판정 없이 **시간 슬롯**으로 돈다 — 슬롯 = 체류 시간 +
         이동 시간(추정). 도착 판정을 쓰면 잡음 억제용 데드존이 남기는
         정지 오차 때문에 그 부위에 갇힌다 (control/body_wind.py 참고).
     러너 교체가 아니라 내부 상 전환이라 카메라 세션이 안 끊기고, 전환마다
     유효 모드 push [0x03, 모드]가 나간다. 폴백 중 가슴을 잃으면 마지막 조준을
     유지한다(재탐색 스윕은 순찰 상의 search 몫 — 알려진 한계).
     부위 러너는 --axis와 무관하게 팬+틸트를 모두 쓴다.
     0x02↔0x03 은 러너가 달라 카메라 세션이 재시작된다.
  6. notify 스레드 규칙 — 러너 스레드는 report_effective/report_recognized로
     보고하고, 서비스가 loop.call_soon_threadsafe로 BLE asyncio 루프에 넘긴다
     (dbus_fast는 스레드 안전하지 않음). 객체 인식 Status [0x02, x]는 인식
     여부가 바뀔 때만 보낸다.
  7. _apply_state 순서 — supervisor(이전 러너 join 완료) 먼저, 릴레이 적용을
     나중에. 반대면 죽어가는 부위 러너의 마지막 프레임이 방금 적용한 릴레이
     값을 덮는 레이스가 있다. 부위 모드 중에는 서비스가 릴레이를 아예 건드리지
     않는다(러너 소유 — 러너 종료 시 finally에서 0으로 놓고 나온다).
  8. 페어링 에이전트 등록 (NoIoAgent) — 에이전트가 없으면 중앙(앱)이 본딩을
     시도할 때 BlueZ가 응답할 수단이 없어 연결이 길게 매달리다 실패한다.
     본딩은 그 기기와 **처음** 연결할 때 일어나 "RPi나 앱을 처음 실행할 때만
     간혹 실패"로 나타난다 (실기 2026-08-27). 그래서 _ble_main이 서비스 등록
     전에 에이전트부터 붙인다.
  9. SIGHUP/SIGTERM 정리 (_exit_on_signals) — SSH가 끊기면 팬이 켜진 채
     남던 문제. 기본 동작이 즉시 종료라 finally가 안 돌던 것을 신호를
     KeyboardInterrupt로 바꿔 Ctrl+C와 같은 정리 경로를 타게 했다.

실행 (RPi 5, 레포 루트에서):
    python3 main.py --axis pan
    python3 main.py --axis pantilt --no-window
    python main.py --axis pan --dry-run --opencv   # 개발 PC
"""

from __future__ import annotations

import argparse
import asyncio
import signal
import sys
import threading
from pathlib import Path

# 레포 루트를 path에 추가 (config / vision / control / hardware / app 해결용)
sys.path.insert(0, str(Path(__file__).parent))

from bluez_peripheral.adapter import Adapter
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.gatt import CharacteristicFlags as CharFlags
from bluez_peripheral.gatt import Service, characteristic
from bluez_peripheral.util import get_message_bus

from config import (CFG, SERVICE_UUID, POWER_UUID, MODE_UUID, WIND_UUID,
                    STATUS_UUID, LOCAL_NAME, MODE_NAMES, WIND_TARGETS)

_TRK = CFG["tracking"]   # 추적 튜닝 기본값 (CLI로 덮어쓸 수 있음)
from vision.pose_estimator import MoveNetMultiPoseDetector
from vision.region_filter import RegionFilter
from app.tracking import add_state_args, open_motor_from_args, _DryRelay
from app.camera import (_WebStreamState, _make_handler, _ThreadedHTTP,
                        _window_viewer)
from app.runners import (ModeSupervisor, _ReportingDetector, _make_body_runner,
                         _make_homer, _make_runner, _make_sweeper,
                         _watch_disconnects)



def _hex(value):
    """수신 바이트 로그용 — b"\x02\x03" → "0x02 0x03"."""
    return " ".join(f"0x{b:02X}" for b in value)


class EswFanService(Service):
    """리모컨 주체 상태기 + STATUS 에코백/스냅샷 (docstring 1~4).

    bluez_peripheral의 characteristic 데코레이터는 클래스 정의에 묶여 있다 —
    상속으로 setter만 부분 교체할 수 없어서, 기능이 바뀔 때마다 서비스 클래스를
    통째로 새로 정의해 왔다.
    """

    _CHAR_NO = {"power": 1, "mode": 2, "wind": 3}   # 에코백의 Characteristic 번호

    def __init__(self, supervisor: ModeSupervisor, fan):
        super().__init__(SERVICE_UUID, True)
        self._supervisor = supervisor
        self._fan = fan
        self._loop: asyncio.AbstractEventLoop | None = None  # attach_loop가 채움
        self._power_on = False   # 시작 시 전원 OFF 가정 (앱이 POWER ON을 먼저 보냄)
        self._mode = 0x00
        self._effective_mode = 0x00
        self._level = 0                                  # 마지막 수신 공용 세기
        # 부위별 세기 (부위 러너용). 앱이 부위 모드 진입 시 자기 값으로 덮어쓴다.
        self._body_levels = {0x01: 0, 0x02: 1, 0x03: 0}

    # ── 상태 적용/통지 ───────────────────────────────────────────────────────

    def _gated_level(self) -> int:
        """게이팅만 한다 — 전원 OFF·부위 모드(러너 소유, 그 전까진 정지)면 0.
        세기 자체는 바꾸지 않는다 (docstring 1)."""
        return self._level if (self._power_on and self._mode != 0x03) else 0

    def _apply_state(self) -> int:
        """현재 상태를 supervisor와 릴레이에 적용한다 — supervisor 먼저
        (이전 러너 join 완료 후 릴레이, docstring 7)."""
        self._supervisor.set_state(self._power_on, self._mode, self._level)
        if self._power_on and self._mode == 0x03:
            return 0  # 부위 모드 릴레이는 러너 소유 — 서비스는 안 건드린다
        # 부위 러너가 아니면 유효 모드 = 요청 모드. join 뒤에 재확정하므로
        # 죽어가는 러너의 마지막 report_effective와 레이스하지 않는다.
        self._set_effective(self._mode)
        level = self._gated_level()
        self._fan.set_speed(level)
        return level

    def _snapshot(self) -> bytes:
        return bytes([0x04, int(self._power_on), self._mode, self._effective_mode,
                      self._level, self._body_levels[0x01],
                      self._body_levels[0x02], self._body_levels[0x03]])

    def _notify(self, payload: bytes) -> None:
        """status notify 송신 — 구독자가 없으면 BlueZ가 버린다.
        본 동작(모터/릴레이)에 영향을 주지 않게 실패는 로그만 남긴다."""
        try:
            self.status.changed(payload)
        except Exception as e:
            print(f"[BLE] status notify 실패: {e}")

    def _echo(self, name: str, value) -> None:
        self._notify(bytes([0x01, self._CHAR_NO[name], *value]))

    # ── 러너 보고 통로 (docstring 6) ─────────────────────────────────────────

    def attach_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """BLE asyncio 루프 연결 — _ble_main이 시작 시 1회 호출."""
        self._loop = loop

    def _notify_threadsafe(self, payload: bytes) -> None:
        """어느 스레드에서든 안전한 notify — dbus_fast는 BLE 루프 전용이라
        call_soon_threadsafe로 넘긴다 (BLE 루프 자신이 불러도 안전)."""
        loop = self._loop
        if loop is None or loop.is_closed():
            return
        loop.call_soon_threadsafe(self._notify, payload)

    @property
    def body_levels(self) -> dict:
        """부위별 저장 세기 — 부위 러너가 매 프레임 읽는다 (int 읽기 원자적)."""
        return self._body_levels

    @property
    def common_level(self) -> int:
        """공용 세기 — 부위 러너의 추적 폴백이 읽는다 (추적 모드와 동일 풍속)."""
        return self._level

    def _set_effective(self, mode: int) -> None:
        if mode == self._effective_mode:
            return
        self._effective_mode = mode
        self._notify_threadsafe(bytes([0x03, mode]))

    def report_effective(self, mode: int) -> None:
        """부위 러너의 상 전환 보고 → 유효 모드 push [0x03, 모드]."""
        self._set_effective(mode)

    def report_recognized(self, recognized: bool) -> None:
        """부위 러너의 객체 인식 변화 보고 → [0x02, 0/1] (변화 시에만 호출됨)."""
        self._notify_threadsafe(bytes([0x02, int(recognized)]))

    # ── Characteristics ──────────────────────────────────────────────────────
    # write-only 특성도 getter 자리(placeholder)가 필요 — 읽기 시도 시에만 쓰인다.

    @characteristic(POWER_UUID, CharFlags.WRITE)
    def power(self, options):
        raise NotImplementedError()

    @power.setter
    def power(self, value, options):
        if len(value) != 1 or value[0] not in (0x00, 0x01):
            print(f"[RX] 전원: 잘못된 값 ({_hex(value)}) — 거부, 에코 없음")
            return
        self._power_on = bool(value[0])
        # 에코는 적용 전에 보낸다(수신·수락 확인) — _apply_state의 러너 전환은
        # 이전 스레드 join으로 수 초 걸릴 수 있어, 뒤에 보내면 앱의 에코
        # 타임아웃이 모드/전원 전환마다 오탐한다. mode/wind setter도 동일.
        self._echo("power", value)
        level = self._apply_state()
        print(f"[RX] 전원: {'ON' if self._power_on else 'OFF'} → 풍속 {level}단")

    @characteristic(MODE_UUID, CharFlags.WRITE)
    def mode(self, options):
        raise NotImplementedError()

    @mode.setter
    def mode(self, value, options):
        if len(value) != 1 or value[0] not in MODE_NAMES:
            print(f"[RX] 모드: 잘못된 값 ({_hex(value)}) — 거부, 에코 없음")
            return
        self._mode = value[0]         # 유효 모드는 _apply_state/부위 러너가 갱신
        self._echo("mode", value)     # 적용 전 송신 (power setter 주석 참고)
        level = self._apply_state()   # 세기 복원 없음 — 게이팅 재평가만 (docstring 1)
        print(f"[RX] 모드: {MODE_NAMES[value[0]]} → 풍속 {level}단 유지")

    @characteristic(WIND_UUID, CharFlags.WRITE)
    def wind(self, options):
        raise NotImplementedError()

    @wind.setter
    def wind(self, value, options):
        if len(value) != 2 or value[0] not in WIND_TARGETS or not 0 <= value[1] <= 3:
            print(f"[RX] 풍량: 잘못된 값 ({_hex(value)}) — 거부, 에코 없음")
            return
        target, level = value[0], value[1]
        if target == 0x00:
            self._level = level
        else:
            self._body_levels[target] = level
        self._echo("wind", value)     # 적용 전 송신 (power setter 주석 참고)
        applied = self._apply_state()
        level_txt = "정지" if level == 0 else f"{level}단"
        print(f"[RX] 풍량: {WIND_TARGETS[target]} {level_txt} → 현재 적용 {applied}단")

    @characteristic(STATUS_UUID, CharFlags.READ | CharFlags.NOTIFY)
    def status(self, options):
        return self._snapshot()   # read = 스냅샷 (앱의 불일치 검증용 — docstring 2)

    def handle_disconnect(self) -> None:
        """앱(중앙) 연결 끊김 = 전원 OFF 처리 (docstring 9 —
        끊기면 양쪽 다 OFF로 수렴하므로 끊긴 동안의 불일치가 성립하지 않는다)."""
        if not self._power_on:
            return
        print("[BLE] 연결 끊김 → 전원 OFF 처리 (풍속 정지 + 0°,0° 파킹)")
        self._power_on = False
        self._apply_state()


def _exit_on_signals() -> None:
    """SIGHUP/SIGTERM을 KeyboardInterrupt로 바꿔 종료 정리가 돌게 한다 (docstring 9).

    두 신호의 기본 동작은 '프로세스 즉시 종료'라 finally도, 컨텍스트 매니저의
    __exit__(릴레이 전부 오픈·모터 해제)도 실행되지 않는다. SSH 세션이 끊기면
    포그라운드 프로세스가 SIGHUP을 받으므로, 팬이 켜진 채 헤드가 그 자리에
    그대로 남는다 (실기 2026-08-27). Ctrl+C(SIGINT)만 멀쩡히 정리되던 이유다.
    두 번째 신호는 기본 동작으로 되돌려 즉시 종료되게 한다 — 정리 도중 다시
    신호가 와도 매달리지 않도록.
    """
    def _raise(signum, _frame):
        signal.signal(signum, signal.SIG_DFL)
        raise KeyboardInterrupt(f"signal {signum}")

    for name in ("SIGHUP", "SIGTERM"):
        sig = getattr(signal, name, None)
        if sig is None:
            continue   # Windows에는 SIGHUP이 없다 (--dry-run 개발 PC)
        try:
            signal.signal(sig, _raise)
        except (ValueError, OSError) as e:
            print(f"[E2E] {name} 핸들러 설치 실패 — 그 신호로 종료 시 정리 안 됨: {e}")


async def _ble_main(service: EswFanService) -> None:
    """BLE 부팅 — 페어링 에이전트(docstring 8) → 서비스 등록 → 끊김 감시
    → Advertising. 시작 시 서비스에 asyncio 루프를 물려준다(docstring 6)."""
    service.attach_loop(asyncio.get_running_loop())
    bus = await get_message_bus()

    # 페어링 에이전트 (docstring 8) — 등록해두지 않으면 중앙(앱)이 본딩을
    # 시도할 때 BlueZ가 응답할 수단이 없어 연결이 길게 매달리다 실패한다.
    # 본딩 시도는 그 기기와 처음 연결할 때 일어나므로 "처음 실행할 때만 간혹
    # 실패"로 나타난다 (실기 2026-08-27). NoIo = 입출력 없는 기기 →
    # "just works" 페어링. 지역 변수로 붙잡아 둔다 — GC되면 D-Bus 객체가
    # 사라져 등록이 풀린다.
    agent = None
    try:
        from bluez_peripheral.agent import NoIoAgent
        agent = NoIoAgent()
        await agent.register(bus)
        print("[BLE] 페어링 에이전트 등록 (NoIo)")
    except Exception as e:
        print(f"[BLE] 페어링 에이전트 등록 실패 — 첫 연결이 불안정할 수 있음: {e}")

    await service.register(bus)
    await _watch_disconnects(bus, service)

    try:
        adapter = await Adapter.get_first(bus)
    except ValueError:
        sys.exit("BLE 어댑터 없음 — 'bluetoothctl power on' 확인")

    advert = Advertisement(LOCAL_NAME, [SERVICE_UUID], appearance=0x0000, timeout=0)
    await advert.register(bus, adapter=adapter)

    print(f"[BLE] Advertising 시작 — {LOCAL_NAME} (Ctrl+C 종료)")
    await bus.wait_for_disconnect()


def main() -> None:
    p = argparse.ArgumentParser(description="AI 스마트 타겟 선풍기 — BLE 추적/부위별 풍속")
    p.add_argument("--axis", choices=("pan", "tilt", "pantilt"), required=True,
                   help="타겟 모드일 때 돌릴 추적 축")
    p.add_argument("--model", default="multipose_lightning.tflite")
    p.add_argument("--conf", type=float, default=_TRK["conf"], help="키포인트 신뢰도 임계값")
    p.add_argument("--threads", type=int, default=_TRK["threads"], help="TFLite 스레드 수")
    # ── 축별 튜닝 (axis에 따라 일부만 실제로 쓰인다) ─────────────────────────
    p.add_argument("--gain-pan", type=float, default=_TRK["gain"]["pan"])
    p.add_argument("--gain-tilt", type=float, default=_TRK["gain"]["tilt"])
    p.add_argument("--deadzone-pan", type=float, default=_TRK["deadzone"]["pan"])
    p.add_argument("--deadzone-tilt", type=float, default=_TRK["deadzone"]["tilt"])
    p.add_argument("--target-cx", type=float, default=_TRK["target"]["cx"])
    p.add_argument("--target-cy", type=float, default=_TRK["target"]["cy"])
    lim = CFG["limits"]
    p.add_argument("--pan-min", type=float, default=lim["pan"]["min"])
    p.add_argument("--pan-max", type=float, default=lim["pan"]["max"])
    p.add_argument("--tilt-min", type=float, default=lim["tilt"]["min"])
    p.add_argument("--tilt-max", type=float, default=lim["tilt"]["max"])
    p.add_argument("--invert-pan", action="store_true")
    p.add_argument("--invert-tilt", action="store_true")
    p.add_argument("--region", choices=("chest", "head", "upper", "lower"), default="chest",
                   help="--axis tilt 전용 조준 부위")
    # ── 기본-회전 모드 (0x01) 스윕 ───────────────────────────────────────────
    p.add_argument("--rotate-span", type=float, default=60.0,
                   help="회전 모드 pan 스윕 반각 — 0° 기준 ±°")
    p.add_argument("--rotate-lead", type=float, default=3.0,
                   help="회전 모드에서 목표를 실제 위치보다 앞세울 각도 (°). "
                        "모터가 큐를 비워 정지→재기동을 반복하지 않을 만큼이면 "
                        "된다. 스윕 속도는 모터 순항 속도로 고정된다.")
    # ── 부위 모드 (0x03) — docstring 5. 세부 시나리오 파라미터(수렴/탐색 등)는
    #    control/body_wind.py 의 기본값을 그대로 쓴다 ──
    p.add_argument("--body-dwell", type=float, default=2.0,
                   help="부위 순찰 체류 시간 (s)")
    p.add_argument("--body-exit-deg", type=float, default=12.0,
                   help="순찰 중 이동 판정 팬 이동량 (° — 추적 폴백 전환)")
    p.add_argument("--body-exit-window", type=float, default=3.0,
                   help="이동 판정 시간창 (s)")
    p.add_argument("--body-still-s", type=float, default=5.0,
                   help="폴백 중 순찰 재진입 정지 시간 (s)")
    p.add_argument("--body-still-deg", type=float, default=3.0,
                   help="정지 판정 팬 각 범위 (°)")
    # 부위 간 각도 간격을 부풀리는 보정. 0 이면 각 부위를 조준점(--target-cy)에
    # 정확히 놓는다 — 기본값이다.
    # 예전 기본값(머리 1.1 / 상체 0.45)은 서 있는 자세에서만 검증됐다. 그때는
    # 머리 조준각이 틸트 리밋(-25°)에 잘려 편향이 드러나지 않았는데, 앉으면
    # 머리가 -10° 근처로 올라와 잘리지 않아 머리보다 8° 위를 겨누고 얼굴이 화면
    # 밖으로 나갔다 (실기 2026-09-02). 렌즈↔송풍구 높이차 보정은 부위마다 다를
    # 이유가 없으므로 --target-cy 가 담당한다.
    p.add_argument("--body-head-ratio", type=float, default=0.0,
                   help="머리 조준을 위로 올릴 배수 (측정 머리↔상체 간격 대비)")
    p.add_argument("--body-upper-ratio", type=float, default=0.0,
                   help="상체 조준을 위로 올릴 배수 (머리와 같은 방향, 음수면 아래로)")
    p.add_argument("--body-spread-ratio", type=float, default=0.15,
                   help="부위 간 최소 조준 간격 배수. 머리가 위 리밋에 붙으면 "
                        "이 간격만큼 상체를 아래로 밀어내므로, 크게 잡으면 "
                        "근거리에서 상체 조준이 가슴보다 내려간다 (실측 92cm "
                        "마운트·1m에서 0.3이면 126cm, 0.15면 132cm)")
    p.add_argument("--body-tilt-rate", type=float, default=11.0,
                   help="틸트 순항 속도 (도/s, 실측) — 슬롯의 이동 시간 산정용")
    # 미세 움직임 둔감화 (실기 2026-08-27: 작은 흔들림에 순찰이 자주 끊김).
    # 시나리오 자체 이동 감지는 게이트(--body-exit-deg)보다 위에 둬야 게이트가
    # 1차 판정자가 된다 — 낮으면 recenter가 먼저 걸려 폴백이 안 나온다.
    p.add_argument("--body-move-thr", type=float, default=20.0,
                   help="시나리오 재조준 트리거 가슴 오차 (° — 게이트보다 크게)")
    p.add_argument("--body-rescan-thr", type=float, default=30.0,
                   help="재조준 후 전신 재스캔 판정 이동량 (°)")
    p.add_argument("--body-deadzone-pan", type=float, default=1.0,
                   help="팬 데드존 (° — 포즈 잡음이 모터로 새는 것 차단)")
    p.add_argument("--body-deadzone-tilt", type=float, default=1.0,
                   help="틸트 데드존 (°)")
    p.add_argument("--body-converge", type=float, default=2.5,
                   help="수렴 판정 오차 (° — 순찰 도착·재조준·탐색 공용)")
    p.add_argument("--body-map-timeout", type=float, default=1.5,
                   help="매핑에서 부위가 안 보일 때 추정으로 넘어가는 시간 (s)")
    # ── 카메라 백엔드 (app/camera.py와 동일) ────────────────────────────────
    p.add_argument("--opencv", action="store_true")
    p.add_argument("--no-rpicam", dest="rpicam", action="store_false",
                   help="Picamera2 를 먼저 시도 (기본: rpicam-vid 캡처)")
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--no-window", action="store_true")
    p.add_argument("--dry-run", action="store_true",
                   help="모터/릴레이/lgpio 없이 각도 계산·풍속 게이팅만 (개발 PC 검증용)")
    add_state_args(p)
    # ── 웹 스트림 ────────────────────────────────────────────────────────────
    p.add_argument("--web", action="store_true", help="MJPEG 웹 스트림 송출")
    p.add_argument("--web-host", default="0.0.0.0")
    p.add_argument("--web-port", type=int, default=8090)
    p.add_argument("--web-quality", type=int, default=75)
    p.add_argument("--web-fps", type=float, default=20.0)
    args = p.parse_args()

    # 창을 원했는지 기억해두고, 추적 세션 스레드에서는 imshow가 절대 안 불리게
    # no_window를 강제한다. 창은 _window_viewer 전용 스레드가 대신 그린다
    # (프레임 통로로 web_state를 재사용 — app/camera.py _window_viewer 참고).
    local_window = not args.no_window
    serve_http = args.web
    args.no_window = True
    if local_window:
        args.web = True

    if not Path(args.model).exists():
        print(f"[ERROR] 모델 없음: {args.model}")
        sys.exit(1)
    if args.tilt_min >= args.tilt_max:
        print("[ERROR] --tilt-min은 --tilt-max보다 작아야 합니다")
        sys.exit(1)
    if args.axis in ("pan", "pantilt") and args.pan_min >= args.pan_max:
        print("[ERROR] --pan-min은 --pan-max보다 작아야 합니다")
        sys.exit(1)
    if not 0 < args.rotate_span <= min(lim["pan"]["max"], -lim["pan"]["min"]):
        print("[ERROR] --rotate-span은 0보다 크고 pan 회전 한계 안이어야 합니다")
        sys.exit(1)
    if args.rotate_lead <= 0:
        print("[ERROR] --rotate-lead는 0보다 커야 합니다")
        sys.exit(1)
    if (args.body_dwell <= 0 or args.body_exit_deg <= 0 or args.body_exit_window <= 0
            or args.body_still_s <= 0 or args.body_still_deg <= 0
            or args.body_spread_ratio < 0 or args.body_tilt_rate <= 0
            or args.body_move_thr <= 0 or args.body_rescan_thr <= 0
            or args.body_deadzone_pan <= 0 or args.body_deadzone_tilt <= 0

            or args.body_converge <= 0 or args.body_map_timeout <= 0):
        print("[ERROR] --body-* 인자 범위가 잘못됐습니다 (help 참고)")
        sys.exit(1)
    if args.body_move_thr <= args.body_exit_deg:
        # 시나리오 recenter가 먼저 걸리면 게이트 창이 비워져 폴백이 안 나온다.
        print(f"[WARN] --body-move-thr({args.body_move_thr:g}°)가 "
              f"--body-exit-deg({args.body_exit_deg:g}°) 이하 — 재조준이 먼저 걸려 "
              "추적 폴백이 잘 안 나올 수 있습니다.")

    # 하드웨어를 열기 전에 신호 처리를 걸어둔다 — 이후 어느 시점에 끊겨도
    # finally/__exit__가 돌아 릴레이가 열린다 (docstring 9).
    _exit_on_signals()

    detector = MoveNetMultiPoseDetector(args.model, conf_thr=args.conf,
                                        min_person_score=_TRK["min_person_score"],
                                        num_threads=args.threads)
    tracker = RegionFilter()

    web_srv = web_state = viewer_thread = None
    viewer_stop = threading.Event()
    if args.web:
        web_state = _WebStreamState(args.web_quality, args.web_fps)
        if serve_http:
            web_srv = _ThreadedHTTP((args.web_host, args.web_port), _make_handler(web_state))
            threading.Thread(target=web_srv.serve_forever, daemon=True).start()
            print(f"[web] http://{args.web_host}:{args.web_port}/  (브라우저에서 열기)")
        if local_window:
            viewer_thread = threading.Thread(target=_window_viewer,
                                             args=(web_state, viewer_stop), daemon=True)
            viewer_thread.start()
            print("[E2E] 로컬 창 뷰어 시작 (전용 GUI 스레드)")

    motor_cm = open_motor_from_args(args)
    try:
        with motor_cm as mc:
            mc.enable()
            # 이전 실행이 돌아간 채 꺼졌으면 저장 위치만큼 되돌아와 중앙을 본다.
            mc.restore_origin()

            if args.dry_run:
                fan_cm = _DryRelay()
            else:
                # lgpio를 최상단에서 import하는 모듈이라 지연 import — --dry-run 개발 PC 대응
                from hardware.relay_controller import FanRelay
                fan_cm = FanRelay(CFG, handle=mc.h)  # gpiochip 핸들 공유 (mc보다 먼저 닫혀야 함)

            with fan_cm as fan:  # mc보다 먼저 닫힘 — 공유 핸들이 살아있을 때 전부 오픈
                body_gains = (args.gain_pan, args.gain_tilt)
                # 일반 타겟 모드(0x02)도 인식 상태를 보고하도록 디텍터를 감싼다.
                reporting_detector = _ReportingDetector(detector)
                track_fn = _make_runner(args.axis, reporting_detector, tracker,
                                        mc, args, web_state)
                supervisor = ModeSupervisor(track_fn, _make_sweeper(mc, args),
                                               _make_homer(mc, home_pan=False),
                                               _make_homer(mc, home_pan=True),
                                               stop_fn=mc.stop,
                                               web_state=web_state)
                service = EswFanService(supervisor, fan)
                # 부위 러너와 인식 보고기는 서비스의 report_*를 쓰므로 나중에 연결.
                supervisor.set_body_runner(_make_body_runner(
                    detector, tracker, mc, fan, service, body_gains, args, web_state))
                reporting_detector.attach(service)

                print(f"[E2E] axis={args.axis} — BLE 전원/모드/풍량 명령 대기 중.")
                try:
                    asyncio.run(_ble_main(service))
                except KeyboardInterrupt:
                    print("\n[E2E] Ctrl+C 종료")
                finally:
                    fan.set_speed(0)  # 0° 복귀(최대 30s) 동안 팬이 계속 돌지 않게 먼저 정지
                    supervisor.stop_and_join()
                    if args.axis in ("tilt", "pantilt"):
                        # 웜기어(틸트)는 수동 복귀가 어려우므로 종료 시 0°로 되돌린다.
                        print("[E2E] 0° 복귀 중...")
                        mc.move_to(0.0, 0.0)
                        if not mc.wait_until_idle(timeout=30):
                            print("[E2E] 복귀 미완료 — 물리 위치를 확인하세요.")
    finally:
        if web_srv:
            web_srv.shutdown(); web_srv.server_close()
        viewer_stop.set()
        if viewer_thread:
            viewer_thread.join(timeout=3)  # destroyAllWindows는 뷰어 스레드 몫
        print("\n[E2E] 종료")


if __name__ == "__main__":
    main()
