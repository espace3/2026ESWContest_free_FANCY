#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py - 진입점: BLE GATT 서비스 + 전체 조립

  EswFanService  앱의 write 를 받는 GATT 서비스 (전원/모드/풍량/STATUS)
  _ble_main      BLE 부팅 — 페어링 에이전트, 서비스 등록, Advertising
  main()            CLI 인자 → 모터·릴레이·카메라·러너·supervisor 조립

모드마다 도는 러너와 그 교대는 app/runners.py, 카메라·시각화는 app/camera.py,
추적 루프는 app/tracking.py, 부위 시나리오는 control/region_patrol.py 에 있다.

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
       인식 Status(0x02)는 부위 러너가 보낸다.
  3. 풍량 세기 0(정지)은 공용·부위 어느 대상에도 허용한다. 부위 세 개가 모두
     0이어도 유효한 설정이며(사용자가 바람만 끈 상태), 그때 부위 러너는 상체
     조준을 계속하되 릴레이를 돌리지 않는다 — control/region_patrol.py _route 참고.
  4. 유효 모드(_effective_mode) — 요청 모드(앱이 write한 값)와 별개로 RPi가
     실제로 돌리는 모드. 예전에는 부위 러너가 사용자 이동 시 추적 폴백(0x02)으로
     갈라졌으나, 폴백을 없앤 뒤(2026-11-03)로는 부위 러너 시작 시 0x03 을
     보고할 뿐 갈라지지 않는다. 앱 계약(push 형식)은 그대로 둔다.
  5. 부위 모드(0x03) — 단일 세션 러너(app/runners.py _make_body_runner)가
     RegionPatrolScenario(control/region_patrol.py)로 돈다.
       조준: 지금 겨누는 부위의 화면 좌표(cx, cy)를 조준점에 놓도록 **팬·틸트를
         함께** 움직인다 — 타겟 모드와 같은 피드백 식에 대상만 부위로 바뀐 것.
         사람이 움직여도 멈추지 않고 그 부위를 따라가며, 부위를 가로·세로
         그대로 쓰므로 누운 자세도 같은 식으로 다룬다.
       순환: 세기 1 이상인 부위를 head→upper→lower→upper 순으로 돈다. 슬롯 =
         이동(조준점 ±--body-arrive 안에 들어오거나 --body-travel-max 경과) +
         체류(--body-dwell). 도착만 기다리면 데드존 정지 오차에 갇히므로
         시간 상한을 둔다.
       보이지 않는 부위: 보이는 부위에서 머리→상체 벡터(측정, EMA)만큼 밀어
         추정해 그쪽으로 간다 — 카메라가 헤드 위에 있어 하체를 겨누면 머리가,
         머리를 겨누면 하체가 화면 밖으로 나가기 쉽다.
       풍속: 지금 겨누는 부위의 저장 세기. 대상 상실 중에는 공용 세기.
       대상 상실: 타겟 모드처럼 그 자리 대기. --body-lost-s 이상 이어지면
         마지막 머리/상체 조준 각도로 돌아가 기다린다(하체를 겨누다 놓쳐
         바닥만 보는 상태 방지).
     부위 러너는 --axis와 무관하게 팬+틸트를 모두 쓴다.
     0x02↔0x03 은 러너가 다르지만 카메라는 닫지 않고 넘겨주며 감속 정지도
     생략한다 (app/runners.py ModeSupervisor "카메라 넘겨주기").
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
import time
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
from vision.pose_estimate import MoveNetMultiPoseDetector
from vision.region_filter import RegionFilter
from app.tracking import (LatencyStat, add_state_args, open_motor_from_args,
                          _DryRelay)
from app.camera import (CameraLease, _WebStreamState, _make_handler, _ThreadedHTTP,
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
        # BLE 응답 실측 (README "성능"). 둘을 나눠 재는 이유: 에코는 적용 전에
        # 보내므로(아래 setter 주석) 앱이 체감하는 응답이고, 적용 완료는 러너
        # 전환의 join 을 포함해 수 초가 될 수 있다.
        self._lat_echo = LatencyStat("BLE 응답(수신→에코)")
        self._lat_apply = LatencyStat("BLE 적용(수신→반영 완료)")

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
        """공용 세기 — 부위 러너가 대상 상실 중에 쓴다 (추적 모드와 동일 풍속)."""
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
        t0 = time.time()
        self._power_on = bool(value[0])
        # 에코는 적용 전에 보낸다(수신·수락 확인) — _apply_state의 러너 전환은
        # 이전 스레드 join으로 수 초 걸릴 수 있어, 뒤에 보내면 앱의 에코
        # 타임아웃이 모드/전원 전환마다 오탐한다. mode/wind setter도 동일.
        self._echo("power", value)
        self._lat_echo.add((time.time() - t0) * 1000.0)
        level = self._apply_state()
        self._lat_apply.add((time.time() - t0) * 1000.0)
        print(f"[RX] 전원: {'ON' if self._power_on else 'OFF'} → 풍속 {level}단"
              f"  ({self._lat_echo.last():.0f}/{self._lat_apply.last():.0f}ms)")

    @characteristic(MODE_UUID, CharFlags.WRITE)
    def mode(self, options):
        raise NotImplementedError()

    @mode.setter
    def mode(self, value, options):
        if len(value) != 1 or value[0] not in MODE_NAMES:
            print(f"[RX] 모드: 잘못된 값 ({_hex(value)}) — 거부, 에코 없음")
            return
        t0 = time.time()
        self._mode = value[0]         # 유효 모드는 _apply_state/부위 러너가 갱신
        self._echo("mode", value)     # 적용 전 송신 (power setter 주석 참고)
        self._lat_echo.add((time.time() - t0) * 1000.0)
        level = self._apply_state()   # 세기 복원 없음 — 게이팅 재평가만 (docstring 1)
        self._lat_apply.add((time.time() - t0) * 1000.0)
        print(f"[RX] 모드: {MODE_NAMES[value[0]]} → 풍속 {level}단 유지"
              f"  ({self._lat_echo.last():.0f}/{self._lat_apply.last():.0f}ms)")

    @characteristic(WIND_UUID, CharFlags.WRITE)
    def wind(self, options):
        raise NotImplementedError()

    @wind.setter
    def wind(self, value, options):
        if len(value) != 2 or value[0] not in WIND_TARGETS or not 0 <= value[1] <= 3:
            print(f"[RX] 풍량: 잘못된 값 ({_hex(value)}) — 거부, 에코 없음")
            return
        t0 = time.time()
        target, level = value[0], value[1]
        if target == 0x00:
            self._level = level
        else:
            self._body_levels[target] = level
        self._echo("wind", value)     # 적용 전 송신 (power setter 주석 참고)
        self._lat_echo.add((time.time() - t0) * 1000.0)
        applied = self._apply_state()
        self._lat_apply.add((time.time() - t0) * 1000.0)
        level_txt = "정지" if level == 0 else f"{level}단"
        print(f"[RX] 풍량: {WIND_TARGETS[target]} {level_txt} → 현재 적용 {applied}단"
              f"  ({self._lat_echo.last():.0f}/{self._lat_apply.last():.0f}ms)")

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
    # ── 부위 모드 (0x03) — docstring 5, control/region_patrol.py ──
    p.add_argument("--body-dwell", type=float, default=2.0,
                   help="부위 체류 시간 (s) — 도착(또는 최대 이동 시간) 후부터 센다")
    p.add_argument("--body-arrive", type=float, default=5.0,
                   help="도착 판정 오차 (° — 두 축 모두 이 안이면 체류 시작). "
                        "데드존이 남기는 정지 오차(팬 3.3°)보다 커야 한다")
    p.add_argument("--body-travel-max", type=float, default=3.0,
                   help="부위 이동 최대 시간 (s) — 사람이 계속 움직여 도착을 못 해도 "
                        "이 시간이 지나면 체류로 넘어간다")
    p.add_argument("--body-lost-s", type=float, default=1.0,
                   help="대상 상실 판정 시간 (s) — 넘으면 마지막 머리/상체 조준 "
                        "각도로 돌아가 대기")
    p.add_argument("--body-deadzone-pan", type=float, default=1.0,
                   help="팬 데드존 (° — 포즈 잡음이 모터로 새는 것 차단)")
    # 틸트는 좁게 — 명령이 현재각 + gain x 오차라 데드존/gain 만큼 못 미친 채
    # 선다. 1.0 이면 5°, 0.25 면 1.25° (이전 순찰의 수렴용 데드존과 같은 값).
    p.add_argument("--body-deadzone-tilt", type=float, default=0.25,
                   help="틸트 데드존 (°)")
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
    if (args.body_dwell <= 0 or args.body_arrive <= 0 or args.body_travel_max <= 0
            or args.body_lost_s <= 0
            or args.body_deadzone_pan <= 0 or args.body_deadzone_tilt <= 0):
        print("[ERROR] --body-* 인자 범위가 잘못됐습니다 (help 참고)")
        sys.exit(1)
    if args.gain_pan > 0 and args.body_arrive <= args.body_deadzone_pan / args.gain_pan:
        # 데드존이 남기는 정지 오차가 도착 문턱보다 크면 도착이 안 돼
        # 매 슬롯 --body-travel-max 를 다 기다린다.
        print(f"[WARN] --body-arrive({args.body_arrive:g}°)가 팬 정지 오차 "
              f"({args.body_deadzone_pan / args.gain_pan:.1f}°) 이하 — 부위마다 "
              f"--body-travel-max 만큼 늦게 넘어갑니다.")

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
                from hardware.relay import FanRelay
                fan_cm = FanRelay(CFG, handle=mc.h)  # gpiochip 핸들 공유 (mc보다 먼저 닫혀야 함)

            with fan_cm as fan:  # mc보다 먼저 닫힘 — 공유 핸들이 살아있을 때 전부 오픈
                body_gains = (args.gain_pan, args.gain_tilt)
                # 일반 타겟 모드(0x02)도 인식 상태를 보고하도록 디텍터를 감싼다.
                reporting_detector = _ReportingDetector(detector)
                cam_lease = CameraLease(args)  # 카메라 러너끼리 카메라를 넘겨준다
                track_fn = _make_runner(args.axis, reporting_detector, tracker,
                                        mc, args, web_state, cam_lease)
                supervisor = ModeSupervisor(track_fn, _make_sweeper(mc, args),
                                               _make_homer(mc, home_pan=False),
                                               _make_homer(mc, home_pan=True),
                                               stop_fn=mc.stop,
                                               web_state=web_state,
                                               camera_lease=cam_lease)
                service = EswFanService(supervisor, fan)
                # 부위 러너와 인식 보고기는 서비스의 report_*를 쓰므로 나중에 연결.
                supervisor.set_body_runner(_make_body_runner(
                    detector, tracker, mc, fan, service, body_gains, args, web_state,
                    cam_lease))
                reporting_detector.attach(service)

                print(f"[E2E] axis={args.axis} — BLE 전원/모드/풍량 명령 대기 중.")
                try:
                    asyncio.run(_ble_main(service))
                except KeyboardInterrupt:
                    print("\n[E2E] Ctrl+C 종료")
                finally:
                    print(f"\n[측정] {service._lat_echo.summary()}")
                    print(f"[측정] {service._lat_apply.summary()}")
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
