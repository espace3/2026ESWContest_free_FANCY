"""
hardware/relay_control.py

선풍기 풍속 릴레이(TS0011 4ch) 구동 — 하드웨어 호출 전용 모듈.
BLE로 받은 풍속 단계(0=정지, 1=미풍, 2=약풍, 3=강풍)를 릴레이 IN1~IN3
GPIO 출력으로 내보내는 역할만 한다. "어느 부위의 풍속을 지금 적용할지"
같은 중재는 상위 통합부 몫 — 여기서는 set_speed(level) 하나만 노출한다.
핀·파라미터는 config.py의 "pins.relay"/"relay"에 있다.

배선 (2026-08-04 확정 — 물리버튼 없음, 릴레이 직접 탭 단속, RPi가 유일한
제어수단):
  - GPIO(레벨컨버터 LV측) → 레벨컨버터 → 릴레이 IN(HV측). 옵토 미사용.
  - **active-LOW 기판** (2026-08-04 실측): IN을 LOW로 당기면 코일이 여자되어
    NO-COM이 닫힘 = 해당 220V 탭 ON. HIGH가 OFF(오픈)다.
  - IN1/IN2/IN3 = 미풍/약풍/강풍. IN4는 완전 미사용. 전부 오픈 = 정지.
  - 동시에 최대 1채널만 닫는다 (두 탭 동시 통전 금지).

극성은 config의 relay.active_high 하나로만 표현되고 _on_level()/_off_level()에
갇혀 있다 — 코드 어디에도 0/1을 직접 박지 말 것.

break-before-make: 상태 전환은 항상 "① 전부 오픈 → ② guard_s 대기 →
③ 최신 목표 재확인 → ④ 하나만 닫기" 순서다 (모터 back-EMF/접점 아크/
역전류 방지). BLE async 콜백을 guard sleep으로 막지 않도록
motor_control의 "최신 목표만 유지 + 전용 워커 스레드" 구조를 축소
차용했다 — set_speed()는 목표만 갱신하고 즉시 리턴, 같은 목표 반복은
무시한다(릴레이 채터 방지).

부팅 안전 (2026-08-10 확인됨): 기판이 active-LOW라 릴레이 핀을 GPIO9~27
(부팅 기본 pull-DOWN) 대역에서 고른 원래 근거("LOW=오픈")는 무효다 —
LOW가 곧 ON이다. 따라서 GPIO가 Hi-Z인 구간(claim 전 = 부팅 중,
gpiochip_close() 후 = 종료·크래시)의 안전은 레벨컨버터 HV측 채널 풀업
(BSS138 양방향형은 10k)이 IN을 5V=OFF로 띄워주는 것에 의존한다.
실기에서 소프트웨어 없이 전원만 인가했을 때 팬이 돌지 않는 것으로 확인됐다.
컨버터를 빼거나 릴레이 핀을 바꾸면 이 의존이 깨지므로 재확인할 것.
"""

from __future__ import annotations

import threading
import time

import lgpio

from hardware.motor_control import open_chip


class FanRelay:
    """선풍기 풍속 릴레이 (논블로킹). 사용 예:

        with FanRelay(CFG) as fan:
            fan.set_speed(2)   # 즉시 리턴 — 워커가 전부 오픈→guard→IN2 ON
            ...
        # with 종료 시 전부 오픈(정지) 보장

    handle에 이미 연 gpiochip 핸들을 주면 공유한다 (MotorController와 동일
    규약 — 상위 통합부가 한 번만 열어 모터/릴레이에 나눠 준다). None이면
    자체 오픈하고 close() 때 반납한다.
    """

    def __init__(self, cfg: dict, handle: int | None = None) -> None:
        self._own_handle = handle is None
        self.h = open_chip() if handle is None else handle
        rc = cfg["relay"]
        pin_map = cfg["pins"]["relay"]
        self._active_high = rc["active_high"]
        self._guard_s = rc["guard_s"]
        # level(1~3) → BCM 핀. 0(정지)은 매핑에 없음 = 전부 오픈.
        self._level_pins = {lv: pin_map[name] for lv, name in rc["level_pins"].items()}

        # OFF 레벨로 claim — 부팅 안전 상태(전부 오픈)와 초기 상태를 일치시킨다.
        off = self._off_level()
        for pin in self._level_pins.values():
            lgpio.gpio_claim_output(self.h, pin, off)

        self.cond = threading.Condition()
        self._target = 0    # 최신 목표 level (0=정지)
        self._applied = 0   # 워커가 반영을 마친 level
        self._quit = False
        self._thread = threading.Thread(target=self._run, daemon=True, name="fan-relay")
        self._thread.start()

    # ── 외부(메인/BLE 콜백)에서 호출 ─────────────────────────────────────────

    def set_speed(self, level: int) -> None:
        """목표 풍속만 갱신하고 즉시 리턴 (0=정지, 1=미풍, 2=약풍, 3=강풍).
        범위 밖 값은 정지(0) 취급. 현재 목표와 같으면 무시."""
        if level not in self._level_pins:
            level = 0
        with self.cond:
            if level == self._target:
                return
            self._target = level
            self.cond.notify_all()

    def current_speed(self) -> int:
        """워커가 반영을 마친 level. 전환 중(guard 대기)에는 직전 값일 수 있고,
        wait_until_applied()가 True를 반환한 뒤에는 목표와 일치한다."""
        with self.cond:
            return self._applied

    def wait_until_applied(self, timeout: float | None = None) -> bool:
        """최신 목표가 릴레이에 반영될 때까지 대기. 검증 스크립트처럼 블로킹이
        필요한 곳 전용 — 실전 메인 루프에서는 쓰지 말 것."""
        deadline = None if timeout is None else time.monotonic() + timeout
        with self.cond:
            while self._applied != self._target:
                rest = None if deadline is None else deadline - time.monotonic()
                if rest is not None and rest <= 0:
                    return False
                self.cond.wait(rest)
            return True

    def close(self) -> None:
        """워커 종료 → 전부 오픈(정지) → (자체 오픈 핸들이면) 핸들 반납."""
        with self.cond:
            self._quit = True
            self.cond.notify_all()
        self._thread.join(timeout=5)
        self._all_off()
        if self._own_handle:
            lgpio.gpiochip_close(self.h)

    def __enter__(self) -> "FanRelay":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ── 워커 스레드 ──────────────────────────────────────────────────────────

    def _run(self) -> None:
        while True:
            with self.cond:
                while not self._quit and self._target == self._applied:
                    self.cond.wait()
                if self._quit:
                    return
                target = self._target
            # break-before-make ①: 어떤 make보다 "전부 오픈"이 먼저 온다.
            self._all_off()
            # ②③: guard 대기 중 목표가 또 바뀌면 전부 오픈 유지한 채 다시 guard.
            while True:
                time.sleep(self._guard_s)
                with self.cond:
                    if self._quit:
                        return
                    if self._target == target:
                        break
                    target = self._target
            # ④: 하나만 닫는다 (0이면 전부 오픈 유지) — 두 탭 동시 통전 구간 없음.
            if target != 0:
                lgpio.gpio_write(self.h, self._level_pins[target], self._on_level())
            with self.cond:
                self._applied = target
                self.cond.notify_all()  # wait_until_applied() 깨우기

    # ── 내부 ─────────────────────────────────────────────────────────────────

    def _on_level(self) -> int:
        return 1 if self._active_high else 0

    def _off_level(self) -> int:
        return 0 if self._active_high else 1

    def _all_off(self) -> None:
        off = self._off_level()
        for pin in self._level_pins.values():
            lgpio.gpio_write(self.h, pin, off)
