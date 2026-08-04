"""
hardware/motor_controller.py

팬틸트 스테퍼 모터 구동 — 하드웨어 호출 전용 모듈.
control/control_signal_generator.py가 계산한 각도(degree)를 받아 실제로
GPIO STEP/DIR 펄스를 내보내는 역할만 한다. 여기에 계산 로직을 넣지 말 것.
핀 배정·드라이버(TMC2209)·마이크로스텝·램프 파라미터는 config.py의
"pins"/"stepper"에 있다 (scripts/verify_motor.py로 실기 검증된 값).

논블로킹 구조 — 메인 루프가 20~30fps로 새 목표를 던져도 막히지 않는다:

  - 축(pan/tilt)마다 전용 워커 스레드. move_to()는 목표 각도만 갱신하고 즉시
    리턴한다. 이동 중 새 목표가 오면 "최신 목표만 유지" — 밀려난 중간 목표는
    버려진다.

  - 펄스는 lgpio tx_pwm으로 "청크"(재생 시간 _CHUNK_S 분량의 조각) 단위로
    큐잉한다. verify_motor.py처럼 순항 전체를 버스트 하나로 보내면 끝날 때까지
    새 목표를 반영할 방법이 없어서, 짧은 조각으로 나눠 보내고 조각 경계마다
    목표 변경을 확인하는 것이다. 한번 큐에 넣은 청크는 절대 취소하지 않는다
    (도중 취소하면 실제 송출된 스텝 수를 알 수 없어 위치 장부가 깨짐). 대신
    큐를 _LOOKAHEAD_S 이상 쌓지 않아 선점 반응 지연에 상한을 둔다.

  - 목표 변경: 같은 방향이고 감속 여유가 남는 목표면 감속 없이 그대로 갈아탄다
    (남은 거리가 늘면 램프도 더 오른다). 추적처럼 목표가 초당 수십 번 갱신되는
    부하에서 매번 "전체 감속 → drain → 재가속"을 돌면 순항에 도달하지 못하고
    저속 램프 구간만 오르내리는 것이 추적 저속의 주원인이었다 (2026-07-30 수정).
    정지·역방향·종료(close)는 기존 시퀀스 그대로 (급정지·급반전 금지 — 모터 보호):
        현재 방향 감속 → 큐 완전 배출(drain) → [필요 시 DIR 변경] → 새 방향 가속
    DIR 변경을 drain 뒤로 미루는 이유: 큐에 남은 펄스가 바뀐 DIR로 송출되면
    장부와 실제 위치가 어긋난다.

  - 위치는 정수 스텝으로만 기록한다 (float 각도로 들고 있으면 반올림 오차
    누적). 각도 환산은 deg_per_step 참고 — 1스텝 = pan 0.001125°(1:100 유성) /
    tilt ≈0.00121°(1:92.6 웜).
    큐잉 시점에 확정 기록하므로 current_position()은 이동 중 실제 로터보다
    최대 _LOOKAHEAD_S+감속 시간만큼 앞설 수 있고, idle이 되면 일치한다.

  - 짧은 이동(가속+감속 거리 미달)은 램프를 앞에서부터 잘라 쓰고, 아주 짧으면
    램프 없이 f_start 저속으로만 보낸다. 기어비 반영 후 이 경로는 pan 0.135°
    (=2구간 120스텝) 미만의 잔이동만 탄다 — 몇 스텝까지 정확히/조용히
    움직이는지 실측 필요 (TODO.md).

TODO — 남은 미결 (hardware/TODO.md 참고):
  - DIR 값 ↔ 각도 부호 매핑 실기 확인 (config stepper.*.dir_for_positive)
  - 호밍: 지금 home()은 "호출 시점의 물리 위치 = 0°" 가정 (임시 방편)
  - 회전 금지 구역: 조립 후 배선/프레임 간섭으로 정해지는 HW 제약 (모터 자체
    한계 아님). 각도 clamp는 control 쪽 몫이라 여기서는 재계산하지 않고,
    구역이 확정되면 config에 min/max로 추가한다.
"""

from __future__ import annotations

import threading
import time

import lgpio

_CHUNK_S = 0.04       # 순항 청크 하나의 재생 시간 (초)
_LOOKAHEAD_S = 0.09   # 큐에 미리 쌓아두는 최대 분량 (초) = 선점 반응 지연의 상한
_DIR_SETUP_S = 0.001  # DIR 신호 셋업 타임


def open_chip() -> int:
    """gpiochip 핸들을 연다 (Pi 5는 펌웨어에 따라 헤더 GPIO가 0 또는 4번)."""
    for chip in (0, 4):
        try:
            return lgpio.gpiochip_open(chip)
        except lgpio.error:
            continue
    raise RuntimeError("gpiochip을 열 수 없습니다. lgpio 설치를 확인하세요.")


class _Axis:
    """한 축의 위치 장부 + 워커 스레드.

    pos_steps/target_steps/idle은 cond로 보호한다. pos_steps는 "송출이 확정된"
    (= 큐에 넣어져 반드시 나가게 될) 위치다.
    """

    def __init__(self, handle: int, name: str, pins: dict, params: dict, spr: int) -> None:
        self.h = handle
        self.name = name
        self.step_pin = pins["STEP"]
        self.dir_pin = pins["DIR"]
        # 1스텝당 각도: pan(1:100 유성) ≈ 0.001125°, tilt(1:92.6 실측) ≈ 0.00121°
        self.deg_per_step = 360.0 / (spr * params["gear_ratio"])
        self.dir_for_positive = params["dir_for_positive"]
        self.f_start = params["f_start"]
        # 램프 한 구간의 스텝 수: 두 축 60스텝 — pan ≈ 0.0675°, tilt ≈ 0.073°
        self.seg_steps = params["ramp_steps_per_seg"]
        n_seg = params["ramp_segments"]
        # 첫 원소를 f_start 그대로 둬서 정지→기동 시 "f_start + 한 구간분" 만큼
        # 튀는 순간 점프를 없앤다 (2026-07-13, tilt 기동 시 탈조음 원인).
        self.ramp = [params["f_start"]] + [
            int(params["f_start"] + (params["f_max"] - params["f_start"]) * (i + 1) / n_seg)
            for i in range(n_seg)
        ]

        lgpio.gpio_claim_output(self.h, self.dir_pin, 0)
        lgpio.gpio_claim_output(self.h, self.step_pin, 0)

        self.cond = threading.Condition()
        self.pos_steps = 0
        self.target_steps = 0
        self.idle = True
        self._quit = False
        self._sched_end = 0.0  # 큐잉된 펄스가 전부 끝나는 예상 시각 (워커 스레드 전용)
        self.thread = threading.Thread(target=self._run, daemon=True, name=f"motor-{name}")
        self.thread.start()

    # ── 외부(메인 스레드)에서 호출 ────────────────────────────────────────────

    def set_target_steps(self, steps: int) -> None:
        with self.cond:
            self.target_steps = steps
            self.cond.notify_all()

    def close(self) -> None:
        """워커 스레드 종료. 이동 중이면 감속까지 마치고 멈춘다."""
        with self.cond:
            self._quit = True
            self.cond.notify_all()
        self.thread.join(timeout=10)

    # ── 워커 스레드 ──────────────────────────────────────────────────────────

    def _run(self) -> None:
        while True:
            with self.cond:
                while not self._quit and self.target_steps == self.pos_steps:
                    self.idle = True
                    self.cond.notify_all()  # wait_until_idle() 깨우기
                    self.cond.wait()
                if self._quit:
                    return
                self.idle = False
                target = self.target_steps
                delta = target - self.pos_steps
            self._execute(target, delta)
            # _execute가 선점으로 일찍 리턴했으면 위 루프가 새 목표로 재계획한다.

    def _preempted(self, planned_target: int) -> bool:
        """이동 도중 목표가 바뀌었으면(또는 종료 요청) True. 청크 경계에서만 확인."""
        with self.cond:
            return self._quit or self.target_steps != planned_target

    def _emit(self, freq: int, steps: int, sign: int) -> None:
        """청크 하나를 큐잉하고 위치 장부에 확정 기록한다. 큐가 _LOOKAHEAD_S
        이상 쌓여 있으면 빠질 때까지 기다린다 — 기다리는 동안에도 이미 큐잉된
        펄스는 끊기지 않고 재생 중이므로 모터는 멈추지 않는다."""
        while True:
            ahead = self._sched_end - time.monotonic()
            if ahead <= _LOOKAHEAD_S:
                break
            time.sleep(min(ahead - _LOOKAHEAD_S, 0.01))
        while lgpio.tx_room(self.h, self.step_pin, lgpio.TX_PWM) < 1:
            time.sleep(0.002)
        lgpio.tx_pwm(self.h, self.step_pin, freq, 50, 0, steps)
        self._sched_end = max(self._sched_end, time.monotonic()) + steps / freq
        with self.cond:
            self.pos_steps += sign * steps

    def _drain(self) -> None:
        """큐잉된 펄스가 전부 송출될 때까지 대기. DIR 변경/재계획 전 필수."""
        rest = self._sched_end - time.monotonic()
        if rest > 0:
            time.sleep(rest)
        while lgpio.tx_busy(self.h, self.step_pin, lgpio.TX_PWM):
            time.sleep(0.005)

    def _retarget(self, sign: int, decel_reserve: int):
        """선점 시 새 목표 판정. 진행 방향 그대로면서 감속 여유(decel_reserve)가
        남는 목표면 (새 목표, 새 남은 스텝)을 반환 — 호출자는 감속 없이 그대로
        이어 간다. 역방향·감속 여유 부족·종료 요청이면 None (감속 경로로)."""
        with self.cond:
            if self._quit:
                return None
            rest = (self.target_steps - self.pos_steps) * sign
            if rest >= decel_reserve:
                return self.target_steps, rest
            return None

    def _execute(self, planned_target: int, delta: int) -> None:
        """delta 스텝만큼 이동 (가속 → 순항 → 감속). 도중 목표 변경이 같은 방향
        연장이면 감속 없이 갈아타고 계속 간다 — 남은 거리가 늘면 램프도 더 오른다
        (2026-07-30, 추적 중 매번 전체 감속→재가속하던 churn 제거). 정지·역방향·
        종료면 감속까지만 마치고 리턴한다 — 바깥 루프(_run)가 새 목표로 재계획
        하므로 방향 전환은 자연히 "감속 → drain → DIR 변경 → 역방향 가속" 순서가
        된다. remaining ≥ 감속분 불변식을 항상 유지하므로 감속이 (마지막으로
        수락한) 목표를 넘는 일은 없고, 리턴 시점에는 항상 큐가 비어 있고
        장부 = 실제 위치다."""
        sign = 1 if delta > 0 else -1
        dir_level = self.dir_for_positive if sign > 0 else 1 - self.dir_for_positive
        lgpio.gpio_write(self.h, self.dir_pin, dir_level)
        time.sleep(_DIR_SETUP_S)

        remaining = abs(delta)
        climbed: list[int] = []  # 올라간 램프 단계 — 감속 시 역순 재생

        while True:
            decel_reserve = len(climbed) * self.seg_steps
            if (len(climbed) < len(self.ramp)
                    and remaining >= decel_reserve + 2 * self.seg_steps):
                # 가속 — 오른 뒤에도 감속분이 남을 때만 한 구간 오른다
                f = self.ramp[len(climbed)]
                self._emit(f, self.seg_steps, sign)
                climbed.append(f)
                remaining -= self.seg_steps
            elif remaining > decel_reserve:
                # 순항 — 감속분은 남겨둔다. 짧은 이동(램프 미진입)은 f_start
                # 저속으로만 나간다 (어떤 이동량이든 처리).
                cruise_f = climbed[-1] if climbed else self.f_start
                n = min(max(1, int(cruise_f * _CHUNK_S)), remaining - decel_reserve)
                self._emit(cruise_f, n, sign)
                remaining -= n
            else:
                break  # 남은 스텝 = 감속분 → 감속으로 정확히 목표 도달

            if self._preempted(planned_target):
                new_plan = self._retarget(sign, len(climbed) * self.seg_steps)
                if new_plan is None:
                    break  # 정지/역방향/종료 — 감속 후 _run이 재계획
                planned_target, remaining = new_plan

        for f in reversed(climbed):  # 감속 — 어느 경로로 나왔든 반드시 수행 (급정지 방지)
            self._emit(f, self.seg_steps, sign)

        self._drain()


class MotorController:
    """팬틸트 스테퍼 컨트롤러 (논블로킹). 사용 예:

        with MotorController(CFG) as mc:
            mc.enable()
            mc.home()
            mc.move_to(12.5, -3.0)   # 즉시 리턴, 백그라운드에서 이동
            mc.wait_until_idle()     # 블로킹이 필요한 곳(검증 스크립트) 전용
    """

    def __init__(self, cfg: dict, handle: int | None = None) -> None:
        self._own_handle = handle is None
        self.h = open_chip() if handle is None else handle
        pins = cfg["pins"]
        st = cfg["stepper"]
        spr = st["steps_per_rev"] * st["microstep"]  # 3200 펄스/모터축 1회전

        # EN은 CNC v3 쉴드 구조상 전 축 공유 — 값이 다르면 배선/설정 불일치
        if pins["pan"]["EN"] != pins["tilt"]["EN"]:
            raise ValueError("현 설계는 EN 핀 공유 전제입니다 — config.py pins 확인")
        self.en_pin = pins["pan"]["EN"]
        lgpio.gpio_claim_output(self.h, self.en_pin, 1)  # 초기 비활성 (LOW 활성)

        self.pan = _Axis(self.h, "pan", pins["pan"], st["pan"], spr)
        self.tilt = _Axis(self.h, "tilt", pins["tilt"], st["tilt"], spr)

    # ── 전원 ─────────────────────────────────────────────────────────────────

    def enable(self) -> None:
        """두 축 공통 enable. 유지 전류가 흐르기 시작한다 (정지 중에도 발열)."""
        lgpio.gpio_write(self.h, self.en_pin, 0)

    def disable(self) -> None:
        """두 축 공통 disable. tilt는 웜기어 자체 잠금으로 위치가 유지되지만
        pan은 직결이라 외력에 밀릴 수 있다 — 밀렸다면 재호밍 필요."""
        lgpio.gpio_write(self.h, self.en_pin, 1)

    # ── 이동 ─────────────────────────────────────────────────────────────────

    def home(self) -> None:
        """임시 호밍: 현재 물리 위치를 원점(0°)으로 삼는다. 이동 중 호출 금지.
        리밋 스위치/위치 저장 도입 여부 결정 전까지의 방편 (TODO.md)."""
        for ax in (self.pan, self.tilt):
            with ax.cond:
                if not (ax.idle and ax.target_steps == ax.pos_steps):
                    raise RuntimeError(f"{ax.name} 이동 중에는 home() 호출 불가")
                ax.pos_steps = 0
                ax.target_steps = 0

    def move_to(self, pan_angle_deg: float, tilt_angle_deg: float) -> None:
        """목표 각도로 이동 시작 — 논블로킹, 즉시 리턴. 이동 중 다시 부르면
        최신 목표로 갈아탄다. control_signal_generator에서 이미 회전 금지 구역
        밖으로 clamp된 각도가 들어온다고 가정한다 (여기서는 재계산하지 않음)."""
        # 각도 → 최근접 정수 스텝 (반올림 오차는 1스텝 = pan 0.1125° 미만)
        self.pan.set_target_steps(round(pan_angle_deg / self.pan.deg_per_step))
        self.tilt.set_target_steps(round(tilt_angle_deg / self.tilt.deg_per_step))

    def current_position(self) -> tuple[float, float]:
        """송출 확정 기준 (pan_angle_deg, tilt_angle_deg). 이동 중에는 실제
        로터보다 최대 _LOOKAHEAD_S+감속 시간만큼 앞설 수 있고, idle이면 일치."""
        with self.pan.cond:
            pan_deg = self.pan.pos_steps * self.pan.deg_per_step
        with self.tilt.cond:
            tilt_deg = self.tilt.pos_steps * self.tilt.deg_per_step
        return pan_deg, tilt_deg

    def wait_until_idle(self, timeout: float | None = None) -> bool:
        """두 축 모두 목표 도달 + 큐 배출 완료까지 대기. 검증 스크립트처럼
        블로킹 동작이 필요한 곳 전용 — 실전 메인 루프에서는 쓰지 말 것."""
        deadline = None if timeout is None else time.monotonic() + timeout
        for ax in (self.pan, self.tilt):
            with ax.cond:
                while not (ax.idle and ax.target_steps == ax.pos_steps):
                    rest = None if deadline is None else deadline - time.monotonic()
                    if rest is not None and rest <= 0:
                        return False
                    ax.cond.wait(rest)
        return True

    # ── 정리 ─────────────────────────────────────────────────────────────────

    def close(self) -> None:
        """워커 종료(진행 중 이동은 감속 후 정지) → disable → 핸들 반납."""
        self.pan.close()
        self.tilt.close()
        self.disable()
        if self._own_handle:
            lgpio.gpiochip_close(self.h)

    def __enter__(self) -> "MotorController":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
