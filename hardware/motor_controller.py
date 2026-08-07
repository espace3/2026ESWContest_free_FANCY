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
    _LOOKAHEAD_S는 "큐가 비기 전까지 워커 스레드가 늦어도 되는 시간"이기도
    하다 — 자세한 건 그 상수 주석 참고.

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

  - 위치 기억(장부 영속화): 리밋 스위치·엔코더가 없어 절대 원점을 잴 수단이
    없으므로, 장부를 주기적으로 파일에 남겨 재시작 때 되살린다
    (hardware/position_store.py, restore_origin()). home()이 "지금 이 위치가
    0°"라는 선언에 불과한 것을 보완하는 임시 수단이며, 탈조와 전원 차단 중
    외력은 여전히 잡지 못한다 — 한계는 position_store.py docstring 참고.

  - 펄스 타이밍: lgpio 펄스 스레드는 소프트웨어 타이밍이라 CPU 경쟁에 선점당하면
    스텝 간격이 흔들린다(추적 중 달그락 소음의 원인). open_chip()이 그 스레드만
    전용 코어 + RT로 격리한다 — 원리·실측·확인법은 hardware/lgpio_patch.md.
    timing=True(스크립트 --timing)로 이동 구간마다 드리프트·언더런을 찍어 재검증할 수
    있다. 언더런이 0이 아니면 펄스가 실제로 끊긴 것이다.

TODO — 남은 미결 (hardware/TODO.md 참고):
  - 각도 정확도: 오차원(기어비·백래시·탈조·원점)을 가르는 실험 순서와 보정 위치는
    hardware/angle_calibration.md. 백래시 보정을 넣을 때는 그 스텝을 장부에
    반영하면 안 된다 (헛도는 구간이라 출력축이 안 움직인다).
  - DIR 값 ↔ 각도 부호 매핑 실기 확인 (config stepper.*.dir_for_positive)
  - 호밍: 리밋 스위치/엔코더 도입 여부 미결. 그 전까지는 위 장부 영속화가
    유일한 원점 복원 수단이다 (사람이 물리 마커로 대조하는 절차 병행 권장).
  - 회전 금지 구역: 조립 후 배선/프레임 간섭으로 정해지는 HW 제약 (모터 자체
    한계 아님). 각도 clamp는 control 쪽 몫이라 여기서는 재계산하지 않고,
    구역이 확정되면 config에 min/max로 추가한다.
"""

from __future__ import annotations

import os
import threading
import time

import lgpio

from hardware.position_store import PositionStore

# 펄스 스레드 RT 우선순위. 실기 검증에 쓴 `chrt -f 10`과 같은 값 (lgpio_patch.md).
_RT_PRIO = 10

_CHUNK_S = 0.04       # 순항 청크 하나의 재생 시간 (초)

# 큐에 미리 쌓아두는 최대 분량 (초) = 선점 반응 지연의 상한. 새 목표가 와도 이미
# 큐에 든 펄스는 취소하지 않으므로 이만큼은 옛 목표대로 더 간다.
#
# [검증됨 — 추적 중 소음의 원인이 아니다 (2026-08-07)]
# 부하가 걸리면 큐가 비어 펄스가 끊기는 것이라 보고 0.25 → 1.0까지 올려봤지만
# 소리는 그대로였다. 1초 버퍼가 안 통한다는 건 워커 스레드가 늦어서 큐가 비는
# 문제가 아니라는 뜻이므로, 이 값을 만지는 방향은 접었다. 원인은 여전히 CPU 부하와
# 상관있다(모터 단독은 조용, 카메라+추론 동시 구동 시 소리) — 개별 펄스의 타이밍
# 지터 쪽을 볼 것.
_LOOKAHEAD_S = 0.09

_DIR_SETUP_S = 0.001  # DIR 신호 셋업 타임


def _open_chip_raw() -> int:
    """gpiochip 핸들을 연다 (Pi 5는 펌웨어에 따라 헤더 GPIO가 0 또는 4번)."""
    for chip in (0, 4):
        try:
            return lgpio.gpiochip_open(chip)
        except lgpio.error:
            continue
    raise RuntimeError("gpiochip을 열 수 없습니다. lgpio 설치를 확인하세요.")


def open_chip(*, pin_pulse_core: bool = True, rt: bool = True) -> int:
    """gpiochip 핸들을 열면서 lgpio 펄스 스레드를 전용 코어에 격리한다.

    추적 중 달그락 소음의 원인은 lgpio의 펄스 생성 스레드가 CPU 경쟁에 선점당해
    스텝 간격이 흔들리는 것이다 (실측: 부하 시 펄스 열이 3~7% 늘어짐,
    hardware/lgpio_patch.md). 그 스레드가 TFLite·카메라와 코어를 다투지 않게 한다.

    스레드는 **만든 쪽의 어피니티 마스크를 물려받는다.** 그래서 tid를 알아낼
    필요 없이 순서만으로 격리된다:

        ① 호출 스레드를 펄스 코어 하나에 묶고
        ② gpiochip을 연다 → 이때 lgpio가 만드는 펄스 스레드가 ①을 상속
        ③ 호출 스레드는 나머지 코어(계산용)로 되돌린다
        ④ 이미 떠 있던 스레드(TFLite 등)도 계산 코어로 몰아낸다
           — 스크립트가 모터보다 먼저 detector를 만들기 때문에 ④가 필요하다.
           순서를 안 고쳐도 되게 하려는 것.

    특권 불필요(RT와 달리 root도 limits.conf도 필요 없다). 다만 이건 **우리
    프로세스의 스레드만** 통제한다 — 커널 스레드·IRQ·bluetoothd 같은 남의
    프로세스는 여전히 펄스 코어에 올라온다. 완전 격리는 부팅 옵션 isolcpus가
    필요하고, 그래도 부족하면 펄스 스레드 RT 승격을 얹는다.

    같은 상속이 **스케줄링 정책에도** 적용된다 (glibc의 pthread_create 기본값이
    PTHREAD_INHERIT_SCHED). 그래서 ①에서 어피니티와 함께 SCHED_FIFO도 걸어두면
    펄스 스레드만 RT가 되고, ③에서 호출 스레드를 SCHED_OTHER로 되돌리면 그 뒤에
    만들어지는 워커·저장 스레드는 일반 우선순위로 남는다. 계산을 하지 않는 펄스
    스레드에만 RT를 주는 것이라 "추적 스크립트 전체를 chrt로 띄우지 말 것"
    (TFLite가 RT로 돌면 Pi가 멈춘다)이라는 금지와 충돌하지 않는다.

    RT는 rtprio 권한이 필요하다. 없으면 경고만 하고 어피니티만 적용한다.

    pin_pulse_core / rt 를 각각 끌 수 있다 (효과 비교용 — 스크립트에서는
    --no-pin / --no-rt, 판정은 --timing 의 드리프트로 한다).
    """
    avail = sorted(os.sched_getaffinity(0))
    if not pin_pulse_core or len(avail) < 2:
        return _open_chip_raw()

    pulse = {avail[-1]}                 # 마지막 코어를 펄스 전용으로
    compute = set(avail) - pulse
    before = set(os.listdir("/proc/self/task"))

    os.sched_setaffinity(0, pulse)
    rt_on = False
    if rt:
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(_RT_PRIO))
            rt_on = True
        except (OSError, PermissionError) as e:
            print(f"[motor] RT 승격 실패({e}) — 어피니티만 적용합니다. 권한을 주려면 "
                  f"/etc/security/limits.conf 에 "
                  f"'{os.environ.get('USER', '<user>')}  -  rtprio  {_RT_PRIO + 10}' "
                  f"추가 후 재로그인.")
    try:
        h = _open_chip_raw()
    finally:
        # 호출 스레드는 반드시 원상복구 — 안 그러면 이후 만들어지는 워커·저장
        # 스레드까지 RT가 되고, 저장 스레드의 fsync가 RT로 도는 건 위험하다.
        if rt_on:
            os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
        os.sched_setaffinity(0, compute)

    moved = 0
    for tid in before:                  # 기존 스레드를 펄스 코어에서 몰아낸다
        try:
            os.sched_setaffinity(int(tid), compute)
            moved += 1
        except (OSError, ValueError):
            pass                        # 그새 끝난 스레드 — 넘어간다

    # 신규 스레드가 실제로 RT를 물려받았는지 확인해서 찍는다 — 상속이 안 먹으면
    # (lgpio가 스레드를 늦게 만들거나 sched attr을 직접 지정하면) 여기서 드러난다.
    new = sorted(set(os.listdir("/proc/self/task")) - before)
    got_rt = sum(1 for t in new if _policy_of(int(t)) == os.SCHED_FIFO)
    print(f"[motor] 펄스 스레드 → 코어 {sorted(pulse)} 전용, "
          f"계산 스레드 {moved}개 → 코어 {sorted(compute)} | "
          f"lgpio 신규 스레드 {len(new)}개, 그중 RT {got_rt}개"
          f"{'' if rt_on else ' (RT 미적용)'}")
    return h


def _policy_of(tid: int) -> int:
    try:
        return os.sched_getscheduler(tid)
    except OSError:
        return -1


class _Axis:
    """한 축의 위치 장부 + 워커 스레드.

    pos_steps/target_steps/idle은 cond로 보호한다. pos_steps는 "송출이 확정된"
    (= 큐에 넣어져 반드시 나가게 될) 위치다.
    """

    def __init__(self, handle: int, name: str, pins: dict, params: dict, spr: int,
                 timing: bool = False) -> None:
        self.h = handle
        self.name = name
        self.timing = timing
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

        # 장부가 바뀔 때마다 부를 콜백 (MotorController가 저장 스레드를 깨우는 용도).
        # 여기서 직접 파일을 쓰면 fsync가 이 스레드를 수십 ms 붙잡아 펄스 큐가
        # 굶고 탈조가 나므로, 신호만 보내고 쓰기는 저장 스레드가 한다.
        self.on_change = None

        self.cond = threading.Condition()
        self.pos_steps = 0
        self.target_steps = 0
        self.idle = True
        self._quit = False
        self._sched_end = 0.0  # 큐잉된 펄스가 전부 끝나는 예상 시각 (워커 스레드 전용)
        # 타이밍 실측 (_TIMING) — 전부 워커 스레드 전용, 락 불필요
        self._t0 = 0.0          # 이번 구간 첫 청크를 큐에 넣은 시각
        self._planned = 0.0     # 이번 구간 펄스 열의 이론 재생 시간 합
        self._chunks = 0
        self._underruns = 0     # 큐가 비어 펄스가 끊긴 횟수
        self._underrun_s = 0.0  # 그 총 시간
        self.thread = threading.Thread(target=self._run, daemon=True, name=f"motor-{name}")
        self.thread.start()

    # ── 외부(메인 스레드)에서 호출 ────────────────────────────────────────────

    def set_target_steps(self, steps: int) -> None:
        with self.cond:
            self.target_steps = steps
            self.cond.notify_all()

    def read_steps(self) -> int:
        with self.cond:
            return self.pos_steps

    def force_position_steps(self, steps: int) -> None:
        """장부를 주어진 값으로 갈아끼운다 — 모터는 움직이지 않는다.

        재시작 시 "이전 실행이 여기서 죽었다"는 저장값을 장부에 되살리는 용도다
        (restore_origin). 목표도 같은 값으로 맞춰서 워커가 곧바로 움직이지
        않게 한다. 이동 중 호출하면 실제 위치와 장부가 어긋나므로 금지한다.
        """
        with self.cond:
            if not (self.idle and self.target_steps == self.pos_steps):
                raise RuntimeError(f"{self.name} 이동 중에는 장부를 바꿀 수 없습니다")
            self.pos_steps = steps
            self.target_steps = steps
        if self.on_change:
            self.on_change()

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
        now = time.monotonic()
        if self._chunks == 0:
            self._t0 = now
        elif now > self._sched_end:
            # 앞 청크가 이미 다 나가버린 뒤에야 다음 청크를 넣었다 = 펄스가 끊겼다.
            # (아래 max()가 이걸 조용히 흡수하므로 여기서 세지 않으면 안 보인다)
            self._underruns += 1
            self._underrun_s += now - self._sched_end
        self._planned += steps / freq
        self._chunks += 1
        self._sched_end = max(self._sched_end, now) + steps / freq
        with self.cond:
            self.pos_steps += sign * steps
        if self.on_change:   # 청크 하나 나갈 때마다 저장 스레드를 깨운다 (쓰기는 그쪽 몫)
            self.on_change()

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
        self._planned = self._underrun_s = 0.0
        self._chunks = self._underruns = 0

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

        # 20ms 미만 구간은 건너뛴다 — 청크 하나(_CHUNK_S=40ms)도 안 되는 잔이동은
        # 고정 오버헤드가 드리프트 %를 지배해서 숫자가 오해를 부른다.
        # 줄 앞의 \n: 추적 스크립트가 \r로 상태줄을 덮어쓰며 찍으므로 새 줄에서 시작해야 한다.
        if self.timing and self._chunks and self._planned >= 0.02:
            actual = time.monotonic() - self._t0
            drift = actual - self._planned
            print(f"\n[timing:{self.name}] {abs(delta):6d}스텝 청크{self._chunks:3d}  "
                  f"계획 {self._planned * 1000:7.1f}ms  실측 {actual * 1000:7.1f}ms  "
                  f"드리프트 {drift * 1000:+7.1f}ms ({100 * drift / self._planned:+5.1f}%)  "
                  f"언더런 {self._underruns:2d}회 {self._underrun_s * 1000:5.1f}ms", flush=True)


class MotorController:
    """팬틸트 스테퍼 컨트롤러 (논블로킹). 사용 예:

        with MotorController(CFG) as mc:
            mc.enable()
            mc.restore_origin()      # 이전 실행이 돌아간 채 죽었으면 원점 복원
            mc.move_to(12.5, -3.0)   # 즉시 리턴, 백그라운드에서 이동
            mc.wait_until_idle()     # 블로킹이 필요한 곳(검증 스크립트) 전용

    home()이나 restore_origin()으로 장부를 확정한 뒤부터 장부를 주기적으로 상태
    파일에 남긴다 — 재시작 때 원점을 되찾기 위한 것이며, 근거와 한계는
    position_store.py 참고. state_path=None이면 config "motor_state".file을 쓴다.
    """

    def __init__(self, cfg: dict, handle: int | None = None, *,
                 state_path=None, timing: bool = False,
                 pin_pulse_core: bool = True, rt: bool = True) -> None:
        self._own_handle = handle is None
        self.h = (open_chip(pin_pulse_core=pin_pulse_core, rt=rt)
                  if handle is None else handle)
        pins = cfg["pins"]
        st = cfg["stepper"]
        spr = st["steps_per_rev"] * st["microstep"]  # 3200 펄스/모터축 1회전

        # EN은 CNC v3 쉴드 구조상 전 축 공유 — 값이 다르면 배선/설정 불일치
        if pins["pan"]["EN"] != pins["tilt"]["EN"]:
            raise ValueError("현 설계는 EN 핀 공유 전제입니다 — config.py pins 확인")
        self.en_pin = pins["pan"]["EN"]
        lgpio.gpio_claim_output(self.h, self.en_pin, 1)  # 초기 비활성 (LOW 활성)

        self.pan = _Axis(self.h, "pan", pins["pan"], st["pan"], spr, timing)
        self.tilt = _Axis(self.h, "tilt", pins["tilt"], st["tilt"], spr, timing)

        # ── 장부 영속화 ──────────────────────────────────────────────────────
        ms = cfg.get("motor_state", {})
        path = state_path if state_path is not None else ms.get("file")
        self.store = PositionStore(path) if path else None
        self._persist_quit = threading.Event()
        self._persist_wake = threading.Event()
        self._persist_warned = False
        self._persist_thread = None
        # 두 축의 장부가 바뀔 때마다 저장 스레드를 깨우게 연결한다.
        self.pan.on_change = self.tilt.on_change = self._persist_wake.set
        # 저장 스레드 자체는 여기서 띄우지 않는다 — _start_persist() 주석 참고.

    # ── 장부 영속화 ──────────────────────────────────────────────────────────

    def _start_persist(self) -> None:
        """저장 스레드를 띄운다 (이미 떠 있으면 무시).

        생성 시점이 아니라 장부가 확정된 뒤(home() 또는 restore_origin())에야
        띄우는 이유: 갓 만든 컨트롤러의 장부는 무조건 0이라, 복원 전에 저장
        스레드가 한 번이라도 돌면 파일의 이전 위치를 0으로 덮어써서 복원이
        통째로 날아간다. 카메라·모델 로딩으로 restore_origin() 호출이 저장
        주기(0.5s)보다 늦어지면 실제로 재현된다.
        """
        if self.store is None or self._persist_thread is not None:
            return
        self._persist_thread = threading.Thread(
            target=self._persist_loop, daemon=True, name="motor-persist")
        self._persist_thread.start()

    def _write_state(self) -> bool:
        """현재 장부를 상태 파일에 기록. 실패해도 구동은 계속한다 (경고 1회)."""
        if not self.store:
            return False
        pan_steps, tilt_steps = self.pan.read_steps(), self.tilt.read_steps()
        try:
            self.store.save(pan_steps, tilt_steps,
                            pan_steps * self.pan.deg_per_step,
                            tilt_steps * self.tilt.deg_per_step)
            return True
        except OSError as e:
            if not self._persist_warned:
                print(f"\n[motor] 위치 상태 저장 실패 — 재시작 시 원점 복원 불가: {e}")
                self._persist_warned = True
            return False

    def _persist_loop(self) -> None:
        """장부가 바뀌면 곧바로 기록한다 (주기 폴링이 아님). 정지 중에는 쓰기 없음.

        _Axis가 펄스 청크를 큐잉할 때마다 on_change로 깨워준다. 쓰기를 이 스레드로
        떼어놓은 이유는 fsync가 SD에서 수십 ms씩 걸려서, 펄스를 내보내는 워커
        스레드에서 직접 부르면 큐가 굶어 탈조가 나기 때문이다.

        쓰는 동안 들어온 변경은 이벤트에 모였다가 다음 바퀴에서 한 번에 나간다
        (자연스럽게 합쳐지므로 쓰기 횟수가 청크 수만큼 늘지는 않는다). 그래서
        파일은 '직전 쓰기 한 번의 지연' 안에서 항상 최신이다.
        """
        last = (self.pan.read_steps(), self.tilt.read_steps())
        while not self._persist_quit.is_set():
            self._persist_wake.wait(timeout=1.0)   # 타임아웃은 종료 확인용
            if self._persist_quit.is_set():
                break
            # 읽기 전에 클리어 — 읽는 도중 장부가 또 바뀌면 다시 깨어난다
            self._persist_wake.clear()
            snap = (self.pan.read_steps(), self.tilt.read_steps())
            if snap != last and self._write_state():
                last = snap

    def restore_origin(self, timeout: float = 120.0) -> bool:
        """재시작 직후 호출 — 저장된 위치만큼 되돌아와 중앙(0°,0°)을 보게 한다.

        enable() 뒤, 추적을 시작하기 전에 부를 것. 저장값이 없으면(첫 실행)
        home()과 같이 현재 위치를 0°로 삼는다. 복귀 이동이 timeout 안에
        끝났으면 True.
        """
        if not self.store:
            self.home()
            return True

        saved = self.store.load()
        if saved is None:
            self.home()
            print("[motor] 저장된 위치 없음 — 현재 위치를 0°로 삼습니다. "
                  "헤드가 0° 마커에 맞춰져 있는지 확인하세요.")
            return True

        pan_steps, tilt_steps = saved
        self.pan.force_position_steps(pan_steps)
        self.tilt.force_position_steps(tilt_steps)
        self._start_persist()   # 장부가 확정된 뒤에 (이 순서가 중요 — _start_persist 참고)
        if pan_steps == 0 and tilt_steps == 0:
            print("[motor] 이전 위치가 원점 — 이동 없이 시작합니다.")
            return True

        pan_deg, tilt_deg = self.current_position()
        print(f"[motor] 이전 위치 pan={pan_deg:+.2f}° tilt={tilt_deg:+.2f}° — "
              f"중앙으로 복귀합니다.")
        self.move_to(0.0, 0.0)
        done = self.wait_until_idle(timeout=timeout)
        if not done:
            print(f"[motor] 복귀가 {timeout:g}s 안에 끝나지 않았습니다 — 물리 위치를 확인하세요.")
        self._write_state()
        return done

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

        원점을 '찾는' 게 아니라 '지금 이 자리를 0°라고 선언'하는 것이므로,
        헤드가 돌아간 채 죽은 다음 실행에서 그냥 부르면 그 자리가 새 0°가 된다.
        재시작 경로에서는 이걸 직접 부르지 말고 restore_origin()을 쓸 것 —
        상태 파일이 있으면 되살리고, 없을 때만(첫 실행) home()으로 떨어진다.
        리밋 스위치/엔코더 도입 전까지의 방편 (TODO.md)."""
        for ax in (self.pan, self.tilt):
            with ax.cond:
                if not (ax.idle and ax.target_steps == ax.pos_steps):
                    raise RuntimeError(f"{ax.name} 이동 중에는 home() 호출 불가")
                ax.pos_steps = 0
                ax.target_steps = 0
        self._start_persist()   # 장부가 확정됐으므로 이제 저장해도 안전하다

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
        """워커 종료(진행 중 이동은 감속 후 정지) → 최종 장부 기록 → disable → 핸들 반납.

        워커가 멈춘 뒤에 기록하므로 장부 = 실제 위치다. 호출부가 종료 전에
        move_to(0,0)로 원점 복귀를 마쳤다면 여기 기록되는 값은 0이고, 다음
        실행은 이동 없이 곧바로 시작한다.
        """
        self.pan.close()
        self.tilt.close()
        self._persist_quit.set()
        self._persist_wake.set()   # 대기 중인 저장 스레드를 곧바로 깨워 끝낸다
        if self._persist_thread:
            # 저장 스레드가 떠 있다 = 장부가 확정됐다(_start_persist 참고). 확정
            # 전이라면 장부의 기준이 없으므로 파일을 덮지 않는다 — 호출부가
            # restore_origin()을 빠뜨렸을 때 저장값을 날리지 않기 위한 것.
            self._persist_thread.join(timeout=2)
            self._write_state()
        self.disable()
        if self._own_handle:
            lgpio.gpiochip_close(self.h)

    def __enter__(self) -> "MotorController":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
