"""
bench/measure_jitter.py - 펄스 스레드 웨이크업 지터 측정 (docs/lgpio_patch.md)

배선 필요 없음. GPIO도 안 건드린다.

lgpio의 펄스 스레드는 "다음 에지 시각까지 자고 → 깨서 → 핀을 토글"을 반복한다.
소리의 원인이 그 스레드가 **제때 못 깨는 것**이라는 게 현재 진단이므로, 재야 할
대상은 GPIO 파형이 아니라 **커널이 스레드를 얼마나 늦게 깨우는가**다. 그건 같은
커널 서비스(clock_nanosleep)를 같은 주기로 두들기는 스레드 하나면 잴 수 있다.

f_max 6000 · 듀티 50%면 에지는 83.3us마다다. 이 스크립트는 같은 간격으로 깨면서
매번 "예정 시각보다 몇 us 늦었는지"를 기록하고 분포를 찍는다.

  - 늦음이 반주기(T/2)를 넘으면 그 에지는 눈에 띄게 밀린 것 = (a) 간격 지터
  - 늦음이 한 주기(T)를 넘으면 그 슬롯을 통째로 놓친 것이고, lgpio는 밀린 펄스를
    몰아서 낸다 = (b) 펄스 몰림 → **탈조**. 이 카운트가 0이 아니면 f_max 상향 금지.

## 쓰는 법 — 부하는 진짜 추적으로 만든다

터미널 2개면 끝난다. 추적 스크립트는 하나도 안 고쳐도 된다 (부하는 시스템 전체
CPU 경합이라 별도 프로세스의 스레드도 똑같이 겪는다).

    # 터미널 1 — 평소대로 추적
    python main.py --axis pantilt   # 평소 쓰는 그대로

    # 터미널 2 — 그동안 재기
    python bench/measure_jitter.py --freq 6000 --sec 10

기준선(무부하)을 먼저 떠두고 비교할 것:

    python bench/measure_jitter.py --freq 6000 --sec 10            # 기준선
    python bench/measure_jitter.py --freq 6000 --sec 10 --load 6   # 합성 부하(추적 대신)
    sudo chrt -f 10 $(which python3) bench/measure_jitter.py --freq 6000 --sec 10 --load 6
    python bench/measure_jitter.py --freq 6000 --sec 10 --cpu 3    # 코어 3 전용(어피니티 대책 예습)

마지막 두 줄이 대책 대조군이다 — 여기서 꼬리가 확 줄면 RT 승격/어피니티가 통한다는
뜻이고, 그때 비로소 motor_controller에 손대면 된다. 이 스크립트 자체는 계산을 안
하므로 chrt로 띄워도 안전하다 (추적 스크립트 전체를 chrt하지 말라는 금지와 무관).

## 이 숫자의 한계 (해석 주의)

파이썬 루프라 인터프리터 오버헤드가 섞여 **절대값은 비관적**이다 (lgpio는 C 스레드라
바닥이 더 낮다). 의미 있는 건 절대값이 아니라 ① 무부하 대비 부하 시 증가분과
② 꼬리(p99/max) — 꼬리는 파이썬 오버헤드가 아니라 선점이 만든다.

실제 STEP 에지를 직접 재고 싶으면 STEP 핀을 남는 GPIO에 점퍼로 물려 alert 콜백으로
타임스탬프를 받는 방법이 있는데, 이 숫자로 판단이 안 설 때만 할 것.
"""

from __future__ import annotations

import argparse
import ctypes
import multiprocessing as mp
import os
import statistics as st
import time

_PR_SET_TIMERSLACK = 29


def set_timer_slack(ns: int) -> None:
    """이 스레드의 타이머 슬랙(ns)을 바꾼다. 특권 불필요.

    Linux는 비-RT 스레드의 nanosleep에 기본 50us 슬랙을 얹는다 — 에지 간격이
    83us(f_max 6000)인 걸 생각하면 부하가 0이어도 이미 타이밍이 망가지는 크기다.
    RT(SCHED_FIFO) 태스크는 커널이 슬랙을 0으로 강제하므로, `chrt -f`가 소리를
    잡았던 효과의 일부는 우선순위가 아니라 **슬랙 제거**였을 가능성이 있다.
    낮춘 값은 이후 생성되는 스레드가 상속하므로(fork/clone이 복사), lgpio 핸들을
    열기 전에 부르면 펄스 스레드에도 적용된다 — 특권도 /proc 탐색도 필요 없다.
    """
    ctypes.CDLL("libc.so.6").prctl(_PR_SET_TIMERSLACK, ctypes.c_ulong(ns), 0, 0, 0)


def _busy(stop) -> None:
    """코어 하나를 채우는 프로세스 — lgpio_patch.md의 busy loop과 같은 역할."""
    while not stop.is_set():
        pass


def probe(period_ns: int, seconds: float) -> list[int]:
    """period_ns만큼 자고 깨기를 반복하며 매번 '예정보다 늦은 시간(ns)'을 기록한다.

    매 바퀴 기준 시각을 다시 잡는다(절대 스케줄 누적이 아님) — 재려는 건
    "이번 한 번의 깨움이 얼마나 늦었나"이지 누적 드리프트가 아니고, 파이썬이
    주기를 못 따라가는 환경에서 지연이 눈덩이처럼 불어나는 것도 막는다."""
    clock = time.CLOCK_MONOTONIC
    sleep_s = period_ns / 1e9
    late: list[int] = []
    deadline = time.clock_gettime_ns(clock) + int(seconds * 1e9)
    while True:
        target = time.clock_gettime_ns(clock) + period_ns
        time.sleep(sleep_s)
        now = time.clock_gettime_ns(clock)
        late.append(now - target)
        if now >= deadline:
            return late


def report(late_ns: list[int], period_ns: int, freq: int) -> None:
    raw = sorted(v / 1000.0 for v in late_ns)
    n = len(raw)
    T = 1e6 / (2 * freq)      # 판정 기준선 = f_max에서의 에지 간격 (샘플링 주기와 별개)
    # 바닥 = 선점이 없었던 최선의 깨움 = sleep 문법·인터프리터 오버헤드.
    # 선점 지연은 그 위에 얹히므로, 판정은 바닥을 뺀 초과분으로 한다.
    floor = raw[max(0, n // 100)]   # p1 — 최솟값 하나에 전체가 흔들리지 않게
    us = [v - floor for v in raw]
    q = lambda p: us[min(n - 1, int(p / 100 * n))]

    over_half = sum(1 for v in us if v > T / 2)
    over_one = sum(1 for v in us if v > T)

    print(f"\n{'=' * 66}")
    print(f"기준선: {freq}Hz 펄스의 에지 간격 T = {T:.1f}us")
    print(f"샘플 {n}개 (주기 {period_ns / 1000:.0f}us), 측정 바닥 {floor:.1f}us"
          f" — 아래는 바닥을 뺀 초과분 = 선점당한 시간")
    print(f"\n[선점 초과분 us]  median {q(50):8.1f}   mean {st.fmean(us):8.1f}")
    print(f"                  p90 {q(90):8.1f}   p99 {q(99):8.1f}   "
          f"p99.9 {q(99.9):8.1f}   max {us[-1]:8.1f}")
    print(f"\n[(a) 간격 지터]  T/2({T / 2:.1f}us) 초과 {over_half}개 "
          f"({100.0 * over_half / n:.2f}%) — 에지가 눈에 띄게 밀린 횟수")
    print(f"[(b) 펄스 몰림]  T({T:.1f}us) 초과 {over_one}개 "
          f"({100.0 * over_one / n:.2f}%) — 슬롯을 통째로 놓쳐 몰아 내는 횟수")
    if over_one:
        print("  ⚠ 몰림 발생 = 탈조 위험. f_max 상향은 이 값이 0이 되기 전엔 금물.")
    else:
        print("  몰림 0 = 스텝을 잃는 쪽은 아니고, 소리는 (a) 지터 쪽이다.")

    # 분포 — 한 덩어리면 지터, 위쪽에 따로 떨어진 덩어리가 있으면 선점
    lo, hi = q(1), q(99.5)
    if hi > lo:
        bins = 16
        w = (hi - lo) / bins
        counts = [0] * bins
        for v in us:
            if lo <= v <= hi:
                counts[min(bins - 1, int((v - lo) / w))] += 1
        peak = max(counts) or 1
        print("\n[분포]")
        for i, c in enumerate(counts):
            e0, e1 = lo + i * w, lo + (i + 1) * w
            mark = " <T/2" if e0 <= T / 2 < e1 else (" <T" if e0 <= T < e1 else "")
            print(f"  {e0:7.1f}~{e1:7.1f}us |{'#' * int(40 * c / peak):<40}| {c:6d}{mark}")
        print(f"  (범위 밖 위쪽 {sum(1 for v in us if v > hi)}개 — 이 꼬리가 선점이다)")
    print("=" * 66)


def main() -> None:
    p = argparse.ArgumentParser(description="스레드 웨이크업 지연 측정 (배선 불필요)")
    p.add_argument("--freq", type=int, default=6000,
                   help="판정 기준선이 될 펄스 주파수 Hz (기본 6000 = config f_max). "
                        "에지 간격 T = 1/(2f). 샘플링 주기와는 별개다")
    p.add_argument("--period-us", type=int, default=500,
                   help="샘플링 주기 us (기본 500). 파이썬 sleep 바닥의 몇 배로 "
                        "넉넉히 잡아야 바닥이 신호를 삼키지 않는다")
    p.add_argument("--sec", type=float, default=10.0, help="측정 시간(초)")
    p.add_argument("--load", type=int, default=0,
                   help="배경 busy loop 프로세스 수 (추적 대신 합성 부하를 쓸 때. Pi 5는 6)")
    p.add_argument("--slack-ns", type=int, default=None,
                   help="타이머 슬랙을 이 값(ns)으로 낮추고 측정 (기본: 커널 기본값 50000 유지). "
                        "--slack-ns 1 과 안 준 것을 비교해볼 것")
    p.add_argument("--cpu", type=int, default=None,
                   help="이 코어에만 고정해서 측정 (어피니티 대책 예습)")
    args = p.parse_args()

    period_ns = args.period_us * 1000

    if args.slack_ns is not None:
        set_timer_slack(args.slack_ns)
    if args.cpu is not None:
        os.sched_setaffinity(0, {args.cpu})

    stop = procs = None
    if args.load > 0:
        stop = mp.Event()
        procs = [mp.Process(target=_busy, args=(stop,), daemon=True) for _ in range(args.load)]
        for pr in procs:
            pr.start()
        time.sleep(0.3)

    aff = sorted(os.sched_getaffinity(0))
    print(f"[jitter] 기준 {args.freq}Hz, 주기 {args.period_us}us, {args.sec:g}s, "
          f"부하 {args.load}개, 코어 {aff} — 측정 중…")
    try:
        late = probe(period_ns, args.sec)
    finally:
        if stop is not None:
            stop.set()
            for pr in procs:
                pr.join(timeout=2)

    report(late, period_ns, args.freq)


if __name__ == "__main__":
    main()
