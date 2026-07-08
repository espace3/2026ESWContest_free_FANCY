"""
scripts/verify_pantilt.py - 팬틸트 모터 2단계 수동 검증 스크립트

이 파일은 계산 로직을 포함하지 않습니다. hardware/motor_controller.py의
MotorController(논블로킹)에 이동 명령을 보내고 결과(소요 시간·최종 장부
위치)를 출력만 합니다. 정확도 판정은 축에 붙인 마커/각도기 눈금과 출력된
장부 각도를 사람이 대조해서 합니다.

실행 (RPi 5, 레포 루트에서):
    python scripts/verify_pantilt.py --axis pan --deg 30       # 단발 이동
    python scripts/verify_pantilt.py --axis pan --sweep 30 --cycles 5
        # ±30° 왕복 5회 후 0° 복귀 — 시작 마커와의 어긋남이 누적 오차 (<2° 목표)
    python scripts/verify_pantilt.py --axis tilt --speed
        # 최고 속도 실측 (tilt 예상 상한 ≈ 22.5°/s — TODO.md 참고)
    python scripts/verify_pantilt.py --axis pan --preempt
        # 이동 중 목표 갈아타기 — 감속→방향 전환→재가속이 부드러운지 눈으로 확인
    python scripts/verify_pantilt.py --axis pan --short
        # 짧은 이동 한계 실측 — 1/5/10/20/50스텝 단발 (TODO.md 항목)
    python scripts/verify_pantilt.py --axis pan --track-sim 10
        # 20Hz 랜덤 잔이동 10초 — 실전 추적 근사 (논블로킹 동작 확인)
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from config import CFG
from hardware.motor_controller import MotorController


def make_mover(mc: MotorController, axis: str):
    """선택한 축만 움직이는 move(deg) 함수를 만든다 (반대 축은 0° 고정)."""
    if axis == "pan":
        return lambda deg: mc.move_to(deg, 0.0)
    return lambda deg: mc.move_to(0.0, deg)


def report(mc: MotorController, label: str, t0: float) -> None:
    pan_deg, tilt_deg = mc.current_position()
    print(f"[{label}] 소요 {time.perf_counter() - t0:.2f}s, "
          f"장부 위치 pan={pan_deg:+.3f}° tilt={tilt_deg:+.3f}°")


def run_single(mc, mv, deg: float) -> None:
    t0 = time.perf_counter()
    mv(deg)
    mc.wait_until_idle()
    report(mc, f"단발 {deg:+g}°", t0)
    print("→ 물리 마커가 장부 각도와 일치하는지 대조하세요.")


def run_sweep(mc, mv, deg: float, cycles: int) -> None:
    t0 = time.perf_counter()
    for i in range(cycles):
        print(f"사이클 {i + 1}/{cycles}")
        mv(+deg); mc.wait_until_idle()
        mv(-deg); mc.wait_until_idle()
    mv(0.0); mc.wait_until_idle()
    report(mc, f"±{deg:g}° x{cycles} 후 0° 복귀", t0)
    print("→ 시작 전에 붙인 마커와의 어긋남이 누적 오차입니다 (<2° 목표).")


def run_speed(mc, mv, axis: str, deg: float) -> None:
    print(f"{axis} {deg:g}° 이동으로 실효 속도를 잽니다...")
    t0 = time.perf_counter()
    mv(deg)
    mc.wait_until_idle()
    dt = time.perf_counter() - t0
    print(f"[속도] {deg:g}° / {dt:.2f}s = {deg / dt:.2f}°/s "
          f"(가감속 포함 평균 — tilt 순항 상한 예상치는 22.5°/s)")
    mv(0.0); mc.wait_until_idle()


def run_preempt(mc, mv, deg: float) -> None:
    print(f"+{deg:g}°로 출발시킨 뒤 0.3s 후 -{deg:g}°로 갈아탑니다...")
    t0 = time.perf_counter()
    mv(+deg)
    time.sleep(0.3)          # 이동 한중간
    mv(-deg)                 # 선점: 감속 → drain → DIR 반전 → 재가속
    mc.wait_until_idle()
    report(mc, "선점 후 최종", t0)
    print("→ 방향 전환이 급반전 없이 감속을 거쳤는지, 최종 마커가 "
          f"-{deg:g}°와 일치하는지 확인하세요.")


def run_short(mc, axis: str) -> None:
    """짧은 이동 실측: 몇 스텝까지 정확히/조용히 움직이는지 (TODO.md 항목)."""
    ax = mc.pan if axis == "pan" else mc.tilt
    print(f"{axis} 1스텝 = {ax.deg_per_step:.5f}°")
    pos = 0
    for steps in (1, 5, 10, 20, 50):
        pos += steps
        deg = pos * ax.deg_per_step
        input(f"Enter를 누르면 +{steps}스텝({steps * ax.deg_per_step:.3f}°) 이동 → ")
        (mc.move_to(deg, 0.0) if axis == "pan" else mc.move_to(0.0, deg))
        mc.wait_until_idle()
        print(f"  장부 누적 {deg:.3f}° — 실제로 움직였는지/소리는 어떤지 기록하세요.")
    mc.move_to(0.0, 0.0)
    mc.wait_until_idle()
    print("0° 복귀 완료.")


def run_track_sim(mc, mv, seconds: float) -> None:
    """20Hz로 ±5° 랜덤워크 목표를 던진다 — 실전 추적(데드존 잔이동) 근사.
    move_to가 즉시 리턴하므로 이 루프는 모터와 무관하게 20Hz를 유지해야 한다."""
    print(f"{seconds:g}s 동안 20Hz 랜덤 목표 송신 (논블로킹 확인)...")
    target, sent, late = 0.0, 0, 0
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        t_loop = time.monotonic()
        target = max(-5.0, min(5.0, target + random.uniform(-1.5, 1.5)))
        mv(target)
        sent += 1
        elapsed = time.monotonic() - t_loop
        if elapsed > 0.05:
            late += 1     # move_to가 블로킹됐다면 여기 잡힌다
        time.sleep(max(0.0, 0.05 - elapsed))
    mv(0.0)
    mc.wait_until_idle()
    print(f"[track-sim] 목표 {sent}회 송신, 50ms 초과 지연 {late}회 "
          f"(0이어야 정상 — 논블로킹 보장), 최종 장부 0° 복귀.")


def main() -> None:
    parser = argparse.ArgumentParser(description="팬틸트 모터 수동 검증")
    parser.add_argument("--axis", choices=["pan", "tilt"], default="pan")
    parser.add_argument("--deg", type=float, default=None, help="이동/왕복 각도")
    parser.add_argument("--sweep", type=float, default=None, help="±N° 왕복 (--cycles회)")
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--speed", action="store_true", help="실효 속도 측정")
    parser.add_argument("--preempt", action="store_true", help="이동 중 목표 갈아타기 검증")
    parser.add_argument("--short", action="store_true", help="짧은 이동(수 스텝) 한계 실측")
    parser.add_argument("--track-sim", type=float, default=None, metavar="SEC",
                        help="20Hz 랜덤 잔이동 시뮬레이션 (초)")
    args = parser.parse_args()

    with MotorController(CFG) as mc:
        mv = make_mover(mc, args.axis)
        mc.enable()
        mc.home()  # 현재 물리 위치 = 0° (임시 호밍) — 시작 위치에 마커를 붙여둘 것
        print(f"[{args.axis}] 시작 위치를 0°로 설정했습니다. 마커를 붙이세요.")
        try:
            if args.sweep is not None:
                run_sweep(mc, mv, args.sweep, args.cycles)
            elif args.speed:
                run_speed(mc, mv, args.axis, args.deg or (90.0 if args.axis == "pan" else 45.0))
            elif args.preempt:
                run_preempt(mc, mv, args.deg or 20.0)
            elif args.short:
                run_short(mc, args.axis)
            elif args.track_sim is not None:
                run_track_sim(mc, mv, args.track_sim)
            else:
                run_single(mc, mv, args.deg if args.deg is not None else 30.0)
        finally:
            mc.wait_until_idle(timeout=30)


if __name__ == "__main__":
    main()
