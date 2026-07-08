#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/verify_motor.py - RPi 5 + lgpio tx_pwm 스텝모터 구동 통합 스크립트

단순 모터 구동 테스트용으로, direction 방향 제어와 x축 일반 모터와 y축 1:100 웜기어 모터 제어를 지원합니다.

설치:
    pip install lgpio

실행 :
    python3 scripts/verify_motor.py --type 1      # X축 일반 모터
    python3 scripts/verify_motor.py --type 100    # Y축 1:100 웜기어 모터

    옵션:
    --rev N        회전수 지정 (기본값: 모터별 기본치)
    --dir 0|1      방향 (기본 1)
    --sweep N      왕복 N사이클 실행 (지정 시 한 방향 테스트 대신 왕복)
"""

import argparse
import time
import lgpio

# ── 모터별 설정 ──
MOTORS = {
    1: {                          # X축: 일반 스텝모터 (감속기 없음)
        "name": "X축 일반 모터",
        "EN": 17,
        "STEP": 27,
        "DIR": 22,
        "F_START": 1500,
        "F_MAX": 10000,           # lgpio tx_pwm 하드 리밋
        "RAMP_SEGMENTS": 8,
        "RAMP_STEPS_PER_SEG": 40,
        "DEFAULT_REV": 10,        # 감속기 없음: 10회전이면 확인 충분
    },
    100: {                        # Y축: 1:100 웜기어 모터
        "name": "Y축 1:100 웜기어 모터",
        "EN": 17,
        "STEP": 20,
        "DIR": 21,
        "F_START": 800,
        "F_MAX": 10000,
        "RAMP_SEGMENTS": 12,
        "RAMP_STEPS_PER_SEG": 60,
        "DEFAULT_REV": 50,        # 모터축 50회전 = 출력축 180도
    },
}

STEPS_PER_REV = 200
MICROSTEP = 8                     # TMC2209 standalone: MS1, MS2 점퍼 없음
SPR = STEPS_PER_REV * MICROSTEP   # 1600 펄스/회전


class Stepper:
    def __init__(self, handle, cfg):
        self.h = handle
        self.cfg = cfg
        lgpio.gpio_claim_output(self.h, cfg["EN"], 1)   # 초기 비활성 (LOW 활성)
        lgpio.gpio_claim_output(self.h, cfg["DIR"], 0)
        lgpio.gpio_claim_output(self.h, cfg["STEP"], 0)

    def enable(self):
        lgpio.gpio_write(self.h, self.cfg["EN"], 0)

    def disable(self):
        lgpio.gpio_write(self.h, self.cfg["EN"], 1)

    def _queue(self, freq, steps):
        while lgpio.tx_room(self.h, self.cfg["STEP"], lgpio.TX_PWM) < 1:
            time.sleep(0.002)
        lgpio.tx_pwm(self.h, self.cfg["STEP"], freq, 50, 0, steps)

    def _wait(self):
        while lgpio.tx_busy(self.h, self.cfg["STEP"], lgpio.TX_PWM):
            time.sleep(0.005)

    def move(self, revolutions, direction):
        cfg = self.cfg
        total_steps = int(revolutions * SPR)
        lgpio.gpio_write(self.h, cfg["DIR"], direction)
        time.sleep(0.001)                                # DIR 셋업 타임

        n_seg = cfg["RAMP_SEGMENTS"]
        seg_steps = cfg["RAMP_STEPS_PER_SEG"]
        ramp = [int(cfg["F_START"] + (cfg["F_MAX"] - cfg["F_START"]) * (i + 1) / n_seg)
                for i in range(n_seg)]
        ramp_steps = n_seg * seg_steps
        if total_steps < 2 * ramp_steps + 100:
            raise ValueError("이동량이 너무 짧아 램프가 겹칩니다. 회전수를 늘리세요.")
        cruise_steps = total_steps - 2 * ramp_steps

        t0 = time.perf_counter()
        for f in ramp:                                   # 가속
            self._queue(f, seg_steps)
        self._queue(cfg["F_MAX"], cruise_steps)          # 순항
        for f in reversed(ramp):                         # 감속
            self._queue(f, seg_steps)
        self._wait()
        print(f"[{cfg['name']}] 이동 완료: {revolutions}회전, "
              f"소요 {time.perf_counter() - t0:.2f}초")

    def sweep(self, revolutions, cycles, pause=0.2):
        for i in range(cycles):
            print(f"[{self.cfg['name']}] 사이클 {i + 1}/{cycles}")
            self.move(revolutions, 1)
            time.sleep(pause)
            self.move(revolutions, 0)
            time.sleep(pause)


def open_chip():
    for chip in (0, 4):
        try:
            return lgpio.gpiochip_open(chip)
        except lgpio.error:
            continue
    raise RuntimeError("gpiochip을 열 수 없습니다. lgpio 설치를 확인하세요.")


def main():
    parser = argparse.ArgumentParser(description="스텝모터 구동 (X: 일반, Y: 1:100 웜기어)")
    parser.add_argument("--type", type=int, choices=[1, 100], required=True,
                        help="1: X축 일반 모터, 100: Y축 웜기어 모터")
    parser.add_argument("--rev", type=float, default=None, help="회전수 (모터축 기준)")
    parser.add_argument("--dir", type=int, choices=[0, 1], default=1, help="방향")
    parser.add_argument("--sweep", type=int, default=None, help="왕복 사이클 수")
    args = parser.parse_args()

    cfg = MOTORS[args.type]
    if cfg["STEP"] == 0:
        raise SystemExit(f"{cfg['name']}의 핀 번호가 더미(0) 상태입니다. "
                         f"MOTORS 설정에서 실제 핀으로 교체하세요.")
    rev = args.rev if args.rev is not None else cfg["DEFAULT_REV"]

    h = open_chip()
    motor = Stepper(h, cfg)
    try:
        motor.enable()
        if args.sweep:
            motor.sweep(rev, args.sweep)
        else:
            motor.move(rev, args.dir)
    finally:
        motor.disable()
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()