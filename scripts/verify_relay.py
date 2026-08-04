"""
scripts/verify_relay.py - 풍속 릴레이(TS0011) 단독 실기 검증

이 파일은 계산 로직을 포함하지 않습니다. hardware/relay_controller.py의
FanRelay(논블로킹)에 풍속 단계를 보내고 전환 로그를 출력만 합니다.
BLE 없이 릴레이 배선/극성/guard 동작을 소프트웨어 문제와 분리해 확인하는
용도입니다. 판정은 릴레이 모듈 LED·접점 소리(딸깍)·멀티미터로 사람이 합니다.

실행 (RPi 5, 레포 루트에서):
    python scripts/verify_relay.py                       # 정지→미풍→약풍→강풍→정지 순환 (기본 2s 유지, 1회)
    python scripts/verify_relay.py --hold 3 --cycles 2   # 각 단계 3초 유지, 2회 순환
    python scripts/verify_relay.py --level 2             # 약풍만 켜고 hold초 유지 후 정지

확인 항목:
    - 전환마다 "전부 오픈 → guard(0.1s) → 단일 ON" 순서로만 바뀌는지 (동시에
      두 IN LED가 켜지는 순간이 없어야 함 — break-before-make)
    - 극성: set_speed 후 해당 채널만 ON이면 config relay.active_high=True 맞음.
      반대로 나머지 둘이 켜지면 active_high가 뒤집힌 것 — 배선 재확인.
    - 종료(Ctrl+C 포함) 시 전부 오픈(정지)으로 끝나는지.

⚠ 부팅 안전 선행 체크 (이 스크립트 실행 전, hardware/TODO.md 필수 항목):
    소프트웨어를 하나도 안 띄운 채 Pi에 전원만 넣고 세 릴레이가 전부 오픈
    (정지)인지 확인할 것. 부팅 중 릴레이가 딸깍하며 닫히면 레벨컨버터 자체
    풀업이 Pi 내부 풀다운을 이기는 배선 — 소프트웨어로 못 막으므로 하드웨어
    대책(풀업 제거/단방향 버퍼/외부 강한 풀다운) 필요.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from config import CFG
from hardware.relay_controller import FanRelay

LEVEL_NAMES = {0: "정지(전부 오픈)", 1: "미풍(IN1)", 2: "약풍(IN2)", 3: "강풍(IN3)"}


def set_and_report(fan: FanRelay, level: int) -> None:
    t0 = time.perf_counter()
    fan.set_speed(level)
    fan.wait_until_applied(timeout=5)
    print(f"[{LEVEL_NAMES[level]}] 반영 완료 ({time.perf_counter() - t0:.2f}s — "
          f"guard {CFG['relay']['guard_s']:g}s 포함)")


def main() -> None:
    parser = argparse.ArgumentParser(description="풍속 릴레이 단독 검증")
    parser.add_argument("--hold", type=float, default=2.0, help="각 단계 유지 시간(초)")
    parser.add_argument("--cycles", type=int, default=1, help="순환 반복 횟수")
    parser.add_argument("--level", type=int, choices=(1, 2, 3), default=None,
                        help="이 단계만 켜고 hold초 유지 후 정지 (순환 생략)")
    args = parser.parse_args()

    with FanRelay(CFG) as fan:
        try:
            if args.level is not None:
                set_and_report(fan, args.level)
                time.sleep(args.hold)
            else:
                for c in range(args.cycles):
                    print(f"사이클 {c + 1}/{args.cycles}")
                    for level in (1, 2, 3):
                        set_and_report(fan, level)
                        time.sleep(args.hold)
            set_and_report(fan, 0)
        except KeyboardInterrupt:
            print("\n중단 — 정지(전부 오픈)로 마무리합니다.")
    print("종료: 세 릴레이 모두 오픈 상태인지 확인하세요.")


if __name__ == "__main__":
    main()
