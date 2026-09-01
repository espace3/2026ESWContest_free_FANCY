"""
vision/pose_tracker.py

선정된 추적 대상 1인의 부위(머리/상체/하체) 중심 좌표를 시간축으로 안정화 —
순수 계산 모듈 (하드웨어 import 없음).

두 가지 일을 한다:
- EMA(지수이동평균, alpha) 스무딩: 프레임마다 미세하게 튀는 좌표 잡음을 완화해서
  모터가 잡음을 그대로 따라 떨지 않게 한다.
- 부위별 miss 카운트: 어떤 부위가 miss_thr 프레임 "연속으로" 안 보이면 그 부위의
  visible을 False로 내려서, 뒤 단계(각도 계산/모터)가 "이 부위는 지금 유효한
  조준 목표가 아니다"를 알 수 있게 한다. 1~2프레임 잠깐 놓친 것(검출 흔들림)은
  마지막 좌표를 유지한 채 버텨서, 순간 드롭마다 모터가 목표-상실 동작으로
  빠지지 않게 하는 유예 역할이다 (miss_thr=5, 20fps 기준 약 0.25초).
- 재획득/교체 시 즉시 점프(스냅): miss가 miss_thr까지 쌓였다가 다시 보이게 된
  부위는(또는 reset() 직후에는) 낡은 좌표와 EMA로 섞지 않고 새 관측값으로 즉시
  점프한다. EMA는 "같은 사람"의 프레임 간 잡음을 없애는 용도지, 서로 다른 두
  위치(이전 사람↔새 사람, 놓친 위치↔재등장 위치)를 섞으라고 있는 게 아니다 —
  섞으면 조준점이 두 위치 사이 허공을 미끄러지는 아티팩트가 생긴다. 추적 대상이
  다른 사람으로 교체될 때는 호출부가 reset()을 불러 같은 방식으로 점프시킨다.
  (물리적인 부드러움은 모터 쪽 속도 제한이 담당할 몫.)

visible=False가 된 뒤에도 cx/cy는 마지막 관측값을 유지한다(참고용) — 이 값을
쓰는 쪽은 반드시 visible 플래그를 먼저 확인해야 한다. 사라진 위치로 계속
조준하는 사고를 막는 책임은 소비자 쪽에 있다.
"""

from __future__ import annotations


class PoseTracker:
    REGIONS = ("head", "upper", "lower")

    def __init__(self, alpha: float = 0.2, miss_thr: int = 5):
        self.alpha = alpha
        self.miss_thr = miss_thr
        self.smooth = {
            k: {"cx": 0.5, "cy": 0.5, "visible": False}
            for k in self.REGIONS
        }
        # 시작 상태를 "이미 오래 놓친 상태"로 둬서 첫 관측 전까지 visible=False 유지
        self.miss = {k: miss_thr for k in self.REGIONS}

    def reset(self) -> None:
        """추적 대상이 "다른 사람"으로 교체됐을 때 호출부가 부른다.

        다음 관측값을 EMA로 섞지 않고 그대로 받아들이게(즉시 점프) 만든다 —
        이전 사람 좌표와 새 사람 좌표를 섞어 조준점이 허공을 미끄러지는 것 방지.
        대상 교체 감지(직전 대상 위치와의 거리 비교)는 prev_center 상태를 가진
        호출부의 몫이다 (app/camera.py 참고).
        """
        for k in self.REGIONS:
            self.miss[k] = self.miss_thr

    def update(self, result: dict) -> dict:
        """result = {"detected": bool, "regions": {head/upper/lower: {cx, cy, visible}}}

        부위별로: 이번 프레임에 보였으면 EMA 갱신 + miss 리셋, 안 보였으면(부위만
        가려졌든 사람 전체를 놓쳤든) miss를 올리고 miss_thr 연속 도달 시 visible을
        내린다. 사람은 계속 보이는데 특정 부위만 오래 가려지는 경우(예: 책상 뒤
        하체)에도 그 부위가 낡은 좌표로 visible=True인 채 남지 않게 하기 위해
        사람 단위가 아니라 부위 단위로 카운트한다.

        단, miss가 miss_thr까지 쌓인 상태에서(또는 reset() 직후) 다시 보이게 된
        부위는 EMA 없이 새 관측값으로 즉시 점프한다 — 낡은 좌표와 새 좌표를
        섞는 건 잡음 제거가 아니라 허공 조준이다.
        """
        detected = result["detected"]

        for name, r in result["regions"].items():
            s = self.smooth[name]
            if detected and r["visible"]:
                if self.miss[name] >= self.miss_thr:
                    # 오래 놓쳤다 재획득(또는 reset() 후 첫 관측) → 즉시 점프
                    s["cx"], s["cy"] = r["cx"], r["cy"]
                else:
                    s["cx"] = self.alpha * r["cx"] + (1 - self.alpha) * s["cx"]
                    s["cy"] = self.alpha * r["cy"] + (1 - self.alpha) * s["cy"]
                self.miss[name] = 0
                s["visible"] = True
            else:
                # 좌표는 마지막 관측값을 유지하고, 연속 miss가 쌓이면 visible만 내림
                self.miss[name] += 1
                if self.miss[name] >= self.miss_thr:
                    s["visible"] = False

        return self.smooth
