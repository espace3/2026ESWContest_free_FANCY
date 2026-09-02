"""
vision/target_select.py

다중 인원 감지 시 추적 대상 1인을 선정 — 순수 계산 모듈.
입력은 pose_estimate.infer()가 낸 result["people"] 리스트, 출력은 선택된
사람의 인덱스(순수 데이터)이다.

[선정 규칙]
  ① 직전 대상(prev_center) 근처(match_radius 이내)에 사람이 있으면 그 사람을 유지
  ② 없으면(첫 프레임 / 놓침 / 화면 밖) 화면 중앙에 가장 가까운 사람을 새로 선정

[왜 면적을 안 쓰는가]
예전에는 "bbox 면적이 가장 큰 사람 = 가장 가까운 사람"으로 골랐다. 그런데 면적은
거리보다 **자세에 더 크게 좌우된다** — 실측 기준 2m에서 팔을 벌리면 면적이 3.8배가
되어(0.052 → 0.198) 1m에 팔을 내리고 선 사람(0.210)과 맞먹는다. 반대로 앉거나
하체가 책상에 가리면 면적이 절반 이하로 떨어져 멀리 서 있는 사람에게 밀린다.
즉 "면적 = 거리"라는 근사가 실사용 자세에서 그대로 깨진다.

대신 **한번 잡은 사람을 놓지 않는 것**으로 바꿨다. 대상이 바뀌는 경우는 그 사람을
실제로 놓쳤을 때뿐이라 동작이 예측 가능하고, 자세·가려짐에 아예 영향을 받지 않는다.
초기 선정을 화면 중앙 기준으로 두는 것은 선풍기 앞에 서는 사람이 사용자라는 전제와
맞고, 추적 중에는 대상이 늘 화면 중앙에 오도록 모터가 움직이므로 잠깐 놓쳤다
재획득할 때도 대개 원래 그 사람이 다시 잡힌다.

[중심 좌표는 모델 bbox에서 가져온다]
키포인트 min/max로 bbox를 다시 만들면 conf 문턱과 가려짐에 따라 중심이 흔들린다
(하체가 가리면 중심이 상반신 쪽으로 올라간다). 모델이 직접 예측한 bbox는 그 영향이
적어 중심이 더 안정적이다.

[알려진 한계]
두 사람이 match_radius 안까지 붙어 스쳐 지나가면 더 가까운 쪽으로 대상이 넘어갈 수
있다. 화면의 20%까지 접근해야 생기는 일이라 감수한다.
"""

from __future__ import annotations

# "직전 대상자와 같은 사람"으로 인정하는 최대 이동 거리 (정규화 좌표 기준).
# select_target()의 기본값이자, 호출부가 "대상자가 다른 사람으로 교체됐는지"
# (→ RegionFilter.reset() 필요 여부) 판단할 때도 같은 기준을 쓰도록 공개해 둔다.
DEFAULT_MATCH_RADIUS = 0.2

# 초기 선정 기준점 — 화면 정중앙.
_SCREEN_CENTER = (0.5, 0.5)


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def person_center(person: dict) -> tuple[float, float] | None:
    """이 사람의 중심 좌표(정규화) = 모델 bbox의 중심. 없으면 None.

    select_target()이 골라준 사람을 다음 프레임에서도 "같은 사람"으로 계속
    추적하려면, 이번에 선택된 사람의 중심을 이걸로 구해서 다음 호출의
    prev_center로 넘겨주면 된다.
    """
    bbox = person.get("model_bbox")
    if bbox is None:
        return None
    x_min, y_min, x_max, y_max = bbox
    return ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0)


def select_target(
    people: list[dict],
    prev_center: tuple[float, float] | None = None,
    match_radius: float = DEFAULT_MATCH_RADIUS,
) -> int | None:
    """추적 대상 1인의 인덱스를 반환한다. 대상이 없으면 None.

    people[i]는 MoveNetMultiPoseDetector.infer()의 result["people"][i] 형식
    (특히 "model_bbox" 키가 있어야 한다)이라고 가정한다.

    규칙은 모듈 docstring 참고 — 직전 대상 유지가 우선, 없으면 화면 중앙 최근접.
    """
    candidates = [(i, c) for i, p in enumerate(people)
                  if (c := person_center(p)) is not None]
    if not candidates:
        return None

    if prev_center is not None:
        idx, center = min(candidates, key=lambda c: _dist(c[1], prev_center))
        if _dist(center, prev_center) <= match_radius:
            return idx     # 같은 사람으로 인정 — 면적·자세와 무관하게 유지

    # 첫 프레임이거나 직전 대상을 놓침 → 화면 중앙에 가장 가까운 사람
    return min(candidates, key=lambda c: _dist(c[1], _SCREEN_CENTER))[0]
