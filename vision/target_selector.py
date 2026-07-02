"""
vision/target_selector.py

다중 인원 감지 시 추적 대상 1인을 선정 — 순수 계산 모듈.
입력은 여러 사람의 키포인트 리스트, 출력은 선택된 사람의 인덱스(순수 데이터)이다.

vision/pose_estimator.py의 MoveNetMultiPoseDetector가 반환하는
result["people"][i]["keypoints"] 리스트를 그대로 입력으로 받아 쓴다.
"""

from __future__ import annotations


def _bbox_from_keypoints(
    keypoints: list[dict], conf_thr: float
) -> tuple[float, float, float, float] | None:
    """신뢰도가 conf_thr 이상인 키포인트들의 bounding box를 구한다.

    반환: (x_min, y_min, x_max, y_max) — 정규화 좌표 [0,1] 기준.
    유효 키포인트가 하나도 없으면 None.
    """
    xs = [kp["x"] for kp in keypoints if kp["conf"] >= conf_thr]
    ys = [kp["y"] for kp in keypoints if kp["conf"] >= conf_thr]
    if not xs:
        return None
    return (min(xs), min(ys), max(xs), max(ys))


def _bbox_area(bbox: tuple[float, float, float, float]) -> float:
    x_min, y_min, x_max, y_max = bbox
    return (x_max - x_min) * (y_max - y_min)


def _bbox_center(bbox: tuple[float, float, float, float]) -> tuple[float, float]:
    x_min, y_min, x_max, y_max = bbox
    return ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0)


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def person_center(keypoints: list[dict], conf_thr: float = 0.25) -> tuple[float, float] | None:
    """이 사람의 bbox 중심 좌표(정규화)를 반환한다. 유효 키포인트가 없으면 None.

    select_target()이 골라준 사람을 다음 프레임에서도 "같은 사람"으로 계속
    추적하려면, 이번에 선택된 사람의 중심 좌표를 이걸로 구해서 다음 호출의
    prev_center로 넘겨주면 된다.
    """
    bbox = _bbox_from_keypoints(keypoints, conf_thr)
    if bbox is None:
        return None
    return _bbox_center(bbox)


def select_target(
    people: list[list[dict]],
    conf_thr: float = 0.25,
    prev_center: tuple[float, float] | None = None,
    switch_margin: float = 0.15,
    match_radius: float = 0.2,
) -> int | None:
    """여러 사람의 키포인트 리스트 중 추적 대상 1인의 인덱스를 반환한다.

    people[i]는 MoveNetMultiPoseDetector.infer()의 result["people"][i]["keypoints"]와
    같은 형식(list of {"x", "y", "conf"}, 17개)이라고 가정한다.

    기본 규칙은 bounding box 면적이 가장 큰 사람을 고르는 것 — "대체로 카메라에
    더 가까운 사람"이라는 근사치를 쓰는 것이다. 다만 사람 여러 명의 면적이 비슷할
    때 매 프레임 살짝 더 큰 쪽으로 대상이 계속 바뀌면(flickering) 팬틸트가 계속
    흔들리게 되므로, 직전에 선택했던 사람(prev_center)이 있으면 그 사람을 계속
    유지하고, 새 후보가 면적 기준으로 switch_margin(기본 15%) 이상 확실히 더 클
    때만 갈아탄다 (히스테리시스).

    이 모델은 사람마다 고유 ID를 주지 않으므로, "직전 대상자와 같은 사람"인지는
    prev_center와의 위치 거리로 판단한다 — match_radius(정규화 좌표 기준) 이내에
    있는 후보만 같은 사람으로 인정하고, 그보다 멀면(놓쳤거나 화면 밖으로 나감)
    다시 면적 최대인 사람으로 새로 선정한다.

    상태(직전 대상자 위치)는 호출하는 쪽이 들고 있다가 넘겨야 한다 — 이 함수는
    순수 함수만 두고 상태를 갖지 않는다. 선택된 사람의 중심 좌표는 person_center()로
    구해서 다음 호출의 prev_center로 넘기면 된다.

    유효한(키포인트가 하나라도 잡힌) 사람이 없으면 None을 반환한다.
    """
    candidates: list[tuple[int, float, tuple[float, float]]] = []
    for i, keypoints in enumerate(people):
        bbox = _bbox_from_keypoints(keypoints, conf_thr)
        if bbox is None:
            continue
        candidates.append((i, _bbox_area(bbox), _bbox_center(bbox)))

    if not candidates:
        return None

    best_idx, best_area, _ = max(candidates, key=lambda c: c[1])

    if prev_center is None:
        return best_idx

    prev_match = min(candidates, key=lambda c: _dist(c[2], prev_center))
    if _dist(prev_match[2], prev_center) > match_radius:
        # 직전 대상자로 볼 만한 사람이 없음 (놓쳤거나 화면 밖으로 나감)
        return best_idx

    prev_idx, prev_area, _ = prev_match
    if best_idx == prev_idx:
        return prev_idx

    if best_area > prev_area * (1.0 + switch_margin):
        return best_idx
    return prev_idx
