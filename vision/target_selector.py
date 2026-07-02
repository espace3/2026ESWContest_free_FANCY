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


def select_target(people: list[list[dict]], conf_thr: float = 0.25) -> int | None:
    """여러 사람의 키포인트 리스트 중 bounding box 면적이 가장 큰 1인의 인덱스를 반환한다.

    people[i]는 MoveNetMultiPoseDetector.infer()의 result["people"][i]["keypoints"]와
    같은 형식(list of {"x", "y", "conf"}, 17개)이라고 가정한다.

    면적이 큰 사람을 고르는 것은 "대체로 카메라에 더 가까운 사람"이라는 근사치를
    쓰는 것이다 — 사람마다 거리(estimate_distance_m)를 계산하는 것보다 연산이 싸다.
    유효한(키포인트가 하나라도 잡힌) 사람이 없으면 None을 반환한다.
    """
    best_idx: int | None = None
    best_area = -1.0
    for i, keypoints in enumerate(people):
        bbox = _bbox_from_keypoints(keypoints, conf_thr)
        if bbox is None:
            continue
        area = _bbox_area(bbox)
        if area > best_area:
            best_area = area
            best_idx = i
    return best_idx
