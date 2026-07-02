"""
vision/pose_estimator.py

MoveNet MultiPose Lightning 기반 포즈 추정 — 순수 계산 모듈.
GPIO/BlueZ 등 하드웨어 라이브러리를 import하지 않습니다.
입력: BGR 프레임(np.ndarray) / 출력: 프레임에 보이는 사람들의 키포인트·부위 중심
좌표(dict) — 순수 데이터.

이 모듈은 "보이는 사람 전부"를 반환하는 것까지만 책임진다. 그중 추적할 1명을
고르는 건 target_selector.select_target()의 역할이고, 시간에 따른 스무딩은
pose_tracker.PoseTracker의 역할이다 (역할 분리).

카메라 캡처, 시각화, 모터/BLE 제어는 이 모듈이 아니라
scripts/, hardware/ 쪽에서 이 모듈의 결과를 가져다 씁니다.
"""

from __future__ import annotations

import sys

import cv2
import numpy as np

# ── MoveNet 키포인트 정의 (COCO 17개) ────────────────────────────────────────
# 0:nose 1:l_eye 2:r_eye 3:l_ear 4:r_ear
# 5:l_shoulder 6:r_shoulder 7:l_elbow 8:r_elbow 9:l_wrist 10:r_wrist
# 11:l_hip 12:r_hip 13:l_knee 14:r_knee 15:l_ankle 16:r_ankle
KP_NAMES = [
    "nose", "l_eye", "r_eye", "l_ear", "r_ear",
    "l_shoulder", "r_shoulder", "l_elbow", "r_elbow", "l_wrist", "r_wrist",
    "l_hip", "r_hip", "l_knee", "r_knee", "l_ankle", "r_ankle",
]
HEAD_IDX = [0, 3, 4]            # nose, l_ear, r_ear
UPPER_IDX = [5, 6, 11, 12]      # shoulders + hips
LOWER_IDX = [13, 14, 15, 16]    # knees + ankles

# 골격 연결선 (시각화 쪽에서 그대로 가져다 씀)
SKELETON = [
    (0, 1), (0, 2), (1, 3), (2, 4),           # 얼굴
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),  # 상체
    (5, 11), (6, 12), (11, 12),               # 몸통
    (11, 13), (13, 15), (12, 14), (14, 16),   # 하체
]


class MoveNetMultiPoseDetector:
    """MoveNet MultiPose Lightning TFLite 래퍼. 순수 계산만 수행 (하드웨어 호출 없음).

    한 프레임에서 최대 MAX_PEOPLE명까지 키포인트를 동시에 뽑아 반환한다.
    """

    # MultiPose Lightning 공식 기본 입력 크기는 256(32의 배수)이지만, Pi 5 FPS
    # 목표(20fps)를 맞추기 위해 192로 낮춰서 사용 중 — 완전 컨볼루션 구조라
    # 32의 배수면 동작은 하지만, 256 기준으로 학습된 모델이라 특히 작게 잡히는
    # 사람/먼 거리 키포인트 정확도가 떨어질 수 있음. mAP 재측정 필요.
    INPUT_SIZE = 192
    MAX_PEOPLE = 6

    # 출력 텐서 한 사람당 56개 값 중: [0:51]=키포인트17*(y,x,score), [51:55]=bbox(ymin,xmin,ymax,xmax), [55]=인물 전체 점수
    _KP_BLOCK_LEN = 51
    _SCORE_IDX = 55

    def __init__(
        self,
        model_path: str,
        conf_thr: float = 0.3,
        min_person_score: float = 0.15,
    ) -> None:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            try:
                from tensorflow.lite.python.interpreter import Interpreter
            except ImportError:
                print("[ERROR] tflite-runtime 또는 tensorflow가 필요합니다.")
                print("  설치: pip install tflite-runtime")
                sys.exit(1)

        self._interp = Interpreter(
            model_path=model_path,
            num_threads=3
        )
        # MultiPose Lightning은 입력 크기가 동적이라, allocate 전에 반드시
        # 실제로 쓸 크기로 resize를 먼저 해줘야 한다. 안 하면 모델에 잡혀있는
        # 임시 크기(보통 1)로 텐서가 할당돼서 "got 256 but expected 1" 같은
        # shape mismatch 에러가 난다.
        input_index = self._interp.get_input_details()[0]['index']
        self._interp.resize_tensor_input(input_index, [1, self.INPUT_SIZE, self.INPUT_SIZE, 3])
        self._interp.allocate_tensors()
        self._inp = self._interp.get_input_details()[0]
        self._out = self._interp.get_output_details()[0]
        self._conf_thr = conf_thr
        self._min_person_score = min_person_score
        self._s = self.INPUT_SIZE

        print(f"[movenet] 모델 로드: {model_path}")
        print(f"[movenet] 입력: {self._inp['shape']}  dtype={self._inp['dtype'].__name__}")
        print(f"[movenet] 출력: {self._out['shape']}")
        print(f"[movenet] conf_thr={conf_thr}  min_person_score={min_person_score}")

    def infer(self, frame_bgr: np.ndarray) -> dict:
        """
        BGR 프레임 → 포즈 결과 dict 반환.
        반환:
          detected: bool                 — 사람이 1명이라도 보이면 True
          people: list of {
              keypoints: list of {x,y,conf}  (정규화 좌표 [0,1], 원본 프레임 기준)
              score:     float               (인물 전체 검출 점수)
              regions:   {head, upper, lower} → {cx, cy, visible} (정규화)
          }
        """
        s = self._s
        h_orig, w_orig = frame_bgr.shape[:2]

        # ── 전처리: letterbox → s x s ────────────────────────────────────
        scale = min(s / w_orig, s / h_orig)
        nw, nh = int(w_orig * scale), int(h_orig * scale)
        pad_x = (s - nw) // 2
        pad_y = (s - nh) // 2

        resized = cv2.resize(frame_bgr, (nw, nh))
        canvas = np.full((s, s, 3), 0, dtype=np.uint8)
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized

        # BGR → RGB
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

        if self._inp['dtype'] == np.uint8:
            inp_arr = rgb[np.newaxis].astype(np.uint8)
        else:
            inp_arr = (rgb.astype(np.float32) / 255.0)[np.newaxis]

        # ── 추론 ─────────────────────────────────────────────────────────
        self._interp.set_tensor(self._inp['index'], inp_arr)
        self._interp.invoke()
        raw = self._interp.get_tensor(self._out['index'])  # (1, MAX_PEOPLE, 56)
        instances = raw[0]

        people = []
        for row in instances:
            person_score = float(row[self._SCORE_IDX])
            if person_score < self._min_person_score:
                continue

            keypoints = []
            kp_block = row[: self._KP_BLOCK_LEN].reshape(17, 3)  # (17, [y, x, score])
            for ky, kx, kc in kp_block:
                # letterbox 내 픽셀 → 원본 픽셀 → 정규화
                px = (float(kx) * s - pad_x) / scale
                py = (float(ky) * s - pad_y) / scale
                keypoints.append({
                    "x": max(0.0, min(1.0, px / w_orig)),
                    "y": max(0.0, min(1.0, py / h_orig)),
                    "conf": float(kc),
                })

            people.append({
                "keypoints": keypoints,
                "score": person_score,
                "regions": self._compute_regions(keypoints),
            })

        return {
            "detected": len(people) > 0,
            "people": people,
        }

    def _compute_regions(self, keypoints: list) -> dict:
        head_conf = [keypoints[i]["conf"] for i in HEAD_IDX]
        upper_conf = [keypoints[i]["conf"] for i in UPPER_IDX]
        lower_conf = [keypoints[i]["conf"] for i in LOWER_IDX]

        head_visible = any(c >= self._conf_thr for c in head_conf)
        upper_visible = sum(c >= self._conf_thr for c in upper_conf) >= 2
        lower_visible = sum(c >= self._conf_thr for c in lower_conf) >= 2

        return {
            "head": self._region_center(keypoints, HEAD_IDX) if head_visible else {"cx": 0.5, "cy": 0.5, "visible": False},
            "upper": self._region_center(keypoints, UPPER_IDX) if upper_visible else {"cx": 0.5, "cy": 0.5, "visible": False},
            "lower": self._region_center(keypoints, LOWER_IDX) if lower_visible else {"cx": 0.5, "cy": 0.5, "visible": False},
        }

    def _region_center(self, kps: list, idxs: list[int]) -> dict:
        pts = [(kps[i]["x"], kps[i]["y"])
               for i in idxs if kps[i]["conf"] >= self._conf_thr]
        if not pts:
            return {"cx": 0.5, "cy": 0.5, "visible": False}
        return {
            "cx": float(sum(p[0] for p in pts) / len(pts)),
            "cy": float(sum(p[1] for p in pts) / len(pts)),
            "visible": True,
        }
