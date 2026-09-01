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
    # 목표(20fps)를 맞추기 위해 낮춰서 사용 중 (256 → 192 → 160 순으로 단계적으로
    # 낮춰가며 테스트) — 완전 컨볼루션 구조라 32의 배수면 동작은 하지만, 256 기준으로
    # 학습된 모델이라 특히 작게 잡히는 사람/먼 거리 키포인트 정확도가 떨어질 수 있음.
    INPUT_SIZE = 160
    MAX_PEOPLE = 6

    # 출력 텐서 한 사람당 56개 값 중: [0:51]=키포인트17*(y,x,score), [51:55]=bbox(ymin,xmin,ymax,xmax), [55]=인물 전체 점수
    _KP_BLOCK_LEN = 51
    _SCORE_IDX = 55

    def __init__(
        self,
        model_path: str,
        conf_thr: float = 0.25,         # 부위 판별 threshold
        min_person_score: float = 0.15, # 인물 판별 threshold
        num_threads: int = 3,           # FPS 튜닝 후 선택
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
            num_threads=num_threads,
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
        print(f"[movenet] conf_thr={conf_thr}  min_person_score={min_person_score}  threads={num_threads}")

    def infer(self, frame_bgr: np.ndarray) -> dict:
        """
        BGR 프레임 → 포즈 결과 dict 반환.
        반환:
          detected: bool                 — 사람이 1명이라도 보이면 True
          people: list of {
              keypoints: list of {x,y,conf}  (정규화 좌표 [0,1], 원본 프레임 기준)
              score:     float               (인물 전체 검출 점수)
              model_bbox: (x_min,y_min,x_max,y_max)  모델이 예측한 bbox(정규화).
                         선정/조준에는 쓰지 않는다 — 비교·시각화용.
              regions:   {head, upper, lower} → {cx, cy, visible} (정규화)
          }
        """
        s = self._s
        h_orig, w_orig = frame_bgr.shape[:2]

        # ── 전처리: letterbox → s x s ────────────────────────────────────
        # 정사각형으로 그냥 늘리면 16:9 프레임의 사람이 가로로 찌그러져
        # 정확도가 떨어진다. 비율을 유지해 넣고 남는 공간을 검게 채운 뒤,
        # 아래 후처리에서 그 패딩만큼 좌표를 되돌린다.
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

            # 모델이 직접 예측한 bbox — 키포인트와 같은 letterbox 좌표계라
            # 같은 역변환을 거친다. 조준/선정에는 쓰지 않고 비교·시각화용이며,
            # 키포인트와 달리 [0,1] 클램프를 하지 않는다(프레임 밖으로 넘어가는
            # 성질 자체가 코드 bbox와의 차이라 보이게 남긴다).
            ymin, xmin, ymax, xmax = row[self._KP_BLOCK_LEN:self._SCORE_IDX]
            model_bbox = tuple(
                v for v in (
                    ((float(xmin) * s - pad_x) / scale) / w_orig,
                    ((float(ymin) * s - pad_y) / scale) / h_orig,
                    ((float(xmax) * s - pad_x) / scale) / w_orig,
                    ((float(ymax) * s - pad_y) / scale) / h_orig,
                )
            )

            people.append({
                "keypoints": keypoints,
                "score": person_score,
                "model_bbox": model_bbox,   # (x_min, y_min, x_max, y_max) 정규화
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
            "upper": self._upper_center(keypoints) if upper_visible else {"cx": 0.5, "cy": 0.5, "visible": False},
            "lower": self._region_center(keypoints, LOWER_IDX) if lower_visible else {"cx": 0.5, "cy": 0.5, "visible": False},
        }

    def _upper_center(self, kps: list) -> dict:
        """상체 조준점. 엉덩이가 둘 다 보이면 기존대로 어깨+엉덩이 4점 평균이고,
        엉덩이가 안 보이면 어깨 중점에서 **머리 반대 방향으로** 연장해 내린다.

        [왜] _region_center는 보이는 점만 평균하므로, 엉덩이가 가려지면(앉은 자세,
        책상) 중심이 어깨선 = 목/쇄골로 올라붙는다. 게다가 부위 모드의 조준 보정
        (main.py --body-upper-ratio)은 "중심이 배꼽 근처"라는 전제로 조준을 더
        위로 올리기 때문에, 두 오차가 같은 방향으로 겹쳐 목·얼굴을 겨눈다.
        엉덩이가 깜빡이면 조준점이 어깨선↔몸통중앙을 프레임마다 오간다.

        [단위] 오프셋을 정규화 좌표의 고정값으로 두면 안 된다 — 화면 속 사람
        크기는 거리에 반비례해서(1m에서 어깨폭 0.162, 3m에서 0.054) 같은 값이
        멀리서는 3배로 과해진다. 머리중점→어깨중점 벡터를 단위로 쓰면 그 길이가
        거리에 같이 줄어 자동으로 상쇄되고, 방향까지 한 번에 얻어 몸이 기울거나
        카메라가 롤 돼도 따라간다.

        [한계] 고개를 숙이면 이 벡터가 짧아져 오프셋도 줄어든다 — 30° 숙임에서
        약 7cm 위로 뜬다. 바람 조준 정밀도에서는 무시할 수준이라 감수한다.
        """
        hips = [i for i in (11, 12) if kps[i]["conf"] >= self._conf_thr]
        if len(hips) == 2:
            return self._region_center(kps, UPPER_IDX)

        sh = self._region_center(kps, [5, 6])
        if not sh["visible"]:
            return self._region_center(kps, UPPER_IDX)   # 엉덩이만 보이는 드문 경우

        head = self._region_center(kps, HEAD_IDX)
        if head["visible"]:
            dx = (sh["cx"] - head["cx"])
            dy = (sh["cy"] - head["cy"])
        else:
            # 머리까지 없으면 방향을 정할 수 없다 — 어깨 폭을 크기로, 화면 아래를 벙향으로
            lx, rx = kps[5]["x"], kps[6]["x"]
            ly, ry = kps[5]["y"], kps[6]["y"]
            dx, dy = 0.0, ((lx - rx) ** 2 + (ly - ry) ** 2) ** 0.5 * 0.62
        return {
            "cx": max(0.0, min(1.0, sh["cx"] + dx)),
            "cy": max(0.0, min(1.0, sh["cy"] + dy)),
            "visible": True,
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
