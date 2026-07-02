# ESW

2026 ESW Contest Codes — Team FANCY, "AI 스마트 타겟 선풍기"

카메라로 사용자를 실시간 인식·추적해 팬틸트 헤드로 바람 방향을 자동 조준하고,
머리/상체/하체 3부위를 구분해 부위별 맞춤 풍량을 제공하는 임베디드 프로젝트입니다.
스마트폰 앱(Flutter)과 BLE로 연동하며, 모든 추론은 온디바이스(Raspberry Pi 5)로 처리합니다.

## 플랫폼 & 스택

- 보드: Raspberry Pi 5 + Pi Camera Module 3
- OS/언어: Raspberry Pi OS Lite 64-bit, Python 3.11.15
- 추론: TFLite Runtime, MoveNet MultiPose Lightning (256x256, 17 keypoints, 최대 6명 동시 검출)
- 영상: OpenCV 4.x / BLE: BlueZ 5.x GATT 서버 / 앱: Flutter(Dart)

## 구조

```
vision/              # 순수 계산 모듈 (하드웨어 의존성 없음)
  pose_estimator.py    # MoveNetMultiPoseDetector: 프레임 → 검출된 모든 사람의 키포인트/부위(머리·상체·하체) 중심 좌표
  pose_tracker.py       # PoseTracker: 선정된 대상자 부위 중심 좌표 EMA 스무딩
  target_selector.py    # 다중 인원 중 bbox 면적 최대 1인 선정 (면적 비슷하면 기존 대상자 유지하는 히스테리시스 포함)
control/             # 순수 계산 모듈 (하드웨어 의존성 없음)
  control_signal_generator.py  # 좌표→각도 변환, 데드존, 거리 추정, 풍속 단계 매핑
hardware/            # 하드웨어 호출 전용 모듈 (계산 모듈이 만든 값을 GPIO로 내보내기만 함)
  motor_controller.py  # 팬틸트 스테퍼 모터 구동 — 아직 인터페이스만 정의된 STUB
scripts/             # 단계별 수동 검증 스크립트
  verify_movenet.py     # 1단계: 카메라 캡처 + 시각화/웹스트림으로 포즈 추정 확인
config.py            # 전체 설정값 (CFG 딕셔너리)
```

## 실행

```bash
pip install tflite-runtime opencv-python numpy
python scripts/verify_movenet.py                                    # 로컬 창으로 확인
python scripts/verify_movenet.py --no-window                        # 헤드리스
python scripts/verify_movenet.py --web --no-window --web-port 8090  # http://<host>:8090/ 로 MJPEG 스트림
python scripts/verify_movenet.py --opencv --cam 0                   # OpenCV/V4L2 캡처 강제 (개발 PC / USB 웹캠)
python scripts/verify_movenet.py --rpicam                           # rpicam-vid 서브프로세스로 캡처 (picamera2 미설치 시)
```

레포 루트에서 실행하세요. `multipose_lightning.tflite` 모델 파일이 레포 루트에 있어야 합니다 (레포에는 포함되어 있지 않음, [Kaggle Models](https://www.kaggle.com/models/google/movenet/tfLite/multipose-lightning-tflite-float16/1)에서 별도 다운로드 필요).

## 개발 단계

1. Pi 5 FPS 검증 (현재 단계 — `scripts/verify_movenet.py`)
2. 팬틸트 모터 제어
3. BLE · 앱 연동
4. 전체 통합 · 성능 지표 측정

각 단계는 독립적으로 검증 가능하도록 모듈화합니다.

## 아키텍처 원칙

**계산 로직과 하드웨어 호출을 절대 같은 함수에 섞지 않습니다.**

- **계산 전용 모듈** (GPIO/BlueZ 등 하드웨어 라이브러리 import 금지, 입력은 프레임/키포인트,
  출력은 각도·신호값 등 순수 데이터): 포즈 추정, 부위 판별, 대상자 선정, 제어 신호 생성.
  지금은 `vision/pose_estimator.py`, `vision/target_selector.py`, `vision/pose_tracker.py`,
  `control/control_signal_generator.py`가 이 원칙을 따릅니다.
- **하드웨어 호출 전용 모듈** (계산 모듈이 만든 값을 받아 GPIO/UART/BLE로 내보내기만 함):
  릴레이 제어, 모터 제어, BLE 서버. `hardware/motor_controller.py`는 아직 배선/드라이버가
  정해지지 않아 인터페이스만 있는 STUB 상태입니다 (실제 GPIO 코드는 하드웨어 스펙이
  정해진 뒤에 채울 것).
- 예: `compute_pan_angle(cx_norm, fov_h_deg) -> float`처럼 순수 함수로 각도를 계산하고,
  `motor_controller.move_to(angle)`이 실제 GPIO 호출을 전담합니다. 이렇게 분리해두면
  모터 드라이버를 바꾸거나 계산 버그를 찾을 때 서로 영향 없이 수정·검증할 수 있습니다.

카메라 캡처·추론·모터 제어·BLE는 동시에 동작하므로 스레딩/멀티프로세싱 구조를 처음부터
설계합니다 (`scripts/verify_movenet.py`의 웹스트림 모드가 캡처/추론 스레드 + HTTP 서버
스레드 분리 예시입니다).

구현 전 알고리즘을 문서로 정리하고, 코드 변경 시 문서도 함께 갱신합니다.

## 성능 목표

- 추론 FPS: 목표 20fps (팀 목표치, 추후 변경 가능) — 모델을 바꾸면(예: 싱글포즈 → 멀티포즈)
  반드시 다시 측정할 것. 한 모델에서 잰 수치가 다른 모델에도 그대로 적용되는 건 아님
- 전체 시스템 응답 시간 < 0.5s
- 객체 인식 mAP ≥ 66%
- 부위 전환 정확도 ≥ 90%
- BLE 제어 응답 < 0.2s (지연 측정 로직 포함 필요)
- 동작 소음 < 35dB
- 모터 위치 정확도 < 2°

성능 목표에 영향을 주는 파이프라인 단계(추론, 모터 이동, BLE 왕복 등)를 추가할 때는
`scripts/verify_movenet.py`의 FPS 로깅 패턴(`fps_hist`, 초당 콘솔 로그)처럼 측정 코드를
함께 포함합니다.
