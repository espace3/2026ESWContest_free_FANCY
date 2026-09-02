# AI 스마트 타겟 선풍기

본 프로젝트는 팀 **FANCY**의 2026 임베디드 소프트웨어 경진대회(자유공모) 출품작입니다.

카메라로 사용자를 실시간 인식·추적해 팬틸트 헤드로 바람 방향을 자동 조준하고,
**머리 / 상체 / 하체** 3부위를 구분해 부위별 맞춤 풍속을 제공합니다.

## 프로젝트 목표

<!-- TODO: 아래 세 줄은 초안입니다. 팀이 실제로 세운 목표 문장으로 바꿔 주세요. -->

- **사람을 따라가는 바람** — 기존 선풍기는 사람이 어디 있든 정해진 궤적으로만 회전합니다.
  카메라로 사용자를 찾아 팬틸트 헤드가 실시간으로 조준합니다.
- **부위별 맞춤 풍속** — 얼굴에는 약하게, 몸통에는 세게처럼 부위마다 원하는 세기가 다릅니다.
  머리·상체·하체를 구분해 사용자가 앱에서 정한 세기를 각각 적용합니다.
- **기존 선풍기에 얹는 구조** — 220V 구동부는 그대로 두고 풍속 탭만 릴레이로 단속합니다.
  새 제품을 만드는 대신 쓰던 선풍기를 스마트화하는 접근입니다.
- 스마트폰 앱(Flutter)과 BLE로 연동하며, **모든 추론은 온디바이스**(Raspberry Pi 5)에서
  처리합니다 — 클라우드·외부 네트워크 호출이 없습니다.

---

## 시스템 개요

```
      Pi Camera Module 3 (Wide NoIR)
                 │  프레임 640×360 @30fps
                 ▼
      vision/    MoveNet 추론 → 대상 1인 선정 → 좌표 안정화
                 │  머리 / 상체 / 하체 중심 좌표
                 ▼
      control/   좌표 → 팬·틸트 목표각 (비례 피드백 + 데드존)
                 부위 순찰 시나리오, 풍속 중재
                 │  목표각              │  풍속 단계
                 ▼                      ▼
      hardware/  스테퍼 드라이버        릴레이 (TS0011)
                 팬 유성 / 틸트 웜기어   선풍기 풍속 탭 3단
                 │                      │
                 ▼                      ▼
            팬틸트 헤드 회전         바람 세기 전환


      Flutter 앱  ◄────── BLE ──────►  main.py
      전원 · 모드 · 부위별 풍량          BlueZ GATT, Pi = Peripheral
      상태 수신 (STATUS notify)
```

### 동작 모드

| 모드 | 동작 |
|---|---|
| 기본-고정 / 기본-회전 | 추적 없이 기존 선풍기처럼 동작 |
| **타겟** | 사용자의 가슴(어깨 중점)을 화면 중앙에 붙잡는 닫힌 루프 추적 |
| **타겟-부위** | 머리 → 상체 → 하체를 순찰하며 앱에서 부위별로 설정한 풍속을 적용.<br>사용자가 이동하면 자동으로 추적으로 폴백했다가, 잠잠해지면 순찰 재개 |

---

## 개발 환경

### 하드웨어

| 구분 | 사용 부품 |
|---|---|
| 보드 | Raspberry Pi 5 |
| 카메라 | Pi Camera Module 3 **Wide NoIR** (IMX708, 화각 102° × 67°) |
| 모터 | 스테퍼 2축 (1.8°/step, 200 step/rev) — 팬: 유성기어 1:100, 틸트: 웜기어 1:92.6 |
| 드라이버 | TMC2209 ×2 + 아두이노 CNC v3 쉴드 (마이크로스텝 1/16) |
| 릴레이 | TS0011 4채널 중 3채널 — 기존 선풍기의 220V 풍속 탭을 단속 |
| 전원 | 24V 5A SMPS (모터부) |
| 그 외 | 기존 선풍기 본체, 팬틸트 기구부 |

> 틸트는 웜기어라 전원이 꺼져도 헤드가 자중으로 처지지 않습니다(백드라이브 방지).
> 팬은 회전량이 커서 유성기어를 씁니다.

### 소프트웨어

| 구분 | 사용 기술 |
|---|---|
| OS | Raspberry Pi OS Lite 64-bit (Debian trixie) |
| 언어 | Python 3.11.15 |
| 추론 | TFLite Runtime + MoveNet MultiPose Lightning (17 COCO keypoints, 최대 6인) |
| 영상 | OpenCV 4.x, rpicam-apps (`rpicam-vid`) |
| BLE | BlueZ 5.x GATT 서버 — `bluez_peripheral`(pre-release) / `dbus_fast` |
| GPIO | `lgpio` |
| 앱 | Flutter (Dart), Android |

---

## 빠른 시작

### 설치 (Raspberry Pi 5)

```bash
# 1. 시스템 패키지
sudo apt install -y bluez
# 카메라는 기본적으로 rpicam-vid(rpicam-apps, Pi OS 기본 포함)로 캡처한다.
# Picamera2 를 쓰려면 아래를 깔고 --no-rpicam 을 준다 (venv 는
# --system-site-packages 로 만들어야 libcamera 바인딩이 보인다).
# sudo apt install -y python3-picamera2

# 2. 파이썬 의존성
pip install -r requirements.txt
pip install --pre bluez_peripheral             # ⚠ --pre 필수 (아래 주의)

# 3. 모델 파일 — 레포에 포함되어 있지 않습니다
#    Kaggle Models에서 multipose-lightning-tflite-float16 을 받아
#    multipose_lightning.tflite 이름으로 레포 루트에 둘 것
#    https://www.kaggle.com/models/google/movenet/tfLite/multipose-lightning-tflite-float16/1

# 4. lgpio 패치 — Pi에서 1회만 실행 (빼먹으면 모터가 도중에 영구 정지합니다)
bash tools/patch_lgpio.sh

# 5. 영점 잡기 — 손으로 헤드를 정면 중앙에 맞춘 뒤 한 번 실행
#    (스테퍼는 오픈루프라 "지금 이 자리가 0°"를 한 번 선언해줘야 한다)
python3 tools/set_origin.py
```

> **`--pre` 주의**: `bluez_peripheral`의 PyPI 기본 stable은 0.1.7(2022)로 최신 BlueZ와
> 동작하지 않습니다. 반드시 pre-release를 설치하세요.

모든 명령은 **레포 루트에서** 실행합니다.

### 실행

```bash
# Raspberry Pi 5 — 레포 루트에서
python3 main.py --axis pantilt --no-window

# 로컬 창으로 화면을 보며 확인
python3 main.py --axis pantilt

# 개발 PC (모터·릴레이 없이 로직만)
python3 main.py --axis pan --dry-run --opencv
```

실행하면 앱에서 `ESW-FAN`으로 검색됩니다. 연결한 뒤 모드·전원·풍량을 보내면 동작합니다.

카메라와 인식만 따로 확인하려면 모터·BLE 없이 돌릴 수 있습니다.

```bash
python3 app/camera.py --web --no-window     # http://<호스트>:8090/
```

---

## 저장소 구조

```
main.py                   실행 파일 — BLE 서비스, 전체 조립
config.py                 튜닝값 전부 + BLE 프로토콜 상수

vision/                   영상 분석 — 사람을 찾고 어디를 겨눌지 좌표로 낸다
 ├── pose_estimate.py     MoveNet 추론 → 키포인트·bbox·부위 3지점
 ├── target_select.py     여러 명 중 대상 1인 선정
 └── region_filter.py     부위 좌표 EMA + 미검출 판정

control/                  제어 로직 — 좌표를 각도로 바꾸고 순찰·풍속을 정한다
 ├── control_signal.py    좌표 → 팬·틸트 각도, 소프트 리밋, 데드존
 ├── region_patrol.py     부위 순찰 상태기계, 풍속 중재, 폴백 판정
 └── recognition_report.py  인식 notify 를 언제 보낼지 판정

hardware/                 하드웨어 구동 — 모터와 릴레이를 실제로 움직인다
 ├── stepper.py           팬틸트 스테퍼 (논블로킹, 가감속, 위치 복원)
 ├── relay.py             풍속 릴레이 (break-before-make)
 └── position_store.py    위치 장부 파일 저장·복원

app/                      실행 계층 — 카메라·추적 루프와 모드별 동작
 ├── runners.py           모드별 러너 4종 + 모드 감독
 ├── tracking.py          팬·틸트 닫힌 루프 본문, 조준점, 오버레이
 └── camera.py            카메라 백엔드 3종, 시각화, 웹스트림 (단독 실행 가능)

tools/                    설치·실측용 단독 실행 도구
 ├── set_origin.py        영점 설정 (설치 5번)
 ├── patch_lgpio.sh       lgpio 패치 (설치 4번)
 ├── drive_motor.py       모터 단독 구동 — 각도·타이밍 실측
 ├── measure_jitter.py    펄스 스레드 지터 측정
 └── enable_hold.py       EN 유지 — 백래시 손측정

docs/                     프로토콜 · 제어 원리 · 문제 해결 기록
```

실행할 수 있는 파일은 `main.py`(전체)와 `app/camera.py`(카메라·인식만) 둘이고,
각 파일의 설계 근거는 상단 docstring에 있습니다.

---

## 성능

<!-- TODO: "실측" 열의 (미측정) 항목을 채워 주세요. 목표만 적힌 표는 대회에서 약합니다. -->

| 항목 | 목표 | 실측 |
|---|---|---|
| 추론 FPS | 20 fps | 약 20 fps (160×160, 캡처 20fps 상한 조건) — 캡처 30fps로 올린 뒤 **재측정 필요** |
| 전체 시스템 응답 시간 | < 0.5 s | (미측정) |
| 객체 인식 mAP | ≥ 66% | (미측정 — 160×160 입력에서 재측정 필요) |
| 부위 전환 정확도 | ≥ 90% | (미측정) |
| BLE 제어 응답 | < 0.2 s | (미측정) |
| 동작 소음 | < 35 dB | (미측정) — 펄스 지터 대책으로 달그락 소음은 제거 |
| 모터 위치 정확도 | < 2° | (미측정) — 펄스 타이밍 드리프트는 3~7% → **1% 미만** |

FPS는 `app/camera.py`가, 펄스 타이밍은 `--timing` 옵션이 실행 중에 찍습니다.

---

## 문서

| 문서 | 내용 |
|---|---|
| [`docs/ble_protocol.md`](docs/ble_protocol.md) | BLE UUID·바이트 형식·상태 동기화 계약 (RPi 구현 기준) |
| [`docs/tracking_feedback.md`](docs/tracking_feedback.md) | 추적 제어 원리 — 왜 절대 조준이 아니라 피드백인지, 거리를 몰라도 되는 이유, 데드존을 각도에 거는 이유 |
| [`docs/lgpio_patch.md`](docs/lgpio_patch.md) | 모터가 시끄럽거나 멈출 때 — 세 가지 문제(펄스 지터 / lgpio EINVAL 스핀 / VREF 과다)의 지문과 원인, 배제한 가설 |
| [`docs/angle_calibration.md`](docs/angle_calibration.md) | 각도 오차의 원인(기어비·백래시·탈조·원점)을 가르는 실험 순서와 보정식 |
| [`docs/pulse_jitter_data.md`](docs/pulse_jitter_data.md) | 펄스 스레드 지터 실측표 — RT 승격 대책의 근거 |

---

## 팀원

| 이름 | 역할 |
|---|---|
| 고대호 | 전체 시스템 통합, BLE 통신, 스마트폰 앱 개발 |
| 김윤우 | 전원부 배선, 릴레이 풍속 제어 회로 구성 |
| 박신형 | 스텝모터 제어 코드 개발, 포즈 추정 모델 튜닝 |
| 임동건 | 구동부 응력 해석, 포즈 추정 모델 선정 실험 |
| 조형우 | 3D 모델링, 구동부 설계, 모터 토크·하중 산정 |
