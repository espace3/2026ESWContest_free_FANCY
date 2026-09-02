# 2026 ESW Contest 자유공모 — Team FANCY

카메라로 사용자를 실시간 인식·추적해 팬틸트 헤드로 바람 방향을 자동 조준하고,
**머리 / 상체 / 하체** 3부위를 구분해 부위별 맞춤 풍속을 제공하는 임베디드 시스템 프로젝트입니다.

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
| **타겟-부위** | 머리 → 상체 → 하체를 순찰하며 부위별로 설정된 풍속을 적용.<br>사용자가 이동하면 자동으로 추적으로 폴백했다가, 잠잠해지면 순찰 재개 |

풍속은 카메라가 아니라 **앱이 정하는 값**입니다 — 부위별 세기를 사용자가 BLE로
직접 지정하고, RPi는 어디를 겨눌지만 정합니다 (계약: [`docs/ble_protocol.md`](docs/ble_protocol.md)).

---

## 개발 환경

- **보드**: Raspberry Pi 5 + Pi Camera Module 3
- **OS / 언어**: Raspberry Pi OS Lite 64-bit, Python 3.11.15
- **추론**: TFLite Runtime + MoveNet MultiPose Lightning
  (17 COCO keypoints, 최대 6인 동시 검출). 기본 입력 256×256이지만 FPS 확보를 위해
  **160×160**으로 낮춰 운용합니다 (`vision/pose_estimate.py`의 `INPUT_SIZE`)
- **영상**: OpenCV 4.x
- **BLE**: BlueZ 5.x GATT 서버 (`bluez_peripheral` / dbus_fast), Pi = Peripheral
- **GPIO**: lgpio — Pi 5는 pigpio(DMA 하드웨어 타이밍)를 쓸 수 없습니다
- **앱**: Flutter (Dart), Android 우선

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

# 4. lgpio 패치 — Pi에서 1회만 실행 (시스템 파일은 수정하지 않음)
bash tools/patch_lgpio.sh

# 5. 영점 잡기 — 손으로 헤드를 정면 중앙에 맞춘 뒤 한 번 실행
python3 tools/set_origin.py
```

> **lgpio 패치가 왜 필요한가**: liblgpio의 송출 스레드가 특정 조건에서 `clock_nanosleep`
> EINVAL을 무한 재시도하며 CPU를 100% 점유하고, **전 핀의 펄스 송출이 영구 정지**합니다
> (실기 확인). 패치본을 `/usr/local/lib`에 설치해 로드 우선순위로 덮는 방식이라 시스템
> 파일은 그대로이고, 사본을 지우면 원복됩니다 — [`docs/lgpio_patch.md`](docs/lgpio_patch.md).

> **`--pre` 주의**: `bluez_peripheral`의 PyPI 기본 stable은 0.1.7(2022)로 최신 BlueZ와
> 동작하지 않습니다. 반드시 pre-release를 설치하세요.

> **영점이 왜 필요한가**: 스테퍼는 오픈루프라 자기 위치를 스스로 알 수 없습니다.
> 보낸 펄스 수를 파일(장부)에 기록해두고 재시작 시 그만큼 되돌아오는 방식이라,
> 최초 한 번은 "지금 이 자리가 0°"라고 선언해줘야 합니다
> (`hardware/position_store.py` 상단 참고).

모든 스크립트는 **레포 루트에서** 실행하세요 — `sys.path`를 스크립트가 직접 잡습니다.

### 실행

전체 기능(BLE + 추적 + 부위별 풍속)이 통합된 **진입점은 `main.py`** 하나입니다.

```bash
# Raspberry Pi 5 — 레포 루트에서
python3 main.py --axis pantilt --no-window

# 로컬 창으로 화면을 보며 확인
python3 main.py --axis pantilt

# 개발 PC (모터·릴레이 없이 로직만)
python3 main.py --axis pan --dry-run --opencv
```

실행하면 `ESW-FAN`으로 광고를 시작하고, 앱이 연결해 모드/전원/풍량을 보내면 동작합니다.

카메라와 인식만 따로 확인하려면 모터·BLE 없이 돌릴 수 있습니다.

```bash
python3 app/camera.py --web --no-window     # http://<호스트>:8090/
```

---

## 저장소 구조

```
main.py                    진입점. GATT 서비스(EswFanService) · BLE 부팅 · 전체 조립
config.py                  튜닝값 전부(카메라·화각·핀·릴레이·스테퍼·리밋·추적)
                           + BLE 프로토콜 상수(UUID·모드·풍량 — 앱과 공유하는 계약)

vision/                    순수 계산 — 프레임 in, 좌표 out. GPIO·BlueZ import 금지
 ├── pose_estimate.py      MoveNet MultiPose Lightning 추론. BGR 프레임 → 최대 6인의
 │                         17 키포인트 + bbox + 부위(머리/상체/하체) 중심
 ├── target_select.py      다중 인원 중 추적 대상 1인 선정 — 직전 대상이 근처에 있으면
 │                         유지, 없으면 화면 중앙 최근접 (bbox 중심 기준)
 └── region_filter.py      선정된 1인의 부위 좌표 안정화 — EMA 스무딩 + 부위별 miss
                           카운트. 재획득 시엔 섞지 않고 즉시 점프(허공을 훑지 않도록)

control/                   순수 계산 — 좌표 in, 각도·판정 out. GPIO·BlueZ·카메라 없음
 ├── control_signal.py     좌표 → 팬/틸트 각도, 소프트 리밋 clamp,
 │                         데드존(포즈 잡음이 모터로 새는 것 차단)
 ├── region_patrol.py      부위 모드 전체. RegionPatrolScenario(한 프레임 매핑 →
 │                         시간 슬롯 순찰 → 재조준·탐색 상태기계),
 │                         region_wind_level(풍속 중재), MotionGate(폴백 전환 판정)
 └── recognition_report.py 객체 인식 notify를 언제 보낼지 판정 — 시간 창 안의 검출
                           비율로 경계 상황의 깜빡임을 흡수

hardware/                  하드웨어 구동과 그 상태 관리 — 계산하지 않음
 ├── stepper.py            팬틸트 스테퍼 구동. 논블로킹(축별 워커 스레드, 최신 목표
 │                         선점), 사다리꼴 가감속, 위치 저장·복원, lgpio 펄스 스레드
 │                         RT 승격 대책. ⚠ tools/patch_lgpio.sh 적용이 전제
 ├── relay.py              선풍기 풍속 릴레이(TS0011) 구동. 논블로킹,
 │                         break-before-make(전부 오픈 → guard → 하나만 닫기)
 └── position_store.py     장부 위치(정수 스텝)를 파일에 저장·복원 — 오픈루프
                           스테퍼가 절대 위치를 못 재는 것에 대한 방편

app/                       입출력·조립 계층 — 위 모듈들을 실제 장치에 연결
 ├── runners.py            모드별 러너 팩토리 4종(추적·부위·회전·복귀)과
 │                         ModeSupervisor(전원·모드·풍속 → 러너 선택, 스레드 교대),
 │                         BLE 연결 끊김 감지
 ├── tracking.py           팬/틸트 닫힌 루프 본문(run_tracking), 조준점 계산
 │                         (chest_point), 오버레이, 모터 핸들 열기, --dry-run 스텁
 └── camera.py             카메라 백엔드 3종(picamera2 / rpicam-vid / OpenCV),
                           재시도 오픈, 포즈 시각화, MJPEG 웹스트림, cv2 창 스레드
                           ※ 단독 실행 가능 — 모터·BLE 없이 인식만 확인

tools/                     단독 실행 도구 — 운용 루프 밖에서 한 번씩 쓴다
 ├── set_origin.py         최초 1회 영점 설정 (설치 5번)
 ├── patch_lgpio.sh        liblgpio EINVAL 무한 스핀 패치 (설치 4번)
 ├── drive_motor.py        모터 단독 구동 — 각도 캘리브레이션 · 펄스 타이밍 실측
 ├── measure_jitter.py     펄스 스레드 웨이크업 지터 측정 (모터·배선 불필요)
 └── enable_hold.py        EN을 켠 채 대기 — 기어 유격(백래시) 손측정용

docs/                      프로토콜 · 제어 원리 · 문제 해결 기록 (아래 표)
```

---

## 왜 이 파일들만 있는가

**운용에 필요한 것**과 **`docs/`의 실험 절차를 실행하는 데 필요한 것**만 두었습니다.
개발 과정에서 기능 단위로 쓴 단독 검증 스크립트(축별 추적, 릴레이 극성 확인, BLE
연동 등)는 그 기능이 `main.py`로 흡수되어 제외했습니다 — `main` 브랜치에 있습니다.

`app/`의 모듈들은 그 검증 스크립트로 시작해, 기능이 확정되면서 상위 단계가 import해
쓰는 라이브러리가 된 것들입니다. 그래서 한동안 파일이 개발 순서대로 나뉘어 있었고 같은
역할의 코드가 여러 파일에 흩어져 있었는데, 제출 전에 **책임 기준으로 다시 묶고** 단계별
진입점은 제거했습니다 — 안 도는 코드가 남아 있으면 읽는 쪽이 매번 "이게 실제로
실행되나"를 확인해야 하기 때문입니다. 그 단계별 코드는 git 이력에 있습니다.

**실행 가능한 진입점은 둘입니다.**

| | 무엇 |
|---|---|
| `main.py` | 전체 시스템 (BLE + 추적 + 부위별 풍속) |
| `app/camera.py` | 카메라·인식만 — 모터·BLE 없이 확인 |

`tools/`의 다섯 개는 설치 절차와 `docs/`의 실험 절차에서 한 번씩 부릅니다.

| 실측 도구 | 무엇을 재나 | 관련 문서 |
|---|---|---|
| `drive_motor.py` | 모터 단독 구동 — 각도 누적 오차, 백래시, 짧은 이동 한계, 펄스 타이밍(`--timing`) | [`angle_calibration.md`](docs/angle_calibration.md) · [`lgpio_patch.md`](docs/lgpio_patch.md) |
| `measure_jitter.py` | 펄스 스레드 웨이크업 지터 (모터·배선 불필요) | [`lgpio_patch.md`](docs/lgpio_patch.md) · [`pulse_jitter_data.md`](docs/pulse_jitter_data.md) |
| `enable_hold.py` | EN을 켠 채 대기 — 기어 유격(백래시) 손측정용 | [`angle_calibration.md`](docs/angle_calibration.md) |

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

**부수 실측값**

| 항목 | 값 | 근거 |
|---|---|---|
| 팬 순항 속도 | 7.87 °/s | `f_max` 7000, 마이크로스텝 1/16, 기어비 100 |
| 틸트 순항 속도 | 7.29 °/s | `f_max` 6000, 기어비 92.6 (무부하 벤치 실측) |
| 틸트 가동 범위 | −25° ~ **+15°** | +15°는 기구 파손 한계 (실측 확정, 초과 금지) |
| 드라이버 VREF | 양축 0.7 V | 1.1/1.3V에서 두 축 동시 구동 시 팬 정지 → 0.7V로 해결 |

목표에 영향을 주는 파이프라인 단계(추론·모터 이동·BLE 왕복)를 추가할 때는
`app/camera.py`의 FPS 로깅 패턴(`fps_hist`, 초당 콘솔 로그)처럼 **측정 코드를
함께 넣습니다.** 주장이 아니라 코드로 잴 수 있어야 합니다.

---

## 아키텍처 원칙

### 계산 로직과 하드웨어 호출을 절대 같은 함수에 섞지 않는다

이 레포의 타협하지 않는 규칙입니다.

- **계산 전용 모듈** — GPIO/BlueZ 등 하드웨어 라이브러리를 import하지 않습니다.
  입력은 프레임/키포인트, 출력은 각도·신호값 같은 순수 데이터입니다.
  `vision/` 전체와 `control/` 전체가 여기 해당합니다.
- **하드웨어 호출 전용 모듈** — 계산 모듈이 만든 값을 받아 GPIO/UART/BLE로 내보내기만
  하고, 계산을 하지 않습니다. `hardware/` 전체가 여기 해당합니다.
- 예: `compute_pan_angle(cx_norm, fov_h_deg) -> float`은 순수 함수이고,
  실제 GPIO 호출은 `stepper.move_to(angle)`이 전담합니다.

이렇게 분리하면 **모터 드라이버를 교체하거나 계산 버그를 찾을 때 서로 영향 없이**
수정·검증할 수 있습니다. 실제로 이 구조 덕분에 이미 검증을 마친 추적 루프
(`app/tracking.py`)를 한 줄도 고치지 않고 그 위에 BLE 통합을 얹을 수 있었습니다.

### 동시성

카메라 캡처 · 추론 · 모터 제어 · BLE가 모두 동시에 동작합니다. 스레딩을 나중에 얹지 않고
처음부터 설계했습니다 — 모터/릴레이는 논블로킹 워커 스레드를 갖고, BLE는 asyncio 루프에서
돌며, 러너 스레드는 `loop.call_soon_threadsafe()`로만 BLE에 보고합니다
(`dbus_fast`가 스레드 안전하지 않기 때문).

---

## 문서

| 문서 | 내용 |
|---|---|
| [`docs/ble_protocol.md`](docs/ble_protocol.md) | BLE UUID·바이트 형식·상태 동기화 계약 (RPi 구현 기준) |
| [`docs/tracking_feedback.md`](docs/tracking_feedback.md) | 추적 제어 원리 — 왜 절대 조준이 아니라 피드백인지, 거리를 몰라도 되는 이유, 데드존을 각도에 거는 이유 |
| [`docs/lgpio_patch.md`](docs/lgpio_patch.md) | 부하 시 모터 달그락 소음 — 배제한 가설 8종, 원인(펄스 스레드 선점), 채택한 대책과 확인법 |
| [`docs/angle_calibration.md`](docs/angle_calibration.md) | 각도 오차의 원인(기어비·백래시·탈조·원점)을 가르는 실험 순서와 보정식 |
| [`docs/pulse_jitter_data.md`](docs/pulse_jitter_data.md) | 펄스 스레드 지터 실측표 — RT 승격 대책의 근거 |
| [`docs/hardware_todo.md`](docs/hardware_todo.md) | 하드웨어 잔여 결정·검증 항목 **과 해결 기록** — 모터가 멈추던 두 문제(lgpio EINVAL 스핀 / VREF 과다)의 진단 경위와 배제한 가설이 여기 있습니다 |

**구현 전에 알고리즘을 문서로 정리하고, 코드가 바뀌면 문서도 함께 갱신합니다.**
