# ESW 개인 맞춤형 스마트 선풍기 제어 애플리케이션

사용자의 위치와 신체 부위를 인식해 맞춤형 바람을 제공하는 **스마트 선풍기**의
**스마트폰 제어 애플리케이션** 저장소입니다.
앱은 **Flutter/Dart**로 작성하며, **BLE(Bluetooth Low Energy)** 로
선풍기 제어 보드(**Raspberry Pi 5**)와 통신합니다.

> RPi 제어 코드는 [`main`](https://github.com/espace3/ESW/tree/main) 브랜치를 참고하세요.

---

## 1. 프로젝트 개요

본 프로젝트(2026 ESW)는 기존 선풍기를 개조하여 다음 기능을 갖춘
**개인 맞춤형 스마트 선풍기**를 개발한다.

- **사용자 추적**: 헤드에 장착된 카메라 + 자세 추정 모델(**MoveNet Multipose Lightning**)
  로 사용자의 머리·상체·하체 키포인트를 실시간 인식
- **팬틸트 2축 구동**: 좌우(pan)/상하(tilt) 모터 2개로 헤드 방향 제어
- **풍량 제어**: 기존 선풍기 보드를 릴레이 모듈로 제어
- **앱 제어**: 스마트폰 앱과 RPi5를 BLE로 연결하여 전원/모드/풍량을 원격 제어

## 2. 시스템 구성 (앱 관점)

```
[Flutter 앱]  ←── BLE (GATT) ──→  [Raspberry Pi 5]  ──→  릴레이/모터/카메라
 Central,                          Peripheral,
 GATT Client                       GATT Server
```

- **GAP 레이어**: RPi5 = Peripheral(Advertising 송출), 앱 = Central(스캔·연결 개시)
- **GATT 레이어**: RPi5 = Server(Characteristic 보유), 앱 = Client(읽기/쓰기 요청)
- 제어 명령(전원, 모드 전환, 풍량 등)은 **저용량·간헐적 송수신**이므로 BLE가 적합
- 연결이 끊기면 앱·RPi 양쪽이 **전원 OFF로 수렴**한다 (풍속 정지 + 헤드 0°,0° 파킹).
  예기치 못한 끊김이면 앱이 30초간 자동 재연결을 시도하고, 성공 시 직전 상태를 복원한다.
  RPi는 **re-advertising**으로 재연결을 대기한다.
- 플랫폼별 BLE 스택: Android `BluetoothGatt`, iOS `CoreBluetooth`

## 3. 애플리케이션 시나리오

1. **전원 제어** - 앱 접속 시 터치로 선풍기 ON/OFF.
2. **모드 선택** - 선풍기가 ON이면 **기본 모드** / **타겟 모드** 제공.
3. **기본 모드** - 회전 여부(고정 / 좌우 회전)와 풍량 선택.
4. **타겟 모드** - 객체 인식 성공 여부를 표시하고 풍량을 선택한다.
   선풍기는 인식된 한 명의 사용자의 **상체를 화면 중앙에 붙잡는 닫힌 루프 추적**으로 송풍.
   - **부위 인식 ON** - **머리·상체·하체** 부위별 바람 세기를 개별 설정한다.
     선풍기는 머리 → 상체 → 하체를 순찰하며 부위별 풍속을 적용하고,
     사용자가 이동하면 추적으로 폴백했다가 잠잠해지면 순찰을 재개한다.

## 4. 개발 로드맵

| 단계 | 내용 | 상태 |
|---|---|---|
| 1 | **Windows 데스크톱 타깃**으로 앱 우선 개발 (UI 골격 + 스와이프 네비게이션) | 완료 |
| 2 | Windows 앱 ↔ RPi **BLE 통신 검증** | 완료 |
| 3 | 스와이프형 모드 전환 등 UX 구체화 | 1차 적용, 개선 중 |
| 4a | **Android** 빌드 + BLE 권한 설정 (`com.esw.fan`) | 완료 |
| 4b | **iOS** 빌드 | 완료 |

## 5. 저장소 구조

```
main.dart                 진입점 — 상태 초기화 후 앱 실행
app.dart                  앱 루트 위젯 · 테마 · 스와이프 설정

pages/                    화면 — 표시·입력만, 상태는 services/가 소유
 ├── home_page.dart         홈 화면 — 전원 버튼, 기본 ↔ 타겟 모드 전환
 ├── ble_scan_page.dart     BLE 스캔·연결 디버그 화면
 ├── basic_mode_page.dart   기본 모드 — 고정/회전 + 바람 세기
 └── target_mode_page.dart  타겟 모드 — 인식 상태, 풍량, 부위별 세기

services/                앱 전역 상태 (싱글턴)
 ├── fan_state_service.dart            설정 주체 — 전원/모드/풍량 보관·영속화·전송·검증
 ├── ble/ble_protocol.dart             GATT 계약 — UUID·바이트 형식·스냅샷 파싱 (RPi와 공유)
 └── ble/ble_connection_service.dart   BLE 연결 — 재시도, write/에코백, 자동 재연결

widgets/                 공용 위젯
 └── wind_strength_selector.dart  바람 세기(정지·1~3단) 선택기

android/ · ios/           플랫폼별 러너 프로젝트 (앱 ID com.esw.fan)
test/                    위젯 테스트 — 전원·모드 전환, 재전송 계약
my_app/release/          배포용 APK (esw-fan-v1.0.apk)
```

## 6. 실행 방법

```bash
flutter pub get
flutter run   # 연결된 Android/iOS 기기·에뮬레이터 대상
```
