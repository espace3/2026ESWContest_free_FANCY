# ESW 개인 맞춤형 스마트 선풍기 제어 애플리케이션 (ESW_BLE_app)

사용자의 위치와 신체 부위를 인식해 맞춤형 바람을 제공하는 **스마트 선풍기**의
**스마트폰 제어 애플리케이션** 저장소입니다.
앱은 **Flutter/Dart**로 작성하며, **BLE(Bluetooth Low Energy)** 로
선풍기 제어 보드(**Raspberry Pi 5**)와 통신합니다.

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
- RPi 측은 연결 해제 시 **re-advertising**으로 재연결 대기, 비정상 종료 복구 루틴 포함 예정
- 플랫폼별 BLE 스택: Android `BluetoothGatt`, iOS `CoreBluetooth`

## 3. 애플리케이션 시나리오

1. **전원 제어** — 앱 접속 시 터치로 선풍기 ON/OFF.
2. **모드 선택** — 선풍기가 ON이면 **기본 모드** / **타겟 모드** 제공.
3. **기본 모드** — 회전 여부(고정 / 좌우 회전)와 풍량 선택.
4. **타겟 모드** — 객체 인식 성공 여부 표시 + 풍량 선택.
   선풍기는 인식된 한 명의 사용자를 추적하며 송풍.
5. **부위 모드** — 타겟 모드에서 ON 하면 **머리·상체·하체** 부위별 바람 세기 개별 설정.
   사용자가 이동하면 타겟 모드로 복귀, 정지하면 다시 부위별 제어.

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
README.md               ← 프로젝트 전체 안내
lib/                    ← Dart 소스
  main.dart               앱 진입점
  app.dart                앱 루트 위젯 · 라우팅
  pages/                  화면 (홈 · BLE 스캔 · 기본 모드 · 타겟 모드)
  services/               상태 관리, BLE 연결/프로토콜
  widgets/                공용 위젯
android/ ios/           ← 플랫폼별 러너 프로젝트
test/                   ← 위젯 테스트
release/                ← 배포용 APK
```

## 6. 실행 방법

```bash
flutter pub get
flutter run   # 연결된 Android/iOS 기기·에뮬레이터 대상
```
