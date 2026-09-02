# -*- coding: utf-8 -*-
"""
app/ble_protocol.py - BLE 프로토콜 상수 · 추적 러너 · supervisor 기반 클래스
(프로토콜 명세는 docs/ble_protocol.md)

라이브러리 전용 — 진입점은 main.py 하나다.

  UUID 상수 / MODE_NAMES / WIND_TARGETS   앱과 맞춰야 하는 프로토콜 값
  _make_runner        타겟 모드(0x02/0x03) 추적 러너 팩토리 (카메라 세션 포함)
  _window_viewer      cv2 창 전용 스레드
  _TrackingSupervisor 전원·모드 → 러너 시작/정지 (app/ble_service.py 가 확장)

아래 카메라·cv2 설계 근거는 그대로 유효하다.

카메라는 모터/디텍터와 달리 프로세스 전체가 아니라 **추적 세션(스레드)마다
새로 열고 끝나면 반드시 해제**한다 — 특히 rpicam-vid(기본) 백엔드는
백엔드는 아무도 안 읽는 동안 파이프가 막혀 캡처 자체가 멎어버리는 게 실기로
확인됐다(기본 모드로 쉬다가 타겟 모드로 돌아오면 화면이 멈춘 채 그대로).

cv2 창은 추적 스레드가 직접 그리지 않는다 — HighGUI(GTK)는 처음 창을 만든
스레드에 묶여서, 세션 스레드가 imshow를 부르면 두 번째 세션부터 얼어붙는다
(실기 확인). 대신 추적 루프는 web_state에 프레임만 밀어 넣고, 프로세스 시작 때
만든 _window_viewer 전용 스레드가 창을 전담한다. --web을 함께 켜면 같은
프레임을 브라우저(http://<호스트>:8090/)로도 볼 수 있다.

"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

# 레포 루트를 path에 추가 (config / vision / control / hardware / app 해결용)
sys.path.insert(0, str(Path(__file__).parent.parent))

from config import CFG

_TRK = CFG["tracking"]   # 추적 튜닝 기본값 (CLI로 덮어쓸 수 있음)
from app.tracking import run_tracking
from app.camera import _open_camera, _read_frame, _release_camera

# ── 프로토콜 (docs/ble_protocol.md 와 일치해야 함) ──
UUID_BASE = "14d7{:04x}-7197-49e5-a017-0b2f308120f0"
SERVICE_UUID = UUID_BASE.format(0x0001)
POWER_UUID = UUID_BASE.format(0x0002)
MODE_UUID = UUID_BASE.format(0x0003)
WIND_UUID = UUID_BASE.format(0x0004)
STATUS_UUID = UUID_BASE.format(0x0005)

LOCAL_NAME = "ESW-FAN"

MODE_NAMES = {0x00: "기본-고정", 0x01: "기본-회전", 0x02: "타겟", 0x03: "타겟-부위"}
WIND_TARGETS = {0x00: "공용", 0x01: "머리", 0x02: "상체", 0x03: "하체"}
_TARGET_MODES = (0x02, 0x03)  # v1은 타겟/타겟-부위를 동일하게 취급


def _hex(value):
    return " ".join(f"0x{b:02X}" for b in value)


def _make_runner(axis, detector, tracker, mc, args, web_state):
    """axis에 맞는 app/tracking.py 루프를 stop_event 하나만 받는 콜러블로 감싼다.

    카메라는 세션(스레드)마다 새로 열고 끝나면 반드시 해제한다 — 특히
    rpicam-vid(기본) 백엔드는 아무도 읽지 않는 동안 파이프
    버퍼가 차서 캡처 자체가 멎어버리는 게 실기로 확인됨. "기본 모드"로 쉬는
    동안 카메라를 계속 열어두면 재진입 시 그 멎은 상태(화면 정지) 그대로
    남는다 — 매번 새로 열면 이 문제를 피한다.
    """
    def _open_cam():
        """카메라를 열고 **첫 프레임까지 실제로 오는지** 확인한 뒤 반환한다.

        재오픈이 실패하는 두 가지 조용한 경로를 모두 잡는다:
        - `_open_camera`는 전부 실패하면 `sys.exit(1)`을 부르는데, 백그라운드
          스레드 안의 SystemExit는 그 스레드만 조용히 죽인다 → 잡아서 재시도.
        - rpicam-vid는 Popen이 성공해도 카메라를 못 잡으면(이전 세션 자원 해제
          지연 등) 곧바로 죽는다(stderr는 버려져 에러도 안 보임). 이러면
          추적 루프가 frame=None만 무한 반복하며 창도 모터도 반응이 없다 —
          그래서 오픈 성공 판정을 "첫 프레임 수신"으로 한다.
        """
        for attempt in range(1, 4):
            try:
                cam, backend = _open_camera(args.opencv, args.cam, use_rpicam=args.rpicam)
            except SystemExit:
                print(f"[E2E] 카메라 오픈 실패 ({attempt}/3) — 1s 후 재시도")
                time.sleep(1.0)
                continue
            deadline = time.time() + 4.0
            while time.time() < deadline:
                if _read_frame(cam, backend) is not None:
                    print(f"[E2E] 카메라 준비 완료 (backend={backend})")
                    return cam, backend
                time.sleep(0.1)
            print(f"[E2E] 카메라가 프레임을 못 줌 ({attempt}/3) — 닫고 재오픈")
            _release_camera(cam, backend)
            time.sleep(1.0)
        raise RuntimeError("[E2E] 카메라를 열 수 없습니다 (재시도 소진)")

    def _release_cam(cam, backend):
        # cv2 창 정리는 여기서 하지 않는다 — GUI 호출은 전용 뷰어 스레드가 전담
        # (세션 스레드에서 cv2 GUI를 부르면 다음 세션에서 얼어붙음, 실기 확인).
        _release_camera(cam, backend)
        # 다음 세션이 바로 재오픈해도 커널/libcamera가 자원을 다 놓을 시간을 준다.
        time.sleep(0.3)

    fov_cfg = CFG["fov"]
    fov_h, fov_v = fov_cfg["h"], fov_cfg["v"]
    sign_pan = -1.0 if args.invert_pan else 1.0
    sign_tilt = -1.0 if args.invert_tilt else 1.0
    # 부위 선택은 틸트에서만 의미가 있다 (팬은 어느 부위를 겨눠도 같은 각도).
    aim_key = args.region if (axis == "tilt" and args.region != "chest") else "upper"

    def _run(stop_event):
        cam, backend = _open_cam()
        try:
            run_tracking(cam, backend, detector, tracker, mc, args, stop_event,
                         axis=axis, fov_h=fov_h, fov_v=fov_v,
                         sign_pan=sign_pan, sign_tilt=sign_tilt,
                         aim_key=aim_key, web_state=web_state)
        finally:
            _release_cam(cam, backend)

    return _run


def _window_viewer(web_state, stop_event) -> None:
    """cv2 창 표시 전담 스레드 (프로세스에서 유일하게 cv2 GUI를 부르는 곳).

    cv2 HighGUI(GTK)는 처음 창을 만든 스레드에 묶여서, 추적 세션 스레드가
    직접 imshow를 부르면 두 번째 세션부터 창이 안 뜨고 루프까지 얼어붙는다
    (실기 확인). 그래서 추적 루프는 _WebStreamState에 프레임(JPEG)만 밀어
    넣고, 프로세스 시작 때 만든 이 스레드 하나가 창의 생성·표시·파괴를
    전부 전담한다 — 세션이 몇 번 재시작되든 GUI 스레드는 항상 같다.
    """
    title = "ESW E2E"
    last = None
    while not stop_event.is_set():
        jpg = web_state.latest()
        if jpg and jpg is not last:
            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow(title, frame)
            last = jpg
        cv2.waitKey(30)  # GUI 이벤트 펌프 겸 표시 주기 (~33fps 상한)
    cv2.destroyAllWindows()


class _TrackingSupervisor:
    """BLE 전원/모드 write에 맞춰 추적 루프를 백그라운드 스레드로 시작/정지한다.

    전원 OFF든 기본 모드 복귀든 즉시 정지 신호(stop_event.set())를 보내고,
    새로 시작할 때만 이전 스레드가 완전히 끝났는지 join으로 확인한다
    (정지 자체는 non-blocking — BLE 콜백을 오래 막지 않음).
    """

    def __init__(self, run_fn, web_state=None) -> None:
        self._run_fn = run_fn
        self._web_state = web_state
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._power_on = False
        self._mode = 0x00

    def _target_active(self) -> bool:
        return self._power_on and self._mode in _TARGET_MODES

    def _thread_main(self, stop_event) -> None:
        """예외로 죽으면 원인이 조용히 사라지지 않게 traceback을 남긴다."""
        try:
            self._run_fn(stop_event)
        except Exception:
            import traceback
            print("\n[E2E] 추적 스레드가 예외로 종료됨:")
            traceback.print_exc()
        print("[E2E] 추적 스레드 종료")
        if self._web_state:
            # 창/웹 화면이 마지막 프레임에 얼어 보이지 않게 대기 안내로 교체.
            self._web_state.update_stall(["tracking stopped",
                                          "switch to target mode to resume"])

    def _apply(self) -> None:
        if self._target_active():
            # 직전 스레드가 (정지 요청을 받았지만 루프 맨 위 체크 지점까지 아직
            # 못 돌아와) 살아있을 수 있다 — "이미 실행 중"으로 오인해 조용히
            # 리턴하면 이후 시작 요청이 영영 씹히므로, 반드시 정지시키고 join한
            # 뒤 새로 시작한다 (기존엔 여기서 is_alive()면 그냥 return 해버려서
            # 레이스가 나면 추적이 재개 안 되는 버그가 있었다).
            if self._thread and self._thread.is_alive():
                print("[E2E] 이전 추적 스레드 정리 중...")
                self._stop_event.set()
                self._thread.join(timeout=15)
                if self._thread.is_alive():
                    print("[E2E] 이전 스레드가 아직 안 끝남 — 이번 시작은 건너뜀 "
                          "(모드를 다시 토글해 보세요)")
                    return
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._thread_main,
                                            args=(self._stop_event,), daemon=True)
            self._thread.start()
            print("[E2E] 추적 시작")
        else:
            self._stop_event.set()
            print("[E2E] 추적 정지 요청 (백그라운드에서 곧 멈춤)")

    def set_power(self, on: bool) -> None:
        self._power_on = on
        self._apply()

    def set_mode(self, mode: int) -> None:
        self._mode = mode
        self._apply()

    def stop_and_join(self, timeout: float = 10) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=timeout)
