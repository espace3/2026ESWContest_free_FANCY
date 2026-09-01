"""
app/camera.py - 카메라 백엔드 · 포즈 시각화 · MJPEG 웹스트림
(개발 이력상 verify_movenet.py)

이 파일은 계산 로직을 포함하지 않습니다. 실제 추정/선정/추적 로직은
vision/pose_estimator.py, vision/target_selector.py, vision/pose_tracker.py에
있고, 여기서는 카메라를 열어 그 결과를 화면/웹에 보여주기만 합니다
(하드웨어 캡처 + 시각화). 같은 vision 모듈을 app/main.py(실제 구동 코드)에서도
그대로 가져다 씁니다.

설치:
    pip install tflite-runtime

모델: multipose_lightning.tflite (MoveNet MultiPose Lightning, 최대 6명 동시 검출)

실행 (레포 루트에서):
    python app/camera.py
    python app/camera.py --no-window
    python app/camera.py --web --no-window --web-port 8090
"""

from __future__ import annotations

import argparse
import select
import sys
import threading
import time
from http import server
from pathlib import Path
from socketserver import ThreadingMixIn

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))
from config import CFG
from vision.pose_estimator import MoveNetMultiPoseDetector, KP_NAMES, SKELETON
from vision.target_selector import select_target, person_center, DEFAULT_MATCH_RADIUS
from vision.pose_tracker import PoseTracker

# ── 색상 ─────────────────────────────────────────────────────────────────────
C_GREEN = (0, 200, 60)
C_YELLOW = (0, 210, 230)
C_CYAN = (220, 200, 0)
C_WHITE = (255, 255, 255)
C_GRAY = (160, 160, 160)
C_PANEL = (30, 30, 30)
C_RED = (0, 50, 220)
_PANEL_W = 340

REGION_COLOR = {"head": (0, 50, 220), "upper": (0, 200, 60), "lower": (220, 80, 0)}
REGION_LABEL = {"head": "HEAD", "upper": "UPPER", "lower": "LOWER"}


# ── NoIR 야간 촬영 근사 시뮬레이션 ─────────────────────────────────────────────
#
# 주의: 이건 어디까지나 근사치다. 실제 IR 파장대에서 재질별 반사율이 가시광과
# 다르게 나오는 부분(예: 검은 옷이 IR에선 밝게 나오는 등)은 컬러 사진 한 장에
# 담긴 정보가 아니라서 이미지 변환으로 재현할 수 없다 — 그 부분은 실제 NoIR
# 센서로 찍어봐야만 확인 가능하다. 여기서 흉내 내는 건 "촬영 조건이 나빠졌을
# 때"에 해당하는 부분만이다:
#   1) 색 정보 손실 (그레이스케일)
#   2) IR LED가 보통 카메라 렌즈 옆에 붙어 있어 생기는 점광원 vignette
#      (중앙부는 밝고 가장자리로 갈수록 급격히 어두워짐)
#   3) 저조도 센서 특유의 노이즈 (신호 대비 노이즈 비율 저하)
#   4) 노출시간이 길어지며 생기는 대비 저하
_NOIR_VIGNETTE_CACHE: dict[tuple[int, int], np.ndarray] = {}


def _noir_vignette_gain(h: int, w: int) -> np.ndarray:
    """중앙이 밝고 가장자리가 어두운 점광원 조명 gain map (0~1)을 캐싱해서 반환."""
    key = (h, w)
    if key not in _NOIR_VIGNETTE_CACHE:
        ys, xs = np.mgrid[0:h, 0:w]
        cy, cx = h / 2.0, w / 2.0
        r = np.sqrt(((xs - cx) / cx) ** 2 + ((ys - cy) / cy) ** 2)
        gain = np.clip(1.15 - 0.55 * (r ** 2), 0.25, 1.15)
        _NOIR_VIGNETTE_CACHE[key] = gain.astype(np.float32)
    return _NOIR_VIGNETTE_CACHE[key]


def simulate_noir(frame_bgr: np.ndarray) -> np.ndarray:
    """가시광 컬러 프레임에 NoIR 야간 촬영과 "촬영 조건" 측면에서 비슷한
    효과(색 손실 + 점광원 vignette + 저조도 노이즈 + 대비 저하)를 적용한다.

    IR 파장대 고유의 재질별 반사율 차이는 재현하지 못하므로, 이 함수를 통과한
    영상에서 검출이 잘 된다고 해서 실제 NoIR 촬영에서도 잘 된다고 단정할 수는
    없다 (필요조건이지 충분조건은 아님).
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)

    h, w = gray.shape
    gray *= _noir_vignette_gain(h, w)

    # 대비 저하 (128 쪽으로 당기기 — 노출은 늘렸지만 동적 범위는 좁아지는 느낌)
    gray = 128.0 + (gray - 128.0) * 0.7

    # 저조도 센서 노이즈
    noise = np.random.normal(0.0, 10.0, gray.shape).astype(np.float32)
    gray += noise

    gray = np.clip(gray, 0, 255).astype(np.uint8)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


# ── 시각화 ───────────────────────────────────────────────────────────────────
# 주의: cv2.putText(Hershey 폰트)는 ASCII 전용이라 한글을 넘기면 ???로 깨져 보인다.
# 화면에 그리는 문자열은 영문으로만 쓸 것 (콘솔 print는 한글 OK).

def draw_pose(frame: np.ndarray, people: list[dict], selected_idx: int | None) -> np.ndarray:
    """검출된 모든 사람의 키포인트+골격을 그리고, 선정된 대상자만 부위 중심까지 강조한다."""
    if not people:
        return frame

    vis = frame.copy()
    h, w = vis.shape[:2]
    conf_thr = 0.2  # 그리기 임계값은 조금 낮게

    for idx, person in enumerate(people):
        kps = person["keypoints"]
        is_target = (idx == selected_idx)
        line_color = C_GREEN if is_target else C_GRAY
        line_thick = 3 if is_target else 1
        pt_color = C_YELLOW if is_target else C_GRAY
        pt_radius = 4 if is_target else 3

        for a, b in SKELETON:
            if kps[a]["conf"] >= conf_thr and kps[b]["conf"] >= conf_thr:
                ax, ay = int(kps[a]["x"] * w), int(kps[a]["y"] * h)
                bx, by = int(kps[b]["x"] * w), int(kps[b]["y"] * h)
                cv2.line(vis, (ax, ay), (bx, by), line_color, line_thick)

        for kp in kps:
            if kp["conf"] >= conf_thr:
                cx, cy = int(kp["x"] * w), int(kp["y"] * h)
                cv2.circle(vis, (cx, cy), pt_radius, pt_color, -1)

        # 부위(머리/상체/하체) 중심 표시는 실제 추적 대상 1인에게만
        if is_target:
            for name, color in REGION_COLOR.items():
                r = person["regions"][name]
                if r["visible"]:
                    cx = int(r["cx"] * w)
                    cy = int(r["cy"] * h)
                    cv2.circle(vis, (cx, cy), 12, color, -1)
                    cv2.circle(vis, (cx, cy), 12, C_WHITE, 2)
                    cv2.putText(vis, REGION_LABEL[name], (cx + 14, cy + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    return vis


def build_panel(
    people: list[dict], selected_idx: int | None, fps: float, frame_ms: float
) -> np.ndarray:
    h_panel = CFG["camera"]["height"]
    panel = np.full((h_panel, _PANEL_W, 3), C_PANEL, dtype=np.uint8)

    def text(msg, row, color=C_WHITE, scale=0.55, bold=1):
        cv2.putText(panel, msg, (10, row), cv2.FONT_HERSHEY_SIMPLEX, scale, color, bold)

    def sep(row):
        cv2.line(panel, (5, row), (_PANEL_W - 5, row), C_GRAY, 1)

    row = 28
    text("== MOVENET MULTIPOSE ==", row, C_YELLOW, 0.6, 2)
    row += 28; sep(row); row += 18
    text(f"FPS : {fps:5.1f}    frame: {frame_ms:.0f} ms", row, C_CYAN)
    row += 28; sep(row); row += 18

    n = len(people)
    text(f"people : {n}    target : {selected_idx if selected_idx is not None else 'NONE'}",
         row, C_GREEN if n > 0 else C_GRAY, 0.6, 2)
    row += 28; sep(row); row += 18

    if selected_idx is not None:
        kps = people[selected_idx]["keypoints"]
        score = people[selected_idx]["score"]
        text(f"target score: {score:.2f}", row, C_CYAN, 0.5)
        row += 22
        for i, name in enumerate(KP_NAMES):
            kp = kps[i]
            conf = kp["conf"]
            bar = "#" * int(conf * 10)
            color = C_GREEN if conf >= 0.3 else (C_YELLOW if conf >= 0.15 else C_GRAY)
            text(f"{name:<12} {conf:.2f} {bar}", row, color, 0.42)
            row += 17
            if row > h_panel - 30:
                break
    else:
        text("(no target)", row, C_GRAY, 0.5)
        row += 22

    row += 6; sep(row); row += 16
    text("q: quit   s: snapshot", row, C_GRAY, 0.45)
    return panel


# ── 카메라 & 웹스트림 ──────────────────────────────────────────────────────────

def _jpeg_encode(frame: np.ndarray, quality: int) -> bytes | None:
    ok, enc = cv2.imencode(".jpg", frame,
                           [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return enc.tobytes() if ok else None


def _status_display_bgr(lines: list[str]) -> np.ndarray:
    """카메라 없을 때·대기 중 웹에 보낼 안내 프레임 (영상+패널 합친 크기)."""
    h = CFG["camera"]["height"]
    w = CFG["camera"]["width"] + _PANEL_W
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (45, 45, 45)
    y = 36
    for line in lines:
        cv2.putText(img, line[:80], (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    C_WHITE, 2, cv2.LINE_AA)
        y += 30
    return img


class _WebStreamState:
    def __init__(self, jpeg_quality=70, max_fps=15.0):
        self.jpeg_quality = max(30, min(95, int(jpeg_quality)))
        self._lock = threading.Lock()
        self._last_t = 0.0
        self._min_dt = 1.0 / max(0.5, float(max_fps))
        self._last_stall_t = 0.0
        # 첫 연결 직후 브라우저가 멈추지 않도록 즉시 보낼 JPEG (latest None 금지)
        ph = _jpeg_encode(
            _status_display_bgr(["MoveNet web stream", "waiting for first camera frame..."]),
            self.jpeg_quality,
        )
        self._latest: bytes = ph or b""

    def update(self, frame: np.ndarray):
        now = time.time()
        if now - self._last_t < self._min_dt:
            return
        enc = _jpeg_encode(frame, self.jpeg_quality)
        if enc is None:
            return
        with self._lock:
            self._latest = enc
            self._last_t = now

    def update_stall(self, lines: list[str], min_interval: float = 0.35) -> None:
        """카메라 타임아웃 등으로 메인 루프에 영상이 없을 때 주기적으로 안내."""
        now = time.time()
        if now - self._last_stall_t < min_interval:
            return
        enc = _jpeg_encode(_status_display_bgr(lines), self.jpeg_quality)
        if enc is None:
            return
        with self._lock:
            self._latest = enc
            self._last_stall_t = now
            self._last_t = now

    def latest(self) -> bytes:
        with self._lock:
            return self._latest


def _make_handler(state: _WebStreamState):
    class H(server.BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                html = (
                    "<!doctype html><html><head><meta charset='utf-8'>"
                    "<title>MoveNet Test</title></head>"
                    "<body style='margin:0;background:#111;color:#eee;font-family:monospace'>"
                    "<h3 style='margin:10px'>MoveNet Lightning Live</h3>"
                    "<img src='/stream.mjpg' style='max-width:100vw'>"
                    "</body></html>"
                ).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
                return
            if self.path != "/stream.mjpg":
                self.send_response(404); self.end_headers(); return
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                frame = state.latest()
                if not frame:
                    time.sleep(0.03); continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    time.sleep(0.01)
                except (BrokenPipeError, ConnectionResetError):
                    break

        def log_message(self, *a): return

    return H


class _ThreadedHTTP(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True


class _RpicamVidCamera:
    """rpicam-vid 서브프로세스로 MJPEG 프레임을 받아오는 백엔드.

    picamera2(libcamera 파이썬 바인딩) 없이도, 검증된 rpicam-vid 바이너리를
    통해 카메라를 사용할 수 있게 해줌. 파이썬 버전/ABI 충돌과 무관하게 동작.

    raw yuv420 대신 MJPEG을 쓰는 이유: libcamera 계열 raw 출력은 실제 프레임에
    stride 정렬 패딩이 붙는 경우가 있어서, width*height*1.5바이트를 "한 프레임"
    이라고 가정하고 잘라 읽으면 프레임 경계가 밀려 화면이 찢어져 보이는(티어링)
    문제가 생긴다. MJPEG은 프레임마다 SOI(FFD8)~EOI(FFD9) 마커로 경계가 스스로
    표시되어 있어서 이런 크기 계산이 아예 필요 없다.
    """

    _SOI = b"\xff\xd8"
    _EOI = b"\xff\xd9"
    _MAX_BUF = 2_000_000  # 마커를 못 찾고 버퍼가 무한히 쌓이는 것 방지

    def __init__(self, width: int, height: int, fps: int = 30, cam_idx: int = 0) -> None:
        import subprocess
        self.width = width
        self.height = height
        cmd = [
            "rpicam-vid",
            "--camera", str(cam_idx),        # CSI 포트 선택 (cam0/cam1)
            "-t", "0",                       # 무제한 실행
            "--width", str(width),
            "--height", str(height),
            "--framerate", str(fps),
            "--codec", "mjpeg",
            "--quality", "90",                # JPEG 압축 손실 최소화 (인식 정확도 영향 줄이기)
            "-o", "-",                        # stdout으로 출력
            "-n",                             # 미리보기 창 비활성화
            "--flush",                        # 버퍼링 최소화 (지연 감소)
        ]
        self.proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0,
        )
        self._buf = bytearray()
        print(f"[camera] rpicam-vid MJPEG cam{cam_idx} ({width}x{height}@{fps}fps, pid={self.proc.pid})")

    def read_jpeg(self) -> bytes | None:
        """스트림에서 가장 최신의 완결된 JPEG 프레임(SOI~EOI)을 찾아 반환한다.

        처리(추론)가 카메라 출력 속도보다 느리면 파이프에 프레임이 여러 개
        밀려 쌓일 수 있다. 오래된 프레임부터 순서대로 꺼내면 지연이 계속
        누적되므로, 파이프에 남은 데이터를 먼저 논블로킹으로 다 긁어온 뒤
        가장 마지막(최신) 프레임만 반환하고 그 이전 것들은 버린다.
        프로세스 종료/에러 시 None.
        """
        if self.proc.poll() is not None:
            return None

        fd = self.proc.stdout.fileno()
        while True:
            # 파이프에 이미 도착해 있는 데이터를 논블로킹으로 최대한 끌어온다
            # (처리 지연으로 밀려 쌓인 프레임을 한 번에 파악하기 위함)
            while select.select([fd], [], [], 0)[0]:
                chunk = self.proc.stdout.read(65536)
                if not chunk:
                    return None
                self._buf.extend(chunk)
                if len(self._buf) > self._MAX_BUF:
                    self._buf.clear()

            # 버퍼 안에서 가장 마지막(최신) SOI~EOI 프레임을 찾는다
            last_start = self._buf.rfind(self._SOI)
            if last_start != -1:
                end = self._buf.find(self._EOI, last_start)
                if end != -1:
                    jpg = bytes(self._buf[last_start:end + 2])
                    # 이 프레임까지 통째로 버림 → 그 이전에 밀려 있던 오래된
                    # 프레임들도 함께 폐기되어 지연이 누적되지 않는다
                    del self._buf[: end + 2]
                    return jpg

            # 완결된 프레임이 아직 없으면 블로킹으로 좀 더 기다린다
            chunk = self.proc.stdout.read(4096)
            if not chunk:  # 파이프 종료
                return None
            self._buf.extend(chunk)
            if len(self._buf) > self._MAX_BUF:
                self._buf.clear()

    def stop(self) -> None:
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except Exception:
                self.proc.kill()


def _open_camera(force_opencv, cam_idx, use_rpicam=False):
    ccfg = CFG["camera"]

    if use_rpicam:
        try:
            cam = _RpicamVidCamera(
                ccfg["width"], ccfg["height"], ccfg.get("fps", 30), cam_idx=cam_idx)
            return cam, "rpicam"
        except Exception as e:
            print(f"[camera] rpicam-vid 실패 → OpenCV ({e})")

    if not force_opencv:
        try:
            from picamera2 import Picamera2
            pc = Picamera2(camera_num=cam_idx)
            pc.configure(pc.create_preview_configuration(
                main={"format": "RGB888", "size": (ccfg["width"], ccfg["height"])}))
            pc.start(); time.sleep(0.5)
            print(f"[camera] Picamera2 cam{cam_idx} ({ccfg['width']}x{ccfg['height']})")
            return pc, "picamera2"
        except Exception as e:
            print(f"[camera] Picamera2 실패 → OpenCV ({e})")

    # Pi에서 기본 캡처가 select 타임아웃 나는 경우 V4L2 명시로 완화되는 경우가 있음
    cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print(f"[ERROR] 카메라 {cam_idx} 열기 실패"); sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, ccfg["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ccfg["height"])
    cap.set(cv2.CAP_PROP_FPS, ccfg["fps"])
    # 추론이 느릴 때 오래된 프레임이 드라이버 버퍼에 쌓여 지연이 누적되지 않게
    # 버퍼를 최소화한다 (rpicam 백엔드의 최신-프레임-드롭과 같은 목적).
    # V4L2 등 일부 백엔드만 지원 — 미지원 환경에서는 조용히 무시된다.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print(f"[camera] OpenCV (index={cam_idx})")
    return cap, "opencv"


def _read_frame(cam, backend):
    if backend == "picamera2":
        return cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)
    if backend == "rpicam":
        jpg = cam.read_jpeg()
        if jpg is None:
            return None
        arr = np.frombuffer(jpg, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    ret, frame = cam.read()
    return frame if ret else None


def _release_camera(cam, backend):
    if backend == "picamera2":
        cam.stop()
    elif backend == "rpicam":
        cam.stop()
    else:
        cam.release()


# ── 메인 ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="MoveNet MultiPose Lightning 포즈 확인용 스크립트")
    parser.add_argument("--model", default="multipose_lightning.tflite")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="키포인트 신뢰도 임계값 (기본 0.25)")
    parser.add_argument("--threads", type=int, default=3,
                        help="TFLite 인터프리터 스레드 수 (기본 3, FPS 튜닝용)")
    parser.add_argument("--opencv", action="store_true")
    parser.add_argument("--rpicam", action="store_true",
                        help="rpicam-vid 서브프로세스로 카메라 캡처 (picamera2 미설치 시 사용)")
    parser.add_argument("--cam", type=int, default=0,
                        help="카메라 인덱스 — CSI 케이블 포트(cam0/cam1) 또는 OpenCV 장치 번호")
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--web", action="store_true")
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=8090)
    parser.add_argument("--web-quality", type=int, default=75)
    parser.add_argument("--web-fps", type=float, default=20.0)
    parser.add_argument("--gray", action="store_true",
                        help="입력 프레임을 그레이스케일로 변환 후 3채널로 복원해 추론"
                             " (색 정보 손실만 흉내, 순수 그레이스케일 검출률 확인용)")
    parser.add_argument("--noir-sim", action="store_true",
                        help="그레이스케일 + 점광원 vignette + 저조도 노이즈 + 대비 저하로"
                             " NoIR 야간 촬영 '촬영 조건'을 근사 흉내내 추론"
                             " (IR 파장대 고유 재질 반사율 차이는 재현 불가 — 근사치일 뿐)")
    args = parser.parse_args()

    tracker = PoseTracker()

    if not Path(args.model).exists():
        print(f"[ERROR] 모델 없음: {args.model}")
        sys.exit(1)

    detector = MoveNetMultiPoseDetector(args.model, conf_thr=args.conf, num_threads=args.threads)
    cam, backend = _open_camera(args.opencv, args.cam, use_rpicam=args.rpicam)

    web_srv = web_state = None
    if args.web:
        web_state = _WebStreamState(args.web_quality, args.web_fps)
        web_srv = _ThreadedHTTP((args.web_host, args.web_port), _make_handler(web_state))
        threading.Thread(target=web_srv.serve_forever, daemon=True).start()
        print(f"[web] http://{args.web_host}:{args.web_port}/")

    fps_hist: list[float] = []
    t_prev = time.time()
    last_log = time.time()
    snap_n = 0
    prev_center: tuple[float, float] | None = None  # 직전에 선정됐던 대상자의 위치 (히스테리시스용)
    empty_regions = {k: {"cx": 0.5, "cy": 0.5, "visible": False}
                      for k in ("head", "upper", "lower")}

    print("\n[camera] 시작! 키: q=종료 s=스크린샷\n")

    try:
        while True:
            t0 = time.time()
            frame = _read_frame(cam, backend)
            if frame is None:
                if args.web and web_state:
                    web_state.update_stall([
                        "no frames from the camera.",
                        "CSI module: install picamera2 in the venv, run without --opencv",
                        "USB webcam: --opencv --cam 0 (or 1)",
                        "check if another process is holding the camera.",
                    ])
                time.sleep(0.05)
                continue

            if args.noir_sim:
                frame = simulate_noir(frame)
            elif args.gray:
                # NoIR로 야간 촬영 시 실제로는 흑백(단채널) 영상만 얻게 되므로,
                # 컬러 정보가 없는 상태에서도 MoveNet이 잘 검출하는지 확인하기 위해
                # 컬러 정보를 실제로 날려서(그레이스케일 변환 후 3채널로 복제) 추론에 넣는다.
                # 3채널로 되돌리는 이유: 모델 입력 shape이 (H,W,3)으로 고정이라 채널 수를
                # 맞춰야 하기 때문 — 각 채널 값은 전부 같아서 색 정보는 여전히 없다.
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            result = detector.infer(frame)          # {"detected": bool, "people": [...]}
            people = result["people"]

            # 다중 인원 중 추적 대상 1인 선정 (bbox 면적 최대, 단 직전 대상자와 면적이
            # 비슷하면 갈아타지 않고 유지 — 히스테리시스)
            target_idx = (
                select_target([p["keypoints"] for p in people], conf_thr=args.conf, prev_center=prev_center)
                if people else None
            )

            if target_idx is not None:
                new_center = person_center(people[target_idx]["keypoints"], conf_thr=args.conf)
                # 선정된 대상이 "다른 사람"으로 교체된 프레임이면(직전 대상 위치에서
                # match_radius보다 멀리 있으면 동일인일 수 없음) 스무딩 상태를 리셋해
                # 새 사람 위치로 즉시 점프시킨다 — EMA가 이전 사람→새 사람 사이
                # 허공을 미끄러지듯 이동하는 것 방지 (부드러움은 모터 속도 제한 몫).
                if (
                    prev_center is not None
                    and new_center is not None
                    and ((new_center[0] - prev_center[0]) ** 2
                         + (new_center[1] - prev_center[1]) ** 2) ** 0.5 > DEFAULT_MATCH_RADIUS
                ):
                    tracker.reset()
                tracker_input = {"detected": True, "regions": people[target_idx]["regions"]}
                prev_center = new_center
            else:
                tracker_input = {"detected": False, "regions": empty_regions}
            smoothed = tracker.update(tracker_input)

            vis = draw_pose(frame, people, target_idx)

            # 스무딩 결과 추가로 그리기 (흰 테두리 대신 파란 테두리로 구분)
            h, w = vis.shape[:2]
            for name, r in smoothed.items():
                if r["visible"]:
                    cx, cy = int(r["cx"] * w), int(r["cy"] * h)
                    cv2.circle(vis, (cx, cy), 16, (255, 100, 0), 3)  # 파란 원

            # FPS
            dt = time.time() - t_prev; t_prev = time.time()
            fps_hist.append(1.0 / dt if dt > 0 else 0.0)
            if len(fps_hist) > 30: fps_hist.pop(0)
            fps = sum(fps_hist) / len(fps_hist)
            frame_ms = dt * 1000

            panel = build_panel(people, target_idx, fps, frame_ms)
            display = np.hstack([vis, panel])
            cv2.putText(display, f"FPS {fps:.1f}", (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, C_CYAN, 2)
            if args.noir_sim:
                cv2.putText(display, "NOIR-SIM MODE (approximation)", (10, 46),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_YELLOW, 2)
            elif args.gray:
                cv2.putText(display, "GRAY MODE", (10, 46),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_YELLOW, 2)

            # 콘솔 로그
            if time.time() - last_log >= 1.0:
                r = people[target_idx]["regions"] if target_idx is not None else empty_regions
                print(
                    f"\r[{time.strftime('%H:%M:%S')}] "
                    f"people={len(people)}  target={target_idx if target_idx is not None else '-':<4}  "
                    f"H={'O' if r['head']['visible'] else 'X'} "
                    f"U={'O' if r['upper']['visible'] else 'X'} "
                    f"L={'O' if r['lower']['visible'] else 'X'}  "
                    f"fps={fps:.1f}   ",
                    end="", flush=True,
                )
                last_log = time.time()

            if args.web and web_state:
                web_state.update(display)

            if not args.no_window:
                cv2.imshow("Target Fan | MoveNet", display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    name = f"snapshot_movenet_{snap_n:04d}.jpg"
                    cv2.imwrite(name, display)
                    snap_n += 1
                    print(f"\n[snap] {name}")
            else:
                time.sleep(max(0, 0.02 - (time.time() - t0)))

    except KeyboardInterrupt:
        print("\n[camera] Ctrl+C 중단")
    finally:
        # q 종료·Ctrl+C·예외 어느 경로로 빠져도 카메라/웹서버를 반드시 정리한다
        # (--no-window 헤드리스 모드는 Ctrl+C가 유일한 종료 수단이라 특히 중요)
        if web_srv:
            web_srv.shutdown(); web_srv.server_close()
        _release_camera(cam, backend)
        cv2.destroyAllWindows()
        print("\n[camera] 종료")


if __name__ == "__main__":
    main()
