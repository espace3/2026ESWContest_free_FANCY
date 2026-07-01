"""
test_movenet.py - MoveNet Lightning 포즈 감지 테스트

MediaPipe 대비 장점:
  - 머리가 안 보여도 몸통/하체만으로 감지 가능
  - Raspberry Pi에서 더 높은 FPS
  - 단일 추론으로 포즈 완성 (2-stage 불필요)

설치:
    pip install tflite-runtime

모델: movenet_lightning.tflite (4.6MB, 192x192 입력)

실행:
    python test_movenet.py
    python test_movenet.py --no-window
    python test_movenet.py --web --no-window --web-port 8090
"""

from __future__ import annotations
from pose_tracker import PoseTracker

import argparse
import sys
import threading
import time
from http import server
from pathlib import Path
from socketserver import ThreadingMixIn

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from config import CFG

# ── MoveNet 키포인트 정의 (COCO 17개) ────────────────────────────────────────
# 0:nose 1:l_eye 2:r_eye 3:l_ear 4:r_ear
# 5:l_shoulder 6:r_shoulder 7:l_elbow 8:r_elbow 9:l_wrist 10:r_wrist
# 11:l_hip 12:r_hip 13:l_knee 14:r_knee 15:l_ankle 16:r_ankle
_KP_NAMES = [
    "nose","l_eye","r_eye","l_ear","r_ear",
    "l_shoulder","r_shoulder","l_elbow","r_elbow","l_wrist","r_wrist",
    "l_hip","r_hip","l_knee","r_knee","l_ankle","r_ankle",
]
_HEAD_IDX  = [0, 3, 4]            # nose, l_ear, r_ear
_UPPER_IDX = [5, 6, 11, 12]       # shoulders + hips
_LOWER_IDX = [13, 14, 15, 16]     # knees + ankles

# 골격 연결선
_SKELETON = [
    (0,1),(0,2),(1,3),(2,4),           # 얼굴
    (5,6),(5,7),(7,9),(6,8),(8,10),    # 상체
    (5,11),(6,12),(11,12),             # 몸통
    (11,13),(13,15),(12,14),(14,16),   # 하체
]

# ── 색상 ─────────────────────────────────────────────────────────────────────
C_GREEN  = (0, 200, 60)
C_YELLOW = (0, 210, 230)
C_CYAN   = (220, 200, 0)
C_WHITE  = (255, 255, 255)
C_GRAY   = (160, 160, 160)
C_PANEL  = (30, 30, 30)
C_RED    = (0, 50, 220)
_PANEL_W = 340

REGION_COLOR = {"head": (0, 50, 220), "upper": (0, 200, 60), "lower": (220, 80, 0)}
REGION_LABEL = {"head": "HEAD", "upper": "UPPER", "lower": "LOWER"}


# ── MoveNet 추론기 ────────────────────────────────────────────────────────────

class MoveNetDetector:
    """MoveNet Lightning TFLite 래퍼."""

    INPUT_SIZE = 192  # Lightning 고정 입력

    def __init__(self, model_path: str, conf_thr: float = 0.3) -> None:
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
        self._interp.allocate_tensors()
        self._inp  = self._interp.get_input_details()[0]
        self._out  = self._interp.get_output_details()[0]
        self._conf_thr = conf_thr
        self._s = self.INPUT_SIZE

        print(f"[movenet] 모델 로드: {model_path}")
        print(f"[movenet] 입력: {self._inp['shape']}  dtype={self._inp['dtype'].__name__}")
        print(f"[movenet] 출력: {self._out['shape']}")
        print(f"[movenet] conf_thr={conf_thr}")

    def infer(self, frame_bgr: np.ndarray) -> dict:
        """
        BGR 프레임 → 포즈 결과 dict 반환.
        반환:
          detected: bool
          keypoints: list of {x,y,conf} (정규화 좌표 [0,1])
          regions:   {head, upper, lower} → {cx, cy, visible} (정규화)
        """
        s = self._s
        h_orig, w_orig = frame_bgr.shape[:2]

        # ── 전처리: letterbox → 192x192 ─────────────────────────────────
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
        raw = self._interp.get_tensor(self._out['index'])  # (1,1,17,3)
        kps_raw = raw[0, 0]  # (17, 3) = [y_norm, x_norm, conf]

        # ── 좌표 변환: letterbox → 원본 정규화 ────────────────────────
        keypoints = []
        for ky, kx, kc in kps_raw:
            # letterbox 내 픽셀 → 원본 픽셀
            px = (float(kx) * s - pad_x) / scale
            py = (float(ky) * s - pad_y) / scale
            # 정규화
            keypoints.append({
                "x": max(0.0, min(1.0, px / w_orig)),
                "y": max(0.0, min(1.0, py / h_orig)),
                "conf": float(kc),
            })

        # ── 영역 중심 계산 ────────────────────────────────────────────
        kps = keypoints
        head_conf  = [kps[i]["conf"] for i in _HEAD_IDX]
        upper_conf = [kps[i]["conf"] for i in _UPPER_IDX]
        lower_conf = [kps[i]["conf"] for i in _LOWER_IDX]

        head_visible  = any(c >= self._conf_thr for c in head_conf)
        upper_visible = sum(c >= self._conf_thr for c in upper_conf) >= 2
        lower_visible = sum(c >= self._conf_thr for c in lower_conf) >= 2

        regions = {
            "head":  self._region_center(keypoints, _HEAD_IDX)  if head_visible  else {"cx": 0.5, "cy": 0.5, "visible": False},
            "upper": self._region_center(keypoints, _UPPER_IDX) if upper_visible else {"cx": 0.5, "cy": 0.5, "visible": False},
            "lower": self._region_center(keypoints, _LOWER_IDX) if lower_visible else {"cx": 0.5, "cy": 0.5, "visible": False},
        }

        any_visible = any(r["visible"] for r in regions.values())
        return {
            "detected":  any_visible,
            "keypoints": keypoints,
            "regions":   regions,
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


# ── 시각화 ───────────────────────────────────────────────────────────────────

def draw_pose(frame: np.ndarray, result: dict) -> np.ndarray:
    """키포인트 + 골격 + 영역 중심을 프레임에 그립니다."""
    if not result["detected"]:
        return frame

    vis = frame.copy()
    h, w = vis.shape[:2]
    kps = result["keypoints"]
    conf_thr = 0.2  # 그리기 임계값은 조금 낮게

    # 골격 선
    for a, b in _SKELETON:
        if kps[a]["conf"] >= conf_thr and kps[b]["conf"] >= conf_thr:
            ax, ay = int(kps[a]["x"] * w), int(kps[a]["y"] * h)
            bx, by = int(kps[b]["x"] * w), int(kps[b]["y"] * h)
            cv2.line(vis, (ax, ay), (bx, by), C_GRAY, 2)

    # 키포인트 점
    for kp in kps:
        if kp["conf"] >= conf_thr:
            cx, cy = int(kp["x"] * w), int(kp["y"] * h)
            cv2.circle(vis, (cx, cy), 4, C_YELLOW, -1)

    # 영역 중심
    for name, color in REGION_COLOR.items():
        r = result["regions"][name]
        if r["visible"]:
            cx = int(r["cx"] * w)
            cy = int(r["cy"] * h)
            cv2.circle(vis, (cx, cy), 12, color, -1)
            cv2.circle(vis, (cx, cy), 12, C_WHITE, 2)
            cv2.putText(vis, REGION_LABEL[name], (cx + 14, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    return vis


def build_panel(result: dict, fps: float, frame_ms: float) -> np.ndarray:
    h_panel = CFG["camera"]["height"]
    panel = np.full((h_panel, _PANEL_W, 3), C_PANEL, dtype=np.uint8)

    def text(msg, row, color=C_WHITE, scale=0.55, bold=1):
        cv2.putText(panel, msg, (10, row), cv2.FONT_HERSHEY_SIMPLEX, scale, color, bold)

    def sep(row):
        cv2.line(panel, (5, row), (_PANEL_W - 5, row), C_GRAY, 1)

    row = 28
    text("== MOVENET LIGHTNING ==", row, C_YELLOW, 0.6, 2)
    row += 28; sep(row); row += 18
    text(f"FPS : {fps:5.1f}    frame: {frame_ms:.0f} ms", row, C_CYAN)
    row += 28; sep(row); row += 18

    detected = result["detected"]
    text(f"person : {'DETECTED' if detected else 'NOT FOUND'}",
         row, C_GREEN if detected else C_GRAY, 0.6, 2)
    row += 28; sep(row); row += 18

    kps = result["keypoints"]
    for i, name in enumerate(_KP_NAMES):
        kp = kps[i]
        conf = kp["conf"]
        bar = "#" * int(conf * 10)
        color = C_GREEN if conf >= 0.3 else (C_YELLOW if conf >= 0.15 else C_GRAY)
        text(f"{name:<12} {conf:.2f} {bar}", row, color, 0.42)
        row += 17
        if row > h_panel - 30:
            break

    row += 6; sep(row); row += 16
    text("q:종료  s:스크린샷", row, C_GRAY, 0.45)
    return panel


# ── 카메라 & 웹스트림 (test_yolo_person.py 동일) ──────────────────────────────

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
            _status_display_bgr(["MoveNet 웹 스트림", "카메라 프레임 대기 중…"]),
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
    """rpicam-vid 서브프로세스로 YUV420 raw 프레임을 받아오는 백엔드.

    picamera2(libcamera 파이썬 바인딩) 없이도, 검증된 rpicam-vid 바이너리를
    통해 카메라를 사용할 수 있게 해줌. 파이썬 버전/ABI 충돌과 무관하게 동작.
    """

    def __init__(self, width: int, height: int, fps: int = 30) -> None:
        import subprocess
        self.width = width
        self.height = height
        # YUV420(I420) 기준 프레임 크기: width*height*1.5 바이트
        self.frame_size = width * height * 3 // 2
        cmd = [
            "rpicam-vid",
            "-t", "0",                       # 무제한 실행
            "--width", str(width),
            "--height", str(height),
            "--framerate", str(fps),
            "--codec", "yuv420",
            "-o", "-",                        # stdout으로 출력
            "-n",                             # 미리보기 창 비활성화
            "--flush",                        # 버퍼링 최소화 (지연 감소)
        ]
        self.proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            bufsize=self.frame_size * 2,
        )
        print(f"[camera] rpicam-vid ({width}x{height}@{fps}fps, pid={self.proc.pid})")

    def read_raw(self) -> bytes | None:
        """정확히 한 프레임 분량의 바이트를 읽음. 프로세스 종료/에러 시 None."""
        if self.proc.poll() is not None:
            return None
        buf = bytearray()
        need = self.frame_size
        while need > 0:
            chunk = self.proc.stdout.read(need)
            if not chunk:  # 파이프 종료
                return None
            buf.extend(chunk)
            need -= len(chunk)
        return bytes(buf)

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
                ccfg["width"], ccfg["height"], ccfg.get("fps", 30))
            return cam, "rpicam"
        except Exception as e:
            print(f"[camera] rpicam-vid 실패 → OpenCV ({e})")

    if not force_opencv:
        try:
            from picamera2 import Picamera2
            pc = Picamera2()
            pc.configure(pc.create_preview_configuration(
                main={"format": "RGB888", "size": (ccfg["width"], ccfg["height"])}))
            pc.start(); time.sleep(0.5)
            print(f"[camera] Picamera2 ({ccfg['width']}x{ccfg['height']})")
            return pc, "picamera2"
        except Exception as e:
            print(f"[camera] Picamera2 실패 → OpenCV ({e})")

    # Pi에서 기본 캡처가 select 타임아웃 나는 경우 V4L2 명시로 완화되는 경우가 있음
    cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print(f"[ERROR] 카메라 {cam_idx} 열기 실패"); sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  ccfg["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ccfg["height"])
    cap.set(cv2.CAP_PROP_FPS,          ccfg["fps"])
    print(f"[camera] OpenCV (index={cam_idx})")
    return cap, "opencv"


def _read_frame(cam, backend):
    if backend == "picamera2":
        return cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)
    if backend == "rpicam":
        raw = cam.read_raw()
        if raw is None:
            return None
        yuv = np.frombuffer(raw, dtype=np.uint8).reshape(
            (cam.height * 3 // 2, cam.width))
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
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
    parser = argparse.ArgumentParser(description="MoveNet Lightning 포즈 테스트")
    parser.add_argument("--model",      default="movenet_lightning.tflite")
    parser.add_argument("--conf",       type=float, default=0.25,
                        help="키포인트 신뢰도 임계값 (기본 0.25)")
    parser.add_argument("--opencv",     action="store_true")
    parser.add_argument("--rpicam",     action="store_true",
                        help="rpicam-vid 서브프로세스로 카메라 캡처 (picamera2 미설치 시 사용)")
    parser.add_argument("--cam",        type=int, default=0)
    parser.add_argument("--no-window",  action="store_true")
    parser.add_argument("--web",        action="store_true")
    parser.add_argument("--web-host",   default="0.0.0.0")
    parser.add_argument("--web-port",   type=int, default=8090)
    parser.add_argument("--web-quality", type=int, default=75)
    parser.add_argument("--web-fps",    type=float, default=20.0)
    args = parser.parse_args()

    tracker = PoseTracker()

    if not Path(args.model).exists():
        print(f"[ERROR] 모델 없음: {args.model}")
        sys.exit(1)

    detector = MoveNetDetector(args.model, conf_thr=args.conf)
    cam, backend = _open_camera(args.opencv, args.cam, use_rpicam=args.rpicam)

    web_srv = web_state = None
    if args.web:
        web_state = _WebStreamState(args.web_quality, args.web_fps)
        web_srv   = _ThreadedHTTP((args.web_host, args.web_port), _make_handler(web_state))
        threading.Thread(target=web_srv.serve_forever, daemon=True).start()
        print(f"[web] http://{args.web_host}:{args.web_port}/")

    fps_hist: list[float] = []
    t_prev   = time.time()
    last_log = time.time()
    snap_n   = 0
    result   = {"detected": False, "keypoints": [{"x":0,"y":0,"conf":0}]*17,
                "regions": {k: {"cx":0.5,"cy":0.5,"visible":False}
                            for k in ("head","upper","lower")}}

    print("\n[test_movenet] 시작! 키: q=종료 s=스크린샷\n")

    while True:
        t0    = time.time()
        frame = _read_frame(cam, backend)
        if frame is None:
            if args.web and web_state:
                web_state.update_stall([
                    "카메라에서 프레임이 오지 않습니다.",
                    "CSI 모듈: venv에 picamera2 설치 후 --opencv 빼고 실행",
                    "USB 웹캠: --opencv --cam 0 또는 1",
                    "다른 프로세스가 카메라 사용 중인지 확인하세요.",
                ])
            time.sleep(0.05)
            continue

        result = detector.infer(frame)
        smoothed = tracker.update(result)
        vis    = draw_pose(frame, result)

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

        panel   = build_panel(result, fps, frame_ms)
        display = np.hstack([vis, panel])
        cv2.putText(display, f"FPS {fps:.1f}", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, C_CYAN, 2)

        # 콘솔 로그
        if time.time() - last_log >= 1.0:
            r = result["regions"]
            print(
                f"\r[{time.strftime('%H:%M:%S')}] "
                f"{'DETECTED' if result['detected'] else 'not found':12s}  "
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

    if web_srv:
        web_srv.shutdown(); web_srv.server_close()
    _release_camera(cam, backend)
    cv2.destroyAllWindows()
    print("\n[test_movenet] 종료")


if __name__ == "__main__":
    main()
