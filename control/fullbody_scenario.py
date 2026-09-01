"""
control/fullbody_scenario.py

전신(머리→발) 추적 시나리오 상태기계 — 순수 계산 모듈.
GPIO/BlueZ 등 하드웨어 라이브러리를 import하지 않습니다. 입력은 프레임별 관측
dict(장부 각도·부위 좌표), 출력은 팬/틸트 목표 각도(degree) — 순수 데이터.
실제 모터 구동은 hardware/motor_controller.py 몫입니다.

전제: 카메라가 팬·틸트 헤드에 함께 실려 움직인다 (2026-07-10 확정). 그래서 틸트도
팬과 같은 피드백(상대 보정) 루프다 — 새 목표각 = 현재 장부각 + gain × 오차각,
오차각은 FOV만으로 정해지고 거리에 의존하지 않는다
(docs/tracking_feedback.md 참고).

상태기계:
  scan     head→upper→lower 순서로 부위를 화면 중앙에 수렴시키고(오차 < converge_deg
           연속 converge_frames), 수렴 시점의 틸트를 웨이포인트로 기록. 아직 안 보이는
           부위는 예상 방향으로 blind_deg씩 틸트를 옮기고, lower는 head·upper 간격 ×
           lower_ratio 추정 지점으로 직행한다.
  patrol   기록된 웨이포인트를 head→upper→lower→upper 순환 재생, 부위별 dwell_s 체류.
           부위가 보이면 피드백 미세 보정 + 웨이포인트 EMA 갱신(trim_alpha).
  recenter 가슴 수평 오차 > move_thr_deg (사용자 이동) → 가슴을 다시 중앙으로. 완료 후
           이동량이 rescan_thr_deg 이하면 중단 지점 재개, 초과면 거리·자세가 변한
           것으로 보고 전신 재스캔.
  search   대상 상실 lost_frames 연속 → 틸트를 어깨가 보일 각도(upper 웨이포인트,
           없으면 0°)로 올리고 팬을 body_pan ± search_step_deg 확장 스윕. 어깨
           재검출 시 recenter로 넘어가 중단 지점을 재개한다. 하체 조준 중에는 사람이
           프레임에 원래 안 담기므로 상실 카운트는 머리/상체 조준 중에만 센다.

가림·리밋 처리:
  - 피드백은 fresh(이번 프레임 실관측) 부위에만 적용 — 미관측 유예 동안 스테일
    좌표를 따라 계속 내려가는 런어웨이를 막는다.
  - 모든 틸트 목표는 [tilt_min, tilt_max]로 clamp (데드라인 실측 전 임시값은
    config.py "limits" 참고). occl_frames 연속 미관측이거나 리밋에 눌린 채 수렴
    불가면 추정 틸트를 estimated=True로 기록하고 다음 부위로 진행한다.

좌표 기억 구조: 수평은 body_pan 단일값(사람은 좌우로 한 덩어리로 움직이므로 팬 보정
시 경로 전체가 함께 이동), 수직은 waypoints[부위]["tilt"].

step()은 상태 전이 로그를 self.events(문자열 리스트)에 채운다 — 출력은 호출부 몫.
파라미터 기본값은 실기 검증 전 시작값이라 호출부(CLI)가 덮어쓴다.
"""

from __future__ import annotations

from control.control_signal_generator import (clamp_angle, compute_pan_angle,
                                              compute_tilt_angle)


class FullBodyScenario:
    """관측 dict → (팬, 틸트) 목표각. 관측 형식:

    {
      "t": float,             # time.time()
      "pos": (pan, tilt),     # 모터 장부 각도 (°)
      "idle": bool,           # 두 축 모두 목표 도달 여부
      "target_idx": int|None, # 선정된 대상 인덱스 (없으면 None)
      "chest": {cx, cy, visible},          # 어깨 중점 (이번 프레임 원시 관측)
      "regions": {head/upper/lower: {cx, cy, visible}},  # 스무딩 좌표
      "fresh": {head/upper/lower: bool},   # 이번 프레임 실관측 여부
    }
    """

    SCAN_ORDER = ("head", "upper", "lower")
    PATROL_ORDER = ("head", "upper", "lower", "upper")

    def __init__(
        self,
        fov_h_deg: float,
        fov_v_deg: float,
        pan_min: float, pan_max: float,
        tilt_min: float, tilt_max: float,
        gain: float = 0.3,
        gain_tilt: float = 0.25,
        invert_pan: bool = False,
        invert_tilt: bool = False,
        converge_deg: float = 1.5,
        converge_frames: int = 4,
        dwell_s: float = 2.0,
        trim_alpha: float = 0.2,
        move_thr_deg: float = 8.0,
        rescan_thr_deg: float = 12.0,
        lost_frames: int = 15,
        occl_frames: int = 50,
        blind_deg: float = 2.0,
        lower_ratio: float = 1.1,
        search_step_deg: float = 15.0,
        search_limit_deg: float = 60.0,
        search_dwell: int = 8,
    ) -> None:
        self.fov_h, self.fov_v = fov_h_deg, fov_v_deg
        self.pan_min, self.pan_max = pan_min, pan_max
        self.tilt_min, self.tilt_max = tilt_min, tilt_max
        self.gain, self.gain_tilt = gain, gain_tilt
        self.sign_pan = -1.0 if invert_pan else 1.0
        self.sign_tilt = -1.0 if invert_tilt else 1.0
        self.converge_deg = converge_deg
        self.converge_frames = converge_frames
        self.dwell_s = dwell_s
        self.trim_alpha = trim_alpha
        self.move_thr_deg = move_thr_deg
        self.rescan_thr_deg = rescan_thr_deg
        self.lost_frames = lost_frames
        self.occl_frames = occl_frames
        self.blind_deg = blind_deg
        self.lower_ratio = lower_ratio
        self.search_step_deg = search_step_deg
        self.search_limit_deg = search_limit_deg
        self.search_dwell = search_dwell

        self.state = "scan"
        self.scan_i = 0
        self.body_pan = 0.0
        self.waypoints: dict[str, dict] = {}   # 부위 → {"tilt": °, "estimated": bool}
        self.events: list[str] = []

        self._cvg = 0
        self._occl = 0
        self._lost = 0
        self._patrol_i = 0
        self._dwell_until: float | None = None
        self._resume: tuple | None = None       # 중단 지점 ("scan", i) | ("patrol",)
        self._recenter_from = 0.0
        self._recenter_miss = 0
        self._search_seq: list[float] = []
        self._search_i = 0
        self._search_hold = 0
        self._search_tilt = 0.0

    # ── 헬퍼 ─────────────────────────────────────────────────────────────────

    def _pan_err(self, cx: float) -> float:
        return self.sign_pan * compute_pan_angle(cx, self.fov_h)

    def _tilt_err(self, cy: float) -> float:
        return self.sign_tilt * compute_tilt_angle(cy, self.fov_v)

    def _cp(self, deg: float) -> float:
        return clamp_angle(deg, self.pan_min, self.pan_max)

    def _ct(self, deg: float) -> float:
        return clamp_angle(deg, self.tilt_min, self.tilt_max)

    def _route(self) -> list[str]:
        return [r for r in self.PATROL_ORDER if r in self.waypoints]

    def active_region(self) -> str:
        if self.state == "scan" and self.scan_i < len(self.SCAN_ORDER):
            return self.SCAN_ORDER[self.scan_i]
        if self.state == "patrol":
            route = self._route()
            if route:
                return route[self._patrol_i % len(route)]
        return "-"

    def _make_resume(self) -> tuple | None:
        if self.state == "scan":
            return ("scan", self.scan_i)
        if self.state == "patrol":
            return ("patrol",)
        return self._resume

    def _est_lower_tilt(self) -> float | None:
        """head·upper 웨이포인트 간격 × lower_ratio로 하체 틸트를 추정.
        비율 근사라 거리(원근)가 변해도 같이 스케일된다."""
        wh, wu = self.waypoints.get("head"), self.waypoints.get("upper")
        if wh is None or wu is None:
            return None
        return self._ct(wu["tilt"] + (wu["tilt"] - wh["tilt"]) * self.lower_ratio)

    # ── 상태 전이 ────────────────────────────────────────────────────────────

    def _restart_scan(self) -> None:
        self.state = "scan"
        self.scan_i = 0
        self._cvg = self._occl = 0

    def _advance_scan(self) -> None:
        self._cvg = self._occl = 0
        self.scan_i += 1
        if self.scan_i >= len(self.SCAN_ORDER):
            self.state = "patrol"
            self._patrol_i = 0
            self._dwell_until = None
            wps = "  ".join(f"{k}:{v['tilt']:+.1f}°{'(추정)' if v['estimated'] else ''}"
                            for k, v in self.waypoints.items())
            self.events.append(f"스캔 완료 → 순찰 시작: {wps} @ pan {self.body_pan:+.1f}°")

    def _enter_search(self, reason: str) -> None:
        if self.state in ("scan", "patrol"):
            self._resume = self._make_resume()
        self.state = "search"
        self._lost = 0
        wu = self.waypoints.get("upper")
        self._search_tilt = self._ct(wu["tilt"]) if wu else 0.0
        seq = [self.body_pan]
        k = 1
        while k * self.search_step_deg <= self.search_limit_deg:
            seq += [self._cp(self.body_pan + k * self.search_step_deg),
                    self._cp(self.body_pan - k * self.search_step_deg)]
            k += 1
        self._search_seq = list(dict.fromkeys(round(s, 3) for s in seq))
        self._search_i = self._search_hold = 0
        self.events.append(f"{reason} → 어깨 탐색 (tilt {self._search_tilt:+.1f}°, "
                           f"pan {self.body_pan:+.1f}°±{self.search_limit_deg:g}°)")

    # ── 메인 스텝 ────────────────────────────────────────────────────────────

    def step(self, obs: dict) -> tuple[float, float]:
        self.events = []
        cur_pan, cur_tilt = obs["pos"]

        # 대상 상실 카운트 — 머리/상체 조준 중에만 (하체 조준 중엔 원래 안 보임)
        if obs["target_idx"] is None:
            if (self.state in ("scan", "patrol")
                    and self.active_region() in ("head", "upper")):
                self._lost += 1
        else:
            self._lost = 0
        if self.state in ("scan", "patrol") and self._lost >= self.lost_frames:
            self._enter_search("대상 상실")

        # 사용자 이동 감지 — 가슴 수평 오차가 크면 재조준
        chest = obs["chest"]
        if self.state in ("scan", "patrol") and chest["visible"]:
            err = self._pan_err(chest["cx"])
            if abs(err) > self.move_thr_deg:
                self._resume = self._make_resume()
                self._recenter_from = cur_pan
                self._recenter_miss = 0
                self.state = "recenter"
                self.events.append(f"이동 감지 (가슴 오차 {err:+.1f}°) → 재조준")

        if self.state == "scan":
            return self._step_scan(obs, cur_pan, cur_tilt)
        if self.state == "patrol":
            return self._step_patrol(obs, cur_pan, cur_tilt)
        if self.state == "recenter":
            return self._step_recenter(obs, cur_pan, cur_tilt)
        return self._step_search(obs, cur_pan, cur_tilt)

    # ── 상태별 처리 ──────────────────────────────────────────────────────────

    def _step_scan(self, obs: dict, cur_pan: float, cur_tilt: float) -> tuple[float, float]:
        region = self.SCAN_ORDER[self.scan_i]
        fresh = obs["fresh"][region]
        r = obs["regions"][region]
        chest = obs["chest"]
        pan_t, tilt_t = self.body_pan, cur_tilt

        if fresh:
            ep, et = self._pan_err(r["cx"]), self._tilt_err(r["cy"])
            pan_t = self._cp(cur_pan + self.gain * ep)
            tilt_t = self._ct(cur_tilt + self.gain_tilt * et)
            self.body_pan = pan_t
            # 리밋에 눌린 채 수렴 불가 = 물리적으로 닿지 않는 부위
            if tilt_t in (self.tilt_min, self.tilt_max) and abs(et) >= self.converge_deg:
                self._occl += 1
                if self._occl >= self.occl_frames:
                    self.waypoints[region] = {"tilt": tilt_t, "estimated": True}
                    self.events.append(
                        f"[scan] {region}: 틸트 리밋 {tilt_t:+.1f}°에서 수렴 불가 — 추정 기록")
                    self._advance_scan()
                return pan_t, tilt_t
            self._occl = 0
            if abs(ep) < self.converge_deg and abs(et) < self.converge_deg:
                self._cvg += 1
                if self._cvg >= self.converge_frames:
                    self.waypoints[region] = {"tilt": tilt_t, "estimated": False}
                    self.events.append(
                        f"[scan] {region} 기록: tilt={tilt_t:+.2f}° (pan={pan_t:+.2f}°)")
                    self._advance_scan()
            else:
                self._cvg = 0
            return pan_t, tilt_t

        # 부위 미관측: 타임아웃이면 추정 기록/건너뜀, 그 전엔 예상 방향 이동
        self._cvg = 0
        self._occl += 1
        if self._occl >= self.occl_frames:
            if region == "lower":
                est = self._est_lower_tilt()
                tilt_rec = est if est is not None else self._ct(cur_tilt)
                self.waypoints["lower"] = {"tilt": tilt_rec, "estimated": True}
                self.events.append(f"[scan] lower 미검출(가림?) — 추정 tilt={tilt_rec:+.2f}° 기록")
            else:
                self.events.append(f"[scan] {region} {self.occl_frames}프레임 미검출 — 건너뜀")
            self._advance_scan()
            return pan_t, tilt_t
        est = self._est_lower_tilt() if region == "lower" else None
        if est is not None:
            tilt_t = est
        else:
            down = -1.0 if region == "head" else 1.0   # head는 화면 위쪽, 나머지는 아래쪽
            tilt_t = self._ct(cur_tilt + self.gain_tilt * self.sign_tilt * down * self.blind_deg)
        if chest["visible"]:
            pan_t = self._cp(cur_pan + self.gain * self._pan_err(chest["cx"]))
            self.body_pan = pan_t
        return pan_t, tilt_t

    def _step_patrol(self, obs: dict, cur_pan: float, cur_tilt: float) -> tuple[float, float]:
        route = self._route()
        if not route:
            self.events.append("웨이포인트 없음 — 스캔 재시작")
            self._restart_scan()
            return self.body_pan, cur_tilt
        region = route[self._patrol_i % len(route)]
        wp = self.waypoints[region]
        fresh = obs["fresh"][region]
        r = obs["regions"][region]
        chest = obs["chest"]

        pan_t, tilt_t = self.body_pan, self._ct(wp["tilt"])
        if fresh:
            ep, et = self._pan_err(r["cx"]), self._tilt_err(r["cy"])
            pan_t = self._cp(cur_pan + self.gain * ep)
            tilt_t = self._ct(cur_tilt + self.gain_tilt * et)
            self.body_pan = pan_t
            if abs(et) < self.converge_deg:   # 보이는 동안 기억 경로를 서서히 최신화
                wp["tilt"] += self.trim_alpha * (tilt_t - wp["tilt"])
                wp["estimated"] = False
        elif chest["visible"]:
            pan_t = self._cp(cur_pan + self.gain * self._pan_err(chest["cx"]))
            self.body_pan = pan_t

        # 도착 후 dwell_s 체류 → 다음 부위
        if obs["idle"] and abs(cur_tilt - self._ct(wp["tilt"])) <= self.converge_deg:
            if self._dwell_until is None:
                self._dwell_until = obs["t"] + self.dwell_s
            elif obs["t"] >= self._dwell_until:
                self._patrol_i = (self._patrol_i + 1) % len(route)
                self._dwell_until = None
        return pan_t, tilt_t

    def _step_recenter(self, obs: dict, cur_pan: float, cur_tilt: float) -> tuple[float, float]:
        chest = obs["chest"]
        if not chest["visible"]:
            self._recenter_miss += 1
            if self._recenter_miss >= self.lost_frames:
                self._enter_search("재조준 중 가슴 상실")
            return self.body_pan, cur_tilt
        self._recenter_miss = 0
        ep, et = self._pan_err(chest["cx"]), self._tilt_err(chest["cy"])
        pan_t = self._cp(cur_pan + self.gain * ep)
        tilt_t = self._ct(cur_tilt + self.gain_tilt * et)
        self.body_pan = pan_t
        if abs(ep) < self.converge_deg:
            moved = abs(pan_t - self._recenter_from)
            if self._resume is None or moved > self.rescan_thr_deg:
                self.events.append(f"재조준 완료 (이동 {moved:.1f}°) — 전신 재스캔")
                self._restart_scan()
            elif self._resume[0] == "scan":
                self.state, self.scan_i = "scan", self._resume[1]
                self._cvg = self._occl = 0
                self.events.append(f"재조준 완료 (이동 {moved:.1f}°) — "
                                   f"스캔 재개: {self.SCAN_ORDER[self.scan_i]}")
            else:
                self.state = "patrol"
                self._dwell_until = None
                self.events.append(f"재조준 완료 (이동 {moved:.1f}°) — 순찰 재개")
            self._resume = None
        return pan_t, tilt_t

    def _step_search(self, obs: dict, cur_pan: float, cur_tilt: float) -> tuple[float, float]:
        if obs["chest"]["visible"]:
            self._recenter_from = self.body_pan   # 상실 지점 기준으로 이동량 판정
            self._recenter_miss = 0
            self.state = "recenter"
            self.events.append("어깨 재검출 → 재조준")
            return cur_pan, cur_tilt
        pan_t = self._search_seq[self._search_i]
        if obs["idle"] and abs(cur_pan - pan_t) <= self.converge_deg:
            self._search_hold += 1
            if self._search_hold >= self.search_dwell:
                self._search_hold = 0
                self._search_i += 1
                if self._search_i >= len(self._search_seq):
                    self._search_i = 0
                    self.events.append("탐색 스윕 미발견 — 반복")
        return pan_t, self._search_tilt
