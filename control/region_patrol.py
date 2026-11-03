"""
control/region_patrol.py

부위 인식 모드(0x03)의 순수 계산 모듈 — GPIO/BlueZ/카메라를 import하지 않는다.
입력은 프레임별 관측 dict(장부 각도·부위 좌표), 출력은 팬/틸트 목표 각도(degree).
실제 모터 구동은 hardware/stepper.py 몫이다. app/runners.py 의 부위 러너가 쓴다.

  - RegionPatrolScenario: 부위 순환 추적 — 지금 겨누는 부위를 두 축으로 따라간다
  - region_wind_level:    풍속 중재 — 지금 겨누는 부위의 저장 세기

전제: 카메라가 팬·틸트 헤드에 함께 실려 움직인다 (2026-07-10 확정). 그래서 두 축
모두 피드백(상대 보정) 루프다 — 새 목표각 = 현재 장부각 + gain × 오차각,
오차각은 FOV만으로 정해지고 거리에 의존하지 않는다
(docs/tracking_feedback.md 참고).

이력:
  - 2026-09-02  FullBodyScenario + RegionPatrolScenario 두 클래스를 하나로 합침.
  - 2026-11-03  순찰(틸트만 부위별, 팬은 가슴) + 추적 폴백(MotionGate) +
    재조준·재스캔·탐색 상태기계를 걷어내고 "두 축 모두 지금 겨누는 부위를 따라가는"
    단일 루프로 바꿨다. 사람이 움직이면 순찰이 멈추고 폴백으로 넘어갔다 돌아오는
    구조라 따라가는 동안 부위별 바람이 끊겼고, 부위 구분이 틸트 전용이라 누운
    자세(부위가 가로로 늘어섬)를 다루지 못했다. 이전 구현은 git 이력에 있다 —
    `git show 26ec29c:control/region_patrol.py`.
"""

from __future__ import annotations

import math

from control.control_signal import (clamp_angle, compute_pan_angle,
                                    compute_tilt_angle)


class RegionPatrolScenario:
    """관측 dict → (팬, 틸트) 목표각, 또는 None(명령 없음 = 그 자리 유지).

    관측 형식:
    {
      "t": float,             # time.time()
      "pos": (pan, tilt),     # 모터 장부 각도 (°)
      "target_idx": int|None, # 선정된 대상 인덱스 (없으면 None)
      "chest": {cx, cy, visible, paired},   # 어깨 중점 (이번 프레임 원시 관측)
      "regions": {head/upper/lower: {cx, cy, visible}},  # 스무딩 좌표
      "fresh": {head/upper/lower: bool},    # 이번 프레임 실관측 여부
    }

    ── 조준 ────────────────────────────────────────────────────────────────
    지금 겨누는 부위(active)의 화면 좌표 (cx, cy)를 조준점(target_cx/cy)에
    놓도록 팬·틸트를 함께 움직인다. 타겟 모드(app/tracking.py)와 같은 식이다.

        팬 목표 = 현재 팬 + gain_pan  × (cx - target_cx) × FOV_H
        틸트 목표 = 현재 틸트 + gain_tilt × (cy - target_cy) × FOV_V

    부위를 가로·세로 좌표 그대로 쓰므로 서 있든 누워 있든 같은 식으로 따라간다.
    서 있으면 부위 간 cx 가 거의 같아 부위 전환은 사실상 틸트만 움직이고,
    누워 있으면 팬이 움직인다.

    좌표는 이번 프레임에 실제로 잡힌 부위(fresh)만 쓴다. 헤드가 움직이는 동안
    화면 좌표는 매 프레임 바뀌므로, 몇 프레임 전 좌표(스무딩 필터가 유예로 들고
    있는 값)는 이미 다른 자리를 가리킨다.

    ── 보이지 않는 부위 ─────────────────────────────────────────────────────
    카메라는 헤드 위에 있어 하체를 겨눌 땐 머리가, 머리를 겨눌 땐 하체가 화면
    밖으로 나가기 쉽다. 겨눌 부위가 안 보이면 보이는 부위에서 **몸 축 벡터**로
    위치를 추정해 그쪽으로 간다:

        부위 위치 = 보이는 부위 + (k[부위] - k[보이는 부위]) × 축 벡터
        k = {머리 0, 상체 1, 하체 1 + lower_ratio}
        축 벡터 = 상체 - 머리 (둘 다 보일 때 측정, EMA)

    벡터라서 방향이 자세를 따라간다 — 누우면 가로가 된다. 가까이 가 실제로
    잡히면 다음 프레임부터 실측 좌표로 바뀌므로 추정은 방향만 맞으면 된다.
    lower_ratio(상체→하체 / 머리→상체 길이비)는 vision/pose_estimate.py 의
    부위 정의(상체 = 어깨 중점 + 0.4 × 머리→어깨, 하체 = 무릎·발목 평균)로
    성인 체형에서 약 4다. 화면 밖으로 크게 넘치는 추정은 어차피 틸트 리밋에
    잘리므로 보수적으로 3을 쓴다.

    ── 부위 순환 ────────────────────────────────────────────────────────────
    세기가 1 이상인 부위를 PATROL_ORDER(head → upper → lower → upper) 순서로
    돈다. 한 부위의 슬롯은

        이동: 부위가 조준점 ± arrive_deg 안에 들어오거나 travel_max_s 가 지나면 끝
        체류: 그때부터 dwell_s

    다. 도착 판정 문턱(arrive_deg)은 데드존이 남기는 정지 오차(데드존/gain,
    팬 3.3° · 틸트 1.25°)보다 넉넉히 크게 두고, 그래도 사람이 계속 움직여
    도착이 안 되면 travel_max_s 로 끊는다 — 예전 "도착해야 진행"이 데드존
    정지 오차에 걸려 한 부위에 갇혔던 문제(실기 2026-08-28)를 피한다.
    이동 시간을 모터 속도로 추정하지 않으므로 기어비가 바뀌어도 손댈 게 없다.

    ── 대상 상실 ────────────────────────────────────────────────────────────
    잡힌 부위가 하나도 없으면 명령을 내지 않는다(None → 그 자리 유지, 타겟
    모드와 같다). lost_s 이상 이어지면 상실로 보고, 마지막으로 머리·상체를
    겨누던 각도(home)로 한 번 돌아가 기다린다 — 하체를 겨누던 중 놓치면
    카메라가 바닥을 본 채 멈춰 다시는 사람을 못 잡기 때문이다. 머리·상체를
    겨누다 놓쳤다면 home ≈ 지금 자리라 사실상 그대로 대기다.

    allowed / levels 는 러너가 매 프레임 앱 설정(부위별 세기)으로 갱신한다.
    모든 틸트 목표는 [tilt_min, tilt_max]로 clamp한다 (config.py "limits" —
    상한 +15°는 실측된 기구 파손 한계다).

    step()은 상태 변화 로그를 self.events(문자열 리스트)에 채운다 — 출력은
    호출부 몫.
    """

    REGIONS = ("head", "upper", "lower")
    PATROL_ORDER = ("head", "upper", "lower", "upper")

    def __init__(
        self,
        fov_h_deg: float,
        fov_v_deg: float,
        pan_min: float, pan_max: float,
        tilt_min: float, tilt_max: float,
        gain_pan: float,
        gain_tilt: float,
        target_cx: float,
        target_cy: float,
        invert_pan: bool = False,
        invert_tilt: bool = False,
        dwell_s: float = 2.0,
        arrive_deg: float = 5.0,
        travel_max_s: float = 3.0,
        lost_s: float = 1.0,
        lower_ratio: float = 3.0,
        default_axis: tuple[float, float] = (0.0, 0.15),
        axis_alpha: float = 0.3,
    ) -> None:
        self.fov_h, self.fov_v = fov_h_deg, fov_v_deg
        self.pan_min, self.pan_max = pan_min, pan_max
        self.tilt_min, self.tilt_max = tilt_min, tilt_max
        self.gain_pan, self.gain_tilt = gain_pan, gain_tilt
        self.sign_pan = -1.0 if invert_pan else 1.0
        self.sign_tilt = -1.0 if invert_tilt else 1.0
        self.target_cx, self.target_cy = target_cx, target_cy
        self.dwell_s = dwell_s
        self.arrive_deg = arrive_deg
        self.travel_max_s = travel_max_s
        self.lost_s = lost_s
        self._k = {"head": 0.0, "upper": 1.0, "lower": 1.0 + lower_ratio}
        self.axis_alpha = axis_alpha
        # 머리→상체 벡터 (정규화 좌표). 처음엔 "화면 아래로, 2m 기준 대략" 값.
        self.axis = default_axis

        # 러너가 매 프레임 갱신하는 값들.
        self.allowed: set[str] = set(self.REGIONS)
        self.levels: dict[str, int] = {}

        self.state = "move"            # move | dwell | lost
        self.events: list[str] = []
        # 마지막 조준점 (오버레이용): {"cx", "cy", "estimated", "region"} 또는 None
        self.aim: dict | None = None

        self._i = 0
        self._route_prev: list[str] | None = None
        self._slot_start: float | None = None
        self._dwell_since: float | None = None
        self._miss_since: float | None = None
        self._home: tuple[float, float] | None = None

    # ── 경로 ────────────────────────────────────────────────────────────────

    def _route(self) -> list[str]:
        """켜진 부위만 PATROL_ORDER 순서로. 전부 꺼져 있으면 상체에 머문다
        (바람은 0 — 조준만 유지)."""
        on = [r for r in self.PATROL_ORDER if r in self.allowed]
        route = [r for i, r in enumerate(on) if i == 0 or r != on[i - 1]]
        if len(route) > 1 and route[0] == route[-1]:
            route.pop()
        return route or ["upper"]

    def active_region(self) -> str:
        """지금 겨누는 부위. 대상 상실 중에는 '-'."""
        if self.state == "lost":
            return "-"
        route = self._route()
        return route[self._i % len(route)]

    def _sync_route(self) -> list[str]:
        """앱에서 세기를 바꿔 경로가 달라졌으면, 지금 부위가 남아 있을 때 이어간다."""
        route = self._route()
        if route != self._route_prev:
            cur = (self._route_prev[self._i % len(self._route_prev)]
                   if self._route_prev else None)
            if cur in route:
                self._i = route.index(cur)
            else:
                self._i %= len(route)
                self._new_slot()
            self._route_prev = route
        return route

    def _new_slot(self) -> None:
        self._slot_start = None
        self._dwell_since = None
        if self.state == "dwell":
            self.state = "move"

    # ── 좌표 ────────────────────────────────────────────────────────────────

    def _update_axis(self, pts: dict) -> None:
        """머리→상체 벡터를 이번 프레임 관측으로 갱신 (EMA)."""
        if "head" in pts and "upper" in pts:
            vx = pts["upper"][0] - pts["head"][0]
            vy = pts["upper"][1] - pts["head"][1]
        elif "upper" in pts and "lower" in pts:
            span = self._k["lower"] - self._k["upper"]
            vx = (pts["lower"][0] - pts["upper"][0]) / span
            vy = (pts["lower"][1] - pts["upper"][1]) / span
        else:
            return
        n = math.hypot(vx, vy)
        if not 0.02 <= n <= 0.5:      # 오검출 방어 (부위가 겹치거나 튐)
            return
        a = self.axis_alpha
        self.axis = (self.axis[0] + a * (vx - self.axis[0]),
                     self.axis[1] + a * (vy - self.axis[1]))

    def _locate(self, region: str, pts: dict) -> tuple[float, float, str] | None:
        """부위의 화면 좌표. 보이면 실측, 안 보이면 가장 가까운 보이는 부위에서
        축 벡터로 추정. 반환 (cx, cy, 출처 부위) — 출처가 자신이면 실측."""
        if region in pts:
            return pts[region][0], pts[region][1], region
        if not pts:
            return None
        src = min(pts, key=lambda r: abs(self._k[r] - self._k[region]))
        d = self._k[region] - self._k[src]
        return (pts[src][0] + d * self.axis[0],
                pts[src][1] + d * self.axis[1], src)

    def _errors(self, cx: float, cy: float) -> tuple[float, float]:
        ep = self.sign_pan * compute_pan_angle(cx - (self.target_cx - 0.5), self.fov_h)
        et = self.sign_tilt * compute_tilt_angle(cy - (self.target_cy - 0.5), self.fov_v)
        return ep, et

    # ── 메인 스텝 ────────────────────────────────────────────────────────────

    def step(self, obs: dict) -> tuple[float, float] | None:
        self.events = []
        t = obs["t"]
        cur_pan, cur_tilt = obs["pos"]
        route = self._sync_route()

        pts = {}
        if obs["target_idx"] is not None:
            pts = {r: (obs["regions"][r]["cx"], obs["regions"][r]["cy"])
                   for r in self.REGIONS
                   if obs["fresh"].get(r) and obs["regions"][r]["visible"]}

        # ── 대상 없음 ──
        if not pts:
            self.aim = None
            if self._miss_since is None:
                self._miss_since = t
            if self.state != "lost" and t - self._miss_since >= self.lost_s:
                self.state = "lost"
                self._new_slot()
                where = (f"pan {self._home[0]:+.1f}° tilt {self._home[1]:+.1f}°"
                         if self._home else "현재 자리")
                self.events.append(f"대상 상실 → {where}에서 대기")
            if self.state == "lost" and self._home is not None:
                return self._home
            return None

        self._miss_since = None
        if self.state == "lost":
            self.state = "move"
            self._new_slot()
            self.events.append(f"대상 재획득 → {route[self._i % len(route)]}부터 재개")
        self._update_axis(pts)

        region = route[self._i % len(route)]
        cx, cy, src = self._locate(region, pts)
        ep, et = self._errors(cx, cy)

        # ── 슬롯 (이동 → 체류 → 다음 부위) ──
        if self._slot_start is None:
            self._slot_start = t
        if self._dwell_since is None:
            arrived = abs(ep) <= self.arrive_deg and abs(et) <= self.arrive_deg
            if arrived or t - self._slot_start >= self.travel_max_s:
                self._dwell_since = t
                self.state = "dwell"
        if (len(route) > 1 and self._dwell_since is not None
                and t - self._dwell_since >= self.dwell_s):
            self._i = (self._i + 1) % len(route)
            self._new_slot()
            self._slot_start = t
            region = route[self._i]
            cx, cy, src = self._locate(region, pts)
            ep, et = self._errors(cx, cy)
            self.events.append(
                f"[aim] {region}{'' if src == region else f' (추정←{src})'} "
                f"오차 pan{ep:+.1f}° tilt{et:+.1f}°")

        # 상체의 cx 는 어깨 중점 기반이라, 한쪽 어깨만 잡힌 프레임에선 어깨폭
        # 절반만큼 치우친다 (app/tracking.py chest_point 참고). 그 프레임은 팬을
        # 움직이지 않는다 — 상체를 직접 겨누든 상체에서 추정하든 같다.
        if src == "upper" and not obs["chest"]["paired"]:
            ep = 0.0

        self.aim = {"cx": cx, "cy": cy, "estimated": src != region, "region": region}
        pan_t = clamp_angle(cur_pan + self.gain_pan * ep, self.pan_min, self.pan_max)
        tilt_t = clamp_angle(cur_tilt + self.gain_tilt * et, self.tilt_min, self.tilt_max)
        if region in ("head", "upper") and src == region:
            self._home = (pan_t, tilt_t)
        return pan_t, tilt_t


def region_wind_level(scenario: RegionPatrolScenario, levels: dict[str, int],
                      common_level: int = 0) -> int:
    """지금 겨누는 부위의 세기, 겨누는 부위가 없으면(대상 상실) common_level.

    상실 중 공용 세기를 쓰는 건 예전 탐색 상태와 같은 규칙이다
    (실기 2026-08-27: 상태마다 다른 세기를 쓰면 풍속이 튄다).
    """
    region = scenario.active_region()
    if region in levels:
        return levels[region]
    return common_level
