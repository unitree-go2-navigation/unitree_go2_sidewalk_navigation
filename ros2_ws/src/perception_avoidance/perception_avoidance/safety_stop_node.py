#!/usr/bin/env python3
"""
Safety Stop Node (Phase 1, cmd_vel safety gate).

This node is a FILTER, not a planner:
  /cmd_vel_in  →  [filter by clearance/TTC]  →  /cmd_vel_safety

States:
  NOMINAL   : pass-through
  SLOW_DOWN : scale cmd_vel proportional to clearance
  STOP      : zero cmd_vel
  WAIT      : zero cmd_vel until front is clear for wait_clear_duration
  RESUME    : ramp cmd_vel back up after a stop
  STUCK     : commanded to move but not moving → active in-place rotation recovery
  ESTOP     : latched zero (repeated stuck) until node restart
Plus sensor timeout → STOP.

Note: STUCK is the one state where the node actively commands motion (a bounded
in-place rotation toward the clearer side) rather than just filtering — a
deliberate, time-bounded exception. Recovery is rotation-only (no blind reverse;
no rear perception).
"""

import math
from collections import deque
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from .social_nav import (
    GapRules, blockers_from_detections, classify_gap, classify_kind,
    compute_gaps, lateral_band)

# Phase 5a 소셜 레이어 내부 상수 (설계검토 확정치 — 파라미터화는 4단계
# 튜닝에서 필요해질 때만).
SOCIAL_POLY_TOPIC = '/perception/sidewalk/boundary'
SOCIAL_X_MIN = -0.5        # blocker 유지 후방 한계 (통과 완료 판정은 고스트가 담당)
SOCIAL_BAND_X = 3.0        # band 평가 전방 한계 — 요 sway의 지렛대 오차 축소
BLOCKER_INFLATE = 0.05     # 클러스터 edge 과소추정 보상 (gap 폭 이중계상 주의)
PERSON_UP_FRAMES = 3       # 이동 승격 지속 (채터링 방어: 승격 느리게)
PERSON_DOWN_FRAMES = 5     # 이동 강등 지속 (강등은 더 느리게)
GHOST_MAX_AGE = 4.0        # 미관측 blocker 유지 상한 (s)
POLY_TIMEOUT = 0.5         # polygon 신선도 — 초과 시 소셜 비활성 (계약: fallback)
SOCIAL_STALE = 0.3         # _social 계산 신선도 — 초과 시 vy 주입 중단
LAT_DEADBAND = 0.05        # 목표 근접 시 vy 0 (미세 진동 방지)
VY_LP_ALPHA = 0.1          # 자기운동 저역통과 (gait sway ~2.5Hz 제거)
GROUND_EMA_ALPHA = 0.4     # track별 지상 속도 평활 (sway·EMA 지연 불일치 흡수)
# 이동 승격의 상대속도 하한: 가림 경계 슬라이딩 청크(울타리·연석 파편)는
# 로봇과 나란히 이동해 상대속도 ≈0이면서 지상속도 ≈로봇속도 — 지상속도만
# 보면 유사 이동체다 (프로브 실측: 킬스위치 영구 래치). 진짜 보행자는
# 정면 1.6/횡단 1.2 등 상대속도가 크다. 정확히 나란히 걷는 사람만 미탐인데
# 그 경우 '사이 통과 위험' 자체가 없어 킬스위치 불필요 — 안전 방향.
SOCIAL_REL_MIN = 0.25
# gap 통과 판정 이탈 히스테리시스: 진입은 정규 요구폭, 유지(직전 활성 측)는
# −0.15까지 허용 — 밴드 3.0m − 사람 클러스터 ~0.9m면 양측 gap이 요구 1.0의
# 칼끝이라 폭 노이즈 ±0.1로 통과/불가 플리커 (프로브 실측 nogap 744프레임).
GAP_EXIT_HYST = 0.15
SOCIAL_HOLD_T = 3.0        # 측 래치·완화 유지 창 — 실측 nogap 스트레치 ~2.5s 커버
# blocker 최소 크기: 두 변 모두 이하인 초소형 파편(연석 조각·debris 반사)은
# gap 계산에서 제외 — 속도·폭 추정이 전부 노이즈. 실물 소형 장애물의 충돌
# 안전은 코리도 게이트가 계속 담당하고, 지면 장애물 대응은 Phase 6 몫.
BLOCKER_MIN_DIM = 0.25
# blind-hold 지상 vy 보정 하한: LP 잔여 sway(α=0.1이 2.5Hz를 ~0.32로만
# 감쇠 → ±0.03~0.06)가 hold의 0.05 문턱(승계·예측 이탈)을 뒤집지 않도록
# 명령성 크랩워크(LP ~0.17+)에서만 보정 발동 — social OFF 경로 결정적 불변
# (코드리뷰 I1: 무하한 보정은 P3 crossing 승계·head_on 영구유지에 간섭).
VY_COMP_MIN = 0.1
BAND_EMA_ALPHA = 0.4       # band 경계 평활 (요 sway 에지 진동 완화)


class State(Enum):
    NOMINAL = 'NOMINAL'
    SLOW_DOWN = 'SLOW_DOWN'
    STOP = 'STOP'
    WAIT = 'WAIT'
    RESUME = 'RESUME'
    STUCK = 'STUCK'
    ESTOP = 'ESTOP'


def cpa(cx, cy, vx, vy):
    """상대 위치/속도 → (t_cpa, d_cpa): 최근접점까지 시간·미스거리 (P4).

    t_cpa = argmin|p + v·t| = -(p·v)/|v|² (과거면 0으로 클램프 = 이미 통과/
    멀어짐). 속도 ~0이면 (inf, 현재거리) — 정지 물체는 거리/코리도 로직 몫.
    """
    v2 = vx * vx + vy * vy
    if v2 < 1e-6:
        return math.inf, math.hypot(cx, cy)
    t = max(0.0, -(cx * vx + cy * vy) / v2)
    return t, math.hypot(cx + vx * t, cy + vy * t)


class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('safety_stop_node')

        # Topics
        self.declare_parameter('cmd_vel_in_topic', '/cmd_vel_in')
        self.declare_parameter('cmd_vel_out_topic', '/cmd_vel_safety')
        self.declare_parameter('obstacles_topic', '/obstacles/lidar')
        self.declare_parameter('odom_topic', '/odom')

        # Geometry — rectangular footprint matching Go2 (~0.70 × 0.31 m →
        # half-extents 0.35 × 0.155, ratio ~0.44).
        #   robot_half_length: forward/rear body extent (incl. leg reach). Used
        #     for longitudinal clearance: clr = front_face_x − half_length.
        #   robot_half_width: lateral extent (URDF-derived; leg_offset_y 0.0465
        #     + thigh_offset 0.0955 = 0.142m, 0.155 covers the hip housing).
        #   corridor_margin: lateral uncertainty budget (gait sway + localization
        #     + LiDAR cluster), added ONLY to the lateral corridor — NOT geometry.
        #     The longitudinal safety gap is emergency_clearance, not this.
        self.declare_parameter('robot_half_length', 0.35)
        self.declare_parameter('robot_half_width', 0.155)
        self.declare_parameter('corridor_margin', 0.18)

        # Zones
        self.declare_parameter('slow_clearance', 3.0)
        self.declare_parameter('slow_ttc', 3.0)
        # Unconditional STOP backstop: a distance-only STOP fires only inside
        # this small margin. Larger-distance reaction is TTC-driven (only when
        # actually approaching), so a static obstacle the robot merely rotates
        # past (closing≈0 → TTC=inf) does not force STOP.
        # emergency + robot_half_length는 lidar min range(0.8m)보다 여유 있게
        # 커야 한다 — 결정 경계가 센서 사각지대에 겹치면 최근접점 소실로
        # STOP을 놓치고 관통한다 (safety_stop.yaml 주석 참조)
        self.declare_parameter('emergency_clearance', 0.60)
        # SLOW 스케일 영점을 emergency보다 이만큼 아래로 → creep이 STOP 문턱을
        # 관통해 래치됨 (영점==문턱이면 점근 접근으로 STOP이 영원히 안 걸림)
        self.declare_parameter('creep_overshoot', 0.10)
        # SLOW 전진 명령 하한 (m/s). CHAMP가 걷지 못하는 초저속(≲0.12)을
        # 명령하면 전진 없이 STUCK 오인만 유발 — 하한 밑으로 줄여야 할 상황의
        # 최종 정지는 emergency_clearance 거리 백스톱(STOP)이 담당한다.
        self.declare_parameter('creep_v_min', 0.12)
        self.declare_parameter('stop_ttc', 1.0)
        # 사각 소실 유지(blind-hold): 이보다 가까이 보이던 코리도 물체가 어떤
        # 탐지로도 이어지지 않고 사라지면 lidar min range 사각 진입으로 간주,
        # 마지막 관측 clearance를 유지한다. 없으면 WAIT가 "클리어"로 풀려
        # RESUME이 사각을 향해 재진입→관통 (2026-07-13 head_on 충돌 2건:
        # 초저속(<closing_eps) 접근 보행자가 사각 경계에서 깜빡임). 옆으로
        # 비켜난 경우(코리도 밖 인근에서 계속 탐지)는 해제.
        # min_range 0.8 − half_length 0.35 + margin 0.15 = 0.60.
        self.declare_parameter('blind_hold_clearance', 0.60)
        # Min closing speed (m/s) to treat an obstacle as approaching
        self.declare_parameter('closing_eps', 0.05)

        # P4 fast-class (자전거/킥보드, |v_rel|>2m/s): 코리도 점유와 무관하게
        # CPA(최근접점) 미스거리로 "진로로 오는" 물체만 조기 반응.
        #   트리거: d_CPA < cpa_halfwidth + cpa_margin + cpa_growth·t_CPA
        #           AND t_CPA < cpa_max_t   (Fiorini VO/CPA + CVM 반경 팽창)
        #   반응: t_CPA ≤ fast_stop_ttc → STOP / ≤ fast_slow_ttc → SLOW
        #   (ISO 22839 AEB 관행 — 고속 물체엔 회피가 아니라 조기 정지)
        self.declare_parameter('fast_speed_thresh', 2.0)
        self.declare_parameter('fast_stop_ttc', 2.0)
        self.declare_parameter('fast_slow_ttc', 4.0)
        self.declare_parameter('cpa_max_t', 4.0)
        # Go2 유효 반폭 (gait sway 포함 유효폭 0.5m의 절반)
        self.declare_parameter('cpa_halfwidth', 0.25)
        # 상대 물체 반경 여유 (m)
        self.declare_parameter('cpa_margin', 0.2)
        # 예측 불확실 팽창 (m/s) — CVM 반경 팽창 0.4m/s·t
        self.declare_parameter('cpa_growth', 0.4)
        # STOP/WAIT 중 fast 물체가 이 미스거리 안에서 아직 접근 중이면 재출발
        # 금지 — CPA 통과(멀어짐 전환)까지 STOP 유지. NOMINAL에는 미적용
        # (측방 통과 자전거에 대한 false stop 방지 — 주행 중 판단은 트리거 몫).
        self.declare_parameter('fast_hold_radius', 2.0)
        # latency 보상 상한 (s): 파이프라인 지연(수신 시각 − LiDAR 스캔 stamp,
        # 메시지별 실측)만큼 fast 물체를 CVM 전방 전파. 5m/s에서 지연 0.15s =
        # 위치 오차 0.75m — fast 경로에만 적용 (저속/정적은 기존 마진이 흡수,
        # 클러터의 노이즈 속도로 clearance를 오염시키지 않기 위함).
        self.declare_parameter('cvm_latency_max', 0.3)
        # fast 트리거 지속 조건 (감쇠 카운트 문턱, 같은 track id): 정적
        # 클러터(울타리 등 확장 물체)의 centroid 요동이 만드는 단발 속도
        # 스파이크(실측 최대 ~8m/s, 방향 무작위)가 유령 STOP을 만들지 않도록.
        # 진짜 자전거는 예측 연관으로 id가 유지되며 매 프레임 트리거 → 4프레임
        # (0.4s, 5m/s 기준 2.2m 접근) 확인 비용. 3에서는 가림 경계의 지속성
        # 유사속도가 ~2/10 run 빈도로 뚫음 (bike_pass false stop 실측).
        self.declare_parameter('fast_trig_frames', 4)
        # fast-class 크기 상한 (m): 자전거/킥보드/사람 부류는 compact —
        # bbox 한 변이 이보다 긴 확장 구조물(울타리/벽 파편)은 fast 후보에서
        # 제외 (가림 경계에서 한쪽으로 '자라는' 클러스터의 유사속도 차단).
        # GLB 자전거+탑승자 대각 투영 ~1.9m < 2.2.
        self.declare_parameter('fast_max_extent', 2.2)
        # Let rotation (angular.z) pass through while SLOW/STOP so the robot can
        # turn away from a blocking obstacle instead of deadlocking facing it.
        self.declare_parameter('pass_rotation_when_blocked', True)

        # Phase 5a 소셜 레이어 (기본 OFF — 판정 기준이 따라잡는 4단계에서
        # 시나리오와 함께 ON). ON이면 보도 polygon 안에서 gap을 골라 횡방향
        # (vy) 주입 + 사람 인접 통과 시 속도 캡. 대면·이동 보행자가 창 안에
        # 있으면 이번 사이클 비활성 (기존 코리도/fast 로직 몫 — 5b YIELD 예정).
        self.declare_parameter('social_enable', False)
        self.declare_parameter('social_lookahead', 5.0)   # blocker 전방 창 (m)
        self.declare_parameter('social_person_v', 0.3)    # 이동 판정 지상 속도 (m/s)
        self.declare_parameter('person_min_top', 0.5)     # 사람 분류 상단 z (base_link, 지상 ~0.8m)
        self.declare_parameter('lat_k', 0.8)              # 횡 비례 이득
        self.declare_parameter('lat_vy_max', 0.2)         # 횡 명령 상한 < gait max_y 0.25

        # Hysteresis + persistence
        self.declare_parameter('hysteresis_clearance', 0.3)
        self.declare_parameter('hysteresis_ttc', 0.5)
        self.declare_parameter('persistence_slow', 0.1)
        self.declare_parameter('persistence_stop', 0.05)

        # WAIT/RESUME
        self.declare_parameter('wait_clear_duration', 1.0)
        self.declare_parameter('resume_time', 1.5)

        # Timeout
        self.declare_parameter('sensor_timeout', 0.5)

        # Rate
        self.declare_parameter('control_rate', 50.0)

        # Gate enable. False = pure pass-through (baseline/teleop testing).
        self.declare_parameter('enable_gate', True)

        # Stuck detection & recovery (Phase 2.5). Detect "commanded to move but
        # not moving" (curb/leg-catch) and recover by in-place rotation toward
        # the clearer side. Rotation-only — no blind reverse (no rear sensing).
        self.declare_parameter('stuck_enable', True)
        self.declare_parameter('stuck_v_min', 0.05)      # output linear = "trying to move"
        self.declare_parameter('stuck_disp_min', 0.1)    # min net move over the window = progressing
        self.declare_parameter('stuck_duration', 2.0)    # window: no-progress must persist this long
        self.declare_parameter('recover_settle_time', 0.3)
        self.declare_parameter('recover_rotate_time', 1.2)
        self.declare_parameter('recover_yaw_rate', 0.5)  # rad/s
        self.declare_parameter('max_recover_attempts', 3)

        gp = self.get_parameter
        self.robot_half_length = gp('robot_half_length').value
        self.corridor_half = (
            gp('robot_half_width').value + gp('corridor_margin').value)
        self.slow_clr = gp('slow_clearance').value
        self.slow_ttc = gp('slow_ttc').value
        self.emergency_clr = gp('emergency_clearance').value
        self.creep_overshoot = gp('creep_overshoot').value
        self.creep_v_min = gp('creep_v_min').value
        self.stop_ttc = gp('stop_ttc').value
        self.blind_hold_clr = gp('blind_hold_clearance').value
        self.hyst_clr = gp('hysteresis_clearance').value
        self.hyst_ttc = gp('hysteresis_ttc').value
        self.persist_slow = gp('persistence_slow').value
        self.persist_stop = gp('persistence_stop').value
        self.wait_dur = gp('wait_clear_duration').value
        self.resume_time = gp('resume_time').value
        self.sensor_timeout = gp('sensor_timeout').value
        self.enable_gate = gp('enable_gate').value
        self.closing_eps = gp('closing_eps').value
        self.fast_speed = gp('fast_speed_thresh').value
        self.fast_stop_ttc = gp('fast_stop_ttc').value
        self.fast_slow_ttc = gp('fast_slow_ttc').value
        self.cpa_max_t = gp('cpa_max_t').value
        self.cpa_halfwidth = gp('cpa_halfwidth').value
        self.cpa_margin = gp('cpa_margin').value
        self.cpa_growth = gp('cpa_growth').value
        self.fast_hold_radius = gp('fast_hold_radius').value
        self.cvm_latency_max = gp('cvm_latency_max').value
        self.fast_trig_frames = int(gp('fast_trig_frames').value)
        self.fast_max_extent = gp('fast_max_extent').value
        self.pass_rotation = gp('pass_rotation_when_blocked').value
        self.social_enable = gp('social_enable').value
        self.social_lookahead = gp('social_lookahead').value
        self.social_person_v = gp('social_person_v').value
        self.person_min_top = gp('person_min_top').value
        self.lat_k = gp('lat_k').value
        self.lat_vy_max = gp('lat_vy_max').value
        self.stuck_enable = gp('stuck_enable').value
        self.stuck_v_min = gp('stuck_v_min').value
        self.stuck_disp_min = gp('stuck_disp_min').value
        self.stuck_duration = gp('stuck_duration').value
        self.recover_settle_time = gp('recover_settle_time').value
        self.recover_rotate_time = gp('recover_rotate_time').value
        self.recover_yaw_rate = gp('recover_yaw_rate').value
        self.max_recover_attempts = gp('max_recover_attempts').value
        period = 1.0 / float(gp('control_rate').value)

        # Sub
        self.create_subscription(
            Twist, gp('cmd_vel_in_topic').value, self.cmd_in_cb, 10)
        self.create_subscription(
            Detection3DArray, gp('obstacles_topic').value, self.obs_cb, 10)
        self.create_subscription(
            Odometry, gp('odom_topic').value, self.odom_cb, 10)
        if self.social_enable:
            self.create_subscription(
                PolygonStamped, SOCIAL_POLY_TOPIC, self.poly_cb, 10)

        # Pub
        self.cmd_out_pub = self.create_publisher(
            Twist, gp('cmd_vel_out_topic').value, 10)
        self.state_pub = self.create_publisher(
            String, '/safety/state', 10)

        # State
        self.cmd_in = Twist()
        self.cmd_in_stamp = None
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.obs_stamp = None
        self.min_clearance = math.inf
        self.min_ttc = math.inf
        self.min_fast_tcpa = math.inf   # fast-class CPA 트리거 중 최소 t_CPA
        self._fast_near = False         # fast 물체가 근접 CPA로 아직 접근 중
        self._fast_trig = {}            # track id → (트리거 카운트, 연속 미관측 수)
        self._blind_hold = None      # (front_x, clr) — 사각 소실 유지 중
        self._left_min = math.inf
        self._right_min = math.inf
        # Phase 5a 소셜 레이어 상태 (social_enable=False면 전부 불변 유지)
        self.robot_vy = 0.0
        self._vy_lp = 0.0            # 저역통과 자기운동 (sway 제거)
        self._vx_lp = 0.0
        self._wz_lp = 0.0
        self._poly = None            # ([(x,y)...], 수신 시각)
        self._band = None            # EMA 평활된 (y_lo, y_hi)
        self._social = None          # (target_y, speed_cap|None, 계산 시각)
        self._social_reason = ''     # 비활성 사유 (디버그: band/moving/noblock/nogap)
        self._social_mem = {}        # track id → 관측/고스트 blocker 상태
        self._person_hist = {}       # track id → 이동 이력 (streak/quiet/ever/moving)
        self._social_side = 0.0      # 선택 gap 측 래치 (+1 좌 / -1 우 / 0 없음)
        self._social_last_t = None   # 마지막 활성 시각 (측 래치·완화 유지 창)
        self._last_social_t = None   # 고스트 후방 전파용 직전 obs 시각
        self._social_rules = GapRules(
            robot_width=2.0 * gp('robot_half_width').value)
        r = self._social_rules
        self._social_rules_relaxed = GapRules(
            robot_width=r.robot_width,
            attempt_min=r.attempt_min - GAP_EXIT_HYST,
            default_min=r.default_min - GAP_EXIT_HYST,
            moving_min=r.moving_min - GAP_EXIT_HYST)
        self.state = State.NOMINAL
        self._t_in_slow_cond = 0.0
        self._t_in_stop_cond = 0.0
        self._t_in_clear = 0.0
        self._resume_scale = 0.0
        self._cmd_out_speed = 0.0
        self._stall_timer = 0.0
        self._pose_hist = deque()
        self._recover_attempts = 0
        self._recover_start = 0.0
        self._recover_dir = 1.0
        self._last_tick = self.get_clock().now().nanoseconds * 1e-9

        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(
            f'safety_stop_node started. gate: {gp("cmd_vel_in_topic").value} → '
            f'{gp("cmd_vel_out_topic").value} | footprint '
            f'(L/2={self.robot_half_length:.2f}, W/2={gp("robot_half_width").value}) '
            f'| corridor_half={self.corridor_half:.3f}m '
            f'(half_width + margin {gp("corridor_margin").value})'
        )

    def cmd_in_cb(self, msg: Twist):
        self.cmd_in = msg
        self.cmd_in_stamp = self.get_clock().now().nanoseconds * 1e-9

    def odom_cb(self, msg: Odometry):
        self.robot_vx = msg.twist.twist.linear.x
        self.robot_wz = msg.twist.twist.angular.z
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # 자기운동 저역통과: gait sway(~2.5Hz, vx·vy ±0.2, wz ±0.2)가 실리므로
        # 명령성 성분만 추출 — blind-hold 지상 vy 보정과 소셜 지상 속도 판정에
        # 사용 (즉시값이 그대로 들어가면 정지물이 0.3 문턱을 넘나들어 보행
        # 시작 순간 소셜 레이어 자기 비활성 — 프로브 실측 2026-07-22).
        self.robot_vy = msg.twist.twist.linear.y
        self._vy_lp += VY_LP_ALPHA * (self.robot_vy - self._vy_lp)
        self._vx_lp += VY_LP_ALPHA * (self.robot_vx - self._vx_lp)
        self._wz_lp += VY_LP_ALPHA * (self.robot_wz - self._wz_lp)

    def poly_cb(self, msg: PolygonStamped):
        self._poly = ([(p.x, p.y) for p in msg.polygon.points],
                      self.get_clock().now().nanoseconds * 1e-9)

    def obs_cb(self, msg: Detection3DArray):
        self.obs_stamp = self.get_clock().now().nanoseconds * 1e-9
        # 파이프라인 latency 실측 (LiDAR 스캔 stamp → 게이트 수신).
        # fast-class CPA 판정에서 물체를 이만큼 CVM 전방 전파한다.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lat = min(max(self.obs_stamp - stamp, 0.0), self.cvm_latency_max)
        min_clr = math.inf
        min_ttc = math.inf
        min_fast_tcpa = math.inf
        fast_near = False
        fast_trig = {}
        left_min = math.inf
        right_min = math.inf
        min_hold = None
        social_dets = []
        for det in msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            hx = 0.5 * det.bbox.size.x
            xmin, xmax = cx - hx, cx + hx

            # Must have some extent ahead of the robot
            if xmax <= 0.0:
                continue
            # Nearest obstacle on each side (picks stuck-recovery rotation dir).
            front_face = max(xmin, 0.0)
            if cy >= 0.0:
                left_min = min(left_min, front_face)
            else:
                right_min = min(right_min, front_face)

            # Relative velocity (base_link) embedded by lidar_obstacle_node in
            # covariance[0]=vx, covariance[1]=vy — travels atomically with the
            # detection, so there is no cross-topic ordering race.
            vx = vy = 0.0
            if det.results:
                cov = det.results[0].pose.covariance
                vx, vy = cov[0], cov[1]
            pdist = math.hypot(cx, cy)

            # P4 fast-class CPA: 코리도 점유와 무관하게 (측방에서 진로로
            # 진입하는 자전거 포함) "진로로 오는" 고속 물체만 조기 반응.
            # 정적 클러터는 |v_rel|≈로봇 속도(≤0.5)라 임계(2.0)에 안 걸림.
            # ⚠ 회전 스윕 보정: base_link는 회전 프레임이라 로봇이 돌면
            # (조향/위빙) 정적 물체가 ω×r로 쓸려 보임 — r=5m·0.5rad/s면
            # 2.5m/s 유령 fast (데모 위빙에서 STOP 연발 실측). 정적 물체의
            # 회전 성분 v_sweep = (ω·y, -ω·x)를 제거한 병진 상대속도로 판정.
            vxc = vx - self.robot_wz * cy
            vyc = vy + self.robot_wz * cx
            if self.social_enable:
                # 소셜은 원시 상대속도를 넘김 — 지상 속도 보상은
                # _update_social에서 저역통과 자기운동으로 수행 (즉시 wz
                # 보정은 sway 노이즈 ±wz·cx가 문턱 0.3을 압도).
                social_dets.append((
                    det.id, cx, cy, det.bbox.size.x, det.bbox.size.y,
                    det.bbox.center.position.z + 0.5 * det.bbox.size.z,
                    vx, vy))
            if (math.hypot(vxc, vyc) > self.fast_speed
                    and det.bbox.size.x <= self.fast_max_extent
                    and det.bbox.size.y <= self.fast_max_extent):
                # latency 보상: 관측 시점 이후 이동분(v·lat)을 전방 전파
                fx, fy = cx + vxc * lat, cy + vyc * lat
                t_cpa, d_cpa = cpa(fx, fy, vxc, vyc)
                if t_cpa < self.cpa_max_t:
                    if d_cpa < (self.cpa_halfwidth + self.cpa_margin
                                + self.cpa_growth * t_cpa):
                        # 지속 조건: 같은 track의 트리거 카운트(+1/미트리거
                        # −1 감쇠, 루프 뒤 처리)가 문턱에 도달해야 반응 —
                        # 경계 노이즈의 교대 트리거는 1~2에 머물러 발화 불가.
                        cnt = min(self._fast_trig.get(det.id, 0) + 1,
                                  self.fast_trig_frames)
                        fast_trig[det.id] = cnt
                        if cnt >= self.fast_trig_frames:
                            min_fast_tcpa = min(min_fast_tcpa, t_cpa)
                    fdist = math.hypot(fx, fy)
                    closing_f = 0.0
                    if fdist > 1e-3:
                        closing_f = -(fx * vxc + fy * vyc) / fdist
                    if (closing_f > self.closing_eps
                            and d_cpa < self.fast_hold_radius):
                        fast_near = True   # STOP/WAIT 중 재출발 금지 근거

            # Forward corridor: 판정 근거는 bbox 겹침이 아니라 클러스터 점 중
            # 코리도 안에 실제로 있는 최근접점 x (covariance[2], 없으면 -1).
            # 긴 평행 구조물(인도변 울타리)은 요 오차 시 axis-aligned bbox가
            # 차선 쪽으로 번져 bbox 판정을 오염시킴 (empty 0/5 원인, 2026-07-10).
            corridor_front_x = -1.0
            if det.results:
                corridor_front_x = det.results[0].pose.covariance[2]
            if corridor_front_x < 0.0:
                continue    # 코리도 내 점 없음 → 차선 비차단

            # Longitudinal clearance: distance from the nearest in-corridor
            # point to the robot's front body edge (footprint half-length).
            clr = corridor_front_x - self.robot_half_length

            #   closing = -dot(p_rel, v_rel) / |p_rel|   (positive = approaching)
            # Clamp to the static robot-speed estimate so we never under-react,
            # and a zero/missing velocity degrades gracefully to the static case.
            if clr < min_clr:
                min_clr = clr
                # blind-hold용: 최근접 물체의 track id·횡위치·횡속도 기억.
                # vy는 지상 기준으로 보정(+자기 횡속도 LP, 하한 VY_COMP_MIN)
                # — 소셜 크랩워크가 정지 보행자를 '이동'으로 오염시키면 예측
                # 이탈 ③이 영구 유지 불변식을 깨고 RESUME 사각 관통을 재발
                # 시킨다 (설계검토 FSM-1). 하한 미만(sway 잔여)은 무보정 =
                # 소셜 OFF에서 기존 거동과 동일 (코드리뷰 I1).
                comp = self._vy_lp if abs(self._vy_lp) >= VY_COMP_MIN else 0.0
                min_hold = {'clr': clr, 'id': det.id, 'cy': cy,
                            'vy': vy + comp}
            closing = 0.0
            if pdist > 1e-3:
                closing = -(cx * vx + cy * vy) / pdist
            closing = max(closing, self.robot_vx)
            if clr <= 0.0:
                ttc = 0.0
            elif closing > self.closing_eps:
                ttc = clr / closing
            else:
                ttc = math.inf
            if ttc < min_ttc:
                min_ttc = ttc
        # 사각 소실 유지(blind-hold): 근접 코리도 물체가 어떤 탐지로도 이어지지
        # 않고 사라짐 → 사각 진입으로 간주, 마지막 관측 clearance 유지 (in_stop
        # 지속 → WAIT 해제 차단). 해제 = "위협 소멸의 적극적 증거"가 있을 때만:
        #   ① 원거리 재관측 (clr > emergency+hyst — 깜빡임 대역 0.6~0.9의 실측
        #      한 프레임으로 풀면 다음 소실 프레임에 RESUME 관통 재발, trial2)
        #   ② 같은 track id가 코리도 밴드 밖으로 명백히 이탈 (정면에 걸친
        #      실루엣 잔여 클러스터로 풀면 관통 재발, trial3 — id로 동일성 보장,
        #      울타리 등 다른 물체의 측면 탐지로는 풀리지 않음)
        #   ③ 예측 이탈(CVM): 마지막 횡속도가 코리도 이탈을 예측하면 예측
        #      시각+1s 후 해제 — track 단절로 ②가 못 잡는 횡단 보행자의 동결
        #      방지(liveness). 횡속도 없는 물체(정면/정지)는 영구 유지(safety).
        if min_clr < math.inf:
            if min_clr <= self.blind_hold_clr:
                min_hold['t'] = self.obs_stamp
                # 횡속도 기억 승계: track 단절 직후(새 track은 vy=0에서 EMA
                # 재수렴)나 노이즈 프레임의 약한 추정이 마지막 갱신에 걸리면
                # 예측 이탈(③)이 죽어 동결됨 (crossing 3/5 freeze 원인).
                prev = self._blind_hold
                if (prev is not None and abs(min_hold['vy']) < 0.05
                        and abs(prev['vy']) >= 0.05):
                    min_hold['vy'] = prev['vy']
                self._blind_hold = min_hold
            elif min_clr > self.emergency_clr + self.hyst_clr:
                self._blind_hold = None
        elif self._blind_hold is not None:
            hold = self._blind_hold
            aside = any(
                d.id == hold['id']
                and (abs(d.bbox.center.position.y) - 0.5 * d.bbox.size.y
                     > self.corridor_half + 0.2)
                for d in msg.detections)
            t_exit = math.inf
            if abs(hold['vy']) > 0.05:
                t_exit = max(0.0, (self.corridor_half + 0.3 - abs(hold['cy']))
                             / abs(hold['vy']))
            if aside or (self.obs_stamp - hold['t']) > t_exit + 1.0:
                self._blind_hold = None
            else:
                min_clr = hold['clr']
        # 감쇠: 이번 프레임에 트리거하지 않은 track은 −1 (0이면 제거).
        # ⚠ "미관측이면 카운트 유지" 변형은 금지 — 유령 fast 카운트가
        # 깜빡임 너머로 존속해 bike_pass false stop 재발 (9회 정지 실측).
        # 열화의 확정 창 부족은 criteria_degraded + 안무(veer 지연)가 담당.
        for tid, cnt in self._fast_trig.items():
            if tid not in fast_trig and cnt > 1:
                fast_trig[tid] = cnt - 1
        self.min_clearance = min_clr
        self.min_ttc = min_ttc
        self.min_fast_tcpa = min_fast_tcpa
        self._fast_near = fast_near
        self._fast_trig = fast_trig
        self._left_min = left_min
        self._right_min = right_min
        if self.social_enable:
            self._update_social(social_dets, self.obs_stamp)

    def _update_social(self, dets, now):
        """Phase 5a: track 이동 이력 → kind 분류 → 고스트 유지 → gap 선택.

        결과는 self._social = (target_y, speed_cap|None, t) 또는 None.
        비활성 조건: polygon 미수신/stale, blocker 없음, 이동 보행자 존재
        (대면·횡단은 코리도/fast 로직 몫 — 5b YIELD 예정), 통과 가능 gap 없음.
        """
        dt = 0.0 if self._last_social_t is None else max(
            0.0, now - self._last_social_t)
        self._last_social_t = now

        seen = set()
        for tid, cx, cy, sx, sy, top, vx, vy in dets:
            seen.add(tid)
            # 지상 속도: 상대속도 + 저역통과 자기운동(병진·회전) 보상 후
            # track별 EMA. 미보상이면 주행 중 정지물 전부가 로봇 속도만큼
            # '이동'으로 보이고 (설계검토), 즉시값 보상이면 sway·트래커 EMA
            # 지연 불일치(±0.2 + wz·cx)가 문턱을 넘나든다 (프로브 실측).
            gx = vx - self._wz_lp * cy + self._vx_lp
            gy = vy + self._wz_lp * cx + self._vy_lp
            g_speed = math.hypot(gx, gy)
            rel_speed = math.hypot(vx, vy)
            h = self._person_hist.setdefault(
                tid, {'streak': 0, 'quiet': 0, 'ever': False, 'moving': False})
            if 'g' in h:
                h['g'] += GROUND_EMA_ALPHA * (g_speed - h['g'])
            else:
                h['g'] = g_speed
            if h['g'] > self.social_person_v and rel_speed > SOCIAL_REL_MIN:
                h['streak'] += 1
                h['quiet'] = 0
            else:
                h['streak'] = 0
                h['quiet'] += 1
            # 승격 3프레임 / 강등 5프레임 — 경계 속도 보행자의 레이어
            # on/off 채터링 방어 (승격은 보수 방향이라 더 빠르게).
            if h['streak'] >= PERSON_UP_FRAMES:
                h['moving'] = True
                h['ever'] = True
            elif h['moving'] and h['quiet'] >= PERSON_DOWN_FRAMES:
                h['moving'] = False
            compact = (sx <= self.fast_max_extent
                       and sy <= self.fast_max_extent)
            kind = classify_kind(h['moving'], h['ever'],
                                 top >= self.person_min_top, compact)
            self._social_mem[tid] = {'x': cx, 'y': cy, 'sx': sx, 'sy': sy,
                                     'kind': kind, 't': now}

        # 미관측 track: 고스트 유지 + 로봇 이동량만큼 후방 전파 — 측면 통과
        # 중 ROI(x<0.5)/min range(0.8) 사각에서 사람이 소실되면 gap이 그쪽으로
        # 재타겟해 커트인 충돌 (설계검토 FSM-2). 완전히 지나가거나 수명 초과
        # 시 제거.
        for tid in list(self._social_mem):
            m = self._social_mem[tid]
            if tid in seen:
                continue
            m['x'] -= self._vx_lp * dt
            m['y'] -= self._vy_lp * dt
            if (now - m['t'] > GHOST_MAX_AGE
                    or m['x'] + 0.5 * m['sx']
                    < -(self.robot_half_length + 0.3)):
                del self._social_mem[tid]
                self._person_hist.pop(tid, None)

        band = None
        if self._poly is not None and now - self._poly[1] <= POLY_TIMEOUT:
            band = lateral_band(self._poly[0], 0.0, SOCIAL_BAND_X)
        # sanity 가드: 밴드는 로봇(y=0)을 포함해야 한다 — 아니면 polygon이
        # 좌표계 오류/드리프트로 깨진 것 (스폰 이중 계상 사고 실측, 2026-07-22).
        # 깨진 밴드로 gap을 고르면 보도 밖 목표로 조향하므로 비활성이 안전.
        if band is not None and not (band[0] < 0.0 < band[1]):
            band = None
        if band is None:
            self._social = None
            self._social_reason = 'band'
            self._band = None
            self._social_side = 0.0
            return
        if self._band is None:
            self._band = band
        else:
            a = BAND_EMA_ALPHA
            self._band = (self._band[0] + a * (band[0] - self._band[0]),
                          self._band[1] + a * (band[1] - self._band[1]))

        # 이동 보행자 베토는 두지 않는다 — '창 안 mover 존재 시 레이어 거부'
        # 설계는 초소형 파편·슬라이딩 청크의 모든 노이즈 모드를 거부로 증폭
        # (프로브 3회 실측: moving 래치로 레이어 상시 비활성). 이동체 대응은
        # classify_gap의 moving_min(1.2m) gap 규칙이 담당 — 진짜로 막으면
        # '통과 gap 없음'으로 자연 비활성, 충돌 안전은 코리도/fast 게이트 몫.
        flat = [(m['x'], m['y'], m['sx'], m['sy'], m['kind'])
                for m in self._social_mem.values()
                if max(m['sx'], m['sy']) >= BLOCKER_MIN_DIM]
        blockers = blockers_from_detections(
            flat, SOCIAL_X_MIN, self.social_lookahead, BLOCKER_INFLATE)
        if not blockers:
            self._social = None
            self._social_reason = 'noblock'
            self._social_side = 0.0
            return

        # ── 기동 커밋 의미론 ────────────────────────────────────────────
        # 프레임 단위 재계획 + 기억 패치(래치·홀드)는 좌우 목표가 교대해
        # 순이동이 0으로 상쇄됨 (프로브 실측: L+ 604 / R− 279 프레임, 변위
        # 왕복 ±0.2). 한 번 측을 고르면 ① 통과 완료(blocker 소진 = noblock)
        # 또는 ② 그 측이 SOCIAL_HOLD_T 동안 지속 불가일 때까지 같은 측
        # 목표만 갱신하고, 일시 nogap 프레임에는 직전 목표를 유지한 채 계속
        # 조향한다. 안전은 코리도/fast 게이트가 상위에서 계속 담당.
        gaps = compute_gaps(blockers, self._band[0], self._band[1])

        # 통과 국면 보호(cut-in lock): 몸 옆·직전(x ∈ [-1.0, 1.5])에 사람
        # blocker(고스트 포함 — 사각 소실에도 유효)가 있으면 그 사람 쪽
        # 절반으로 향하는 목표를 금지. 근접장에서 파편 필터·사각의 잔여
        # 오차가 만드는 '통과 중 사람 쪽 재타겟'을 구조적으로 차단
        # (프로브 실측: 통과 직전 남측 재선택 → min_clr 0.08m).
        locked_sides = {math.copysign(1.0, m['y'])
                        for m in self._social_mem.values()
                        if m['kind'] in ('person', 'person_moving')
                        and -1.0 < m['x'] < 1.5 and abs(m['y']) > 0.05}

        def side_cands(rules, side=0.0, allow_moving=True):
            out = []
            for g in gaps:
                if not allow_moving and 'person_moving' in (g.lo_kind,
                                                            g.hi_kind):
                    continue
                ok, ty, cap = classify_gap(g, rules)
                if not ok or (side != 0.0 and ty * side <= 0):
                    continue
                if abs(ty) > 1e-6 and math.copysign(1.0, ty) in locked_sides:
                    continue
                out.append((ty, cap, g))
            return out

        committed = self._social_side != 0.0
        if committed:
            # 커밋 측: 정규 → 완화(폭 노이즈 전용) 순으로 목표 갱신
            cands = (side_cands(self._social_rules, self._social_side)
                     or side_cands(self._social_rules_relaxed,
                                   self._social_side, allow_moving=False))
            if cands:
                pick = min(cands, key=lambda c: abs(c[0]))
                self._social = (pick[0], pick[1], now)
                self._social_last_t = now
                self._social_reason = ''
                return
            # 격상 예외: 커밋 측 gap이 이동 보행자 관여로 막힌 것이면 hold
            # 없이 즉시 해제 — 폭 노이즈가 아니라 상황이 실제로 바뀐 것.
            moving_on_side = any(
                'person_moving' in (g.lo_kind, g.hi_kind)
                and (g.y_min + g.y_max) * self._social_side > 0
                for g in gaps)
            if (not moving_on_side
                    and self._social_last_t is not None
                    and now - self._social_last_t <= SOCIAL_HOLD_T):
                # 일시 불가: 직전 목표 유지 (조향 지속 — 시각 갱신으로
                # _apply_social의 신선도 검사 통과)
                if self._social is not None:
                    self._social = (self._social[0], self._social[1], now)
                self._social_reason = 'hold'
                return
            self._social_side = 0.0        # 지속 불가 → 커밋 해제 후 재선택

        cands = side_cands(self._social_rules)
        if not cands:
            self._social = None
            self._social_reason = 'nogap'
            return
        # 신규 커밋: 최소 이동 우선, 동률(0.15)은 우측 통과 관습
        best_abs = min(abs(c[0]) for c in cands)
        near = [c for c in cands if abs(c[0]) <= best_abs + 0.15]
        pick = min(near, key=lambda c: c[0])
        # edge|edge(장애물이 gap을 나누지 않는 빈 밴드)는 커밋하지 않는다 —
        # keep-right 바이어스가 기동 커밋으로 승격되면 이후 등장하는 장애물
        # 상황에서 잘못된 측이 선점됨 (프로브 실측: 북측 협로 0.85m를 완화폭
        # 으로 통과, 사람 여유 0.17m). 커밋은 장애물 제약 하의 선택에만.
        constrained = not (pick[2].lo_kind == 'edge'
                          and pick[2].hi_kind == 'edge')
        self._social_side = (math.copysign(1.0, pick[0])
                             if constrained and abs(pick[0]) > 1e-6 else 0.0)
        self._social = (pick[0], pick[1], now)
        self._social_last_t = now if constrained else self._social_last_t
        self._social_reason = ''

    def _apply_social(self, out, s, now):
        """소셜 vy·속도 캡을 출력에 합성 (복사본 반환 — cmd_in 원본 불변).

        s: 현재 상태의 병진 스케일 (RESUME 램프/SLOW 스케일) — vy도 같은
        스케일을 타야 정지 직후 풀 크랩워크 명령이 나가지 않는다.
        전진 의도(cmd_in.vx > stuck_v_min) 없으면 무개입 = 필터 계약 유지.
        """
        if (self._social is None
                or now - self._social[2] > SOCIAL_STALE
                or self.cmd_in.linear.x <= self.stuck_v_min):
            return out
        target_y, cap, _ = self._social
        o = Twist()
        o.linear.x = out.linear.x
        o.linear.y = out.linear.y
        o.linear.z = out.linear.z
        o.angular.x = out.angular.x
        o.angular.y = out.angular.y
        o.angular.z = out.angular.z
        if abs(target_y) >= LAT_DEADBAND:
            vy = max(-self.lat_vy_max,
                     min(self.lat_vy_max, self.lat_k * target_y))
            o.linear.y += s * vy
        if cap is not None:
            o.linear.x = min(o.linear.x, cap)
        return o

    def _danger_levels(self):
        # Distance-only STOP fires only inside the small emergency margin; the
        # main STOP/SLOW reaction is TTC-driven (actually approaching). Sticky
        # hysteresis: while already restricted, require extra room to relax.
        # Both signals get hysteresis — TTC especially, since it depends on the
        # (noisier) velocity estimate and so chatters more near the boundary.
        clr = self.min_clearance
        ttc = self.min_ttc
        emergency_clr = self.emergency_clr
        slow_clr = self.slow_clr
        stop_ttc = self.stop_ttc
        slow_ttc = self.slow_ttc
        fast_stop_ttc = self.fast_stop_ttc
        fast_slow_ttc = self.fast_slow_ttc
        # WAIT도 STOP과 같은 히스테리시스 유지: WAIT를 기본 임계로 판정하면
        # 물체가 emergency 바로 위(0.6~0.9)에 있을 때 "클리어"로 풀려
        # RESUME 돌진→재STOP 사이클로 마진을 갉아먹음 (2026-07-13 head_on).
        if self.state in (State.STOP, State.WAIT):
            emergency_clr += self.hyst_clr
            slow_clr += self.hyst_clr
            stop_ttc += self.hyst_ttc
            slow_ttc += self.hyst_ttc
            fast_stop_ttc += self.hyst_ttc
            fast_slow_ttc += self.hyst_ttc
        elif self.state == State.SLOW_DOWN:
            slow_clr += self.hyst_clr
            slow_ttc += self.hyst_ttc
            fast_slow_ttc += self.hyst_ttc
        in_stop = (clr <= emergency_clr) or (ttc <= stop_ttc) \
            or (self.min_fast_tcpa <= fast_stop_ttc)
        in_slow = (clr <= slow_clr) or (ttc <= slow_ttc) \
            or (self.min_fast_tcpa <= fast_slow_ttc)
        # CPA 통과까지 STOP 유지: 정지 중에는 fast 물체가 근접 미스거리에서
        # 아직 접근 중인 한(트리거 여부 무관 — 비켜 지나가는 중 포함) 재출발
        # 금지. 멀어짐 전환(closing≤0) = CPA 통과가 해제 조건.
        if self.state in (State.STOP, State.WAIT) and self._fast_near:
            in_stop = True
        return in_stop, in_slow

    def _scale_for_slow(self):
        # Ease forward speed from full (at slow_clearance) to zero at
        # emergency_clr - creep_overshoot: 영점이 STOP 문턱 아래에 있어야
        # creep이 문턱을 실제로 넘어 STOP이 래치된다. Rotation is not scaled.
        clr = self.min_clearance
        floor = self.emergency_clr - self.creep_overshoot
        if clr <= floor:
            return 0.0
        if clr >= self.slow_clr:
            return 1.0
        return (clr - floor) / max(1e-6, (self.slow_clr - floor))

    def _scaled_cmd(self, s):
        # Scale translation by s; rotation passes through (turn away freely).
        out = Twist()
        out.linear.x = s * self.cmd_in.linear.x
        out.linear.y = s * self.cmd_in.linear.y
        out.angular.z = (self.cmd_in.angular.z if self.pass_rotation
                         else s * self.cmd_in.angular.z)
        return out

    def _slow_cmd(self):
        # SLOW 출력: 비례 감속 + creep_v_min 하한 (전진 명령이 하한 이상일 때만)
        out = self._scaled_cmd(self._scale_for_slow())
        if (self.cmd_in.linear.x > self.creep_v_min
                and 0.0 < out.linear.x < self.creep_v_min):
            out.linear.x = self.creep_v_min
        return out

    def _next_state(self, in_stop, in_slow, clear):
        # 5-state FSM: NOMINAL → SLOW_DOWN → STOP → WAIT → RESUME → NOMINAL.
        # Hard STOP dominates from any state. After a STOP, WAIT confirms the
        # front stays clear before RESUME ramps speed back up (gait stability).
        s = self.state
        if in_stop and self._t_in_stop_cond >= self.persist_stop:
            return State.STOP
        if s == State.STOP:
            return State.WAIT                  # stop released → confirm clear
        if s == State.WAIT:
            if clear and self._t_in_clear >= self.wait_dur:
                return State.RESUME
            return State.WAIT
        if s == State.RESUME:
            if self._resume_scale >= 1.0:
                return State.NOMINAL
            return State.RESUME
        # NOMINAL / SLOW_DOWN
        if in_slow and self._t_in_slow_cond >= self.persist_slow:
            return State.SLOW_DOWN
        return State.NOMINAL

    def _record_pose(self, now):
        # Position history for windowed progress detection (odom frame; only
        # deltas are used, so the odom origin is irrelevant).
        self._pose_hist.append((now, self.robot_x, self.robot_y))
        cutoff = now - self.stuck_duration - 0.5
        while len(self._pose_hist) > 1 and self._pose_hist[0][0] < cutoff:
            self._pose_hist.popleft()

    def _window_disp(self, now):
        # Net displacement over the last stuck_duration. None until the buffer
        # spans a full window (can't judge progress yet). Instantaneous odom
        # speed is unusable on a legged robot — gait sway oscillates |v| ±0.2 m/s
        # even when the body isn't translating; it cancels over the window.
        target = now - self.stuck_duration
        if not self._pose_hist or self._pose_hist[0][0] > target:
            return None
        ref = self._pose_hist[0]
        for s in self._pose_hist:
            if s[0] <= target:
                ref = s
            else:
                break
        return math.hypot(self.robot_x - ref[1], self.robot_y - ref[2])

    def _begin_recovery(self, now):
        # Rotate toward the side with the farther nearest-obstacle (more open).
        self._recover_dir = 1.0 if self._left_min >= self._right_min else -1.0
        self._recover_start = now
        self._set_state(State.STUCK, reason='stall_detected')

    def _run_recovery(self, now):
        # Bounded in-place recovery: brief settle, then rotate toward the
        # clearer side. Rotation-only. Returns True when the sequence completes.
        t = now - self._recover_start
        out = Twist()
        done = False
        if t < self.recover_settle_time:
            pass                                   # settle (zero)
        elif t < self.recover_settle_time + self.recover_rotate_time:
            out.angular.z = self._recover_dir * self.recover_yaw_rate
        else:
            done = True
        self._publish_cmd(out)
        return done

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now

        # Bypass mode: pure pass-through (baseline / teleop testing)
        if not self.enable_gate:
            self.state = State.NOMINAL
            self._publish_cmd(self.cmd_in)
            self._publish_state()
            return

        # Latched emergency stop (repeated stuck) — stays until node restart.
        if self.state == State.ESTOP:
            self._publish_zero()
            self._publish_state()
            return

        # Sensor timeout → STOP
        if (self.obs_stamp is None
                or (now - self.obs_stamp) > self.sensor_timeout):
            self._publish_zero()
            self._stall_timer = 0.0
            self._set_state(State.STOP, reason='sensor_timeout')
            self._publish_state()
            return

        # Stuck detection & recovery (Phase 2.5). "Not moving" = net displacement
        # over a stuck_duration window below stuck_disp_min, NOT instantaneous odom
        # speed (a legged robot's gait sway makes |v| oscillate ±0.2 m/s in place).
        if self.stuck_enable:
            if self.state == State.STUCK:
                if self._run_recovery(now):
                    self._stall_timer = 0.0
                    self._pose_hist.clear()        # re-arm: refill window before re-trigger
                    self._set_state(State.NOMINAL, reason='recovery_done')
                self._publish_state()
                return

            self._record_pose(now)
            disp = self._window_disp(now)
            if disp is not None and disp >= self.stuck_disp_min:
                self._recover_attempts = 0         # progressing → clear escalation

            commanding = self._cmd_out_speed > self.stuck_v_min
            self._stall_timer = self._stall_timer + dt if commanding else 0.0

            if (self._stall_timer >= self.stuck_duration
                    and disp is not None and disp < self.stuck_disp_min):
                self._recover_attempts += 1
                if self._recover_attempts > self.max_recover_attempts:
                    self._publish_zero()
                    self._set_state(State.ESTOP, reason='stuck_max_attempts')
                    self._publish_state()
                    return
                self._begin_recovery(now)
                self._pose_hist.clear()            # re-arm window after trigger
                self._run_recovery(now)
                self._publish_state()
                return

        in_stop, in_slow = self._danger_levels()

        # Persistence (debounce entry into more restrictive states) + clear timer
        self._t_in_stop_cond = self._t_in_stop_cond + dt if in_stop else 0.0
        self._t_in_slow_cond = self._t_in_slow_cond + dt if in_slow else 0.0
        # "clear" = no hard-stop threat AND nothing approaching (TTC comfortable)
        clear = (not in_stop) and (self.min_ttc > self.slow_ttc)
        self._t_in_clear = self._t_in_clear + dt if clear else 0.0

        prev = self.state
        new_state = self._next_state(in_stop, in_slow, clear)
        if new_state == State.RESUME and prev != State.RESUME:
            self._resume_scale = 0.0           # ramp restarts from standstill
        self._set_state(new_state)

        # Output. Rotation (angular.z) always passes so the robot can turn away;
        # only translation is gated. Reverse is gated too (no rear perception).
        # 소셜 레이어(5a)는 이동 상태에서만 합성 — STOP/WAIT/STUCK 무개입.
        if self.state == State.NOMINAL:
            out = self.cmd_in
            if self.social_enable:
                out = self._apply_social(out, 1.0, now)
            self._publish_cmd(out)
        elif self.state == State.SLOW_DOWN:
            out = self._slow_cmd()
            if self.social_enable:
                out = self._apply_social(out, self._scale_for_slow(), now)
            self._publish_cmd(out)
        elif self.state == State.RESUME:
            self._resume_scale = min(
                1.0, self._resume_scale + dt / self.resume_time)
            s = self._resume_scale
            if in_slow:
                s = min(s, self._scale_for_slow())
            out = self._scaled_cmd(s)
            if self.social_enable:
                out = self._apply_social(out, s, now)
            self._publish_cmd(out)
        else:  # STOP / WAIT — block translation, allow rotation to turn away
            out = Twist()
            if self.pass_rotation:
                out.angular.z = self.cmd_in.angular.z
            self._publish_cmd(out)

        self._publish_state()

    def _publish_cmd(self, twist: Twist):
        self._cmd_out_speed = math.hypot(twist.linear.x, twist.linear.y)
        self.cmd_out_pub.publish(twist)

    def _publish_zero(self):
        self._cmd_out_speed = 0.0
        self.cmd_out_pub.publish(Twist())

    def _set_state(self, new: State, reason: str = ''):
        if self.state != new:
            self.get_logger().info(f'state: {self.state.value} → {new.value} ({reason})')
            self.state = new

    def _publish_state(self):
        msg = String()
        msg.data = (
            f'{self.state.value} | clr={self.min_clearance:.2f}m | '
            f'ttc={self.min_ttc:.2f}s | v={self.robot_vx:.2f}m/s | '
            f'stall={self._stall_timer:.1f}s att={self._recover_attempts}'
        )
        if self.social_enable:
            if self._social is not None:
                cap = self._social[1]
                msg.data += (f' | soc={self._social[0]:+.2f}'
                             + (f'@{cap:.1f}' if cap is not None else ''))
            elif self._social_reason:
                msg.data += f' | soc=off:{self._social_reason}'
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
