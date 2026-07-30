"""Phase 5a 소셜 레이어의 게이트 통합 테스트 (Gazebo 불필요).

obs_cb→_update_social의 분류·고스트·gap 선택과 _apply_social의 주입
규칙(스케일·의도 게이트·aliasing 금지)을 노드 콜백 직접 호출로 검증한다.
설계검토(2026-07-21)에서 확정된 결함 모드가 각 테스트의 근거다.
"""

import math

import pytest
from geometry_msgs.msg import Point32, PolygonStamped, Twist
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from perception_avoidance.safety_stop_node import SafetyStopNode


def make_det(cx, cy, sx=0.4, sy=0.4, vx=0.0, vy=0.0, det_id='t1',
             cz=0.0, sz=1.0):
    det = Detection3D()
    det.bbox.center.position.x = float(cx)
    det.bbox.center.position.y = float(cy)
    det.bbox.center.position.z = float(cz)
    det.bbox.size.x = float(sx)
    det.bbox.size.y = float(sy)
    det.bbox.size.z = float(sz)
    hyp = ObjectHypothesisWithPose()
    hyp.pose.covariance[0] = float(vx)
    hyp.pose.covariance[1] = float(vy)
    in_band = abs(cy) - 0.5 * sy <= 0.335
    hyp.pose.covariance[2] = (cx - 0.5 * sx) if in_band else -1.0
    det.results.append(hyp)
    det.id = str(det_id)
    return det


def feed(node, dets, robot_vx=0.0):
    # 정상 상태 가정: 저역통과 자기속도 = 순간 속도 (odom_cb 미호출 대체)
    node.robot_vx = robot_vx
    node._vx_lp = robot_vx
    msg = Detection3DArray()
    t = node.get_clock().now().nanoseconds * 1e-9
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int((t - int(t)) * 1e9)
    msg.detections.extend(dets)
    node.obs_cb(msg)


def feed_poly(node, y_lo=-1.5, y_hi=1.5):
    msg = PolygonStamped()
    for x, y in [(-6.0, y_lo), (10.0, y_lo), (10.0, y_hi), (-6.0, y_hi)]:
        msg.polygon.points.append(Point32(x=x, y=y, z=0.0))
    node.poly_cb(msg)


@pytest.fixture()
def node():
    n = SafetyStopNode()
    yield n
    n.destroy_node()


@pytest.fixture()
def snode():
    n = SafetyStopNode()
    n.social_enable = True
    yield n
    n.destroy_node()


# --- 기본 OFF 불활성 ---------------------------------------------------------

def test_off_social_stays_none(node):
    node.social_enable = False      # P5-4부터 기본 ON — OFF 경로 명시 검증
    feed_poly(node)
    feed(node, [make_det(2.5, 0.8)])
    assert node._social is None


# --- 활성 조건 / 분류 --------------------------------------------------------

def test_standing_person_right_pass_target(snode):
    # 정지 보행자(키 1.0 → top 0.5 = person) 정면 — 우측 gap 선택,
    # 최소 이탈: comfort(0.6)+반폭 지점 (edge 밀착보다 이탈이 작으면 우선)
    feed_poly(snode)
    feed(snode, [make_det(2.5, 0.0)])
    assert snode._social is not None
    target, cap, _ = snode._social
    assert cap == pytest.approx(0.3)
    # blocker [-0.3,0.3] (min폭 0.5 + inflate) → comfort 지점이 edge 한도에
    # 클립: -(1.5-0.3-0.155)
    assert target == pytest.approx(-1.045)


def test_no_polygon_disables(snode):
    feed(snode, [make_det(2.5, 0.0)])
    assert snode._social is None


def test_band_not_containing_robot_disables(snode):
    # polygon 좌표계 오류(예: 무변환 월드 좌표)로 밴드가 로봇 y=0을 포함하지
    # 않으면 소셜 비활성 — 보도 밖 목표 조향 차단 (2026-07-22 사고 가드)
    feed_poly(snode, y_lo=3.7, y_hi=6.7)
    feed(snode, [make_det(2.5, 4.5)])
    assert snode._social is None


def test_low_box_is_static_squeeze_no_cap(snode):
    # 낮은 박스(top -0.2, 차선 걸침): static — 사람 캡 없이 넓은 쪽 회피
    feed_poly(snode)
    feed(snode, [make_det(2.5, 0.55, cz=-0.4, sz=0.4)])
    target, cap, _ = snode._social
    assert cap is None          # 정적만 관여 + 넓은 gap → 캡 없음
    assert target < 0           # 우측(넓은 쪽) 선택


def test_moving_person_blocks_via_gap_rule(snode):
    # 지상 속도 1.0 m/s 보행자가 중앙 점유 — 베토가 아니라 gap 규칙으로
    # 비활성: 양측 gap이 moving_min(1.2m) 미달 → 통과 gap 없음
    feed_poly(snode, y_lo=-1.35, y_hi=1.35)
    for _ in range(3):
        feed(snode, [make_det(3.0, 0.0, vx=-1.0)])
    assert snode._social is None


def test_moving_person_with_wide_gap_passable(snode):
    # 차선을 걸친 이동 보행자라도 반대측에 1.2m 이상 gap이 있으면 레이어
    # 유지 (초소형 노이즈 track의 베토 래치가 레이어를 죽이던 회귀 방지)
    feed_poly(snode)
    for _ in range(3):
        feed(snode, [make_det(3.0, 0.5, vx=-1.0)])
    assert snode._social is not None


def test_off_lane_blockers_no_steering(snode):
    # 직진 차선이 비어 있으면 조향 불개입 — 가장자리 관목 person들이 상시
    # 크랩을 만들어 실효 전진이 절반이 되던 회귀 방지 (final 게이트 실측)
    feed_poly(snode)
    feed(snode, [make_det(2.5, 1.2), make_det(3.5, -1.1)])
    assert snode._social is None


def test_ground_speed_compensation(snode):
    # 로봇 0.44 주행 중 정지물의 상대속도는 -0.44 — 지상 속도 ≈0이므로
    # 이동으로 승격되면 안 됨 (미보상이면 레이어 자기 비활성 — 설계검토)
    feed_poly(snode)
    for _ in range(5):
        feed(snode, [make_det(2.5, 0.0, vx=-0.44)], robot_vx=0.44)
    assert snode._social is not None


def test_sliding_chunk_not_moving(snode):
    # 가림 경계 슬라이딩 청크: 상대속도 ≈0 + 지상속도 ≈로봇속도 —
    # 상대속도 하한(0.25) 미달로 이동 승격 금지 (킬스위치 래치 방지)
    feed_poly(snode)
    for _ in range(6):
        feed(snode, [make_det(2.5, 0.0),                     # 정지 보행자
                     make_det(4.0, 1.1, vx=0.0, vy=0.0,      # 슬라이딩 청크
                              det_id='chunk', cz=0.2, sz=1.0)],
             robot_vx=0.44)
    assert snode._social is not None


def test_ghost_persists_through_dropout(snode):
    # 관측 소실 후에도 blocker가 고스트로 유지 → target 유지 (커트인 방지)
    feed_poly(snode)
    feed(snode, [make_det(1.2, 0.0)])
    t0 = snode._social[0]
    for _ in range(3):
        feed(snode, [], robot_vx=0.4)      # 사람 소실 (사각 진입 가정)
    assert snode._social is not None
    assert snode._social[0] == pytest.approx(t0, abs=0.05)


def test_ghost_dropped_when_passed(snode):
    # 고스트가 로봇 후방으로 완전히 전파되면 제거 → 소셜 해제
    feed_poly(snode)
    feed(snode, [make_det(0.3, 0.0)])
    assert snode._social is not None
    # 0.4 m/s × 0.1s/프레임 → x 0.3 → -0.85(= -(0.35+0.3)-sx/2) 아래까지
    prev_t = snode._last_social_t
    for k in range(40):
        snode._last_social_t = prev_t - 0.1   # dt=0.1 강제
        feed(snode, [], robot_vx=0.4)
        prev_t = snode._last_social_t
        if snode._social is None:
            break
    assert snode._social is None


# --- 주입 규칙 (_apply_social) ----------------------------------------------

def _now(node):
    return node.get_clock().now().nanoseconds * 1e-9


def test_injection_scales_and_caps(snode):
    feed_poly(snode)
    feed(snode, [make_det(2.5, 0.0)])
    snode.cmd_in.linear.x = 0.5
    base = Twist()
    base.linear.x = 0.5
    out = snode._apply_social(base, 1.0, _now(snode))
    assert out.linear.y == pytest.approx(-0.2)      # 클램프 (0.8×1.045 > 0.2)
    assert out.linear.x == pytest.approx(0.3)       # 사람 인접 캡
    half = snode._apply_social(base, 0.5, _now(snode))
    assert half.linear.y == pytest.approx(-0.1)     # RESUME/SLOW 스케일 적용


def test_injection_requires_forward_intent(snode):
    feed_poly(snode)
    feed(snode, [make_det(2.5, 0.0)])
    snode.cmd_in.linear.x = 0.0                     # 조종 의도 없음
    base = Twist()
    out = snode._apply_social(base, 1.0, _now(snode))
    assert out.linear.y == 0.0


def test_injection_does_not_mutate_cmd_in(snode):
    # 설계검토 blocker: cmd_in aliasing → vy 누적. 복사본에만 합성해야 함.
    feed_poly(snode)
    feed(snode, [make_det(2.5, 0.0)])
    snode.cmd_in.linear.x = 0.5
    for _ in range(5):
        snode._apply_social(snode.cmd_in, 1.0, _now(snode))
    assert snode.cmd_in.linear.y == 0.0


def test_stale_social_no_injection(snode):
    feed_poly(snode)
    feed(snode, [make_det(2.5, 0.0)])
    snode.cmd_in.linear.x = 0.5
    base = Twist()
    base.linear.x = 0.5
    out = snode._apply_social(base, 1.0, _now(snode) + 1.0)  # 1s 경과 가정
    assert out.linear.y == 0.0 and out.linear.x == pytest.approx(0.5)


def test_cut_in_lock_while_passing(snode):
    # 사람이 몸 옆(x 0.5, y +0.7)을 지나는 중 — 전방에 새 blocker가 생겨도
    # 사람 쪽(+)으로 향하는 목표는 금지 (통과 중 커트인 → min_clr 0.08 실측)
    feed_poly(snode)
    feed(snode, [make_det(0.5, 0.7, det_id='ped'),
                 make_det(3.0, -0.9, det_id='box', cz=-0.4, sz=0.4)])
    if snode._social is not None:
        assert snode._social[0] < 0     # 사람 반대쪽(−)만 허용


# --- 5b YIELD ---------------------------------------------------------------

from perception_avoidance.safety_stop_node import (
    State, YIELD_CLEAR_T, YIELD_EDGE_OFF, YIELD_REACH)


def feed_oncoming(node, frames=10, y=0.0, band=0.6):
    # 좁은 밴드(±band) + 정면 접근 보행자 (실제처럼 매 프레임 접근):
    # 이동 승격 3 + 위치 접근 증거 5 + 트리거 3 프레임 소요
    feed_poly(node, y_lo=-band, y_hi=band)
    for i in range(frames):
        feed(node, [make_det(4.6 - 0.15 * i, y, vx=-1.6, det_id='onc')])


def test_yield_triggers_on_oncoming_nogap(snode):
    feed_oncoming(snode)
    assert snode._yield_req
    assert snode._yield_side == -1.0            # 접근자 중앙(≥-0.1) → 우측
    assert snode._yield_target == pytest.approx(-0.6 + YIELD_EDGE_OFF)


def test_no_yield_when_gap_passable(snode):
    # 넓은 밴드(±1.5): 측방 gap 1.2 ≥ moving_min → 통과 후보 존재 → YIELD 없음
    feed_poly(snode)
    for _ in range(7):
        feed(snode, [make_det(3.5, 0.0, vx=-1.6, det_id='onc')])
    assert not snode._yield_req


def test_no_yield_for_lateral_passer(snode):
    # 측방 평행 통과(자전거 등): 고속 접근 + nogap이어도 차선 충돌이
    # 아니면 양보 금지 (bike_pass false-yield 방어)
    feed_poly(snode)
    for i in range(10):
        feed(snode, [make_det(6.0 - 0.5 * i, 1.2, sy=0.3, vx=-5.0,
                              det_id='bike'),
                     make_det(3.0, -0.5, det_id='p1'),   # 중앙 통과 불가
                     make_det(3.0, 0.5, det_id='p2')])
    assert not snode._yield_req


def test_no_yield_for_static_person(snode):
    # 정지 보행자 + nogap이어도 접근자가 아니면 YIELD 없음 (5a 영역)
    feed_oncoming(snode, frames=0)
    for _ in range(7):
        feed(snode, [make_det(3.5, 0.0, det_id='sp')])
    assert not snode._yield_req


def test_yield_fsm_transitions(snode):
    feed_oncoming(snode)
    snode.state = State.NOMINAL
    assert snode._next_state(False, False, True) == State.YIELD_MOVE
    # 이동 중 도달 → 대기
    snode.state = State.YIELD_MOVE
    snode._yield_start = snode._last_tick
    snode._yield_target = 0.05                  # < YIELD_REACH
    assert snode._next_state(False, False, True) == State.YIELD_WAIT
    # 대기 중 접근자 소멸 1s 지속 → RESUME
    snode.state = State.YIELD_WAIT
    snode._t_yield_clear = YIELD_CLEAR_T + 0.1
    assert snode._next_state(False, False, True) == State.RESUME
    # 이동 중 접근자 소멸 → NOMINAL 복귀
    snode.state = State.YIELD_MOVE
    snode._social_oncoming = None
    snode._yield_target = 0.5
    assert snode._next_state(False, False, True) == State.NOMINAL


def test_yield_move_continues_through_stop(snode):
    # YIELD_MOVE는 전진 0 + 측방 탈출 — in_stop에도 크랩 지속 (얼리면
    # 접근자 차선 안 정지 → actor 관통, 프로브 실측). WAIT은 STOP 우선.
    feed_oncoming(snode)
    snode.state = State.YIELD_MOVE
    snode._yield_start = snode._last_tick
    snode._yield_target = 0.5
    snode._t_in_stop_cond = 1.0
    assert snode._next_state(True, True, False) == State.YIELD_MOVE
    snode.state = State.YIELD_WAIT
    assert snode._next_state(True, True, False) == State.STOP


def test_yield_move_output(snode):
    feed_oncoming(snode)
    snode.state = State.NOMINAL
    snode.cmd_in.linear.x = 0.5
    captured = []
    snode.cmd_out_pub.publish = lambda m: captured.append(m)
    snode.tick()
    assert snode.state == State.YIELD_MOVE
    out = captured[-1]
    assert out.linear.x <= 0.15 + 1e-6          # 감속 상한
    assert out.linear.y < 0.0                   # 우측 가장자리로 크랩


# --- blind-hold 지상 vy 보정 -------------------------------------------------

def test_blind_hold_vy_ground_compensated(node):
    # 크랩워크(vy_lp -0.2) 중 정지 보행자(상대 vy +0.2)가 hold에 래치되면
    # 지상 vy ≈ 0으로 저장되어야 예측 이탈 ③이 오발하지 않는다 (설계검토
    # FSM-1: 오염 시 hold 조기 해제 → RESUME 사각 관통 재발)
    node._vy_lp = -0.2
    feed(node, [make_det(0.8, 0.0, vy=0.2)])   # clr 0.25 ≤ blind_hold 0.6
    assert node._blind_hold is not None
    assert abs(node._blind_hold['vy']) < 0.05



def test_steering_off_keeps_yield(snode):
    # Nav2 모드(v2): social_steering=false — vy 주입은 죽고 YIELD 요구는 산다
    snode.social_steering = False
    feed_oncoming(snode)
    assert snode._yield_req                      # 양보 상태기계 유지
    snode.cmd_in.linear.x = 0.5
    base = Twist()
    base.linear.x = 0.5
    out = snode._apply_social(base, 1.0, _now(snode))
    assert out.linear.y == 0.0                   # 조향 주입 차단


# --- 회전 통과 차단 + 도달 실변위 증거 (2026-07-30 결합 버그 수정) ---

def test_nav_rotation_blocked_while_oncoming_latched(node):
    node.social_steering = False
    node._social_oncoming = {'id': 'onc', 'x': 4.0, 'y': 0.0, 'vx': -1.0}
    assert node._pass_rot() is False


def test_rotation_passes_without_latch(node):
    node.social_steering = False
    node._social_oncoming = None
    assert node._pass_rot() is True


def test_yield_reach_requires_real_displacement(node):
    from perception_avoidance.safety_stop_node import State
    now = node.get_clock().now().nanoseconds * 1e-9
    node.obs_stamp = now
    node._last_tick = now
    node.state = State.YIELD_MOVE
    node._social_oncoming = {'id': 'onc', 'x': 4.0, 'y': 0.0, 'vx': -1.0}
    node._yield_start = now          # 시한 미도래
    node._yield_start_y = node.robot_y
    node._yield_need_disp = 0.35     # 시작 시 잔여 이탈 큼
    node._yield_target = 0.05        # 요잉으로 목표가 순간 0 근처 (오판 조건)
    nxt = node._next_state(False, False, True)
    assert nxt == State.YIELD_MOVE   # 실변위 없음 → 도달 아님


def test_yield_reach_with_displacement(node):
    from perception_avoidance.safety_stop_node import State
    now = node.get_clock().now().nanoseconds * 1e-9
    node.obs_stamp = now
    node._last_tick = now
    node.state = State.YIELD_MOVE
    node._social_oncoming = {'id': 'onc', 'x': 4.0, 'y': 0.0, 'vx': -1.0}
    node._yield_start = now
    node._yield_start_y = node.robot_y - 0.5   # 0.5m 실이동 완료
    node._yield_need_disp = 0.35
    node._yield_target = 0.05
    nxt = node._next_state(False, False, True)
    assert nxt == State.YIELD_WAIT
