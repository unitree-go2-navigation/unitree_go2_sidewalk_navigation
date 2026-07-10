"""safety_stop_node 순수 로직 테스트 (Gazebo 불필요).

obs_cb의 clearance/TTC/corridor 계산, FSM 전이, SLOW 스케일,
stuck 창(window) 변위 로직을 노드 콜백 직접 호출로 검증한다.
"""

import math

import pytest
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from perception_avoidance.safety_stop_node import SafetyStopNode, State


CORRIDOR_HALF = 0.335  # robot_half_width(0.155) + corridor_margin(0.18)


def make_det(cx, cy, sx=0.4, sy=0.4, vx=0.0, vy=0.0, cfx=None):
    """cfx(코리도 내 최근접점 x, covariance[2])는 기본적으로 발행 노드처럼
    bbox에서 유도: 밴드 겹침 시 bbox 앞면, 아니면 -1. 명시 지정으로 점-bbox
    불일치(평행 구조물 bbox 번짐) 케이스를 표현한다."""
    det = Detection3D()
    det.bbox.center.position.x = float(cx)
    det.bbox.center.position.y = float(cy)
    det.bbox.size.x = float(sx)
    det.bbox.size.y = float(sy)
    det.bbox.size.z = 1.0
    if cfx is None:
        in_band = (cy - 0.5 * sy) <= CORRIDOR_HALF and (cy + 0.5 * sy) >= -CORRIDOR_HALF
        cfx = (cx - 0.5 * sx) if in_band else -1.0
    hyp = ObjectHypothesisWithPose()
    hyp.pose.covariance[0] = float(vx)
    hyp.pose.covariance[1] = float(vy)
    hyp.pose.covariance[2] = float(cfx)
    det.results.append(hyp)
    return det


def feed(node, dets, robot_vx=0.0):
    node.robot_vx = robot_vx
    msg = Detection3DArray()
    msg.detections.extend(dets)
    node.obs_cb(msg)


@pytest.fixture()
def node():
    n = SafetyStopNode()
    yield n
    n.destroy_node()


# --- clearance / TTC / corridor (obs_cb) ---

def test_static_obstacle_clearance_rectangular_footprint(node):
    # bbox center x=2.0, size 0.4 → front face 1.8 → clr = 1.8 - 0.35 = 1.45
    feed(node, [make_det(2.0, 0.0)])
    assert node.min_clearance == pytest.approx(1.45, abs=1e-6)


def test_static_obstacle_robot_stopped_ttc_inf(node):
    feed(node, [make_det(2.0, 0.0)], robot_vx=0.0)
    assert math.isinf(node.min_ttc)


def test_approaching_obstacle_finite_ttc(node):
    # v_rel = (-1, 0) toward robot at (2,0): closing = 1.0 → ttc = clr/1.0
    feed(node, [make_det(2.0, 0.0, vx=-1.0)])
    assert node.min_ttc == pytest.approx(1.45, abs=1e-6)


def test_receding_obstacle_ttc_inf(node):
    feed(node, [make_det(2.0, 0.0, vx=1.0)], robot_vx=0.0)
    assert math.isinf(node.min_ttc)


def test_static_obstacle_robot_moving_uses_robot_speed(node):
    # closing clamped to robot_vx=0.5 → ttc = 1.45/0.5 = 2.9
    feed(node, [make_det(2.0, 0.0)], robot_vx=0.5)
    assert node.min_ttc == pytest.approx(2.9, abs=1e-6)


def test_overlapping_obstacle_zero_ttc(node):
    # front face 0.2 → clr = -0.15 ≤ 0 → ttc = 0
    feed(node, [make_det(0.3, 0.0, sx=0.2)])
    assert node.min_clearance == pytest.approx(-0.15, abs=1e-6)
    assert node.min_ttc == 0.0


def test_obstacle_outside_corridor_ignored(node):
    # corridor_half = 0.155 + 0.18 = 0.335; bbox ymin = 1.0-0.2 = 0.8 > 0.335
    feed(node, [make_det(2.0, 1.0)])
    assert math.isinf(node.min_clearance)


def test_obstacle_straddling_corridor_counted(node):
    # bbox ymin = 0.5-0.2=0.3 < 0.335 → in corridor
    feed(node, [make_det(2.0, 0.5)])
    assert node.min_clearance == pytest.approx(1.45, abs=1e-6)


def test_parallel_wall_bbox_smear_ignored(node):
    # 회귀 케이스 (2026-07-10 empty 0/5): 긴 평행 구조물의 axis-aligned bbox가
    # 요 오차로 코리도를 스치지만(ymax=-0.15 > -0.335) 실제 점은 전부 밴드 밖
    # (cfx=-1) → 차단으로 치지 않아야 한다. 구 bbox 판정은 clr=0.2로 영구 STOP.
    feed(node, [make_det(2.3, -1.1, sx=3.5, sy=1.9, cfx=-1.0)])
    assert math.isinf(node.min_clearance)
    assert math.isinf(node.min_ttc)


def test_in_corridor_point_overrides_bbox_center_offset(node):
    # 중심은 코리도 밖(cy=-1.1)이어도 점이 밴드 안에 있으면(cfx=1.2) 차단으로 판정
    feed(node, [make_det(2.3, -1.1, sx=3.5, sy=1.9, cfx=1.2)])
    assert node.min_clearance == pytest.approx(1.2 - 0.35, abs=1e-6)


def test_obstacle_behind_ignored(node):
    feed(node, [make_det(-1.0, 0.0)])
    assert math.isinf(node.min_clearance)


def test_left_right_nearest_for_recovery_direction(node):
    feed(node, [make_det(1.0, 1.0), make_det(2.0, -1.0)])
    assert node._left_min == pytest.approx(0.8, abs=1e-6)   # front face 1.0-0.2
    assert node._right_min == pytest.approx(1.8, abs=1e-6)


# --- FSM transitions ---

def test_nominal_to_slow_requires_persistence(node):
    node.state = State.NOMINAL
    node._t_in_slow_cond = 0.0
    assert node._next_state(False, True, False) == State.NOMINAL
    node._t_in_slow_cond = node.persist_slow
    assert node._next_state(False, True, False) == State.SLOW_DOWN


def test_stop_dominates_from_any_state(node):
    node._t_in_stop_cond = node.persist_stop
    for s in (State.NOMINAL, State.SLOW_DOWN, State.WAIT, State.RESUME):
        node.state = s
        assert node._next_state(True, True, False) == State.STOP


def test_stop_to_wait_when_released(node):
    node.state = State.STOP
    node._t_in_stop_cond = 0.0
    assert node._next_state(False, True, False) == State.WAIT


def test_wait_to_resume_after_clear_duration(node):
    node.state = State.WAIT
    node._t_in_clear = node.wait_dur - 0.01
    assert node._next_state(False, False, True) == State.WAIT
    node._t_in_clear = node.wait_dur
    assert node._next_state(False, False, True) == State.RESUME


def test_resume_to_nominal_when_ramp_done(node):
    node.state = State.RESUME
    node._resume_scale = 0.5
    assert node._next_state(False, False, True) == State.RESUME
    node._resume_scale = 1.0
    assert node._next_state(False, False, True) == State.NOMINAL


def test_hysteresis_inflates_thresholds_in_stop(node):
    # clr slightly above emergency: NOMINAL → not in_stop, STOP → still in_stop
    node.min_clearance = node.emergency_clr + 0.5 * node.hyst_clr
    node.min_ttc = math.inf
    node.state = State.NOMINAL
    in_stop, _ = node._danger_levels()
    assert not in_stop
    node.state = State.STOP
    in_stop, _ = node._danger_levels()
    assert in_stop


# --- SLOW scaling ---

def test_scale_for_slow_boundaries(node):
    floor = node.emergency_clr - node.creep_overshoot
    node.min_clearance = floor
    assert node._scale_for_slow() == 0.0
    node.min_clearance = node.slow_clr
    assert node._scale_for_slow() == 1.0
    node.min_clearance = 0.5 * (floor + node.slow_clr)
    assert node._scale_for_slow() == pytest.approx(0.5, abs=1e-6)


def test_slow_cmd_enforces_creep_floor(node):
    # scale이 만드는 초저속(<creep_v_min)은 걷지 못해 STUCK 오인 → 하한 적용
    node.cmd_in.linear.x = 0.3
    node.min_clearance = (node.emergency_clr - node.creep_overshoot) + 0.02
    assert node._scaled_cmd(node._scale_for_slow()).linear.x < node.creep_v_min
    assert node._slow_cmd().linear.x == pytest.approx(node.creep_v_min)
    # 명령 자체가 하한 이하면 그대로 (하한이 명령을 키우지 않음)
    node.cmd_in.linear.x = 0.05
    assert node._slow_cmd().linear.x <= 0.05


def test_creep_crosses_stop_threshold(node):
    # STOP 문턱(emergency_clr)에서 스케일이 0보다 커야 creep이 문턱을 관통해
    # STOP이 래치된다 (점근 접근으로 STOP 미진입하던 회귀 케이스 고정)
    node.min_clearance = node.emergency_clr
    assert node._scale_for_slow() > 0.0


# --- stuck window displacement ---

def test_window_disp_none_until_window_filled(node):
    node.robot_x, node.robot_y = 0.0, 0.0
    node._record_pose(100.0)
    assert node._window_disp(100.0) is None   # window not yet spanned


def test_window_disp_detects_progress_vs_stall(node):
    # stalled: jitter around origin over the full window
    t0 = 100.0
    for i in range(25):
        t = t0 + i * 0.1
        node.robot_x = 0.02 * (1 if i % 2 else -1)
        node.robot_y = 0.0
        node._record_pose(t)
    node.robot_x, node.robot_y = 0.0, 0.0
    disp = node._window_disp(t0 + 2.4)
    assert disp is not None and disp < node.stuck_disp_min

    # progressing: 0.45 m over the window
    node._pose_hist.clear()
    for i in range(25):
        t = t0 + 10 + i * 0.1
        node.robot_x = i * 0.02
        node.robot_y = 0.0
        node._record_pose(t)
    disp = node._window_disp(t0 + 10 + 2.4)
    assert disp is not None and disp >= node.stuck_disp_min
