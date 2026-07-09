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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray


class State(Enum):
    NOMINAL = 'NOMINAL'
    SLOW_DOWN = 'SLOW_DOWN'
    STOP = 'STOP'
    WAIT = 'WAIT'
    RESUME = 'RESUME'
    STUCK = 'STUCK'
    ESTOP = 'ESTOP'


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
        self.declare_parameter('emergency_clearance', 0.25)
        self.declare_parameter('stop_ttc', 1.0)
        # Min closing speed (m/s) to treat an obstacle as approaching
        self.declare_parameter('closing_eps', 0.05)
        # Let rotation (angular.z) pass through while SLOW/STOP so the robot can
        # turn away from a blocking obstacle instead of deadlocking facing it.
        self.declare_parameter('pass_rotation_when_blocked', True)

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
        self.stop_ttc = gp('stop_ttc').value
        self.hyst_clr = gp('hysteresis_clearance').value
        self.hyst_ttc = gp('hysteresis_ttc').value
        self.persist_slow = gp('persistence_slow').value
        self.persist_stop = gp('persistence_stop').value
        self.wait_dur = gp('wait_clear_duration').value
        self.resume_time = gp('resume_time').value
        self.sensor_timeout = gp('sensor_timeout').value
        self.enable_gate = gp('enable_gate').value
        self.closing_eps = gp('closing_eps').value
        self.pass_rotation = gp('pass_rotation_when_blocked').value
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

        # Pub
        self.cmd_out_pub = self.create_publisher(
            Twist, gp('cmd_vel_out_topic').value, 10)
        self.state_pub = self.create_publisher(
            String, '/safety/state', 10)

        # State
        self.cmd_in = Twist()
        self.cmd_in_stamp = None
        self.robot_vx = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.obs_stamp = None
        self.min_clearance = math.inf
        self.min_ttc = math.inf
        self._left_min = math.inf
        self._right_min = math.inf
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
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def obs_cb(self, msg: Detection3DArray):
        self.obs_stamp = self.get_clock().now().nanoseconds * 1e-9
        min_clr = math.inf
        min_ttc = math.inf
        left_min = math.inf
        right_min = math.inf
        for det in msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            hx = 0.5 * det.bbox.size.x
            hy = 0.5 * det.bbox.size.y
            xmin, xmax = cx - hx, cx + hx
            ymin, ymax = cy - hy, cy + hy

            # Must have some extent ahead of the robot
            if xmax <= 0.0:
                continue
            # Nearest obstacle on each side (picks stuck-recovery rotation dir).
            front_face = max(xmin, 0.0)
            if cy >= 0.0:
                left_min = min(left_min, front_face)
            else:
                right_min = min(right_min, front_face)
            # Forward corridor: ignore obstacles whose bbox is laterally
            # outside the robot's travel lane (e.g. parallel side walls).
            if ymin > self.corridor_half or ymax < -self.corridor_half:
                continue

            # Longitudinal clearance: distance from the bbox front face to the
            # robot's front body edge (rectangular footprint half-length).
            front_x = max(xmin, 0.0)
            clr = front_x - self.robot_half_length
            if clr < min_clr:
                min_clr = clr

            # Relative velocity (base_link) embedded by lidar_obstacle_node in
            # covariance[0]=vx, covariance[1]=vy — travels atomically with the
            # detection, so there is no cross-topic ordering race.
            #   closing = -dot(p_rel, v_rel) / |p_rel|   (positive = approaching)
            # Clamp to the static robot-speed estimate so we never under-react,
            # and a zero/missing velocity degrades gracefully to the static case.
            vx = vy = 0.0
            if det.results:
                cov = det.results[0].pose.covariance
                vx, vy = cov[0], cov[1]
            pdist = math.hypot(cx, cy)
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
        self.min_clearance = min_clr
        self.min_ttc = min_ttc
        self._left_min = left_min
        self._right_min = right_min

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
        if self.state == State.STOP:
            emergency_clr += self.hyst_clr
            slow_clr += self.hyst_clr
            stop_ttc += self.hyst_ttc
            slow_ttc += self.hyst_ttc
        elif self.state == State.SLOW_DOWN:
            slow_clr += self.hyst_clr
            slow_ttc += self.hyst_ttc
        in_stop = (clr <= emergency_clr) or (ttc <= stop_ttc)
        in_slow = (clr <= slow_clr) or (ttc <= slow_ttc)
        return in_stop, in_slow

    def _scale_for_slow(self):
        # Ease forward speed from full (at slow_clearance) to zero (at the
        # emergency margin). Rotation is not scaled (see tick).
        clr = self.min_clearance
        if clr <= self.emergency_clr:
            return 0.0
        if clr >= self.slow_clr:
            return 1.0
        return (clr - self.emergency_clr) / max(1e-6, (self.slow_clr - self.emergency_clr))

    def _scaled_cmd(self, s):
        # Scale translation by s; rotation passes through (turn away freely).
        out = Twist()
        out.linear.x = s * self.cmd_in.linear.x
        out.linear.y = s * self.cmd_in.linear.y
        out.angular.z = (self.cmd_in.angular.z if self.pass_rotation
                         else s * self.cmd_in.angular.z)
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
        if self.state == State.NOMINAL:
            self._publish_cmd(self.cmd_in)
        elif self.state == State.SLOW_DOWN:
            self._publish_cmd(self._scaled_cmd(self._scale_for_slow()))
        elif self.state == State.RESUME:
            self._resume_scale = min(
                1.0, self._resume_scale + dt / self.resume_time)
            s = self._resume_scale
            if in_slow:
                s = min(s, self._scale_for_slow())
            self._publish_cmd(self._scaled_cmd(s))
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
