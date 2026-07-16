#!/usr/bin/env python3
"""데모 전용 odom 폐루프 waypoint follower (pure-pursuit 근사).

⚠ 임시 데모용 — 검증 회귀에 쓰지 않는다. 조향(경로)은 사전 계획이고,
안전 반응(STOP/SLOW/재개)은 그 아래 safety_stop 게이트가 실시간 인지로
수행한다는 최종 아키텍처(planner 위 / 게이트 아래)의 시연이 목적.
경로 계획 부분은 이후 Nav2(Track N)로 대체된다.

/odom 기준으로 waypoint를 순서대로 추종해 /cmd_vel(게이트 입력)에 발행.
게이트가 STOP을 걸면 출력이 0으로 잘리고, 해제되면 자동으로 이어 간다.
"""

import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

REACH_DIST = 0.35     # waypoint 도달 판정 (m)
CRUISE_V = 0.4        # 순항 전진 (m/s) — 보행 안정 범위
TURN_V = 0.15         # 중간 조향 오차 시 전진 (creep 이상 유지)
MAX_WZ = 0.5          # rad/s (gait.yaml max_angular_velocity_z)
K_HEADING = 1.5
SETTLE_S = 8.0        # 보행 컨트롤러 안정화 대기 (드라이버와 동일)


def norm_ang(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Follower(Node):
    def __init__(self, waypoints):
        super().__init__(
            'demo_waypoint_follower',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.waypoints = waypoints    # [x, y] 또는 [x, y, hold_s(도달 후 대기)]
        self.idx = 0
        self.pose = None          # (x, y, yaw)
        self.t0 = None
        self.hold_until = None    # 대기 waypoint: 이 시각까지 정지 유지
        self.done = False
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self._tick)   # 20Hz
        self.get_logger().info(
            f'follower: {len(waypoints)} waypoints, settle {SETTLE_S}s')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def _tick(self):
        if self.pose is None or self.done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        if now - self.t0 < SETTLE_S:
            return
        if self.hold_until is not None:
            if now < self.hold_until:
                self.cmd_pub.publish(Twist())   # 양보 대기 (정지 유지)
                return
            self.hold_until = None
            self.get_logger().info(f'hold 종료 → 재개 t_sim={now - self.t0:.1f}s')
        x, y, yaw = self.pose
        tx, ty = self.waypoints[self.idx][:2]
        if math.hypot(tx - x, ty - y) < REACH_DIST:
            wp = self.waypoints[self.idx]
            if len(wp) > 2 and wp[2] > 0:
                self.hold_until = now + float(wp[2])
                self.get_logger().info(
                    f'waypoint {self.idx} 도달 — {wp[2]}s 양보 대기 '
                    f't_sim={now - self.t0:.1f}s')
            self.idx += 1
            if self.idx >= len(self.waypoints):
                self.cmd_pub.publish(Twist())
                self.get_logger().info('course complete')
                self.done = True
                return
            tx, ty = self.waypoints[self.idx][:2]
            self.get_logger().info(
                f'→ waypoint {self.idx}: ({tx}, {ty}) t_sim={now - self.t0:.1f}s')
        err = norm_ang(math.atan2(ty - y, tx - x) - yaw)
        cmd = Twist()
        cmd.angular.z = max(-MAX_WZ, min(MAX_WZ, K_HEADING * err))
        # 3단계 조향: 오차가 크면 제자리 회전으로 방향부터 잡는다 —
        # 지그재그 꼭짓점에서 회전 중 전진하면 헤딩이 다음 장애물을 훑는
        # 동안 emergency 경계까지 파고들어 게이트 STOP/RESUME이 반복됨
        # (실측: 꼭짓점마다 멈칫거림 — 데모 인상 저하)
        if abs(err) > 0.6:
            cmd.linear.x = 0.0        # pivot
        elif abs(err) > 0.3:
            cmd.linear.x = TURN_V
        else:
            cmd.linear.x = CRUISE_V
        self.cmd_pub.publish(cmd)


def main():
    waypoints = json.loads(sys.argv[1])
    trace_path = sys.argv[2] if len(sys.argv) > 2 else None
    rclpy.init()
    node = Follower(waypoints)
    if trace_path:
        # odom 궤적 기록 (0.5s 간격) — 런 후 장애물 최소 거리 수치 검증용
        node._trace = open(trace_path, 'w')
        node._trace.write('t,x,y\n')
        node._last_trace = [0.0]

        def dump():
            if node.pose is None:
                return
            now = node.get_clock().now().nanoseconds * 1e-9
            if now - node._last_trace[0] >= 0.5:
                node._last_trace[0] = now
                node._trace.write(
                    f'{now:.1f},{node.pose[0]:.3f},{node.pose[1]:.3f}\n')
                node._trace.flush()
        node.create_timer(0.25, dump)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.cmd_pub.publish(Twist())
    except (KeyboardInterrupt, Exception):
        pass    # 종료 경로(SIGINT/컨텍스트 무효)에서는 정리만
    node.destroy_node()


if __name__ == '__main__':
    main()
