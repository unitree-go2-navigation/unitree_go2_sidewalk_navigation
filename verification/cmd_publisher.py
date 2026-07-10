#!/usr/bin/env python3
"""시나리오 드라이버: 준비 대기 → settle → cmd 프로파일 재생 + 상태 기록.

수동 teleop을 대체하는 결정적 입력 소스. sim time 기준으로 동작한다.

  1. /odom + /obstacles/lidar 첫 수신까지 대기 (스택 준비)
  2. settle 시간 대기 (스폰 낙하/자세 안정화)
  3. cmd 프로파일([[t_start, vx, wz], ...])을 20Hz로 /cmd_vel에 발행
     (프로파일 wz==0이면 오돔 요 기준 heading hold P-보정을 wz로 출력 —
      CHAMP 보행 드리프트로 코스를 이탈해 인도변 구조물로 접근하는 것 방지.
      실운용에는 상위 조향이 있으므로 무보정 직진이 오히려 비현실적 가혹 조건)
  4. /safety/state 전이, /collision_events, /odom 이동거리를 states CSV에 기록
  5. duration(sim) 경과 후 zero cmd 발행하고 종료

exit code: 0 = 완료, 2 = 준비 타임아웃.
"""

import argparse
import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray


HEADING_KP = 0.8        # heading hold P 이득 (1/s)
HEADING_WZ_MAX = 0.3    # 보정 각속도 상한 (rad/s, 게이트 pass_rotation 대상)


class ScenarioDriver(Node):
    def __init__(self, args):
        super().__init__(
            'scenario_driver',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.profile = args.profile          # [[t_start, vx, wz], ...]
        self.settle = args.settle
        self.duration = args.duration
        self.states_csv = args.states_csv

        self.odom_ready = False
        self.obs_ready = False
        self.robot_xy = None
        self.robot_yaw = None
        self.yaw_ref = None
        self.travel = 0.0
        self.state = None
        self.state_log = []                  # (t, state)
        self.stop_entries = 0
        self.collision_events = 0

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            Detection3DArray, '/obstacles/lidar', self._obs_cb, 10)
        self.create_subscription(String, '/safety/state', self._state_cb, 10)
        self.create_subscription(
            String, '/collision_events', self._collision_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, msg):
        self.odom_ready = True
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.robot_xy is not None:
            self.travel += math.hypot(
                xy[0] - self.robot_xy[0], xy[1] - self.robot_xy[1])
        self.robot_xy = xy
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _obs_cb(self, msg):
        self.obs_ready = True

    def _state_cb(self, msg):
        state = msg.data.split('|')[0].strip()
        if state != self.state:
            t = self.now_s()
            self.state_log.append((t, state))
            if state == 'STOP':
                self.stop_entries += 1
            self.state = state

    def _collision_cb(self, msg):
        self.collision_events += 1

    def _spin_until(self, cond, wall_timeout_s):
        import time
        deadline = time.monotonic() + wall_timeout_s
        while rclpy.ok() and not cond():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() > deadline:
                return False
        return True

    def run(self):
        if not self._spin_until(
                lambda: self.odom_ready and self.obs_ready, 180.0):
            self.get_logger().error('readiness timeout (/odom, /obstacles/lidar)')
            return 2
        self.get_logger().info('stack ready — settling')

        t_ready = self.now_s()
        if not self._spin_until(
                lambda: self.now_s() - t_ready >= self.settle, 300.0):
            return 2
        # 집계는 주행 시작부터 (스폰 낙하·기동 중 sensor-timeout STOP 제외)
        self.travel = 0.0
        self.state_log = []
        self.stop_entries = 0
        self.collision_events = 0

        t0 = self.now_s()
        self.yaw_ref = self.robot_yaw   # 주행 시작 시점 헤딩을 유지 목표로
        self.get_logger().info(f'driving: profile={self.profile} t0={t0:.1f}')
        period = 0.05
        next_pub = t0
        while rclpy.ok():
            t = self.now_s() - t0
            if t >= self.duration:
                break
            if self.now_s() >= next_pub:
                vx, wz = 0.0, 0.0
                for t_start, pvx, pwz in self.profile:
                    if t >= t_start:
                        vx, wz = pvx, pwz
                if wz == 0.0 and self.yaw_ref is not None \
                        and self.robot_yaw is not None:
                    err = math.remainder(
                        self.yaw_ref - self.robot_yaw, 2.0 * math.pi)
                    wz = max(-HEADING_WZ_MAX,
                             min(HEADING_WZ_MAX, HEADING_KP * err))
                cmd = Twist()
                cmd.linear.x = float(vx)
                cmd.angular.z = float(wz)
                self.cmd_pub.publish(cmd)
                next_pub += period
            rclpy.spin_once(self, timeout_sec=0.02)

        for _ in range(10):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

        with open(self.states_csv, 'w') as f:
            f.write('t,state\n')
            for t, s in self.state_log:
                f.write(f'{t:.3f},{s}\n')
            f.write(f'# summary travel={self.travel:.3f} '
                    f'stop_entries={self.stop_entries} '
                    f'collision_events={self.collision_events} '
                    f'states={"/".join(dict.fromkeys(s for _, s in self.state_log))}\n')
        self.get_logger().info(
            f'done: travel={self.travel:.2f}m stop_entries={self.stop_entries}')
        return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', required=True,
                        help='JSON [[t_start, vx, wz], ...]')
    parser.add_argument('--settle', type=float, default=8.0)
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--states-csv', required=True)
    args = parser.parse_args()
    args.profile = json.loads(args.profile)

    rclpy.init()
    driver = ScenarioDriver(args)
    try:
        rc = driver.run()
    finally:
        driver.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
