#!/usr/bin/python3
"""Collect simple odom metrics for lateral gait visual tests."""

import math
import sys
import time
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Sample:
    x: float
    y: float
    yaw: float


class LateralMetrics(Node):
    def __init__(self) -> None:
        super().__init__("record_lateral_gait_metrics")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("duration_sec", 20.0)
        self.declare_parameter("expected_direction", "none")
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.expected_direction = str(self.get_parameter("expected_direction").value)
        self.first = None
        self.last = None
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.callback, 30)
        self.get_logger().info("Recording odom metrics from %s" % self.odom_topic)

    def callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        sample = Sample(
            pose.position.x,
            pose.position.y,
            quaternion_to_yaw(pose.orientation),
        )
        if self.first is None:
            self.first = sample
        self.last = sample

    def report(self) -> int:
        if self.first is None or self.last is None:
            self.get_logger().error("No odometry samples received")
            return 2

        dx_world = self.last.x - self.first.x
        dy_world = self.last.y - self.first.y
        cos0 = math.cos(-self.first.yaw)
        sin0 = math.sin(-self.first.yaw)
        dx_body = cos0 * dx_world - sin0 * dy_world
        dy_body = sin0 * dx_world + cos0 * dy_world
        yaw_delta = normalize_angle(self.last.yaw - self.first.yaw)

        self.get_logger().info(
            "delta_body: x=%.3f m, y=%.3f m, yaw=%.2f deg"
            % (dx_body, dy_body, math.degrees(yaw_delta))
        )

        if self.expected_direction == "left" and dy_body <= 0.0:
            self.get_logger().error("Expected left movement, but body-frame y did not increase")
            return 1
        if self.expected_direction == "right" and dy_body >= 0.0:
            self.get_logger().error("Expected right movement, but body-frame y did not decrease")
            return 1

        self.get_logger().info("Metric recording complete")
        return 0


def print_help_if_requested(args) -> bool:
    argv = sys.argv[1:] if args is None else list(args)
    if "-h" not in argv and "--help" not in argv:
        return False

    print(
        """Record odom metrics for lateral gait visual tests.

Common ROS parameter overrides:
  -p odom_topic:=/odom
  -p duration_sec:=20.0
  -p expected_direction:=left|right|none
"""
    )
    return True


def main(args=None) -> None:
    if print_help_if_requested(args):
        return

    rclpy.init(args=args)
    node = LateralMetrics()
    deadline = time.monotonic() + node.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
