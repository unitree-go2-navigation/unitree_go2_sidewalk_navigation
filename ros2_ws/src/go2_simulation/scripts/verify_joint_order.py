#!/usr/bin/python3
"""Report whether UGSN joint_states contain the expected controller joint order."""

import time
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


EXPECTED_ORDER = [
    "rf_hip_joint",
    "rf_upper_leg_joint",
    "rf_lower_leg_joint",
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "rh_hip_joint",
    "rh_upper_leg_joint",
    "rh_lower_leg_joint",
    "lh_hip_joint",
    "lh_upper_leg_joint",
    "lh_lower_leg_joint",
]


class JointOrderVerifier(Node):
    def __init__(self) -> None:
        super().__init__("verify_joint_order")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("timeout_sec", 8.0)
        topic = str(self.get_parameter("joint_state_topic").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.msg = None
        self.sub = self.create_subscription(JointState, topic, self.callback, 10)
        self.get_logger().info("Waiting for JointState on %s" % topic)

    def callback(self, msg: JointState) -> None:
        self.msg = msg

    def report(self) -> int:
        if self.msg is None:
            self.get_logger().error("No JointState received")
            return 2

        name_to_index = {name: idx for idx, name in enumerate(self.msg.name)}
        missing = [name for name in EXPECTED_ORDER if name not in name_to_index]
        if missing:
            self.get_logger().error("Missing joints: %s" % ", ".join(missing))
            return 1

        self.get_logger().info("Controller command order expected by ros_control.yaml:")
        for cmd_index, name in enumerate(EXPECTED_ORDER):
            self.get_logger().info(
                "  command[%02d] -> %-24s joint_states index=%02d"
                % (cmd_index, name, name_to_index[name])
            )
        self.get_logger().info("Joint order verification complete")
        return 0


def print_help_if_requested(args) -> bool:
    argv = sys.argv[1:] if args is None else list(args)
    if "-h" not in argv and "--help" not in argv:
        return False

    print(
        """Verify UGSN joint order against ros_control.yaml command order.

Common ROS parameter overrides:
  -p joint_state_topic:=/joint_states
  -p timeout_sec:=8.0
"""
    )
    return True


def main(args=None) -> None:
    if print_help_if_requested(args):
        return

    rclpy.init(args=args)
    node = JointOrderVerifier()
    deadline = time.monotonic() + node.timeout_sec
    try:
        while rclpy.ok() and node.msg is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
