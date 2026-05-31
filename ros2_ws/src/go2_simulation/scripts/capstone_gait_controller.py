#!/usr/bin/python3
"""Capstone-style position gait controller adapted for UGSN small_city tests."""

import json
import math
import sys
from typing import Dict, Iterable, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty, Float64MultiArray, Int32

try:
    from unitree_api.msg import Request
except Exception:  # pragma: no cover - optional dependency in some workspaces.
    Request = None


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


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


class CapstoneGaitController(Node):
    MODE_STAND = 1
    MODE_WALK = 2
    MODE_SIT = 3
    MODE_LIE = 4

    # UGSN ros_control.yaml order:
    # RF, LF, RH, LH, with hip / upper leg / lower leg per leg.
    OUTPUT_LEG_ORDER = ("RF", "LF", "RH", "LH")

    def __init__(self) -> None:
        super().__init__("capstone_gait_controller")

        self._declare_parameters()
        self._load_parameters()

        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 30
        )
        self.mode_sub = self.create_subscription(
            Int32, self.sport_mode_topic, self.mode_callback, 10
        )
        self.reset_sub = self.create_subscription(
            Empty, self.reset_topic, self.reset_callback, 10
        )
        if Request is not None:
            self.api_sub = self.create_subscription(
                Request, self.api_sport_topic, self.api_sport_callback, 10
            )
        else:
            self.api_sub = None

        self.joint_pub = self.create_publisher(
            Float64MultiArray, self.controller_topic, 10
        )

        self.mode = self.MODE_STAND
        self.phase = 0.0
        self.last_cmd_time = None
        self.last_update_time = None

        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.filtered_linear_x = 0.0
        self.filtered_linear_y = 0.0
        self.filtered_angular_z = 0.0

        self.current_yaw = 0.0
        self.have_odom = False
        self.heading_hold_active = False
        self.heading_target_yaw = 0.0
        self.prev_heading_error = 0.0

        self.timer = self.create_timer(1.0 / self.publish_rate, self.update)
        self.publish_pose(self.stand_pose_array())
        self.get_logger().info(
            "Capstone gait controller active: %s -> %s"
            % (self.cmd_vel_topic, self.controller_topic)
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "cmd_vel_topic": "/cmd_vel",
            "controller_topic": "/joint_group_controller/commands",
            "odom_topic": "/odom",
            "sport_mode_topic": "/go2_sim/sport_mode_request",
            "reset_topic": "/go2_sim/reset_request",
            "api_sport_topic": "/api/sport/request",
            "publish_rate": 150.0,
            "command_timeout": 0.6,
            "auto_walk_on_cmd": True,
            "max_linear_speed": 0.20,
            "max_lateral_speed": 0.08,
            "max_angular_speed": 0.35,
            "linear_accel_limit": 0.28,
            "lateral_accel_limit": 0.16,
            "angular_accel_limit": 0.45,
            "linear_decel_limit": 0.45,
            "lateral_decel_limit": 0.30,
            "angular_decel_limit": 0.70,
            "base_frequency": 0.55,
            "frequency_gain": 0.25,
            "stride_scale": 0.040,
            "lateral_stride_scale": 0.030,
            "turn_scale": 0.025,
            "lift_scale": 0.030,
            "hip_sway_scale": 0.012,
            "stand_hip": 0.0,
            "stand_thigh": 0.67,
            "stand_calf": -1.30,
            "sit_hip": 0.0,
            "sit_thigh": 1.10,
            "sit_calf": -2.00,
            "lie_hip": 0.0,
            "lie_thigh": 1.35,
            "lie_calf": -2.35,
            "heading_hold_enabled": True,
            "heading_hold_kp": 1.0,
            "heading_hold_kd": 0.10,
            "heading_hold_deadband_deg": 2.0,
            "heading_hold_max_yaw_cmd": 0.08,
            "heading_hold_lateral_threshold": 0.01,
            "heading_hold_yaw_threshold": 0.01,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        def get(name: str):
            return self.get_parameter(name).value

        self.cmd_vel_topic = str(get("cmd_vel_topic"))
        self.controller_topic = str(get("controller_topic"))
        self.odom_topic = str(get("odom_topic"))
        self.sport_mode_topic = str(get("sport_mode_topic"))
        self.reset_topic = str(get("reset_topic"))
        self.api_sport_topic = str(get("api_sport_topic"))

        self.publish_rate = float(get("publish_rate"))
        self.command_timeout = float(get("command_timeout"))
        self.auto_walk_on_cmd = bool(get("auto_walk_on_cmd"))

        self.max_linear_speed = float(get("max_linear_speed"))
        self.max_lateral_speed = float(get("max_lateral_speed"))
        self.max_angular_speed = float(get("max_angular_speed"))
        self.linear_accel_limit = float(get("linear_accel_limit"))
        self.lateral_accel_limit = float(get("lateral_accel_limit"))
        self.angular_accel_limit = float(get("angular_accel_limit"))
        self.linear_decel_limit = float(get("linear_decel_limit"))
        self.lateral_decel_limit = float(get("lateral_decel_limit"))
        self.angular_decel_limit = float(get("angular_decel_limit"))

        self.base_frequency = float(get("base_frequency"))
        self.frequency_gain = float(get("frequency_gain"))
        self.stride_scale = float(get("stride_scale"))
        self.lateral_stride_scale = float(get("lateral_stride_scale"))
        self.turn_scale = float(get("turn_scale"))
        self.lift_scale = float(get("lift_scale"))
        self.hip_sway_scale = float(get("hip_sway_scale"))

        self.stand_hip = float(get("stand_hip"))
        self.stand_thigh = float(get("stand_thigh"))
        self.stand_calf = float(get("stand_calf"))
        self.sit_hip = float(get("sit_hip"))
        self.sit_thigh = float(get("sit_thigh"))
        self.sit_calf = float(get("sit_calf"))
        self.lie_hip = float(get("lie_hip"))
        self.lie_thigh = float(get("lie_thigh"))
        self.lie_calf = float(get("lie_calf"))

        self.heading_hold_enabled = bool(get("heading_hold_enabled"))
        self.heading_hold_kp = float(get("heading_hold_kp"))
        self.heading_hold_kd = float(get("heading_hold_kd"))
        self.heading_hold_deadband = math.radians(float(get("heading_hold_deadband_deg")))
        self.heading_hold_max_yaw_cmd = float(get("heading_hold_max_yaw_cmd"))
        self.heading_hold_lateral_threshold = float(get("heading_hold_lateral_threshold"))
        self.heading_hold_yaw_threshold = float(get("heading_hold_yaw_threshold"))

    def cmd_callback(self, msg: Twist) -> None:
        self.last_cmd_time = self.get_clock().now()
        self.target_linear_x = clamp(
            float(msg.linear.x), -self.max_linear_speed, self.max_linear_speed
        )
        self.target_linear_y = clamp(
            float(msg.linear.y), -self.max_lateral_speed, self.max_lateral_speed
        )
        self.target_angular_z = clamp(
            float(msg.angular.z), -self.max_angular_speed, self.max_angular_speed
        )

        moving = (
            abs(self.target_linear_x) > 1.0e-4
            or abs(self.target_linear_y) > 1.0e-4
            or abs(self.target_angular_z) > 1.0e-4
        )
        if moving and self.auto_walk_on_cmd and self.mode in (self.MODE_STAND, self.MODE_WALK):
            self.mode = self.MODE_WALK

        lateral_only = (
            self.heading_hold_enabled
            and self.have_odom
            and abs(self.target_linear_y) >= self.heading_hold_lateral_threshold
            and abs(self.target_angular_z) <= self.heading_hold_yaw_threshold
        )
        if lateral_only and not self.heading_hold_active:
            self.heading_target_yaw = self.current_yaw
            self.prev_heading_error = 0.0
            self.heading_hold_active = True
            self.get_logger().info(
                "Heading hold latched at %.1f deg" % math.degrees(self.heading_target_yaw)
            )
        elif not lateral_only:
            self.heading_hold_active = False

    def odom_callback(self, msg: Odometry) -> None:
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True

    def mode_callback(self, msg: Int32) -> None:
        if msg.data in (self.MODE_STAND, self.MODE_WALK, self.MODE_SIT, self.MODE_LIE):
            self.mode = int(msg.data)
        elif msg.data == 0:
            self.mode = self.MODE_STAND
        self.get_logger().info("Sport mode request: %d" % msg.data)

    def reset_callback(self, _msg: Empty) -> None:
        self.phase = 0.0
        self.mode = self.MODE_STAND
        self.zero_targets()
        self.filtered_linear_x = 0.0
        self.filtered_linear_y = 0.0
        self.filtered_angular_z = 0.0
        self.publish_pose(self.stand_pose_array())
        self.get_logger().info("Reset gait state")

    def api_sport_callback(self, msg) -> None:
        api_id = int(getattr(msg, "api_id", 0))
        parameter = str(getattr(msg, "parameter", ""))
        command = ""
        if parameter:
            try:
                data = json.loads(parameter)
                command = str(data.get("command", "")).lower()
            except json.JSONDecodeError:
                command = parameter.lower()

        if command in ("stand", "stand_up") or api_id in (1002, 1004):
            self.mode = self.MODE_STAND
        elif command in ("walk", "trot") or api_id in (1008, 1016):
            self.mode = self.MODE_WALK
        elif command == "sit" or api_id == 1009:
            self.mode = self.MODE_SIT
        elif command == "lie" or api_id == 1001:
            self.mode = self.MODE_LIE

    def zero_targets(self) -> None:
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.heading_hold_active = False

    def update(self) -> None:
        now = self.get_clock().now()
        if self.last_update_time is None:
            dt = 1.0 / self.publish_rate
        else:
            dt = max((now - self.last_update_time).nanoseconds * 1.0e-9, 1.0e-4)
        self.last_update_time = now

        if self.last_cmd_time is None:
            self.zero_targets()
        else:
            age = (now - self.last_cmd_time).nanoseconds * 1.0e-9
            if age > self.command_timeout:
                self.zero_targets()

        self.filtered_linear_x = self.slew(
            self.filtered_linear_x,
            self.target_linear_x,
            self.linear_accel_limit,
            self.linear_decel_limit,
            dt,
        )
        self.filtered_linear_y = self.slew(
            self.filtered_linear_y,
            self.target_linear_y,
            self.lateral_accel_limit,
            self.lateral_decel_limit,
            dt,
        )
        self.filtered_angular_z = self.slew(
            self.filtered_angular_z,
            self.target_angular_z,
            self.angular_accel_limit,
            self.angular_decel_limit,
            dt,
        )

        yaw_cmd = self.filtered_angular_z + self.heading_hold_correction(dt)
        yaw_cmd = clamp(yaw_cmd, -self.max_angular_speed, self.max_angular_speed)

        moving = (
            abs(self.filtered_linear_x) > 1.0e-3
            or abs(self.filtered_linear_y) > 1.0e-3
            or abs(yaw_cmd) > 1.0e-3
        )

        if self.mode == self.MODE_SIT:
            self.publish_pose(self.sit_pose_array())
        elif self.mode == self.MODE_LIE:
            self.publish_pose(self.lie_pose_array())
        elif moving:
            self.publish_pose(
                self.crawl_gait_pose(
                    self.filtered_linear_x,
                    self.filtered_linear_y,
                    yaw_cmd,
                    dt,
                )
            )
        else:
            self.phase = 0.0
            if self.mode == self.MODE_WALK:
                self.mode = self.MODE_STAND
            self.publish_pose(self.stand_pose_array())

    def heading_hold_correction(self, dt: float) -> float:
        if not self.heading_hold_active or not self.have_odom:
            return 0.0

        error = normalize_angle(self.heading_target_yaw - self.current_yaw)
        if abs(error) <= self.heading_hold_deadband:
            self.prev_heading_error = error
            return 0.0

        derivative = (error - self.prev_heading_error) / max(dt, 1.0e-4)
        self.prev_heading_error = error
        correction = self.heading_hold_kp * error + self.heading_hold_kd * derivative
        return clamp(
            correction,
            -self.heading_hold_max_yaw_cmd,
            self.heading_hold_max_yaw_cmd,
        )

    @staticmethod
    def slew(current: float, target: float, accel: float, decel: float, dt: float) -> float:
        delta = target - current
        if abs(delta) < 1.0e-6:
            return target
        rate = accel if abs(target) > abs(current) else decel
        step = rate * dt
        if abs(delta) <= step:
            return target
        return current + math.copysign(step, delta)

    def crawl_gait_pose(self, vx: float, vy: float, wz: float, dt: float) -> Iterable[float]:
        linear_cmd = vx / max(self.max_linear_speed, 1.0e-6)
        lateral_cmd = vy / max(self.max_lateral_speed, 1.0e-6)
        angular_cmd = wz / max(self.max_angular_speed, 1.0e-6)
        linear_cmd = clamp(linear_cmd, -1.0, 1.0)
        lateral_cmd = clamp(lateral_cmd, -1.0, 1.0)
        angular_cmd = clamp(angular_cmd, -1.0, 1.0)
        motion_scale = max(abs(linear_cmd), abs(lateral_cmd), abs(angular_cmd))

        frequency = self.base_frequency + self.frequency_gain * motion_scale
        self.phase = (self.phase + frequency * dt) % 1.0

        leg_phase_offsets = {
            "RF": 0.00,
            "LH": 0.25,
            "LF": 0.50,
            "RH": 0.75,
        }
        side_sign = {"RF": -1.0, "LF": 1.0, "RH": -1.0, "LH": 1.0}
        front_sign = {"RF": 1.0, "LF": 1.0, "RH": -1.0, "LH": -1.0}

        poses: Dict[str, Tuple[float, float, float]] = {}
        for leg, offset in leg_phase_offsets.items():
            phase = (self.phase + offset) % 1.0
            stride_wave = math.sin(2.0 * math.pi * phase)
            lift_wave = max(0.0, math.sin(2.0 * math.pi * phase))
            side = side_sign[leg]
            front = front_sign[leg]

            lateral_hip = side * self.lateral_stride_scale * lateral_cmd * stride_wave
            turn_hip = side * self.turn_scale * angular_cmd
            sway = side * self.hip_sway_scale * motion_scale * math.sin(4.0 * math.pi * phase)

            fore_aft = self.stride_scale * linear_cmd * stride_wave
            turn_stride = 0.60 * self.turn_scale * angular_cmd * front * stride_wave
            lateral_leg_load = 0.35 * self.lateral_stride_scale * lateral_cmd * stride_wave

            hip = self.stand_hip + lateral_hip + turn_hip + sway
            thigh = self.stand_thigh + fore_aft + turn_stride + lateral_leg_load
            calf = (
                self.stand_calf
                - self.lift_scale * lift_wave * motion_scale
                - 0.25 * fore_aft
                - 0.45 * turn_stride
                - 0.20 * lateral_leg_load
            )

            poses[leg] = (
                clamp(hip, -0.55, 0.55),
                clamp(thigh, -0.35, 1.65),
                clamp(calf, -2.25, -0.85),
            )

        return self.pose_dict_to_array(poses)

    def pose_dict_to_array(self, poses: Dict[str, Tuple[float, float, float]]) -> Iterable[float]:
        values = []
        for leg in self.OUTPUT_LEG_ORDER:
            values.extend(poses[leg])
        return values

    def stand_pose_array(self) -> Iterable[float]:
        return self.fixed_pose_array(self.stand_hip, self.stand_thigh, self.stand_calf)

    def sit_pose_array(self) -> Iterable[float]:
        return self.fixed_pose_array(self.sit_hip, self.sit_thigh, self.sit_calf)

    def lie_pose_array(self) -> Iterable[float]:
        return self.fixed_pose_array(self.lie_hip, self.lie_thigh, self.lie_calf)

    def fixed_pose_array(self, hip: float, thigh: float, calf: float) -> Iterable[float]:
        data = []
        for _leg in self.OUTPUT_LEG_ORDER:
            data.extend((hip, thigh, calf))
        return data

    def publish_pose(self, values: Iterable[float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(value) for value in values]
        self.joint_pub.publish(msg)


def print_help_if_requested(args) -> bool:
    argv = sys.argv[1:] if args is None else list(args)
    if "-h" not in argv and "--help" not in argv:
        return False

    print(
        """Capstone-style gait controller for UGSN.

Subscribes:
  /cmd_vel                         geometry_msgs/Twist
  /odom                            nav_msgs/Odometry

Publishes:
  /joint_group_controller/commands std_msgs/Float64MultiArray

Common ROS parameter overrides:
  -p cmd_vel_topic:=/cmd_vel
  -p controller_topic:=/joint_group_controller/commands
  -p odom_topic:=/odom
  -p max_linear_speed:=0.20
  -p max_lateral_speed:=0.08
  -p max_angular_speed:=0.35
"""
    )
    return True


def main(args=None) -> None:
    if print_help_if_requested(args):
        return

    rclpy.init(args=args)
    node = CapstoneGaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.zero_targets()
        for _ in range(10):
            node.publish_pose(node.stand_pose_array())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
