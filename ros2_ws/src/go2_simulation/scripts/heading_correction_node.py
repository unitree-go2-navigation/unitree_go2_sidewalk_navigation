#!/usr/bin/python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class HeadingCorrectionNode(Node):
    def __init__(self):
        super().__init__('heading_correction_node')

        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('max_correction', 0.3)
        self.declare_parameter('deadband', 0.01)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_correction = self.get_parameter('max_correction').value
        self.deadband = self.get_parameter('deadband').value

        self.target_yaw = None
        self.current_yaw = None
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None
        self.last_cmd = Twist()

        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_vel_callback, 10)
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def imu_callback(self, msg):
        self.current_yaw = quaternion_to_yaw(msg.orientation)

    def cmd_vel_callback(self, msg):
        out = Twist()
        out.linear = msg.linear
        out.angular = msg.angular

        moving_forward = abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01
        user_turning = abs(msg.angular.z) > self.deadband

        if user_turning or not moving_forward:
            self.target_yaw = None
            self.integral = 0.0
            self.prev_error = 0.0
            self.pub_cmd.publish(out)
            return

        if self.current_yaw is None:
            self.pub_cmd.publish(out)
            return

        now = self.get_clock().now()

        if self.target_yaw is None:
            self.target_yaw = self.current_yaw
            self.last_time = now
            self.pub_cmd.publish(out)
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 1.0:
            self.pub_cmd.publish(out)
            return

        error = self.target_yaw - self.current_yaw
        # Normalize to [-pi, pi]
        error = math.atan2(math.sin(error), math.cos(error))

        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        correction = self.kp * error + self.ki * self.integral + self.kd * derivative
        correction = max(-self.max_correction, min(self.max_correction, correction))

        out.angular.z = correction
        self.pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
