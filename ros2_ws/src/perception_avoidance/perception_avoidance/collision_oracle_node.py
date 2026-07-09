#!/usr/bin/env python3
"""
Collision Oracle (evaluation only).

Ground-truth clearance metric from Gazebo poses, published by the
ActorPosePublisher plugin on /world/default/actor_pose/info as a TFMessage
(robot model + every actor).

    clearance = center_distance - robot_radius - actor_radius

Every transform whose child_frame_id != robot_name is treated as an actor, so
multiple pedestrians are handled automatically; the reported metric is the
minimum clearance over all actors.

This node is fully decoupled from avoidance: it never touches cmd_vel.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class CollisionOracleNode(Node):
    def __init__(self):
        super().__init__('collision_oracle_node')

        self.declare_parameter('robot_name', 'go2')
        self.declare_parameter('robot_radius', 0.25)
        self.declare_parameter('actor_radius', 0.30)
        self.declare_parameter('collision_threshold', 0.0)
        self.declare_parameter('warning_threshold', 1.5)
        self.declare_parameter(
            'ground_truth_topic', '/world/default/actor_pose/info')

        self.robot_name = self.get_parameter('robot_name').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.actor_radius = self.get_parameter('actor_radius').value
        self.collision_threshold = self.get_parameter('collision_threshold').value
        self.warning_threshold = self.get_parameter('warning_threshold').value
        topic = self.get_parameter('ground_truth_topic').value

        self.robot_xy = None
        self.actors = {}          # actor name -> (x, y)
        self.in_collision = {}    # actor name -> bool (inside collision band)

        self.collision_count = 0
        self.min_clearance = float('inf')
        self.min_clearance_actor = None

        self.create_subscription(TFMessage, topic, self.pose_cb, 10)
        self.event_pub = self.create_publisher(String, '/collision_events', 10)
        self.create_timer(0.05, self.check_clearance)

        self.get_logger().info(
            f'Collision oracle started. topic={topic}, robot={self.robot_name} '
            f'(r={self.robot_radius:.2f}m), actor_r={self.actor_radius:.2f}m, '
            f'collision<={self.collision_threshold:.2f}m, '
            f'warning<={self.warning_threshold:.2f}m')

    def pose_cb(self, msg):
        for t in msg.transforms:
            name = t.child_frame_id
            xy = (t.transform.translation.x, t.transform.translation.y)
            if name == self.robot_name:
                self.robot_xy = xy
            elif name:
                self.actors[name] = xy

    def check_clearance(self):
        if self.robot_xy is None or not self.actors:
            return

        rx, ry = self.robot_xy
        frame_min = float('inf')
        frame_min_actor = None
        for name, (ax, ay) in self.actors.items():
            center_dist = math.hypot(rx - ax, ry - ay)
            clearance = center_dist - self.robot_radius - self.actor_radius

            if clearance < frame_min:
                frame_min = clearance
                frame_min_actor = name

            was = self.in_collision.get(name, False)
            if clearance <= self.collision_threshold and not was:
                self.in_collision[name] = True
                self.collision_count += 1
                event = String()
                event.data = (
                    f'COLLISION #{self.collision_count} | actor={name} | '
                    f'clearance={clearance:.3f}m | center_dist={center_dist:.3f}m | '
                    f'robot=({rx:.2f},{ry:.2f}) | actor=({ax:.2f},{ay:.2f})')
                self.event_pub.publish(event)
                self.get_logger().error(event.data)
            elif clearance > self.collision_threshold and was:
                self.in_collision[name] = False

        if frame_min < self.min_clearance:
            self.min_clearance = frame_min
            self.min_clearance_actor = frame_min_actor

        if self.collision_threshold < frame_min < self.warning_threshold:
            self.get_logger().warn(
                f'WARNING: actor approaching | actor={frame_min_actor} | '
                f'clearance={frame_min:.2f}m',
                throttle_duration_sec=2.0)

    def destroy_node(self):
        self.get_logger().info(
            f'=== Collision Oracle Summary ===\n'
            f'  Total collisions: {self.collision_count}\n'
            f'  Min clearance: {self.min_clearance:.3f}m '
            f'(actor={self.min_clearance_actor})')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CollisionOracleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
