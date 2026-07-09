#!/usr/bin/env python3
"""Sensor degradation harness (sim2real robustness testing).

Injects real-L1 characteristics into the clean Gazebo point cloud so
sim-tuned perception is stress-tested before touching hardware:

  - density downsample (non-repetitive scan is ~1/3–1/10 of sim density)
  - random dropout (reflectivity loss on dark/wet surfaces)
  - radial range noise (cm-level jitter)
  - pipeline latency (delayed republish)

Wiring: /unitree_lidar/points → [this node] → /unitree_lidar/points_degraded,
and lidar_obstacle_node subscribes to the degraded topic (launch arg).
Profiles live in verification/profiles/*.yaml.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


class DegradePointcloudNode(Node):
    def __init__(self, **kwargs):
        super().__init__('degrade_pointcloud_node', **kwargs)

        self.declare_parameter('input_topic', '/unitree_lidar/points')
        self.declare_parameter('output_topic', '/unitree_lidar/points_degraded')
        # Fraction of points kept to mimic sparse non-repetitive scan (1.0 = all)
        self.declare_parameter('density_keep_ratio', 1.0)
        # Additional random dropout (reflectivity loss)
        self.declare_parameter('dropout_ratio', 0.0)
        # Gaussian noise along the ray (m)
        self.declare_parameter('range_noise_std', 0.0)
        # Republish delay (s). 0 = immediate.
        self.declare_parameter('latency_s', 0.0)
        self.declare_parameter('seed', 42)

        gp = self.get_parameter
        self.keep = float(gp('density_keep_ratio').value) * \
            (1.0 - float(gp('dropout_ratio').value))
        self.noise_std = float(gp('range_noise_std').value)
        self.latency = float(gp('latency_s').value)
        self._rng = np.random.default_rng(int(gp('seed').value))
        self._queue = deque()   # (due_time_s, msg)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            PointCloud2, gp('input_topic').value, self.cloud_cb, sensor_qos)
        self.pub = self.create_publisher(
            PointCloud2, gp('output_topic').value, sensor_qos)
        if self.latency > 0.0:
            self.create_timer(0.005, self._drain_queue)

        self.get_logger().info(
            f'degrade_pointcloud_node: keep={self.keep:.2f} '
            f'noise_std={self.noise_std:.3f}m latency={self.latency:.3f}s | '
            f'{gp("input_topic").value} → {gp("output_topic").value}')

    def degrade(self, pts):
        """Pure degradation transform on an Nx3 array (testable w/o ROS I/O)."""
        n = pts.shape[0]
        if n == 0:
            return pts
        if self.keep < 1.0:
            mask = self._rng.random(n) < self.keep
            pts = pts[mask]
            n = pts.shape[0]
        if self.noise_std > 0.0 and n > 0:
            r = np.linalg.norm(pts, axis=1)
            r = np.maximum(r, 1e-6)
            scale = (r + self._rng.normal(0.0, self.noise_std, n)) / r
            pts = pts * scale[:, None]
        return pts

    def cloud_cb(self, msg: PointCloud2):
        pts = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = self.degrade(pts.astype(np.float32, copy=False))

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out = pc2.create_cloud(msg.header, fields, pts)

        if self.latency <= 0.0:
            self.pub.publish(out)
        else:
            due = self.get_clock().now().nanoseconds * 1e-9 + self.latency
            self._queue.append((due, out))

    def _drain_queue(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        while self._queue and self._queue[0][0] <= now:
            _, msg = self._queue.popleft()
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DegradePointcloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
