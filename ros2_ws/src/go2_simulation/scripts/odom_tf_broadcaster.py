#!/usr/bin/env python3
"""/odom → odom→base_footprint TF 브로드캐스터 (Nav2 통합용).

이 스택의 TF 트리는 base_footprint→base_link→(센서들)만 있고 odom→base가
없다 (P0~P5 파이프라인은 전부 base_link 토픽 기반이라 불필요했음). Nav2는
map→odom→base 체인을 요구하므로 CHAMP /odom(월드 좌표 초기화, child =
base_link)을 odom→base_footprint TF로 중계한다. 기존 EKF 체인은 설정
행렬이 비어 실질 미동작(원 리포 유산) — 건드리지 않는다.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.cb, 20)

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        p = msg.pose.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
