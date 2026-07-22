"""보도 경계 polygon 발행 (Phase 5a) — 팀원 segmentation 계약의 시뮬 대역.

계약 (계획 v2 인터페이스 표): `/perception/sidewalk/boundary`,
geometry_msgs/PolygonStamped, base_link frame, ≥5Hz. 실기체에서는 팀원
segmentation 노드가 같은 토픽을 발행하므로 remap만으로 스왑된다.

시뮬 구현: 월드 고정 폴리곤(YAML) → base_link 역변환. 이 스택의 /odom은
월드 좌표로 초기화되므로 (CHAMP 상태추정이 스폰 포즈에서 시작 — 2026-07-22
계측으로 확정, 데모 follower도 같은 가정으로 동작) odom pose를 그대로 로봇
월드 포즈로 사용한다. 스폰 오프셋 합성 금지 — 이중 계상이 된다.
첫 odom 수신 전에는 발행하지 않는다 (소비자는 timeout을 폴백 신호로 사용).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, PolygonStamped
from nav_msgs.msg import Odometry


class SidewalkPolygonNode(Node):
    def __init__(self):
        super().__init__('sidewalk_polygon_node')
        self.declare_parameter('polygon_x', [-11.3, 41.3, 41.3, -11.3])
        self.declare_parameter('polygon_y', [3.7, 3.7, 6.7, 6.7])
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_topic', '/perception/sidewalk/boundary')

        gp = lambda n: self.get_parameter(n).value
        self.poly_world = list(zip(gp('polygon_x'), gp('polygon_y')))
        self._odom = None
        self._logged_first = False

        self.pub = self.create_publisher(PolygonStamped, gp('output_topic'), 10)
        self.create_subscription(Odometry, gp('odom_topic'), self._odom_cb, 20)
        self.create_timer(1.0 / gp('publish_rate'), self._tick)
        self.get_logger().info(
            f'sidewalk_polygon: vertices={len(self.poly_world)} '
            f'(odom = world pose 가정)')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = (p.x, p.y, yaw, msg.header.stamp)

    def _tick(self):
        if self._odom is None:
            return
        rx, ry, ryaw, stamp = self._odom
        cr, sr = math.cos(ryaw), math.sin(ryaw)

        msg = PolygonStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        for wx, wy in self.poly_world:
            dx, dy = wx - rx, wy - ry
            msg.polygon.points.append(Point32(
                x=cr * dx + sr * dy, y=-sr * dx + cr * dy, z=0.0))
        if not self._logged_first:
            self._logged_first = True
            p0 = msg.polygon.points[0]
            self.get_logger().info(
                f'first publish: robot_world=({rx:.2f}, {ry:.2f}, {ryaw:.3f}) '
                f'v0_base=({p0.x:.2f}, {p0.y:.2f})')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SidewalkPolygonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
