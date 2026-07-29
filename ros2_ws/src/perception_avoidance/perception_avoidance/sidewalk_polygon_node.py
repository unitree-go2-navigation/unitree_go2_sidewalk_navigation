"""보도 경계 polygon 발행 (Phase 5a) — 팀원 segmentation 계약의 시뮬 대역.

계약 (계획 v2 인터페이스 표): `/perception/sidewalk/boundary`,
geometry_msgs/PolygonStamped, base_link frame, ≥5Hz. 실기체에서는 팀원
segmentation 노드가 같은 토픽을 발행하므로 remap만으로 스왑된다.

두 모드 (band_source):
- 'config' (기본): 월드 고정 폴리곤(YAML) → base_link 역변환. 결정적 —
  러너 회귀·시나리오 평가용. 이 스택의 /odom은 월드 좌표로 초기화되므로
  (2026-07-22 계측 확정) odom pose를 그대로 로봇 월드 포즈로 사용.
  스폰 오프셋 합성 금지 — 이중 계상이 된다.
- 'lidar': 원시 라이다에서 밴드를 실시간 추정 — 사전 주입 정보 제거
  (2026-07-29 사용자 방침: 실시간 정보만 활용). 남측 경계 = 연석 낙차
  (보도가 도로보다 0.16m 높음), 북측 경계 = 건물 벽(수직 점군).
  실기체에서 세그멘테이션/연석 인식이 채울 자리의 시뮬 등가물.

첫 발행 전(odom/클라우드 미수신)에는 발행하지 않는다 (소비자는 timeout을
폴백 신호로 사용). lidar 모드는 추정 실패 시 직전 유효값을 최대
stale_hold_s 동안 유지 후 발행 중단.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point32, PolygonStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import tf2_ros
from tf2_ros import TransformException


def quat_to_rot_matrix(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
         2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)],
    ])


def find_band_edges(bin_y, ground_h, wall_n, ref_h,
                    drop_thresh=0.10, wall_min=5, max_gap_bins=5):
    """지상 높이 프로파일에서 보도 밴드의 좌/우 경계(base y)를 찾는다.

    bin_y: 각 lateral bin 중심 y (오름차순), ground_h: bin별 지면 높이
    (NaN=표본 없음), wall_n: bin별 수직(벽 후보) 점 수, ref_h: 로봇 발밑
    기준 지면 높이. 경계 = ref 대비 drop_thresh 이상 낮아지는 첫 bin(연석)
    또는 wall_n >= wall_min 첫 bin(벽). 표본 없는 bin이 max_gap_bins 연속
    이면 그 직전까지를 경계로 본다(폐색 보수). 반환 (y_min, y_max).
    """
    n = len(bin_y)
    c = int(np.argmin(np.abs(bin_y)))  # 로봇 정면(0) bin

    def scan(direction):
        edge = bin_y[c]
        gap = 0
        i = c
        while 0 <= i < n:
            h, w = ground_h[i], wall_n[i]
            if w >= wall_min:
                return bin_y[i]
            if math.isnan(h):
                gap += 1
                if gap > max_gap_bins:
                    return edge
            else:
                gap = 0
                if h < ref_h - drop_thresh:
                    return bin_y[i]
                edge = bin_y[i]
            i += direction
        return edge

    return scan(-1), scan(+1)


class SidewalkPolygonNode(Node):
    def __init__(self):
        super().__init__('sidewalk_polygon_node')
        self.declare_parameter('polygon_x', [-11.3, 41.3, 41.3, -11.3])
        self.declare_parameter('polygon_y', [3.7, 3.7, 6.7, 6.7])
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_topic', '/perception/sidewalk/boundary')
        # --- lidar 추정 모드 ---
        self.declare_parameter('band_source', 'config')   # config | lidar
        self.declare_parameter('points_topic', '/unitree_lidar/points')
        self.declare_parameter('strip_x', [0.5, 8.0])   # 전방 추정 창
        self.declare_parameter('bin_size', 0.3)
        self.declare_parameter('lateral_range', 8.0)
        self.declare_parameter('curb_drop', 0.10)       # 연석 낙차 문턱 (실제 0.16)
        self.declare_parameter('wall_z', [0.3, 1.8])    # 벽 후보 z 대역
        self.declare_parameter('wall_min_pts', 5)
        self.declare_parameter('edge_alpha', 0.3)       # 경계 EMA
        self.declare_parameter('stale_hold_s', 2.0)
        self.declare_parameter('poly_x_extent', [-2.0, 8.0])  # 출력 직사각 x

        gp = lambda n: self.get_parameter(n).value
        self.mode = gp('band_source')
        self.poly_world = list(zip(gp('polygon_x'), gp('polygon_y')))
        self._odom = None
        self._logged_first = False

        self.pub = self.create_publisher(PolygonStamped, gp('output_topic'), 10)
        self.create_subscription(Odometry, gp('odom_topic'), self._odom_cb, 20)

        if self.mode == 'lidar':
            self.strip_x = gp('strip_x')
            self.bin_size = gp('bin_size')
            self.lat_range = gp('lateral_range')
            self.curb_drop = gp('curb_drop')
            self.wall_z = gp('wall_z')
            self.wall_min = int(gp('wall_min_pts'))
            self.alpha = gp('edge_alpha')
            self.stale_hold = gp('stale_hold_s')
            self.poly_x = gp('poly_x_extent')
            self._edges = None          # (y_min, y_max) EMA
            self._last_est = None       # 마지막 유효 추정 시각 (sec)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=5)
            self.create_subscription(
                PointCloud2, gp('points_topic'), self._cloud_cb, sensor_qos)
        else:
            self.create_timer(1.0 / gp('publish_rate'), self._tick)

        self.get_logger().info(
            f'sidewalk_polygon: mode={self.mode} '
            + (f'vertices={len(self.poly_world)} (odom = world pose 가정)'
               if self.mode == 'config' else
               f'strip_x={gp("strip_x")} curb_drop={gp("curb_drop")}'))

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = (p.x, p.y, yaw, msg.header.stamp)

    # --- config 모드: 월드 고정 폴리곤 역변환 ---
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

    # --- lidar 모드: 연석 낙차 + 벽으로 밴드 추정 ---
    def _cloud_cb(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except TransformException:
            return
        pts = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = pts.astype(np.float64, copy=False).reshape(-1, 3)
        if pts.shape[0] == 0:
            return
        t = tf.transform.translation
        r = tf.transform.rotation
        R = quat_to_rot_matrix(r.x, r.y, r.z, r.w)
        pts = pts @ R.T + np.array([t.x, t.y, t.z])

        # 전방 스트립 + 횡 범위
        m = ((pts[:, 0] >= self.strip_x[0]) & (pts[:, 0] <= self.strip_x[1])
             & (np.abs(pts[:, 1]) <= self.lat_range))
        pts = pts[m]
        if pts.shape[0] < 30:
            self._publish_lidar(msg.header.stamp)
            return

        nb = int(2 * self.lat_range / self.bin_size)
        idx = np.clip(((pts[:, 1] + self.lat_range) / self.bin_size)
                      .astype(int), 0, nb - 1)
        bin_y = -self.lat_range + (np.arange(nb) + 0.5) * self.bin_size
        ground_h = np.full(nb, np.nan)
        wall_n = np.zeros(nb, dtype=int)

        z = pts[:, 2]
        # 지면 후보: 발밑 부근 대역 (base_link 기준 대략 -0.5~0.0).
        # 연석 아래 도로면(-0.16 추가)까지 포함되도록 하한 넉넉히.
        gmask = (z > -0.6) & (z < 0.0)
        wmask = (z > self.wall_z[0]) & (z < self.wall_z[1])
        for b in range(nb):
            sel = idx == b
            gz = z[sel & gmask]
            if gz.size >= 3:
                ground_h[b] = float(np.median(gz))
            wall_n[b] = int(np.count_nonzero(sel & wmask))

        # 기준 높이 = 로봇 정면 ±0.5m bin들의 지면 중앙값
        near = np.abs(bin_y) <= 0.5
        ref_vals = ground_h[near]
        ref_vals = ref_vals[~np.isnan(ref_vals)]
        if ref_vals.size == 0:
            self._publish_lidar(msg.header.stamp)
            return
        ref_h = float(np.median(ref_vals))

        y_min, y_max = find_band_edges(
            bin_y, ground_h, wall_n, ref_h,
            drop_thresh=self.curb_drop, wall_min=self.wall_min)
        if y_max - y_min < 1.0:      # 비상식적 협폭 → 노이즈 판정, 유지
            self._publish_lidar(msg.header.stamp)
            return

        if self._edges is None:
            self._edges = [y_min, y_max]
        else:
            a = self.alpha
            self._edges[0] += a * (y_min - self._edges[0])
            self._edges[1] += a * (y_max - self._edges[1])
        self._last_est = self.get_clock().now().nanoseconds * 1e-9
        if not self._logged_first:
            self._logged_first = True
            self.get_logger().info(
                f'lidar band first estimate: y=[{y_min:.2f}, {y_max:.2f}] '
                f'width={y_max - y_min:.2f}m ref_h={ref_h:.2f}')
        self._publish_lidar(msg.header.stamp)

    def _publish_lidar(self, stamp):
        if self._edges is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_est is not None and now - self._last_est > self.stale_hold:
            return                     # 오래 실패 → 발행 중단 (소비자 폴백)
        y0, y1 = self._edges
        x0, x1 = self.poly_x
        msg = PolygonStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            msg.polygon.points.append(Point32(x=float(x), y=float(y), z=0.0))
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
