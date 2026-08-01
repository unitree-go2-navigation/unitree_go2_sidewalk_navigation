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


def fit_ground_plane(pts):
    """근거리 지면 점들로 평면 z = ax + by + c 최소제곱 적합.

    보행 중 몸통 피치/롤이 base_link 높이 프로파일을 통째로 기울여
    (1° ≈ 8m 횡거리에서 0.14m — 연석 문턱 0.10 초과) 경계가 요동하는
    것을 막는 핵심. 표본 부족 시 None.
    """
    if pts.shape[0] < 20:
        return None
    A = np.column_stack([pts[:, 0], pts[:, 1], np.ones(pts.shape[0])])
    coef, *_ = np.linalg.lstsq(A, pts[:, 2], rcond=None)
    return coef  # (a, b, c)


class EdgeTracker:
    """밴드 경계 1개의 시간 평활 — 히스테리시스 + 변화율 제한.

    작은 편차(<jump)는 느린 EMA로 흡수, 큰 편차는 confirm 프레임 연속
    같은 방향일 때만 rate 한도로 이동 (원거리 표본 깜빡임이 만드는
    프레임 단위 수 m 점프 차단).
    """

    def __init__(self, init, alpha=0.15, jump=0.15, confirm=3, rate=0.3):
        self.v = float(init)
        self.alpha, self.jump, self.confirm, self.rate = alpha, jump, confirm, rate
        self._pend_dir = 0
        self._pend_n = 0

    def update(self, new):
        d = new - self.v
        if abs(d) < self.jump:
            self.v += self.alpha * d
            self._pend_dir, self._pend_n = 0, 0
        else:
            direction = 1 if d > 0 else -1
            if direction == self._pend_dir:
                self._pend_n += 1
            else:
                self._pend_dir, self._pend_n = direction, 1
            if self._pend_n >= self.confirm:
                self.v += max(-self.rate, min(self.rate, d))
        return self.v


def band_to_odom(a, b_lo, b_hi, rx, ry, ryaw):
    """base 좌표 밴드 직선(기울기 a, 절편 b_lo/b_hi) → odom 모델.

    반환 (ux, uy, c_lo, c_hi): 밴드 방향 단위벡터(odom) + 각 경계선의
    법선 오프셋 (점P가 경계선 위 ⇔ P·n = c, n = (-uy, ux)).
    요잉과 무관한 좌표에서 추적하기 위한 표현 (2026-07-30 v3).
    """
    phi = ryaw + math.atan(a)
    ux, uy = math.cos(phi), math.sin(phi)
    cr, sr = math.cos(ryaw), math.sin(ryaw)
    out = []
    for b in (b_lo, b_hi):
        px = rx + (-sr) * b          # base (0, b) → odom
        py = ry + cr * b
        out.append(-uy * px + ux * py)   # P·n
    return ux, uy, out[0], out[1]


def band_to_base(ux, uy, c_lo, c_hi, rx, ry, ryaw, x0, x1):
    """odom 밴드 모델 → 현재 base 좌표 꼭짓점 4개 [(x,y)...].

    로봇의 밴드 방향 투영점 기준 전방 x0~x1 구간의 평행사변형.
    """
    s_r = rx * ux + ry * uy
    cr, sr = math.cos(ryaw), math.sin(ryaw)
    lo, hi = min(c_lo, c_hi), max(c_lo, c_hi)
    pts = []
    for s, c in ((s_r + x0, lo), (s_r + x1, lo),
                 (s_r + x1, hi), (s_r + x0, hi)):
        px = ux * s - uy * c
        py = uy * s + ux * c
        dx, dy = px - rx, py - ry
        pts.append((cr * dx + sr * dy, -sr * dx + cr * dy))
    return pts


def fit_band_lines(slice_x, lo_list, hi_list, slope_max=0.6):
    """x 구간별 좌/우 경계 표본 → 공통 기울기 직선 (a, b_lo, b_hi).

    보도 방향을 로봇 머리가 아니라 연석 자체에서 추정 (2026-07-30 —
    요잉 시 밴드가 몸과 같이 돌아 대각으로 보이던 문제). 양측 기울기
    평균(평행 밴드 가정) 후 절편 재산출. 표본 2개 미만이면 기울기 0
    (기존 단일 추정 동작). NaN 표본은 측별로 제외.
    """
    def side_fit(vals):
        xs = [x for x, v in zip(slice_x, vals) if not math.isnan(v)]
        ys = [v for v in vals if not math.isnan(v)]
        if len(xs) < 2:
            return None, (ys[0] if ys else math.nan)
        a, b = np.polyfit(xs, ys, 1)
        return float(a), float(b)
    a_lo, b_lo = side_fit(lo_list)
    a_hi, b_hi = side_fit(hi_list)
    slopes = [a for a in (a_lo, a_hi) if a is not None]
    if not slopes:
        return 0.0, b_lo, b_hi
    a = max(-slope_max, min(slope_max, sum(slopes) / len(slopes)))

    def intercept(vals):
        pts = [(x, v) for x, v in zip(slice_x, vals) if not math.isnan(v)]
        if not pts:
            return math.nan
        return sum(v - a * x for x, v in pts) / len(pts)
    return a, intercept(lo_list), intercept(hi_list)


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
        """(경계 y, 양성 검출 여부). 양성 = 연석 낙차/벽을 실제로 봄;
        gap/범위 정지는 '모름'(보수 경계) — 방향 적합에 쓰면 안 됨
        (희소 조각의 gap-stop이 기울기를 오염, 2026-07-30 실측)."""
        edge = bin_y[c]
        gap = 0
        i = c
        while 0 <= i < n:
            h, w = ground_h[i], wall_n[i]
            if w >= wall_min:
                return bin_y[i], True
            if math.isnan(h):
                gap += 1
                if gap > max_gap_bins:
                    return edge, False
            else:
                gap = 0
                if h < ref_h - drop_thresh:
                    return bin_y[i], True
                edge = bin_y[i]
            i += direction
        return edge, False

    lo, lo_pos = scan(-1)
    hi, hi_pos = scan(+1)
    return lo, hi, lo_pos, hi_pos


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
        self.declare_parameter('edge_alpha', 0.15)      # 경계 EMA (히스테리시스 동반)
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
            self._edges = None          # 호환 잔재 (v3: _dir/_offs 사용)
            self._dir = None            # 밴드 방향 단위벡터 (odom)
            self._offs = None           # 경계 법선 오프셋 추적기 2개 (odom)
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

        # 지면 평면 보정: 근거리(|y|≤1.2) 지면 점으로 평면 적합 →
        # 이후 높이는 전부 '평면 대비 상대 높이' (피치/롤 오차 제거)
        z = pts[:, 2]
        near_g = pts[(np.abs(pts[:, 1]) <= 1.2) & (z > -0.6) & (z < 0.0)]
        plane = fit_ground_plane(near_g)
        if plane is None:
            self._publish_lidar(msg.header.stamp)
            return
        a_, b_, c_ = plane
        h = z - (a_ * pts[:, 0] + b_ * pts[:, 1] + c_)

        nb = int(2 * self.lat_range / self.bin_size)
        bin_y = -self.lat_range + (np.arange(nb) + 0.5) * self.bin_size
        gmask = np.abs(h) < 0.35          # 평면 대비 지면 대역
        wmask = (h > self.wall_z[0]) & (h < self.wall_z[1])

        def edges_of(sel_slice, positive_only=True):
            """점 부분집합(전방 x 구간)의 좌/우 경계. 표본 부족 시 NaN.
            positive_only: 양성 검출(연석/벽 실견)만 반환 — 방향 적합용.
            False면 gap-stop 보수 경계도 반환 — 절편 폴백용 (원거리
            연석은 조각 표본으로 양성이 안 잡히는 w7 실측)."""
            if np.count_nonzero(sel_slice) < 20:
                return math.nan, math.nan
            idx = np.clip(((pts[sel_slice, 1] + self.lat_range)
                           / self.bin_size).astype(int), 0, nb - 1)
            hh = h[sel_slice]
            gm = gmask[sel_slice]
            wm = wmask[sel_slice]
            ground_h = np.full(nb, np.nan)
            wall_n = np.zeros(nb, dtype=int)
            for b in range(nb):
                bs = idx == b
                gz = hh[bs & gm]
                if gz.size >= 3:
                    ground_h[b] = float(np.median(gz))
                wall_n[b] = int(np.count_nonzero(bs & wm))
            # 평면 보정 후 기준 높이 = 0 (평면 자체가 발밑 지면)
            lo, hi, lo_pos, hi_pos = find_band_edges(
                bin_y, ground_h, wall_n, 0.0,
                drop_thresh=self.curb_drop, wall_min=self.wall_min)
            if positive_only:
                return (lo if lo_pos else math.nan,
                        hi if hi_pos else math.nan)
            return lo, hi

        # 전방 스트립을 x 구간으로 나눠 구간별 경계 → 연석 '방향' 추정
        # (로봇 머리 기준 고정 → 요잉 시 밴드가 대각으로 도는 문제 해소)
        x0s, x1s = self.strip_x
        n_slice = 4
        bounds = np.linspace(x0s, x1s, n_slice + 1)
        slice_x, lo_l, hi_l = [], [], []
        for i in range(n_slice):
            sel = (pts[:, 0] >= bounds[i]) & (pts[:, 0] < bounds[i + 1])
            lo, hi = edges_of(sel)
            slice_x.append(0.5 * (bounds[i] + bounds[i + 1]))
            lo_l.append(lo)
            hi_l.append(hi)
        a, b_lo, b_hi = fit_band_lines(slice_x, lo_l, hi_l)
        # 양성 검출이 전무한 측은 전체 스트립 집계(보수 gap-stop 포함)로
        # 절편 폴백 — 방향은 오염시키지 않되 폭은 유지 (w7 원거리 연석)
        if math.isnan(b_lo) or math.isnan(b_hi):
            all_sel = np.ones(pts.shape[0], dtype=bool)
            lo_all, hi_all = edges_of(all_sel, positive_only=False)
            x_c = 0.5 * (x0s + x1s)
            if math.isnan(b_lo) and not math.isnan(lo_all):
                b_lo = lo_all - a * x_c
            if math.isnan(b_hi) and not math.isnan(hi_all):
                b_hi = hi_all - a * x_c
        if math.isnan(b_lo) or math.isnan(b_hi) or b_hi - b_lo < 1.0:
            self._publish_lidar(msg.header.stamp)   # 표본 부족/협폭 → 유지
            return

        if self._odom is None:
            return                       # odom 자세 없이는 모델 갱신 불가
        rx, ry, ryaw, _ = self._odom
        ux, uy, c_lo, c_hi = band_to_odom(a, b_lo, b_hi, rx, ry, ryaw)
        if self._dir is None:
            self._dir = [ux, uy]
            self._offs = [EdgeTracker(c_lo, alpha=self.alpha),
                          EdgeTracker(c_hi, alpha=self.alpha)]
        else:
            if ux * self._dir[0] + uy * self._dir[1] < 0:
                ux, uy = -ux, -uy        # 무방향 직선 — 부호 정렬 후 EMA
                c_lo, c_hi = -c_hi, -c_lo
            self._dir[0] += 0.2 * (ux - self._dir[0])
            self._dir[1] += 0.2 * (uy - self._dir[1])
            n = math.hypot(*self._dir)
            self._dir = [self._dir[0] / n, self._dir[1] / n]
            self._offs[0].update(c_lo)
            self._offs[1].update(c_hi)
        self._last_est = self.get_clock().now().nanoseconds * 1e-9
        if not self._logged_first:
            self._logged_first = True
            self.get_logger().info(
                f'lidar band first estimate: y=[{b_lo:.2f}, {b_hi:.2f}] '
                f'width={b_hi - b_lo:.2f}m slope={a:.3f} '
                f'slices lo={[round(v,2) for v in lo_l]} '
                f'hi={[round(v,2) for v in hi_l]}')
        self._publish_lidar(msg.header.stamp)

    def _publish_lidar(self, stamp):
        if self._dir is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_est is not None and now - self._last_est > self.stale_hold:
            return                     # 오래 실패 → 발행 중단 (소비자 폴백)
        if self._dir is None or self._odom is None:
            return
        rx, ry, ryaw, _ = self._odom
        x0, x1 = self.poly_x
        pts4 = band_to_base(self._dir[0], self._dir[1],
                            self._offs[0].v, self._offs[1].v,
                            rx, ry, ryaw, x0, x1)
        msg = PolygonStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        # odom 고정 모델을 현재 자세로 역변환 — 요잉해도 밴드는 보도에 고정
        for x, y in pts4:
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
