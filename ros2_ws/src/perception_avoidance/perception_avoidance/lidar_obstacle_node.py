#!/usr/bin/env python3
"""
LiDAR Obstacle Node (Phase 1 PoC).

Pipeline:
  PointCloud2 (lidar_l1_link)
    → TF to base_link
    → ROI crop
    → self-filter (body/leg box)
    → ground removal (Z cutoff)
    → voxel downsample
    → 2D grid clustering
    → frame-to-frame track association (greedy NN + EMA) → per-cluster velocity
    → publish Detection3DArray (centroid + bbox, v_rel embedded in
      results[0].pose.covariance[0/1], 코리도 내 최근접점 x가 covariance[2],
      코리도 내 점 없으면 -1) + debug PoseArray

covariance[2] (corridor_front_x)를 점 단위로 계산하는 이유: 긴 평행 구조물
(인도변 울타리 등)은 요 오차가 있으면 axis-aligned bbox가 차선 쪽으로 번져
게이트의 bbox 겹침 판정을 오염시킴 (2026-07-10 회귀에서 empty 0/5 원인).
실제 점이 코리도 안에 있을 때만 유효한 front distance를 준다.

Velocities are estimated in base_link (= relative to the robot), which is the
v_rel input for safety_stop_node's relative-velocity TTC.
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

import tf2_ros
from tf2_ros import TransformException


def quat_to_rot_matrix(qx, qy, qz, qw):
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx = qx * qx * s
    yy = qy * qy * s
    zz = qz * qz * s
    xy = qx * qy * s
    xz = qx * qz * s
    yz = qy * qz * s
    wx = qw * qx * s
    wy = qw * qy * s
    wz = qw * qz * s
    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ], dtype=np.float64)


class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_node')

        self.declare_parameter('roi_x_min', 0.5)
        self.declare_parameter('roi_x_max', 15.0)
        self.declare_parameter('roi_y_min', -3.0)
        self.declare_parameter('roi_y_max', 3.0)
        self.declare_parameter('roi_z_min', -0.3)
        self.declare_parameter('roi_z_max', 1.2)
        # ∓0.45 (Phase 3.5): L2 min_range 0.05에서 self box가 하중을 받음 —
        # 몸통 반장 0.35 + 다리 전후 스윙·마운트 돌출 0.10 (self_filter.yaml).
        self.declare_parameter('self_x_min', -0.45)
        self.declare_parameter('self_x_max',  0.45)
        self.declare_parameter('self_y_min', -0.3)
        self.declare_parameter('self_y_max',  0.3)
        self.declare_parameter('self_z_min', -0.5)
        self.declare_parameter('self_z_max',  0.25)
        self.declare_parameter('ground_z', -0.05)
        self.declare_parameter('voxel_size', 0.10)
        self.declare_parameter('cluster_grid', 0.20)
        self.declare_parameter('cluster_min_points', 4)
        # 전방 코리도 반폭 (m) = safety_stop의 robot_half_width + corridor_margin.
        # 두 yaml 간 정합은 test_config_consistency로 고정.
        self.declare_parameter('corridor_half', 0.335)
        self.declare_parameter('points_topic', '/unitree_lidar/points')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('output_topic', '/obstacles/lidar')
        # L2 수평 스캔 5.55Hz와 동일 (Phase 3.5, 10.0→5.55).
        self.declare_parameter('publish_rate', 5.55)
        # Velocity (frame-to-frame tracking) — Phase 1 relative-velocity TTC input.
        self.declare_parameter('vel_topic', '/obstacles/lidar/velocity')
        # 2.2 (Phase 3.5, 1.6→2.2): L2 5.55Hz 프레임당 변위 재유도 (self_filter.yaml).
        self.declare_parameter('assoc_max_dist', 2.2)
        self.declare_parameter('vel_alpha', 0.5)
        self.declare_parameter('vel_min_dt', 0.02)
        # 0.8 (Phase 3.5, 0.5→0.8): 코스팅 상한(track_coast_frames 3) 직후 재연관
        # 시 dtm = 4×T_frame = 0.72s — 이보다 작으면 속도 갱신이 조용히 스킵돼
        # 스테일 속도가 유지됨. (coast+1)×0.18 + 여유.
        self.declare_parameter('vel_max_dt', 0.8)
        # 미매칭 track 유지(코스팅) 한도 (연속 프레임 수). 열화(dropout) 한두
        # 프레임에 track이 죽으면 id·속도가 리셋돼 게이트의 fast 트리거 지속
        # 카운트가 영원히 미달 (real_l1 4m/s STOP 미진입 원인). 코스팅 track은
        # 발행하지 않고 재연관용으로만 유지 — 게이트 의미론(사각 소실 즉시
        # 인지, blind-hold) 불변.
        # ⚠ 6으로 늘리면 스테일 track이 예측 위치로 무관 클러스터를 포획해
        # 유령 fast가 지속됨 (bike_pass false stop 3/5 실측) — 3 유지.
        self.declare_parameter('track_coast_frames', 3)
        # 클라우드 프레임 누적 (최근 K union, 1=끔). ⚠ 기본 OFF: 고속 물체는
        # 프레임 레이어가 cluster_grid보다 멀리 떨어져 각자 클러스터가 되고,
        # 이전 레이어의 "유령"(고정 위치)이 기존 track을 거리 0으로 강탈해
        # 실물 클러스터가 매 프레임 새 id(v=0)를 받는다 (2026-07-16 실측,
        # problem.md). 원거리 희소 보완은 far_cluster_min_points가 담당.
        self.declare_parameter('accum_frames', 1)

        self.target_frame = self.get_parameter('target_frame').value
        self.publish_period = 1.0 / float(self.get_parameter('publish_rate').value)
        self._last_pub = 0.0

        self.assoc_max_dist = self.get_parameter('assoc_max_dist').value
        self.vel_alpha = self.get_parameter('vel_alpha').value
        self.vel_min_dt = self.get_parameter('vel_min_dt').value
        self.vel_max_dt = self.get_parameter('vel_max_dt').value
        self.track_coast = int(self.get_parameter('track_coast_frames').value)
        self._cloud_hist = deque(
            maxlen=max(1, int(self.get_parameter('accum_frames').value)))
        self._frame_seq = 0
        self._tracks = []        # [{id, mx, my, mt, vx, vy, miss}]
        self._next_id = 0
        self._prev_stamp_s = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('points_topic').value,
            self.cloud_callback,
            sensor_qos,
        )
        self.pub = self.create_publisher(
            Detection3DArray,
            self.get_parameter('output_topic').value,
            10,
        )
        self.vel_pub = self.create_publisher(
            PoseArray,
            self.get_parameter('vel_topic').value,
            10,
        )
        # 정제 클라우드 (ROI+self+ground 필터 후, 최신 프레임만) — Nav2
        # 코스트맵 관측용. /scan 브리지가 없어 obstacle_layer가 장님이었음.
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/perception/obstacle_cloud', 10,
        )
        self.get_logger().info(
            f'lidar_obstacle_node started. '
            f'in={self.get_parameter("points_topic").value} → '
            f'out={self.get_parameter("output_topic").value} '
            f'(frame={self.target_frame})'
        )

    def cloud_callback(self, msg: PointCloud2):
        # Throttle to configured publish rate. 0.9 여유: 입력도 publish_period
        # 주기일 때 위상 지터로 매번 '조금 이르게' 도착하면 절반이 스킵돼
        # 실효 5Hz로 반감됨 (P4 자전거 추적에서 실측 — 연관 실패 원인).
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_pub < self.publish_period * 0.9:
            return
        self._last_pub = now

        # Look up transform from cloud frame to base_link
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException as e:
            self.get_logger().warn(
                f'TF {msg.header.frame_id}→{self.target_frame} unavailable: {e}',
                throttle_duration_sec=2.0,
            )
            return

        # Read points as Nx3 numpy array (Jazzy API)
        pts = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = pts.astype(np.float64, copy=False).reshape(-1, 3)
        # 프레임 누적: 최근 K 클라우드 union (accum_frames 주석 참조).
        # union은 클러스터 "성립"(min_points)에만 쓰고, centroid/최근접점은
        # 최신 프레임 포인트로 계산한다 — union centroid는 이동 물체의 trail
        # 중간을 가리켜 속도를 희석시킴 (P4 실측: 4.4m/s 접근이 1.5m/s로
        # 측정돼 fast 임계 미달). 프레임 나이는 4번째 열로 태깅.
        self._frame_seq += 1
        pts = np.hstack(
            [pts, np.full((pts.shape[0], 1), float(self._frame_seq))])
        self._cloud_hist.append(pts)
        if len(self._cloud_hist) > 1:
            # 최신 프레임 먼저 — voxel dedup(unique 첫 등장 유지)이 같은
            # voxel에서 최신 포인트를 남기도록
            pts = np.concatenate(list(self._cloud_hist)[::-1], axis=0)
        if pts.size == 0:
            return
        ages = pts[:, 3]
        pts = pts[:, :3]

        # Transform to base_link
        t = tf.transform.translation
        r = tf.transform.rotation
        R = quat_to_rot_matrix(r.x, r.y, r.z, r.w)
        T = np.array([t.x, t.y, t.z], dtype=np.float64)
        pts_b = pts @ R.T + T

        # ROI crop
        rx_min = self.get_parameter('roi_x_min').value
        rx_max = self.get_parameter('roi_x_max').value
        ry_min = self.get_parameter('roi_y_min').value
        ry_max = self.get_parameter('roi_y_max').value
        rz_min = self.get_parameter('roi_z_min').value
        rz_max = self.get_parameter('roi_z_max').value
        mask = (
            (pts_b[:, 0] >= rx_min) & (pts_b[:, 0] <= rx_max) &
            (pts_b[:, 1] >= ry_min) & (pts_b[:, 1] <= ry_max) &
            (pts_b[:, 2] >= rz_min) & (pts_b[:, 2] <= rz_max)
        )
        pts_b = pts_b[mask]
        ages = ages[mask]
        if pts_b.shape[0] == 0:
            self._publish_empty(msg.header.stamp)
            return

        # Self-filter
        sx_min = self.get_parameter('self_x_min').value
        sx_max = self.get_parameter('self_x_max').value
        sy_min = self.get_parameter('self_y_min').value
        sy_max = self.get_parameter('self_y_max').value
        sz_min = self.get_parameter('self_z_min').value
        sz_max = self.get_parameter('self_z_max').value
        in_self = (
            (pts_b[:, 0] >= sx_min) & (pts_b[:, 0] <= sx_max) &
            (pts_b[:, 1] >= sy_min) & (pts_b[:, 1] <= sy_max) &
            (pts_b[:, 2] >= sz_min) & (pts_b[:, 2] <= sz_max)
        )
        pts_b = pts_b[~in_self]
        ages = ages[~in_self]

        # Ground removal
        ground_z = self.get_parameter('ground_z').value
        above = pts_b[:, 2] > ground_z
        pts_b = pts_b[above]
        ages = ages[above]
        if pts_b.shape[0] == 0:
            self._publish_empty(msg.header.stamp)
            return

        # 정제 클라우드 발행 (누적 이력 제외 — mover trail이 코스트맵에
        # 잔상으로 남는 것 방지)
        latest = pts_b[ages == float(self._frame_seq)]
        hdr = Header()
        hdr.stamp = msg.header.stamp
        hdr.frame_id = self.target_frame
        self.cloud_pub.publish(
            pc2.create_cloud_xyz32(hdr, latest.astype(np.float32)))

        # Voxel downsample (3D grid) — 입력이 최신 프레임 우선 정렬이라
        # 같은 voxel에서는 최신 포인트가 남는다
        voxel = self.get_parameter('voxel_size').value
        keys = np.floor(pts_b / voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        pts_b = pts_b[idx]
        ages = ages[idx]

        # 2D grid-based clustering (project to xy)
        grid = self.get_parameter('cluster_grid').value
        min_pts = int(self.get_parameter('cluster_min_points').value)
        cells = np.floor(pts_b[:, :2] / grid).astype(np.int64)
        # Group cells via dict
        cell_map = {}
        for i, (cx, cy) in enumerate(cells):
            cell_map.setdefault((int(cx), int(cy)), []).append(i)

        visited = set()
        clusters = []
        for cell in cell_map.keys():
            if cell in visited:
                continue
            # BFS over 4-connected cells (xy plane)
            stack = [cell]
            members = []
            while stack:
                cur = stack.pop()
                if cur in visited:
                    continue
                visited.add(cur)
                if cur not in cell_map:
                    continue
                members.extend(cell_map[cur])
                cx, cy = cur
                for dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nb = (cx + dc[0], cy + dc[1])
                    if nb in cell_map and nb not in visited:
                        stack.append(nb)
            clusters.append(members)

        # Per-cluster bbox + center. The center is the tracking/velocity anchor
        # and matches the p_rel anchor safety_stop uses.
        corridor_half = self.get_parameter('corridor_half').value
        centers = []
        bboxes = []
        for member_idx in clusters:
            if len(member_idx) < min_pts:
                # ⚠ 원거리 완화(>8m 2점) 시도는 폐기 — 2점 클러스터의 centroid
                # 요동이 지속성 필터를 뚫는 가짜 fast 물체가 됨 (empty 유령
                # STOP 6회, problem.md 2026-07-16)
                continue
            cluster_pts = pts_b[member_idx]
            cluster_ages = ages[member_idx]
            # 유령 방지 가드(누적 사용 시): 현재 프레임 포인트가 하나도 없는
            # 클러스터(과거 레이어 잔상)는 폐기 — 고정 위치 유령이 기존
            # track을 강탈해 실물이 매 프레임 새 id를 받는 것 방지
            if cluster_ages.max() != self._frame_seq:
                continue
            # anchor = 최신 프레임 포인트만 — trail이 centroid를 과거로
            # 끌어 속도를 희석하지 않도록 (성립 판정만 union 기준)
            newest = cluster_ages == cluster_ages.max()
            anchor_pts = cluster_pts[newest]
            mn = anchor_pts.min(axis=0)
            mx = anchor_pts.max(axis=0)
            ctr = (mn + mx) * 0.5
            size = np.maximum(mx - mn, 0.05)
            # 코리도 내 최근접점 x (점 없으면 -1): 게이트의 front clearance 근거
            in_band = np.abs(anchor_pts[:, 1]) <= corridor_half
            front_x = float(anchor_pts[in_band, 0].min()) if in_band.any() else -1.0
            centers.append((float(ctr[0]), float(ctr[1]), float(ctr[2])))
            bboxes.append((ctr, size, front_x))

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tracks = self._associate(centers, stamp_s)

        # Build Detection3DArray (positions) + PoseArray (index-aligned velocity)
        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame
        vel_out = PoseArray()
        vel_out.header = out.header

        for (ctr, size, front_x), (tid, vx, vy) in zip(bboxes, tracks):
            det = Detection3D()
            det.header = out.header
            det.bbox = BoundingBox3D()
            det.bbox.center.position.x = float(ctr[0])
            det.bbox.center.position.y = float(ctr[1])
            det.bbox.center.position.z = float(ctr[2])
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x = float(size[0])
            det.bbox.size.y = float(size[1])
            det.bbox.size.z = float(size[2])
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'unknown'
            hyp.hypothesis.score = 1.0
            hyp.pose.pose = det.bbox.center
            # Embed relative velocity (base_link) so it travels atomically with
            # the detection: covariance[0]=vx, covariance[1]=vy. Avoids the
            # cross-topic ordering race a separate velocity topic suffers from
            # under a single-threaded executor.
            # covariance[2] = 코리도 내 최근접점 x (없으면 -1) — 모듈 docstring 참조.
            hyp.pose.covariance[0] = float(vx)
            hyp.pose.covariance[1] = float(vy)
            hyp.pose.covariance[2] = float(front_x)
            det.results.append(hyp)
            det.id = str(tid)
            out.detections.append(det)

            vp = Pose()
            vp.position.x = float(vx)
            vp.position.y = float(vy)
            vp.position.z = 0.0
            vp.orientation.w = 1.0
            vel_out.poses.append(vp)

        # DEBUG: log nearest cluster within forward cone (±45°, x>0)
        nearest = None
        for det in out.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            cz = det.bbox.center.position.z
            if cx <= 0.0 or abs(math.atan2(cy, cx)) > 0.7854:
                continue
            d = math.hypot(cx, cy)
            if nearest is None or d < nearest[0]:
                nearest = (d, cx, cy, cz, det.bbox.size.x, det.bbox.size.y)
        if nearest is not None:
            self.get_logger().info(
                f'nearest fwd cluster: dist={nearest[0]:.2f}m '
                f'pos=({nearest[1]:.2f},{nearest[2]:.2f},{nearest[3]:.2f}) '
                f'size=({nearest[4]:.2f}x{nearest[5]:.2f})',
                throttle_duration_sec=1.0)

        self.pub.publish(out)
        self.vel_pub.publish(vel_out)

    def _associate(self, centers, stamp_s):
        """Greedy nearest-neighbor tracking → (track_id, vx, vy) per center.

        Velocities are in base_link, i.e. relative to the robot (centroids are
        tracked in the robot frame), which is exactly v_rel for the
        closing-speed TTC computed downstream.

        매칭은 track의 CVM 예측 위치(마지막 실측 mx + vx·경과시간) 기준 —
        고속 물체(자전거 ~5m/s)는 프레임당 변위가 게이트에 육박해 마지막
        위치 기준 NN이 끊긴다 (P4 실측: id churn → vx EMA 재수렴 반복).
        미매칭 track은 track_coast_frames까지 유지(코스팅, 미발행) — 열화
        dropout 한두 프레임에 id·속도가 리셋되지 않도록. 속도는 항상 실측
        위치·시각(mx, mt) 간 변위로 계산해 예측 누적 오차가 섞이지 않는다.
        """
        pairs = []
        for i, (cx, cy, _) in enumerate(centers):
            for j, tr in enumerate(self._tracks):
                dtm = stamp_s - tr['mt']
                usable = self.vel_min_dt <= dtm <= self.vel_max_dt
                px = tr['mx'] + (tr['vx'] * dtm if usable else 0.0)
                py = tr['my'] + (tr['vy'] * dtm if usable else 0.0)
                d = math.hypot(cx - px, cy - py)
                if d <= self.assoc_max_dist:
                    pairs.append((d, i, j))
        pairs.sort(key=lambda p: p[0])
        used_cur, used_trk, matched = set(), set(), {}
        for _d, i, j in pairs:
            if i in used_cur or j in used_trk:
                continue
            used_cur.add(i)
            used_trk.add(j)
            matched[i] = j

        results = []
        new_tracks = []
        for i, (cx, cy, _) in enumerate(centers):
            if i in matched:
                tr = self._tracks[matched[i]]
                dtm = stamp_s - tr['mt']
                if self.vel_min_dt <= dtm <= self.vel_max_dt:
                    raw_vx = (cx - tr['mx']) / dtm
                    raw_vy = (cy - tr['my']) / dtm
                    vx = self.vel_alpha * raw_vx + (1.0 - self.vel_alpha) * tr['vx']
                    vy = self.vel_alpha * raw_vy + (1.0 - self.vel_alpha) * tr['vy']
                else:
                    vx, vy = tr['vx'], tr['vy']
                tid = tr['id']
            else:
                vx = vy = 0.0
                tid = self._next_id
                self._next_id += 1
            results.append((tid, vx, vy))
            new_tracks.append({'id': tid, 'mx': cx, 'my': cy, 'mt': stamp_s,
                               'vx': vx, 'vy': vy, 'miss': 0})

        # 코스팅: 이번 프레임에 매칭되지 않은 track을 한도 내에서 유지
        for j, tr in enumerate(self._tracks):
            if j in used_trk:
                continue
            miss = tr.get('miss', 0) + 1
            if miss <= self.track_coast:
                tr['miss'] = miss
                new_tracks.append(tr)

        self._tracks = new_tracks
        self._prev_stamp_s = stamp_s
        return results

    def _publish_empty(self, stamp):
        out = Detection3DArray()
        out.header.stamp = stamp
        out.header.frame_id = self.target_frame
        self.pub.publish(out)
        vel_out = PoseArray()
        vel_out.header = out.header
        self.vel_pub.publish(vel_out)
        # 빈 클라우드도 발행 — 코스트맵 센서 신선도 유지
        hdr = Header()
        hdr.stamp = stamp
        hdr.frame_id = self.target_frame
        self.cloud_pub.publish(
            pc2.create_cloud_xyz32(hdr, np.zeros((0, 3), dtype=np.float32)))
        # track을 지우지 않고 코스팅 경로로 노화 — 빈 프레임(전체 dropout)
        # 한 번에 모든 id·속도가 리셋되지 않도록
        self._associate([], stamp.sec + stamp.nanosec * 1e-9)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
