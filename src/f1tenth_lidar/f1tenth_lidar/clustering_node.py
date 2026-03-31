#!/usr/bin/env python3
"""
LiDAR Clustering Node
======================
LaserScan 데이터를 구독해서 DBSCAN 기반 클러스터링을 수행합니다.
각 클러스터의 중심점(centroid)과 바운딩박스를 MarkerArray로 퍼블리시합니다.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


def dbscan(points: np.ndarray, eps: float, min_samples: int):
    """
    Numpy 기반 DBSCAN 구현 (sklearn 미사용).

    Args:
        points: (N, 2) Cartesian 좌표 배열
        eps: 이웃 반경 (m)
        min_samples: core point 최소 이웃 수

    Returns:
        labels: (N,) int 배열. -1 = noise, 0~ = cluster id
    """
    n = len(points)
    labels = np.full(n, -2, dtype=int)  # -2: 미방문
    cluster_id = 0

    # 거리 행렬 계산 (N이 작을 경우 효율적)
    diff = points[:, None, :] - points[None, :, :]   # (N, N, 2)
    dist_matrix = np.sqrt((diff ** 2).sum(axis=2))   # (N, N)

    def region_query(idx):
        return np.where(dist_matrix[idx] <= eps)[0]

    def expand_cluster(idx, neighbors, cid):
        labels[idx] = cid
        i = 0
        while i < len(neighbors):
            nb = neighbors[i]
            if labels[nb] == -2:          # 미방문
                labels[nb] = cid
                new_neighbors = region_query(nb)
                if len(new_neighbors) >= min_samples:
                    neighbors = np.union1d(neighbors, new_neighbors)
            elif labels[nb] == -1:        # 이전에 노이즈 판정됐던 점
                labels[nb] = cid
            i += 1

    for idx in range(n):
        if labels[idx] != -2:
            continue
        neighbors = region_query(idx)
        if len(neighbors) < min_samples:
            labels[idx] = -1  # noise
        else:
            expand_cluster(idx, neighbors, cluster_id)
            cluster_id += 1

    return labels


class LidarClusteringNode(Node):
    def __init__(self):
        super().__init__('lidar_clustering_node')

        # ── 파라미터 선언 ──────────────────────────────────────────────────────
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('clusters_marker_topic', '/lidar_clusters')
        self.declare_parameter('frame_id', 'laser')

        # 범위 필터
        self.declare_parameter('min_range', 0.05)    # m
        self.declare_parameter('max_range', 10.0)    # m

        # DBSCAN 파라미터
        self.declare_parameter('cluster_tolerance', 0.15)  # eps (m)
        self.declare_parameter('min_cluster_size', 3)      # min_samples
        self.declare_parameter('max_cluster_size', 500)    # 너무 큰 클러스터 제외

        # 시각화
        self.declare_parameter('marker_lifetime', 0.2)     # sec
        self.declare_parameter('show_bbox', True)
        self.declare_parameter('show_centroid', True)
        self.declare_parameter('show_id_text', True)

        self._load_params()

        # ── Pub / Sub ──────────────────────────────────────────────────────────
        scan_topic = self.get_parameter('scan_topic').value
        marker_topic = self.get_parameter('clusters_marker_topic').value

        self.sub_scan = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, marker_topic, 10)

        self.get_logger().info(
            f'LiDAR Clustering Node 시작\n'
            f'  scan topic    : {scan_topic}\n'
            f'  marker topic  : {marker_topic}\n'
            f'  eps           : {self.eps} m\n'
            f'  min_samples   : {self.min_samples}\n'
            f'  range         : [{self.min_range}, {self.max_range}] m'
        )

        # 파라미터 동적 변경 콜백
        self.add_on_set_parameters_callback(self._on_param_change)

    # ── 파라미터 로드 ──────────────────────────────────────────────────────────
    def _load_params(self):
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.eps = self.get_parameter('cluster_tolerance').value
        self.min_samples = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        self.show_bbox = self.get_parameter('show_bbox').value
        self.show_centroid = self.get_parameter('show_centroid').value
        self.show_id_text = self.get_parameter('show_id_text').value
        self.frame_id = self.get_parameter('frame_id').value

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        self._load_params()
        self.get_logger().info('파라미터 업데이트됨')
        return SetParametersResult(successful=True)

    # ── LaserScan 콜백 ─────────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        points = self._scan_to_cartesian(msg)
        if len(points) < self.min_samples:
            self.pub_markers.publish(MarkerArray())
            return

        labels = dbscan(points, self.eps, self.min_samples)
        clusters = self._extract_clusters(points, labels)
        markers = self._build_markers(clusters, msg.header)
        self.pub_markers.publish(markers)

    # ── LaserScan → Cartesian 변환 ─────────────────────────────────────────────
    def _scan_to_cartesian(self, msg: LaserScan) -> np.ndarray:
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        ranges = np.array(msg.ranges, dtype=np.float32)

        valid = np.isfinite(ranges) & (ranges >= self.min_range) & (ranges <= self.max_range)
        r = ranges[valid]
        a = angles[valid]

        x = r * np.cos(a)
        y = r * np.sin(a)
        return np.column_stack((x, y))

    # ── 클러스터 추출 ──────────────────────────────────────────────────────────
    def _extract_clusters(self, points: np.ndarray, labels: np.ndarray):
        """Returns list of dicts: {centroid, points, bbox_min, bbox_max}"""
        clusters = []
        unique_ids = set(labels) - {-1}
        for cid in sorted(unique_ids):
            mask = labels == cid
            cluster_pts = points[mask]
            n = len(cluster_pts)
            if n < self.min_samples or n > self.max_cluster_size:
                continue
            centroid = cluster_pts.mean(axis=0)
            bbox_min = cluster_pts.min(axis=0)
            bbox_max = cluster_pts.max(axis=0)
            clusters.append({
                'id': cid,
                'points': cluster_pts,
                'centroid': centroid,
                'bbox_min': bbox_min,
                'bbox_max': bbox_max,
                'size': n,
            })
        return clusters

    # ── MarkerArray 생성 ───────────────────────────────────────────────────────
    def _build_markers(self, clusters, header) -> MarkerArray:
        ma = MarkerArray()
        colors = self._get_colors()

        for i, cl in enumerate(clusters):
            color = colors[i % len(colors)]
            stamp = header.stamp
            frame = self.frame_id

            if self.show_bbox:
                ma.markers.append(
                    self._bbox_marker(i * 3, stamp, frame, cl, color))

            if self.show_centroid:
                ma.markers.append(
                    self._sphere_marker(i * 3 + 1, stamp, frame, cl, color))

            if self.show_id_text:
                ma.markers.append(
                    self._text_marker(i * 3 + 2, stamp, frame, cl, color))

        # 이전 마커 삭제용 DELETE_ALL (첫 번째 마커로 추가)
        delete_all = Marker()
        delete_all.header.stamp = header.stamp
        delete_all.header.frame_id = self.frame_id
        delete_all.action = Marker.DELETEALL
        ma.markers.insert(0, delete_all)

        return ma

    def _bbox_marker(self, mid, stamp, frame, cl, color) -> Marker:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.ns = 'bbox'
        m.id = mid
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color = color

        from rclpy.duration import Duration
        m.lifetime = Duration(seconds=self.marker_lifetime).to_msg()

        mn = cl['bbox_min']
        mx = cl['bbox_max']
        # 2D 사각형 8개 선분
        corners = [
            (mn[0], mn[1], 0.0),
            (mx[0], mn[1], 0.0),
            (mx[0], mx[1], 0.0),
            (mn[0], mx[1], 0.0),
        ]
        edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
        for a, b in edges:
            pa, pb = corners[a], corners[b]
            m.points.append(Point(x=pa[0], y=pa[1], z=pa[2]))
            m.points.append(Point(x=pb[0], y=pb[1], z=pb[2]))
        return m

    def _sphere_marker(self, mid, stamp, frame, cl, color) -> Marker:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.ns = 'centroid'
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        cx, cy = cl['centroid']
        m.pose.position.x = float(cx)
        m.pose.position.y = float(cy)
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color = color

        from rclpy.duration import Duration
        m.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
        return m

    def _text_marker(self, mid, stamp, frame, cl, color) -> Marker:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.ns = 'label'
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        cx, cy = cl['centroid']
        m.pose.position.x = float(cx)
        m.pose.position.y = float(cy)
        m.pose.position.z = 0.15
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.text = f'C{cl["id"]} (n={cl["size"]})'

        from rclpy.duration import Duration
        m.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
        return m

    @staticmethod
    def _get_colors():
        palette = [
            (1.0, 0.2, 0.2),
            (0.2, 1.0, 0.2),
            (0.2, 0.4, 1.0),
            (1.0, 0.8, 0.0),
            (0.8, 0.2, 1.0),
            (0.2, 1.0, 1.0),
            (1.0, 0.5, 0.0),
            (0.5, 1.0, 0.5),
        ]
        colors = []
        for r, g, b in palette:
            c = ColorRGBA()
            c.r = r
            c.g = g
            c.b = b
            c.a = 0.85
            colors.append(c)
        return colors


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
