"""ArUco 마커를 찾아 map frame 위치를 추정하고 RViz 표시 + 파일 저장.

위치 추정 방법 (마커 크기 비공개 대응):
  1) 카메라로 마커 중심 픽셀 -> 카메라에서 마커로 향하는 '방향' 계산
  2) 그 방향을 라이다 frame으로 옮겨, 같은 방향에서 라이다가 잰 '거리'로 실제 점을 구함
     (공지: 마커 붙은 면은 바닥까지 이어져 2D LiDAR에 잡힘)
  3) TF로 map frame 변환, 여러 번 관측한 값의 중앙값을 최종 위치로 사용
"""

import csv
import os
from collections import defaultdict

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

from osp_explorer.geometry import (tf_to_matrix, apply_tf, pixel_ray,
                                   scan_to_points, ray_hit)


def make_detector():
    """OpenCV 버전 차이(4.6 apt / 4.7+ pip) 모두 지원."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)
    params = cv2.aruco.DetectorParameters_create()
    return lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)


def image_to_gray(msg):
    """cv_bridge 없이 Image -> grayscale numpy."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('mono8', '8UC1'):
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    code = cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(img, code)


class MarkerMapper(Node):
    def __init__(self):
        super().__init__('marker_mapper')
        p = self.declare_parameter
        self.image_topic = p('image_topic', '/oakd/rgb/preview/image_raw').value
        self.info_topic = p('camera_info_topic', '/oakd/rgb/preview/camera_info').value
        self.scan_topic = p('scan_topic', '/scan').value
        self.map_frame = p('map_frame', 'map').value
        self.output_dir = os.path.expanduser(p('output_dir', '~/osp_result').value)
        self.min_detections = p('min_detections', 3).value     # 이 횟수 이상 보여야 확정
        self.max_range = p('max_range', 4.0).value             # 너무 먼 마커는 오차 커서 무시
        self.process_period = p('process_period', 0.2).value   # 영상 처리 주기(초)

        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, 'markers.csv')

        self.detect = make_detector()
        self.K = None
        self.scan = None
        self.last_proc = self.get_clock().now()
        self.obs = defaultdict(list)    # id -> [(x, y), ...]
        self.confirmed = {}             # id -> (x, y)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/aruco_markers', latched)
        self.count_pub = self.create_publisher(Int32, '/aruco_found_count', latched)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(f'marker_mapper started, image={self.image_topic}, '
                               f'output={self.csv_path}')

    # ------------------------------------------------------------ callbacks
    def on_info(self, msg):
        self.K = list(msg.k)

    def on_scan(self, msg):
        self.scan = msg

    def on_image(self, msg):
        now = self.get_clock().now()
        if (now - self.last_proc).nanoseconds < self.process_period * 1e9:
            return
        self.last_proc = now
        if self.K is None or self.scan is None:
            return

        gray = image_to_gray(msg)
        corners, ids, _ = self.detect(gray)
        if ids is None:
            return

        try:
            cam_to_laser = self.tf_buffer.lookup_transform(
                self.scan.header.frame_id, msg.header.frame_id, Time())
            laser_to_map = self.tf_buffer.lookup_transform(
                self.map_frame, self.scan.header.frame_id, Time())
        except Exception as e:  # TF 아직 준비 안 됨
            self.get_logger().warn(f'TF not ready: {e}', throttle_duration_sec=2.0)
            return

        R_cl, t_cl = tf_to_matrix(cam_to_laser.transform)
        R_lm, t_lm = tf_to_matrix(laser_to_map.transform)
        s = self.scan
        pts = scan_to_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)

        for c, marker_id in zip(corners, ids.flatten()):
            u, v = c.reshape(-1, 2).mean(axis=0)
            ray = pixel_ray(u, v, self.K)
            o_l = apply_tf(R_cl, t_cl, [0, 0, 0])
            d_l = apply_tf(R_cl, t_cl, ray) - o_l
            hit = ray_hit(pts, o_l, d_l, max_dist=self.max_range)
            if hit is None:
                continue
            xm, ym, _ = apply_tf(R_lm, t_lm, [hit[0], hit[1], 0.0])
            self.add_observation(int(marker_id), xm, ym)

    # ------------------------------------------------------------ bookkeeping
    def add_observation(self, mid, x, y):
        self.obs[mid].append((x, y))
        self.obs[mid] = self.obs[mid][-30:]       # 최근 30개만 유지
        if len(self.obs[mid]) < self.min_detections:
            return
        est = tuple(np.median(np.array(self.obs[mid]), axis=0))
        is_new = mid not in self.confirmed
        self.confirmed[mid] = est
        if is_new:
            self.get_logger().info(
                f'*** FOUND marker ID {mid} at ({est[0]:.2f}, {est[1]:.2f}) '
                f'[{len(self.confirmed)} found]')
        self.save()

    def save(self):
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['id', 'x', 'y'])
            for mid in sorted(self.confirmed):
                x, y = self.confirmed[mid]
                w.writerow([mid, f'{x:.3f}', f'{y:.3f}'])

    def publish_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (mid, (x, y)) in enumerate(sorted(self.confirmed.items())):
            for kind in ('sphere', 'text'):
                m = Marker()
                m.header.frame_id = self.map_frame
                m.header.stamp = stamp
                m.ns = 'aruco_' + kind
                m.id = mid
                m.action = Marker.ADD
                m.pose.position.x = float(x)
                m.pose.position.y = float(y)
                m.pose.orientation.w = 1.0
                if kind == 'sphere':
                    m.type = Marker.SPHERE
                    m.pose.position.z = 0.15
                    m.scale.x = m.scale.y = m.scale.z = 0.25
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.9
                else:
                    m.type = Marker.TEXT_VIEW_FACING
                    m.pose.position.z = 0.55
                    m.scale.z = 0.3
                    m.text = f'ID {mid}'
                    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                arr.markers.append(m)
        self.marker_pub.publish(arr)
        self.count_pub.publish(Int32(data=len(self.confirmed)))


def main(args=None):
    rclpy.init(args=args)
    node = MarkerMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
