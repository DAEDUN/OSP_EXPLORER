import math
import sys
import os

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from osp_explorer.geometry import (quat_to_rot, pixel_ray, scan_to_points, ray_hit)  # noqa


def test_ray_hit_with_camera_offset():
    # 벽: x = 2.0 (laser frame), 라이다 360도 스캔 시뮬레이션
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    ranges = []
    for a in angles:
        c = math.cos(a)
        ranges.append(2.0 / c if c > 1e-3 else float('inf'))
    pts = scan_to_points(ranges, angles[0], angles[1] - angles[0], 0.1, 12.0)
    # 카메라가 라이다보다 0.1m 앞, 0.05m 왼쪽. 마커는 벽 위 (2.0, 0.6)
    origin = np.array([0.1, 0.05])
    target = np.array([2.0, 0.6])
    hit = ray_hit(pts, origin, target - origin)
    assert hit is not None
    assert np.linalg.norm(hit - target) < 0.05, hit


def test_pixel_ray_center():
    K = [500, 0, 320, 0, 500, 240, 0, 0, 1]
    r = pixel_ray(320, 240, K)
    assert np.allclose(r, [0, 0, 1])


def test_quat_identity_and_yaw90():
    assert np.allclose(quat_to_rot(0, 0, 0, 1), np.eye(3))
    R = quat_to_rot(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])


def test_aruco_detection_original_dict():
    import cv2
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    img = cv2.aruco.generateImageMarker(d, 7, 200)
    img = cv2.copyMakeBorder(img, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
    det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(img)
    assert ids is not None and int(np.array(ids).flatten()[0]) == 7
