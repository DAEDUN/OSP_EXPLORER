"""ROS 의존성 없는 순수 계산 함수 모음 (단위 테스트 가능)."""

import math

import numpy as np


# ---------------------------------------------------------------- 좌표 변환
def quat_to_rot(x, y, z, w):
    """쿼터니언 -> 3x3 회전행렬."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_to_matrix(transform):
    """geometry_msgs/Transform -> (R, t)."""
    q = transform.rotation
    t = transform.translation
    return quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])


def apply_tf(R, t, p):
    return R @ np.asarray(p, dtype=float) + t


def yaw_from_quat(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# ---------------------------------------------------------------- 카메라 / 라이다
def pixel_ray(u, v, K):
    """픽셀 (u, v) -> 카메라 optical frame 방향벡터. K는 camera_info.k (9개)."""
    fx, cx, fy, cy = K[0], K[2], K[4], K[5]
    return np.array([(u - cx) / fx, (v - cy) / fy, 1.0])


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """LaserScan -> laser frame 기준 (N, 2) 점들."""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    return np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], axis=1)


def ray_hit(points, origin, direction, perp_tol=0.06, perp_slope=0.03,
            min_dist=0.15, max_dist=4.0):
    """origin에서 direction 방향(2D)으로 쏜 반직선이 처음 닿는 라이다 점.

    마커 크기를 몰라 카메라만으로는 거리를 알 수 없으므로,
    카메라가 본 '방향'과 라이다가 잰 '거리'를 합쳐 위치를 구한다.
    반환: 닿은 점 근처 점들의 중앙값 (2,) 또는 None.
    """
    if len(points) == 0:
        return None
    d = np.asarray(direction, dtype=float)[:2]
    n = np.linalg.norm(d)
    if n < 1e-9:
        return None
    d = d / n
    v = points - np.asarray(origin, dtype=float)[:2]
    along = v @ d
    perp = np.abs(v[:, 0] * d[1] - v[:, 1] * d[0])
    mask = (along > min_dist) & (along < max_dist) & (perp < perp_tol + perp_slope * along)
    if not np.any(mask):
        return None
    first = along[mask].min()
    near = mask & (along < first + 0.10)          # 첫 표면 근처 점만 사용
    return np.median(points[near], axis=0)

