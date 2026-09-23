"""doll_detect's arena geotag: an NED point through the newest lidar fix."""
import math
import time
from types import SimpleNamespace

import numpy as np

from drone_testing.doll_detect import DollDetect


def _node(ax, ay, arena_yaw, heading, n0=5.0, e0=-3.0):
    rot = arena_yaw - (math.pi / 2.0 - heading)
    return SimpleNamespace(lidar_fix=(ax, ay, rot, n0, e0, time.monotonic()),
                           lidar_max_age=1.0)


def test_facing_into_the_room_arena_is_enu():
    n = _node(1.0, 0.5, math.pi / 2, 0.0)
    p = DollDetect._to_arena(n, np.array([5.3, -3.0, -1.0]))   # 0.3 m north
    assert np.allclose(p[:2], [1.0, 0.8])


def test_rotated_frames_map_the_nose_direction():
    # EKF says north is the nose; the lidar says the nose points along +X.
    n = _node(1.0, 0.5, 0.0, 0.0)
    p = DollDetect._to_arena(n, np.array([5.3, -3.0, -1.0]))
    assert np.allclose(p[:2], [1.3, 0.5])


def test_stale_fix_is_not_placeable():
    n = _node(1.0, 0.5, 0.0, 0.0)
    n.lidar_fix = n.lidar_fix[:5] + (time.monotonic() - 5.0,)
    assert DollDetect._to_arena(n, np.array([5.3, -3.0, -1.0])) is None
