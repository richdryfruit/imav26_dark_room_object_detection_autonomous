"""The three exit cues and the vote, on synthetic frames of a known room."""

import math

import numpy as np

from drone_testing.window_exit import bright_region, depth_hole, lidar_gap, vote

WALL_Y = -1.25          # window wall of a 2.5 m room
FACING = -math.pi / 2   # arena yaw looking at it (-Y)


def _scan(fix, win_x0, win_x1, far=float('inf')):
    """360-bin levelled scan of a straight wall with a hole in it."""
    fx, fy, psi = fix
    inc = 2 * math.pi / 360
    ranges = []
    for i in range(360):
        d = psi + (-math.pi + i * inc)
        s = math.sin(d)
        if s >= -1e-6:
            ranges.append(3.0)      # side/back walls, irrelevant here
            continue
        t = (WALL_Y - fy) / s
        hx = fx + t * math.cos(d)
        ranges.append(far if win_x0 <= hx <= win_x1 else t)
    return ranges, -math.pi, inc


def test_lidar_gap_finds_centre_and_width():
    fix = (0.10, 0.25, FACING)
    r, a0, inc = _scan(fix, -0.20, 0.40)
    g = lidar_gap(r, a0, inc, fix, WALL_Y, min_width=0.3, max_width=1.0)
    assert g is not None
    assert abs(g['x'] - 0.10) < 0.03
    assert abs(g['width'] - 0.60) < 0.06


def test_lidar_gap_ignores_open_edge_and_small_holes():
    fix = (0.0, 0.25, FACING)
    # A gap running off the side of the field of view is not a window.
    r, a0, inc = _scan(fix, 0.0, 10.0)
    assert lidar_gap(r, a0, inc, fix, WALL_Y, min_width=0.3, max_width=1.0) is None
    # A 10 cm slot is too narrow.
    r, a0, inc = _scan(fix, 0.0, 0.10)
    assert lidar_gap(r, a0, inc, fix, WALL_Y, min_width=0.3, max_width=1.0) is None


def test_lidar_gap_far_return_counts_as_through():
    fix = (0.0, 0.25, FACING)
    r, a0, inc = _scan(fix, -0.3, 0.3, far=3.2)
    g = lidar_gap(r, a0, inc, fix, WALL_Y, min_width=0.3, max_width=1.0)
    assert g is not None and abs(g['x']) < 0.03


def _camera_frames(cam, win_x0, win_x1, win_z0, win_z1, w=640, h=480,
                   f=554.0):
    """Depth (m, inf through the window) and grey (bright through it)."""
    cx, cy = w / 2, h / 2
    x, y, psi = cam
    depth = np.zeros((h, w), np.float32)
    gray = np.full((h, w), 30, np.uint8)
    for u in range(w):
        alpha = -math.atan((u - cx) / f)
        d = psi + alpha
        t = (WALL_Y - y) / math.sin(d)
        hx = x + t * math.cos(d)
        z = t * math.cos(alpha)
        for v in range(0, h):
            up = -(v - cy) / f * z
            inside = win_x0 <= hx <= win_x1 and win_z0 <= up <= win_z1
            depth[v, u] = np.inf if inside else z
            if inside:
                gray[v, u] = 200
    return depth, gray, (f, f, cx, cy)


def test_depth_hole_and_brightness_agree_with_truth():
    cam = (0.05, 0.40, FACING)
    depth, gray, K = _camera_frames(cam, -0.25, 0.35, -0.20, 0.40)
    dh = depth_hole(depth, K, cam, WALL_Y, min_width=0.3, max_width=1.0)
    br = bright_region(gray, K, cam, WALL_Y, min_width=0.3, max_width=1.0)
    for c in (dh, br):
        assert c is not None
        assert abs(c['x'] - 0.05) < 0.04
        assert abs(c['width'] - 0.60) < 0.06
        assert abs(c['up'] - 0.10) < 0.05
    assert br['contrast'] > 3.0


def test_truncated_hole_is_rejected():
    cam = (0.0, 0.40, FACING)
    # Window taller than the frame covers: touches top and bottom.
    depth, gray, K = _camera_frames(cam, -0.3, 0.3, -2.0, 2.0)
    assert depth_hole(depth, K, cam, WALL_Y, min_width=0.3, max_width=1.0) is None
    assert bright_region(gray, K, cam, WALL_Y, min_width=0.3, max_width=1.0) is None


def test_vote_needs_two_that_agree():
    a = {'x': 0.10}
    assert vote({'lidar': a, 'depth': None, 'bright': None}, 0.15) == (None, [])
    x, names = vote({'lidar': {'x': 0.10}, 'depth': {'x': 0.16},
                     'bright': {'x': 0.90}}, 0.15)
    assert x == 0.10 and sorted(names) == ['depth', 'lidar']
    x, names = vote({'lidar': None, 'depth': {'x': 0.2}, 'bright': {'x': 0.3}}, 0.15)
    assert abs(x - 0.25) < 1e-9 and len(names) == 2
    # Two cues that disagree are not a window.
    assert vote({'lidar': {'x': 0.0}, 'depth': {'x': 0.5}, 'bright': None},
                0.15) == (None, [])


def _body_scan(d, phi, lo, hi, far=float('inf')):
    """Body-frame scan of a wall at distance d, normal at FLU angle phi, with
    a gap between along-wall offsets lo..hi (+ = left of the foot point)."""
    inc = 2 * math.pi / 360
    out = []
    for i in range(360):
        a = -math.pi + i * inc
        c = math.cos(a - phi)
        if c <= 1e-3:
            out.append(far)
            continue
        t = d / c
        s = t * math.sin(a - phi)
        out.append(far if lo <= s <= hi else t)
    return out, -math.pi, inc


def test_wall_gap_body_square_and_skewed():
    from drone_testing.window_exit import wall_gap_body
    r, a0, inc = _body_scan(2.0, 0.0, -0.40, 0.20)      # gap centred 10 cm right
    g = wall_gap_body(r, a0, inc, min_width=0.4, max_width=1.0)
    assert g is not None
    assert abs(g['d'] - 2.0) < 0.02 and abs(g['phi']) < math.radians(0.5)
    assert abs(g['lateral'] + 0.10) < 0.04 and abs(g['width'] - 0.60) < 0.08
    phi = math.radians(4.0)                              # 4 deg off square
    r, a0, inc = _body_scan(1.5, phi, -0.30, 0.30)
    g = wall_gap_body(r, a0, inc, min_width=0.4, max_width=1.0)
    assert abs(g['phi'] - phi) < math.radians(0.5)
    cx, cy = g['centre']
    assert math.hypot(cx - 1.5 * math.cos(phi), cy - 1.5 * math.sin(phi)) < 0.03


def test_wall_gap_body_none_without_a_gap():
    from drone_testing.window_exit import wall_gap_body
    r, a0, inc = _body_scan(2.0, 0.0, 5.0, 6.0)
    assert wall_gap_body(r, a0, inc, min_width=0.4, max_width=1.0) is None
