"""
Finding the window again FROM INSIDE the dark room, where it is not coloured.

The blue band is on the outside face only, so window_detect cannot see the
aperture on the way out. Three independent cues can, and the exit flies only
when at least two of them agree on where it is:

    LIDAR GAP     the 2D lidar's scan plane passes through the opening, so the
                  window wall has a hole in it: beams that should hit the wall
                  plane come back far beyond it, or not at all.
    DEPTH HOLE    the front depth camera sees the wall at a known distance; the
                  opening is a compact region of pixels much FARTHER than that
                  (or with no return at all: the arena beyond is out of range).
    BRIGHTNESS    the room is dark and the arena beyond is lit, so the opening
                  is a compact region clearly brighter than the rest of the
                  frame.

Each cue is a pure function of one sensor frame plus where the aircraft is in
the ARENA frame (room_lidar_scan's: room centre at the origin, the window wall
along +X at y = wall_y, i.e. -room_y/2). Each returns the window's centre as an
arena x on that wall, and its width in metres, or None. vote() then asks for a
cluster of at least `need` cues within a tolerance of each other.

Everything here is geometry on the measured wall line, not on EKF2: the arena
fix comes from wall_localizer, which re-derives it from the walls every scan.

ANGLES: the levelled scan (/lidar/scan_level, frame base_link_level) is FLU --
0 rad = forward, positive = LEFT. Arena yaw is ENU-like, counter-clockwise from
+X. A beam at body angle a therefore points along arena yaw psi + a.
"""

import math

import numpy as np


def _wall_hit(px, py, direction, wall_y):
    """Distance along `direction` from (px, py) to the line y = wall_y, and
    the arena x where it lands. (None, None) if the ray never reaches it."""
    s = math.sin(direction)
    if abs(s) < 1e-6:
        return None, None
    t = (wall_y - py) / s
    if t <= 0.0:
        return None, None
    return t, px + t * math.cos(direction)


# ------------------------------------------------------------- lidar gap

def lidar_gap(ranges, angle_min, angle_inc, fix, wall_y, *, fov=math.radians(60.0),
              margin=0.30, min_width=0.4, max_width=2.0, max_range=float('inf')):
    """The window as a run of beams that pass THROUGH the wall line.

    fix: (x, y, yaw) of the lidar in the arena frame.
    A beam is 'through' if it returns nothing (inf/NaN, or at/after
    max_range) or returns more than `margin` beyond the wall line; it is
    'wall' if it lands within `margin` of the wall line. Anything nearer is an
    obstacle in front of the wall and breaks a run. A run counts only if it is
    bounded by wall beams on BOTH sides -- a gap running off the edge of the
    field of view is not a measurable window.

    Returns {'x': arena x of the centre, 'width': m, 'beams': n} for the run
    nearest the aircraft's own x, or None.
    """
    fx, fy, psi = fix
    beams = []      # (arena_x_on_wall, kind) in angle order
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_inc
        d = psi + a
        t, hx = _wall_hit(fx, fy, d, wall_y)
        if t is None:
            continue
        # Only beams within `fov` of straight at the wall.
        towards = math.atan2(wall_y - fy, 0.0)
        off = math.atan2(math.sin(d - towards), math.cos(d - towards))
        if abs(off) > fov:
            continue
        r = float(r)
        if not math.isfinite(r) or r >= max_range:
            kind = 'through'
        elif r > t + margin:
            kind = 'through'
        elif r >= t - margin:
            kind = 'wall'
        else:
            kind = 'near'
        beams.append((hx, kind))
    if not beams:
        return None
    beams.sort(key=lambda b: b[0])

    best = None
    i = 0
    n = len(beams)
    while i < n:
        if beams[i][1] != 'through':
            i += 1
            continue
        j = i
        while j + 1 < n and beams[j + 1][1] == 'through':
            j += 1
        if i > 0 and j < n - 1 and beams[i - 1][1] == 'wall' \
                and beams[j + 1][1] == 'wall':
            # Edges halfway between the last wall beam and the first through.
            left = 0.5 * (beams[i - 1][0] + beams[i][0])
            right = 0.5 * (beams[j][0] + beams[j + 1][0])
            width = right - left
            if min_width <= width <= max_width:
                cand = {'x': 0.5 * (left + right), 'width': width,
                        'beams': j - i + 1}
                if best is None or abs(cand['x'] - fx) < abs(best['x'] - fx):
                    best = cand
        i = j + 1
    return best


# ----------------------------------------------------- camera helpers

def _column_geometry(width_px, fx_px, cx_px, cam, wall_y):
    """Per image column: arena x where its ray meets the wall, and the
    expected optical-axis depth to the wall. cam = (x, y, yaw) in the arena.
    Image u grows to the RIGHT, i.e. towards negative body angle."""
    cx_, cy_, psi = cam
    u = np.arange(width_px, dtype=np.float64)
    alpha = -np.arctan((u - cx_px) / fx_px)          # + = left
    d = psi + alpha
    s = np.sin(d)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (wall_y - cy_) / s
    ok = (np.abs(s) > 1e-6) & (t > 0.0)
    t = np.where(ok, t, np.nan)
    hx = cx_ + t * np.cos(d)
    z = t * np.cos(alpha)
    return hx, z


def _best_blob(mask, hx, z_col, fy_px, cy_px, min_width, max_width,
               min_fill, min_px):
    """Largest compact, untruncated blob -> window candidate (arena x etc.)."""
    import cv2
    m = mask.astype(np.uint8)
    n, _, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    h_img, w_img = mask.shape
    best = None
    for k in range(1, n):
        x, y, w, h, area = stats[k]
        if area < min_px:
            continue
        # Touching the frame edge = truncated, not measurable.
        if x <= 0 or y <= 0 or x + w >= w_img or y + h >= h_img:
            continue
        fill = area / float(w * h)
        if fill < min_fill:
            continue
        xl, xr = hx[x], hx[x + w - 1]
        if not (math.isfinite(xl) and math.isfinite(xr)):
            continue
        width = abs(xr - xl)
        zc = z_col[x + w // 2]
        if not math.isfinite(zc):
            continue
        height = h / fy_px * zc
        if not (min_width <= width <= max_width
                and min_width <= height <= max_width):
            continue
        v_mid = y + 0.5 * h
        cand = {'x': 0.5 * (xl + xr), 'width': width, 'height': height,
                # + = window centre ABOVE the camera's optical axis
                'up': -(v_mid - cy_px) / fy_px * zc,
                'fill': fill, 'area': int(area)}
        if best is None or area > best['area']:
            best = cand
    return best


# ------------------------------------------------------------- depth hole

def depth_hole(depth, K, cam, wall_y, *, stride=4, margin=0.50,
               min_width=0.4, max_width=2.0, min_fill=0.6, min_px=20):
    """The window as a compact region well BEYOND the wall (or no return).

    depth: HxW float metres, NaN/inf/0 = no reading. K: (fx, fy, cx, cy) of
    the image depth is aligned to. cam: camera (x, y, yaw) in the arena.
    """
    d = np.asarray(depth, dtype=np.float32)[::stride, ::stride]
    fx, fy, cx, cy = (K[0] / stride, K[1] / stride, K[2] / stride, K[3] / stride)
    hx, z_col = _column_geometry(d.shape[1], fx, cx, cam, wall_y)
    with np.errstate(invalid='ignore'):
        far = (~np.isfinite(d)) | (d <= 0.0) | (d > (z_col[None, :] + margin))
    far &= np.isfinite(z_col)[None, :]
    return _best_blob(far, hx, z_col, fy, cy, min_width, max_width,
                      min_fill, min_px)


# ------------------------------------------------------------- brightness

def bright_region(gray, K, cam, wall_y, *, stride=4, ratio=1.8, min_step=25.0,
                  min_width=0.4, max_width=2.0, min_fill=0.6, min_px=20):
    """The window as a compact region much BRIGHTER than the dark room.

    gray: HxW uint8. Threshold = max(median * ratio, median + min_step): the
    lit arena through the opening against the unlit walls. Returns the blob
    plus 'contrast' = mean inside / median of the frame.
    """
    g = np.asarray(gray, dtype=np.float32)[::stride, ::stride]
    fx, fy, cx, cy = (K[0] / stride, K[1] / stride, K[2] / stride, K[3] / stride)
    med = float(np.median(g))
    thr = max(med * ratio, med + min_step)
    mask = g > thr
    hx, z_col = _column_geometry(g.shape[1], fx, cx, cam, wall_y)
    best = _best_blob(mask, hx, z_col, fy, cy, min_width, max_width,
                      min_fill, min_px)
    if best is not None:
        best['contrast'] = float(np.mean(g[mask])) / max(med, 1.0)
    return best


# ------------------------------------------------------------------ vote

def vote(cues, tolerance, need=2, prefer='lidar'):
    """Do at least `need` cues agree on the window's arena x?

    cues: {name: result-or-None}. Returns (x, agreeing_names) or (None, []).
    The fused x is the preferred cue's own value when it is in the agreeing
    set (the lidar is the finest laterally: ~1 deg beams on a known wall),
    otherwise the mean of the set.
    """
    got = [(k, v['x']) for k, v in cues.items() if v is not None]
    best = []
    for _, x0 in got:
        group = [(k, x) for k, x in got if abs(x - x0) <= tolerance]
        if len(group) > len(best):
            best = group
    if len(best) < need:
        return None, []
    names = [k for k, _ in best]
    xs = dict(best)
    if prefer in xs:
        return xs[prefer], names
    return float(sum(xs.values()) / len(xs)), names


# ------------------------------------------------ the wall ahead, body frame

def fit_wall_ahead(ranges, angle_min, angle_inc, *, fov=math.radians(60.0),
                   max_range=3.5, inlier=0.08, iters=4):
    """Robust straight-line fit to the returns in front of the aircraft.

    Needs no map, so it works OUTSIDE the room where there is no arena fix.
    Returns (d, phi): perpendicular distance to the wall line (m) and the
    body-frame FLU angle of that perpendicular (0 = straight ahead, + = left),
    or None. Beams through the window (no return, or far beyond) simply do
    not take part; iterative trimming drops anything off the line.
    """
    pts = []
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_inc
        a = math.atan2(math.sin(a), math.cos(a))
        r = float(r)
        if abs(a) > fov or not math.isfinite(r) or r <= 0.05 or r >= max_range:
            continue
        pts.append((r * math.cos(a), r * math.sin(a)))
    if len(pts) < 8:
        return None
    p = np.asarray(pts)
    keep = np.ones(len(p), bool)
    n = d = None
    for _ in range(iters):
        q = p[keep]
        if len(q) < 8:
            return None
        c = q.mean(axis=0)
        _, _, vt = np.linalg.svd(q - c)
        n = vt[1]                           # unit normal of the best line
        d = float(np.dot(c, n))
        if d < 0.0:
            n, d = -n, -d                   # normal points AWAY from us
        res = np.abs(p @ n - d)
        new = res < max(inlier, 2.5 * float(np.median(res[keep])))
        if new.sum() < 8 or np.array_equal(new, keep):
            keep = new if new.sum() >= 8 else keep
            break
        keep = new
    return d, math.atan2(n[1], n[0])


def wall_gap_body(ranges, angle_min, angle_inc, *, fov=math.radians(60.0),
                  max_range=3.5, margin=0.30, min_width=0.4, max_width=2.0):
    """The window in the wall ahead, in the BODY frame, from the scan alone.

    Returns {'d', 'phi', 'centre': (x, y) FLU of the gap centre on the wall
    line, 'lateral': its offset along the wall (+ = left), 'width'} or None.
    """
    wall = fit_wall_ahead(ranges, angle_min, angle_inc, fov=fov,
                          max_range=max_range)
    if wall is None:
        return None
    d, phi = wall
    # A frame where that wall is the line y = -d and we stand at the origin:
    # yaw' + phi = -pi/2. Its +x runs along the wall, to our left.
    gap = lidar_gap(ranges, angle_min, angle_inc, (0.0, 0.0, -math.pi / 2 - phi),
                    -d, fov=fov, margin=margin, min_width=min_width,
                    max_width=max_width, max_range=max_range)
    if gap is None:
        return None
    s = gap['x']
    foot = (d * math.cos(phi), d * math.sin(phi))
    along = (-math.sin(phi), math.cos(phi))
    return {'d': d, 'phi': phi, 'lateral': s, 'width': gap['width'],
            'centre': (foot[0] + s * along[0], foot[1] + s * along[1])}
