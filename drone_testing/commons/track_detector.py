#!/usr/bin/env python3
"""Track detectors for the track-following nodes.

Two algorithms, same result dict (see `make_track_detector`):
  * `TrackDetector` (detector_mode 'red_flanks', documented below) — track
    sandwiched between two red panels; used by track_centering_node and
    obstacle_traversal_node.
  * `GreyBackgroundTrackDetector` ('grey_background') — track against a
    greyish floor with distinct edges; the default for track_traversal. See
    its class docstring.

Colour-thresholded red-flank detector (TrackDetector):

An earlier version of this detector deliberately never looked at colour —
see the package README's note on the texture-based track-crossing stage
that used to live in `obstacle_course_landing_node` and was removed for
being fragile to lighting/floor changes. That reasoning held as long as the
track's appearance was unknown. It no longer applies: the arena's
off-track floor is a distinct, consistent reddish colour, and the
line-detection approach this module used before (Canny + probabilistic
Hough transform, filtered by orientation/length) was producing frequent
false detections in practice. Colour thresholding on a known, fixed colour
is simple and reliable where geometric line-fitting on an unknown texture
was not.

The track sits sandwiched between two red regions (left and right) — it is
*not* the case that everything non-red is track: the floor beyond the red
panels (visible below/around them in a down-facing frame) is also non-red,
so a naive "biggest non-red blob" merges the track with that floor and
reports a bogus width. Instead, per frame:

 1. THRESHOLD  HSV-threshold the reddish colour (`_red_mask`) — two hue
               ranges since red wraps around hue 0/179 in OpenCV's HSV —
               then clean it up with a morphological open (drop red-mask
               speckle noise) and close (fill small holes from
               track-texture pixels that happened to look reddish).

 2. ROWS       For each sampled row, find every contiguous run of red
               pixels, keep the two widest (the left and right red
               regions — a straightforward left/right split, not a
               largest-gap heuristic, since here there are exactly two
               *known* flanking regions, not an unknown number of edge
               candidates), and take the *inner* edge of each as that
               row's left/right track boundary. A row without two clearly
               separated red regions (e.g. the floor beyond the track,
               which isn't flanked by red at all) is simply skipped, which
               is what keeps this from ever merging the track with
               anything past its actual red-bounded extent.

               This alone isn't quite enough: at some rows the floor is
               still visible *between* two red-looking regions (e.g. a
               shallow camera angle letting the floor peek under the near
               edge of the track panel), which would otherwise pass the
               two-red-region check and pull the fit toward the floor.
               Each such row is additionally checked for greyness
               (`grey_sat_max`) — the row is dropped if its *median*
               saturation between the two edges is at or below that, since
               a floor-coloured gap reads as consistently low-saturation
               while the track's own pattern (even its lighter background)
               does not.

 3. FIT        Each side's surviving per-row boundary points are combined
               into one robust line via `cv2.fitLine` (least squares over
               every sampled row, not just two points).

 4. REPORT     The track's centreline is the midpoint between the two
               fitted side-lines at every row; `offset_norm` is that
               centreline's x-position at the frame's own vertical centre,
               normalised to +/-1 the same way every other detector in
               this workspace reports offset. `angle_deg` is the
               centreline's tilt from vertical (0 = perfectly vertical
               in-image) — see its own docstring below for the exact sign
               convention. Gates: of the rows that see any red at all,
               enough of them must yield a clean two-red-region split
               (`min_row_coverage_frac`) or the reading is too fragmented
               to trust; the measured width must
               fall inside (`min_track_width_frac`, `max_track_width_frac`)
               of frame width; the fitted tilt must be inside
               `max_angle_from_vertical_deg`.
"""

import math

import cv2
import numpy as np


def _red_mask(hsv, h_low1, h_high1, h_low2, h_high2, sat_min, val_min):
    """Binary mask (255 = reddish) of the off-track floor, from an already
    HSV-converted frame.

    Red wraps around hue 0 in OpenCV's HSV (H in [0, 179]), so it needs two
    ranges — one just above 0, one just below 179 — combined with OR.
    `sat_min`/`val_min` keep desaturated (grey/white track texture) and
    dark (shadow) pixels from being misread as red.
    """
    lower1 = np.array([h_low1, sat_min, val_min], dtype=np.uint8)
    upper1 = np.array([h_high1, 255, 255], dtype=np.uint8)
    lower2 = np.array([h_low2, sat_min, val_min], dtype=np.uint8)
    upper2 = np.array([h_high2, 255, 255], dtype=np.uint8)
    return cv2.bitwise_or(
        cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))


def _row_boundary(red_row):
    """Given one row of the red mask, return (x_left_edge, x_right_edge) —
    the inner edges of the two widest red runs, i.e. this row's track
    boundary — or None if the row doesn't show a clean two-sided split
    (fewer than two red runs, or the two widest runs aren't in left/right
    order with a positive gap between them)."""
    xs = np.nonzero(red_row)[0]
    if xs.size == 0:
        return None
    breaks = np.where(np.diff(xs) > 1)[0] + 1
    runs = np.split(xs, breaks)
    if len(runs) < 2:
        return None
    widest = sorted(runs, key=len, reverse=True)[:2]
    widest.sort(key=lambda r: r[0])
    left_run, right_run = widest
    x_left_edge = float(left_run[-1])
    x_right_edge = float(right_run[0])
    if x_right_edge <= x_left_edge:
        return None
    return x_left_edge, x_right_edge


def _looks_like_floor(sat_row, x_left_edge, x_right_edge, grey_sat_max):
    """True if the span between two candidate track boundaries is
    typically low-saturation — i.e. floor colour peeking through a
    shallow camera angle, not the track's own (more colourful) pattern.

    Uses the row's *median* saturation, not a per-pixel fraction: a
    track's own light/cream background pixels can individually dip to a
    fairly low saturation too (measured ~15-30 on the real arena photo),
    so counting pixels below a hard cutoff is noisy. The floor's median
    saturation measured close to 0 on that same photo — a clean gap from
    any real track row's median (~16-30) — so comparing medians is far
    more robust than comparing individual pixels.
    """
    x0 = max(0, int(round(x_left_edge)))
    x1 = min(sat_row.shape[0], int(round(x_right_edge)) + 1)
    span = sat_row[x0:x1]
    if span.size == 0:
        return True
    return float(np.median(span)) <= grey_sat_max


def _fit_side(points, dist=cv2.DIST_L2):
    """Least-squares line through every (x, y) sample on one side
    (`dist=cv2.DIST_HUBER` for an outlier-resistant fit).

    Returns a function y -> x on the fitted line, or None if the fit is
    degenerate (near-horizontal boundary — shouldn't happen for a real
    track side, but division-by-zero guards are cheap insurance).
    """
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    vx, vy, x0, y0 = cv2.fitLine(pts, dist, 0, 0.01, 0.01).flatten()
    if abs(vy) < 1e-6:
        return None
    slope = vx / vy   # dx/dy
    return lambda y: float(x0 + slope * (y - y0))


class TrackDetector:
    """Stateful, confirm-gated track detector.

    Same calling convention as pad_landing's PadDetector: `det = self(frame)`
    returns None (with `.reason` explaining why) or a dict with `ok` gated
    on `confirm` consecutive good frames.
    """

    def __init__(self, work_width=384,
                 red_hue_low1=0, red_hue_high1=10,
                 red_hue_low2=170, red_hue_high2=179,
                 red_sat_min=60, red_val_min=40,
                 grey_sat_max=12,
                 morph_kernel_px=7, row_stride=4,
                 min_row_coverage_frac=0.6,
                 min_track_width_frac=0.15, max_track_width_frac=0.95,
                 max_angle_from_vertical_deg=40.0,
                 confirm=3):
        self.work_width = work_width
        self.red_hue_low1 = red_hue_low1
        self.red_hue_high1 = red_hue_high1
        self.red_hue_low2 = red_hue_low2
        self.red_hue_high2 = red_hue_high2
        self.red_sat_min = red_sat_min
        self.red_val_min = red_val_min
        self.grey_sat_max = grey_sat_max
        self.morph_kernel_px = max(1, int(morph_kernel_px))
        self.row_stride = max(1, int(row_stride))
        self.min_row_coverage_frac = min_row_coverage_frac
        self.min_track_width_frac = min_track_width_frac
        self.max_track_width_frac = max_track_width_frac
        self.max_angle_from_vertical_deg = max_angle_from_vertical_deg
        self.confirm = confirm

        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.morph_kernel_px, self.morph_kernel_px))

        self.reason = 'no frame yet'
        self.hits = 0
        self.lost = 0

    def __call__(self, frame):
        H0, W0 = frame.shape[:2]
        s = min(1.0, float(self.work_width) / W0)
        work = frame
        if s < 1.0:
            work = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        h, w = work.shape[:2]

        hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
        sat = hsv[:, :, 1]
        red = _red_mask(
            hsv, self.red_hue_low1, self.red_hue_high1,
            self.red_hue_low2, self.red_hue_high2,
            self.red_sat_min, self.red_val_min)
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, self._kernel)
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, self._kernel)

        # Coverage is measured against rows that see *any* red, not every
        # sampled row in the frame — the red regions are a finite panel,
        # not something guaranteed to span the whole vertical FOV, so rows
        # entirely above/below it (floor, background) shouldn't count
        # against how clean the split is where red actually is present.
        rows_with_red = 0
        ys, x_lefts, x_rights = [], [], []
        for y in range(0, h, self.row_stride):
            row = red[y]
            if not np.any(row):
                continue
            rows_with_red += 1
            edges = _row_boundary(row)
            if edges is None:
                continue
            if _looks_like_floor(sat[y], edges[0], edges[1], self.grey_sat_max):
                continue
            ys.append(float(y))
            x_lefts.append(edges[0])
            x_rights.append(edges[1])
        if rows_with_red == 0:
            return self._reject('no red found anywhere in frame', red)
        if (len(ys) < 4
                or len(ys) / rows_with_red < self.min_row_coverage_frac):
            return self._reject(
                f'insufficient row coverage ({len(ys)}/{rows_with_red} '
                'rows with red)', red)

        left_x = _fit_side(list(zip(x_lefts, ys)))
        right_x = _fit_side(list(zip(x_rights, ys)))
        if left_x is None or right_x is None:
            return self._reject('degenerate side fit', red)

        y_top, y_bot = ys[0], ys[-1]
        lx_top, lx_bot = left_x(y_top), left_x(y_bot)
        rx_top, rx_bot = right_x(y_top), right_x(y_bot)
        if rx_top < lx_top or rx_bot < lx_bot:
            return self._reject('sides crossed (bad fit)', red)

        width_top, width_bot = rx_top - lx_top, rx_bot - lx_bot
        min_w, max_w = self.min_track_width_frac * w, self.max_track_width_frac * w
        if not (min_w <= width_top <= max_w and min_w <= width_bot <= max_w):
            return self._reject(
                f'bad track width ({width_top / w * 100:.0f}%/'
                f'{width_bot / w * 100:.0f}% of frame)', red)

        cx_top, cx_bot = (lx_top + rx_top) / 2.0, (lx_bot + rx_bot) / 2.0
        angle_deg = math.degrees(math.atan2(cx_bot - cx_top, y_bot - y_top))
        if abs(angle_deg) > self.max_angle_from_vertical_deg:
            return self._reject(f'excessive tilt ({angle_deg:+.0f} deg)', red)

        # centreline is straight by construction (average of two fitted
        # lines) -> a plain linear interpolation to any row is exact.
        cx_mid = cx_top + (cx_bot - cx_top) * ((h / 2.0 - y_top) / (y_bot - y_top))
        offset_norm = (cx_mid - w / 2.0) / (w / 2.0)

        self.lost = 0
        self.hits += 1
        ok = self.hits >= self.confirm
        self.reason = 'ok' if ok else 'confirming'

        inv = 1.0 / s
        return {
            'ok': ok,
            'reason': self.reason,
            'offset_norm': offset_norm,
            'angle_deg': angle_deg,
            'track_width_norm': ((width_top + width_bot) / 2.0) / w,
            'center_line': ((cx_top * inv, y_top * inv), (cx_bot * inv, y_bot * inv)),
            'left_line': ((lx_top * inv, y_top * inv), (lx_bot * inv, y_bot * inv)),
            'right_line': ((rx_top * inv, y_top * inv), (rx_bot * inv, y_bot * inv)),
            'row_coverage': len(ys) / rows_with_red,
            'work': (w, h),
            '_red_mask': red,
        }

    def _reject(self, why, red=None):
        self.lost += 1
        self.hits = 0
        self.reason = why
        # Kept even on rejection so the debug view still shows *why* — an
        # all-red frame (fully off-track) or a shredded mask (bad thresholds)
        # look very different from each other and from "no frame yet".
        self._last_red = red
        return None


# ------------------------ grey-background detector ------------------------- #

class GreyBackgroundTrackDetector:
    """Track on a greyish floor, bounded by distinct edges.

    Same calling convention and result dict as `TrackDetector` (so every
    consumer — control law, `draw()` — works unchanged), different
    segmentation:

    Tuned on a real arena photo: dark-grey speckled terrazzo floor (blurred
    L* ~112, chroma ~1-5) and a white paper track with an orange/blue/purple
    leaf print (blurred L* ~186; chroma mostly ~5 — the paper itself is
    nearly colourless, only the print reaches ~17-25). Brightness, not
    colour, is what separates them.

     1. SPLIT       Box-blur the L* (CIELAB lightness) channel (`blur_px`,
                    so the print and the floor's speckles average out),
                    then Otsu-threshold it: floor and track come out as the
                    two brightness classes, with the split adapting to the
                    lighting every frame. If the two class means differ by
                    less than `min_contrast` there is no track in view (e.g.
                    only speckled floor) and the frame is rejected.
     2. TRACK MASK  Track = the `track_polarity` class ('brighter' for the
                    white track on dark floor, 'darker' for the reverse),
                    optionally OR clearly coloured (blurred chroma >
                    `bg_chroma_max`; 0 = off, the default — it made no
                    difference on the real photo, and it must stay off
                    over a coloured floor such as the SITL arena's red one).
                    Morphological open/close, keep the largest connected
                    region, and fill its holes (dark leaves in the print).
     3. EDGES       Per sampled row: the widest track run gives a rough
                    left/right boundary; each is snapped to the strongest
                    horizontal gradient (|dL/dx| + |dChroma/dx|, on an image
                    box-blurred by `edge_blur_px` so single floor speckles
                    don't read as edges) within `edge_search_px`. A side is
                    dropped for that row if no gradient reaches
                    `edge_min_grad` (not a distinct edge) or the run touches
                    the frame border there (the frame edge is not a track
                    edge).
     4. FIT/REPORT  Huber (outlier-resistant) line fit per side; the
                    centreline, offset_norm, angle_deg and the tilt gate are
                    exactly as in `TrackDetector`. The track-width gate is
                    off unless `check_width` (a distinct-edged straight
                    track is trusted at any apparent width).
    """

    def __init__(self, work_width=384, blur_px=15,
                 track_polarity='brighter', min_contrast=45.0, bg_chroma_max=0.0,
                 morph_kernel_px=7, row_stride=4,
                 edge_blur_px=9, edge_search_px=8, edge_min_grad=40.0, min_edge_points=6,
                 check_width=False, min_track_width_frac=0.10, max_track_width_frac=0.95,
                 max_angle_from_vertical_deg=40.0, confirm=3):
        self.work_width = work_width
        self.blur_px = max(1, int(blur_px)) | 1
        self.track_polarity = str(track_polarity).strip().lower()
        if self.track_polarity not in ('brighter', 'darker'):
            raise ValueError("track_polarity must be 'brighter' or 'darker'")
        self.min_contrast = float(min_contrast)
        self.bg_chroma_max = float(bg_chroma_max)
        self.edge_blur_px = max(1, int(edge_blur_px))
        self.morph_kernel_px = max(1, int(morph_kernel_px))
        self.row_stride = max(1, int(row_stride))
        self.edge_search_px = max(1, int(edge_search_px))
        self.edge_min_grad = float(edge_min_grad)
        self.min_edge_points = max(2, int(min_edge_points))
        self.check_width = bool(check_width)
        self.min_track_width_frac = min_track_width_frac
        self.max_track_width_frac = max_track_width_frac
        self.max_angle_from_vertical_deg = max_angle_from_vertical_deg
        self.confirm = confirm

        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.morph_kernel_px, self.morph_kernel_px))
        self.reason = 'no frame yet'
        self.hits = 0
        self.lost = 0
        self.floor_lum = None
        self.track_lum = None

    def _snap(self, grad_row, x, w):
        """Strongest edge within edge_search_px of x, or None if too weak."""
        x0 = max(0, int(x) - self.edge_search_px)
        x1 = min(w, int(x) + self.edge_search_px + 1)
        if x1 <= x0:
            return None
        j = int(np.argmax(grad_row[x0:x1]))
        if grad_row[x0 + j] < self.edge_min_grad:
            return None
        return float(x0 + j)

    def __call__(self, frame):
        H0, W0 = frame.shape[:2]
        s = min(1.0, float(self.work_width) / W0)
        work = frame
        if s < 1.0:
            work = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        h, w = work.shape[:2]

        lab = cv2.cvtColor(work, cv2.COLOR_BGR2LAB).astype(np.float32)

        # 1: adaptive floor/track brightness split
        lab_b = cv2.blur(lab, (self.blur_px, self.blur_px))
        lum_b = np.clip(lab_b[:, :, 0], 0, 255).astype(np.uint8)
        chroma_b = np.hypot(lab_b[:, :, 1] - 128.0, lab_b[:, :, 2] - 128.0)
        thresh, bright = cv2.threshold(lum_b, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        if not np.any(bright) or np.all(bright):
            return self._reject('uniform brightness — no track/floor split', None)
        lo = float(lum_b[bright == 0].mean())
        hi = float(lum_b[bright > 0].mean())
        if self.track_polarity == 'brighter':
            self.floor_lum, self.track_lum, track = lo, hi, bright
        else:
            self.floor_lum, self.track_lum, track = hi, lo, 255 - bright
        if hi - lo < self.min_contrast:
            return self._reject(
                f'low contrast ({hi - lo:.0f} < {self.min_contrast:.0f}) — no track in view',
                255 - track)

        # 2: track mask = brightness class OR clearly coloured; largest
        # region, holes (dark print) filled
        if self.bg_chroma_max > 0:
            track = cv2.bitwise_or(
                track, np.where(chroma_b > self.bg_chroma_max, 255, 0).astype(np.uint8))
        track = cv2.morphologyEx(track, cv2.MORPH_OPEN, self._kernel)
        track = cv2.morphologyEx(track, cv2.MORPH_CLOSE, self._kernel)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(track, 8)
        if n <= 1:
            return self._reject('nothing distinct from the floor', 255 - track)
        best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        track = np.where(labels == best, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(track, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(track, contours, -1, 255, cv2.FILLED)
        offtrack = 255 - track

        # 3: rough boundaries from the mask, snapped to the distinct edges
        lab_e = cv2.blur(lab, (self.edge_blur_px, self.edge_blur_px))
        chroma_e = np.hypot(lab_e[:, :, 1] - 128.0, lab_e[:, :, 2] - 128.0)
        grad = (np.abs(cv2.Sobel(lab_e[:, :, 0], cv2.CV_32F, 1, 0, ksize=3))
                + np.abs(cv2.Sobel(chroma_e, cv2.CV_32F, 1, 0, ksize=3)))

        left_pts, right_pts = [], []
        for y in range(0, h, self.row_stride):
            xs = np.nonzero(track[y])[0]
            if xs.size == 0:
                continue
            runs = np.split(xs, np.where(np.diff(xs) > 1)[0] + 1)
            run = max(runs, key=len)
            xl, xr = int(run[0]), int(run[-1])
            if xl > 0:
                e = self._snap(grad[y], xl, w)
                if e is not None:
                    left_pts.append((e, float(y)))
            if xr < w - 1:
                e = self._snap(grad[y], xr, w)
                if e is not None:
                    right_pts.append((e, float(y)))

        if len(left_pts) < self.min_edge_points or len(right_pts) < self.min_edge_points:
            return self._reject(
                f'too few distinct edge points (L {len(left_pts)} / R {len(right_pts)})',
                offtrack)

        # 4: fit + report (same as TrackDetector)
        left_x = _fit_side(left_pts, cv2.DIST_HUBER)
        right_x = _fit_side(right_pts, cv2.DIST_HUBER)
        if left_x is None or right_x is None:
            return self._reject('degenerate side fit', offtrack)

        ys = [p[1] for p in left_pts + right_pts]
        y_top, y_bot = min(ys), max(ys)
        if y_bot - y_top < 2 * self.row_stride:
            return self._reject('track spans too few rows', offtrack)
        lx_top, lx_bot = left_x(y_top), left_x(y_bot)
        rx_top, rx_bot = right_x(y_top), right_x(y_bot)
        if rx_top < lx_top or rx_bot < lx_bot:
            return self._reject('sides crossed (bad fit)', offtrack)

        width_top, width_bot = rx_top - lx_top, rx_bot - lx_bot
        min_w, max_w = self.min_track_width_frac * w, self.max_track_width_frac * w
        if self.check_width and not (min_w <= width_top <= max_w
                                     and min_w <= width_bot <= max_w):
            return self._reject(
                f'bad track width ({width_top / w * 100:.0f}%/'
                f'{width_bot / w * 100:.0f}% of frame)', offtrack)

        cx_top, cx_bot = (lx_top + rx_top) / 2.0, (lx_bot + rx_bot) / 2.0
        angle_deg = math.degrees(math.atan2(cx_bot - cx_top, y_bot - y_top))
        if abs(angle_deg) > self.max_angle_from_vertical_deg:
            return self._reject(f'excessive tilt ({angle_deg:+.0f} deg)', offtrack)

        cx_mid = cx_top + (cx_bot - cx_top) * ((h / 2.0 - y_top) / (y_bot - y_top))
        offset_norm = (cx_mid - w / 2.0) / (w / 2.0)

        self.lost = 0
        self.hits += 1
        ok = self.hits >= self.confirm
        self.reason = 'ok' if ok else 'confirming'

        inv = 1.0 / s
        rows = len(range(0, h, self.row_stride))
        return {
            'ok': ok,
            'reason': self.reason,
            'offset_norm': offset_norm,
            'angle_deg': angle_deg,
            'track_width_norm': ((width_top + width_bot) / 2.0) / w,
            'center_line': ((cx_top * inv, y_top * inv), (cx_bot * inv, y_bot * inv)),
            'left_line': ((lx_top * inv, y_top * inv), (lx_bot * inv, y_bot * inv)),
            'right_line': ((rx_top * inv, y_top * inv), (rx_bot * inv, y_bot * inv)),
            'row_coverage': min(len(left_pts), len(right_pts)) / max(1, rows),
            'edge_points': ([(x * inv, y * inv) for x, y in left_pts],
                            [(x * inv, y * inv) for x, y in right_pts]),
            'work': (w, h),
            '_red_mask': offtrack,   # 255 = not track (draw() tints it red)
        }

    def _reject(self, why, offtrack=None):
        self.lost += 1
        self.hits = 0
        self.reason = why
        self._last_red = offtrack   # draw() shows it on rejected frames too
        return None


# --------------------------- shared construction --------------------------- #

# Every detector parameter, under the ROS parameter names the track nodes
# and the standalone replay script use — declared from this one table so
# the two can't drift apart. `detector_mode` picks the algorithm.
TRACK_DETECTOR_PARAMS = {
    'detector_mode': 'grey_background',    # 'grey_background' | 'red_flanks'
    'detector_work_width': 384,
    'detector_morph_kernel_px': 7,
    'detector_row_stride': 4,
    'detector_min_track_width_frac': 0.10,
    'detector_max_track_width_frac': 0.95,
    'detector_max_angle_from_vertical_deg': 40.0,
    'detector_confirm_frames': 3,
    # grey_background
    'detector_blur_px': 15,
    'detector_track_polarity': 'brighter',   # white track on dark floor
    'detector_min_contrast': 45.0,
    'detector_bg_chroma_max': 0.0,           # 0 = colour cue off
    'detector_edge_blur_px': 9,
    'detector_edge_search_px': 8,
    'detector_edge_min_grad': 40.0,
    'detector_min_edge_points': 6,
    'detector_check_width': False,           # grey_background: width gate off
    # red_flanks
    'detector_red_hue_low1': 0,
    'detector_red_hue_high1': 10,
    'detector_red_hue_low2': 170,
    'detector_red_hue_high2': 179,
    'detector_red_sat_min': 60,
    'detector_red_val_min': 40,
    'detector_grey_sat_max': 12,
    'detector_min_row_coverage_frac': 0.0,
}


def make_track_detector(params):
    """Build the detector selected by params['detector_mode'] from a flat
    dict keyed by the TRACK_DETECTOR_PARAMS names (missing keys use its
    defaults)."""
    p = dict(TRACK_DETECTOR_PARAMS)
    p.update({k: v for k, v in params.items() if k in TRACK_DETECTOR_PARAMS})
    common = dict(
        work_width=int(p['detector_work_width']),
        morph_kernel_px=int(p['detector_morph_kernel_px']),
        row_stride=int(p['detector_row_stride']),
        min_track_width_frac=float(p['detector_min_track_width_frac']),
        max_track_width_frac=float(p['detector_max_track_width_frac']),
        max_angle_from_vertical_deg=float(p['detector_max_angle_from_vertical_deg']),
        confirm=int(p['detector_confirm_frames']))
    mode = str(p['detector_mode']).strip().lower()
    if mode == 'red_flanks':
        return TrackDetector(
            red_hue_low1=int(p['detector_red_hue_low1']),
            red_hue_high1=int(p['detector_red_hue_high1']),
            red_hue_low2=int(p['detector_red_hue_low2']),
            red_hue_high2=int(p['detector_red_hue_high2']),
            red_sat_min=int(p['detector_red_sat_min']),
            red_val_min=int(p['detector_red_val_min']),
            grey_sat_max=int(p['detector_grey_sat_max']),
            min_row_coverage_frac=float(p['detector_min_row_coverage_frac']),
            **common)
    if mode != 'grey_background':
        raise ValueError(f"detector_mode must be 'grey_background' or 'red_flanks', not {mode!r}")
    return GreyBackgroundTrackDetector(
        blur_px=int(p['detector_blur_px']),
        track_polarity=str(p['detector_track_polarity']),
        min_contrast=float(p['detector_min_contrast']),
        bg_chroma_max=float(p['detector_bg_chroma_max']),
        edge_blur_px=int(p['detector_edge_blur_px']),
        edge_search_px=int(p['detector_edge_search_px']),
        edge_min_grad=float(p['detector_edge_min_grad']),
        min_edge_points=int(p['detector_min_edge_points']),
        check_width=bool(p['detector_check_width']),
        **common)


# -------------------------------- drawing ---------------------------------- #

def draw(frame, det, detector=None, show_mask=True, extra_lines=()):
    """Debug overlay: frame-centre crosshair, the red/track segmentation
    mask, the two fitted boundary lines, the derived centreline, and the
    offset/angle readout. Pass `detector` (the `TrackDetector` instance) to
    still see the mask on a rejected frame (`det is None`) — useful for
    telling "no red found" apart from "red found everywhere" apart from
    "mask too fragmented" while tuning the HSV thresholds."""
    out = frame.copy()
    H, W = out.shape[:2]
    u = max(W, H) / 1200.0
    lw = max(1, int(2.5 * u))

    red_src = det.get('_red_mask') if det is not None else (
        getattr(detector, '_last_red', None) if detector is not None else None)
    if show_mask and red_src is not None:
        m = cv2.resize(red_src, (W, H), interpolation=cv2.INTER_NEAREST)
        tint = np.zeros_like(out)
        tint[m == 0] = (0, 200, 0)     # green = candidate track
        tint[m > 0] = (0, 0, 200)      # red = off-track (red floor / grey floor)
        out = cv2.addWeighted(out, 0.7, tint, 0.3, 0)

    cv2.drawMarker(out, (W // 2, H // 2), (255, 255, 255),
                   cv2.MARKER_CROSS, int(20 * u), lw)

    def put(txt, i, col):
        o = (int(14 * u), int((26 + 24 * i) * u))
        cv2.putText(out, txt, o, cv2.FONT_HERSHEY_SIMPLEX, 0.52 * u,
                    (0, 0, 0), int(3 * u) + 2, cv2.LINE_AA)
        cv2.putText(out, txt, o, cv2.FONT_HERSHEY_SIMPLEX, 0.52 * u,
                    col, max(1, int(1.4 * u)), cv2.LINE_AA)

    line = 0
    if det is None:
        reason = detector.reason if detector is not None else 'no track'
        put(f'NO TRACK ({reason})', line, (60, 60, 255))
        line += 1
        for txt in extra_lines:
            put(txt, line, (255, 255, 255))
            line += 1
        return out

    def draw_seg(seg, col):
        (x1, y1), (x2, y2) = seg
        cv2.line(out, (int(x1), int(y1)), (int(x2), int(y2)), col, lw, cv2.LINE_AA)

    for side, col in zip(det.get('edge_points', ()), ((255, 160, 0), (255, 0, 160))):
        for x, y in side:   # the distinct-edge samples each side was fitted to
            cv2.circle(out, (int(x), int(y)), max(2, int(3 * u)), col, -1, cv2.LINE_AA)
    draw_seg(det['left_line'], (255, 160, 0))
    draw_seg(det['right_line'], (255, 0, 160))
    col = (0, 230, 90) if det['ok'] else (0, 180, 255)
    draw_seg(det['center_line'], col)

    put('offset %+.3f  angle %+.1f deg  width %.0f%%%s' % (
        det['offset_norm'], det['angle_deg'], det['track_width_norm'] * 100,
        '' if det['ok'] else '  LOW'), line, col)
    line += 1
    for txt in extra_lines:
        put(txt, line, (255, 255, 255))
        line += 1
    return out
