#!/usr/bin/env python3
"""ArUco PnP over a multi-marker board, for the ring grab.

PORTED FROM ring_grab/scripts/aruco_math.py in the slam workspace, and
INVERTED. That file knows where the board is in the world -- it is a Gazebo
model at a surveyed pose -- and solves for the DRONE. Here nobody surveyed
anything: the board is wherever it was hung this morning, so this file solves
for the BOARD relative to the camera and lets the flight node place it in NED
with PX4's own attitude and position. That is the same division of labour
window_detect / window_traverse already use on this airframe, and it is what
lets this module be bench-tested with no flight controller attached.

Also gone with the inversion: the EKF2 path. ring_grab fed /ring/odom through
odom_align and vio_px4_bridge into EV_CTRL so the board became a position
source. Nothing here does that. Position on this vehicle is PMW3901 optical
flow + TFmini Plus through EKF2, exactly as in window_traverse, and the vision
only ever produces a SETPOINT.

FRAMES
------
Detection and solvePnP happen in the OpenCV optical frame:

    x right in the image, y DOWN, z FORWARD along the view axis

but everything this module returns has already been rotated into what the rest
of this package calls the camera frame, which is FRD-aligned:

    x FORWARD, y RIGHT, z DOWN

so the flight node can apply the same r_cam / t_cam mounting matrix
window_traverse applies, with the same sign conventions, and the two nodes
cannot disagree about what cam_pitch means.

The board's own frame is the marker frame OpenCV gives back for a square built
from obj_points() below:

    X right across the board, Y UP the board, Z OUT of the board towards the
    camera

so the three markers differ only by a translation along Y, and the board
normal is the third column of the PnP rotation.

BOARD GEOMETRY
--------------
Three markers in a vertical stack, ids 4 (bottom) / 5 (middle) / 6 (top), with
known pitch. Any non-empty subset gives a pose; more markers give a better
one, and they are solved as ONE RIGID BODY rather than solved separately and
averaged -- see BoardLocator for the measurements that decided that.

Unlike the original this is NOT a set of module constants to be edited and
rebuilt. BoardGeometry is built from ROS parameters in ring_detect.py, because
the sim board was 0.22 m markers and the real one is 0.10 m, and that
difference silently scales every distance downstream.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


# Ratio of best to second-best reprojection error above which the two PnP
# solutions for a planar square are too close to tell apart. Straight from
# aruco_math.py; it is a property of the solver, not of this board.
AMBIGUOUS_ERR_RATIO = 0.55

# OpenCV optical (x right, y down, z fwd) -> camera FRD (x fwd, y right, z down).
# One relabelling of axes, so it is a permutation matrix and its own documentation:
#   frd_x = opt_z,  frd_y = opt_x,  frd_z = opt_y
R_FRD_OPT = np.array([[0.0, 0.0, 1.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0]], dtype=float)


def dict_id(name: str) -> int:
    """cv2.aruco predefined dictionary id from its name, e.g. 'DICT_4X4_50'."""
    key = str(name).strip().upper()
    if not hasattr(cv2.aruco, key):
        raise ValueError(
            f"unknown ArUco dictionary '{name}'. Expected something like "
            "DICT_4X4_50, DICT_5X5_100, DICT_APRILTAG_36h11.")
    return int(getattr(cv2.aruco, key))


def make_detector(dictionary_id: int):
    """A callable(gray) -> (corners, ids), across both OpenCV aruco APIs."""
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    try:
        params = cv2.aruco.DetectorParameters()
    except AttributeError:                      # OpenCV < 4.7
        params = cv2.aruco.DetectorParameters_create()
    try:
        # Subpixel corners are worth having: the whole pose comes from four
        # corner locations, and at 3 m half a pixel is about a centimetre.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    except AttributeError:
        pass
    try:
        det = cv2.aruco.ArucoDetector(dictionary, params)
        return lambda g: det.detectMarkers(g)[:2]
    except AttributeError:                      # OpenCV < 4.7
        return lambda g: cv2.aruco.detectMarkers(
            g, dictionary, parameters=params)[:2]


def obj_points(marker_size: float) -> np.ndarray:
    """The four corners of a marker in ITS OWN frame, in detector order.

    cv2.aruco returns corners as TL, TR, BR, BL, so these must be listed the
    same way round or solvePnP fits a marker that is mirrored and the normal
    comes back pointing into the wall.
    """
    h = marker_size / 2.0
    return np.array([[-h,  h, 0.0],
                     [ h,  h, 0.0],
                     [ h, -h, 0.0],
                     [-h, -h, 0.0]], dtype=np.float64)


def fov_K(width: int, height: int, hfov_rad: float) -> np.ndarray:
    """Intrinsics guessed from a field of view, for when CameraInfo is absent.

    A fallback, not a plan. It gets the BEARING to the board about right and
    the RANGE wrong by whatever the quoted FOV is wrong by, and range is what
    the standoff and the 20 cm grab depth are measured in. Let the D435i's own
    factory calibration arrive on camera_info instead.
    """
    fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
    return np.array([[fx, 0.0, width / 2.0],
                     [0.0, fx, height / 2.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def rot_geodesic_deg(ra: np.ndarray, rb: np.ndarray) -> float:
    """Angle between two rotations, in degrees."""
    c = (float(np.trace(ra.T @ rb)) - 1.0) * 0.5
    return float(np.degrees(math.acos(max(-1.0, min(1.0, c)))))


# ------------------------------------------------------------ board geometry

@dataclass
class BoardGeometry:
    """Where the markers sit on the board, and which one is the target.

    offsets are marker centres in the BOARD frame (X right, Y up, Z out),
    keyed by id. The board origin is wherever those offsets are zero -- the
    middle marker by construction -- but nothing downstream cares, because
    every pose this module returns is expressed at the GRAB marker.
    """

    marker_size: float
    grab_id: int
    offsets: Dict[int, np.ndarray]
    dictionary_id: int

    @classmethod
    def vertical_stack(cls, ids: Sequence[int], marker_size: float,
                       marker_gap: float, grab_id: int,
                       dictionary_id: int) -> 'BoardGeometry':
        """Evenly spaced markers in a vertical line, ids given bottom to top.

        pitch is centre-to-centre, so it is the marker edge plus the gap
        between markers -- the same definition aruco_math.py used, and the
        one you get if you measure between the middles of two markers with a
        tape.
        """
        ids = [int(i) for i in ids]
        if len(ids) != len(set(ids)):
            raise ValueError(f"duplicate marker id in {ids}")
        if not ids:
            raise ValueError("no marker ids given")
        if int(grab_id) not in ids:
            raise ValueError(
                f"grab_marker_id {grab_id} is not one of the board ids {ids}. "
                "The grab target must be a marker the board geometry knows "
                "about, or its position cannot be derived from the others.")
        if marker_size <= 0.0:
            raise ValueError(f"marker_size {marker_size} must be positive")
        pitch = float(marker_size) + float(marker_gap)
        # ids[0] is the bottom marker, so index 0 is the lowest Y. Centre the
        # stack on the middle marker; the choice of origin is arbitrary but
        # keeping it in the middle keeps the numbers small and readable.
        mid = (len(ids) - 1) / 2.0
        offsets = {mid_id: np.array([0.0, (i - mid) * pitch, 0.0], dtype=float)
                   for i, mid_id in enumerate(ids)}
        return cls(marker_size=float(marker_size), grab_id=int(grab_id),
                   offsets=offsets, dictionary_id=int(dictionary_id))

    @property
    def ids(self) -> List[int]:
        return sorted(self.offsets)

    def describe(self) -> str:
        rows = ', '.join(
            f"id{i} y={self.offsets[i][1]:+.3f}" for i in self.ids)
        return (f"{self.marker_size * 100:.0f} cm markers, {rows}, "
                f"grabbing at id{self.grab_id}")


# --------------------------------------------------------------- ambiguity

class AmbiguityResolver:
    """Picks between the two poses IPPE returns for a planar target.

    A square seen head-on is very nearly symmetric under a flip about its own
    plane, so there are two rotations that reproject the four corners almost
    equally well and the solver honestly reports both. Taking the lower
    reprojection error is right when they differ; when they do not, the choice
    flickers frame to frame and the board normal flaps by tens of degrees --
    which downstream is the difference between approaching the board and
    approaching a point behind it.

    So: when the two errors are within AMBIGUOUS_ERR_RATIO of each other, keep
    whichever solution is closer to the one accepted last frame, and say so.

    MEASURED on synthetic boards, 10 cm markers, 600 px focal length: the two
    solutions are within 6 % of each other whenever the board is within about
    10 degrees of square-on, at every range. So "ambiguous" is the NORMAL
    state on a well-aimed approach and must never be treated as a fault --
    what it means is that the tilt is undetermined, and the tilt is the one
    thing this mission does not use. Position stays good: 0.3 cm at 2 m.
    """

    def __init__(self, ratio: float = AMBIGUOUS_ERR_RATIO):
        self.ratio = ratio
        self.prev_R: Optional[np.ndarray] = None
        self.ambiguous = False
        self.errors: Tuple[float, float] = (0.0, float('nan'))
        self.last_ratio = 0.0       # measured best/second error ratio

    def reset(self):
        self.prev_R = None
        self.ambiguous = False

    def solve(self, objp, corners, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE):
        try:
            n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
                objp, corners, K, D, flags=flags)
        except cv2.error:
            return None, None
        if not n:
            return None, None

        cands = sorted(
            ((float(np.ravel(errs[i])[0]) if errs is not None else 0.0,
              rvecs[i], tvecs[i]) for i in range(n)),
            key=lambda c: c[0])
        best = cands[0]
        self.ambiguous = False
        self.errors = (cands[0][0],
                       cands[1][0] if len(cands) > 1 else float('nan'))
        self.last_ratio = (cands[0][0] / max(cands[1][0], 1e-9)
                           if len(cands) > 1 else 0.0)

        if len(cands) > 1:
            self.ambiguous = self.last_ratio > self.ratio
            if self.ambiguous and self.prev_R is not None:
                r0 = cv2.Rodrigues(cands[0][1])[0]
                r1 = cv2.Rodrigues(cands[1][1])[0]
                if (rot_geodesic_deg(r1, self.prev_R)
                        < rot_geodesic_deg(r0, self.prev_R)):
                    best = cands[1]

        self.prev_R = cv2.Rodrigues(best[1])[0]
        return best[1], best[2].reshape(3)


# ---------------------------------------------------------------- sightings

@dataclass
class MarkerSighting:
    """One detected marker. Kept for the annotated view and the id list."""

    marker_id: int
    corners: np.ndarray


@dataclass
class BoardSighting:
    """The board's pose, in the camera FRD frame (x fwd, y right, z down)."""

    grab_cam: np.ndarray        # grab-marker centre
    right_cam: np.ndarray       # board +X, unit
    up_cam: np.ndarray          # board +Y, unit
    normal_cam: np.ndarray      # board +Z, unit, pointing towards the camera
    distance: float             # metres to the grab marker
    ambiguous: bool
    ambiguity_ratio: float      # best/second reprojection error. Near 1 means
                                # the two PnP solutions are indistinguishable.
    rms_px: float               # reprojection error of the accepted solution
    worst_px: float             # ... and its worst single corner
    marker_ids: List[int]
    joint: bool                 # True = one rigid solve over every corner
    per_marker: List[MarkerSighting] = field(default_factory=list)

    @property
    def n_markers(self) -> int:
        return len(self.marker_ids)


def reprojection_error(objp, imgp, rvec, tvec, K, D):
    """(rms, worst) pixel error of a solution, over every corner given."""
    projected, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
    residual = np.linalg.norm(projected.reshape(-1, 2) - imgp.reshape(-1, 2), axis=1)
    return float(np.sqrt(np.mean(residual ** 2))), float(np.max(residual))


# ----------------------------------------------------------------- locator

class BoardLocator:
    """Detect the board markers in one frame and solve for the board's pose.

    ONE RIGID SOLVE, NOT A FUSION OF SEPARATE ONES
    ----------------------------------------------
    The original ring_grab solved each marker independently with IPPE_SQUARE
    and then averaged the results, weighting by 1/d^2. That is the wrong
    shape of estimator for this problem and it was measurably worse.

    A single square is very weakly constrained in tilt -- that is the whole
    reason IPPE returns two solutions -- so the three markers flip
    INDEPENDENTLY, and averaging one correct rotation with one flipped one
    produces a rotation that is neither. On a synthetic board at 2 m the
    averaged answer put the grab marker 7 cm out with the board plane 9
    degrees off, and the three markers disagreed with each other by 33 cm.

    Solving all twelve corners as one rigid body, with the known board
    geometry supplying the relative positions, removes that entirely: the
    same board at the same range comes out 0.3 cm out. The markers can no
    longer flip separately because there is only one rotation to solve for,
    and the half-metre vertical baseline between the top and bottom markers
    constrains it far better than any single 10 cm square can.

    So: IPPE over every visible corner when two or more markers are up, and
    IPPE_SQUARE on the one marker when only one is. The single-marker case is
    the weak one, which is why min_markers can be raised to 2.

    WHAT IS STILL WEAK, AND IT MATTERS
    ----------------------------------
    Rotation ABOUT THE BOARD'S VERTICAL AXIS -- the board yaw, which is what
    the approach flies along -- is the least observable quantity a VERTICAL
    STACK of markers can offer, because rotating about a vertical axis barely
    moves markers that are themselves stacked vertically. Only each marker's
    own 10 cm of width sees it.

    Measured on synthetic boards, worst case at each range:

        0.6 - 1.5 m     under 1 degree      the fine alignment lives here
        2.0 m           3 - 9 degrees       course line-up, good enough
        3.0 m           4 - 13 degrees      acquisition only

    which is exactly why the mission re-measures at the fine standoff instead
    of committing to the normal it saw during the sweep, and why the fine
    standoff defaults to 1.0 m. If you need the yaw better than that, the fix
    is board geometry, not code: put two markers side by side.
    """

    def __init__(self, geometry: BoardGeometry):
        self.geometry = geometry
        self.detect = make_detector(geometry.dictionary_id)
        # One resolver, not one per marker: there is one rotation being solved
        # for now, so there is one flip to break the tie on.
        self.resolver = AmbiguityResolver()
        self.objp = {i: obj_points(geometry.marker_size) + geometry.offsets[i]
                     for i in geometry.ids}
        self.last_seen_ids: List[int] = []
        self.last_all_ids: List[int] = []

    def find(self, gray: np.ndarray, K: np.ndarray,
             D: np.ndarray) -> Optional[BoardSighting]:
        corners, ids = self.detect(gray)
        self.last_all_ids = [] if ids is None else [int(i) for i in ids.flatten()]

        if not self.last_all_ids:
            self.resolver.reset()
            self.last_seen_ids = []
            return None

        found = {int(mid): c for c, mid in zip(corners, ids.flatten())
                 if int(mid) in self.geometry.offsets}
        self.last_seen_ids = sorted(found)
        if not found:
            # Markers in frame, but none of ours. Do NOT reset the resolver:
            # the board may be briefly occluded by something carrying its own
            # tags, and the previous rotation is still the best tiebreaker
            # there is when it comes back.
            return None

        order = sorted(found)
        objp = np.concatenate([self.objp[i] for i in order]).astype(np.float64)
        imgp = np.concatenate(
            [found[i].reshape(4, 2) for i in order]).astype(np.float64)

        # IPPE_SQUARE is only valid for a single square and is the better
        # conditioned solver for that case; IPPE handles the general planar
        # point set the multi-marker board is.
        flags = (cv2.SOLVEPNP_IPPE_SQUARE if len(order) == 1
                 else cv2.SOLVEPNP_IPPE)
        rvec, tvec = self.resolver.solve(objp, imgp, K, D, flags=flags)
        if rvec is None:
            return None

        rms, worst = reprojection_error(objp, imgp, rvec, tvec, K, D)
        R, _ = cv2.Rodrigues(rvec)

        # The board frame's origin is the middle of the stack; the mission
        # measures everything from the grab marker, so shift there.
        grab_opt = np.asarray(tvec, dtype=float) + R @ self.geometry.offsets[
            self.geometry.grab_id]

        to_frd = R_FRD_OPT
        return BoardSighting(
            grab_cam=to_frd @ grab_opt,
            right_cam=to_frd @ R[:, 0],
            up_cam=to_frd @ R[:, 1],
            normal_cam=to_frd @ R[:, 2],
            distance=float(np.linalg.norm(grab_opt)),
            ambiguous=self.resolver.ambiguous,
            ambiguity_ratio=self.resolver.last_ratio,
            rms_px=rms,
            worst_px=worst,
            marker_ids=order,
            joint=len(order) > 1,
            per_marker=[MarkerSighting(i, found[i]) for i in order],
        )
