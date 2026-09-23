"""
Takeoff -> sweep for the window -> estimate where it is -> line up on it ->
fly through it -> land. Localisation is PMW3901 optical flow + TFmini Plus rangefinder, fused in PX4.

PORTED TO THE REALSENSE D435i + PMW3901. The airframe this was written for
had a ZED stereo camera and an ARK Flow (optical flow WITH an integrated
rangefinder). It now has an Intel RealSense D435i and a PMW3901 optical flow
sensor with a SEPARATE Benewake TFmini Plus lidar. What that changed:

    camera      /zed/zed_node/... -> /camera/camera/... and the depth map is
                16UC1 millimetres instead of 32FC1 metres. All of that lives
                in window_detect.py; this node only ever sees the ANGLES on
                /window_geometry, which are metric either way.
    field of view  ~90x60 deg -> 70x43 deg. This one DID reach this file:
                STANDOFF_DISTANCE went 1.60 -> 2.00 m and effective_standoff()
                was added, because on this lens a big aperture stops fitting
                in the frame at distances the approach actually flies.
    flow        ARK Flow -> PMW3901 + TFmini Plus. NO CODE CHANGE. Every
                horizontal gate in the inherited OffboardSequence is written
                against EKF2's own flags and VehicleLocalPosition, never
                against a sensor-specific topic, so which flow sensor is
                bolted on is a PX4 parameter question. See flow_is_healthy()
                in offboard_sequence.py and the notes in the launch file.

The D435i is a CAMERA here and nothing else. It supplies the colour and depth
frames the window is found and measured in; it does NOT supply the vehicle's
position. That comes from PX4's EKF2 fusing the PMW3901's optical flow and
the TFmini Plus, exactly as in sequence_test.launch.py -- no VIO bridge, no
external vision, no EKF2_EV_* parameters. See window_traverse.launch.py.

What that costs, and it is not nothing: optical flow needs ground texture and
a rangefinder reading above FLOW_MIN_AGL, so the horizontal estimate is not
trustworthy until the aircraft is off the ground. The inherited x/y latch
already handles this -- it holds zero velocity until the flow has been healthy
for FLOW_SETTLE_SECONDS and only then anchors to a point. The window estimate
is built in NED from that estimate, so it is no better than the flow is.

HOW FAR IT IS ALLOWED TO TURN
    Two things bound the yaw, and both exist because a large, fast turn after
    takeoff is the one failure of this flight that is always a bug and never
    a manoeuvre.

    yaw_cone_deg is a hard cone about the heading the aircraft armed on --
    50 degrees either side by default. Every heading this node commands is
    clamped into it, and a window estimate whose bearing falls outside it is
    refused, not flown at: at 90 or 180 degrees off the takeoff heading the
    thing being measured is a reflection, a doorway behind the aircraft or a
    pose built on a bad attitude, never the window the aircraft was pointed
    at before it armed.

    The other is EKF2's own yaw. The commanded yaw is an ABSOLUTE NED
    heading, so when EKF2 re-datums its heading -- mag fusion settling after
    takeoff is the usual trigger indoors -- the setpoint suddenly names a
    direction the airframe is not pointing, and PX4 spins to it at full rate.
    OffboardSequence._apply_heading_reset shifts the commanded yaw by the
    same delta so nothing turns, and _on_heading_reset here rotates the
    window pose and the traverse line with the frame so the approach survives
    the reset too. The log line to look for is "EKF2 HEADING reset".

    arm -> sit on the ground -> climb -> hold -> SCAN (yaw sweep) ->
    LOCK (stop, face it, build a pose estimate) -> [RECENTRE, only if the
    window is hanging off the edge of the frame: yaw at what is visible until
    the whole aperture is in view] -> AIM (yaw onto the window normal) ->
    ALIGN (fly to a point standoff_distance in front of the window, on its
    axis, at a height that clears the airframe through the aperture -- NOT
    the window centre, see window_altitude) -> TRAVERSE (commit and fly
    through) -> CLEAR (hold beyond it) -> land

    q -> abort into a controlled descent.   k -> force-disarm.

WHAT IS INHERITED AND WHY
-------------------------
Everything up to and including the lock is somebody else's already-flown
code, reached by inheriting from both halves of it:

    WindowScan          the yaw sweep, the debounced detection, the lock,
                        the flight clock
    OffboardSequence    arming, the climb, the ramps and leashes, the
                        optical-flow health predicate and the x/y latch, the
                        estimator-reset bookkeeping, the descent, the
                        touchdown detection, the keyboard aborts

so the only new flight code in this file is the four stages after the lock,
and the only new sensing code is the window pose estimator. Nothing about
how the aircraft climbs or lands is duplicated here, which is the point:
those are the parts that hurt people, and they should exist once.

WHICH FRAME THE SETPOINTS ARE IN
--------------------------------
This is the question that has to be answered before any of the rest makes
sense, so: every setpoint this node sends PX4 is an ABSOLUTE POINT IN THE
PX4 LOCAL NED FRAME -- the same frame /fmu/out/vehicle_local_position
reports x, y and z in, with its origin wherever EKF2 datumed itself and z
POSITIVE DOWN. Not body-relative, not relative to the arming point.

The inherited sequence node hides that behind direction words ("forward
1.0" resolves against a reference yaw into an NED target), but underneath
it is walking `hold_x` / `hold_y` -- an NED point -- towards `move_target_x`
/ `move_target_y`, also an NED point, and publishing that point. This node
skips the direction words and computes the NED targets directly, because
the window's position is naturally an absolute point and converting it into
"forward 1.4, right 0.3" and back again would only lose precision.

The one axis that is relative is altitude, and only in the bookkeeping:
`commanded_altitude` is metres above the ARMING POINT, and it is turned into
an NED z as `home_z - commanded_altitude` before it goes anywhere near PX4.

WHERE THE WINDOW'S POSITION COMES FROM
--------------------------------------
window_detect publishes /window_geometry every frame it sees a quadrilateral:
four corners and the centre, each as (depth, azimuth, elevation) in the
CAMERA frame. Turning that into a point in NED is three transforms:

    1. rays to camera-frame points   x=d, y=d*tan(az), z=-d*tan(el)
       This is exact rather than approximate because the D435i's depth is the
       distance along the optical axis, not the slant range.
    2. camera frame to body FRD      the fixed mounting rotation and lever
       arm (cam_x/cam_y/cam_z, cam_roll/cam_pitch/cam_yaw -- measured to
       the D435i's LEFT IMAGER, which is where librealsense puts the optical
       frame origin, not to the middle of the case)
    3. body FRD to NED               rotate by the vehicle attitude
       quaternion, then add the vehicle position

Step 3 uses the full attitude, not just the heading. That matters more than
it looks: a vehicle translating at 0.4 m/s sits at 5-10 degrees of pitch,
and at 3 m range a 10 degree pitch error puts the window half a metre off
vertically. Using the heading alone would make the aircraft fly at a window
that appears to move up and down as it accelerates.

THE OUTLIERS, WHICH ARE THE ACTUAL PROBLEM
------------------------------------------
Stereo depth on a thin green frame is not a well-behaved measurement. It
fails in one specific way: a sample box that lands a few pixels off the
frame reads the WALL BEHIND (metres too far) or nothing at all (zero), and
one such corner drags a naive four-corner average metres out of position.
So no single frame is ever trusted. Five independent filters stand between
a depth pixel and a setpoint:

    truncation      a quad with a corner at the image edge is refused
                    outright, before any of the rest of this runs. It is the
                    only filter that is not a consistency test, because a
                    truncated window is entirely self-consistent: it is a
                    real, planar, correctly proportioned rectangle. It is
                    just not the window -- it is the part of it that fits in
                    the frame, narrower than the real aperture and with its
                    centre pulled off the real one by half of whatever was
                    cut off. No amount of filtering recovers the missing
                    half, so the sample is not taken at all, and RECENTRE
                    goes and gets a view that contains the whole thing.
    per corner      a non-positive depth, or one outside
                    [depth_min, depth_max], voids that corner
    per sample      the four corner depths must agree with their own median
                    to within corner_spread; the reconstructed quad must be
                    planar to within plane_tolerance, must have sides
                    between window_min_size and window_max_size, must have
                    opposite sides matching to within side_mismatch, and its
                    normal must be within max_tilt_deg of horizontal
    innovation      once an estimate exists, a sample whose centre is more
                    than gate_metres from it, or whose normal is more than
                    gate_yaw_deg off it, is rejected outright
    temporal        the estimate is the component-wise MEDIAN of every
                    accepted sample in the last buffer_seconds, not a mean
                    and not an EMA of one -- a median is the filter that
                    ignores an outlier instead of averaging it in
    quorum          nothing flies anywhere until min_samples accepted
                    samples exist and the newest is younger than pose_max_age

A median over a rolling window is deliberately chosen over the exponential
average in drone_imav_obs_course: an EMA with alpha=0.25 still moves 25 cm
towards a sample that is a metre wrong, and it does so on the first frame.
The median moves not at all until half the buffer agrees.

If the gate rejects gate_reset_count samples in a row, the buffer is thrown
away and rebuilt from scratch -- that is the case where the estimate itself
is the thing that is wrong, and refusing every sample forever is worse than
starting again.

DOES IT KEEP UPDATING WHILE IT FLIES?
-------------------------------------
Yes, through SCAN, LOCK, AIM and ALIGN: every frame goes into the estimator
and the approach target is recomputed from the current estimate on every
tick, so the aircraft is chasing the window's best-known position, not the
one it had when it first saw it.

At the start of TRAVERSE it COMMITS: the target is frozen and the estimator
is ignored. That is not laziness, it is the only safe reading of the
geometry. Passing through a window means the window leaves the field of
view, fills it, and finally is behind the camera; the last few metres of
depth on a frame edge are the least trustworthy data the camera produces,
and the aircraft is at its least able to act on a correction. The estimate
that lined the aircraft up from a metre and a half away, with the whole
window in frame, is better than anything measurable from inside it.

If vision itself dies during the traverse, the aircraft does NOT stop in the
window frame. It flies the remaining distance as a fixed velocity along the
committed heading for up to blind_traverse_seconds and then lands. Stopping
halfway through an aperture is the one outcome worth spending open-loop
seconds to avoid.

SEEING THE WHOLE WINDOW BEFORE MEASURING IT
-------------------------------------------
The sweep stops on the FIRST frame that contains a window, and there is no
reason that frame should contain all of it: the nose is turning, and the
aperture enters the field of view from one side. Everything downstream --
the centre, the width, the normal, the clearance arithmetic -- is computed
from a quadrilateral, and a quadrilateral clipped by the image border is a
perfectly good quadrilateral describing the wrong aperture.

So window_detect flags any quad with a corner at the frame edge, the
estimator refuses those samples, and RECENTRE is what makes that refusal
useful instead of merely correct: it nudges the yaw towards the visible
centroid, which is always on the opposite side of the image centre from the
clipped edge, so turning towards it brings the missing part into view. The
nudge is deliberately small -- recentre_yaw_step_deg per tick, at most
recentre_yaw_limit_deg in total for an attempt -- because the question being
asked is only "is that window edge actually the frame edge?", and a few
degrees answers it: a genuinely clipped window un-clips almost immediately,
a window that merely sits near the border does not move off it. Commanding
the whole centroid bearing at once would instead swing the aperture out of
frame on the far side and set the loop oscillating. It turns
rather than translates -- a turn on the spot leaves the flow-based position
hold undisturbed and costs nothing if the bearing is wrong, whereas
translating on a measurement already known to be wrong is the failure being
prevented. If yawing does not help within recentre_backoff_seconds the
window is too wide for the field of view from where the aircraft is, and it
retreats along its line of sight and looks again.

YAW
---
The nose is kept pointing along the direction of travel -- i.e. at the
window, and then through it -- for the whole approach. Two reasons, neither
of them cosmetic: the camera has to keep seeing the window for the estimate
to keep updating, and a vehicle crabbing sideways through an aperture needs
the aperture to be wider than its diagonal rather than its width -- which is
now charged for explicitly rather than assumed away, see swept_width(). The
yaw is
walked by the inherited ramp at yaw_rate, leashed to the measured heading,
and AIM does the bulk of the turn standing still, before any translation, so
the two never happen fast at the same time -- simultaneous yaw and
translation is the single most reliable way to make an IMU-less stereo
camera lose tracking.
"""

import math
import threading
import time
from collections import deque

import numpy as np

import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitude, VehicleStatus
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from drone_testing.offboard_sequence import spin_node, wrap_pi
from drone_testing.window_scan import WindowScan


# ----------------------------------------------------------------- geometry

def quat_rotate(q, v):
    """Rotate v by the Hamiltonian quaternion q = (w, x, y, z).

    px4_msgs/VehicleAttitude.q is the rotation that takes a vector expressed
    in the BODY (FRD) frame to the same vector expressed in the local NED
    frame, so this is exactly the body -> NED step and needs no inverse.
    """
    w = q[0]
    u = np.asarray(q[1:4], dtype=float)
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def rpy_to_matrix_frd(roll, pitch, yaw):
    """Mounting rotation as an FRD matrix, from angles given ROS-style.

    The angles are the pose of the camera in the body frame in the ROS
    convention -- x forward, y LEFT, z UP, applied yaw then pitch then roll --
    because those are the three numbers any vision bridge on this airframe takes
    for the same camera, and having the two nodes disagree about the sign of
    cam_pitch is a bug nobody would find in the air.

    FLU and FRD differ by flipping y and z, and D = diag(1, -1, -1) is its own
    inverse, so R_frd = D R_flu D.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    d = np.diag([1.0, -1.0, -1.0])
    return d @ (rz @ ry @ rx) @ d


# ------------------------------------------------------------- the estimator

class WindowEstimator:
    """Turns a stream of camera-frame quadrilaterals into one NED window pose.

    Deliberately free of ROS and of the flight node: it takes numbers in and
    gives numbers out, which is what makes the rejection logic -- the part
    that actually decides whether the aircraft flies at the right place --
    testable on the bench without a vehicle.

    add() returns (accepted, reason). Every rejection carries the reason it
    was rejected, and the node logs a rolling tally of them, because "the
    window estimate is not converging" is useless and "37 of the last 40
    samples failed the planarity test" tells you the sample box is landing on
    the wall behind the frame.
    """

    def __init__(self, *, depth_min, depth_max, corner_spread, corner_spread_frac,
                 plane_tolerance, min_size, max_size, side_mismatch, max_tilt,
                 buffer_seconds, buffer_max, min_samples, gate_metres, gate_yaw,
                 gate_reset_count):
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.corner_spread = corner_spread
        self.corner_spread_frac = corner_spread_frac
        self.plane_tolerance = plane_tolerance
        self.min_size = min_size
        self.max_size = max_size
        self.side_mismatch = side_mismatch
        self.max_tilt = max_tilt            # rad, off horizontal
        self.buffer_seconds = buffer_seconds
        self.min_samples = min_samples
        self.gate_metres = gate_metres
        self.gate_yaw = gate_yaw            # rad
        self.gate_reset_count = gate_reset_count

        self.samples = deque(maxlen=buffer_max)
        self.rejections = {}
        # add() runs on the subscription thread and estimate()/fresh_count()
        # on the setpoint-timer thread (see OffboardSequence's callback
        # groups). deque.append is atomic, but iterating one while it is
        # appended to is not -- CPython raises "deque mutated during
        # iteration" -- and that iteration is _fresh(), on the path that
        # produces every setpoint the approach flies. Hence a lock rather than
        # a bet on the GIL.
        self._lock = threading.RLock()
        self.accepted_total = 0
        self.consecutive_gated = 0
        self.last_reason = ''

    # ------------------------------------------------------------ ingestion

    def add(self, geometry, q_att, p_ned, r_cam, t_cam, now):
        """One /window_geometry message, with the vehicle pose it belongs to.

        geometry is the 5x3 (depth_m, az_deg, el_deg) array; only the four
        corner rows are used. The centre row is redundant here -- the centroid
        of four validated corners is a better centre than one depth sample at
        the middle of an OPEN window, where the depth pixel is looking at
        whatever is on the far side of the room.
        """
        with self._lock:
            return self._add(geometry, q_att, p_ned, r_cam, t_cam, now)

    def _add(self, geometry, q_att, p_ned, r_cam, t_cam, now):
        corners_cam = []
        for depth, az_deg, el_deg in geometry[:4]:
            if not np.isfinite(depth) or depth <= 0.0:
                return self._reject('corner depth missing')
            if depth < self.depth_min or depth > self.depth_max:
                return self._reject('corner depth out of range')
            az = math.radians(float(az_deg))
            el = math.radians(float(el_deg))
            corners_cam.append([float(depth),
                                float(depth) * math.tan(az),
                                -float(depth) * math.tan(el)])
        corners_cam = np.array(corners_cam)

        # The four depths must agree with each other. A window seen at an
        # angle genuinely has a depth spread, so the allowance is the larger of
        # an absolute and a proportional one -- a 20 cm disagreement at 1.5 m
        # is a squint, the same 20 cm at 6 m is a corner on the far wall.
        depths = corners_cam[:, 0]
        median_depth = float(np.median(depths))
        allowed = max(self.corner_spread, self.corner_spread_frac * median_depth)
        if float(np.max(np.abs(depths - median_depth))) > allowed:
            return self._reject('corner depths disagree')

        # Camera -> body FRD -> NED. Done before the shape tests so those are
        # applied to the same points the setpoint will be derived from.
        corners_body = corners_cam @ r_cam.T + t_cam
        corners_ned = np.array([quat_rotate(q_att, p) for p in corners_body]) + p_ned

        centre = corners_ned.mean(axis=0)
        rel = corners_ned - centre

        # Sides, in the detector's corner order: TL, TR, BR, BL walking round
        # the quad, so 0-1 and 3-2 are the horizontals and 0-3 and 1-2 the
        # verticals.
        top = np.linalg.norm(corners_ned[1] - corners_ned[0])
        bottom = np.linalg.norm(corners_ned[2] - corners_ned[3])
        left = np.linalg.norm(corners_ned[3] - corners_ned[0])
        right = np.linalg.norm(corners_ned[2] - corners_ned[1])
        width = 0.5 * (top + bottom)
        height = 0.5 * (left + right)

        if not (self.min_size <= width <= self.max_size
                and self.min_size <= height <= self.max_size):
            return self._reject('implausible window size')
        if (abs(top - bottom) > self.side_mismatch * max(top, bottom)
                or abs(left - right) > self.side_mismatch * max(left, right)):
            return self._reject('opposite sides disagree')

        # Planarity. The smallest right-singular vector of the centred corners
        # is the plane normal, and the residual along it is how far from a
        # plane those four points are. A corner that has grabbed the wall
        # behind fails this even when it passed the depth-spread test, because
        # it is off the plane in the direction the spread test cannot see.
        try:
            _, _, vt = np.linalg.svd(rel)
        except np.linalg.LinAlgError:
            return self._reject('degenerate quad')
        normal = vt[2]
        if float(np.max(np.abs(rel @ normal))) > self.plane_tolerance:
            return self._reject('corners not coplanar')

        # Point the normal back at the aircraft, so "in front of the window"
        # is unambiguously +normal for everything downstream.
        if float(np.dot(normal, p_ned - centre)) < 0.0:
            normal = -normal

        # A window is vertical. A normal that is not horizontal means the quad
        # is the floor, a ceiling light or a badly-cornered detection, and
        # flying along it would fly the aircraft into the ground.
        horizontal = math.hypot(normal[0], normal[1])
        if horizontal < math.cos(self.max_tilt):
            return self._reject('window is not vertical')
        normal_h = np.array([normal[0], normal[1], 0.0]) / horizontal

        # Innovation gate against the estimate we already believe.
        est = self.estimate(now)
        if est is not None:
            if float(np.linalg.norm(centre - est['centre'])) > self.gate_metres:
                return self._gated('centre jumped')
            if abs(wrap_pi(math.atan2(normal_h[1], normal_h[0])
                           - math.atan2(est['normal'][1], est['normal'][0]))) > self.gate_yaw:
                return self._gated('normal swung')

        self.consecutive_gated = 0
        self.accepted_total += 1
        self.last_reason = ''
        self.samples.append({
            't': now,
            'centre': centre,
            'normal': normal_h,
            'width': width,
            'height': height,
        })
        return True, ''

    def rotate_frame(self, delta, pivot):
        """Turn every stored sample by `delta` about `pivot` (NED x/y).

        Called when EKF2 re-datums yaw. The window did not move and the
        aircraft did not move, but every sample in here was placed using an
        attitude that has just been declared wrong by `delta`, so as a set
        they are rotated by exactly that much about the point they were
        measured from. Rotating them back keeps the estimate continuous
        across the reset instead of throwing away a buffer that took several
        seconds of stationary hover to fill.
        """
        if abs(delta) < 1e-6:
            return
        c, s = math.cos(delta), math.sin(delta)
        px, py = float(pivot[0]), float(pivot[1])
        with self._lock:
            for sample in self.samples:
                centre = sample['centre']
                dx, dy = centre[0] - px, centre[1] - py
                centre[0] = px + c * dx - s * dy
                centre[1] = py + s * dx + c * dy
                normal = sample['normal']
                nx, ny = normal[0], normal[1]
                normal[0] = c * nx - s * ny
                normal[1] = s * nx + c * ny

    def _reject(self, reason):
        self.rejections[reason] = self.rejections.get(reason, 0) + 1
        self.last_reason = reason
        return False, reason

    def _gated(self, reason):
        self.consecutive_gated += 1
        if self.consecutive_gated >= self.gate_reset_count:
            # Every sample disagrees with the estimate. At that point the
            # estimate is the minority opinion and keeping it is how a vehicle
            # ends up flying confidently at nothing.
            self.samples.clear()
            self.consecutive_gated = 0
            return self._reject(reason + ' -- estimate discarded, rebuilding')
        return self._reject(reason)

    # -------------------------------------------------------------- output

    def _fresh(self, now):
        cutoff = now - self.buffer_seconds
        return [s for s in self.samples if s['t'] >= cutoff]

    def estimate(self, now):
        """The current robust pose, or None if there is not enough evidence.

        centre and normal are component-wise medians over the buffer. The
        median of a set of unit vectors is not a unit vector, so the normal is
        re-normalised; with samples that have already passed the innovation
        gate they are all within gate_yaw of each other and the renormalised
        median is a sane direction.
        """
        with self._lock:
            fresh = self._fresh(now)
            if len(fresh) < self.min_samples:
                return None

            centre = np.median(np.array([s['centre'] for s in fresh]), axis=0)
            normal = np.median(np.array([s['normal'] for s in fresh]), axis=0)
            norm = float(np.linalg.norm(normal[:2]))
            if norm < 1e-6:
                return None
            normal = np.array([normal[0] / norm, normal[1] / norm, 0.0])

            return {
                'centre': centre,
                'normal': normal,
                'width': float(np.median([s['width'] for s in fresh])),
                'height': float(np.median([s['height'] for s in fresh])),
                'samples': len(fresh),
                'age': now - fresh[-1]['t'],
            }

    def fresh_count(self, now):
        """How many accepted samples are inside the buffer window."""
        with self._lock:
            return len(self._fresh(now))

    def rejection_summary(self, limit=3):
        with self._lock:
            if not self.rejections:
                return 'none'
            worst = sorted(self.rejections.items(), key=lambda kv: -kv[1])[:limit]
        return ', '.join(f"{name} x{count}" for name, count in worst)


# ---------------------------------------------------------------- the flight

class WindowTraverse(WindowScan):
    """Sweep, lock, line up, fly through, on PMW3901 + TFmini Plus localisation.

    One base, and that is the point. WindowScan brings the sweep, the lock and
    the flight clock; the OffboardSequence underneath it brings arming, the
    climb, the ramps, the descent AND `flow_is_healthy`, which is the ARK
    Flow predicate every inherited horizontal gate is written against. So the
    only new flight code here is the four stages after the lock.

    This deliberately does NOT inherit OffboardSequenceVio. That class exists
    to point the same gates at external visual odometry, and it is the right
    base if you ever go back to it -- but the ZED's VO was resetting EKF2's
    horizontal estimate several times a second on this airframe, so the camera
    is used for seeing the window and nothing else. Nothing about the
    RealSense swap revisits that decision: the D435i has no odometry of its
    own at all (that was the T265), so using it for position would mean
    standing up RTAB-Map or OpenVINS first. See
    tools/ekf_reset_rate.py for how that was measured, and note the one
    behavioural consequence: unlike the vision version, `flow_is_healthy`
    tests dist_bottom against FLOW_MIN_AGL, so the lateral estimate is not
    usable while the aircraft is sitting on the ground.
    """

    # No sweep by default, unlike window_scan. This flight is pointed at the
    # window before it arms, and the aircraft that is already looking at one
    # has nothing to search for -- so it climbs, holds, and waits for the
    # detector on the heading it took off with. It is also the difference
    # between the launch file (which passes scan_span_deg=0) and running the
    # node bare with `ros2 run`, which used to inherit WindowScan's 20 degree
    # sweep and give the two paths different behaviour. Set scan_span_deg to
    # bring the sweep back.
    SCAN_SPAN = 0.0

    RECENTRE = "RECENTRE"
    AIM = "AIM"
    ALIGN = "ALIGN"
    TRAVERSE = "TRAVERSE"
    CLEAR = "CLEAR"

    TRAVERSE_STAGES = (RECENTRE, AIM, ALIGN, TRAVERSE, CLEAR)

    # ---- the approach -----------------------------------------------------
    STANDOFF_DISTANCE = 2.00    # m in front of the window plane the approach
                                # aims for. Far enough that the whole window is
                                # still in frame and close enough that the run
                                # through it is short.
                                #
                                # RAISED FROM 1.60 FOR THE REALSENSE D435i.
                                # This number is set by the camera's field of
                                # view, and the D435i's COLOUR sensor is much
                                # narrower than the ZED's was -- in the axis
                                # that binds, by a lot:
                                #
                                #     ZED (HD720)   ~90 deg H x ~60 deg V
                                #     D435i colour   70 deg H x  43 deg V
                                #       (measured on this unit: CameraInfo
                                #        fx=906.8 fy=907.4 at 1280x720)
                                #
                                # VERTICAL is what binds, and it is the axis
                                # that got worse. To keep a window of height H
                                # fully in frame the camera must be at least
                                #
                                #     d = (H/2) / tan(VFOV/2)
                                #
                                # tan(43.3/2) = 0.397, so d = 1.26*H on the
                                # D435i where it was 0.87*H on the ZED. A 1.0 m
                                # window needs 1.26 m and a 1.2 m one needs
                                # 1.51 m -- so the old 1.60 m default had gone
                                # from ~1.8x margin to ~1.06x, i.e. none.
                                #
                                # 2.00 m restores a sane margin for windows up
                                # to ~1.3 m tall, and MIN_STANDOFF_MARGIN below
                                # pushes it out further at run time for anything
                                # bigger, using the aperture this flight has
                                # actually measured rather than an assumption.
    # ---- keeping the window in frame at the standoff point ----------------
    # The FOV the standoff clamp is computed against. Defaults are the D435i's
    # colour sensor. These are NOT read from CameraInfo: window_detect already
    # subscribes to it and bakes the true intrinsics into the ANGLES it
    # publishes on /window_geometry, so this node never needs the intrinsics
    # for measurement -- only for this one geometric sanity clamp. Override
    # them if you fit a different lens.
    CAMERA_HFOV_DEG = 70.4
    CAMERA_VFOV_DEG = 43.3
    MIN_STANDOFF_MARGIN = 1.25  # how much bigger than the bare "just fits"
                                # distance the standoff must be. 1.0 would put
                                # the window corners exactly on the frame edge,
                                # where window_detect flags them TRUNCATED and
                                # the corner depths are least trustworthy --
                                # which is precisely when the estimate the
                                # approach is being steered by would go bad.
    MAX_STANDOFF_DISTANCE = 4.0 # m. A ceiling on that clamp, so a wildly
                                # over-estimated aperture cannot walk the
                                # approach point backwards out of the arena.
    EXIT_DISTANCE = 1.50        # m beyond the window plane the traverse ends
    ALTITUDE_OFFSET = 0.0       # m added to the window centre height. Positive
                                # is higher. Leave at 0 unless the detected
                                # quad is known to sit off-centre on the frame.

    # ---- the airframe -----------------------------------------------------
    # The aircraft is not a point. It is 260 mm tall and 260 mm wide, and the
    # camera sits 120 mm above the bottom of the landing gear -- so the thing that
    # actually has to fit through the aperture hangs BELOW the thing that
    # measures it. Flying the vehicle origin at the window centre put the
    # landing gear on the sill on the first attempt and tipped the aircraft
    # over; these numbers exist so that cannot happen again.
    GEAR_BELOW_CAMERA = 0.120   # m from the camera down to the bottom of the
                                # landing gear
    DRONE_HEIGHT = 0.260        # m total, gear bottom to the highest point
    DRONE_WIDTH = 0.260         # m across, the widest point (prop tips)
    VERTICAL_CLEARANCE = 0.150  # m of air wanted between the landing gear and
                                # the sill, and between the top and the lintel,
                                # when the aperture is big enough to afford it
    LATERAL_CLEARANCE = 0.150   # m wanted either side. Advisory: nothing
                                # steers sideways off it, it only warns.
    HARD_CLEARANCE = 0.030      # m. An aperture that cannot give even this
                                # much around the airframe is not flyable and
                                # the attempt is abandoned rather than flown.
    SILL_BIAS = 0.100           # m of deliberate extra height above the
                                # airframe-centred solution, spent only if the
                                # aperture has it to give. The errors here are
                                # not symmetric: altitude tracking sags under
                                # load, the rangefinder's idea of the floor
                                # moves, and the two failures are not
                                # equivalent -- brushing the lintel with a
                                # prop guard is a bad traverse, catching the
                                # gear on the sill flips the aircraft.

    APPROACH_SPEED = 0.30       # m/s the carrot is walked at during ALIGN
    TRAVERSE_SPEED = 0.45       # m/s during the run through. Faster than the
                                # approach: less time in the aperture, and by
                                # then the estimate is frozen so there is
                                # nothing left to track.

    ALIGN_TOLERANCE = 0.18      # m. A plain radius, used where a distance is
                                # just a distance: arriving at the far side of
                                # the traverse, and arriving at a back-off
                                # point. NOT the alignment gate -- see below.
    # The alignment gate splits that radius along the two axes it actually has,
    # because they do not cost the same. CROSS is perpendicular to the traverse
    # line: every centimetre of it is a centimetre off the window's centreline
    # and comes straight out of the lateral clearance, which on a 0.6 m window
    # is only 0.17 m per side to begin with. ALONG is up and down the approach
    # line, where being 20 cm early or late changes nothing except how long the
    # run through is. One 0.18 m sphere charged them at the same rate and let
    # the whole budget be spent sideways, which is how a prop found a jamb.
    ALIGN_CROSS_TOLERANCE = 0.06   # m perpendicular to the approach line
    ALIGN_ALONG_TOLERANCE = 0.25   # m along it
    ALIGN_ALT_TOLERANCE = 0.08  # m. Tight, because the vertical budget in a
                                # window is now spent on airframe clearance:
                                # 15 cm of altitude error is most of the gap
                                # between the landing gear and the sill.
    ALIGN_YAW_TOLERANCE = math.radians(8.0)
    ALIGN_SETTLE_SECONDS = 1.5  # all three held simultaneously for this long
    AIM_YAW_TOLERANCE = math.radians(12.0)

    # ---- re-centring on a truncated window --------------------------------
    # window_detect flags a quad with a corner at the image edge as TRUNCATED:
    # the aperture it measured is the visible PART of a window, narrower than
    # the real one and with its centre pulled off the real one by half of
    # whatever was cut off. Those samples are refused by the estimator, so a
    # window first spotted at the edge of the frame produces no pose at all --
    # which is correct, and useless on its own. RECENTRE is what does something
    # about it: yaw towards the visible centroid, which brings the clipped side
    # into view, and only then let the estimator build a pose.
    #
    # Yaw and not translate. Turning on the spot leaves the flow-based position
    # hold undisturbed and costs nothing if the estimate is wrong, whereas
    # translating on a measurement already known to be wrong is the failure it
    # is trying to prevent.
    #
    # The correction is a NUDGE, not a slew. The bearing of the visible
    # centroid can be tens of degrees off the optical axis, and commanding all
    # of it at once swings the aircraft far enough that the window leaves the
    # frame on the other side, the flow estimate is smeared, and the whole
    # thing oscillates. What is actually wanted is "is the right edge of the
    # window the right edge of the FRAME?" -- a couple of degrees of turn is
    # enough to answer that, because a window that was genuinely clipped
    # un-clips within a few degrees and one that was not stays put. So each
    # tick asks for at most RECENTRE_YAW_STEP, re-measuring in between, and the
    # cumulative turn away from the heading RECENTRE started at is capped at
    # RECENTRE_YAW_LIMIT. Past that the window is not merely nudged off the
    # edge -- it does not fit -- and the back-off is the right answer.
    RECENTRE_CLEAR_SECONDS = 0.6    # s the detection must stay untruncated
    RECENTRE_YAW_DEADBAND = math.radians(3.0)
    RECENTRE_YAW_STEP = math.radians(4.0)   # max commanded correction per tick
    RECENTRE_YAW_LIMIT = math.radians(20.0)  # max cumulative turn per attempt
    RECENTRE_TIMEOUT = 20.0         # s per attempt, before backing off
    RECENTRE_BACKOFF_SECONDS = 7.0  # s of fruitless yawing before deciding the
                                    # window is simply too close to fit in the
                                    # frame and backing away from it
    RECENTRE_BACKOFF = 0.60         # m backwards along the current heading
    RECENTRE_MAX_BACKOFFS = 2       # then give up

    # ---- how far the flight is allowed to turn ----------------------------
    # A hard cone about the heading the aircraft armed on, and the last line
    # of defence against flying at something that is not the window. The
    # mission is "the window is out in front, go through it": a target whose
    # bearing is 90 or 180 degrees off the takeoff heading is not that window,
    # it is a reflection, a doorway behind the aircraft, or an estimate built
    # on an attitude that was wrong. Nothing inside this node is allowed to
    # command a heading outside the cone, and an estimate that sits outside it
    # is refused rather than flown at. Set yaw_cone_deg to 0 to disable it.
    YAW_CONE_DEG = 50.0

    AIM_TIMEOUT = 25.0
    ALIGN_TIMEOUT = 60.0
    TRAVERSE_TIMEOUT = 25.0
    CLEAR_SECONDS = 4.0

    # ---- the estimate -----------------------------------------------------
    GEOMETRY_TOPIC = 'window_geometry'
    ATTITUDE_MAX_HZ = 30.0      # what the attitude subscription is decimated
                                # to. Geometry samples arrive at the camera
                                # frame rate at most, so anything above that
                                # is attitude nobody will ever pair with a
                                # window, bought at the price of executor time
                                # the setpoint timer needs.
    POSE_MAX_AGE = 1.5          # s. Older than this and the estimate is not
                                # evidence about where the window is now.
    POSE_LOST_TIMEOUT = 6.0     # s without a usable estimate during AIM/ALIGN
                                # before the attempt is given up on
    BLIND_TRAVERSE_SECONDS = 3.0

    DEPTH_MIN = 0.35
    DEPTH_MAX = 8.00
    CORNER_SPREAD = 0.25        # m
    CORNER_SPREAD_FRAC = 0.15   # of the median corner depth
    PLANE_TOLERANCE = 0.15      # m. Note the factor of four: a best-fit plane
                                # through four points splits a single bad
                                # corner's error between all of them, so a
                                # corner that is X out of plane only shows a
                                # residual of X/4 here. This test alone would
                                # pass a 60 cm outlier, which is why the depth
                                # spread test above it is the primary defence
                                # and this one is the backstop for the case the
                                # spread test cannot see -- a corner displaced
                                # ACROSS the frame rather than along the ray.
    WINDOW_MIN_SIZE = 0.35      # m
    WINDOW_MAX_SIZE = 3.00      # m
    SIDE_MISMATCH = 0.40        # fraction
    MAX_TILT_DEG = 35.0         # of the normal, off horizontal
    BUFFER_SECONDS = 2.5
    BUFFER_MAX = 60
    MIN_SAMPLES = 6
    GATE_METRES = 1.00
    GATE_YAW_DEG = 40.0
    GATE_RESET_COUNT = 25

    # A traversal is a longer flight than a scan, so the inherited 40 s clock
    # would land the aircraft in the middle of the approach.
    FLIGHT_SECONDS = 150.0

    def __init__(self):
        super().__init__('window_traverse')

        self.STANDOFF_DISTANCE = float(self._declare_number(
            'standoff_distance', self.STANDOFF_DISTANCE))

        # ADDED FOR THE D435i: the standoff clamp that keeps the whole window
        # inside a much narrower frame than the ZED gave. See
        # effective_standoff(). Params so a different lens does not need a
        # code change; tangents cached because this runs every tick.
        self.CAMERA_HFOV_DEG = float(self._declare_number(
            'camera_hfov_deg', self.CAMERA_HFOV_DEG))
        self.CAMERA_VFOV_DEG = float(self._declare_number(
            'camera_vfov_deg', self.CAMERA_VFOV_DEG))
        self.MIN_STANDOFF_MARGIN = float(self._declare_number(
            'min_standoff_margin', self.MIN_STANDOFF_MARGIN))
        self.MAX_STANDOFF_DISTANCE = float(self._declare_number(
            'max_standoff_distance', self.MAX_STANDOFF_DISTANCE))
        self._tan_half_hfov = math.tan(math.radians(self.CAMERA_HFOV_DEG) / 2.0)
        self._tan_half_vfov = math.tan(math.radians(self.CAMERA_VFOV_DEG) / 2.0)
        if self._tan_half_hfov <= 0.0 or self._tan_half_vfov <= 0.0:
            raise SystemExit(
                f"camera_hfov_deg={self.CAMERA_HFOV_DEG} and "
                f"camera_vfov_deg={self.CAMERA_VFOV_DEG} must both be in "
                "(0, 180). They divide into the standoff clamp.")
        self._last_standoff_logged = 0.0
        self.EXIT_DISTANCE = float(self._declare_number(
            'exit_distance', self.EXIT_DISTANCE))
        self.ALTITUDE_OFFSET = float(self._declare_number(
            'altitude_offset', self.ALTITUDE_OFFSET))
        self.GEAR_BELOW_CAMERA = float(self._declare_number(
            'gear_below_camera', self.GEAR_BELOW_CAMERA))
        self.DRONE_HEIGHT = float(self._declare_number(
            'drone_height', self.DRONE_HEIGHT))
        self.DRONE_WIDTH = float(self._declare_number(
            'drone_width', self.DRONE_WIDTH))
        self.VERTICAL_CLEARANCE = float(self._declare_number(
            'vertical_clearance', self.VERTICAL_CLEARANCE))
        self.LATERAL_CLEARANCE = float(self._declare_number(
            'lateral_clearance', self.LATERAL_CLEARANCE))
        self.HARD_CLEARANCE = float(self._declare_number(
            'hard_clearance', self.HARD_CLEARANCE))
        self.SILL_BIAS = float(self._declare_number('sill_bias', self.SILL_BIAS))
        # NaN = the airframe-centred rule above. A number = window centre plus
        # this many metres, e.g. 0.10 to aim just above the centre.
        self.TRAVERSE_CENTRE_OFFSET = float(self._declare_number(
            'traverse_centre_offset', getattr(self, 'TRAVERSE_CENTRE_OFFSET',
                                              float('nan'))))
        self.ALIGN_ALT_TOLERANCE = float(self._declare_number(
            'align_alt_tolerance', self.ALIGN_ALT_TOLERANCE))
        self.APPROACH_SPEED = float(self._declare_number(
            'approach_speed', self.APPROACH_SPEED))
        self.TRAVERSE_SPEED = float(self._declare_number(
            'traverse_speed', self.TRAVERSE_SPEED))
        self.ALIGN_TOLERANCE = float(self._declare_number(
            'align_tolerance', self.ALIGN_TOLERANCE))
        self.ALIGN_CROSS_TOLERANCE = float(self._declare_number(
            'align_cross_tolerance', self.ALIGN_CROSS_TOLERANCE))
        self.ALIGN_ALONG_TOLERANCE = float(self._declare_number(
            'align_along_tolerance', self.ALIGN_ALONG_TOLERANCE))
        self.RECENTRE_CLEAR_SECONDS = float(self._declare_number(
            'recentre_clear_seconds', self.RECENTRE_CLEAR_SECONDS))
        self.RECENTRE_YAW_STEP = math.radians(float(self._declare_number(
            'recentre_yaw_step_deg', math.degrees(self.RECENTRE_YAW_STEP))))
        self.RECENTRE_YAW_LIMIT = math.radians(float(self._declare_number(
            'recentre_yaw_limit_deg', math.degrees(self.RECENTRE_YAW_LIMIT))))
        self.RECENTRE_TIMEOUT = float(self._declare_number(
            'recentre_timeout', self.RECENTRE_TIMEOUT))
        self.RECENTRE_BACKOFF_SECONDS = float(self._declare_number(
            'recentre_backoff_seconds', self.RECENTRE_BACKOFF_SECONDS))
        self.RECENTRE_BACKOFF = float(self._declare_number(
            'recentre_backoff', self.RECENTRE_BACKOFF))
        self.RECENTRE_MAX_BACKOFFS = int(self._declare_number(
            'recentre_max_backoffs', self.RECENTRE_MAX_BACKOFFS))
        self.ALIGN_SETTLE_SECONDS = float(self._declare_number(
            'align_settle_seconds', self.ALIGN_SETTLE_SECONDS))
        self.ALIGN_YAW_TOLERANCE = math.radians(float(self._declare_number(
            'align_yaw_tolerance_deg', math.degrees(self.ALIGN_YAW_TOLERANCE))))
        self.ALIGN_TIMEOUT = float(self._declare_number(
            'align_timeout', self.ALIGN_TIMEOUT))
        self.TRAVERSE_TIMEOUT = float(self._declare_number(
            'traverse_timeout', self.TRAVERSE_TIMEOUT))
        self.CLEAR_SECONDS = float(self._declare_number(
            'clear_seconds', self.CLEAR_SECONDS))
        self.POSE_MAX_AGE = float(self._declare_number(
            'pose_max_age', self.POSE_MAX_AGE))
        self.POSE_LOST_TIMEOUT = float(self._declare_number(
            'pose_lost_timeout', self.POSE_LOST_TIMEOUT))
        self.BLIND_TRAVERSE_SECONDS = float(self._declare_number(
            'blind_traverse_seconds', self.BLIND_TRAVERSE_SECONDS))
        self.MIN_SAMPLES = int(self._declare_number('pose_min_samples', self.MIN_SAMPLES))
        self.YAW_CONE = math.radians(float(self._declare_number(
            'yaw_cone_deg', self.YAW_CONE_DEG)))
        self.bearing_rejected = 0
        self.bearing_rejected_at = 0.0

        # The approach is flown by the inherited carrot, whose speed is
        # MOVE_SPEED. Setting it here rather than threading a second speed
        # through _step_xy_ramp keeps that ramp -- and its leash -- the only
        # thing that ever moves the horizontal setpoint.
        self.MOVE_SPEED = self.APPROACH_SPEED

        # Camera mounting: the pose of the camera in the body frame, ROS
        # convention (x fwd, y LEFT, z UP), measured to the D435i's LEFT imager
        # it. Give both nodes the same numbers.
        cam_x = float(self._declare_number('cam_x', 0.0))
        cam_y = float(self._declare_number('cam_y', 0.0))
        cam_z = float(self._declare_number('cam_z', 0.0))
        cam_roll = float(self._declare_number('cam_roll', 0.0))
        cam_pitch = float(self._declare_number('cam_pitch', 0.0))
        cam_yaw = float(self._declare_number('cam_yaw', 0.0))
        self.r_cam = rpy_to_matrix_frd(cam_roll, cam_pitch, cam_yaw)
        self.t_cam = np.array([cam_x, -cam_y, -cam_z])   # ROS FLU -> body FRD

        # How far the airframe sticks out below and above the point PX4 flies.
        # commanded_altitude positions the VEHICLE ORIGIN, the camera sits
        # cam_z above it (FLU, so a negative cam_z is a camera below the
        # origin), and the landing gear hangs GEAR_BELOW_CAMERA under the
        # camera. Everything vertical downstream is expressed against these
        # two numbers rather than against the origin, because the origin is
        # not the part that hits the sill.
        self.body_below = self.GEAR_BELOW_CAMERA - cam_z
        self.body_above = self.DRONE_HEIGHT - self.body_below
        self.get_logger().info(
            f"Airframe: {self.DRONE_HEIGHT:.3f} m tall, {self.DRONE_WIDTH:.3f} m "
            f"wide, extending {self.body_below:.3f} m below and "
            f"{self.body_above:.3f} m above the commanded point. A window must "
            f"measure at least "
            f"{self.DRONE_HEIGHT + 2 * self.HARD_CLEARANCE:.2f} x "
            f"{self.DRONE_WIDTH + 2 * self.HARD_CLEARANCE:.2f} m to be flown.")

        self.estimator = WindowEstimator(
            depth_min=float(self._declare_number('depth_min', self.DEPTH_MIN)),
            depth_max=float(self._declare_number('depth_max', self.DEPTH_MAX)),
            corner_spread=float(self._declare_number('corner_spread', self.CORNER_SPREAD)),
            corner_spread_frac=float(self._declare_number(
                'corner_spread_frac', self.CORNER_SPREAD_FRAC)),
            plane_tolerance=float(self._declare_number(
                'plane_tolerance', self.PLANE_TOLERANCE)),
            min_size=float(self._declare_number('window_min_size', self.WINDOW_MIN_SIZE)),
            max_size=float(self._declare_number('window_max_size', self.WINDOW_MAX_SIZE)),
            side_mismatch=float(self._declare_number('side_mismatch', self.SIDE_MISMATCH)),
            max_tilt=math.radians(float(self._declare_number(
                'max_tilt_deg', self.MAX_TILT_DEG))),
            buffer_seconds=float(self._declare_number('buffer_seconds', self.BUFFER_SECONDS)),
            buffer_max=self.BUFFER_MAX,
            min_samples=self.MIN_SAMPLES,
            gate_metres=float(self._declare_number('gate_metres', self.GATE_METRES)),
            gate_yaw=math.radians(float(self._declare_number(
                'gate_yaw_deg', self.GATE_YAW_DEG))),
            gate_reset_count=int(self._declare_number(
                'gate_reset_count', self.GATE_RESET_COUNT)),
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # The attitude is a separate topic from the local position; the local
        # position message carries only the heading, and the heading alone is
        # not enough to place a point that is 3 m in front of a pitching
        # vehicle. See the header.
        self.attitude = None
        self.attitude_time = None
        # PX4 publishes this at the EKF output rate -- 100-250 Hz depending on
        # the board, an order of magnitude faster than every other topic this
        # node takes put together, and far faster than anything here can use.
        # The callback decimates to ATTITUDE_MAX_HZ; see attitude_callback.
        self.attitude_min_interval = 1.0 / self.ATTITUDE_MAX_HZ
        self.attitude_last_kept = 0.0
        self.create_subscription(VehicleAttitude, '/uav_2/fmu/out/vehicle_attitude',
                                 self.attitude_callback, qos_profile=sensor_qos,
                                 callback_group=self.sensor_cbg)

        geometry_topic = str(self.declare_parameter(
            'geometry_topic', self.GEOMETRY_TOPIC).value)
        sub = self.create_subscription(Float32MultiArray, geometry_topic,
                                       self.geometry_callback, 10,
                                       callback_group=self.sensor_cbg)
        # Resolved, so count_publishers() below asks about the same name the
        # subscription is actually bound to and not the relative one.
        self.geometry_topic = sub.topic_name

        # What the estimator currently believes, for anyone watching from the
        # ground: x|y|z|yaw_deg|width|height|samples|age.
        self.window_pose_pub = self.create_publisher(String, 'window_pose', 10)

        self.geometry_seen = 0
        self.pose_ok_since = None       # monotonic time the estimate went usable
        self.pose_lost_since = None     # ... and when it stopped being usable

        # The committed traverse, frozen at the start of TRAVERSE.
        self.traverse_entry = None      # np(3) NED, the standoff point
        self.traverse_exit = None       # np(3) NED, beyond the window
        self.traverse_heading = None    # rad, NED
        self.traverse_standoff = None   # m, the EFFECTIVE standoff frozen at
                                        # commit. Must be the same number the
                                        # entry point was built from, or the
                                        # run-through measures itself against
                                        # a line it is not flying -- see
                                        # effective_standoff().
        self.traverse_window = None     # the estimate it was committed from
        self.blind_traverse_since = None
        self.blind_traverse_limit = self.BLIND_TRAVERSE_SECONDS
        self.blind_traverse_left = 0.0

        self.align_in_band_since = None
        self.outcome = 'not attempted'

        # The newest raw detection, truncated or not. Kept separately from the
        # estimator because the two want opposite things from it: the estimator
        # must never see a truncated sample, and RECENTRE has nothing else to
        # steer by.
        self.last_detection = None      # dict, see geometry_callback
        self.truncated_frames = 0
        self.recentre_untruncated_since = None
        self.recentre_ref_heading = 0.0
        self.recentre_backoffs = 0
        self.recentre_backoff_target = None
        self.recentre_attempt_since = None
        self.last_good_est = None
        self.last_good_est_time = 0.0

        self.get_logger().warning(
            f"Window traversal on OPTICAL FLOW: climb {self.TAKEOFF_ALTITUDE:.2f} m, "
            f"hold {self.HOLD_SECONDS:.0f} s, "
            + ("wait on the takeoff heading for the window (no sweep), lock, "
               if self.SCAN_SPAN <= 0.0 else
               f"sweep +/-{math.degrees(self.SCAN_SPAN) / 2:.0f} deg for the "
               "window, lock, ") +
            f"line up {self.STANDOFF_DISTANCE:.2f} m in front of it and fly "
            f"through to {self.EXIT_DISTANCE:.2f} m beyond, then land. "
            f"Approach {self.APPROACH_SPEED:.2f} m/s, traverse "
            f"{self.TRAVERSE_SPEED:.2f} m/s. Hard limit "
            f"{self.FLIGHT_SECONDS:.0f} s from the start of the climb. "
            "Press q to abort into a descent, k to force-disarm.")

    # ------------------------------------------------------------------ subs

    def attitude_callback(self, msg):
        """Keep the newest attitude, at no more than ATTITUDE_MAX_HZ.

        Dropping here rather than subscribing at a lower rate is the only
        option -- the publication rate is PX4's to choose and there is no way
        to ask it for less over uXRCE-DDS. The message is already deserialised
        by the time we are called, so this saves the callback body rather than
        the transport, but the body is what was costing the setpoint timer its
        thread. The kept sample is always the freshest one, because it is
        whichever message happens to arrive after the interval expires.
        """
        now = time.monotonic()
        if now - self.attitude_last_kept < self.attitude_min_interval:
            return
        self.attitude_last_kept = now
        self.attitude = msg
        self.attitude_time = now

    def geometry_callback(self, msg):
        """One frame's worth of window geometry, paired with where we are.

        The pairing is by ARRIVAL, not by timestamp: Float32MultiArray has no
        header to carry one. The lag between the frame being grabbed and this
        callback is a frame time plus transport, call it 60-100 ms, which at
        the 0.3-0.45 m/s this node flies is 2-5 cm of position error on the
        sample. That is well inside the tolerances everything downstream is
        built on, and it is systematic rather than random, so the median does
        not remove it -- worth knowing before you chase the last 5 cm of
        alignment accuracy.
        """
        self.geometry_seen += 1

        data = np.asarray(msg.data, dtype=float)
        if data.size < 15:
            self.get_logger().warning(
                f"/window_geometry has {data.size} values, expected at least 15. "
                "Is window_detect up to date?", throttle_duration_sec=5.0)
            return
        geometry = data[:15].reshape(5, 3)

        # Row 5, when present, is (truncated, border_margin_px, 0) -- see
        # window_detect.publish_geometry. An older detector that does not send
        # it reads as "not truncated", which is the pre-existing behaviour, so
        # a version mismatch degrades to what this node did before rather than
        # refusing every sample.
        if data.size >= 18:
            truncated = bool(data[15] > 0.5)
            margin_px = float(data[16])
        else:
            truncated = False
            margin_px = float('nan')
            self.get_logger().warning(
                "/window_geometry carries no truncation row: this window_detect "
                "cannot tell a window from part of a window. Update it.",
                throttle_duration_sec=10.0)

        # Kept whatever the verdict: RECENTRE steers on the truncated ones.
        self.last_detection = {
            'time': time.monotonic(),
            'truncated': truncated,
            'margin_px': margin_px,
            # The centre row's bearing off the optical axis, positive right.
            'centre_az': math.radians(float(geometry[4][1])),
            'centre_el': math.radians(float(geometry[4][2])),
        }

        if truncated:
            # Refused outright rather than gated. Every other filter in the
            # estimator asks "is this measurement consistent?", and a truncated
            # quad is perfectly consistent -- it is a real, planar, correctly
            # sized rectangle. It is just not the window. Nothing downstream
            # can recover the missing half, so the only safe thing to do with
            # the sample is not have it.
            self.truncated_frames += 1
            return

        lp = self.local_position
        if lp is None or not lp.xy_valid or not lp.z_valid:
            return
        if self.attitude is None:
            self.get_logger().warning(
                "No /fmu/out/vehicle_attitude yet; cannot place the window.",
                throttle_duration_sec=5.0)
            return

        p_ned = np.array([lp.x, lp.y, lp.z])
        self.estimator.add(geometry, np.asarray(self.attitude.q, dtype=float),
                           p_ned, self.r_cam, self.t_cam, time.monotonic())

    # -------------------------------------------------------------- estimate

    def fresh_detection(self):
        """The newest raw detection if it is recent enough to act on.

        "Recent" is the same POSE_MAX_AGE the estimate is held to. This is a
        single frame and is deliberately NOT filtered: it is used to answer
        "is the camera looking at part of a window right now, and which way",
        which is a question about this instant.
        """
        det = self.last_detection
        if det is None or time.monotonic() - det['time'] > self.POSE_MAX_AGE:
            return None
        return det

    def window_estimate(self):
        """The estimate, or None if it is missing, thin, stale or off-cone."""
        est = self.estimator.estimate(time.monotonic())
        if est is None or est['age'] > self.POSE_MAX_AGE:
            return None
        if self.YAW_CONE > 0.0 and self.current_stage != self.TRAVERSE:
            # Not flown at. A window whose bearing is outside the cone is not
            # the window this flight was pointed at before it armed, and the
            # cheapest way to not fly at it is to not believe in it. Skipped
            # during TRAVERSE only because the target there is already frozen
            # and the aircraft is committed.
            bearing = self._bearing_to(est['centre'])
            if abs(bearing) > self.YAW_CONE:
                self.bearing_rejected += 1
                self.bearing_rejected_at = time.monotonic()
                return None
        return est

    def _track_pose_health(self):
        """Bookkeeping for how long the estimate has been usable, or not."""
        now = time.monotonic()
        est = self.window_estimate()
        if est is not None:
            # Kept so a converged approach can still be finished -- and still
            # clearance-checked -- when the detector drops out at the worst
            # possible moment, which on a thin window frame is most moments.
            self.last_good_est = est
            self.last_good_est_time = now
            self.pose_lost_since = None
            if self.pose_ok_since is None:
                self.pose_ok_since = now
        else:
            self.pose_ok_since = None
            if self.pose_lost_since is None:
                self.pose_lost_since = now

    def pose_summary(self):
        est = self.window_estimate()
        if est is None:
            fresh = self.estimator.fresh_count(time.monotonic())
            if self.geometry_seen == 0:
                # Two very different faults look the same from here: nobody is
                # publishing the topic at all (wrong node, wrong remap, old
                # window_detect), or window_detect is up and simply is not
                # seeing a window. Only the first is worth restarting for.
                if self.count_publishers(self.geometry_topic) == 0:
                    return (f"nothing is publishing {self.geometry_topic} -- "
                            "is window_detect running, and new enough to have "
                            "publish_geometry?")
                return (f"{self.geometry_topic} has a publisher but has never "
                        "carried a message: window_detect is not detecting the "
                        "window. Check the colour, min_area and the lighting.")
            det = self.fresh_detection()
            if det is not None and det['truncated']:
                # Worth saying explicitly. "No usable pose" while the camera is
                # plainly looking at a window reads as a detector fault, and
                # this is the one case where it is not one.
                return (f"window in sight but TRUNCATED "
                        f"({det['margin_px']:.0f} px to the frame edge, "
                        f"{self.truncated_frames} such frames) -- not measurable "
                        "until the whole aperture is in view")
            if time.monotonic() - self.bearing_rejected_at < self.POSE_MAX_AGE:
                return (f"a window pose exists but it is outside the "
                        f"{math.degrees(self.YAW_CONE):.0f} deg yaw cone about "
                        f"the takeoff heading ({self.bearing_rejected} frames) "
                        "-- not the window this flight was aimed at, so it is "
                        "not being flown at. Point the aircraft at the window "
                        "before arming, or raise yaw_cone_deg")
            return (f"no usable pose ({fresh}/{self.MIN_SAMPLES} fresh samples, "
                    f"{self.estimator.accepted_total} accepted ever, "
                    f"{self.truncated_frames} truncated; "
                    f"rejections: {self.estimator.rejection_summary()})")
        return (f"window at ({est['centre'][0]:+.2f}, {est['centre'][1]:+.2f}, "
                f"{est['centre'][2]:+.2f}) NED, facing "
                f"{math.degrees(math.atan2(est['normal'][1], est['normal'][0])):+.0f} deg, "
                f"{est['width']:.2f}x{est['height']:.2f} m, {est['samples']} samples, "
                f"{est['age'] * 1000:.0f} ms old")

    def publish_window_pose(self):
        est = self.window_estimate()
        msg = String()
        if est is None:
            msg.data = ''
        else:
            msg.data = "|".join([
                f"{est['centre'][0]:.3f}", f"{est['centre'][1]:.3f}",
                f"{est['centre'][2]:.3f}",
                f"{math.degrees(math.atan2(est['normal'][1], est['normal'][0])):.1f}",
                f"{est['width']:.3f}", f"{est['height']:.3f}",
                f"{est['samples']}", f"{est['age']:.3f}",
            ])
        self.window_pose_pub.publish(msg)

    # --------------------------------------------------------- the geometry

    def approach_points(self, est):
        """(entry, exit, heading) for an estimate, all in NED.

        entry is standoff_distance in front of the window ON ITS AXIS -- along
        the normal, which is what makes the approach square to the aperture
        rather than merely near it. exit is exit_distance past the window on
        the same line. heading points from entry to exit, i.e. at the window
        and then through it.

        The altitude of both is the window centre's, clamped into the flight
        envelope by the caller before it becomes a setpoint.

        The standoff is the EFFECTIVE one, not the parameter -- see
        effective_standoff(). On the narrow-FOV D435i a big aperture has to be
        approached from further back or it stops fitting in the frame exactly
        when the approach needs to see it.
        """
        centre = est['centre']
        normal = est['normal']
        entry = centre + normal * self.effective_standoff(est)
        exit_point = centre - normal * self.EXIT_DISTANCE
        heading = math.atan2(-normal[1], -normal[0])
        return entry, exit_point, heading

    def effective_standoff(self, est):
        """standoff_distance, pushed back if the window would not fit in frame.

        ADDED FOR THE REALSENSE D435i. The ZED saw ~90x60 deg and the standoff
        that made the run through the window short was always comfortably
        further out than the distance at which the aperture still fitted in
        the picture, so the two never competed and a fixed number was fine.

        The D435i's colour sensor is 70x43 deg. At 43 deg vertical the "still
        fits" distance is 1.26 * the window height, which for the apertures
        this mission actually flies is the SAME order as the standoff -- so it
        can bind, and when it binds the failure is nasty: the window touches
        the frame edge, window_detect flags the corners TRUNCATED, the corner
        depths there are the least reliable it produces, and the estimate the
        approach is being steered by degrades at the exact moment the aircraft
        is committing to it.

        So the standoff is the larger of what the operator asked for and what
        the optics require, with MIN_STANDOFF_MARGIN of headroom, capped at
        MAX_STANDOFF_DISTANCE so a nonsense aperture cannot walk the approach
        point out of the arena. Both axes are checked, though in practice the
        vertical one always wins on this camera.

        Returns the parameter unchanged when there is no size estimate yet --
        this is a refinement of the approach point, never a gate on having one.
        """
        width = float(est.get('width') or 0.0)
        height = float(est.get('height') or 0.0)
        if width <= 0.0 and height <= 0.0:
            return self.STANDOFF_DISTANCE

        needed = 0.0
        if width > 0.0:
            needed = max(needed, (width / 2.0) / self._tan_half_hfov)
        if height > 0.0:
            needed = max(needed, (height / 2.0) / self._tan_half_vfov)
        needed *= self.MIN_STANDOFF_MARGIN

        standoff = min(max(self.STANDOFF_DISTANCE, needed),
                       self.MAX_STANDOFF_DISTANCE)

        # Log only when the clamp actually moved the approach point, and only
        # when the answer changes, so the reason a flight stood off further
        # than it was told to is in the console without spamming it at 10 Hz.
        if standoff > self.STANDOFF_DISTANCE + 0.01:
            if abs(standoff - self._last_standoff_logged) > 0.05:
                self._last_standoff_logged = standoff
                self.get_logger().info(
                    f"standoff pushed out to {standoff:.2f} m (asked "
                    f"{self.STANDOFF_DISTANCE:.2f} m): a "
                    f"{width:.2f}x{height:.2f} m aperture does not fit in a "
                    f"{self.CAMERA_HFOV_DEG:.0f}x{self.CAMERA_VFOV_DEG:.0f} deg "
                    f"frame any closer than that with "
                    f"{self.MIN_STANDOFF_MARGIN:.2f}x margin.")
            if standoff >= self.MAX_STANDOFF_DISTANCE - 0.01:
                self.get_logger().warning(
                    f"standoff hit the {self.MAX_STANDOFF_DISTANCE:.1f} m cap "
                    f"for a {width:.2f}x{height:.2f} m aperture. Either the "
                    "size estimate is wrong or this window is too big for "
                    "this lens -- check /window_pose against a tape measure.",
                    throttle_duration_sec=10.0)
        return standoff

    def window_altitude(self, est):
        """Height above the arming point to fly the traverse at, clamped.

        NOT the window centre. The aircraft is 260 mm tall and hangs mostly
        BELOW the camera that measured the window, so putting the commanded
        point on the window centre puts the landing gear ~160 mm lower than
        the centre -- which is what walked the gear into the sill and tipped
        the aircraft over on the first attempt.

        What is solved for here is a commanded height at which the whole
        airframe fits inside the aperture:

            sill + body_below + clearance  <=  z  <=  lintel - body_above - clearance

        Within that band the preferred answer centres the AIRFRAME (not the
        origin) on the window -- which by itself already biases the command
        upward by (body_below - body_above)/2 -- and then adds SILL_BIAS on
        top, clipped by the upper bound, because the two ways of getting this
        wrong are not equally expensive. When the aperture is too tight for
        the full clearance the band inverts, and the LOWER bound wins:
        clipping the lintel with a prop guard is survivable, catching the gear
        on the sill is what flips the aircraft.

        Returns None before home_z exists, which cannot happen from any stage
        that calls it -- home is captured at arming -- but the flight envelope
        clamp is real: a window estimate that has gone wrong vertically must
        not be able to command a climb past max_altitude or a descent into the
        floor.
        """
        if self.home_z is None:
            return None

        # Height of the window centre above the arming point, and the sill and
        # lintel either side of it.
        centre = self.home_z - est['centre'][2]
        half = 0.5 * float(est.get('height') or 0.0)
        sill = centre - half
        lintel = centre + half

        lower = sill + self.body_below + self.VERTICAL_CLEARANCE
        upper = lintel - self.body_above - self.VERTICAL_CLEARANCE

        # Airframe centred in the aperture: the commanded point sits
        # (body_below - body_above)/2 above the window centre.
        wanted = centre + 0.5 * (self.body_below - self.body_above) + self.SILL_BIAS

        if math.isfinite(self.TRAVERSE_CENTRE_OFFSET):
            # Explicit rule: the window centre plus a fixed offset (part 2
            # aims 0.10 m above centre), kept inside the hard-clearance band
            # so the gear still clears the sill and the top the lintel.
            wanted = centre + self.TRAVERSE_CENTRE_OFFSET
            lo = sill + self.body_below + self.HARD_CLEARANCE
            hi = lintel - self.body_above - self.HARD_CLEARANCE
            if hi >= lo:
                wanted = min(max(wanted, lo), hi)
            lower, upper = wanted, wanted

        if lower > upper:
            # Not enough room for the full clearance either side. Take the
            # lower bound -- gear clear of the sill first -- but never command
            # a height whose gear is below the sill at all.
            lower_hard = sill + self.body_below + self.HARD_CLEARANCE
            upper_hard = lintel - self.body_above - self.HARD_CLEARANCE
            wanted = max(lower, lower_hard)
            if upper_hard >= lower_hard:
                # There is still a hard-clearance band, just not a comfortable
                # one. Stay inside it: favouring the sill must not be allowed
                # to push the airframe out through the top of the aperture.
                wanted = min(wanted, upper_hard)
            self.get_logger().warning(
                f"Window is {2 * half:.2f} m tall: too tight for "
                f"{self.VERTICAL_CLEARANCE:.2f} m clearance around a "
                f"{self.DRONE_HEIGHT:.2f} m airframe. Favouring the sill and "
                f"flying at {wanted:.2f} m.",
                throttle_duration_sec=5.0)
        else:
            wanted = min(max(wanted, lower), upper)

        wanted += self.ALTITUDE_OFFSET

        clamped = min(max(wanted, self.MIN_ALTITUDE), self.MAX_ALTITUDE)
        if abs(clamped - wanted) > 1e-3:
            self.get_logger().warning(
                f"The traverse wants {wanted:.2f} m above the arming point "
                f"(window centre {centre:.2f} m, sill {sill:.2f} m), outside "
                f"the {self.MIN_ALTITUDE:.2f}-{self.MAX_ALTITUDE:.2f} m "
                f"envelope. Flying the traverse at {clamped:.2f} m instead -- "
                "check the gear clears the sill before trusting this.",
                throttle_duration_sec=5.0)
        return clamped

    def swept_width(self, yaw_error=0.0):
        """How wide the airframe actually is when it is not square to the frame.

        A square of side W, yawed by e relative to the aperture, presents
        W(|cos e| + |sin e|) across it -- the diagonal at 45 degrees, which for
        a 260 mm airframe is 368 mm, 108 mm more than the number the clearance
        arithmetic used to be done with. Eight degrees of yaw, which is the
        alignment tolerance, is already 34 mm of it, and 34 mm is a fifth of
        the entire per-side margin on a 0.6 m window.
        """
        return self.DRONE_WIDTH * (abs(math.cos(yaw_error)) + abs(math.sin(yaw_error)))

    def cross_track_of(self, est):
        """Signed distance from the window's centreline, metres, positive right.

        The window axis is the line through the window centre along its normal;
        this is how far off it the aircraft is right now. Distinct from the
        ALIGN cross-track error, which is measured against the STANDOFF POINT
        the estimate currently implies -- the same thing when the estimate is
        steady, and not the same thing at the instant the estimate has moved.
        This is the one that has to be right at the commit, because it is the
        offset the aircraft will carry through the aperture.

        None if there is no position to measure.
        """
        lp = self.local_position
        if lp is None:
            return None
        normal = est['normal']
        offset = np.array([lp.x - est['centre'][0], lp.y - est['centre'][1]])
        # Horizontal perpendicular to the (horizontal component of the) normal.
        perp = np.array([-normal[1], normal[0]])
        norm = float(np.linalg.norm(perp))
        if norm < 1e-6:
            return None
        return float(np.dot(offset, perp / norm))

    def aperture_margins(self, est, cross=0.0, yaw_error=0.0):
        """(vertical, lateral) metres of spare aperture around the airframe.

        Vertical is per side at the height window_altitude would command.

        Lateral used to assume the aircraft was exactly on the window axis and
        exactly square to it. It is neither. Both departures are charged here:
        `cross` is how far off the centreline the aircraft actually is, which
        comes off one side of the margin entirely rather than being shared,
        and `yaw_error` widens the airframe itself via swept_width. With the
        defaults it reduces to the old geometric answer, which is what the
        pre-commit reporting wants; the commit passes the measured values.

        Both can be negative, which means the airframe does not fit.
        """
        height = float(est.get('height') or 0.0)
        width = float(est.get('width') or 0.0)
        alt = self.window_altitude(est)
        if alt is None:
            vertical = 0.5 * (height - self.DRONE_HEIGHT)
        else:
            centre = self.home_z - est['centre'][2]
            sill = centre - 0.5 * height
            lintel = centre + 0.5 * height
            vertical = min(alt - self.body_below - sill,
                           lintel - (alt + self.body_above))
        lateral = 0.5 * (width - self.swept_width(yaw_error)) - abs(cross)
        return vertical, lateral

    def _set_target(self, x, y, altitude=None):
        """Point the inherited ramps at an NED point and an altitude.

        This is the whole of "how a setpoint is given" in this node: the x/y
        carrot target and the z ramp target. Everything about how they are
        walked -- the speed, the leash to the measured position, the fact that
        what is published is an absolute NED point -- is the base class's, and
        is not re-implemented here.
        """
        self.move_target_x = float(x)
        self.move_target_y = float(y)
        self.moving = True
        if altitude is not None:
            self.commanded_altitude = float(altitude)
            self.target_z = self.home_z - float(altitude)

    def _aim_yaw_at(self, heading):
        """Walk the commanded yaw towards an absolute NED heading.

        Rewritten every tick rather than latched once, so a target that moves
        as the estimate refines is followed instead of being flown to once and
        forgotten. yaw_remaining is what the inherited ramp consumes, and the
        ramp is still what limits the rate and holds the leash.
        """
        self.yaw_remaining = wrap_pi(self._clamp_to_cone(heading) - self.yaw_setpoint)

    def _clamp_to_cone(self, heading):
        """Pull an absolute heading back inside the yaw cone.

        Every yaw this node commands goes through here, so whatever else goes
        wrong upstream -- a truncated-window centroid that never clears, a
        pose built on a bad attitude -- the nose cannot end up more than
        yaw_cone_deg from the heading the aircraft took off on.
        """
        if self.YAW_CONE <= 0.0 or self.home_z is None:
            return heading
        off = wrap_pi(heading - self.home_yaw)
        if abs(off) <= self.YAW_CONE:
            return heading
        clamped = wrap_pi(self.home_yaw + math.copysign(self.YAW_CONE, off))
        self.get_logger().warning(
            f"Yaw cone: {math.degrees(heading):+.0f} deg is "
            f"{math.degrees(abs(off)):.0f} deg off the takeoff heading; "
            f"commanding {math.degrees(clamped):+.0f} deg instead. Nothing "
            "here turns further than "
            f"{math.degrees(self.YAW_CONE):.0f} deg.",
            throttle_duration_sec=2.0)
        return clamped

    def _bearing_to(self, point):
        """Bearing from the vehicle to a NED point, off the takeoff heading."""
        lp = self.local_position
        if lp is None:
            return 0.0
        return wrap_pi(math.atan2(point[1] - lp.y, point[0] - lp.x) - self.home_yaw)

    def _heading_error(self, heading):
        lp = self.local_position
        if lp is None:
            return math.pi
        return abs(wrap_pi(heading - lp.heading))

    # ---------------------------------------------------------- EKF2 resets

    def _on_heading_reset(self, delta):
        """Turn everything this node placed in NED with the frame.

        The base class has already shifted yaw_setpoint and home_yaw, and
        WindowScan its scan/lock headings. What is left here is geometry: the
        window pose, the frozen traverse line and the move target were all
        derived from an attitude EKF2 has just corrected by `delta`, so
        relative to the aircraft they are now rotated by exactly that much.
        Rotating them about the vehicle puts them back where the camera
        actually saw them, and the approach carries on instead of jumping
        sideways onto a window that appears to have swung round the room.
        """
        super()._on_heading_reset(delta)
        lp = self.local_position
        if lp is None:
            # No pivot to rotate about. The estimate is the only thing that
            # matters here and it is cheaper to rebuild it than to guess.
            self.estimator.samples.clear()
            return

        pivot = (lp.x, lp.y)
        self.estimator.rotate_frame(delta, pivot)

        if self.recentre_ref_heading is not None:
            self.recentre_ref_heading = wrap_pi(self.recentre_ref_heading + delta)
        if self.traverse_heading is not None:
            self.traverse_heading = wrap_pi(self.traverse_heading + delta)

        c, sn = math.cos(delta), math.sin(delta)

        def turn(x, y):
            dx, dy = x - pivot[0], y - pivot[1]
            return (pivot[0] + c * dx - sn * dy, pivot[1] + sn * dx + c * dy)

        for name in ('traverse_entry', 'traverse_exit', 'recentre_backoff_target'):
            point = getattr(self, name, None)
            if point is not None:
                point[0], point[1] = turn(point[0], point[1])
        if self.move_target_x is not None:
            self.move_target_x, self.move_target_y = turn(
                self.move_target_x, self.move_target_y)
            self.move_start_x, self.move_start_y = turn(
                self.move_start_x, self.move_start_y)

    # ---------------------------------------------------------- state machine

    def _clock_stages(self):
        """The inherited hard clock, extended to the new stages.

        TRAVERSE is deliberately NOT on the list. Once committed, the aircraft
        is somewhere between a metre and a half in front of an aperture and a
        metre and a half past it, and starting a descent from inside a window
        frame because a stopwatch ran out is not a safety behaviour. The
        traverse has its own, much shorter, timeout; the clock catches the
        aircraft again in CLEAR immediately afterwards.
        """
        return super()._clock_stages() + (self.RECENTRE, self.AIM, self.ALIGN,
                                          self.CLEAR)

    def timer_callback(self):
        self.publish_window_pose()

        if self.current_stage not in self.TRAVERSE_STAGES:
            # SCAN and LOCK are WindowScan's; everything else is the base
            # class's. Both are reached through the same call.
            super().timer_callback()
            return

        if self._check_flight_clock():
            return

        if self.stream_setpoints:
            self.publish_offboard_control_mode()
            self.publish_position_setpoint()
        self.publish_status()

        if self.kill_requested:
            self._enter_stage(self.KILLING)
            return
        if self.abort_requested:
            self.abort_requested = False
            self._begin_landing("operator abort")
            return

        self._track_pose_health()

        {
            self.RECENTRE: self._handle_recentre,
            self.AIM: self._handle_aim,
            self.ALIGN: self._handle_align,
            self.TRAVERSE: self._handle_traverse,
            self.CLEAR: self._handle_clear,
        }[self.current_stage]()

    # ------------------------------------------------------------- the lock

    def _handle_lock(self):
        """Hold facing the window until the pose estimate is good enough.

        WindowScan's version of this stage parks and waits for the flight
        clock. Here it is the sampling stage: the aircraft is stationary and
        pointed at the window, which is the best possible geometry for stereo
        depth on a thin frame, and it stays there until min_samples of the
        estimator's accepted samples exist.
        """
        super()._handle_lock()
        if self.current_stage != self.LOCK:
            # The parent went back to scanning, or started a landing.
            return

        self._track_pose_health()

        est = self.window_estimate()
        if est is None:
            det = self.fresh_detection()
            if det is not None and det['truncated']:
                # The sweep stopped on a window that is hanging off the edge of
                # the frame. There is no pose and there is not going to be one
                # from here, because every sample is being refused. Go and look
                # at the whole thing first.
                self._begin_recentre(det)
                return
            self.get_logger().info(
                f"Locked, building the window pose: {self.pose_summary()}",
                throttle_duration_sec=1.0)
            return

        if not self.hold_xy:
            self.get_logger().warning(
                "Window pose is ready but the vision estimate is not healthy "
                "enough to fly to a point. Waiting.", throttle_duration_sec=2.0)
            return

        self._begin_aim(est)

    # ------------------------------------------------------------ RECENTRE

    def _begin_recentre(self, det):
        """Stop, and turn to look at the whole window before measuring it."""
        self._enter_stage(self.RECENTRE)
        self.moving = False
        self.recentre_untruncated_since = None
        self.recentre_backoff_target = None
        self.recentre_attempt_since = time.monotonic()
        lp = self.local_position
        self.recentre_ref_heading = self.yaw_setpoint if lp is None else lp.heading
        self.get_logger().warning(
            f"RECENTRE: the window is truncated by the frame edge "
            f"({det['margin_px']:.0f} px), so the quad being measured is only "
            f"part of it and its centre is not the window's centre. Nudging "
            f"{'clockwise' if det['centre_az'] > 0 else 'anticlockwise'} in "
            f"{math.degrees(self.RECENTRE_YAW_STEP):.0f} deg steps (up to "
            f"{math.degrees(self.RECENTRE_YAW_LIMIT):.0f} deg) to bring the "
            "rest into view before anything is flown at it.")

    def _handle_recentre(self):
        """Yaw at the visible part of the window until the whole one is in frame.

        The steering signal is the bearing of the VISIBLE centroid off the
        optical axis. That centroid is biased towards the middle of the image
        relative to the true window centre -- exactly by the clipping -- so
        turning towards it always turns towards the clipped side. It does not
        need to be accurate, only correctly signed, and it is: whichever edge
        the window is falling off, the centroid sits on the other side of the
        image centre from it.

        Three ways out. The window comes fully into view and AIM takes over;
        the aperture is simply too big for the field of view from here, so the
        aircraft backs away and tries again; or neither works and the attempt
        is abandoned rather than flown at a window whose extent was never
        measured.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        # A back-off in progress owns the stage until it arrives: yawing while
        # translating is the one thing the whole approach is written to avoid.
        if self.recentre_backoff_target is not None:
            if self._arrived_at_backoff():
                self.recentre_backoff_target = None
                self.recentre_attempt_since = time.monotonic()
                lp = self.local_position
                if lp is not None:
                    self.recentre_ref_heading = lp.heading
                # Both clocks are per ATTEMPT, not per stage: a back-off that
                # used half the stage timeout getting there must not leave the
                # look that follows it no time to succeed.
                self._restart_stage_clock()
                self.get_logger().info("RECENTRE: backed off; looking again.")
            else:
                self.get_logger().info(
                    "RECENTRE: backing away from the window to get it all in "
                    "frame.", throttle_duration_sec=1.0)
                return

        # Checked before the branches below, so it catches every way of being
        # stuck here -- still truncated, or fully in frame but never producing
        # enough accepted samples to build a pose from.
        if self._in_stage_for() > self.RECENTRE_TIMEOUT:
            self._abandon(
                f"never got a measurable view of the whole window in "
                f"{self.RECENTRE_TIMEOUT:.0f} s of re-centring")
            return

        det = self.fresh_detection()
        if det is None:
            # The window has gone entirely. Hold, and let the shared pose-lost
            # timeout decide when that has gone on too long.
            if self._give_up_on_pose('RECENTRE'):
                return
            self.get_logger().info(
                f"RECENTRE: lost sight of the window. {self.pose_summary()}",
                throttle_duration_sec=1.0)
            return

        if not det['truncated']:
            now = time.monotonic()
            if self.recentre_untruncated_since is None:
                self.recentre_untruncated_since = now
            elif now - self.recentre_untruncated_since >= self.RECENTRE_CLEAR_SECONDS:
                est = self.window_estimate()
                if est is not None and self.hold_xy:
                    self.get_logger().warning(
                        "RECENTRE: the whole window is in frame and measured. "
                        f"{self.pose_summary()}.")
                    self._begin_aim(est)
                    return
                # In view but not yet enough accepted samples for a pose. That
                # is the estimator filling its buffer; wait for it here rather
                # than in AIM, where the aircraft would already be turning.
                self.get_logger().info(
                    f"RECENTRE: window fully in frame, building the pose. "
                    f"{self.pose_summary()}", throttle_duration_sec=1.0)
            return

        # Still truncated: keep nudging towards what we can see. One small
        # step at a time, off the CURRENT measured heading, so the next frame
        # is a fresh answer to "is it still clipped?" rather than the tail of a
        # turn commanded several degrees ago.
        self.recentre_untruncated_since = None
        lp = self.local_position
        az = det['centre_az']
        step = 0.0
        if lp is not None and abs(az) > self.RECENTRE_YAW_DEADBAND:
            step = math.copysign(min(abs(az), self.RECENTRE_YAW_STEP), az)
            # Leash the whole correction to RECENTRE_YAW_LIMIT either side of
            # where this attempt started, so a bad centroid cannot walk the
            # aircraft round in a circle chasing an edge that never clears.
            turned = wrap_pi(lp.heading - self.recentre_ref_heading)
            # How much of the budget this DIRECTION has already spent. It is
            # the signed turn projected onto the direction of the step, not
            # `copysign(turned, step)` -- that spelling discards the sign of
            # `turned` and reads every turn as if it had gone the way this
            # step is going, so one direction could never spend the budget at
            # all and the aircraft could walk right round chasing an edge.
            spent = turned if step > 0.0 else -turned
            allowed = self.RECENTRE_YAW_LIMIT - spent
            if allowed <= 0.0:
                self.get_logger().warning(
                    f"RECENTRE: turned the full "
                    f"{math.degrees(self.RECENTRE_YAW_LIMIT):.0f} deg and the "
                    "window is still clipped -- it does not fit in the frame "
                    "from here.", throttle_duration_sec=2.0)
                self._begin_backoff()
                return
            step = math.copysign(min(abs(step), allowed), step)
            self._aim_yaw_at(wrap_pi(lp.heading + step))

        attempt = time.monotonic() - (self.recentre_attempt_since or time.monotonic())
        if attempt > self.RECENTRE_BACKOFF_SECONDS:
            self._begin_backoff()
            return

        self.get_logger().info(
            f"RECENTRE: truncated ({det['margin_px']:.0f} px to the edge, "
            f"centroid {math.degrees(az):+.0f} deg), nudging "
            f"{math.degrees(step):+.1f} deg.",
            throttle_duration_sec=1.0)

    def _begin_backoff(self):
        """Give up on turning and move away from the window instead.

        Yawing only helps while the window fits in the field of view at all.
        Once it does not -- the aircraft has ended up close to a wide aperture
        -- turning just swaps which edge is clipped, and the only thing that
        puts the whole window in frame is distance. Backwards along the
        current heading, which points at the window, so this retreats along
        the line of sight and does not lose it.
        """
        lp = self.local_position
        if lp is None or not self.hold_xy:
            self.get_logger().warning(
                "RECENTRE: want to back off but there is no lateral estimate to "
                "do it on. Holding.", throttle_duration_sec=2.0)
            self.recentre_attempt_since = time.monotonic()
            return

        if self.recentre_backoffs >= self.RECENTRE_MAX_BACKOFFS:
            self._abandon(
                f"the window was still truncated after "
                f"{self.RECENTRE_MAX_BACKOFFS} back-offs -- it does not fit in "
                "the field of view from anywhere this approach can reach")
            return

        self.recentre_backoffs += 1
        heading = lp.heading
        target = (self.hold_x - math.cos(heading) * self.RECENTRE_BACKOFF,
                  self.hold_y - math.sin(heading) * self.RECENTRE_BACKOFF)
        self.recentre_backoff_target = np.array(target)
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._set_target(target[0], target[1])
        self._restart_stage_clock()
        self.get_logger().warning(
            f"RECENTRE: yawing has not un-truncated the window in "
            f"{self.RECENTRE_BACKOFF_SECONDS:.0f} s, so it does not fit in the "
            f"frame from here. Backing off {self.RECENTRE_BACKOFF:.2f} m "
            f"(attempt {self.recentre_backoffs}/{self.RECENTRE_MAX_BACKOFFS}).")

    def _arrived_at_backoff(self):
        lp = self.local_position
        if lp is None or self.recentre_backoff_target is None:
            return True
        return math.hypot(self.recentre_backoff_target[0] - lp.x,
                          self.recentre_backoff_target[1] - lp.y) <= self.ALIGN_TOLERANCE

    # ---------------------------------------------------------------- AIM

    def _begin_aim(self, est):
        _, _, heading = self.approach_points(est)
        self._enter_stage(self.AIM)
        self.align_in_band_since = None
        self.get_logger().warning(
            f"AIM: turning to {math.degrees(heading):+.0f} deg to face the "
            f"window square-on, holding position. {self.pose_summary()}.")

    def _handle_aim(self):
        """Yaw onto the window normal while standing still.

        Doing the turn before the translation, rather than during it, is the
        one concession this whole approach makes to the camera: an IMU-less
        stereo camera loses tracking on a fast yaw, and it loses it much more
        readily when the scene is also translating. After this the remaining
        yaw corrections are a few degrees and can be flown alongside the
        approach without noticing.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        est = self.window_estimate()
        if est is None:
            if self._give_up_on_pose('AIM'):
                return
            self.get_logger().info(
                f"AIM: waiting for the window pose. {self.pose_summary()}",
                throttle_duration_sec=1.0)
            return

        _, _, heading = self.approach_points(est)
        self._aim_yaw_at(heading)

        error = self._heading_error(heading)
        if error <= self.AIM_YAW_TOLERANCE and abs(self.yaw_remaining) < math.radians(2.0):
            self._begin_align(est)
            return

        if self._in_stage_for() > self.AIM_TIMEOUT:
            self.get_logger().warning(
                f"AIM timed out {math.degrees(error):.0f} deg short. Starting "
                "the approach anyway -- the alignment gate still has to pass "
                "before anything is flown through.")
            self._begin_align(est)
            return

        self.get_logger().info(
            f"AIM: {math.degrees(error):.0f} deg to turn. {self.pose_summary()}",
            throttle_duration_sec=1.0)

    # -------------------------------------------------------------- ALIGN

    def _begin_align(self, est):
        entry, _, heading = self.approach_points(est)
        self._enter_stage(self.ALIGN)
        self.align_in_band_since = None
        self.MOVE_SPEED = self.APPROACH_SPEED
        # Set the target HERE rather than leaving it to the first tick of the
        # stage. A single-frame pose dropout on that tick would otherwise leave
        # move_target_x as None while the stage is already running, and every
        # distance-to-target in ALIGN would be arithmetic on it.
        self._set_target(entry[0], entry[1], self.window_altitude(est))
        self.get_logger().warning(
            f"ALIGN: flying to ({entry[0]:+.2f}, {entry[1]:+.2f}) NED, "
            f"{self.STANDOFF_DISTANCE:.2f} m in front of the window on its "
            f"axis, at {self.APPROACH_SPEED:.2f} m/s. {self.pose_summary()}.")

    def _handle_align(self):
        """Fly to the standoff point, re-deriving it from the live estimate.

        The target is recomputed every tick. The carrot the base class walks
        towards it does the smoothing: the setpoint moves at MOVE_SPEED and is
        leashed to the measured position, so an estimate that shifts 10 cm
        does not produce a 10 cm step in what PX4 is asked for -- it produces a
        slightly different direction of travel for the next tick.

        The gate into TRAVERSE is three simultaneous conditions held for
        align_settle_seconds: inside align_tolerance of the standoff point,
        within align_alt_tolerance of the traverse altitude, and within
        align_yaw_tolerance of the window normal. Position alone is not
        alignment -- being in the right place pointing 20 degrees off puts the
        aircraft through the frame sideways.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        est = self.window_estimate()
        if est is None:
            # Keep the last target rather than stopping dead: a one-second
            # dropout in the middle of an approach is normal, and freezing the
            # carrot on every one of them makes the approach jerky.
            #
            # Note what is NOT here: the give-up check. It used to run at the
            # top of this stage and return, which meant a vehicle that was 3 cm
            # from the approach point, 1 degree off the normal and still
            # closing got abandoned -- and landed -- because the detector had
            # been quiet for six seconds. The target, the heading and the
            # alignment gate are all still valid without a live pose; they are
            # measured against move_target_x/y, which is frozen. So the gate
            # gets its chance first, and the give-up is checked below, only
            # once we know this tick did not finish the job.
            self.get_logger().info(
                f"ALIGN: pose stale, holding the last target. {self.pose_summary()}",
                throttle_duration_sec=1.0)
        else:
            entry, _, heading = self.approach_points(est)
            altitude = self.window_altitude(est)
            self._set_target(entry[0], entry[1], altitude)
            self._aim_yaw_at(heading)

        if not self.hold_xy:
            # No lateral estimate -> the inherited setpoint publisher has
            # already dropped back to "do not translate". Stop walking a
            # carrot the vehicle is not chasing.
            self.moving = False
            self.align_in_band_since = None
            self.get_logger().warning(
                "ALIGN: vision unhealthy, holding still until it comes back.",
                throttle_duration_sec=2.0)
            if self._in_stage_for() > self.ALIGN_TIMEOUT:
                self._abandon("vision never recovered during the approach")
            return

        if self._aligned():
            if self.align_in_band_since is None:
                self.align_in_band_since = time.monotonic()
            elif time.monotonic() - self.align_in_band_since >= self.ALIGN_SETTLE_SECONDS:
                self._begin_traverse()
            return

        self.align_in_band_since = None

        # Not aligned and no live pose: now the stale clock is allowed to end
        # the attempt.
        if est is None and self._give_up_on_pose('ALIGN'):
            return

        if self._in_stage_for() > self.ALIGN_TIMEOUT:
            self._abandon(
                f"could not settle on the approach point in "
                f"{self.ALIGN_TIMEOUT:.0f} s")
            return

        along, cross = self._approach_errors()
        if along is None:
            along = cross = 0.0
        alt = self.relative_altitude()
        self.get_logger().info(
            f"ALIGN: {-along:+.2f} m along / {cross:+.2f} m across to the "
            f"approach point (tolerances {self.ALIGN_ALONG_TOLERANCE:.2f}/"
            f"{self.ALIGN_CROSS_TOLERANCE:.2f}), "
            f"{math.degrees(self._heading_error(self._target_heading())):.0f} deg "
            f"off the normal, alt {'n/a' if alt is None else f'{alt:+.2f}'}/"
            f"{self.commanded_altitude:.2f} m. {self.pose_summary()}",
            throttle_duration_sec=1.0)

    def _target_heading(self):
        est = self.window_estimate()
        if est is not None:
            return self.approach_points(est)[2]
        return self.yaw_setpoint

    def _approach_errors(self):
        """(along, cross) metres from the standoff point, in the traverse frame.

        along is signed along the direction of travel -- positive means past
        the standoff point, towards the window. cross is signed perpendicular
        to it, positive to the right of the direction of travel, and is the
        number that matters: it is the aircraft's offset from the window's
        centreline, and it is subtracted from the lateral clearance one for one.

        Returns (None, None) if there is nothing to measure against.
        """
        lp = self.local_position
        if lp is None or self.move_target_x is None:
            return None, None
        heading = self._target_heading()
        ex = lp.x - self.move_target_x
        ey = lp.y - self.move_target_y
        c, sn = math.cos(heading), math.sin(heading)
        return ex * c + ey * sn, -ex * sn + ey * c

    def _aligned(self):
        """All four axes of "lined up", simultaneously.

        Four rather than three now: along-track and cross-track are separate
        gates with different tolerances, because they buy different things.
        Cross-track is charged at the tight rate -- it is the aircraft's
        distance from the centreline of the aperture it is about to fly
        through, and it is spent out of a clearance budget that on a typical
        window is under 20 cm a side.
        """
        along, cross = self._approach_errors()
        if along is None:
            return False
        if abs(cross) > self.ALIGN_CROSS_TOLERANCE:
            return False
        if abs(along) > self.ALIGN_ALONG_TOLERANCE:
            return False
        alt = self.relative_altitude()
        if alt is None or abs(alt - self.commanded_altitude) > self.ALIGN_ALT_TOLERANCE:
            return False
        return self._heading_error(self._target_heading()) <= self.ALIGN_YAW_TOLERANCE

    # ----------------------------------------------------------- TRAVERSE

    def _aperture_is_flyable(self, est):
        """Is there physically room for this aircraft, AS IT IS NOW, in this aperture?

        Called once, at the commit, because that is the last moment anything
        can still be called off cheaply -- after it the target is frozen and
        the camera is no longer steering. A window that cannot clear the
        airframe by HARD_CLEARANCE is abandoned rather than attempted: the
        failure mode of trying is the gear catching an edge and the aircraft
        going over, which is exactly what happened when nothing checked.

        The lateral half of this used to be a statement about the WINDOW --
        half its width less half the airframe's -- and that made it a check
        the aircraft could pass while sitting well off the centreline pointing
        several degrees off the normal, which is the state a prop hits a jamb
        from. It is now a statement about the AIRCRAFT IN THIS WINDOW: the
        measured cross-track offset comes out of the margin, and the measured
        heading error widens the airframe. The alignment gate keeps both small;
        this is what makes sure they were actually small.
        """
        cross = self.cross_track_of(est)
        if cross is None:
            cross = 0.0
        _, _, heading = self.approach_points(est)
        yaw_error = self._heading_error(heading)

        vertical, lateral = self.aperture_margins(est, cross=cross, yaw_error=yaw_error)
        geometric = 0.5 * (float(est.get('width') or 0.0) - self.DRONE_WIDTH)

        if vertical < self.HARD_CLEARANCE or lateral < self.HARD_CLEARANCE:
            self._abandon(
                f"the window measures {est['width']:.2f}x{est['height']:.2f} m "
                f"and the aircraft is {cross:+.2f} m off its centreline at "
                f"{math.degrees(yaw_error):.0f} deg of yaw error, which leaves "
                f"{vertical:.2f} m vertically and {lateral:.2f} m laterally "
                f"around a {self.DRONE_WIDTH:.2f}x{self.DRONE_HEIGHT:.2f} m "
                f"airframe (geometric lateral margin would be {geometric:.2f} m "
                f"if it were centred and square) -- less than the "
                f"{self.HARD_CLEARANCE:.2f} m minimum")
            return False
        if vertical < self.VERTICAL_CLEARANCE or lateral < self.LATERAL_CLEARANCE:
            self.get_logger().warning(
                f"Tight fit: {vertical:.2f} m vertical and {lateral:.2f} m "
                f"lateral margin, below the {self.VERTICAL_CLEARANCE:.2f}/"
                f"{self.LATERAL_CLEARANCE:.2f} m wanted. Off the centreline by "
                f"{cross:+.2f} m at {math.degrees(yaw_error):.0f} deg. "
                "Flying it anyway.")
        return True

    def _begin_traverse(self):
        """Commit: freeze the target and stop listening to the camera.

        Everything about the run through the window is decided here, from the
        one place in the flight where the geometry is best known: stationary,
        square to the aperture, standoff_distance away with the whole window in
        frame. Nothing measured after this point can improve on that, and the
        things that could be measured -- a frame filling the field of view,
        depth on an image edge -- are the camera's worst cases.
        """
        est = self.window_estimate()
        if est is None and self.last_good_est is not None:
            # The live pose is gone but the approach was flown on a real
            # measurement and the aircraft is sitting in the gate derived from
            # it. Commit on that measurement rather than on nothing: it is
            # what the standoff point, the altitude and the heading were all
            # built from, and it is what makes _aperture_is_flyable below a
            # real check instead of a skipped one. The window has not moved;
            # only the camera's opinion of it has gone quiet.
            age = time.monotonic() - self.last_good_est_time
            self.get_logger().warning(
                f"Committing on the last good window pose ({age:.1f} s old): "
                "the detector went quiet during the alignment settle, but the "
                "aircraft is aligned on the geometry that pose produced.")
            est = self.last_good_est

        if est is None:
            # Only reachable if the estimate died in the settle window. The
            # aircraft is in the right place pointing the right way; use the
            # geometry it settled onto.
            heading = self.yaw_setpoint
            entry = np.array([self.move_target_x, self.move_target_y, 0.0])
            # The last estimate is gone, so the effective standoff cannot be
            # recomputed. The one the aircraft actually flew to is the last
            # one logged; fall back to the parameter only if there is none.
            standoff = (self._last_standoff_logged
                        if self._last_standoff_logged > self.STANDOFF_DISTANCE
                        else self.STANDOFF_DISTANCE)
            exit_point = entry + np.array([
                math.cos(heading) * (standoff + self.EXIT_DISTANCE),
                math.sin(heading) * (standoff + self.EXIT_DISTANCE),
                0.0])
            self.get_logger().warning(
                "Committing to the traverse on the settled heading: the pose "
                "estimate went stale during the alignment settle.")
        else:
            if not self._aperture_is_flyable(est):
                return
            entry, exit_point, heading = self.approach_points(est)
            self.traverse_window = est

        self.traverse_entry = entry
        self.traverse_exit = exit_point
        self.traverse_heading = heading
        # Freeze the standoff the entry point was actually built from. TRAVERSE
        # stops looking at the camera, so recomputing it from a newer estimate
        # would be measuring progress along a line the aircraft is not on.
        self.traverse_standoff = (standoff if est is None
                                  else self.effective_standoff(est))
        self.blind_traverse_since = None
        self.MOVE_SPEED = self.TRAVERSE_SPEED
        self._set_target(exit_point[0], exit_point[1])
        self.yaw_remaining = wrap_pi(heading - self.yaw_setpoint)
        self._enter_stage(self.TRAVERSE)

        if self.traverse_window is None:
            size = 'unknown size'
        else:
            commit_cross = self.cross_track_of(self.traverse_window) or 0.0
            v_margin, h_margin = self.aperture_margins(
                self.traverse_window, cross=commit_cross,
                yaw_error=self._heading_error(heading))
            size = (f"{self.traverse_window['width']:.2f}x"
                    f"{self.traverse_window['height']:.2f} m, leaving "
                    f"{v_margin:.2f} m above and below the airframe and "
                    f"{h_margin:.2f} m either side")
        self.get_logger().warning(
            f"TRAVERSE: committed. Flying through to ({exit_point[0]:+.2f}, "
            f"{exit_point[1]:+.2f}) NED at {self.TRAVERSE_SPEED:.2f} m/s on a "
            f"heading of {math.degrees(heading):+.0f} deg, altitude "
            f"{self.commanded_altitude:.2f} m. Window {size}. The camera is "
            "no longer steering: this target is frozen.")

    def _handle_traverse(self):
        if not self._still_flyable():
            return

        # This is what notices vision dying mid-run, and it is safe to call
        # here: the latching branch only fires when hold_xy is already False,
        # so it cannot disturb the carrot while the run is going well. When it
        # does fire -- vision coming back after a blind push -- re-anchoring on
        # the current position is exactly what should happen, and the carrot
        # then walks on to the frozen exit point from there.
        self._try_latch_xy_hold()
        self.log_flight_state()

        if not self.hold_xy:
            self._handle_blind_traverse()
            return

        self.blind_traverse_since = None
        self.yaw_remaining = wrap_pi(self.traverse_heading - self.yaw_setpoint)

        # Progress is measured ALONG the traverse line, not as a distance to
        # the exit point: a metre of crosstrack error would otherwise read as
        # "not there yet" forever and burn the timeout.
        along = self._distance_along_traverse()
        total = (self.traverse_standoff or self.STANDOFF_DISTANCE) + self.EXIT_DISTANCE
        if along >= total - self.ALIGN_TOLERANCE:
            self._begin_clear(f"through, {along:.2f} m flown of {total:.2f} m")
            return

        if self._in_stage_for() > self.TRAVERSE_TIMEOUT:
            self._abandon(
                f"traverse timed out {total - along:.2f} m short of the far side")
            return

        self.get_logger().info(
            f"TRAVERSE: {along:.2f}/{total:.2f} m, "
            f"{math.degrees(self._heading_error(self.traverse_heading)):.0f} deg "
            f"off the committed heading.", throttle_duration_sec=0.5)

    def _distance_along_traverse(self):
        """How far down the committed line the vehicle is, from the entry."""
        lp = self.local_position
        if lp is None or self.traverse_entry is None:
            return 0.0
        direction = np.array([math.cos(self.traverse_heading),
                              math.sin(self.traverse_heading)])
        offset = np.array([lp.x - self.traverse_entry[0],
                           lp.y - self.traverse_entry[1]])
        return float(np.dot(offset, direction))

    def _handle_blind_traverse(self):
        """Vision died mid-run: push on for a few seconds, then land.

        A position setpoint against a dead lateral estimate is a setpoint
        against a number that means nothing, so the inherited publisher has
        already stopped sending one. What replaces it here is an open-loop
        velocity along the committed heading -- open loop is exactly what the
        base class refuses to do anywhere else, and it is right to, but the
        alternative in this one place is stopping inside an aperture with no
        way to tell which side of it the aircraft is on.

        Bounded hard: blind_traverse_seconds of it, then a landing. It is a way
        out of the frame, not a way to complete the mission.
        """
        now = time.monotonic()
        total = (self.traverse_standoff or self.STANDOFF_DISTANCE) + self.EXIT_DISTANCE
        if self.blind_traverse_since is None:
            self.blind_traverse_since = now
            # Bounded by DISTANCE as well as time: push only as far as the
            # rest of the traverse, measured where vision was lost. A time
            # limit alone let an 8 s push cross a whole 5 m room into the
            # far wall.
            left = max(0.0, total - self._distance_along_traverse())
            self.blind_traverse_limit = min(
                self.BLIND_TRAVERSE_SECONDS,
                left / max(self.TRAVERSE_SPEED, 0.05) + 0.5)
            self.blind_traverse_left = left
            self.get_logger().error(
                "TRAVERSE: vision lost mid-run. Pushing on open-loop along the "
                f"committed heading for {self.blind_traverse_limit:.1f} s "
                f"(the remaining {left:.2f} m) rather than stopping in the window.")

        elapsed = now - self.blind_traverse_since
        if elapsed >= self.blind_traverse_limit:
            self._blind_push_done(total)
            return

        self.get_logger().warning(
            f"TRAVERSE: blind, {self.blind_traverse_limit - elapsed:.1f} s left.",
            throttle_duration_sec=0.5)

    def _blind_push_done(self, total):
        """The bounded blind push ran out without vision. Default: land.

        Subclasses that have another way to know where they are on the far
        side (a lidar in the room) may treat a push that covered the whole
        remaining distance as a completed traverse instead.
        """
        self.outcome = (
            f"BLIND: vision lost mid-traverse, pushed on "
            f"{self.blind_traverse_left:.2f} m open-loop of {total:.2f} m")
        self._begin_landing(
            f"vision did not come back within {self.blind_traverse_limit:.1f} s "
            "of the blind push")

    def _blind_traverse_active(self):
        return (self.current_stage == self.TRAVERSE
                and self.blind_traverse_since is not None
                and not self.hold_xy)

    # -------------------------------------------------------------- CLEAR

    def _begin_clear(self, reason):
        self.outcome = reason
        self.moving = False
        self.MOVE_SPEED = self.APPROACH_SPEED
        lp = self.local_position
        if lp is not None and self.hold_xy:
            self.hold_x = lp.x
            self.hold_y = lp.y
        self._enter_stage(self.CLEAR)
        self.get_logger().warning(
            f"WINDOW TRAVERSED ({reason}). Holding {self.CLEAR_SECONDS:.0f} s "
            "on the far side, then landing.")

    def _handle_clear(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        remaining = self.CLEAR_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_landing("traverse complete")
            return

        self.get_logger().info(
            f"CLEAR: holding, {remaining:.1f} s to the descent.",
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------ giving up

    def _give_up_on_pose(self, stage):
        """True if the estimate has been gone long enough to abandon the run."""
        if self.pose_lost_since is None:
            return False
        if time.monotonic() - self.pose_lost_since < self.POSE_LOST_TIMEOUT:
            return False
        self._abandon(
            f"no usable window pose for {self.POSE_LOST_TIMEOUT:.0f} s during {stage}")
        return True

    def _abandon(self, reason):
        """Stop the attempt and land from wherever we are.

        Not a retry. Going back to SCAN after a failed approach means a vehicle
        that is now somewhere other than where it swept from, with an unknown
        amount of battery left, starting the same attempt with the same
        conditions that just failed. Land, look at the log, fly it again.
        """
        self.outcome = f"ABANDONED: {reason}"
        self.moving = False
        self.yaw_remaining = 0.0
        self.MOVE_SPEED = self.APPROACH_SPEED
        self.get_logger().error(f"Traverse abandoned: {reason}.")
        self._begin_landing(f"traverse abandoned -- {reason}")

    # ------------------------------------------------------------ publishers

    def publish_position_setpoint(self):
        """The inherited setpoint, except during a blind traverse.

        Everything about this node except those few seconds publishes an
        absolute NED point through the base class. The blind push is the one
        exception, and it is written out here rather than bolted into the base
        class because it is a behaviour that only makes sense inside a window.
        """
        if not self._blind_traverse_active() or self.home_z is None:
            super().publish_position_setpoint()
            return

        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Altitude stays a position setpoint on the ramp: the height estimate
        # is the lidar's and is unaffected by whatever happened to vision.
        self._step_setpoint_ramp()
        self._step_yaw_ramp()

        speed = self.TRAVERSE_SPEED
        msg.position = [nan, nan, self.setpoint_z]
        msg.velocity = [speed * math.cos(self.traverse_heading),
                        speed * math.sin(self.traverse_heading), nan]
        msg.yaw = self.yaw_setpoint
        self.trajectory_setpoint_pub.publish(msg)

    def publish_status(self):
        """stage|armed|altitude|xy|detail, the format the LCD node reads."""
        if self.current_stage not in self.TRAVERSE_STAGES:
            super().publish_status()
            return

        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        lp = self.local_position

        if self.current_stage == self.RECENTRE:
            det = self.fresh_detection()
            if det is None:
                detail = "rc?"
            elif det['truncated']:
                detail = f"rc{math.degrees(det['centre_az']):+.0f}"
            else:
                detail = "rcOK"
        elif self.current_stage == self.AIM:
            detail = f"aim{math.degrees(abs(self.yaw_remaining)):.0f}"
        elif self.current_stage == self.ALIGN:
            # Cross-track is the one worth the four characters on the LCD: it
            # is what the traverse clearance is spent on.
            _, cross = self._approach_errors()
            detail = "algn?" if cross is None else f"algnX{cross:+.2f}"
        elif self.current_stage == self.TRAVERSE:
            total = (self.traverse_standoff or self.STANDOFF_DISTANCE) + self.EXIT_DISTANCE
            detail = f"thru{self._distance_along_traverse():.1f}/{total:.1f}"
        else:
            detail = f"{max(0.0, self.CLEAR_SECONDS - self._in_stage_for()):.0f}s"

        msg = String()
        msg.data = "|".join([
            self.current_stage,
            'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('FLO' if self.flow_is_healthy() else '---'),
            detail,
        ])
        self.status_pub.publish(msg)

    # --------------------------------------------------------------- logging

    def log_flight_state(self):
        super().log_flight_state()
        self.get_logger().info(
            f"window: {self.pose_summary()}", throttle_duration_sec=2.0)

    def destroy_node(self):
        self.get_logger().warning(
            f"Traverse outcome: {self.outcome}. "
            f"{self.estimator.accepted_total} geometry samples accepted of "
            f"{self.geometry_seen} received; rejections: "
            f"{self.estimator.rejection_summary(limit=5)}.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WindowTraverse()
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
