#!/usr/bin/env python3
"""Take off, find the ArUco ring board, fly the ring, come back and land.

    climb -> hold -> sweep +/-45 deg for the board -> lock -> COARSE line-up
    at a standoff -> FINE proportional alignment -> GRAB, 20 cm behind the
    grab marker -> back out to where the fine alignment finished -> land.

TWO MODES, AND THE DEFAULT IS THE SAFE ONE
------------------------------------------
    mode:=pose      TROUBLESHOOTING. Commands nothing, arms nothing, never
                    touches the flight controller. It listens to
                    /ring_geometry and reports, as accurately as it can, where
                    the drone is IN THE BOARD FRAME -- right, up, depth and
                    yaw -- with the jitter of each over a rolling window so
                    you can see how good the measurement actually is. This is
                    the default, so forgetting to pass anything gets you the
                    harmless one.

    mode:=grab      THE FLIGHT, above.

Same escalation the rest of this package uses: bench the numbers first, fly
second. A sign error in the board frame does not wobble -- it flies the
aircraft at a point behind the board and accelerates, because every new frame
says the target is further that way.

    q -> abort into a controlled descent.   k -> force-disarm.

WHAT THIS BORROWS, AND WHAT IT ADDS
-----------------------------------
Arming, the climb, the flow-health gates, the yaw ramp, the x/y carrot and its
leash, the descent and the touchdown detection are all inherited from
offboard_sequence.OffboardSequence. The +/-45 degree search sweep and the lock
are inherited from window_scan.WindowScan -- the same sweep the window mission
flies, at the same deliberately slow rate, for the same reason (a detector
needs several consecutive frames on a target, and a fast yaw sweeps past
things it technically saw). Nothing about any of that is re-implemented here.

What is new is the four stages after the lock, and the geometry that feeds
them.

NO EKF FUSION. NO BRIDGE.
-------------------------
The ring_grab package this is ported from fed the board pose into EKF2 as an
external-vision source, through odom_align and vio_px4_bridge, so the board
became a position reference. None of that is here and none of it is wanted.
Position on this airframe is PMW3901 optical flow plus a TFmini Plus through
EKF2, exactly as in the window mission; the board only ever produces a
SETPOINT. That means a bad detection can steer the aircraft, but it can never
corrupt the state estimate, and losing the board at the last moment leaves a
vehicle that still knows where it is.

THE 20 CM, AND WHICH SIDE OF THE BOARD IT IS ON
-----------------------------------------------
grab_behind is measured from the GRAB MARKER's centre, along the board's
normal, AWAY from the drone -- so the commanded point is 20 cm on the far side
of the board plane and the aircraft flies THROUGH the ring. Positive means
further through. The marker it is measured from is the detector's
grab_marker_id (id 4 by default, the bottom of the stack), and because the
three markers are fused into one board pose, that point is known even on
frames where marker 4 itself is out of view -- which it will be, since it
leaves the frame before the aircraft gets there.

Everything from the start of GRAB is flown OPEN LOOP on a target frozen at
that moment. It has to be: at 20 cm the board is far too close to see, so
there is nothing left to correct against. This is the same commit the window
traversal makes for the same reason, and it is why FINE has to finish the job.

COARSE NEVER FLIES BACKWARDS
----------------------------
The natural way to write the line-up is "go to fine_standoff metres in front
of the board". That is wrong when the sweep happens to end with the aircraft
already closer than that, because it makes the first thing the vehicle does be
a retreat -- away from a target it can see, burning battery and clearance for
nothing.

So the standoff COARSE flies to is the nearer of (fine_standoff, the depth the
aircraft is at right now), frozen at the moment COARSE begins:

    approach_standoff = min(fine_standoff, current depth)      (>= min_standoff)

Far away, that is fine_standoff and the vehicle closes in. Already inside it,
that is where it already is, and the vehicle only aligns. The one case it does
back off is closer than min_standoff, which is too close to align at; that is
logged loudly when it happens.

Frozen, not recomputed: a min() against the live depth would ratchet the
target inwards every tick as the aircraft closed on it and walk it into the
board.

FINE IS A PROPORTIONAL POSE SETPOINT, NOT A VELOCITY
-----------------------------------------------------
Nothing in this file publishes a velocity setpoint. Every stage sets an
absolute NED point and lets the inherited carrot walk to it at MOVE_SPEED,
leashed to the measured position -- the same interface _step_xy_ramp consumes.

FINE additionally commands only align_gain of the measured error each cycle,
the way precision_land does:

    target = current position + align_gain * (ideal point - current position)

A gain below 1 is not about the scale. It is phase margin against camera and
link latency, and it guarantees monotone convergence even if the vision scale
is off by a third in the wrong direction. Raise align_tolerance if the
approach is too fussy; do not raise align_gain.

/ring/board_pose -- WHERE THE DRONE IS, IN THE BOARD'S OWN FRAME
-----------------------------------------------------------------
Published in every mode, on every accepted detection, whether or not anything
is flying:

    /ring/board_pose    geometry_msgs/PoseStamped   frame_id 'ring_board'
    /ring/board_yaw     std_msgs/Float32            degrees
    /ring/board_pose_info  std_msgs/String          the same, in words

The frame's origin is the GRAB MARKER's centre and its axes are the board's
own, right-handed:

    x  RIGHT across the board, as seen by someone looking AT it
    y  UP the board
    z  OUT of the board, towards the drone -- so z IS THE DEPTH and a drone in
       front of the board has positive z

The pose is the vehicle BODY origin, not the camera: the cam_x/cam_y/cam_z
lever arm and the cam_roll/cam_pitch/cam_yaw mounting rotation are both
applied. The orientation is a real quaternion of the body FRD axes in that
frame -- unlike the original ring_grab, which packed four unrelated scalars
into a PoseStamped's quaternion and left it to the reader.

/ring/board_yaw is the nose angle about the board's vertical: 0 is squared up
to the board, positive is nosed to the RIGHT. This is the number to watch in
mode:=pose. It is derived from the marker corners alone, so it is independent
of the EKF and of the magnetometer, and it is the one measurement that decides
whether the aircraft goes through the ring or catches its shoulder on it.
"""

import math
import threading
import time
from collections import deque

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleAttitude, VehicleStatus
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Float32, Float32MultiArray, String

from drone_testing.offboard_sequence import spin_node, wrap_pi
from drone_testing.window_scan import WindowScan
# Imported rather than copied. These two are the vehicle-frame geometry the
# window mission already flies on, and a second copy that drifts by a sign is
# exactly the bug that is impossible to find in the air.
from drone_testing.window_traverse import quat_rotate, rpy_to_matrix_frd


def rot_to_quat(R):
    """SO(3) -> (w, x, y, z). Shepperd's method, the largest-denominator branch."""
    t = float(np.trace(R))
    if t > 0.0:
        s = 0.5 / math.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    return q / max(float(np.linalg.norm(q)), 1e-12)


# ------------------------------------------------------------- the estimator

class RingEstimator:
    """A rolling median of the grab point and the board normal, in NED.

    Deliberately much simpler than WindowEstimator: an ArUco pose is a far
    stronger measurement than four depth pixels on a window frame, so there is
    no shape validation to do here -- ring_detect has already thrown out
    anything that did not solve, sat outside the depth band, or did not fit
    the corners it was solved from.

    What is left is the job a single frame cannot do: reject the occasional
    outlier that survived all that, and average down the jitter. Median rather
    than mean, because the failure mode being defended against is one bad
    sample rather than gaussian noise.
    """

    def __init__(self, *, buffer_seconds, buffer_max, min_samples,
                 gate_metres, gate_yaw, gate_reset_count):
        self.buffer_seconds = buffer_seconds
        self.min_samples = min_samples
        self.gate_metres = gate_metres
        self.gate_yaw = gate_yaw
        self.gate_reset_count = gate_reset_count
        self.samples = deque(maxlen=buffer_max)
        self._lock = threading.Lock()
        self.consecutive_gated = 0
        self.accepted_total = 0
        self.gated_total = 0
        self.last_reason = ''

    def add(self, grab_ned, normal_h, now):
        with self._lock:
            return self._add(grab_ned, normal_h, now)

    def _add(self, grab_ned, normal_h, now):
        est = self._estimate(now)
        if est is not None:
            if float(np.linalg.norm(grab_ned - est['grab'])) > self.gate_metres:
                return self._gated('grab point jumped')
            swing = abs(wrap_pi(math.atan2(normal_h[1], normal_h[0])
                                - math.atan2(est['normal'][1], est['normal'][0])))
            if swing > self.gate_yaw:
                return self._gated('board normal swung')

        self.consecutive_gated = 0
        self.accepted_total += 1
        self.last_reason = ''
        self.samples.append({'t': now, 'grab': grab_ned, 'normal': normal_h})
        return True, ''

    def _gated(self, reason):
        self.consecutive_gated += 1
        self.gated_total += 1
        self.last_reason = reason
        if self.consecutive_gated >= self.gate_reset_count:
            # Every new sample disagrees with the estimate, so the estimate is
            # the minority opinion. Throwing it away is what stops the
            # aircraft flying confidently at a board that is not there.
            self.samples.clear()
            self.consecutive_gated = 0
            return False, reason + ' (buffer reset)'
        return False, reason

    def _fresh(self, now):
        return [s for s in self.samples if now - s['t'] <= self.buffer_seconds]

    def fresh_count(self, now):
        with self._lock:
            return len(self._fresh(now))

    def estimate(self, now):
        with self._lock:
            return self._estimate(now)

    def _estimate(self, now):
        fresh = self._fresh(now)
        if len(fresh) < self.min_samples:
            return None
        grab = np.median(np.array([s['grab'] for s in fresh]), axis=0)
        # Circular mean of the normal's bearing: averaging the vectors would
        # be wrong across the +/-180 wrap and would also shorten the result.
        angles = np.array([math.atan2(s['normal'][1], s['normal'][0])
                           for s in fresh])
        bearing = math.atan2(float(np.mean(np.sin(angles))),
                             float(np.mean(np.cos(angles))))
        return {
            'grab': grab,
            'normal': np.array([math.cos(bearing), math.sin(bearing), 0.0]),
            'bearing': bearing,
            'samples': len(fresh),
            'age': now - max(s['t'] for s in fresh),
        }

    def rotate_frame(self, delta, pivot):
        """EKF2 re-datumed yaw: turn every stored sample with the frame.

        The board did not move and the aircraft did not move, but every sample
        in here was placed using an attitude EKF2 has just declared wrong by
        `delta`, so as a set they are rotated by exactly that much about the
        point they were measured from.
        """
        if abs(delta) < 1e-6:
            return
        with self._lock:
            self._rotate_frame(delta, pivot)

    def _rotate_frame(self, delta, pivot):
        c, s = math.cos(delta), math.sin(delta)
        for sample in self.samples:
            dx = sample['grab'][0] - pivot[0]
            dy = sample['grab'][1] - pivot[1]
            sample['grab'] = np.array([pivot[0] + c * dx - s * dy,
                                       pivot[1] + s * dx + c * dy,
                                       sample['grab'][2]])
            nx, ny = sample['normal'][0], sample['normal'][1]
            sample['normal'] = np.array([c * nx - s * ny, s * nx + c * ny, 0.0])


# ---------------------------------------------------------------- the flight

class RingGrab(WindowScan):

    COARSE = "COARSE"
    FINE = "FINE"
    GRAB = "GRAB"
    BACKOUT = "BACKOUT"

    # ---- inherited knobs, re-defaulted for this mission -------------------
    DETECT_TOPIC = 'ring_detected'      # what WindowScan's lock waits on
    SCAN_SPAN = math.radians(90.0)      # +/-45 deg about the takeoff heading
    TAKEOFF_ALTITUDE = 1.50
    FLIGHT_SECONDS = 180.0

    MODE = 'pose'                       # pose | grab. The safe one by default.

    # ---- geometry ---------------------------------------------------------
    GRAB_BEHIND = 0.20          # m past the board plane, measured at the grab
                                # marker. THE number this mission is about.
    FINE_STANDOFF = 1.00        # m in front of the board that FINE aligns at
    MIN_STANDOFF = 0.35         # m. Closer than this there is not enough board
                                # left in frame to align on, so COARSE does
                                # back off to here -- the one exception to
                                # never flying backwards.
    BACKOUT_EXTRA = 0.00        # m beyond the fine point to retreat to. 0 =
                                # come back exactly where the alignment ended.

    # ---- speeds -----------------------------------------------------------
    APPROACH_SPEED = 0.30       # m/s, COARSE and FINE
    GRAB_SPEED = 0.35           # m/s through the ring. Committed and open
                                # loop, so brisk enough not to linger but slow
                                # enough that a frozen target that is 10 cm out
                                # is still a graze rather than an impact.
    BACKOUT_SPEED = 0.30        # m/s coming back out, nose still on the board

    # ---- tolerances -------------------------------------------------------
    COARSE_TOLERANCE = 0.25     # m from the standoff point to call COARSE done
    COARSE_SETTLE_SECONDS = 1.0
    ALIGN_GAIN = 0.6            # fraction of the measured error commanded per
                                # cycle. < 1 guarantees monotone convergence.
    ALIGN_TOLERANCE = 0.08      # m. This is the number that decides whether
                                # the aircraft goes through the ring.
    ALIGN_SETTLE_SECONDS = 1.5
    YAW_TOLERANCE_DEG = 6.0     # deg of heading error allowed at the end of
                                # FINE. A yaw error at 1 m standoff walks the
                                # aircraft sideways by standoff*sin(err) as it
                                # runs in, and nothing is watching by then.
    GRAB_TOLERANCE = 0.12       # m from the frozen grab point to call it done
    GRAB_HOLD_SECONDS = 2.0     # s parked at the grab point before backing out
    BACKOUT_TOLERANCE = 0.25    # m

    # ---- timeouts ---------------------------------------------------------
    LOCK_TIMEOUT = 20.0         # s after the lock to build a usable pose
    COARSE_TIMEOUT = 45.0
    FINE_TIMEOUT = 45.0
    GRAB_TIMEOUT = 25.0
    BACKOUT_TIMEOUT = 30.0
    POSE_MAX_AGE = 1.0          # s. Older than this is not evidence.
    POSE_LOST_TIMEOUT = 6.0     # s without a usable pose in COARSE/FINE before
                                # the attempt is abandoned into a landing

    # ---- the estimator ----------------------------------------------------
    BUFFER_SECONDS = 1.5
    BUFFER_MAX = 60
    MIN_SAMPLES = 5
    GATE_METRES = 0.60
    GATE_YAW_DEG = 35.0
    GATE_RESET_COUNT = 25
    MAX_TILT_DEG = 30.0         # how far off vertical the board may be before
                                # the detection is rejected as the floor, a
                                # ceiling or a bad solve

    YAW_CONE_DEG = 70.0         # hard limit on how far the nose may turn from
                                # the takeoff heading, in ANY stage

    ATTITUDE_MAX_HZ = 50.0

    def __init__(self):
        super().__init__('ring_grab')

        self.MODE = str(self.declare_parameter('mode', self.MODE).value).strip().lower()
        if self.MODE not in ('pose', 'grab'):
            raise SystemExit(
                f"mode '{self.MODE}' is not 'pose' or 'grab'. pose = report the "
                "board-frame pose and command nothing; grab = fly the mission.")

        self.GRAB_BEHIND = float(self._declare_number('grab_behind', self.GRAB_BEHIND))
        self.FINE_STANDOFF = float(self._declare_number(
            'fine_standoff', self.FINE_STANDOFF))
        self.MIN_STANDOFF = float(self._declare_number('min_standoff', self.MIN_STANDOFF))
        self.BACKOUT_EXTRA = float(self._declare_number(
            'backout_extra', self.BACKOUT_EXTRA))
        self.APPROACH_SPEED = float(self._declare_number(
            'approach_speed', self.APPROACH_SPEED))
        self.GRAB_SPEED = float(self._declare_number('grab_speed', self.GRAB_SPEED))
        self.BACKOUT_SPEED = float(self._declare_number(
            'backout_speed', self.BACKOUT_SPEED))
        self.COARSE_TOLERANCE = float(self._declare_number(
            'coarse_tolerance', self.COARSE_TOLERANCE))
        self.COARSE_SETTLE_SECONDS = float(self._declare_number(
            'coarse_settle_seconds', self.COARSE_SETTLE_SECONDS))
        self.ALIGN_GAIN = float(self._declare_number('align_gain', self.ALIGN_GAIN))
        self.ALIGN_TOLERANCE = float(self._declare_number(
            'align_tolerance', self.ALIGN_TOLERANCE))
        self.ALIGN_SETTLE_SECONDS = float(self._declare_number(
            'align_settle_seconds', self.ALIGN_SETTLE_SECONDS))
        self.YAW_TOLERANCE = math.radians(float(self._declare_number(
            'yaw_tolerance_deg', self.YAW_TOLERANCE_DEG)))
        self.GRAB_TOLERANCE = float(self._declare_number(
            'grab_tolerance', self.GRAB_TOLERANCE))
        self.GRAB_HOLD_SECONDS = float(self._declare_number(
            'grab_hold_seconds', self.GRAB_HOLD_SECONDS))
        self.BACKOUT_TOLERANCE = float(self._declare_number(
            'backout_tolerance', self.BACKOUT_TOLERANCE))
        self.LOCK_TIMEOUT = float(self._declare_number('lock_timeout', self.LOCK_TIMEOUT))
        self.COARSE_TIMEOUT = float(self._declare_number(
            'coarse_timeout', self.COARSE_TIMEOUT))
        self.FINE_TIMEOUT = float(self._declare_number('fine_timeout', self.FINE_TIMEOUT))
        self.GRAB_TIMEOUT = float(self._declare_number('grab_timeout', self.GRAB_TIMEOUT))
        self.BACKOUT_TIMEOUT = float(self._declare_number(
            'backout_timeout', self.BACKOUT_TIMEOUT))
        self.POSE_MAX_AGE = float(self._declare_number('pose_max_age', self.POSE_MAX_AGE))
        self.POSE_LOST_TIMEOUT = float(self._declare_number(
            'pose_lost_timeout', self.POSE_LOST_TIMEOUT))
        self.MAX_TILT = math.radians(float(self._declare_number(
            'max_tilt_deg', self.MAX_TILT_DEG)))
        self.YAW_CONE = math.radians(float(self._declare_number(
            'yaw_cone_deg', self.YAW_CONE_DEG)))

        if not 0.0 < self.ALIGN_GAIN <= 1.0:
            self.get_logger().error(
                f"align_gain {self.ALIGN_GAIN} is outside (0, 1]; using 0.6. "
                "Above 1 the correction overshoots and the approach rings.")
            self.ALIGN_GAIN = 0.6
        if self.MIN_STANDOFF > self.FINE_STANDOFF:
            raise SystemExit(
                f"min_standoff {self.MIN_STANDOFF} is further out than "
                f"fine_standoff {self.FINE_STANDOFF}; COARSE would have nowhere "
                "to go.")

        # The approach is flown by the inherited carrot, whose speed is
        # MOVE_SPEED. Each stage sets it rather than threading a second speed
        # through _step_xy_ramp, so that ramp -- and its leash -- stays the
        # only thing that ever moves the horizontal setpoint.
        self.MOVE_SPEED = self.APPROACH_SPEED

        # Camera mounting, ROS convention (x fwd, y LEFT, z UP), measured to
        # the D435i's colour imager. The SAME six numbers window_traverse
        # takes, consumed by the same function, on purpose.
        cam_x = float(self._declare_number('cam_x', 0.0))
        cam_y = float(self._declare_number('cam_y', 0.0))
        cam_z = float(self._declare_number('cam_z', 0.0))
        cam_roll = float(self._declare_number('cam_roll', 0.0))
        cam_pitch = float(self._declare_number('cam_pitch', 0.0))
        cam_yaw = float(self._declare_number('cam_yaw', 0.0))
        self.r_cam = rpy_to_matrix_frd(cam_roll, cam_pitch, cam_yaw)
        self.t_cam = np.array([cam_x, -cam_y, -cam_z])      # ROS FLU -> body FRD
        # Where the body origin sits in the camera's own frame. Needed for the
        # board-frame readout, which reports the BODY, not the lens.
        self.body_in_cam = -self.r_cam.T @ self.t_cam

        self.estimator = RingEstimator(
            buffer_seconds=float(self._declare_number(
                'buffer_seconds', self.BUFFER_SECONDS)),
            buffer_max=self.BUFFER_MAX,
            min_samples=int(self._declare_number('pose_min_samples', self.MIN_SAMPLES)),
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
        # The heading in VehicleLocalPosition is not enough to place a point
        # three metres in front of a pitching aircraft; the full quaternion is.
        self.attitude = None
        self.attitude_time = None
        self.attitude_min_interval = 1.0 / self.ATTITUDE_MAX_HZ
        self.attitude_last_kept = 0.0
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self.attitude_callback, qos_profile=sensor_qos,
                                 callback_group=self.sensor_cbg)

        geometry_topic = str(self.declare_parameter('geometry_topic',
                                                    'ring_geometry').value)
        sub = self.create_subscription(Float32MultiArray, geometry_topic,
                                       self.geometry_callback, 10,
                                       callback_group=self.sensor_cbg)
        self.geometry_topic = sub.topic_name
        self.create_subscription(String, 'ring_info', self.ring_info_callback, 10,
                                 callback_group=self.sensor_cbg)

        # Where the drone is in the board's own frame. Published in BOTH modes.
        self.board_pose_pub = self.create_publisher(PoseStamped, '/ring/board_pose', 10)
        self.board_yaw_pub = self.create_publisher(Float32, '/ring/board_yaw', 10)
        self.board_info_pub = self.create_publisher(String, '/ring/board_pose_info', 10)
        # The NED estimate, for anyone watching from the ground.
        self.ring_pose_pub = self.create_publisher(String, 'ring_pose', 10)

        self.ring_info = ''
        self.geometry_seen = 0
        self.pose_ok_since = None
        self.pose_lost_since = None

        # The board-frame readout, and a rolling window of it for mode:=pose.
        self.board_frame_pose = None        # dict, see _publish_board_pose
        self.board_frame_history = deque(maxlen=90)
        # Written in the sensor callback, read by the timer. See RingEstimator.
        self._history_lock = threading.Lock()

        # Frozen at the stage boundaries they are named for.
        self.approach_standoff = None
        self.fine_exit = None               # NED point FINE finished at
        self.fine_exit_altitude = None
        self.grab_point = None              # NED point, 20 cm past the board
        self.backout_point = None           # NED point BACKOUT retreats to
        self.grab_altitude = None
        self.grab_heading = None
        self.grab_axis = None               # unit NED, the run-in direction
        self.grab_start = None              # NED point the run-in started from
        self.grab_arrived_at = None
        self.align_in_band_since = None
        self.coarse_in_band_since = None
        self.outcome = 'not attempted'

        if self.MODE == 'pose':
            # Nothing is going to be commanded, so do not stream setpoints at
            # PX4 either. A vehicle that never sees a setpoint stream cannot
            # be put into Offboard by accident.
            self.stream_setpoints = False
            self.get_logger().warning(
                "mode:=pose -- TROUBLESHOOTING. Nothing will be armed and "
                "nothing will be commanded. Reporting the drone's pose in the "
                "board frame on /ring/board_pose, /ring/board_yaw and "
                "/ring/board_pose_info. Pass mode:=grab to fly.")
        else:
            self.get_logger().warning(
                f"RING GRAB: climb {self.TAKEOFF_ALTITUDE:.2f} m, hold "
                f"{self.HOLD_SECONDS:.0f} s, sweep +/-"
                f"{math.degrees(self.SCAN_SPAN) / 2:.0f} deg for the board, "
                f"line up at {self.FINE_STANDOFF:.2f} m (never further out than "
                "where the sweep left us), align to "
                f"{self.ALIGN_TOLERANCE * 100:.0f} cm at gain "
                f"{self.ALIGN_GAIN:.2f}, then run in to {self.GRAB_BEHIND:.2f} m "
                "PAST the grab marker, back out and land. Hard limit "
                f"{self.FLIGHT_SECONDS:.0f} s from the start of the climb. "
                "q aborts into a descent, k force-disarms.")

    # ------------------------------------------------------------------ subs

    def attitude_callback(self, msg):
        """Newest attitude, decimated to ATTITUDE_MAX_HZ.

        PX4 publishes this at the EKF output rate, an order of magnitude
        faster than anything here can use, and the deserialisation was costing
        the 20 Hz setpoint timer its thread.
        """
        now = time.monotonic()
        if now - self.attitude_last_kept < self.attitude_min_interval:
            return
        self.attitude_last_kept = now
        self.attitude = msg
        self.attitude_time = now

    def ring_info_callback(self, msg):
        self.ring_info = msg.data

    def geometry_callback(self, msg):
        """One accepted detection: report it in the board frame, then in NED.

        The two halves are independent on purpose. The board-frame readout
        needs nothing but the detection and the camera mounting, so it works
        on a bench with no flight controller -- that is what mode:=pose is.
        The NED half needs the vehicle's attitude and position, and is skipped
        when they are not there yet.
        """
        self.geometry_seen += 1
        try:
            rows = np.array(msg.data, dtype=float).reshape(6, 3)
        except ValueError:
            self.get_logger().error(
                f"{self.geometry_topic}: expected 18 floats, got {len(msg.data)}. "
                "Is something else publishing on this topic?",
                throttle_duration_sec=5.0)
            return

        grab_cam = rows[0]
        right_cam, up_cam, normal_cam = rows[1], rows[2], rows[3]
        n_markers, ambiguous, distance = rows[4]
        seen = rows[5]

        self._publish_board_pose(grab_cam, right_cam, up_cam, normal_cam,
                                 int(n_markers), bool(ambiguous), float(distance),
                                 seen)

        if self.MODE != 'grab':
            return
        if self.attitude is None or self.local_position is None:
            return
        if self.home_z is None:
            # Not armed yet, so there is no home to measure an altitude
            # against and NED x/y is not being held. Nothing useful to store.
            return

        now = time.monotonic()
        if now - (self.attitude_time or 0.0) > self.POSE_MAX_AGE:
            return

        lp = self.local_position
        if not (lp.xy_valid and lp.z_valid):
            return

        # Camera -> body FRD -> NED. A point translates, an axis does not.
        q = self.attitude.q
        p_vehicle = np.array([lp.x, lp.y, lp.z])
        grab_ned = quat_rotate(q, self.r_cam @ grab_cam + self.t_cam) + p_vehicle
        normal_ned = quat_rotate(q, self.r_cam @ normal_cam)

        horizontal = math.hypot(normal_ned[0], normal_ned[1])
        if horizontal < math.cos(self.MAX_TILT):
            # The board is vertical. A normal that is not horizontal means the
            # solve is wrong or the thing in frame is not the board, and
            # approaching along it would fly the aircraft at the floor.
            self.get_logger().warning(
                "Board normal is not horizontal; rejecting the sample.",
                throttle_duration_sec=5.0)
            return
        normal_h = np.array([normal_ned[0], normal_ned[1], 0.0]) / horizontal

        ok, reason = self.estimator.add(grab_ned, normal_h, now)
        if not ok:
            self.get_logger().info(f"Sample gated: {reason}.",
                                   throttle_duration_sec=2.0)
        self._publish_ring_pose(now)

    # -------------------------------------------------- the board-frame pose

    def _publish_board_pose(self, grab_cam, right_cam, up_cam, normal_cam,
                            n_markers, ambiguous, distance, seen):
        """Where the DRONE is, in the board's own right-handed frame.

        The detection gives the board's axes as seen from the camera; this is
        the inverse -- the camera, and then the body, as seen from the board.
        Nothing here touches the EKF, the magnetometer or the flight
        controller, which is exactly why it is the measurement to trust when
        something disagrees.
        """
        # Board -> camera rotation, columns being the board axes in camera FRD.
        R_cb = np.column_stack((right_cam, up_cam, normal_cam))
        # Re-orthonormalise. The three axes came through a float32 topic and a
        # weighted rotation average; a few 1e-4 of skew here becomes a
        # misleading yaw below.
        U, _, Vt = np.linalg.svd(R_cb)
        R_cb = U @ Vt
        if np.linalg.det(R_cb) < 0.0:
            U[:, -1] *= -1.0
            R_cb = U @ Vt

        R_bc = R_cb.T                       # camera -> board
        body_board = R_bc @ (self.body_in_cam - grab_cam)
        R_board_body = R_bc @ self.r_cam.T  # body FRD -> board

        # Nose direction in the board frame. Board Z points out at the drone,
        # so a drone squared up to the board has forward = -Z and psi = 0.
        f = R_board_body @ np.array([1.0, 0.0, 0.0])
        psi = math.atan2(f[0], -f[2])

        pose = {
            't': time.monotonic(),
            'right': float(body_board[0]),
            'up': float(body_board[1]),
            'depth': float(body_board[2]),
            'yaw_deg': math.degrees(psi),
            'n_markers': n_markers,
            'ambiguous': ambiguous,
            'distance': distance,
            # Row 5 is one flag per board id in ascending order; this node
            # does not know the ids themselves (only ring_detect does), so
            # they are reported as slots -- slot 0 is the lowest id.
            'slots': [i for i, s in enumerate(seen) if s > 0.5],
        }
        self.board_frame_pose = pose
        with self._history_lock:
            self.board_frame_history.append(pose)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ring_board'
        msg.pose.position.x = pose['right']
        msg.pose.position.y = pose['up']
        msg.pose.position.z = pose['depth']
        qw, qx, qy, qz = rot_to_quat(R_board_body)
        msg.pose.orientation.w = float(qw)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        self.board_pose_pub.publish(msg)

        yaw_msg = Float32()
        yaw_msg.data = float(pose['yaw_deg'])
        self.board_yaw_pub.publish(yaw_msg)

        info = String()
        info.data = (
            f"depth={pose['depth']:+.3f} "
            f"{'RIGHT' if pose['right'] >= 0 else 'LEFT'} {abs(pose['right']):.3f} "
            f"{'UP' if pose['up'] >= 0 else 'DOWN'} {abs(pose['up']):.3f} "
            f"yaw={pose['yaw_deg']:+.1f} deg "
            f"n={n_markers}{' AMBIGUOUS' if ambiguous else ''}")
        self.board_info_pub.publish(info)

    def _board_pose_jitter(self):
        """Std deviation of the readout over the rolling window, for mode:=pose.

        The mean tells you whether the geometry is right. This tells you
        whether the measurement is good enough to fly a 20 cm grab depth off,
        and it is the number that changes when you turn the IR emitter on,
        move a lamp, or print the markers bigger.
        """
        with self._history_lock:
            recent = [p for p in self.board_frame_history
                      if time.monotonic() - p['t'] <= 2.0]
        if len(recent) < 3:
            return None
        out = {}
        for key in ('right', 'up', 'depth', 'yaw_deg'):
            values = np.array([p[key] for p in recent], dtype=float)
            out[key] = (float(values.mean()), float(values.std()))
        out['n'] = len(recent)
        return out

    def _publish_ring_pose(self, now):
        est = self.estimator.estimate(now)
        msg = String()
        if est is None:
            msg.data = (f"no estimate|fresh={self.estimator.fresh_count(now)}"
                        f"|{self.estimator.last_reason}")
        else:
            msg.data = "|".join([
                f"{est['grab'][0]:+.2f}", f"{est['grab'][1]:+.2f}",
                f"{est['grab'][2]:+.2f}",
                f"{math.degrees(est['bearing']):+.0f}",
                f"n={est['samples']}", f"age={est['age']:.2f}",
            ])
        self.ring_pose_pub.publish(msg)

    # -------------------------------------------------------------- estimate

    def ring_estimate(self):
        """The current belief, or None if it is stale or too thin."""
        now = time.monotonic()
        est = self.estimator.estimate(now)
        if est is None:
            return None
        if est['age'] > self.POSE_MAX_AGE:
            return None
        return est

    def _track_pose_health(self):
        est = self.ring_estimate()
        now = time.monotonic()
        if est is None:
            if self.pose_lost_since is None:
                self.pose_lost_since = now
        else:
            self.pose_lost_since = None
            if self.pose_ok_since is None:
                self.pose_ok_since = now
        return est

    def _pose_lost_for(self):
        if self.pose_lost_since is None:
            return 0.0
        return time.monotonic() - self.pose_lost_since

    def pose_summary(self):
        if self.geometry_seen == 0:
            return (f"no messages on {self.geometry_topic} -- is ring_detect "
                    "running?")
        est = self.ring_estimate()
        if est is None:
            return (f"no usable pose ({self.estimator.fresh_count(time.monotonic())}"
                    f" fresh samples; {self.estimator.last_reason or 'too few'})")
        return (f"board at ({est['grab'][0]:+.2f}, {est['grab'][1]:+.2f}, "
                f"{est['grab'][2]:+.2f}) NED, normal "
                f"{math.degrees(est['bearing']):+.0f} deg, {est['samples']} samples")

    def window_summary(self):
        """Overrides WindowScan's, so the sweep logs talk about the board."""
        if not self.window_ever_seen:
            return f"no /{self.DETECT_TOPIC} messages -- is ring_detect running?"
        if time.monotonic() - self.window_msg_time > self.DETECT_MAX_AGE:
            return "ring_detect has gone quiet"
        return "BOARD IN SIGHT" if self.window_flag else "no board"

    # ------------------------------------------------------------- geometry

    def _standoff_point(self, est, standoff):
        """The point `standoff` metres in front of the board, on the normal."""
        return est['grab'] + standoff * est['normal']

    def _depth_now(self, est):
        """How far in front of the board plane the vehicle is, right now.

        Measured along the normal, not as a straight-line distance, so being
        off to one side does not read as being further away.
        """
        lp = self.local_position
        if lp is None:
            return None
        return float(np.dot(np.array([lp.x, lp.y, 0.0])
                            - np.array([est['grab'][0], est['grab'][1], 0.0]),
                            est['normal']))

    def _board_heading(self, est):
        """The NED heading that points the nose AT the board."""
        return math.atan2(-est['normal'][1], -est['normal'][0])

    def _target_altitude(self, est):
        """Altitude above home of the grab marker, clamped to the flight band.

        The vision supplies the vertical OFFSET to the marker; the vehicle's
        own z -- which is the TFmini through EKF2 -- supplies where it is
        measuring from. So this is a lidar-referenced altitude with a vision
        correction on top, not a height guessed from a camera.
        """
        altitude = self.home_z - float(est['grab'][2])
        clamped = min(max(altitude, self.MIN_ALTITUDE), self.MAX_ALTITUDE)
        if abs(clamped - altitude) > 0.01:
            self.get_logger().warning(
                f"Grab marker is at {altitude:.2f} m above the pad, outside "
                f"[{self.MIN_ALTITUDE:.2f}, {self.MAX_ALTITUDE:.2f}]; flying "
                f"{clamped:.2f} m instead. The run-in will be "
                f"{abs(clamped - altitude):.2f} m off vertically.",
                throttle_duration_sec=5.0)
        return clamped

    def _set_target(self, x, y, altitude=None):
        """Point the inherited ramps at an NED point and an altitude.

        This is the whole of "how a setpoint is given" here. How it is walked
        -- the speed, the leash to the measured position, the fact that what
        reaches PX4 is an absolute position -- is the base class's.
        """
        self.move_target_x = float(x)
        self.move_target_y = float(y)
        self.moving = True
        if altitude is not None:
            self.commanded_altitude = float(altitude)
            self.target_z = self.home_z - float(altitude)

    def _aim_yaw_at(self, heading):
        self.yaw_remaining = wrap_pi(self._clamp_to_cone(heading) - self.yaw_setpoint)

    def _clamp_to_cone(self, heading):
        """Pull an absolute heading back inside the yaw cone.

        Every yaw this node commands goes through here, so whatever goes wrong
        upstream, the nose cannot end up more than yaw_cone_deg from the
        heading the aircraft took off on.
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
            f"commanding {math.degrees(clamped):+.0f} deg instead.",
            throttle_duration_sec=2.0)
        return clamped

    def _heading_error(self, heading):
        lp = self.local_position
        if lp is None:
            return math.pi
        return abs(wrap_pi(heading - lp.heading))

    def _position(self):
        lp = self.local_position
        return None if lp is None else np.array([lp.x, lp.y, lp.z])

    # ---------------------------------------------------------- EKF2 resets

    def _on_heading_reset(self, delta):
        """Turn everything this node placed in NED with the frame."""
        super()._on_heading_reset(delta)
        lp = self.local_position
        pivot = (0.0, 0.0) if lp is None else (lp.x, lp.y)
        self.estimator.rotate_frame(delta, pivot)
        for name in ('fine_exit', 'grab_point', 'grab_start', 'backout_point'):
            point = getattr(self, name)
            if point is None:
                continue
            c, s = math.cos(delta), math.sin(delta)
            dx, dy = point[0] - pivot[0], point[1] - pivot[1]
            setattr(self, name, np.array([pivot[0] + c * dx - s * dy,
                                          pivot[1] + s * dx + c * dy,
                                          point[2]]))
        if self.grab_axis is not None:
            c, s = math.cos(delta), math.sin(delta)
            ax, ay = self.grab_axis[0], self.grab_axis[1]
            self.grab_axis = np.array([c * ax - s * ay, s * ax + c * ay,
                                       self.grab_axis[2]])
        if self.grab_heading is not None:
            self.grab_heading = wrap_pi(self.grab_heading + delta)

    # ------------------------------------------------------- state machine

    def _clock_stages(self):
        return super()._clock_stages() + (self.COARSE, self.FINE, self.GRAB,
                                          self.BACKOUT)

    def timer_callback(self):
        if self.MODE == 'pose':
            # No heartbeat, no setpoints, no arming. Just the readout.
            self._log_pose_mode()
            return

        if self._check_flight_clock():
            return

        stages = (self.COARSE, self.FINE, self.GRAB, self.BACKOUT)
        if self.current_stage not in stages:
            super().timer_callback()
            return

        # Same preamble every stage handler in this tree runs.
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

        {
            self.COARSE: self._handle_coarse,
            self.FINE: self._handle_fine,
            self.GRAB: self._handle_grab,
            self.BACKOUT: self._handle_backout,
        }[self.current_stage]()

    # --------------------------------------------------------- mode:=pose

    def _log_pose_mode(self):
        pose = self.board_frame_pose
        if pose is None or time.monotonic() - pose['t'] > 1.0:
            self.get_logger().info(
                f"No board. {self.ring_info or 'nothing on /ring_info yet'}",
                throttle_duration_sec=1.0)
            return

        jitter = self._board_pose_jitter()
        if jitter is None:
            self.get_logger().info("Board seen, gathering samples...",
                                   throttle_duration_sec=1.0)
            return

        self.get_logger().info(
            "DRONE IN BOARD FRAME  "
            f"depth {jitter['depth'][0]:+.3f} +/-{jitter['depth'][1] * 100:.1f} cm | "
            f"right {jitter['right'][0]:+.3f} +/-{jitter['right'][1] * 100:.1f} cm | "
            f"up {jitter['up'][0]:+.3f} +/-{jitter['up'][1] * 100:.1f} cm | "
            f"yaw {jitter['yaw_deg'][0]:+.1f} +/-{jitter['yaw_deg'][1]:.1f} deg | "
            f"slots {pose['slots']} n={pose['n_markers']}"
            f"{' AMBIGUOUS' if pose['ambiguous'] else ''} "
            f"({jitter['n']} samples/2 s)",
            throttle_duration_sec=0.5)

    # ------------------------------------------------------------- the lock

    def _handle_lock(self):
        """WindowScan parks here. Wait for a usable pose, then go and get it."""
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        est = self._track_pose_health()
        if est is not None and self.hold_xy:
            self._begin_coarse(est)
            return

        if self._in_stage_for() > self.LOCK_TIMEOUT:
            self._abandon(
                f"locked on the board but no usable pose after "
                f"{self.LOCK_TIMEOUT:.0f} s ({self.pose_summary()})")
            return

        self.get_logger().info(
            f"Locked, building a pose: {self.pose_summary()}"
            + ("" if self.hold_xy else " (waiting for a flow-healthy x/y latch)"),
            throttle_duration_sec=1.0)

    # ----------------------------------------------------------- the course

    def _begin_coarse(self, est):
        """Freeze the standoff, then fly to it. See the header on never
        flying backwards."""
        depth = self._depth_now(est)
        if depth is None:
            return
        standoff = min(self.FINE_STANDOFF, depth)
        if standoff < self.MIN_STANDOFF:
            self.get_logger().warning(
                f"Already {depth:.2f} m from the board, inside min_standoff "
                f"{self.MIN_STANDOFF:.2f} m -- backing out to it. This is the "
                "one case where the course does fly backwards; there is not "
                "enough board left in frame to align any closer.")
            standoff = self.MIN_STANDOFF
        self.approach_standoff = standoff
        self.MOVE_SPEED = self.APPROACH_SPEED
        self.coarse_in_band_since = None
        self._enter_stage(self.COARSE)

        target = self._standoff_point(est, standoff)
        self.get_logger().warning(
            f"COURSE: board normal {math.degrees(est['bearing']):+.0f} deg, "
            f"depth now {depth:.2f} m -> lining up at {standoff:.2f} m "
            f"({'closing in' if depth > standoff + 0.05 else 'holding depth, aligning only'}). "
            f"Target ({target[0]:+.2f}, {target[1]:+.2f}) NED at "
            f"{self._target_altitude(est):.2f} m.")

    def _handle_coarse(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        est = self._track_pose_health()
        if est is None:
            if self._pose_lost_for() > self.POSE_LOST_TIMEOUT:
                self._abandon(f"lost the board during the course "
                              f"({self.pose_summary()})")
            else:
                self.get_logger().warning(
                    f"No pose for {self._pose_lost_for():.1f} s; holding.",
                    throttle_duration_sec=1.0)
            return
        if self._in_stage_for() > self.COARSE_TIMEOUT:
            self._abandon(f"course did not converge in {self.COARSE_TIMEOUT:.0f} s")
            return

        # Recomputed every tick, not latched: the estimate keeps refining as
        # the aircraft gets closer, and a target flown to once and forgotten
        # would be the estimate from the worst viewing angle of the flight.
        heading = self._board_heading(est)
        target = self._standoff_point(est, self.approach_standoff)
        altitude = self._target_altitude(est)
        self._aim_yaw_at(heading)
        self._set_target(target[0], target[1], altitude)

        pos = self._position()
        if pos is None:
            return
        error = math.hypot(target[0] - pos[0], target[1] - pos[1])
        alt_error = abs(altitude - (self.home_z - pos[2]))
        total = math.hypot(error, alt_error)

        if total <= self.COARSE_TOLERANCE and self._heading_error(heading) <= self.YAW_TOLERANCE:
            if self.coarse_in_band_since is None:
                self.coarse_in_band_since = time.monotonic()
            elif (time.monotonic() - self.coarse_in_band_since
                    >= self.COARSE_SETTLE_SECONDS):
                self._begin_fine(est)
                return
        else:
            self.coarse_in_band_since = None

        self.get_logger().info(
            f"Course: {total:.2f} m to the standoff point "
            f"(want {self.COARSE_TOLERANCE:.2f}), yaw off "
            f"{math.degrees(self._heading_error(heading)):.0f} deg.",
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------- the fine

    def _begin_fine(self, est):
        self.align_in_band_since = None
        self._enter_stage(self.FINE)
        self.get_logger().warning(
            f"FINE: proportional alignment at {self.approach_standoff:.2f} m, "
            f"gain {self.ALIGN_GAIN:.2f}, until the error is under "
            f"{self.ALIGN_TOLERANCE * 100:.0f} cm and the yaw under "
            f"{math.degrees(self.YAW_TOLERANCE):.0f} deg for "
            f"{self.ALIGN_SETTLE_SECONDS:.1f} s.")

    def _handle_fine(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        est = self._track_pose_health()
        if est is None:
            if self._pose_lost_for() > self.POSE_LOST_TIMEOUT:
                self._abandon(f"lost the board during the fine alignment "
                              f"({self.pose_summary()})")
            else:
                self.get_logger().warning(
                    f"No pose for {self._pose_lost_for():.1f} s; holding.",
                    throttle_duration_sec=1.0)
            return
        if self._in_stage_for() > self.FINE_TIMEOUT:
            self._abandon(f"fine alignment did not converge in "
                          f"{self.FINE_TIMEOUT:.0f} s")
            return
        if not self.hold_xy:
            self.get_logger().warning(
                "Flow unhealthy: cannot correct, holding.",
                throttle_duration_sec=1.0)
            return

        ideal = self._standoff_point(est, self.approach_standoff)
        altitude = self._target_altitude(est)
        heading = self._board_heading(est)
        pos = self._position()
        if pos is None:
            return
        current_altitude = self.home_z - pos[2]

        err_x = ideal[0] - pos[0]
        err_y = ideal[1] - pos[1]
        err_alt = altitude - current_altitude
        error = math.sqrt(err_x ** 2 + err_y ** 2 + err_alt ** 2)
        yaw_error = self._heading_error(heading)

        # Arrival first, so a vehicle that is already there does not get one
        # more nudge before being allowed to settle.
        if error <= self.ALIGN_TOLERANCE and yaw_error <= self.YAW_TOLERANCE:
            if self.align_in_band_since is None:
                self.align_in_band_since = time.monotonic()
            elif (time.monotonic() - self.align_in_band_since
                    >= self.ALIGN_SETTLE_SECONDS):
                self._begin_grab(est, error)
                return
        else:
            self.align_in_band_since = None

        # THE PROPORTIONAL POSE SETPOINT. A fraction of the measured error,
        # as an absolute NED point, handed to the same carrot every other
        # stage uses.
        self._aim_yaw_at(heading)
        self._set_target(pos[0] + self.ALIGN_GAIN * err_x,
                         pos[1] + self.ALIGN_GAIN * err_y,
                         current_altitude + self.ALIGN_GAIN * err_alt)

        self.get_logger().info(
            f"Fine: err {error:.3f} m (want {self.ALIGN_TOLERANCE:.2f}), yaw "
            f"{math.degrees(yaw_error):.1f} deg, depth "
            f"{self._depth_now(est):.2f} m.",
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------- the grab

    def _begin_grab(self, est, error):
        """Freeze the target and commit.

        Everything from here is open loop. At 20 cm past the board plane the
        markers are long out of frame, so there is nothing left to correct
        against and a target that kept updating would be updating on noise.
        """
        pos = self._position()
        if pos is None:
            return
        self.fine_exit = pos.copy()
        self.fine_exit_altitude = self.home_z - pos[2]

        # Along the normal, AWAY from the drone: through the ring.
        self.grab_point = est['grab'] - self.GRAB_BEHIND * est['normal']
        self.grab_altitude = self._target_altitude(est)
        self.grab_heading = self._clamp_to_cone(self._board_heading(est))
        self.grab_axis = -est['normal']
        self.grab_start = pos.copy()
        self.grab_arrived_at = None
        self.MOVE_SPEED = self.GRAB_SPEED
        self._enter_stage(self.GRAB)

        run_in = float(np.linalg.norm(self.grab_point[:2] - pos[:2]))
        self.get_logger().warning(
            f"GRAB: aligned to {error * 100:.0f} cm. Committing to a "
            f"{run_in:.2f} m run-in, open loop, at {self.GRAB_SPEED:.2f} m/s, "
            f"to {self.GRAB_BEHIND:.2f} m PAST the grab marker "
            f"({self.grab_point[0]:+.2f}, {self.grab_point[1]:+.2f}) NED at "
            f"{self.grab_altitude:.2f} m, heading "
            f"{math.degrees(self.grab_heading):+.0f} deg.")

    def _distance_along_run_in(self):
        pos = self._position()
        if pos is None or self.grab_start is None:
            return 0.0
        return float(np.dot(pos[:2] - self.grab_start[:2], self.grab_axis[:2]))

    def _handle_grab(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        # No pose gate here, and that is deliberate: the board being invisible
        # is the EXPECTED state at this range, not a fault.
        self.yaw_remaining = wrap_pi(self.grab_heading - self.yaw_setpoint)
        self._set_target(self.grab_point[0], self.grab_point[1], self.grab_altitude)

        pos = self._position()
        if pos is None:
            return
        remaining = float(np.linalg.norm(self.grab_point[:2] - pos[:2]))
        travelled = self._distance_along_run_in()
        total = float(np.linalg.norm(self.grab_point[:2] - self.grab_start[:2]))

        arrived = remaining <= self.GRAB_TOLERANCE or travelled >= total
        if arrived:
            if self.grab_arrived_at is None:
                self.grab_arrived_at = time.monotonic()
                self.get_logger().warning(
                    f"At the grab point ({remaining:.2f} m residual). Holding "
                    f"{self.GRAB_HOLD_SECONDS:.1f} s.")
            elif time.monotonic() - self.grab_arrived_at >= self.GRAB_HOLD_SECONDS:
                self.outcome = 'grabbed'
                self._begin_backout()
            return

        if self._in_stage_for() > self.GRAB_TIMEOUT:
            # Do NOT press on. Something is holding the aircraft back and the
            # only thing in front of it is the board.
            self.get_logger().error(
                f"Run-in did not finish in {self.GRAB_TIMEOUT:.0f} s "
                f"({remaining:.2f} m short). Backing out.")
            self.outcome = 'run-in timed out'
            self._begin_backout()
            return

        self.get_logger().info(
            f"Grab: {travelled:.2f} / {total:.2f} m in, {remaining:.2f} m to go.",
            throttle_duration_sec=0.5)

    # ---------------------------------------------------------- coming back

    def _begin_backout(self):
        """Straight back out the way we came in, nose still on the board."""
        target = self.fine_exit.copy()
        if self.BACKOUT_EXTRA > 0.0:
            target = target - self.BACKOUT_EXTRA * self.grab_axis
        self.backout_point = target
        self.MOVE_SPEED = self.BACKOUT_SPEED
        self._enter_stage(self.BACKOUT)
        self.get_logger().warning(
            f"BACKOUT: {self.outcome}. Retreating to "
            f"({target[0]:+.2f}, {target[1]:+.2f}) NED at "
            f"{self.fine_exit_altitude:.2f} m, then landing. The nose stays on "
            "the board the whole way.")

    def _handle_backout(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        self.yaw_remaining = wrap_pi(self.grab_heading - self.yaw_setpoint)
        self._set_target(self.backout_point[0], self.backout_point[1],
                         self.fine_exit_altitude)

        pos = self._position()
        if pos is None:
            return
        remaining = float(np.linalg.norm(self.backout_point[:2] - pos[:2]))
        if remaining <= self.BACKOUT_TOLERANCE:
            self._begin_landing(f"clear of the board ({self.outcome})")
            return
        if self._in_stage_for() > self.BACKOUT_TIMEOUT:
            self._begin_landing(
                f"backout timed out {remaining:.2f} m short; landing from here")
            return

        self.get_logger().info(f"Backing out: {remaining:.2f} m to go.",
                               throttle_duration_sec=1.0)

    # ------------------------------------------------------------- giving up

    def _abandon(self, reason):
        self.outcome = f"abandoned: {reason}"
        self.get_logger().error(f"Abandoning the grab -- {reason}. Landing.")
        self._begin_landing(reason)

    # --------------------------------------------------------------- status

    def publish_status(self):
        """stage|armed|altitude|flow|detail -- the format the LCD node reads."""
        stages = (self.COARSE, self.FINE, self.GRAB, self.BACKOUT)
        if self.current_stage not in stages:
            super().publish_status()
            return

        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        est = self.ring_estimate()

        if self.current_stage == self.COARSE:
            detail = f"crs{self.approach_standoff:.1f}"
        elif self.current_stage == self.FINE:
            detail = ('fine--' if est is None
                      else f"fin{self.estimator.fresh_count(time.monotonic())}")
        elif self.current_stage == self.GRAB:
            pos = self._position()
            detail = ('grab' if pos is None else
                      f"grb{float(np.linalg.norm(self.grab_point[:2] - pos[:2])):.2f}")
        else:
            detail = 'back'

        msg = String()
        msg.data = "|".join([
            self.current_stage,
            'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('FLO' if self.flow_is_healthy() else '---'),
            detail,
        ])
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info(
            f"Ring grab finished: {self.outcome}. "
            f"{self.estimator.accepted_total} samples accepted, "
            f"{self.estimator.gated_total} gated.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RingGrab()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
