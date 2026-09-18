"""
Takeoff -> find a horizontal bar -> measure how high it is -> climb over it
(or drop under it) -> fly across -> land. ARK Flow localisation.

This is the obstacle-course bar, flown as its own mission. It is deliberately
NOT bolted onto the end of window_traverse yet: one obstacle at a time, each
with its own launch file, is how the window got debugged and it is how this
one will be. Joining them is a later, smaller job -- the stages here are
written so the whole sequence can become a sub-mission of a longer flight
without being rewritten.

    arm -> sit on the ground -> climb -> hold -> SEARCH (stare ahead for the
    bar) -> LOCK (stop, measure it, decide the crossing) -> SET (climb or
    descend to the crossing height, still holding position) -> CROSS (fly
    across the bar, committed and blind) -> CLEAR -> land.

    q -> abort into a controlled descent.   k -> force-disarm.

BLIND OR MEASURED
    assume_bar_height > 0 skips SEARCH and LOCK and flies the geometry from
    the parameters instead: the rules publish the bar heights (1200 / 1600 /
    1980 mm red, 400 / 800 / 1200 blue), so there is nothing for the camera
    to discover, and making a vision measurement the gate on a number given
    in advance only adds a way to fail. The detector stays up and logs a
    CROSS-CHECK line against the assumed height; the flight does not wait on
    it. assumed_estimate() builds the same dict a real measurement produces,
    so there is exactly one crossing implementation either way.

    The two are not equally safe. Assuming wrong going OVER costs altitude;
    assuming wrong going UNDER hits the bar. Blind is the right default for
    the red bar and a deliberate choice for the blue ones.

ONE NODE FOR BOTH BARS
    The red bar is flown OVER and the blue bars are flown UNDER, and that is
    the only difference between the two missions. Same detector, same
    estimator, same approach, same commit, same blind crossing; the sign of
    the clearance changes and so does the colour parameter. Two nodes would be
    two copies of all of that, drifting apart the first time one of them got a
    fix -- so this is one node with pass_mode:=over|under, and the blue
    mission is this launch file with three arguments changed.

WHY THE FLOOR DOES NOT FOOL IT
    The arena floor is red, in strips, with textured matting between them.
    bar_detect cannot tell a red floor strip from a red bar from one image --
    both are long red horizontal regions -- and it says so. What settles it is
    here, and it is height:

        every accepted sample is placed in NED using the vehicle attitude and
        position, exactly as window_traverse places window corners, and any
        bar whose centre is less than min_bar_height above the ARMING PLANE
        is refused outright.

    The floor is at zero height by construction. It cannot pass. A bar at
    1.2 m -- the lowest setting in the rules -- clears the 0.60 m default by
    twice over, so the test is not a close-run thing at either end.

    Two more gates back it up. A bar is HORIZONTAL: the two endpoints must sit
    within max_slope_deg of level, which a floor strip seen in perspective
    does not (its near end is metres closer than its far end, so in NED it
    rises steeply away from the camera). And it has a plausible LENGTH,
    between min_length and max_length.

THE CROSSING
    The bar is a line, not a point, so what is estimated is a line: a centre,
    a horizontal direction along it, and a height. The crossing runs
    PERPENDICULAR to that line, through the centre, which is the shortest way
    across and the one that keeps the aircraft furthest from both ends.

    The crossing height is solved for the AIRFRAME, not the origin, the same
    way window_traverse solves the window:

        over:   the landing gear must clear the top of the bar
        under:  the highest point of the aircraft must clear the bottom

    so the commanded altitude carries body_below or body_above plus
    cross_clearance, and is then clamped into [min_altitude, max_altitude].
    If the clamp moves it by more than a few centimetres the aperture the
    rules describe is not reachable by this aircraft in this arena and the
    attempt is abandoned rather than flown at the ceiling.

    Like the window traverse, the crossing COMMITS: at the start of CROSS the
    target is frozen and the camera stops steering. It has to be. Once the
    aircraft is above a bar the bar is below the field of view, and a
    controller still chasing a detection it can no longer see is a controller
    chasing noise.

THE HEIGHTS IN THE RULES
    1200, 1600 and 1980 mm, higher scoring more. Nothing here is tuned to a
    specific one -- the bar is measured, not assumed -- but max_altitude has
    to be able to contain the answer. At 1980 mm the crossing altitude is
    about 1.98 + 0.05 + 0.20 + 0.12 = 2.35 m, so max_altitude must be at
    least 2.6 to leave the clamp any room. The default here is 3.0.
"""

import math
import time

import numpy as np

import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleStatus
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, Float32MultiArray, String

from drone_testing.offboard_sequence import OffboardSequence, spin_node, wrap_pi
from drone_testing.window_traverse import quat_rotate, rpy_to_matrix_frd


class BarEstimator:
    """A stream of camera-frame bar endpoints -> one NED line.

    Same shape as WindowEstimator and for the same reason: the rejection
    logic is the part that decides where the aircraft flies, so it is kept
    free of ROS and of the flight node and can be exercised on the bench.

    add() returns (accepted, reason).
    """

    def __init__(self, depth_min, depth_max, min_length, max_length,
                 max_slope, min_height, max_height, buffer_seconds, buffer_max,
                 min_samples, gate_metres, gate_yaw, gate_reset_count):
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.min_length = min_length
        self.max_length = max_length
        self.max_slope = max_slope          # rad, endpoints off level
        self.min_height = min_height        # m above the arming plane
        self.max_height = max_height
        self.buffer_seconds = buffer_seconds
        self.buffer_max = buffer_max
        self.min_samples = min_samples
        self.gate_metres = gate_metres
        self.gate_yaw = gate_yaw
        self.gate_reset_count = gate_reset_count

        import collections
        import threading
        # RE-entrant: _add() consults estimate() for the innovation gate while
        # already holding it, and a plain Lock deadlocks the callback thread
        # on the first sample.
        self._lock = threading.RLock()
        self.samples = collections.deque(maxlen=buffer_max)
        self.rejections = {}
        self.accepted_total = 0
        self.consecutive_gated = 0
        self.last_reason = ''

    # ------------------------------------------------------------ ingestion

    def add(self, geometry, q_att, p_ned, r_cam, t_cam, home_z, now):
        with self._lock:
            return self._add(geometry, q_att, p_ned, r_cam, t_cam, home_z, now)

    def _add(self, geometry, q_att, p_ned, r_cam, t_cam, home_z, now):
        ends_cam = []
        for depth, az_deg, el_deg in geometry[:2]:
            if not np.isfinite(depth) or depth <= 0.0:
                return self._reject('endpoint depth missing')
            if depth < self.depth_min or depth > self.depth_max:
                return self._reject('endpoint depth out of range')
            az = math.radians(float(az_deg))
            el = math.radians(float(el_deg))
            ends_cam.append([float(depth),
                             float(depth) * math.tan(az),
                             -float(depth) * math.tan(el)])
        ends_cam = np.array(ends_cam)

        # Camera -> body FRD -> NED, identical to window_traverse.
        ends_body = ends_cam @ r_cam.T + t_cam
        ends_ned = np.array([quat_rotate(q_att, p) for p in ends_body]) + p_ned

        centre = ends_ned.mean(axis=0)
        span = ends_ned[1] - ends_ned[0]
        length = float(np.linalg.norm(span))
        if not (self.min_length <= length <= self.max_length):
            return self._reject('implausible bar length')

        # A bar is horizontal. This is the gate that throws out a floor strip
        # that got through the shape test: seen in perspective its near end is
        # metres closer than its far end, so once both are in NED the "bar"
        # climbs away from the camera at a slope no real bar has.
        horizontal = math.hypot(span[0], span[1])
        if horizontal < 1e-6:
            return self._reject('degenerate bar')
        slope = abs(math.atan2(span[2], horizontal))
        if slope > self.max_slope:
            return self._reject('bar is not level')

        # THE test. Height above the arming plane, positive up.
        height = home_z - centre[2]
        if height < self.min_height:
            return self._reject('too low to be the bar (floor?)')
        if height > self.max_height:
            return self._reject('too high to be the bar')

        direction = np.array([span[0], span[1], 0.0]) / horizontal

        est = self.estimate(now)
        if est is not None:
            if float(np.linalg.norm(centre - est['centre'])) > self.gate_metres:
                return self._gated('centre jumped')
            # The direction is a LINE, not an arrow: a bar measured left-to-
            # right one frame and right-to-left the next is the same bar, so
            # the gate is on the undirected angle between them.
            cos = abs(float(np.dot(direction[:2], est['direction'][:2])))
            if math.acos(min(1.0, cos)) > self.gate_yaw:
                return self._gated('bar swung')

        self.consecutive_gated = 0
        self.accepted_total += 1
        self.last_reason = ''
        # Store the direction with a consistent sign so the median over the
        # buffer is not the average of a vector and its negation.
        if direction[0] < 0.0 or (abs(direction[0]) < 1e-9 and direction[1] < 0.0):
            direction = -direction
        self.samples.append({
            't': now,
            'centre': centre,
            'direction': direction,
            'length': length,
            'height': height,
        })
        return True, ''

    def _reject(self, reason):
        self.rejections[reason] = self.rejections.get(reason, 0) + 1
        self.last_reason = reason
        return False, reason

    def _gated(self, reason):
        self.consecutive_gated += 1
        if self.consecutive_gated >= self.gate_reset_count:
            self.samples.clear()
            self.consecutive_gated = 0
            return self._reject(reason + ' -- estimate discarded, rebuilding')
        return self._reject(reason)

    # -------------------------------------------------------------- output

    def _fresh(self, now):
        cutoff = now - self.buffer_seconds
        return [s for s in self.samples if s['t'] >= cutoff]

    def estimate(self, now):
        with self._lock:
            fresh = self._fresh(now)
            if len(fresh) < self.min_samples:
                return None
            centre = np.median(np.array([s['centre'] for s in fresh]), axis=0)
            direction = np.median(np.array([s['direction'] for s in fresh]), axis=0)
            norm = float(np.linalg.norm(direction[:2]))
            if norm < 1e-6:
                return None
            direction = np.array([direction[0] / norm, direction[1] / norm, 0.0])
            return {
                'centre': centre,
                'direction': direction,
                'length': float(np.median([s['length'] for s in fresh])),
                'height': float(np.median([s['height'] for s in fresh])),
                'samples': len(fresh),
                'age': now - fresh[-1]['t'],
            }

    def fresh_count(self, now):
        with self._lock:
            return len(self._fresh(now))

    def rejection_summary(self, limit=3):
        with self._lock:
            if not self.rejections:
                return 'none'
            worst = sorted(self.rejections.items(), key=lambda kv: -kv[1])[:limit]
        return ', '.join(f"{name} x{count}" for name, count in worst)


class BarCross(OffboardSequence):

    SEARCH = "SEARCH"
    LOCK = "LOCK"
    SET = "SET"
    CROSS = "CROSS"
    CLEAR = "CLEAR"

    BAR_STAGES = (SEARCH, LOCK, SET, CROSS, CLEAR)

    # ---- the mission ------------------------------------------------------
    PASS_MODE = 'over'          # 'over' (red bar) or 'under' (blue bars)
    STANDOFF_DISTANCE = 1.20    # m short of the bar the crossing starts from
    EXIT_DISTANCE = 1.20        # m beyond it the crossing ends
    CROSS_CLEARANCE = 0.20      # m of air wanted between the airframe and the
                                # bar. Bigger than the window's because
                                # nothing here is constrained on the other
                                # side -- over a bar there is only sky, so
                                # clearance is free and there is no reason to
                                # be mean with it.
    BAR_RADIUS = 0.05           # m. Half the bar's thickness, added to the
                                # clearance. The estimate measures the
                                # CENTRELINE of what the mask saw, so the
                                # surface is this much nearer.

    # ---- the airframe (same numbers as window_traverse) -------------------
    GEAR_BELOW_CAMERA = 0.120
    DRONE_HEIGHT = 0.260
    DRONE_WIDTH = 0.260

    APPROACH_SPEED = 0.30
    CROSS_SPEED = 0.40

    SET_ALT_TOLERANCE = 0.08    # m. Held before the crossing starts.
    SET_SETTLE_SECONDS = 1.5
    SET_TIMEOUT = 40.0
    ALIGN_CROSS_TOLERANCE = 0.20    # m off the crossing line. Looser than the
                                    # window's 0.06: a bar has no jambs, so
                                    # being off to one side costs nothing as
                                    # long as it is not off an END.
    ALIGN_ALONG_TOLERANCE = 0.30
    ALIGN_YAW_TOLERANCE = math.radians(12.0)
    END_MARGIN = 0.35           # m of bar wanted either side of the crossing
                                # point. Crossing near an end is how a prop
                                # finds the upright the bar is hanging from.

    CROSS_TIMEOUT = 25.0
    CLEAR_SECONDS = 4.0
    LOCK_TIMEOUT = 30.0

    YAW_CONE_DEG = 50.0

    # ---- the estimate -----------------------------------------------------
    GEOMETRY_TOPIC = 'bar_geometry'
    DETECT_TOPIC = 'bar_detected'
    DETECT_SECONDS = 0.4
    DETECT_MAX_AGE = 1.0
    ATTITUDE_MAX_HZ = 30.0
    POSE_MAX_AGE = 1.5
    POSE_LOST_TIMEOUT = 8.0

    DEPTH_MIN = 0.35
    DEPTH_MAX = 9.00
    MIN_BAR_HEIGHT = 0.60       # m above the arming plane. THE floor gate.
    MAX_BAR_HEIGHT = 2.60
    MIN_LENGTH = 0.60
    MAX_LENGTH = 6.00
    MAX_SLOPE_DEG = 20.0
    BUFFER_SECONDS = 2.5
    BUFFER_MAX = 60
    MIN_SAMPLES = 6
    GATE_METRES = 1.00
    GATE_YAW_DEG = 35.0
    GATE_RESET_COUNT = 25

    TAKEOFF_ALTITUDE = 1.20
    MAX_ALTITUDE = 3.00
    FLIGHT_SECONDS = 120.0

    def __init__(self, node_name='bar_cross'):
        super().__init__(node_name)

        mode = str(self.declare_parameter('pass_mode', self.PASS_MODE).value).strip().lower()
        if mode not in ('over', 'under'):
            self.get_logger().warning(
                f"pass_mode '{mode}' is not 'over' or 'under'; using "
                f"'{self.PASS_MODE}'.")
            mode = self.PASS_MODE
        self.pass_mode = mode

        self.STANDOFF_DISTANCE = float(self._declare_number(
            'standoff_distance', self.STANDOFF_DISTANCE))
        self.EXIT_DISTANCE = float(self._declare_number(
            'exit_distance', self.EXIT_DISTANCE))
        self.CROSS_CLEARANCE = float(self._declare_number(
            'cross_clearance', self.CROSS_CLEARANCE))
        self.BAR_RADIUS = float(self._declare_number('bar_radius', self.BAR_RADIUS))
        self.GEAR_BELOW_CAMERA = float(self._declare_number(
            'gear_below_camera', self.GEAR_BELOW_CAMERA))
        self.DRONE_HEIGHT = float(self._declare_number('drone_height', self.DRONE_HEIGHT))
        self.DRONE_WIDTH = float(self._declare_number('drone_width', self.DRONE_WIDTH))
        self.APPROACH_SPEED = float(self._declare_number(
            'approach_speed', self.APPROACH_SPEED))
        self.CROSS_SPEED = float(self._declare_number('cross_speed', self.CROSS_SPEED))
        self.SET_ALT_TOLERANCE = float(self._declare_number(
            'set_alt_tolerance', self.SET_ALT_TOLERANCE))
        self.SET_SETTLE_SECONDS = float(self._declare_number(
            'set_settle_seconds', self.SET_SETTLE_SECONDS))
        self.ALIGN_CROSS_TOLERANCE = float(self._declare_number(
            'align_cross_tolerance', self.ALIGN_CROSS_TOLERANCE))
        self.ALIGN_ALONG_TOLERANCE = float(self._declare_number(
            'align_along_tolerance', self.ALIGN_ALONG_TOLERANCE))
        self.END_MARGIN = float(self._declare_number('end_margin', self.END_MARGIN))
        self.DETECT_SECONDS = float(self._declare_number(
            'detect_seconds', self.DETECT_SECONDS))
        self.POSE_LOST_TIMEOUT = float(self._declare_number(
            'pose_lost_timeout', self.POSE_LOST_TIMEOUT))
        self.FLIGHT_SECONDS = float(self._declare_number(
            'flight_seconds', self.FLIGHT_SECONDS))
        self.MIN_SAMPLES = int(self._declare_number('pose_min_samples', self.MIN_SAMPLES))
        self.YAW_CONE = math.radians(float(self._declare_number(
            'yaw_cone_deg', self.YAW_CONE_DEG)))

        # ---- flying it blind ----
        # The rules give the bar's height: 1200, 1600 or 1980 mm for the red
        # one, 400, 800 or 1200 for the blue. When the setting is known there
        # is nothing for the camera to discover, and making a vision
        # measurement the gate on a number that was published in advance adds
        # a way to fail without adding anything.
        #
        # assume_bar_height > 0 therefore skips SEARCH and LOCK entirely and
        # flies the geometry straight from these three numbers. The detector
        # stays up and keeps reporting, but as a CROSS-CHECK in the log rather
        # than as something the flight waits on.
        #
        # Worth being clear about the asymmetry, because it decides which
        # obstacle should use this. Going OVER, assuming wrong by 30 cm means
        # flying 30 cm higher than necessary. Going UNDER, it means hitting
        # the bar. Blind is the right default for the red bar and a deliberate
        # choice for the blue ones.
        self.ASSUME_BAR_HEIGHT = float(self._declare_number('assume_bar_height', 0.0))
        self.ASSUME_BAR_DISTANCE = float(self._declare_number(
            'assume_bar_distance', 3.0))
        self.ASSUME_BAR_LENGTH = float(self._declare_number('assume_bar_length', 3.0))
        self.flying_blind = self.ASSUME_BAR_HEIGHT > 0.0

        cam_x = float(self._declare_number('cam_x', 0.0))
        cam_y = float(self._declare_number('cam_y', 0.0))
        cam_z = float(self._declare_number('cam_z', 0.0))
        cam_roll = float(self._declare_number('cam_roll', 0.0))
        cam_pitch = float(self._declare_number('cam_pitch', 0.0))
        cam_yaw = float(self._declare_number('cam_yaw', 0.0))
        self.r_cam = rpy_to_matrix_frd(cam_roll, cam_pitch, cam_yaw)
        self.t_cam = np.array([cam_x, -cam_y, -cam_z])
        self.body_below = self.GEAR_BELOW_CAMERA - cam_z
        self.body_above = self.DRONE_HEIGHT - self.body_below

        self.estimator = BarEstimator(
            depth_min=float(self._declare_number('depth_min', self.DEPTH_MIN)),
            depth_max=float(self._declare_number('depth_max', self.DEPTH_MAX)),
            min_length=float(self._declare_number('min_length', self.MIN_LENGTH)),
            max_length=float(self._declare_number('max_length', self.MAX_LENGTH)),
            max_slope=math.radians(float(self._declare_number(
                'max_slope_deg', self.MAX_SLOPE_DEG))),
            min_height=float(self._declare_number('min_bar_height', self.MIN_BAR_HEIGHT)),
            max_height=float(self._declare_number('max_bar_height', self.MAX_BAR_HEIGHT)),
            buffer_seconds=float(self._declare_number('buffer_seconds', self.BUFFER_SECONDS)),
            buffer_max=self.BUFFER_MAX,
            min_samples=self.MIN_SAMPLES,
            gate_metres=float(self._declare_number('gate_metres', self.GATE_METRES)),
            gate_yaw=math.radians(float(self._declare_number(
                'gate_yaw_deg', self.GATE_YAW_DEG))),
            gate_reset_count=int(self._declare_number(
                'gate_reset_count', self.GATE_RESET_COUNT)),
        )

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.attitude = None
        self.attitude_time = None
        self._attitude_min_interval = 1.0 / self.ATTITUDE_MAX_HZ
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self.attitude_callback, sensor_qos,
                                 callback_group=self.sensor_cbg)

        detect_topic = str(self.declare_parameter('detect_topic', self.DETECT_TOPIC).value)
        self.geometry_topic = str(self.declare_parameter(
            'geometry_topic', self.GEOMETRY_TOPIC).value)
        self.create_subscription(Bool, detect_topic, self.bar_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.create_subscription(String, 'bar_info', self.bar_info_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.create_subscription(Float32MultiArray, self.geometry_topic,
                                 self.geometry_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.bar_pose_pub = self.create_publisher(String, 'bar_pose', 10)

        self.bar_flag = False
        self.bar_msg_time = 0.0
        self.bar_true_since = None
        self.bar_info = ''
        self.bar_ever_seen = False
        self.geometry_seen = 0
        self.last_detection = None

        self.pose_ok_since = None
        self.pose_lost_since = None
        self.last_good_est = None
        self.last_good_est_time = 0.0

        self.cross_entry = None
        self.cross_exit = None
        self.cross_heading = None
        self.cross_altitude = None
        self.cross_bar = None
        self.set_in_band_since = None
        self.flight_start = None
        self.outcome = 'not attempted'

        if self.flying_blind:
            self.get_logger().warning(
                f"Bar crossing on ARK FLOW, FLYING BLIND: climb "
                f"{self.TAKEOFF_ALTITUDE:.2f} m, hold {self.HOLD_SECONDS:.0f} s, "
                f"then go {self.pass_mode.upper()} a bar ASSUMED to be "
                f"{self.ASSUME_BAR_HEIGHT:.2f} m high and "
                f"{self.ASSUME_BAR_DISTANCE:.2f} m ahead on the takeoff "
                f"heading, with {self.CROSS_CLEARANCE:.2f} m of clearance. "
                "The camera is NOT steering this: point the aircraft along "
                "the course and measure the distance to the bar before you "
                "arm. bar_detect still runs and still reports, as a "
                "cross-check in the log. Hard limit "
                f"{self.FLIGHT_SECONDS:.0f} s from the start of the climb. "
                "Press q to abort into a descent, k to force-disarm.")
            return

        self.get_logger().warning(
            f"Bar crossing on ARK FLOW: climb {self.TAKEOFF_ALTITUDE:.2f} m, "
            f"hold {self.HOLD_SECONDS:.0f} s, find the {self.pass_mode.upper()} "
            f"bar on the takeoff heading, measure it, then go {self.pass_mode} "
            f"it with {self.CROSS_CLEARANCE:.2f} m of clearance at "
            f"{self.CROSS_SPEED:.2f} m/s. Anything less than "
            f"{self.estimator.min_height:.2f} m above the arming plane is "
            "treated as the floor and refused. Hard limit "
            f"{self.FLIGHT_SECONDS:.0f} s from the start of the climb. "
            "Press q to abort into a descent, k to force-disarm.")

    # ------------------------------------------------------------------ subs

    def attitude_callback(self, msg):
        now = time.monotonic()
        if (self.attitude_time is not None
                and now - self.attitude_time < self._attitude_min_interval):
            return
        self.attitude = msg
        self.attitude_time = now

    def bar_callback(self, msg):
        self.bar_msg_time = time.monotonic()
        self.bar_ever_seen = True
        if msg.data:
            if not self.bar_flag:
                self.bar_true_since = self.bar_msg_time
        else:
            self.bar_true_since = None
        self.bar_flag = msg.data

    def bar_info_callback(self, msg):
        self.bar_info = msg.data

    def bar_is_confirmed(self):
        if not self.bar_flag or self.bar_true_since is None:
            return False
        now = time.monotonic()
        if now - self.bar_msg_time > self.DETECT_MAX_AGE:
            return False
        return now - self.bar_true_since >= self.DETECT_SECONDS

    def geometry_callback(self, msg):
        self.geometry_seen += 1
        data = np.asarray(msg.data, dtype=float)
        if data.size < 9:
            self.get_logger().warning(
                f"/{self.geometry_topic} has {data.size} values, expected at "
                "least 9. Is bar_detect up to date?", throttle_duration_sec=5.0)
            return
        geometry = data[:9].reshape(3, 3)
        truncated = bool(data[9] > 0.5) if data.size >= 12 else False

        self.last_detection = {
            'time': time.monotonic(),
            'truncated': truncated,
            'centre_az': math.radians(float(geometry[2][1])),
            'centre_el': math.radians(float(geometry[2][2])),
        }

        lp = self.local_position
        if lp is None or not lp.xy_valid or not lp.z_valid or self.home_z is None:
            return
        if self.attitude is None:
            self.get_logger().warning(
                "No /fmu/out/vehicle_attitude yet; cannot place the bar.",
                throttle_duration_sec=5.0)
            return
        self.estimator.add(geometry, np.asarray(self.attitude.q, dtype=float),
                           np.array([lp.x, lp.y, lp.z]), self.r_cam, self.t_cam,
                           self.home_z, time.monotonic())

    # -------------------------------------------------------------- estimate

    def bar_estimate(self):
        est = self.estimator.estimate(time.monotonic())
        if est is None or est['age'] > self.POSE_MAX_AGE:
            return None
        return est

    def _track_pose_health(self):
        now = time.monotonic()
        est = self.bar_estimate()
        if est is not None:
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
        est = self.bar_estimate()
        if est is None:
            fresh = self.estimator.fresh_count(time.monotonic())
            if self.geometry_seen == 0:
                if self.count_publishers(self.geometry_topic) == 0:
                    return (f"nothing is publishing {self.geometry_topic} -- "
                            "is bar_detect running?")
                return (f"{self.geometry_topic} has a publisher but has never "
                        "carried a message: bar_detect is not seeing a "
                        "bar-shaped contour. Check the colour and min_area.")
            return (f"no usable bar pose ({fresh}/{self.MIN_SAMPLES} fresh "
                    f"samples, {self.estimator.accepted_total} accepted ever; "
                    f"rejections: {self.estimator.rejection_summary()})")
        return (f"bar at ({est['centre'][0]:+.2f}, {est['centre'][1]:+.2f}), "
                f"{est['height']:.2f} m up, {est['length']:.2f} m long, lying "
                f"{math.degrees(math.atan2(est['direction'][1], est['direction'][0])):+.0f} deg, "
                f"{est['samples']} samples, {est['age'] * 1000:.0f} ms old")

    def publish_bar_pose(self):
        est = self.bar_estimate()
        msg = String()
        if est is None:
            msg.data = ''
        else:
            msg.data = "|".join([
                f"{est['centre'][0]:.3f}", f"{est['centre'][1]:.3f}",
                f"{est['height']:.3f}",
                f"{math.degrees(math.atan2(est['direction'][1], est['direction'][0])):.1f}",
                f"{est['length']:.3f}", f"{est['samples']}", f"{est['age']:.3f}",
            ])
        self.bar_pose_pub.publish(msg)

    # -------------------------------------------------------- the geometry

    def assumed_estimate(self):
        """The bar the parameters say is there, in the same shape a real one has.

        Built so everything downstream -- crossing_points, crossing_altitude,
        the end-margin check, the clearance arithmetic in _begin_set -- runs
        unchanged. A blind flight and a measured one differ only in where this
        dict came from, which is the point: there is one crossing
        implementation and it is the one that has been flown.

        The bar is placed assume_bar_distance ahead of WHERE THE AIRCRAFT IS
        NOW, along the heading it took off on, lying across that heading. It
        is not placed relative to the arming x/y, because the ground estimate
        is not trustworthy and the climb may have drifted -- the aircraft's
        current position, anchored on flow, is the better datum.
        """
        lp = self.local_position
        if lp is None or self.home_z is None:
            return None

        heading = self.home_yaw
        forward = np.array([math.cos(heading), math.sin(heading), 0.0])
        centre = (np.array([lp.x, lp.y, self.home_z - self.ASSUME_BAR_HEIGHT])
                  + forward * self.ASSUME_BAR_DISTANCE)
        # The bar lies ACROSS the course, so its direction is perpendicular to
        # the heading the aircraft will cross on.
        direction = np.array([-math.sin(heading), math.cos(heading), 0.0])

        return {
            'centre': centre,
            'direction': direction,
            'height': self.ASSUME_BAR_HEIGHT,
            'length': self.ASSUME_BAR_LENGTH,
            'samples': 0,
            'age': 0.0,
            'assumed': True,
        }

    def crossing_points(self, est):
        """(entry, exit, heading) for a bar estimate, all in NED.

        The crossing is perpendicular to the bar and through its centre. Which
        of the two perpendicular directions is "across" is decided by where
        the aircraft is: the one that points from the aircraft towards the bar
        is the one that ends up on the far side.
        """
        centre = est['centre']
        direction = est['direction']
        # Horizontal normal to the bar. Either sign is a valid normal; pick
        # the one pointing away from the aircraft.
        normal = np.array([-direction[1], direction[0], 0.0])
        lp = self.local_position
        if lp is not None:
            to_bar = np.array([centre[0] - lp.x, centre[1] - lp.y, 0.0])
            if float(np.dot(normal, to_bar)) < 0.0:
                normal = -normal
        entry = centre - normal * self.STANDOFF_DISTANCE
        exit_point = centre + normal * self.EXIT_DISTANCE
        heading = math.atan2(normal[1], normal[0])
        return entry, exit_point, heading

    def crossing_altitude(self, est):
        """Height above the arming point to fly the crossing at.

        Solved for the airframe, not the origin. Going OVER, the part that has
        to clear the bar is the landing gear, which hangs body_below under the
        commanded point; going UNDER it is the top of the airframe, body_above
        over it. Returns (altitude, reachable).
        """
        surface = (est['height'] + self.BAR_RADIUS if self.pass_mode == 'over'
                   else est['height'] - self.BAR_RADIUS)
        if self.pass_mode == 'over':
            wanted = surface + self.CROSS_CLEARANCE + self.body_below
        else:
            wanted = surface - self.CROSS_CLEARANCE - self.body_above
        clamped = min(max(wanted, self.MIN_ALTITUDE), self.MAX_ALTITUDE)
        return clamped, abs(clamped - wanted) <= 0.05

    def _cross_track(self, est=None):
        """(along, cross) metres from the entry point, in the crossing frame."""
        lp = self.local_position
        if lp is None or self.move_target_x is None or self.cross_heading is None:
            return None, None
        ex = lp.x - self.move_target_x
        ey = lp.y - self.move_target_y
        c, s = math.cos(self.cross_heading), math.sin(self.cross_heading)
        return ex * c + ey * s, -ex * s + ey * c

    def _distance_along_crossing(self):
        lp = self.local_position
        if lp is None or self.cross_entry is None:
            return 0.0
        c, s = math.cos(self.cross_heading), math.sin(self.cross_heading)
        return (lp.x - self.cross_entry[0]) * c + (lp.y - self.cross_entry[1]) * s

    def _clamp_to_cone(self, heading):
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

    def _aim_yaw_at(self, heading):
        self.yaw_remaining = wrap_pi(self._clamp_to_cone(heading) - self.yaw_setpoint)

    def _heading_error(self, heading):
        lp = self.local_position
        if lp is None:
            return math.pi
        return abs(wrap_pi(heading - lp.heading))

    def _set_target(self, x, y, altitude=None):
        self.move_target_x = float(x)
        self.move_target_y = float(y)
        self.moving = True
        if altitude is not None:
            self.commanded_altitude = float(altitude)
            self.target_z = self.home_z - float(altitude)

    # ---------------------------------------------------------- EKF2 resets

    def _on_heading_reset(self, delta):
        """Turn the bar, and the crossing built from it, with the frame."""
        super()._on_heading_reset(delta)
        lp = self.local_position
        if lp is None:
            self.estimator.samples.clear()
            return
        pivot = (lp.x, lp.y)
        c, s = math.cos(delta), math.sin(delta)

        def turn(x, y):
            dx, dy = x - pivot[0], y - pivot[1]
            return (pivot[0] + c * dx - s * dy, pivot[1] + s * dx + c * dy)

        with self.estimator._lock:
            for sample in self.estimator.samples:
                centre = sample['centre']
                centre[0], centre[1] = turn(centre[0], centre[1])
                d = sample['direction']
                d[0], d[1] = c * d[0] - s * d[1], s * d[0] + c * d[1]

        if self.cross_heading is not None:
            self.cross_heading = wrap_pi(self.cross_heading + delta)
        for name in ('cross_entry', 'cross_exit'):
            point = getattr(self, name, None)
            if point is not None:
                point[0], point[1] = turn(point[0], point[1])
        if self.move_target_x is not None:
            self.move_target_x, self.move_target_y = turn(
                self.move_target_x, self.move_target_y)
            self.move_start_x, self.move_start_y = turn(
                self.move_start_x, self.move_start_y)

    # ------------------------------------------------------- the flight clock

    def flight_time(self):
        if self.flight_start is None:
            return 0.0
        return time.monotonic() - self.flight_start

    def _check_flight_clock(self):
        """Land at flight_seconds -- but never from inside the crossing."""
        if self.flight_start is None:
            return False
        if self.current_stage in (self.CROSS,):
            return False
        if self.current_stage not in (self.TAKEOFF, self.HOLD) + self.BAR_STAGES:
            return False
        if self.flight_time() < self.FLIGHT_SECONDS:
            return False
        self._begin_landing(f"{self.FLIGHT_SECONDS:.0f} s airborne")
        return True

    # ------------------------------------------------------- state machine

    def timer_callback(self):
        self.publish_bar_pose()

        if self.current_stage not in self.BAR_STAGES:
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
            self.SEARCH: self._handle_search,
            self.LOCK: self._handle_lock,
            self.SET: self._handle_set,
            self.CROSS: self._handle_cross,
            self.CLEAR: self._handle_clear,
        }[self.current_stage]()

    def _handle_takeoff(self):
        if self.flight_start is None:
            self.flight_start = time.monotonic()
            self.get_logger().warning(
                f"Flight clock started: hard limit {self.FLIGHT_SECONDS:.0f} s.")
        super()._handle_takeoff()

    def _handle_hold(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            if self.flying_blind:
                if not self.hold_xy:
                    # The crossing is flown to a POINT, and there is no point
                    # without a lateral estimate. Blind about the bar is fine;
                    # blind about where the aircraft is, is not.
                    self.get_logger().warning(
                        "Waiting for a healthy lateral estimate before the "
                        "blind crossing.", throttle_duration_sec=2.0)
                    return
                est = self.assumed_estimate()
                if est is None:
                    return
                self._begin_set(est)
                return
            self._enter_stage(self.SEARCH)
            self.get_logger().warning(
                "SEARCH: holding the takeoff heading, looking for the "
                f"{self.pass_mode} bar. Point the aircraft at it before you arm.")
            return
        self.get_logger().info(
            f"Holding, {remaining:.1f} s to the search...", throttle_duration_sec=1.0)
        self.log_flight_state()

    # ------------------------------------------------------------- SEARCH

    def _handle_search(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        if self.bar_is_confirmed() and self.bar_estimate() is not None:
            if not self.hold_xy:
                self.get_logger().warning(
                    "Bar measured but the lateral estimate is not healthy "
                    "enough to fly on. Waiting.", throttle_duration_sec=2.0)
                return
            self._enter_stage(self.LOCK)
            self.get_logger().warning(f"LOCK: {self.pose_summary()}.")
            return

        self.get_logger().info(
            f"SEARCH: {self.pose_summary()}", throttle_duration_sec=1.0)

    # --------------------------------------------------------------- LOCK

    def _handle_lock(self):
        """Stand still and let the estimate converge, then commit to a plan."""
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        est = self.bar_estimate()
        if est is None:
            if self._give_up_on_pose('LOCK'):
                return
            self.get_logger().info(
                f"LOCK: rebuilding the bar pose. {self.pose_summary()}",
                throttle_duration_sec=1.0)
            return

        if self._in_stage_for() < self.SET_SETTLE_SECONDS:
            self.get_logger().info(
                f"LOCK: settling on the measurement. {self.pose_summary()}",
                throttle_duration_sec=1.0)
            return

        self._begin_set(est)

    # ---------------------------------------------------------------- SET

    def _begin_set(self, est):
        """Freeze the crossing, then climb (or descend) onto its altitude."""
        entry, exit_point, heading = self.crossing_points(est)
        altitude, reachable = self.crossing_altitude(est)

        if not reachable:
            self._abandon(
                f"the bar is {est['height']:.2f} m up, which needs a crossing "
                f"altitude outside [{self.MIN_ALTITUDE:.2f}, "
                f"{self.MAX_ALTITUDE:.2f}] m. Raise max_altitude if the arena "
                "allows it.")
            return

        # Crossing near an end of the bar is how a prop finds the upright it
        # hangs from. The crossing point is the bar's CENTRE, so this is really
        # a check that the bar was measured end to end.
        if est['length'] < 2.0 * (0.5 * self.DRONE_WIDTH + self.END_MARGIN):
            self._abandon(
                f"the bar measures only {est['length']:.2f} m, which does not "
                f"leave {self.END_MARGIN:.2f} m either side of a "
                f"{self.DRONE_WIDTH:.2f} m airframe crossing at its centre")
            return

        self.cross_entry = entry
        self.cross_exit = exit_point
        self.cross_heading = heading
        self.cross_altitude = altitude
        self.cross_bar = est
        self.set_in_band_since = None
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._enter_stage(self.SET)
        self._set_target(entry[0], entry[1], altitude)
        self._aim_yaw_at(heading)

        gap = (altitude - self.body_below - (est['height'] + self.BAR_RADIUS)
               if self.pass_mode == 'over' else
               (est['height'] - self.BAR_RADIUS) - (altitude + self.body_above))
        source = 'ASSUMED' if est.get('assumed') else 'measured'
        self.get_logger().warning(
            f"SET: the bar is {source} {est['height']:.2f} m up and "
            f"{est['length']:.2f} m "
            f"long. Going {self.pass_mode.upper()} it at {altitude:.2f} m, "
            f"which leaves {gap:.2f} m between the airframe and the bar. "
            f"Flying to ({entry[0]:+.2f}, {entry[1]:+.2f}) NED on a heading of "
            f"{math.degrees(heading):+.0f} deg first.")

    def _handle_set(self):
        """Get to the entry point, on the crossing altitude, pointing across."""
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        # The plan is NOT re-derived here. Unlike the window approach, the
        # thing being measured leaves the field of view as the aircraft climbs
        # towards it -- a bar the aircraft is about to fly over drops out of
        # frame long before the aircraft is above it -- so re-deriving would
        # mean tracking a target that is disappearing by design. The estimate
        # was built stationary, from the best view of the bar this flight will
        # ever have, and it is the estimate the crossing uses.
        self._aim_yaw_at(self.cross_heading)
        self._cross_check()

        if not self.hold_xy:
            self.moving = False
            self.set_in_band_since = None
            self.get_logger().warning(
                "SET: lateral estimate unhealthy, holding still.",
                throttle_duration_sec=2.0)
            if self._in_stage_for() > self.SET_TIMEOUT:
                self._abandon("the lateral estimate never recovered before the crossing")
            return

        along, cross = self._cross_track()
        alt = self.relative_altitude()
        ready = (along is not None
                 and abs(along) <= self.ALIGN_ALONG_TOLERANCE
                 and abs(cross) <= self.ALIGN_CROSS_TOLERANCE
                 and alt is not None
                 and abs(alt - self.cross_altitude) <= self.SET_ALT_TOLERANCE
                 and self._heading_error(self.cross_heading) <= self.ALIGN_YAW_TOLERANCE)

        if ready:
            if self.set_in_band_since is None:
                self.set_in_band_since = time.monotonic()
            elif time.monotonic() - self.set_in_band_since >= self.SET_SETTLE_SECONDS:
                self._begin_cross()
            return

        self.set_in_band_since = None

        if self._in_stage_for() > self.SET_TIMEOUT:
            self._abandon(
                f"could not settle on the crossing entry in {self.SET_TIMEOUT:.0f} s")
            return

        self.get_logger().info(
            f"SET: {along:+.2f} m along / {cross:+.2f} m across to the entry "
            f"(tolerances {self.ALIGN_ALONG_TOLERANCE:.2f}/"
            f"{self.ALIGN_CROSS_TOLERANCE:.2f}), alt "
            f"{'n/a' if alt is None else f'{alt:+.2f}'}/{self.cross_altitude:.2f} m, "
            f"{math.degrees(self._heading_error(self.cross_heading)):.0f} deg "
            "off the crossing heading.", throttle_duration_sec=1.0)

    def _cross_check(self):
        """Say what the camera thinks, when the flight is not listening to it.

        Only runs on a blind crossing, and changes nothing: the point is that
        the log carries both numbers, so after the flight you can tell whether
        the assumed height was right without having to have trusted it in the
        air. A disagreement here is the cheapest possible way to find out that
        assume_bar_height is set to the wrong rules setting.
        """
        if not self.flying_blind or self.cross_bar is None:
            return
        est = self.bar_estimate()
        if est is None:
            return
        error = est['height'] - self.ASSUME_BAR_HEIGHT
        level = (self.get_logger().warning if abs(error) > 0.25
                 else self.get_logger().info)
        level(f"CROSS-CHECK: assumed {self.ASSUME_BAR_HEIGHT:.2f} m, camera "
              f"measures {est['height']:.2f} m ({error:+.2f} m). Flying the "
              "assumed height either way.", throttle_duration_sec=2.0)

    # -------------------------------------------------------------- CROSS

    def _begin_cross(self):
        self.MOVE_SPEED = self.CROSS_SPEED
        self._set_target(self.cross_exit[0], self.cross_exit[1], self.cross_altitude)
        self._enter_stage(self.CROSS)
        self.get_logger().warning(
            f"CROSS: committed. Flying {self.pass_mode} the bar to "
            f"({self.cross_exit[0]:+.2f}, {self.cross_exit[1]:+.2f}) NED at "
            f"{self.CROSS_SPEED:.2f} m/s, altitude {self.cross_altitude:.2f} m. "
            "The camera is no longer steering: this target is frozen.")

    def _handle_cross(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()
        self.yaw_remaining = wrap_pi(self.cross_heading - self.yaw_setpoint)

        along = self._distance_along_crossing()
        total = self.STANDOFF_DISTANCE + self.EXIT_DISTANCE
        if along >= total - self.ALIGN_ALONG_TOLERANCE:
            self.outcome = f"CROSSED ({self.pass_mode}) at {self.cross_altitude:.2f} m"
            self._enter_stage(self.CLEAR)
            self.get_logger().warning(
                f"CLEAR: {along:.2f} m of {total:.2f} m flown, the bar is "
                f"behind us. Holding {self.CLEAR_SECONDS:.0f} s before the "
                "descent.")
            return

        if self._in_stage_for() > self.CROSS_TIMEOUT:
            self._abandon(
                f"the crossing timed out {total - along:.2f} m short")
            return

        self.get_logger().info(
            f"CROSS: {along:.2f}/{total:.2f} m.", throttle_duration_sec=0.5)

    # -------------------------------------------------------------- CLEAR

    def _handle_clear(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()
        remaining = self.CLEAR_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_landing(f"bar crossed ({self.pass_mode})")
            return
        self.get_logger().info(
            f"CLEAR: holding, {remaining:.1f} s to the descent.",
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------ giving up

    def _give_up_on_pose(self, stage):
        if self.pose_lost_since is None:
            return False
        if time.monotonic() - self.pose_lost_since < self.POSE_LOST_TIMEOUT:
            return False
        self._abandon(
            f"no usable bar pose for {self.POSE_LOST_TIMEOUT:.0f} s during {stage}")
        return True

    def _abandon(self, reason):
        self.outcome = f"ABANDONED: {reason}"
        self.moving = False
        self.yaw_remaining = 0.0
        self.MOVE_SPEED = self.APPROACH_SPEED
        self.get_logger().error(f"Bar crossing abandoned: {reason}.")
        self._begin_landing(f"bar crossing abandoned -- {reason}")

    # --------------------------------------------------------------- status

    def publish_status(self):
        if self.current_stage not in self.BAR_STAGES:
            super().publish_status()
            return
        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED

        if self.current_stage == self.SEARCH:
            detail = 'look' if not self.bar_flag else 'bar?'
        elif self.current_stage == self.LOCK:
            detail = f"lock{self.estimator.fresh_count(time.monotonic())}"
        elif self.current_stage == self.SET:
            detail = ('set?' if self.cross_altitude is None
                      else f"set{self.cross_altitude:.2f}")
        elif self.current_stage == self.CROSS:
            total = self.STANDOFF_DISTANCE + self.EXIT_DISTANCE
            detail = f"x{self._distance_along_crossing():.1f}/{total:.1f}"
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

    def log_flight_state(self):
        super().log_flight_state()
        self.get_logger().info(f"bar: {self.pose_summary()}",
                               throttle_duration_sec=2.0)

    def destroy_node(self):
        self.get_logger().warning(
            f"Bar crossing outcome: {self.outcome}. "
            f"{self.estimator.accepted_total} geometry samples accepted of "
            f"{self.geometry_seen} received; rejections: "
            f"{self.estimator.rejection_summary(limit=5)}.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BarCross()
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
