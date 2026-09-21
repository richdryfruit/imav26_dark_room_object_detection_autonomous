"""
Offboard takeoff -> four commanded motions -> land.

Sequence: stream offboard setpoints -> enter Offboard -> arm -> sit armed
on the ground -> climb to the takeoff altitude -> hold until the optical
flow is healthy and x/y is latched -> execute the requested motions one at
a time, holding between each -> hold -> descend slowly -> disarm.

Each motion is one of:

    forward / backward / left / right   <metres>   horizontal translation
    up / down                           <metres>   altitude change
    yaw                                 <degrees>  rotation in place

given as a single `sequence` string, e.g.

    -p sequence:="forward 1.0, yaw 30, up 0.5, right 1.0"

Comma-separated; each item is a name and a number separated by a space, a
colon or an equals sign. Any number of steps is accepted (up to MAX_STEPS);
four is what this test was written for.

WHICH FRAME THE DIRECTIONS ARE IN
---------------------------------
By default `forward` means the direction the vehicle was FACING WHEN IT
ARMED, and it keeps meaning that for the whole flight. A `yaw 30` step in
the middle of the sequence rotates the airframe but does NOT rotate what
`forward` means -- a following `forward 1.0` flies along the same ground
track it would have flown without the yaw, just crabbing 30 degrees.

That is the behaviour asked for, and it is also the honest one: every
setpoint PX4 is given here is in the NED local frame (this is the same
reason a velocity setpoint in SITL does not turn with the airframe), so a
fixed reference yaw is the only interpretation that does not silently
depend on how well the yaw step tracked.

Set `direction_frame:=current` if you want the other convention, where
each move is resolved against the yaw currently being commanded and the
sequence above would fly a 30-degree dog-leg.

WHY EACH MOVE IS FLOWN AS A POSITION SETPOINT, NOT A VELOCITY
-------------------------------------------------------------
A velocity setpoint is open loop with respect to distance: "1 m forward"
becomes "0.5 m/s for 2 s and hope", and every source of error -- the
optical flow's velocity bias, the acceleration and deceleration ramps PX4
puts on the ends, wind -- integrates straight into the distance actually
travelled with nothing to correct it. There is no feedback on the thing
you asked for.

A position setpoint closes that loop: PX4 flies to a point and holds it,
so flow bias produces a bounded offset instead of an unbounded drift, and
the vehicle actively stops itself at the end instead of coasting.

The catch is that a position setpoint is only as good as the x/y estimate
it is expressed in, and on this flow-only airframe that estimate is
garbage on the ground and only becomes meaningful once the vehicle is at
altitude with the flow sensor actually correcting. So the horizontal moves
are gated on exactly that:

  * the ground and the climb are flown as ZERO VELOCITY in x/y, because
    "stay still" has no memory and cannot fly out an accumulated
    dead-reckoning error;
  * once airborne, x/y position hold is latched onto a FRESH estimate
    (see _try_latch_xy_hold);
  * ONLY THEN is a horizontal move started, as a ramped position setpoint:
    a carrot walked from the latched point to the target at MOVE_SPEED,
    leashed to the measured position so it can never run away.

If the flow never becomes healthy, a horizontal step is SKIPPED rather
than dead-reckoned. Altitude and yaw steps do not need the lateral
estimate and still run: they are measured by the lidar and the compass /
gyro respectively.

Keys:  q -> abort into a controlled descent from wherever we are.
       k -> force-disarm immediately (motors cut, vehicle drops).

The vertical estimate must be healthy for this to be safe. The node
refuses to arm without z_valid.
"""

import math
import re
import select
import sys
import threading
import termios
import time
import tty

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import (
    EstimatorStatusFlags,
    FailsafeFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from drone_testing.px4_topics import versioned_names


# PX4's nav_state is an integer in the log and unreadable at 3 a.m. on a
# flight line. Built from the message constants rather than hard-coded so it
# cannot drift out of date with px4_msgs.
NAV_STATE_NAMES = {
    getattr(VehicleStatus, _n): _n[len('NAVIGATION_STATE_'):]
    for _n in dir(VehicleStatus) if _n.startswith('NAVIGATION_STATE_')
}


def nav_state_name(value):
    return f"{NAV_STATE_NAMES.get(value, 'UNKNOWN')}({value})"


def wrap_pi(angle):
    """Wrap an angle in radians into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# Every boolean in FailsafeFlags that can plausibly explain PX4 taking the
# aircraft off us. Checked by name with getattr so a firmware/px4_msgs pair
# that lacks one of them degrades quietly instead of crashing the node.
FAILSAFE_FLAG_NAMES = (
    'angular_velocity_invalid',
    'attitude_invalid',
    'local_altitude_invalid',
    'local_position_invalid',
    'local_position_invalid_relaxed',
    'local_velocity_invalid',
    'offboard_control_signal_lost',
    'manual_control_signal_lost',
    'gcs_connection_lost',
    'home_position_invalid',
    'battery_low_remaining_time',
    'battery_unhealthy',
    'fd_critical_failure',
    'fd_esc_arming_failure',
    'fd_motor_failure',
    'fd_alt_loss',
    'geofence_breached',
    'position_accuracy_low',
    'navigator_failure',
    'wind_limit_exceeded',
    'flight_time_limit_exceeded',
)

# Conditions that are permanently true on THIS airframe and mean nothing here.
# See the same list in offboard_translate.py for the full reasoning.
EXPECTED_FAILSAFE_FLAGS = (
    'home_position_invalid',
    'gcs_connection_lost',
)


class Step:
    """One commanded motion, parsed from the `sequence` string.

    kind is 'move' (horizontal, arg = metres, with a body-frame unit vector
    picked at execution time), 'alt' (arg = metres, positive up) or 'yaw'
    (arg = radians, positive clockwise seen from above, i.e. PX4's yaw sign).
    """

    def __init__(self, kind, name, arg):
        self.kind = kind
        self.name = name
        self.arg = arg

    def __str__(self):
        if self.kind == 'yaw':
            return f"yaw {math.degrees(self.arg):+.0f} deg"
        if self.kind == 'alt':
            return f"{self.name} {abs(self.arg):.2f} m"
        return f"{self.name} {self.arg:.2f} m"


# Body-frame horizontal unit vectors in NED, as a function of the reference
# yaw (measured from North, x = North, y = East). Forward is (cos, sin);
# right is forward rotated 90 degrees clockwise seen from above, (-sin, cos).
DIRECTIONS = {
    'forward':  lambda c, s: (c, s),
    'backward': lambda c, s: (-c, -s),
    'right':    lambda c, s: (-s, c),
    'left':     lambda c, s: (s, -c),
}

# Everything the sequence parser accepts, mapped to a canonical name.
STEP_ALIASES = {
    'forward': 'forward', 'fwd': 'forward', 'front': 'forward', 'f': 'forward',
    'backward': 'backward', 'back': 'backward', 'bwd': 'backward', 'b': 'backward',
    'left': 'left', 'l': 'left',
    'right': 'right', 'r': 'right',
    'up': 'up', 'u': 'up', 'climb': 'up', 'ascend': 'up',
    'down': 'down', 'd': 'down', 'descend': 'down',
    'yaw': 'yaw', 'turn': 'yaw', 'rotate': 'yaw', 'heading': 'yaw',
}


def parse_sequence(text):
    """Parse "forward 1.0, yaw 30, up 0.5, right 1.0" into a list of Steps.

    Raises ValueError with a message aimed at whoever typed the string, since
    a typo here is a typo in a flight plan and must not be quietly guessed at.
    """
    steps = []
    for item in re.split(r'[,;]', str(text)):
        item = item.strip()
        if not item:
            continue
        parts = [p for p in re.split(r'[:\s=]+', item) if p]
        if len(parts) != 2:
            raise ValueError(
                f"cannot read step '{item}': expected a name and a number, "
                "e.g. 'forward 1.0' or 'yaw:30'")
        name, value = parts[0].lower(), parts[1]
        if name not in STEP_ALIASES:
            raise ValueError(
                f"unknown motion '{parts[0]}' in '{item}'; expected one of "
                f"{sorted(set(STEP_ALIASES.values()))}")
        name = STEP_ALIASES[name]
        try:
            value = float(value)
        except ValueError:
            raise ValueError(f"'{parts[1]}' in '{item}' is not a number")

        if name == 'yaw':
            steps.append(Step('yaw', 'yaw', math.radians(value)))
        elif name in ('up', 'down'):
            # A negative distance is almost always a typo rather than an
            # inverted intent -- "down -1" reads as "up 1" but nobody means
            # that -- so reject it instead of flying it.
            if value < 0.0:
                raise ValueError(
                    f"'{item}': use 'up'/'down' to choose the direction, not a "
                    "negative distance")
            steps.append(Step('alt', name, value if name == 'up' else -value))
        else:
            if value < 0.0:
                raise ValueError(
                    f"'{item}': use the opposite direction rather than a "
                    "negative distance")
            steps.append(Step('move', name, value))
    return steps


class OffboardSequence(Node):

    PREPARATION = "PREPARATION"
    OFFBOARD_REQUEST = "OFFBOARD_REQUEST"
    ARMING = "ARMING"
    GROUND_WAIT = "GROUND_WAIT"
    TAKEOFF = "TAKEOFF"
    HOLD = "HOLD"
    STEP = "STEP"
    STEP_HOLD = "STEP_HOLD"
    POST_HOLD = "POST_HOLD"
    LANDING = "LANDING"
    DISARMING = "DISARMING"
    KILLING = "KILLING"
    DONE = "DONE"

    # ---- the mission ------------------------------------------------------
    SEQUENCE = "forward 1.0, yaw 30, up 0.5, right 1.0"
    MAX_STEPS = 12              # sanity cap, not a design limit
    STEP_HOLD_SECONDS = 3.0     # station keeping between steps. Each step is
                                # measured from where the previous one ended,
                                # so this is what stops errors compounding
                                # while the vehicle is still settling.
    DIRECTION_FRAME = 'home'    # home | current -- see the module docstring

    # ---- flight parameters -----------------------------------------------
    TAKEOFF_ALTITUDE = 1.00     # m above the arming point
    GROUND_WAIT_SECONDS = 5.0   # armed on the ground before the climb starts
    HOLD_SECONDS = 5.0          # station keeping before the first step
    POST_HOLD_SECONDS = 5.0     # station keeping after the last step
    CLIMB_SPEED = 0.80          # m/s. Brisk on purpose: a slow climb lingers in
                                # ground effect with no valid flow, which is the
                                # least stable place the vehicle can be.
    LAND_SPEED = 0.15           # m/s, rate the descent setpoint is ramped at
    ALTITUDE_TOLERANCE = 0.08   # m, "we are there" band around the target
    SETTLE_SECONDS = 0.5        # time inside the band before the hold starts
    LAND_OVERSHOOT = 0.50       # m the descent setpoint is pushed below ground
    GROUND_PRESS = 0.15         # m the setpoint is held BELOW ground while armed
                                # and waiting, so the vehicle stays firmly planted
                                # instead of skittering at the edge of liftoff.
    LIFTOFF_AGL = 0.15          # m AGL above which we consider ourselves airborne
    OVERSHOOT_ABORT = 0.50      # m above the commanded altitude before we call
                                # the climb a runaway and land.
    LEASH_RELEASE_VZ = 0.10     # m/s. Above this the vehicle is tracking, so the
                                # leash lets go (see _step_setpoint_ramp).
    SETPOINT_LEASH = 0.60       # m the commanded z may lead the measured z by.

    # Altitude steps are clamped into this envelope. A "+0.5" typed as "+5"
    # should not be a flight into the ceiling.
    MIN_ALTITUDE = 0.40         # m above the arming point
    MAX_ALTITUDE = 3.00         # m above the arming point

    # ---- horizontal translation ------------------------------------------
    MOVE_SPEED = 0.30           # m/s the horizontal setpoint carrot is walked
                                # at. Slow on purpose: the flow estimate is the
                                # only thing measuring this move.
    MOVE_TOLERANCE = 0.15       # m, "we are there" radius around the target
    MOVE_SETTLE_SECONDS = 1.0   # time inside that radius before we call it done
    MOVE_LEASH = 0.40           # m the commanded x/y may lead the measured x/y
                                # by. Same job as SETPOINT_LEASH does for z.
    MOVE_LATCH_TIMEOUT = 15.0   # s waiting for a flow-healthy x/y latch before
                                # giving up on a horizontal step
    MOVE_TIMEOUT = 30.0         # s for one horizontal step
    ALT_STEP_TIMEOUT = 20.0     # s for one altitude step

    # ---- yaw --------------------------------------------------------------
    YAW_RATE = 0.35             # rad/s (~20 deg/s) the yaw setpoint is walked
                                # at. Slow: a fast yaw smears the optical flow
                                # and the whole point of holding position
                                # through the turn is that the flow keeps
                                # working.
    YAW_TOLERANCE = math.radians(5.0)
    YAW_SETTLE_SECONDS = 1.0    # time inside tolerance before the step is done
    YAW_LEASH = math.radians(25.0)  # rad the commanded yaw may lead the measured
                                # heading by, for the same reason as the x/y and
                                # z leashes: stop the ramp walking away from an
                                # airframe that is not keeping up and having the
                                # error flown out as one fast spin at the end.
    YAW_TIMEOUT = 30.0          # s for one yaw step

    # ---- flow / estimator health -----------------------------------------
    # Below this AGL the rangefinder and optical flow are not trustworthy:
    # too close for the lidar, too little parallax for the flow.
    FLOW_MIN_AGL = 0.30
    # x/y position hold is only latched after flow has been continuously
    # healthy this long, so a single good sample cannot trigger it.
    FLOW_SETTLE_SECONDS = 1.0
    # How far the vehicle may appear to have moved during the climb before the
    # arming point stops being somewhere we are willing to fly back to. Half a
    # metre is more than a well-behaved climb ever produces and less than the
    # distance at which flying to a stale point is itself the hazard.
    TAKEOFF_ANCHOR_MAX_DRIFT = 0.50
    TAKEOFF_ANCHOR_DEADBAND = 0.10   # m. Below this, do not bother correcting.
    # Whether to fly BACK to the arming x/y once the flow anchors the climb.
    #
    # Off by default, and the reason is in the module header: the x/y estimate
    # on the ground is not trustworthy. It is captured before the flow is
    # fused, it drifts while the aircraft sits armed through ground_wait, and
    # EKF2 re-datums it on the way up. Flying to it is therefore flying to a
    # number of unknown quality -- and doing it as the first thing after the
    # climb, which is when the aircraft pitches over and translates in a way
    # that reads as "it took off backwards". Holding the point the flow
    # actually anchored is a better estimate of where the aircraft is, and it
    # costs only that the vehicle ends the climb wherever the drift left it,
    # which the drift warning below reports either way.
    TAKEOFF_RETURN_TO_PAD = False

    # ---- timings / limits -------------------------------------------------
    SETPOINT_WARMUP = 20        # setpoints streamed before requesting Offboard (@20 Hz = 1 s)
    OFFBOARD_TIMEOUT = 10.0
    ARMING_TIMEOUT = 10.0
    TAKEOFF_TIMEOUT = 20.0
    LANDING_TIMEOUT = 30.0
    DISARM_TIMEOUT = 5.0
    LANDED_CONFIRM_SECONDS = 1.0    # land-detector must agree this long
    COMMAND_INTERVAL = 0.25         # s between repeats of a vehicle command.

    # ---- touchdown fallback ----------------------------------------------
    # PX4's land detector is the primary answer, but it is not sufficient on
    # its own in Offboard -- see _touchdown_confirmed() for exactly why it can
    # sit at landed=false on a vehicle that is plainly sitting on the floor.
    # These are the thresholds for the independent, geometric fallback.
    STALL_CONFIRM_SECONDS = 2.0     # how long the stalled descent must persist
    STALL_VZ = 0.10                 # m/s below which the descent has stopped
    STALL_AGL = 0.25                # m AGL below which we are plausibly down
    STALL_SETPOINT_BURIED = 0.20    # m the commanded z must be below measured z,
                                    # i.e. we are definitely still pushing down

    # If you switch to Offboard from your RC transmitter instead of from
    # this node, set this to False.
    REQUEST_OFFBOARD_FROM_ROS = True
    # ----------------------------------------------------------------------

    def __init__(self, node_name='offboard_sequence'):
        # node_name is a parameter so a subclass -- window_scan is the one that
        # does this -- can reuse the whole state machine under its own name.
        super().__init__(node_name)

        # The numbers you actually want to change between hardware tests are
        # exposed as ROS parameters; the rest stay as class constants above.
        # float() on the way out: launch passes parameters as YAML, so
        # `takeoff_altitude:=1` arrives as an int.
        self.TAKEOFF_ALTITUDE = float(self._declare_number(
            'takeoff_altitude', self.TAKEOFF_ALTITUDE))
        self.HOLD_SECONDS = float(self._declare_number(
            'hold_seconds', self.HOLD_SECONDS))
        self.POST_HOLD_SECONDS = float(self._declare_number(
            'post_hold_seconds', self.POST_HOLD_SECONDS))
        self.STEP_HOLD_SECONDS = float(self._declare_number(
            'step_hold_seconds', self.STEP_HOLD_SECONDS))
        self.MOVE_SPEED = float(self._declare_number(
            'move_speed', self.MOVE_SPEED))
        self.YAW_RATE = float(self._declare_number('yaw_rate', self.YAW_RATE))
        self.TAKEOFF_RETURN_TO_PAD = bool(self.declare_parameter(
            'takeoff_return_to_pad', self.TAKEOFF_RETURN_TO_PAD).value)
        self.GROUND_WAIT_SECONDS = float(self._declare_number(
            'ground_wait_seconds', self.GROUND_WAIT_SECONDS))
        self.CLIMB_SPEED = float(self._declare_number(
            'climb_speed', self.CLIMB_SPEED))
        self.LAND_SPEED = float(self._declare_number(
            'land_speed', self.LAND_SPEED))
        # SITL ONLY. A simulated rangefinder on the pad reads a perfectly
        # constant value, EKF2 flags it stuck and stops fusing it -- which a
        # real, noisy sensor never does. false skips the EKF2 rangefinder-fusion
        # gate below. Never set false on the aircraft.
        self.RANGEFINDER_CHECKS = bool(self.declare_parameter(
            'rangefinder_checks', True).value)
        self.MIN_ALTITUDE = float(self._declare_number(
            'min_altitude', self.MIN_ALTITUDE))
        self.MAX_ALTITUDE = float(self._declare_number(
            'max_altitude', self.MAX_ALTITUDE))
        self.REQUEST_OFFBOARD_FROM_ROS = bool(self.declare_parameter(
            'request_offboard_from_ros', self.REQUEST_OFFBOARD_FROM_ROS).value)

        self.DIRECTION_FRAME = str(self.declare_parameter(
            'direction_frame', self.DIRECTION_FRAME).value).strip().lower()
        if self.DIRECTION_FRAME not in ('home', 'current'):
            self.get_logger().error(
                f"Unknown direction_frame '{self.DIRECTION_FRAME}'; expected "
                "'home' or 'current'. Falling back to 'home'.")
            self.DIRECTION_FRAME = 'home'

        # The flight plan. A bad sequence string is fatal: there is no sane
        # default to fall back on, and taking off with a mission other than
        # the one that was typed is the worst possible failure mode here.
        sequence_text = str(self.declare_parameter('sequence', self.SEQUENCE).value)
        try:
            self.steps = parse_sequence(sequence_text)
        except ValueError as exc:
            raise SystemExit(f"Bad `sequence` parameter: {exc}")
        if not self.steps:
            raise SystemExit("Bad `sequence` parameter: no steps in it.")
        if len(self.steps) > self.MAX_STEPS:
            raise SystemExit(
                f"Bad `sequence` parameter: {len(self.steps)} steps, "
                f"the cap is {self.MAX_STEPS}.")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # Compact machine-readable status for the LCD node. Same pipe-separated
        # format as the takeoff/translate nodes, and the same topic, so the LCD
        # needs no changes.
        self.status_pub = self.create_publisher(String, 'takeoff_status', 10)

        # Two callback groups on a MultiThreadedExecutor (see spin_node), and
        # this is a flight-safety measure, not a performance one.
        #
        # rclpy's default is ONE thread and ONE mutually-exclusive group for
        # everything, and its executor does not prioritise timers: it takes
        # whatever work is ready. Every /fmu/out topic we subscribe to is
        # therefore competing with the 20 Hz setpoint timer for the same
        # thread, and PX4 hands the aircraft back if that timer is late by
        # more than COM_OF_LOSS_T. /fmu/out/vehicle_attitude, which
        # window_traverse needs and which PX4 publishes at the EKF rate
        # (100-250 Hz, an order of magnitude above everything else here), is
        # enough on its own to starve it -- that is the
        # "offboard_control_signal_lost a second after arming" failure.
        #
        # control_cbg holds the timer and nothing else, so the heartbeat runs
        # on its own thread and cannot be delayed by callback load. sensor_cbg
        # holds every subscription, so they stay serialised with each other
        # and only the timer/subscription pair can actually run concurrently.
        # Anything mutable shared across that one boundary needs a lock; see
        # window_traverse's estimator.
        self.control_cbg = MutuallyExclusiveCallbackGroup()
        self.sensor_cbg = MutuallyExclusiveCallbackGroup()

        self.vehicle_status_subs = [
            self.create_subscription(
                VehicleStatus, name, self.vehicle_status_callback,
                qos_profile=sensor_qos, callback_group=self.sensor_cbg)
            for name in versioned_names('vehicle_status')
        ]
        self.local_position_subs = [
            self.create_subscription(
                VehicleLocalPosition, name, self.local_position_callback,
                qos_profile=sensor_qos, callback_group=self.sensor_cbg)
            for name in versioned_names('vehicle_local_position')
        ]

        # Unversioned topic, so the name is the same on every firmware that
        # bridges it. This is the only place that tells us whether EKF2 is
        # actually fusing the rangefinder -- see rangefinder_is_healthy().
        self.estimator_flags_sub = self.create_subscription(
            EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
            self.estimator_flags_callback, qos_profile=sensor_qos,
            callback_group=self.sensor_cbg)

        # PX4's own account of why it would take the aircraft away from us.
        self.failsafe_flags_sub = self.create_subscription(
            FailsafeFlags, '/fmu/out/failsafe_flags',
            self.failsafe_flags_callback, qos_profile=sensor_qos,
            callback_group=self.sensor_cbg)

        # The land detector topic is unversioned on some builds and _v1 on
        # others; subscribe to both and take whichever one actually arrives.
        self.land_detected_subs = [
            self.create_subscription(
                VehicleLandDetected, topic, self.land_detected_callback,
                qos_profile=sensor_qos, callback_group=self.sensor_cbg)
            for topic in ('/fmu/out/vehicle_land_detected',
                          '/fmu/out/vehicle_land_detected_v1')
        ]

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.status_received = False
        self._last_nav_state = None

        self.local_position = None
        self.landed = True
        self.land_detector_seen = False
        self.estimator_flags = None
        self.failsafe_flags = None
        self._last_failsafes = []

        # EKF2 reset bookkeeping. When the estimator re-datums its height or
        # lateral position it jumps x/y/z instantly and tells us by how much.
        self._z_reset_counter = None
        self._xy_reset_counter = None
        # ...and when it re-datums its YAW. This one is not cosmetic: the
        # commanded yaw is an absolute NED heading, so an unhandled heading
        # reset leaves PX4 holding a setpoint that now points somewhere else
        # entirely and it spins the airframe -- as fast as MC_YAWRATE_MAX
        # allows -- to get there. A 90 or 180 degree snap a second or two
        # after takeoff, in either direction, is this and nothing else.
        self._heading_reset_counter = None

        # Captured at the moment of arming; every setpoint is relative to it.
        self.home_x = None
        self.home_y = None
        self.home_z = None
        self.home_yaw = 0.0

        self.target_z = None        # NED z the ramp is currently walking towards
        self.setpoint_z = None      # NED z actually being commanded right now
        self.commanded_altitude = self.TAKEOFF_ALTITUDE  # m above home, what we
                                    # are currently asking for. Altitude steps
                                    # move this; the overshoot guard reads it.
        self.in_band_since = None
        self.landed_since = None
        self.stall_since = None

        # Horizontal control. hold_xy False -> command zero velocity;
        # True -> hold hold_x/hold_y, latched once airborne with good flow.
        self.hold_xy = False
        self.hold_x = None
        self.hold_y = None
        self.flow_healthy_since = None

        # Yaw control. yaw_setpoint is what is commanded every tick;
        # yaw_target is where a yaw step is walking it to.
        self.yaw_setpoint = 0.0
        self.yaw_remaining = 0.0    # rad still to be walked, signed
        self.yaw_in_band_since = None

        # Horizontal translation bookkeeping for the step in progress.
        self.moving = False
        self.move_start_x = None
        self.move_start_y = None
        self.move_target_x = None
        self.move_target_y = None
        self.move_in_band_since = None

        # Where we are in the sequence. step_index is the step being executed
        # (or about to be); step_started guards the one-shot setup.
        self.step_index = 0
        self.step_started = False
        self.step_results = []

        self.setpoint_counter = 0
        self.stage_enter_time = time.monotonic()
        self.current_stage = self.PREPARATION
        self.abort_requested = False
        self.kill_requested = False
        # Set when the height estimate dies in flight: the descent is then
        # flown as a velocity, because a position setpoint against a dead
        # z estimate is a setpoint against a number that means nothing.
        self.blind_descent = False
        # Cleared once we have stood down.
        self.stream_setpoints = True
        self._last_command_time = {}

        self._stop_event = threading.Event()
        self._stdin_is_tty = False
        self._stdin_old_settings = None
        self._keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._keyboard_thread.start()

        # 20 Hz. PX4 drops Offboard if setpoints arrive slower than 2 Hz.
        # On control_cbg so no volume of subscription traffic can delay it.
        self.timer = self.create_timer(0.05, self.timer_callback,
                                       callback_group=self.control_cbg)

        # Subclasses (window_scan) fly their own plan and print their own
        # summary; the sequence plan below would only be misleading there.
        if self.__class__ is not OffboardSequence:
            return

        plan = " -> ".join(str(s) for s in self.steps)
        self.get_logger().warning(
            f"Sequence test: climb {self.TAKEOFF_ALTITUDE:.2f} m, hold "
            f"{self.HOLD_SECONDS:.0f} s, then {len(self.steps)} steps: {plan}, "
            f"hold {self.POST_HOLD_SECONDS:.0f} s, land. Directions are in the "
            f"'{self.DIRECTION_FRAME}' yaw frame. Press q to abort into a "
            "descent, k to force-disarm.")

    # ------------------------------------------------------------ parameters

    def _declare_number(self, name, default):
        """Declare a numeric parameter that tolerates being given an int.

        The launch file feeds parameters in as YAML, so `takeoff_altitude:=1`
        arrives as an int and rclpy rejects it against a double-typed
        declaration. Declaring it dynamically-typed and converting here means
        both `1` and `1.0` work.
        """
        from rcl_interfaces.msg import ParameterDescriptor
        return self.declare_parameter(
            name, float(default),
            ParameterDescriptor(dynamic_typing=True)).value

    # ------------------------------------------------------------------ subs

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.status_received = True

        if self.nav_state != self._last_nav_state:
            self._last_nav_state = self.nav_state
            self.get_logger().info(f"nav_state -> {nav_state_name(self.nav_state)}")

    def local_position_callback(self, msg):
        self._handle_estimator_resets(msg)
        self.local_position = msg

    def _handle_estimator_resets(self, msg):
        """Shift everything we latched when EKF2 jumps its own origin.

        This is the bug that made the vehicle believe it had reached takeoff
        altitude while still sitting on the ground: EKF2 re-datums height when
        the rangefinder starts being fused, z jumps by up to the target
        altitude in a single sample, and a home_z captured before the reset is
        suddenly measuring against a frame that no longer exists.
        """
        if self._z_reset_counter is None:
            self._z_reset_counter = msg.z_reset_counter
            self._xy_reset_counter = msg.xy_reset_counter
            self._heading_reset_counter = msg.heading_reset_counter
            return

        if msg.heading_reset_counter != self._heading_reset_counter:
            self._heading_reset_counter = msg.heading_reset_counter
            self._apply_heading_reset(float(msg.delta_heading))

        if msg.z_reset_counter != self._z_reset_counter:
            self._z_reset_counter = msg.z_reset_counter
            if self.home_z is not None:
                self.home_z += msg.delta_z
                self.target_z += msg.delta_z
                self.setpoint_z += msg.delta_z
                self.get_logger().warning(
                    f"EKF2 height reset: delta_z={msg.delta_z:+.2f} m, "
                    "shifted altitude reference to match.")

        if msg.xy_reset_counter != self._xy_reset_counter:
            self._xy_reset_counter = msg.xy_reset_counter
            if self.hold_x is not None:
                self.hold_x += msg.delta_xy[0]
                self.hold_y += msg.delta_xy[1]
            # The move target lives in the same frame and has to move with it,
            # or a reset mid-translation turns "1 m forward" into "1 m forward
            # plus however far EKF2 just decided we actually were".
            if self.move_target_x is not None:
                self.move_target_x += msg.delta_xy[0]
                self.move_target_y += msg.delta_xy[1]
                self.move_start_x += msg.delta_xy[0]
                self.move_start_y += msg.delta_xy[1]
            if self.hold_x is not None:
                self.get_logger().warning(
                    f"EKF2 lateral reset: delta_xy=({msg.delta_xy[0]:+.2f}, "
                    f"{msg.delta_xy[1]:+.2f}) m, shifted x/y hold to match.")

    def _apply_heading_reset(self, delta):
        """EKF2 has just moved its idea of north; move ours with it.

        `heading` in VehicleLocalPosition jumps by delta_heading when the
        estimator re-datums yaw -- mag fusion coming in after takeoff is the
        usual trigger indoors, where the airframe's own current is a decent
        fraction of the earth field. The physical aircraft did not move. But
        yaw_setpoint is an ABSOLUTE heading in that same frame, so if it is
        left alone it now describes a direction the airframe is no longer
        pointing, and PX4 obligingly spins to it at full yaw rate.

        Shifting it by the same delta means the commanded heading still names
        the direction the nose is actually pointing, and nothing turns.
        Everything else that latched a heading is a subclass's, which is what
        _on_heading_reset is for.
        """
        if abs(delta) < 1e-6:
            return
        self.home_yaw = wrap_pi(self.home_yaw + delta)
        self.yaw_setpoint = wrap_pi(self.yaw_setpoint + delta)
        self._on_heading_reset(delta)
        self.get_logger().warning(
            f"EKF2 HEADING reset: delta={math.degrees(delta):+.1f} deg. "
            "Shifted the commanded yaw with it, so the airframe holds the "
            "direction it is actually pointing instead of spinning to the old "
            "setpoint.")

    def _on_heading_reset(self, delta):
        """Hook: a subclass that latched a heading of its own fixes it here."""

    def estimator_flags_callback(self, msg):
        self.estimator_flags = msg

    def failsafe_flags_callback(self, msg):
        """Log PX4's failsafe conditions as they change.

        PX4 does not tell the offboard node why it took the aircraft; it just
        changes nav_state. These flags are the why.
        """
        self.failsafe_flags = msg
        active = self.active_failsafes()
        if active == self._last_failsafes:
            return
        appeared = [f for f in active if f not in self._last_failsafes]
        cleared = [f for f in self._last_failsafes if f not in active]
        self._last_failsafes = active
        if appeared:
            real = [f for f in appeared if f not in EXPECTED_FAILSAFE_FLAGS]
            expected = [f for f in appeared if f in EXPECTED_FAILSAFE_FLAGS]
            if expected:
                self.get_logger().info(
                    "PX4 failsafe (expected on this airframe, harmless): "
                    + ", ".join(expected))
            if real:
                self.get_logger().error("PX4 failsafe SET: " + ", ".join(real))
        if cleared:
            self.get_logger().info("PX4 failsafe cleared: " + ", ".join(cleared))

    def active_failsafes(self):
        f = self.failsafe_flags
        if f is None:
            return []
        active = [n for n in FAILSAFE_FLAG_NAMES if getattr(f, n, False)]
        warning = getattr(f, 'battery_warning', 0)
        if warning:
            active.append(f"battery_warning={warning}")
        return active

    def failsafe_summary(self):
        if self.failsafe_flags is None:
            return "/fmu/out/failsafe_flags is not being published"
        active = self.active_failsafes()
        return ", ".join(active) if active else "none active"

    def land_detected_callback(self, msg):
        self.landed = msg.landed
        self.land_detector_seen = True

    def rangefinder_is_healthy(self):
        """Is EKF2 actually fusing the downward rangefinder?

        NOT the same question as VehicleLocalPosition.dist_bottom_valid, which
        on PX4 up to and including v1.17 is just isTerrainEstimateValid():

            EKF2.cpp:1622   lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid();

        With EKF2_HGT_REF = 2 (Range) that flag can NEVER be true, whatever the
        sensor does, because the terrain state is not estimated at all in that
        configuration -- the ground IS the height datum, so terrain is pinned
        to zero and both terrain aiding paths are switched off by construction.

        EstimatorStatusFlags answers it directly. Fall back to dist_bottom_valid
        only if those flags are not being published.

        cs_rng_kin_consistent IS required, and that is not negotiable, because
        it is the switch that actually gates fusion:

            range_height_control.cpp:208
                if (_range_sensor.isDataHealthy()
                    && _control_status.flags.rng_kin_consistent) {
                        fuseHaglRng(...);
                }

        With it false, no range measurement is ever fused, cs_rng_hgt stays
        true, and the height estimate quietly free-runs on integrated
        accelerometer data -- which is exactly how a vehicle sitting on the
        ground once reported 2.9 m while never leaving the floor. The flag
        starts true and can only be re-earned at |vz| > 0.5 m/s, so if it is
        false on the ground the vehicle genuinely cannot be flown safely until
        PX4 is rebooted or it is flown up and down manually. Refusing to arm is
        the correct answer, not an inconvenience.

        See offboard_translate.py for the full annotated version of this note.
        """
        if not self.RANGEFINDER_CHECKS:
            return True
        f = self.estimator_flags
        if f is None:
            lp = self.local_position
            return lp is not None and lp.dist_bottom_valid
        # getattr: older px4_msgs (e.g. a laptop SITL build) lack some of
        # these fields. A missing one reads as the benign value.
        flag = lambda name, default: bool(getattr(f, name, default))  # noqa: E731
        return ((flag('cs_rng_hgt', False) or flag('cs_rng_terrain', False))
                and not flag('cs_rng_fault', False)
                and not flag('cs_rng_stuck', False)
                and flag('cs_rng_kin_consistent', True))

    def position_is_usable(self):
        """What we need to fly at all: a height estimate and a rangefinder.

        Deliberately does NOT require xy_valid. On a flow-only airframe the
        lateral estimate cannot converge until the vehicle is off the ground
        and the flow sensor can see motion -- demanding xy_valid before arming
        is a chicken-and-egg that only passes by luck.
        """
        lp = self.local_position
        return lp is not None and lp.z_valid and self.rangefinder_is_healthy()

    def flow_is_healthy(self):
        """Is the optical flow actually correcting, or just dead-reckoning?

        xy_valid alone is not enough -- EKF2 keeps it true while coasting on
        the IMU. Require a live velocity estimate and a rangefinder reading
        far enough off the ground for the flow to see anything.
        """
        lp = self.local_position
        return (lp is not None and lp.xy_valid and lp.v_xy_valid
                and self.rangefinder_is_healthy()
                and lp.dist_bottom > self.FLOW_MIN_AGL)

    def flow_healthy_for(self):
        """Seconds the flow has been continuously healthy, 0.0 if it is not."""
        if not self.flow_is_healthy():
            self.flow_healthy_since = None
            return 0.0
        if self.flow_healthy_since is None:
            self.flow_healthy_since = time.monotonic()
        return time.monotonic() - self.flow_healthy_since

    def relative_altitude(self):
        """Height above the arming point, positive up. None if unknown."""
        if self.home_z is None or self.local_position is None:
            return None
        return self.home_z - self.local_position.z

    def agl(self):
        """Height above ground straight from the lidar, or None.

        Independent of the EKF's height datum, so it survives an estimator
        reset that would corrupt relative_altitude().
        """
        lp = self.local_position
        if lp is None or not self.rangefinder_is_healthy():
            return None
        return lp.dist_bottom

    def is_airborne(self):
        """Conservative: only true when something concrete says we left ground."""
        if self.land_detector_seen and self.landed:
            return False
        agl = self.agl()
        if agl is not None:
            return agl > self.LIFTOFF_AGL
        return not self.landed

    def log_flight_state(self):
        lp = self.local_position
        if lp is None:
            self.get_logger().warning("No VehicleLocalPosition being published at all.",
                                      throttle_duration_sec=2.0)
            return

        alt = self.relative_altitude()
        alt_str = f"{alt:+.2f} m" if alt is not None else "n/a"
        self.get_logger().info(
            f"alt={alt_str} airborne={self.is_airborne()} "
            f"xy_valid={lp.xy_valid} z_valid={lp.z_valid} | "
            f"dist_bottom={lp.dist_bottom:.2f} m rng_ok={self.rangefinder_is_healthy()} "
            f"vz={lp.vz:+.2f} m/s landed={self.landed} | "
            f"xy={'POS-HOLD' if self.hold_xy else 'VEL-HOLD'} "
            f"flow_ok={self.flow_is_healthy()} "
            f"vxy=({lp.vx:+.2f},{lp.vy:+.2f}) | "
            f"hdg={math.degrees(lp.heading):+.0f} deg "
            f"yaw_sp={math.degrees(self.yaw_setpoint):+.0f} deg",
            throttle_duration_sec=1.0)

    # -------------------------------------------------------------- keyboard

    def _keyboard_listener(self):
        if not sys.stdin.isatty():
            self.get_logger().warning("stdin is not a tty; q/k abort is unavailable.")
            return

        fd = sys.stdin.fileno()
        try:
            self._stdin_is_tty = True
            self._stdin_old_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)

            while not self._stop_event.is_set():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue
                ch = sys.stdin.read(1).lower()
                if ch == 'q':
                    self.abort_requested = True
                    self.get_logger().warning("Abort requested: descending now.")
                elif ch == 'k':
                    self.kill_requested = True
                    self.get_logger().error("FORCE DISARM requested from keyboard.")
                    return
        except Exception as exc:
            self.get_logger().warning(f"Keyboard listener disabled: {exc}")
        finally:
            self._restore_stdin()

    def _restore_stdin(self):
        if self._stdin_is_tty and self._stdin_old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN,
                                  self._stdin_old_settings)
            except Exception:
                pass

    # ------------------------------------------------------------ state m/c

    def _enter_stage(self, stage):
        if self.current_stage != stage:
            self.current_stage = stage
            self.stage_enter_time = time.monotonic()

    def _in_stage_for(self):
        return time.monotonic() - self.stage_enter_time

    def _restart_stage_clock(self):
        """Reset the per-stage timeout without leaving the stage.

        A step begins while we are already in STEP, so _enter_stage() would
        no-op and the step would inherit whatever time the previous phase --
        typically the wait for a flow-healthy x/y latch -- had already burned
        off its timeout.
        """
        self.stage_enter_time = time.monotonic()

    def _begin_landing(self, reason):
        self.get_logger().warning(f"Landing: {reason}")
        self.landed_since = None
        self.stall_since = None
        # Whatever we were doing horizontally, stop walking the setpoint.
        self.moving = False
        self.yaw_remaining = 0.0
        self._update_blind_descent()
        # Flow degrades as we approach the ground, so stop chasing a latched
        # x/y point and go back to "just don't translate".
        if self.hold_xy:
            self.get_logger().info("Horizontal control -> zero-velocity hold for descent.")
        self.hold_xy = False
        self._enter_stage(self.LANDING)

    def timer_callback(self):
        # Heartbeat + setpoint go out on every tick while we still hold the
        # aircraft, so Offboard never times out mid-flight. Once we have stood
        # down they stop: continuing to offer setpoints to a vehicle PX4 or the
        # pilot has taken back is how you get handed it again unexpectedly.
        if self.stream_setpoints:
            self.publish_offboard_control_mode()
            self.publish_position_setpoint()
        self.publish_status()

        if self.kill_requested and self.current_stage not in (self.KILLING, self.DONE):
            self._enter_stage(self.KILLING)
        elif self.abort_requested:
            if self.current_stage in (self.GROUND_WAIT, self.TAKEOFF, self.HOLD,
                                      self.STEP, self.STEP_HOLD, self.POST_HOLD):
                self.abort_requested = False
                self._begin_landing("operator abort")
            elif self.current_stage in (self.PREPARATION, self.OFFBOARD_REQUEST,
                                        self.ARMING):
                # Nothing is flying yet, so there is nothing to descend from.
                self.abort_requested = False
                self.get_logger().warning("Abort before takeoff: standing down.")
                self._enter_stage(self.DISARMING)

        handler = {
            self.PREPARATION: self._handle_preparation,
            self.OFFBOARD_REQUEST: self._handle_offboard_request,
            self.ARMING: self._handle_arming,
            self.GROUND_WAIT: self._handle_ground_wait,
            self.TAKEOFF: self._handle_takeoff,
            self.HOLD: self._handle_hold,
            self.STEP: self._handle_step,
            self.STEP_HOLD: self._handle_step_hold,
            self.POST_HOLD: self._handle_post_hold,
            self.LANDING: self._handle_landing,
            self.DISARMING: self._handle_disarming,
            self.KILLING: self._handle_killing,
            self.DONE: self._handle_done,
        }[self.current_stage]
        handler()

    def _handle_preparation(self):
        if not self.status_received:
            self.get_logger().info("Waiting for VehicleStatus from PX4...",
                                   throttle_duration_sec=2.0)
            return

        if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().error("Vehicle already armed at startup. Disarming.")
            self._enter_stage(self.DISARMING)
            return

        # No height estimate or no rangefinder, no flight. The one hard gate.
        if not self.position_is_usable():
            lp = self.local_position
            reason = "no VehicleLocalPosition yet"
            if lp is not None:
                if not lp.z_valid:
                    reason = "z_valid is false (no height estimate)"
                else:
                    f = self.estimator_flags
                    if f is None:
                        reason = ("rangefinder unusable: dist_bottom_valid is false "
                                  "and /fmu/out/estimator_status_flags is not being "
                                  "published, so there is no second opinion")
                    elif f.cs_rng_fault:
                        reason = "EKF2 has declared the rangefinder FAULTY (cs_rng_fault)"
                    elif f.cs_rng_stuck:
                        reason = "rangefinder data is stuck (cs_rng_stuck)"
                    elif not f.cs_rng_kin_consistent:
                        reason = ("rangefinder is NOT being fused (cs_rng_kin_consistent "
                                  "false) -- the height estimate is unanchored and would "
                                  "free-run. Reboot the flight controller, or fly it up "
                                  "and down faster than 0.5 m/s in Position mode to let "
                                  "the check re-latch")
                    elif not (f.cs_rng_hgt or getattr(f, "cs_rng_terrain", False)):
                        reason = ("EKF2 is not fusing the rangefinder at all "
                                  "(cs_rng_hgt and cs_rng_terrain both false) -- "
                                  "check EKF2_RNG_CTRL and that the sensor is on the bus")
            self.get_logger().error(
                f"Not arming: {reason}.", throttle_duration_sec=2.0)
            self.log_flight_state()
            self.setpoint_counter = 0
            return

        self.log_flight_state()

        self.setpoint_counter += 1
        if self.setpoint_counter >= self.SETPOINT_WARMUP:
            self.get_logger().info("Setpoint stream established, position estimate healthy.")
            self._enter_stage(self.OFFBOARD_REQUEST)

    def _handle_offboard_request(self):
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info("Offboard mode active.")
            self._enter_stage(self.ARMING)
            return

        if self.REQUEST_OFFBOARD_FROM_ROS:
            self.get_logger().info("Requesting Offboard mode...", throttle_duration_sec=1.0)
            # param1 = 1 -> custom mode enabled, param2 = 6 -> PX4 OFFBOARD
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        else:
            self.get_logger().info("Waiting for you to flip the Offboard switch on the TX...",
                                   throttle_duration_sec=2.0)

        # The timeout only applies when WE are the ones requesting the mode.
        if (self.REQUEST_OFFBOARD_FROM_ROS
                and self._in_stage_for() > self.OFFBOARD_TIMEOUT):
            self.get_logger().error("Offboard mode not entered in time. Aborting.")
            self.kill_requested = True
            self._enter_stage(self.KILLING)

    def _handle_arming(self):
        if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self._capture_home()
            self.get_logger().warning(
                f"ARMED. Holding on the ground for {self.GROUND_WAIT_SECONDS:.0f} s.")
            self._enter_stage(self.GROUND_WAIT)
            return

        # Lost Offboard before we got the chance to arm.
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().error(
                f"Dropped out of Offboard into {nav_state_name(self.nav_state)} "
                f"before arming completed. PX4 failsafe: {self.failsafe_summary()}.")
            self.kill_requested = True
            self._enter_stage(self.KILLING)
            return

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        if self._in_stage_for() > self.ARMING_TIMEOUT:
            self.get_logger().error("Arming rejected / timed out. Aborting.")
            self.kill_requested = True
            self._enter_stage(self.KILLING)

    def _capture_home(self):
        # z and yaw are what is actually flown from this. x/y are recorded for
        # logging only -- the ground x/y estimate is not trustworthy enough
        # to be a setpoint (see the module docstring).
        lp = self.local_position
        self.home_x = lp.x
        self.home_y = lp.y
        self.home_z = lp.z
        self.home_yaw = lp.heading
        self.yaw_setpoint = lp.heading
        self.target_z = lp.z
        self.setpoint_z = lp.z
        self.get_logger().info(
            f"Arming point captured: x={self.home_x:.2f} y={self.home_y:.2f} "
            f"z={self.home_z:.2f} yaw={math.degrees(self.home_yaw):+.0f} deg. "
            f"'forward' means this heading for the whole flight."
            if self.DIRECTION_FRAME == 'home' else
            f"Arming point captured: x={self.home_x:.2f} y={self.home_y:.2f} "
            f"z={self.home_z:.2f} yaw={math.degrees(self.home_yaw):+.0f} deg. "
            f"Directions follow the commanded yaw (direction_frame=current).")

    def _handle_ground_wait(self):
        if not self._still_flyable():
            return

        remaining = self.GROUND_WAIT_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            # Re-latch the height datum immediately before the climb rather
            # than trusting the one from arming: less time for drift, and any
            # reset during the ground wait is already behind us.
            self.home_z = self.local_position.z
            self.setpoint_z = self.home_z
            self.commanded_altitude = self.TAKEOFF_ALTITUDE
            self.target_z = self.home_z - self.commanded_altitude
            self.in_band_since = None
            self.get_logger().warning(
                f"Climbing to {self.TAKEOFF_ALTITUDE:.2f} m "
                f"(datum z={self.home_z:.2f}).")
            self._enter_stage(self.TAKEOFF)
            return

        # Hold the setpoint BELOW ground level. At home_z exactly, the
        # controller sits at near-hover thrust for the whole wait and the
        # vehicle skitters on the edge of liftoff, which is what makes it roll
        # off when it finally goes. Pressing down keeps it planted.
        self.target_z = self.home_z + self.GROUND_PRESS
        self.get_logger().info(f"Armed on the ground, {remaining:.1f} s to takeoff...",
                               throttle_duration_sec=1.0)
        self.log_flight_state()

    def _handle_takeoff(self):
        if not self._still_flyable():
            return

        # Anchor x/y AS SOON AS the flow is usable, which is partway up the
        # climb rather than at the top of it.
        #
        # Without this the whole climb is flown on the zero-velocity branch of
        # publish_position_setpoint -- position [nan, nan, z], velocity
        # [0, 0, vz] -- which has no position term at all. Below FLOW_MIN_AGL
        # the flow is not fused, so EKF2 is dead-reckoning on the IMU, and any
        # residual velocity bias integrates into a translation that nothing
        # ever undoes: the vehicle leaves the pad in some direction, and the
        # first thing that anchors it is the latch at the top of the climb,
        # by which time it is already metres away. Latching here stops that
        # drift the moment there is an estimate good enough to stop it with.
        #
        # If the drift so far is small, walk the held point back to the arming
        # position with the inherited carrot rather than jumping to it -- so
        # the vehicle returns over the pad instead of merely stopping wherever
        # the drift left it. Beyond TAKEOFF_ANCHOR_MAX_DRIFT the arming point
        # is not trusted (that much apparent motion during a climb is an
        # estimate problem, not a real translation) and we simply hold here.
        latched_now = not self.hold_xy
        self._try_latch_xy_hold()
        if latched_now and self.hold_xy and self.home_x is not None:
            drift = math.hypot(self.hold_x - self.home_x, self.hold_y - self.home_y)
            if drift > self.TAKEOFF_ANCHOR_DEADBAND and not self.TAKEOFF_RETURN_TO_PAD:
                self.get_logger().warning(
                    f"Drifted {drift:.2f} m during the climb; holding here "
                    "rather than flying back to the arming point "
                    "(takeoff_return_to_pad is false). If this number is "
                    "large every flight, the drift is real: check "
                    "SENS_FLOW_ROT, the flow mounting and the floor texture.")
            elif drift > self.TAKEOFF_ANCHOR_DEADBAND:
                if drift <= self.TAKEOFF_ANCHOR_MAX_DRIFT:
                    self.move_target_x = self.home_x
                    self.move_target_y = self.home_y
                    self.moving = True
                    self.get_logger().warning(
                        f"Drifted {drift:.2f} m during the climb; walking back "
                        "to the arming point.")
                else:
                    self.get_logger().error(
                        f"Drifted {drift:.2f} m during the climb -- more than "
                        f"the {self.TAKEOFF_ANCHOR_MAX_DRIFT:.2f} m that is "
                        "credible. Holding here instead of flying back. CHECK "
                        "SENS_FLOW_ROT AND THE FLOW MOUNTING.")

        self.log_flight_state()

        # Runaway guard. The arrival test only fires INSIDE a +/-8 cm band, so
        # a vehicle that blows through the target is never "there" and would
        # otherwise keep climbing for the whole TAKEOFF_TIMEOUT.
        alt = self.relative_altitude()
        if alt is not None and alt > self.commanded_altitude + self.OVERSHOOT_ABORT:
            self._begin_landing(
                f"climb overshot: {alt:.2f} m vs {self.commanded_altitude:.2f} m target")
            return

        if self._at_commanded_altitude():
            if self.in_band_since is None:
                self.in_band_since = time.monotonic()
            elif time.monotonic() - self.in_band_since >= self.SETTLE_SECONDS:
                alt = self.relative_altitude()
                self.get_logger().warning(
                    f"Reached {alt:.2f} m. Holding for {self.HOLD_SECONDS:.0f} s.")
                self._enter_stage(self.HOLD)
            return

        self.in_band_since = None

        if self._in_stage_for() > self.TAKEOFF_TIMEOUT:
            self._begin_landing("takeoff did not settle in time")

    def _at_commanded_altitude(self):
        """Three independent things must agree before we believe we arrived.

        The EKF-relative altitude alone is not enough. A height reset can put
        it at the target while the vehicle has not moved, which previously let
        the node "arrive" at 0.80 m sitting on the ground and start its hold
        countdown with the drone still on its feet.
        """
        # 1. Something concrete says we are off the ground.
        if not self.is_airborne():
            return False

        # 2. The EKF-relative altitude is in the band.
        alt = self.relative_altitude()
        if alt is None or abs(alt - self.commanded_altitude) > self.ALTITUDE_TOLERANCE:
            return False

        # 3. The lidar, which knows nothing of the EKF datum, roughly agrees.
        # Wider band than the EKF check: the ground is not perfectly flat and
        # this is a cross-check, not the primary measurement.
        agl = self.agl()
        if agl is not None and abs(agl - self.commanded_altitude) > 4 * self.ALTITUDE_TOLERANCE:
            self.get_logger().warning(
                f"Altitude disagreement: ekf={alt:.2f} m lidar={agl:.2f} m. "
                "Not accepting arrival.", throttle_duration_sec=2.0)
            return False

        return True

    def _handle_hold(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()

        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self.step_index = 0
            self.step_started = False
            self._enter_stage(self.STEP)
            return

        self.get_logger().info(f"Holding, {remaining:.1f} s remaining...",
                               throttle_duration_sec=1.0)
        self.log_flight_state()

    # ----------------------------------------------------------- the sequence

    def current_step(self):
        if self.step_index < len(self.steps):
            return self.steps[self.step_index]
        return None

    def _finish_step(self, outcome):
        """Record how a step went and hold before starting the next one."""
        step = self.current_step()
        self.step_results.append(f"{step} -> {outcome}")
        self.get_logger().warning(
            f"Step {self.step_index + 1}/{len(self.steps)} ({step}): {outcome}")
        self.moving = False
        self.yaw_remaining = 0.0
        self.step_started = False
        self._enter_stage(self.STEP_HOLD)

    def _handle_step(self):
        if not self._still_flyable():
            return

        step = self.current_step()
        if step is None:
            self._enter_stage(self.POST_HOLD)
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        if step.kind == 'move':
            self._run_move_step(step)
        elif step.kind == 'alt':
            self._run_alt_step(step)
        else:
            self._run_yaw_step(step)

    def _handle_step_hold(self):
        """Settle between steps, so each one starts from a stationary vehicle.

        Without this the next step's start point is sampled while the vehicle
        is still overshooting the last one, and the errors compound down the
        sequence instead of each step correcting from where the previous one
        actually finished.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()

        remaining = self.STEP_HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self.step_index += 1
            self.step_started = False
            if self.current_step() is None:
                self._enter_stage(self.POST_HOLD)
            else:
                self._enter_stage(self.STEP)
            return

        self.get_logger().info(
            f"Settling between steps, {remaining:.1f} s remaining...",
            throttle_duration_sec=1.0)
        self.log_flight_state()

    # ------------------------------------------------------- horizontal step

    def _run_move_step(self, step):
        """Fly step.arg metres in the step's direction, on position.

        Gated on a flow-healthy x/y latch: without one there is no meaningful
        frame to express a target point in, and the honest answer is to skip
        the step rather than dead-reckon it on velocity.
        """
        if not self.hold_xy:
            if self.step_started:
                # Lost the estimate we were measuring the move against. Stop
                # where we are; do not coast onwards on a number we no longer
                # believe. The rest of the sequence still runs -- the next step
                # may well be a yaw or an altitude change, which do not need
                # the lateral estimate at all.
                self.moving = False
                self._finish_step("ABANDONED, flow lost mid-move")
                return
            if self._in_stage_for() > self.MOVE_LATCH_TIMEOUT:
                self._finish_step(
                    "SKIPPED, flow never healthy enough to latch x/y")
            else:
                self.get_logger().info(
                    "Waiting for a flow-healthy x/y hold before moving...",
                    throttle_duration_sec=1.0)
            return

        if not self.step_started:
            self._begin_move(step)
            return

        lp = self.local_position
        remaining = math.hypot(self.move_target_x - lp.x, self.move_target_y - lp.y)

        if remaining <= self.MOVE_TOLERANCE:
            if self.move_in_band_since is None:
                self.move_in_band_since = time.monotonic()
            elif time.monotonic() - self.move_in_band_since >= self.MOVE_SETTLE_SECONDS:
                travelled = math.hypot(lp.x - self.move_start_x,
                                       lp.y - self.move_start_y)
                # Park the hold exactly on the target so the settle is station
                # keeping, not a slow continuation of the move.
                self.hold_x = self.move_target_x
                self.hold_y = self.move_target_y
                self._finish_step(
                    f"done, {travelled:.2f} m travelled of {step.arg:.2f} m")
            return

        self.move_in_band_since = None
        self.get_logger().info(
            f"Moving {step.name}: {remaining:.2f} m to go.",
            throttle_duration_sec=1.0)

        if self._in_stage_for() > self.MOVE_TIMEOUT:
            # Stop pushing towards a target we are evidently not reaching, and
            # hold wherever the vehicle actually is instead.
            self.moving = False
            self.hold_x = lp.x
            self.hold_y = lp.y
            self._finish_step(f"TIMED OUT {remaining:.2f} m short")

    def _reference_yaw(self):
        """The yaw that 'forward' is measured against.

        'home' (the default) is the heading held at arming, so a yaw step in
        the middle of the sequence changes where the airframe points but not
        what 'forward' means. 'current' is the yaw currently commanded, which
        makes each move relative to whatever the last yaw step left us at.
        """
        return self.home_yaw if self.DIRECTION_FRAME == 'home' else self.yaw_setpoint

    def _begin_move(self, step):
        ref_yaw = self._reference_yaw()
        ux, uy = DIRECTIONS[step.name](math.cos(ref_yaw), math.sin(ref_yaw))

        # From the LATCHED point, not from the raw estimate: hold_x/hold_y is
        # what the vehicle is currently being commanded to, so measuring the
        # move from it is what makes the commanded distance the flown distance.
        self.move_start_x = self.hold_x
        self.move_start_y = self.hold_y
        self.move_target_x = self.hold_x + ux * step.arg
        self.move_target_y = self.hold_y + uy * step.arg
        self.move_in_band_since = None
        self.moving = True
        self.step_started = True
        self._restart_stage_clock()

        self.get_logger().warning(
            f"Step {self.step_index + 1}/{len(self.steps)}: {step.arg:.2f} m "
            f"{step.name} at {self.MOVE_SPEED:.2f} m/s, in the "
            f"{math.degrees(ref_yaw):+.0f} deg frame: "
            f"({self.move_start_x:.2f}, {self.move_start_y:.2f}) -> "
            f"({self.move_target_x:.2f}, {self.move_target_y:.2f}) NED.")

    # --------------------------------------------------------- altitude step

    def _run_alt_step(self, step):
        """Climb or descend step.arg metres (signed, positive up).

        Flown by the same z ramp as the takeoff, just with a new target, so
        the leash and the climb/descend rate limits all still apply.
        """
        if not self.step_started:
            requested = self.commanded_altitude + step.arg
            clamped = min(max(requested, self.MIN_ALTITUDE), self.MAX_ALTITUDE)
            if abs(clamped - requested) > 1e-3:
                self.get_logger().error(
                    f"Altitude step would take us to {requested:.2f} m, outside "
                    f"the {self.MIN_ALTITUDE:.2f}-{self.MAX_ALTITUDE:.2f} m "
                    f"envelope. Clamping to {clamped:.2f} m.")
            self.commanded_altitude = clamped
            self.target_z = self.home_z - self.commanded_altitude
            self.in_band_since = None
            self.step_started = True
            self._restart_stage_clock()
            self.get_logger().warning(
                f"Step {self.step_index + 1}/{len(self.steps)}: "
                f"{'up' if step.arg > 0 else 'down'} {abs(step.arg):.2f} m "
                f"to {self.commanded_altitude:.2f} m above the arming point.")
            return

        # Same runaway guard as the takeoff: the arrival test only fires inside
        # a narrow band, so without this a vehicle that sails past the target
        # is simply never "there".
        alt = self.relative_altitude()
        if alt is not None and alt > self.commanded_altitude + self.OVERSHOOT_ABORT:
            self._begin_landing(
                f"altitude step overshot: {alt:.2f} m vs "
                f"{self.commanded_altitude:.2f} m target")
            return

        if self._at_commanded_altitude():
            if self.in_band_since is None:
                self.in_band_since = time.monotonic()
            elif time.monotonic() - self.in_band_since >= self.SETTLE_SECONDS:
                self._finish_step(f"done, now at {alt:.2f} m")
            return

        self.in_band_since = None
        self.get_logger().info(
            f"Changing altitude: {alt:+.2f} m now, "
            f"{self.commanded_altitude:.2f} m wanted.",
            throttle_duration_sec=1.0)

        if self._in_stage_for() > self.ALT_STEP_TIMEOUT:
            # Freeze the ramp where the vehicle actually is rather than leaving
            # a setpoint it is evidently not reaching hanging over the rest of
            # the sequence.
            if alt is not None:
                self.commanded_altitude = alt
                self.target_z = self.local_position.z
                self.setpoint_z = self.local_position.z
            self._finish_step("TIMED OUT, holding the altitude we reached")

    # -------------------------------------------------------------- yaw step

    def _run_yaw_step(self, step):
        """Rotate step.arg radians in place.

        Walked as a ramped yaw setpoint rather than commanded as a step, for
        the same reason the translations are ramped: a step change makes PX4
        spin as fast as MC_YAWRATE_MAX allows, and a fast yaw both smears the
        optical flow and, on a vehicle holding position from that flow, turns
        a rotation into a translation.

        The requested sign is respected rather than taking the short way
        round, so `yaw -270` really does rotate 270 degrees anticlockwise.
        """
        if not self.step_started:
            self.yaw_remaining = step.arg
            self.yaw_in_band_since = None
            self.step_started = True
            self._restart_stage_clock()
            target = math.degrees(wrap_pi(self.yaw_setpoint + step.arg))
            self.get_logger().warning(
                f"Step {self.step_index + 1}/{len(self.steps)}: yaw "
                f"{math.degrees(step.arg):+.0f} deg at "
                f"{math.degrees(self.YAW_RATE):.0f} deg/s, to a heading of "
                f"{target:+.0f} deg. Holding position through the turn.")
            return

        lp = self.local_position
        error = abs(wrap_pi(self.yaw_setpoint - lp.heading))

        if abs(self.yaw_remaining) < 1e-3 and error <= self.YAW_TOLERANCE:
            if self.yaw_in_band_since is None:
                self.yaw_in_band_since = time.monotonic()
            elif time.monotonic() - self.yaw_in_band_since >= self.YAW_SETTLE_SECONDS:
                self._finish_step(
                    f"done, heading {math.degrees(lp.heading):+.0f} deg")
            return

        self.yaw_in_band_since = None
        self.get_logger().info(
            f"Yawing: {math.degrees(abs(self.yaw_remaining)):.0f} deg of setpoint "
            f"left, airframe {math.degrees(error):.0f} deg behind it.",
            throttle_duration_sec=1.0)

        if self._in_stage_for() > self.YAW_TIMEOUT:
            # Accept whatever heading we actually have and stop asking for the
            # rest, so the next step is not fighting a yaw error forever.
            self.yaw_remaining = 0.0
            self.yaw_setpoint = lp.heading
            self._finish_step(
                f"TIMED OUT at {math.degrees(lp.heading):+.0f} deg, "
                f"{math.degrees(error):.0f} deg short")

    def _step_yaw_ramp(self):
        """Walk the commanded yaw one tick towards the requested rotation."""
        if abs(self.yaw_remaining) < 1e-9:
            return

        lp = self.local_position
        # Leash: while the airframe is more than YAW_LEASH behind the commanded
        # yaw it is not keeping up, and walking further just banks up an error
        # PX4 pays back as one fast spin when it finally catches up.
        if lp is not None:
            if abs(wrap_pi(self.yaw_setpoint - lp.heading)) > self.YAW_LEASH:
                return

        step = self.YAW_RATE * 0.05
        if abs(self.yaw_remaining) <= step:
            self.yaw_setpoint = wrap_pi(self.yaw_setpoint + self.yaw_remaining)
            self.yaw_remaining = 0.0
        else:
            move = step if self.yaw_remaining > 0 else -step
            self.yaw_setpoint = wrap_pi(self.yaw_setpoint + move)
            self.yaw_remaining -= move

    # ------------------------------------------------------------- post hold

    def _handle_post_hold(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()

        remaining = self.POST_HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self.get_logger().warning(
                "Sequence complete: " + "; ".join(self.step_results))
            self._begin_landing("sequence complete")
            return

        self.get_logger().info(
            f"Holding after the last step, {remaining:.1f} s remaining...",
            throttle_duration_sec=1.0)
        self.log_flight_state()

    def _try_latch_xy_hold(self):
        """Once airborne with good flow, anchor x/y to a fresh estimate.

        Zero-velocity hold drifts slowly with the flow's velocity bias. Once
        the estimate is being corrected by flow we can do better by holding an
        actual point -- but only a point sampled up here, never the one from
        the ground. If flow drops out later we fall back to velocity hold.
        """
        if not self.hold_xy:
            if self.flow_healthy_for() >= self.FLOW_SETTLE_SECONDS:
                self.hold_x = self.local_position.x
                self.hold_y = self.local_position.y
                self.hold_xy = True
                self.get_logger().info(
                    f"Flow healthy: latching x/y hold at "
                    f"({self.hold_x:.2f}, {self.hold_y:.2f}).")
        elif not self.flow_is_healthy():
            self.hold_xy = False
            self.get_logger().warning(
                "Flow unhealthy: reverting to zero-velocity hold.")

    # --------------------------------------------------------------- landing

    def _handle_landing(self):
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Disarmed during descent. Done.")
            self._stand_down()
            return

        # If PX4 or the pilot has taken the aircraft off us mid-descent, let
        # go of it completely.
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().error(
                "Offboard lost during descent; PX4 has control now "
                f"(nav_state {nav_state_name(self.nav_state)}). Standing down. "
                f"PX4 failsafe: {self.failsafe_summary()}.")
            self._stand_down()
            return

        self._update_blind_descent()

        # Walk the setpoint below the arming point so the vehicle keeps
        # pushing down into the ground instead of hovering just above it.
        # (Ignored while blind: there the descent is flown as a velocity.)
        self.target_z = self.home_z + self.LAND_OVERSHOOT
        self.log_flight_state()

        if self._touchdown_confirmed():
            self.get_logger().warning("Touchdown detected. Disarming.")
            self._enter_stage(self.DISARMING)
            return

        if self._in_stage_for() > self.LANDING_TIMEOUT:
            # Never disarm on a timeout while we may still be in the air --
            # that is a free-fall, not a landing. Hand the aircraft to PX4's
            # own auto-land and get out of its way.
            if self.is_airborne():
                self.get_logger().error(
                    "Landing timed out and we may still be airborne. "
                    "Handing over to PX4 AUTO.LAND.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND,
                                             force=True)
                self._stand_down()
                return
            self.get_logger().error("Landing timed out on the ground. Disarming.")
            self._enter_stage(self.DISARMING)

    def _update_blind_descent(self):
        """Descend on velocity, not position, when the height estimate is gone.

        A position setpoint is a number in the estimator's frame. If z_valid
        has dropped there is no frame, and commanding `home_z + 0.5` in it is
        commanding a place that does not exist. A steady downward velocity is
        still meaningful, so that is what we fall back to.
        """
        lp = self.local_position
        blind = lp is None or not lp.z_valid
        if blind != self.blind_descent:
            self.blind_descent = blind
            if blind:
                self.get_logger().error(
                    "Height estimate invalid: descending on velocity "
                    f"({self.LAND_SPEED:.2f} m/s) instead of position.")
            else:
                self.get_logger().warning(
                    "Height estimate back: resuming position-controlled descent.")
                # Restart the ramp from where we actually are rather than from
                # a stale pre-dropout value in a frame that has since reset.
                self.setpoint_z = lp.z

    def _touchdown_confirmed(self):
        """Are we down? PX4's land detector, OR an independent geometric test.

        WHY THE LAND DETECTOR IS NOT ENOUGH ON ITS OWN
        ----------------------------------------------
        This is the bug from the last translate flight: the vehicle was
        visibly sitting on the ground and VehicleLandDetected.landed stayed
        false until LANDING_TIMEOUT expired.

        MulticopterLandDetector::_get_ground_contact_state() refuses to declare
        ground contact while it believes the vehicle is being asked to hold or
        climb. It reads the trajectory setpoint we publish and computes

            _in_descend = PX4_ISFINITE(sp.velocity[2])
                          && (sp.velocity[2] >= 0.9f * MPC_LAND_SPEED)

        and, when altitude/climb-rate control is active and _in_descend is
        false, it treats the vehicle as still flying. A pure position setpoint
        publishes velocity[2] = NaN, so _in_descend is false by construction
        no matter how firmly the vehicle is planted. That is why publish_
        position_setpoint() now sends a finite descent velocity alongside the
        position ramp while landing -- but note it only satisfies PX4's test if
        land_speed >= 0.9 * MPC_LAND_SPEED, so set MPC_LAND_SPEED at or below
        the land_speed used here (0.2 is a sensible pair for land_speed 0.15+).

        Rather than depend on getting that parameter pairing right on the day,
        the fallback below answers the question from geometry we measure
        ourselves and holds regardless:

            * the commanded z is buried well below the measured z, so we are
              definitely still pushing down and not hovering;
            * the vehicle is nevertheless not descending;
            * and the lidar says we are within a couple of hand-widths of the
              floor.

        The only thing that stops a descending aircraft in mid-air is the
        ground, so all three together for STALL_CONFIRM_SECONDS means down.
        """
        touched = self.land_detector_seen and self.landed
        if touched:
            self.stall_since = None
        else:
            if self.landed_since is not None:
                self.landed_since = None
            if self._descent_has_stalled():
                if self.stall_since is None:
                    self.stall_since = time.monotonic()
                    self.get_logger().warning(
                        "Descent has stalled against something solid; PX4's land "
                        "detector still says airborne. Confirming touchdown "
                        f"ourselves over {self.STALL_CONFIRM_SECONDS:.0f} s.")
                elif time.monotonic() - self.stall_since >= self.STALL_CONFIRM_SECONDS:
                    self.get_logger().error(
                        "Touchdown confirmed by stalled descent, NOT by PX4's "
                        "land detector. Check MPC_LAND_SPEED against land_speed "
                        "and the LNDMC_* thresholds.")
                    return True
            else:
                self.stall_since = None
            return False

        if self.landed_since is None:
            self.landed_since = time.monotonic()
        return time.monotonic() - self.landed_since >= self.LANDED_CONFIRM_SECONDS

    def _descent_has_stalled(self):
        """The independent touchdown test. See _touchdown_confirmed()."""
        lp = self.local_position
        if lp is None or not lp.z_valid or self.setpoint_z is None:
            return False

        # Are we actually still commanding downwards? Without this the test
        # would also fire on a vehicle that is simply hovering low.
        if self.setpoint_z - lp.z < self.STALL_SETPOINT_BURIED:
            return False

        if abs(lp.vz) > self.STALL_VZ:
            return False

        # Close to the floor, by the lidar if we have it and by the arming
        # datum if we do not.
        agl = self.agl()
        if agl is not None:
            return agl < self.STALL_AGL
        alt = self.relative_altitude()
        return alt is not None and alt < self.STALL_AGL

    def _handle_disarming(self):
        if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.get_logger().info("Disarmed. Flight complete.")
            if self.step_results:
                self.get_logger().info(
                    "Sequence result: " + "; ".join(self.step_results))
            self._enter_stage(self.DONE)
            return

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

        if self._in_stage_for() > self.DISARM_TIMEOUT:
            # PX4 refuses a normal disarm in the air, and that refusal is
            # correct. Force-disarming here would cut the motors on a flying
            # aircraft, so escalate only once something says we are down.
            if self.is_airborne():
                self.get_logger().error(
                    "Disarm refused and we still look airborne. Handing over to "
                    "PX4 AUTO.LAND rather than cutting the motors.",
                    throttle_duration_sec=2.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND,
                                             force=True)
                self._stand_down()
                return
            self.get_logger().error("Normal disarm ignored. Escalating to force disarm.")
            self.kill_requested = True
            self._enter_stage(self.KILLING)

    def _stand_down(self):
        """Let go of the aircraft: stop the offboard heartbeat and setpoints."""
        self.stream_setpoints = False
        self._enter_stage(self.DONE)

    def _handle_killing(self):
        if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.get_logger().warning("Aborted; vehicle is disarmed.")
            self._stand_down()
            return

        # param2 = 21196 is the PX4/MAVLink "force" magic number.
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0, param2=21196.0, force=True)

    def _handle_done(self):
        self.get_logger().info("Idle. Ctrl-C to exit.", throttle_duration_sec=5.0)

    def _still_flyable(self):
        """Common bail-outs for every stage where the vehicle is under our control."""
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warning("Vehicle disarmed by PX4. Stopping.")
            self._stand_down()
            return False

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # PX4 (or the pilot) took the aircraft off us -- stop commanding it,
            # and stop the heartbeat too so it cannot be handed back to us.
            self.get_logger().error(
                "Offboard lost; PX4 has control now "
                f"(nav_state {nav_state_name(self.nav_state)}). Standing down. "
                f"PX4 failsafe: {self.failsafe_summary()}.")
            self._stand_down()
            return False

        # In flight the bar is lower than for arming: a momentary rangefinder
        # dropout is survivable, losing the height estimate entirely is not.
        lp = self.local_position
        if lp is None or not lp.z_valid:
            self._begin_landing("height estimate went invalid")
            return False

        # The height estimate is only worth anything while the rangefinder is
        # actually being fused into it. If fusion stops mid-flight the estimate
        # free-runs on the IMU and every altitude number here becomes fiction --
        # including dist_bottom, which is the same estimate. Get down now.
        if not self.rangefinder_is_healthy():
            self._begin_landing("rangefinder fusion stopped; altitude is unanchored")
            return False

        # Ground truth. If PX4's land detector still says we are on the floor
        # while our altitude claims otherwise, believe the floor: the estimate
        # is running away and commanding more thrust will only feed it.
        if (self.land_detector_seen and self.landed
                and self.current_stage == self.TAKEOFF
                and self._in_stage_for() > 3.0):
            alt = self.relative_altitude()
            if alt is not None and alt > self.LIFTOFF_AGL:
                self._begin_landing(
                    f"altitude says {alt:.2f} m but the land detector still reports "
                    "landed -- the height estimate is running away")
                return False

        return True

    # ------------------------------------------------------------ publishers

    def publish_status(self):
        """One pipe-separated line: stage|armed|altitude|flow|detail.

        Fields are always present and always in this order so the consumer
        can split on '|' without guessing. Empty detail is an empty field.
        Same format and topic as the takeoff and translate nodes, so the LCD
        node reads this one unchanged.
        """
        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED

        detail = ''
        if self.current_stage == self.GROUND_WAIT:
            detail = f"{max(0.0, self.GROUND_WAIT_SECONDS - self._in_stage_for()):.0f}s"
        elif self.current_stage == self.HOLD:
            detail = f"{max(0.0, self.HOLD_SECONDS - self._in_stage_for()):.0f}s"
        elif self.current_stage == self.POST_HOLD:
            detail = f"{max(0.0, self.POST_HOLD_SECONDS - self._in_stage_for()):.0f}s"
        elif self.current_stage == self.TAKEOFF:
            detail = f"tgt{self.commanded_altitude:.2f}"
        elif self.current_stage == self.STEP_HOLD:
            detail = f"{self.step_index + 1}/{len(self.steps)} ok"
        elif self.current_stage == self.STEP:
            step = self.current_step()
            n = f"{self.step_index + 1}/{len(self.steps)}"
            if step is None:
                detail = n
            elif step.kind == 'move' and self.moving and self.local_position is not None:
                left = math.hypot(self.move_target_x - self.local_position.x,
                                  self.move_target_y - self.local_position.y)
                detail = f"{n} {step.name[:3]}{left:.2f}"
            elif step.kind == 'alt':
                detail = f"{n} {step.name[:2]}{self.commanded_altitude:.2f}"
            elif step.kind == 'yaw':
                detail = f"{n} yaw{math.degrees(abs(self.yaw_remaining)):.0f}"
            else:
                detail = f"{n} {step.name[:3]} wait"
        elif self.current_stage == self.OFFBOARD_REQUEST:
            detail = 'flip sw'

        msg = String()
        msg.data = "|".join([
            self.current_stage,
            'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('FLO' if self.flow_is_healthy() else '---'),
            detail,
        ])
        self.status_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # Both flags on: PX4 selects per-axis from which fields are NaN, so
        # we can fly z as a position and x/y as a velocity in one setpoint.
        msg.position = not self.blind_descent
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self):
        """Ramp z as a position; hold x/y as either a velocity or a point.

        NaN in a TrajectorySetpoint field means "do not control this axis",
        which is what lets one message mix the two.
        """
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.home_z is None:
            # Not armed yet: warm the stream up holding the live altitude and
            # zero horizontal velocity, so nothing moves when Offboard engages.
            lp = self.local_position
            if lp is None:
                return
            msg.position = [nan, nan, lp.z]
            msg.velocity = [0.0, 0.0, nan]
            msg.yaw = lp.heading
            self.trajectory_setpoint_pub.publish(msg)
            return

        if self.blind_descent:
            # No usable height estimate: no z position setpoint at all, just a
            # steady sink rate and "do not translate".
            msg.position = [nan, nan, nan]
            msg.velocity = [0.0, 0.0, self.LAND_SPEED]
            msg.yaw = self.yaw_setpoint
            self.trajectory_setpoint_pub.publish(msg)
            return

        self._step_setpoint_ramp()
        self._step_xy_ramp()
        self._step_yaw_ramp()

        # While landing, publish the sink rate as well as the position ramp.
        # PX4's land detector reads velocity[2] out of this very message and
        # will not declare ground contact while it is NaN -- see
        # _touchdown_confirmed() for the full story. As a setpoint it is only
        # a feed-forward on top of the position ramp, which is already walking
        # down at exactly this rate, so it changes nothing about the descent.
        vz = self.LAND_SPEED if self.current_stage == self.LANDING else nan

        if self.hold_xy:
            # Latched in flight on a flow-corrected estimate.
            msg.position = [self.hold_x, self.hold_y, self.setpoint_z]
            msg.velocity = [nan, nan, vz]
        else:
            # "Stay still." Memoryless: a jump in the position estimate cannot
            # be flown out, and an initial tilt off uneven ground just gets
            # corrected as soon as it produces velocity.
            msg.position = [nan, nan, self.setpoint_z]
            msg.velocity = [0.0, 0.0, vz]

        msg.yaw = self.yaw_setpoint
        self.trajectory_setpoint_pub.publish(msg)

    def _step_setpoint_ramp(self):
        """Move the commanded z one timer tick towards target_z.

        Ramping instead of jumping straight to the target keeps the climb and
        especially the descent gentle -- the vehicle chases a setpoint that is
        never more than a fraction of a metre away from it.
        """
        dt = 0.05
        descending = self.target_z > self.setpoint_z
        speed = self.LAND_SPEED if descending else self.CLIMB_SPEED
        step = speed * dt

        delta = self.target_z - self.setpoint_z
        if abs(delta) <= step:
            self.setpoint_z = self.target_z
        else:
            self.setpoint_z += step if delta > 0 else -step

        # Leash the commanded z to the measured one. PX4 spends MPC_TKO_RAMP_T
        # ramping thrust at the start of the climb, during which the vehicle
        # does not move; an unleashed ramp walks the full way to the target in
        # that time, and the position error it banks up gets flown out as a
        # lurch the moment there is thrust to do it with.
        #
        # Only bind the leash while the vehicle is not actually following. Once
        # it is moving vertically, holding the setpoint a fixed distance ahead
        # just manufactures a constant position error for PX4's velocity
        # integrator to wind up on, and that windup is paid back as overshoot.
        lp = self.local_position
        if lp is not None and lp.z_valid and abs(lp.vz) < self.LEASH_RELEASE_VZ:
            self.setpoint_z = min(max(self.setpoint_z, lp.z - self.SETPOINT_LEASH),
                                  lp.z + self.SETPOINT_LEASH)

    def _step_xy_ramp(self):
        """Walk the latched x/y hold one tick towards the move target.

        The vehicle is never commanded to the far end of the move directly.
        Instead the held point -- which is what PX4 is flying to anyway -- is
        walked there at MOVE_SPEED, so the horizontal speed is set by the
        carrot rather than by whatever MPC_XY_VEL_MAX happens to be, and the
        position error PX4 is correcting stays small the whole way.
        """
        if not self.moving or not self.hold_xy or self.move_target_x is None:
            return

        dt = 0.05
        step = self.MOVE_SPEED * dt

        dx = self.move_target_x - self.hold_x
        dy = self.move_target_y - self.hold_y
        remaining = math.hypot(dx, dy)
        if remaining <= step:
            self.hold_x = self.move_target_x
            self.hold_y = self.move_target_y
        else:
            self.hold_x += step * dx / remaining
            self.hold_y += step * dy / remaining

        # Same leash as the z ramp, for the same reason: while the vehicle is
        # still accelerating up to MOVE_SPEED the carrot would otherwise walk
        # away from it, bank up a position error, and have PX4 fly that error
        # out as an overshoot at the far end. Unlike the z leash this one stays
        # bound for the whole move -- the horizontal axes have no equivalent of
        # PX4's takeoff thrust ramp to release it after, and capping the lead
        # distance is also what stops a stuck vehicle from being dragged.
        lp = self.local_position
        if lp is not None and lp.xy_valid:
            ex = self.hold_x - lp.x
            ey = self.hold_y - lp.y
            error = math.hypot(ex, ey)
            if error > self.MOVE_LEASH:
                self.hold_x = lp.x + ex / error * self.MOVE_LEASH
                self.hold_y = lp.y + ey / error * self.MOVE_LEASH

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, force=False):
        """Send a VehicleCommand, at most once every COMMAND_INTERVAL.

        Every stage that sends a command sends it from a 20 Hz timer tick.
        Unthrottled that is 20 identical commands a second into PX4's command
        queue, which overruns it and gets commands dropped. One every 250 ms is
        still four chances a second and leaves the queue room.
        `force=True` bypasses the throttle for one-shot commands.
        """
        now = time.monotonic()
        if not force:
            last = self._last_command_time.get(command)
            if last is not None and now - last < self.COMMAND_INTERVAL:
                return
        self._last_command_time[command] = now

        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def destroy_node(self):
        self._stop_event.set()
        self._restore_stdin()
        super().destroy_node()


def spin_node(node):
    """Spin on two threads: one for the setpoint timer, one for callbacks.

    rclpy.spin() would put both on one thread and let a busy topic delay the
    heartbeat. Two threads is exactly enough for the two callback groups the
    node declares -- more would only let subscriptions run concurrently with
    each other, which buys nothing and costs the guarantee that they do not.
    """
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardSequence()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
