#!/usr/bin/env python3
"""
Offboard takeoff / hold / land test.

Sequence: stream offboard setpoints -> enter Offboard -> arm -> sit armed
on the ground for 5 s -> climb to 0.80 m above the arming point -> hold
there for 15 s -> descend slowly -> disarm once landed.

Altitude is flown as a position setpoint relative to wherever the vehicle
was standing when it armed. Horizontal is flown as a ZERO VELOCITY
setpoint, not a position setpoint: on an optical-flow airframe the x/y
position estimate on the ground is dead-reckoned garbage, and holding a
position latched down there makes the vehicle fly out the accumulated
error the moment flow starts correcting it. "Stay still" has no memory
and cannot do that.

Once the vehicle is at altitude with healthy flow, x/y position hold is
latched onto a FRESH estimate for the duration of the hold, which removes
the slow creep that a pure velocity hold has. The descent drops back to
velocity hold, because flow degrades again near the ground.

Keys:  q -> abort into a controlled descent from wherever we are.
       k -> force-disarm immediately (motors cut, vehicle drops).

The vertical estimate must be healthy for this to be safe. The node
refuses to arm without z_valid.
"""

import select
import sys
import threading
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import (
    BatteryStatus,
    DistanceSensor,
    EstimatorStatusFlags,
    FailsafeFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)


# PX4's nav_state is an integer in the log and unreadable at 3 a.m. on a
# flight line. Built from the message constants rather than hard-coded so it
# cannot drift out of date with px4_msgs.
NAV_STATE_NAMES = {
    getattr(VehicleStatus, _n): _n[len('NAVIGATION_STATE_'):]
    for _n in dir(VehicleStatus) if _n.startswith('NAVIGATION_STATE_')
}


# MAV_RESULT, so a rejected command reads as DENIED rather than as a bare 2.
CMD_RESULT_NAMES = {
    getattr(VehicleCommandAck, _n): _n[len('VEHICLE_CMD_RESULT_'):]
    for _n in dir(VehicleCommandAck) if _n.startswith('VEHICLE_CMD_RESULT_')
}


def cmd_result_name(value):
    return f"{CMD_RESULT_NAMES.get(value, 'UNKNOWN')}({value})"


def nav_state_name(value):
    return f"{NAV_STATE_NAMES.get(value, 'UNKNOWN')}({value})"


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
# None of them is one of Offboard's mode requirements (which are only angular
# velocity, attitude and offboard signal -- see mode_requirements.cpp), so
# none can take the aircraft off us:
#
#   home_position_invalid  PX4 sets home from GPS. There is no GPS, so there
#                          is never a home position. Only RTL consumes it, and
#                          RTL is not available on this vehicle anyway.
#   gcs_connection_lost    No ground station is connected to the FCU. Normal
#                          when flying from the companion computer alone.
#
# Still tracked and still reported in the "PX4 failsafe:" summary line, just
# not shouted about as if something had gone wrong.
EXPECTED_FAILSAFE_FLAGS = (
    'home_position_invalid',
    'gcs_connection_lost',
)


class OffboardTakeoff(Node):

    PREPARATION = "PREPARATION"
    OFFBOARD_REQUEST = "OFFBOARD_REQUEST"
    ARMING = "ARMING"
    GROUND_WAIT = "GROUND_WAIT"
    TAKEOFF = "TAKEOFF"
    HOLD = "HOLD"
    LANDING = "LANDING"
    DISARMING = "DISARMING"
    KILLING = "KILLING"
    DONE = "DONE"

    # ---- flight parameters -----------------------------------------------
    TAKEOFF_ALTITUDE = 0.80     # m above the arming point
    GROUND_WAIT_SECONDS = 5.0   # armed on the ground before the climb starts
    HOLD_SECONDS = 15.0         # station keeping once the altitude is reached
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
    OVERSHOOT_ABORT = 0.50      # m above the target before we call the climb a
                                # runaway and land. Without this the node will
                                # happily watch the vehicle sail past the target
                                # until TAKEOFF_TIMEOUT expires -- which is what
                                # let a 1.00 m takeoff reach 2.9 m.
    LEASH_RELEASE_VZ = 0.10     # m/s. Above this the vehicle is tracking, so the
                                # leash lets go (see _step_setpoint_ramp).
    SETPOINT_LEASH = 0.60       # m the commanded z may lead the measured z by.
                                # Without this the ramp keeps walking while the
                                # vehicle is still thrust-limited on PX4's
                                # takeoff ramp, builds a big position error, and
                                # then flies it out in one jump.

    # ---- flow / estimator health -----------------------------------------
    # Minimum AGL at which the optical flow output is treated as meaningful.
    # CORRECTION (2026-09-13): this was 0.50, on the same wrong premise as the
    # old 0.40 m lidar minimum. Neither sensor has a 0.40 m floor -- the flow
    # works from 0.10 m, which is also the vehicle's standing height, so that
    # is the bound used here.
    FLOW_MIN_AGL = 0.10

    # ---- lidar validity ----------------------------------------------------
    # CORRECTION (2026-09-13): earlier versions of this file claimed the TFmini
    # had a 0.40 m minimum range and was therefore blind while the vehicle sat
    # on the ground. That was wrong. This TFmini Plus reads accurately below
    # 0.10 m, which is the vehicle's own standing height, so the lidar can see
    # the floor from the moment it powers up. There is no dead zone, and there
    # is no phase of the flight in which a silent lidar is expected.
    #
    # Every concession that premise bought has been removed with it. A valid
    # lidar reading is now a hard precondition for arming and a continuous
    # requirement in flight, which is a much stronger gate than the one it
    # replaces: the 2026-09-13 runaway (5.27 m logged, lidar=n/a on every
    # line, vehicle never off the floor) cannot get past preflight now.
    #
    # The message's own min_distance is deliberately NOT used to judge a
    # reading. PX4's driver declares a conservative family-wide minimum and
    # stamps signal_quality = 0 on anything under it, and believing that field
    # is precisely what threw away every true 0.10 m reading in the runaway
    # log. We bound the reading ourselves instead.
    LIDAR_MIN_VALID = 0.03      # m. Below this a return is coming off the
                                # sensor housing or is a no-return reported as
                                # zero, not a measurement of the floor.
    LIDAR_MAX_VALID = 12.0      # m, TFmini Plus spec. Only used when the
                                # driver does not declare a sane max_distance.
    # Dropping the signal_quality veto leaves "sensor is not actually
    # measuring" to be caught elsewhere. That job belongs to EKF2's own
    # cs_rng_stuck flag (see rangefinder_is_healthy) and to the divergence
    # cross-check below, both of which look at whether the reading MOVES.
    # A timer on an unchanging reading would be wrong here: parked on the
    # floor, a healthy 1 cm-resolution lidar reports exactly 0.10 m over and
    # over, and that is correct behaviour, not a stuck sensor.

    # ---- raw rangefinder cross-check --------------------------------------
    # Everything in VehicleLocalPosition -- z, vz AND dist_bottom -- is one
    # EKF state. They cannot disagree with each other, so they cannot catch
    # the EKF free-running. /fmu/out/distance_sensor is the driver's reading
    # BEFORE the estimator sees it, and is the only independent opinion we
    # have about where the ground is. Everything below hangs off it.
    RAW_RANGE_TIMEOUT = 1.0     # s without a raw sample -> the lidar is gone
    RANGE_DIVERGENCE_MAX = 0.35 # m the EKF height may differ from the lidar
    RANGE_DIVERGENCE_SECONDS = 0.5  # ...continuously, before we call it a runaway
    GROUND_TRUTH_AGL = 0.25     # m. Below this the lidar says we are on the
                                # floor, whatever the estimator claims. The
                                # vehicle stands 0.10 m tall, so parked it
                                # reads well inside this.
    # x/y position hold is only latched after flow has been continuously
    # healthy this long, so a single good sample cannot trigger it.
    FLOW_SETTLE_SECONDS = 1.0

    # ---- timings / limits -------------------------------------------------
    SETPOINT_WARMUP = 20        # setpoints streamed before requesting Offboard (@20 Hz = 1 s)
    OFFBOARD_TIMEOUT = 10.0
    ARMING_TIMEOUT = 10.0
    PX4_DATA_GRACE = 3.0             # s in PREPARATION before a missing PX4 topic is an error
    # Grace before a NAKed arm request is treated as final rather than as a
    # race with PX4 still finishing its checks.
    ARM_DENIAL_GRACE = 2.0
    TAKEOFF_TIMEOUT = 20.0
    LANDING_TIMEOUT = 30.0
    DISARM_TIMEOUT = 5.0
    LANDED_CONFIRM_SECONDS = 1.0    # land-detector must agree this long
    COMMAND_INTERVAL = 0.25         # s between repeats of a vehicle command.
                                    # /fmu/in/vehicle_command at the full 20 Hz
                                    # floods PX4's command queue and gets
                                    # commands dropped rather than acted on.
    # If you switch to Offboard from your RC transmitter instead of from
    # this node, set this to False.
    REQUEST_OFFBOARD_FROM_ROS = False  # the TX (or the dashboard) switches Offboard
    # ----------------------------------------------------------------------

    def __init__(self):
        super().__init__('offboard_takeoff')

        # The numbers you actually want to change between hardware tests are
        # exposed as ROS parameters; the rest stay as class constants above.
        # e.g. ros2 run ... --ros-args -p takeoff_altitude:=0.30
        # float() on the way out: launch passes parameters as YAML, so
        # `takeoff_altitude:=1` arrives as an int and an unconverted int would
        # make every altitude comparison integer-ish. (An int also gets
        # rejected outright against a double-typed declaration.)
        self.TAKEOFF_ALTITUDE = float(self._declare_number(
            'takeoff_altitude', self.TAKEOFF_ALTITUDE))
        self.HOLD_SECONDS = float(self._declare_number(
            'hold_seconds', self.HOLD_SECONDS))
        self.GROUND_WAIT_SECONDS = float(self._declare_number(
            'ground_wait_seconds', self.GROUND_WAIT_SECONDS))
        self.CLIMB_SPEED = float(self._declare_number(
            'climb_speed', self.CLIMB_SPEED))
        self.LAND_SPEED = float(self._declare_number(
            'land_speed', self.LAND_SPEED))
        self.REQUEST_OFFBOARD_FROM_ROS = bool(self.declare_parameter(
            'request_offboard_from_ros', self.REQUEST_OFFBOARD_FROM_ROS).value)
        # Opt-in escape hatch for firmware that does not bridge
        # /fmu/out/distance_sensor. Default True: you have to ask for the
        # degraded mode, it is never entered by accident. See
        # rangefinder_is_healthy() and _divergence_is_acceptable() for exactly
        # what protection you give up.
        self.REQUIRE_RAW_RANGE = bool(self.declare_parameter(
            'require_raw_range', True).value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/uav_2/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/uav_2/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/uav_2/fmu/in/trajectory_setpoint', 10)

        # Compact machine-readable status for the LCD node (and anything else
        # that wants to watch the state machine without parsing log text).
        self.status_pub = self.create_publisher(String, 'takeoff_status', 10)

        # Both of these are versioned on some firmware builds and
        # unversioned on others -- it depends purely on what names the FC's
        # dds_topics.yaml gives them, and the two do not have to agree with
        # each other. Subscribing to both and taking whichever arrives is the
        # only thing that survives a reflash. Getting this wrong is silent:
        # the node simply never receives a position and sits waiting forever.
        self.vehicle_status_subs = [
            self.create_subscription(
                VehicleStatus, topic, self.vehicle_status_callback,
                qos_profile=sensor_qos)
            for topic in ('/uav_2/fmu/out/vehicle_status',
                          '/uav_2/fmu/out/vehicle_status_v1')
        ]
        self.local_position_subs = [
            self.create_subscription(
                VehicleLocalPosition, topic, self.local_position_callback,
                qos_profile=sensor_qos)
            for topic in ('/uav_2/fmu/out/vehicle_local_position',
                          '/uav_2/fmu/out/vehicle_local_position_v1')
        ]

        # Unversioned topic, so the name is the same on every firmware that
        # bridges it. This is the only place that tells us whether EKF2 is
        # actually fusing the rangefinder -- see rangefinder_is_healthy().
        self.estimator_flags_sub = self.create_subscription(
            EstimatorStatusFlags, '/uav_2/fmu/out/estimator_status_flags',
            self.estimator_flags_callback, qos_profile=sensor_qos)

        # The raw TFmini reading, straight off the driver. This is the ONLY
        # number in this node that the estimator has not already touched.
        self.distance_sensor_sub = self.create_subscription(
            DistanceSensor, '/uav_2/fmu/out/distance_sensor',
            self.distance_sensor_callback, qos_profile=sensor_qos)

        # PX4's own account of why it would take the aircraft away from us.
        # Purely diagnostic -- nothing here gates a decision -- but it is the
        # difference between "Offboard lost" and knowing WHICH condition
        # tripped, which is otherwise only visible in the ulog or QGC.
        self.failsafe_flags_sub = self.create_subscription(
            FailsafeFlags, '/uav_2/fmu/out/failsafe_flags',
            self.failsafe_flags_callback, qos_profile=sensor_qos)

        # Battery telemetry. failsafe_flags gives us battery_warning, which
        # says "critical" without ever saying WHY -- and a warning that
        # escalates under climb thrust and recovers a second after disarm is
        # sag, not depletion. Those are different problems with different
        # fixes, and the numbers below are what separates them.
        # Unversioned on some builds and _v1 on others -- subscribe to both
        # and take whichever one actually arrives. Getting this wrong is not
        # harmless: it reports as "no battery samples", which looks like a
        # missing sensor when it is really a missing subscription, and it
        # hides the battery warning that is blocking arming.
        self.battery_subs = [
            self.create_subscription(
                BatteryStatus, topic, self.battery_callback,
                qos_profile=sensor_qos)
            for topic in ('/uav_2/fmu/out/battery_status',
                          '/uav_2/fmu/out/battery_status_v1')
        ]

        # PX4's reply to every command we send. Without this an arm that PX4
        # refuses looks identical to an arm that never arrived, and the reason
        # -- which PX4 knows and states -- is only visible in QGC or the ulog.
        self.command_ack_subs = [
            self.create_subscription(
                VehicleCommandAck, topic, self.command_ack_callback,
                qos_profile=sensor_qos)
            for topic in ('/uav_2/fmu/out/vehicle_command_ack',
                          '/uav_2/fmu/out/vehicle_command_ack_v1')
        ]

        # The land detector topic is unversioned on some builds and _v1 on
        # others; subscribe to both and take whichever one actually arrives.
        self.land_detected_subs = [
            self.create_subscription(
                VehicleLandDetected, topic, self.land_detected_callback,
                qos_profile=sensor_qos)
            for topic in ('/uav_2/fmu/out/vehicle_land_detected',
                          '/uav_2/fmu/out/vehicle_land_detected_v1')
        ]

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        # Last non-accepted ack for an arm command, as a human-readable string.
        self.last_arm_denial = None
        self.status_received = False
        self._last_nav_state = None

        self.local_position = None
        self.landed = True
        self.land_detector_seen = False
        self.estimator_flags = None
        self.raw_range = None           # last DistanceSensor message
        self.raw_range_stamp = None     # time.monotonic() it arrived
        self.diverging_since = None
        self.failsafe_flags = None
        self._last_failsafes = []
        self.battery = None
        # Whole VehicleStatus, kept for pre_flight_checks_pass -- the rest of
        # the node only ever needed nav_state and arming_state.
        self.vehicle_status = None

        # EKF2 reset bookkeeping. When the estimator re-datums its height or
        # lateral position it jumps x/y/z instantly and tells us by how much.
        # Anything we latched in the old frame has to be shifted with it or it
        # silently becomes a setpoint in the wrong place.
        self._z_reset_counter = None
        self._xy_reset_counter = None

        # Captured at the moment of arming; every setpoint is relative to it.
        self.home_x = None
        self.home_y = None
        self.home_z = None
        self.home_yaw = 0.0

        self.target_z = None        # NED z the ramp is currently walking towards
        self.setpoint_z = None      # NED z actually being commanded right now
        self.in_band_since = None
        self.landed_since = None

        # Horizontal control. hold_xy False -> command zero velocity;
        # True -> hold hold_x/hold_y, latched once airborne with good flow.
        self.hold_xy = False
        self.hold_x = None
        self.hold_y = None
        self.flow_healthy_since = None

        self.setpoint_counter = 0
        self.stage_enter_time = time.monotonic()
        self.current_stage = self.PREPARATION
        self.abort_requested = False
        self.kill_requested = False
        # Set when the height estimate dies in flight: the descent is then
        # flown as a velocity, because a position setpoint against a dead
        # z estimate is a setpoint against a number that means nothing.
        self.blind_descent = False
        # Cleared once we have stood down, so a vehicle PX4 or the pilot has
        # taken back is not still being offered offboard setpoints.
        self.stream_setpoints = True
        self._last_command_time = {}

        self._stop_event = threading.Event()
        self._stdin_is_tty = False
        self._stdin_old_settings = None
        self._keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._keyboard_thread.start()

        # 20 Hz. PX4 drops Offboard if setpoints arrive slower than 2 Hz.
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().warning(
            f"Takeoff test: {self.TAKEOFF_ALTITUDE:.2f} m, "
            f"{self.HOLD_SECONDS:.0f} s hold. Press q to abort into a descent, "
            "k to force-disarm.")

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
        self.vehicle_status = msg
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
            return

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
            if self.hold_xy and self.hold_x is not None:
                self.hold_x += msg.delta_xy[0]
                self.hold_y += msg.delta_xy[1]
                self.get_logger().warning(
                    f"EKF2 lateral reset: delta_xy=({msg.delta_xy[0]:+.2f}, "
                    f"{msg.delta_xy[1]:+.2f}) m, shifted x/y hold to match.")

    def estimator_flags_callback(self, msg):
        self.estimator_flags = msg

    def battery_callback(self, msg):
        self.battery = msg

    def battery_summary(self):
        """Pack voltage, per-cell, current and PX4's own SoC estimate.

        Per-cell is the number to watch: PX4's thresholds are all relative to
        it, and under load a healthy-looking 22 V pack can be sitting at 3.4
        V/cell. current_a is what turns sag into a diagnosis -- if voltage
        collapses while current is unremarkable, the problem is a connector,
        a lead, or the divider calibration, not the cells.
        """
        b = self.battery
        if b is None:
            return "battery: no /fmu/out/battery_status samples"
        cells = b.cell_count if b.cell_count > 0 else 0
        per_cell = (b.voltage_v / cells) if cells else float('nan')
        return (f"battery: {b.voltage_v:.2f} V ({cells}S, {per_cell:.2f} V/cell) "
                f"{b.current_a:+.1f} A remaining={b.remaining * 100:.0f}% "
                f"warning={b.warning}")

    def battery_blocks_arming(self):
        """Why PX4 will refuse to arm on the battery, or None if it will not.

        PX4 fails the battery preflight check at CRITICAL and above, and that
        refusal arrives as a bare TEMPORARILY_REJECTED with no text. Checking
        it here turns a ten-second silent arming timeout into one line naming
        the pack.

        A low `remaining` with a healthy per-cell voltage is usually a wrong
        BAT_N_CELLS rather than a flat pack -- PX4 derives the percentage from
        volts per cell, so a 4S pack declared as 5S reads ~25%% low and can sit
        at EMERGENCY while the cells are fine. Both numbers are printed so the
        two cases can be told apart without a GCS.
        """
        b = self.battery
        if b is None:
            return None
        if not b.connected:
            return "PX4 reports no battery connected"
        if b.warning >= BatteryStatus.WARNING_CRITICAL:
            names = {2: 'CRITICAL', 3: 'EMERGENCY', 4: 'FAILED'}
            cells = b.cell_count if b.cell_count > 0 else 0
            per_cell = (b.voltage_v / cells) if cells else float('nan')
            return (
                f"battery warning is {names.get(b.warning, b.warning)} "
                f"({b.voltage_v:.2f} V, {cells}S = {per_cell:.2f} V/cell, "
                f"remaining={b.remaining * 100:.0f}%). PX4 fails the battery "
                f"preflight check at CRITICAL and above, so it will not arm. "
                f"If the per-cell figure looks healthy, suspect BAT_N_CELLS "
                f"rather than the pack")
        return None

    def preflight_blocks_arming(self):
        """True when PX4 itself says the arming checks do not pass.

        Purely advisory: PX4 is the authority on arming and this flag is only
        used to explain a refusal, never to veto a flight PX4 would allow.
        """
        st = self.vehicle_status
        return st is not None and not getattr(st, 'pre_flight_checks_pass', True)

    def failsafe_flags_callback(self, msg):
        """Log PX4's failsafe conditions as they change.

        PX4 does not tell the offboard node why it took the aircraft; it just
        changes nav_state. These flags are the why, and logging the edges
        means the answer is sitting in the terminal scrollback next to the
        "Offboard lost" line instead of only in the ulog.
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
            return "/uav_2/fmu/out/failsafe_flags is not being published"
        active = self.active_failsafes()
        return ", ".join(active) if active else "none active"

    def command_ack_callback(self, msg):
        # Only the rejections are worth saying out loud; an accepted command
        # already shows up as the state change it caused.
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return
        if msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.last_arm_denial = (
                f"{cmd_result_name(msg.result)}, reason={msg.result_param1}")
        self.get_logger().warning(
            f"PX4 rejected command {msg.command}: "
            f"{cmd_result_name(msg.result)} (result_param1={msg.result_param1})",
            throttle_duration_sec=2.0)

    def land_detected_callback(self, msg):
        self.landed = msg.landed
        self.land_detector_seen = True

    def distance_sensor_callback(self, msg):
        self.raw_range = msg
        self.raw_range_stamp = time.monotonic()

    def raw_range_is_live(self):
        """Is the lidar driver actually producing samples right now?"""
        return (self.raw_range is not None
                and self.raw_range_stamp is not None
                and (time.monotonic() - self.raw_range_stamp)
                < self.RAW_RANGE_TIMEOUT)

    def raw_range_top(self):
        """Largest reading we will believe, from the driver if it says."""
        m = self.raw_range
        if m is not None and m.max_distance > 0.0:
            return float(m.max_distance)
        return self.LIDAR_MAX_VALID

    def raw_agl(self):
        """Height above ground from the lidar itself, or None.

        None means "the lidar is not telling us anything usable". On this
        airframe that is ALWAYS a fault: the sensor reads accurately from well
        below the vehicle's 0.10 m standing height, so there is no height --
        parked on the floor included -- at which a working lidar has nothing
        to say. Callers are entitled to treat None as a reason not to fly.

        m.min_distance is not consulted (see LIDAR_MIN_VALID) and
        signal_quality == 0 is a warning rather than a veto, because PX4
        stamps it on readings under the driver's declared minimum and those
        readings are good on this sensor.
        """
        if not self.raw_range_is_live():
            return None
        m = self.raw_range
        if not (self.LIDAR_MIN_VALID <= m.current_distance <= self.raw_range_top()):
            return None
        if m.signal_quality == 0:
            self.get_logger().warning(
                f"lidar reports signal_quality=0 at {m.current_distance:.2f} m "
                f"(driver declares a {m.min_distance:.2f} m minimum). Using the "
                "reading anyway: this sensor is accurate below that minimum. "
                "If the number itself looks wrong, THAT is the problem.",
                throttle_duration_sec=5.0)
        return float(m.current_distance)

    def ekf_will_reject_range(self):
        """True when the current lidar sample is out of usable range.

        CORRECTION (2026-09-13): this used to compare the reading against the
        DRIVER-declared min_distance and refuse to arm below it, on the theory
        that EKF2 bins any such sample and the height then free-runs. That
        premise was wrong for this airframe. The driver's declared minimum is a
        conservative family-wide value; this TFmini Plus reads accurately from
        0.10 m, and EKF2 is observably fusing those readings (dist_bottom
        tracks the lidar, z_valid stays true). Gating arming on min_distance
        only blocked flight on perfectly good 0.10-0.12 m samples.

        Only the upper bound is judged here, and against our own band -- past
        the sensor's real maximum a return genuinely is not a measurement of
        the floor.
        """
        if not self.raw_range_is_live():
            return False
        m = self.raw_range
        return m.current_distance > self.raw_range_top()

    def on_the_ground_per_lidar(self):
        """True only when the lidar positively says we have not left the floor.

        Used to decide whether a runaway estimate can be answered by cutting
        the motors. Returns False when the lidar is silent: without it we do
        not know where the ground is and disarming would be a coin flip.
        """
        agl = self.raw_agl()
        return agl is not None and agl < self.GROUND_TRUTH_AGL

    def estimate_divergence(self):
        """metres the EKF height differs from the lidar, or None if unknowable."""
        raw = self.raw_agl()
        lp = self.local_position
        if raw is None or lp is None:
            return None
        return abs(lp.dist_bottom - raw)

    def rangefinder_is_healthy(self):
        """Is EKF2 actually fusing the downward rangefinder?

        CONFIGURATION (2026-09-13): EKF2_HGT_REF = 2 (Range) with
        EKF2_RNG_CTRL = 1. The lidar is the height reference at every altitude
        and the baro is NOT used as a datum -- it is too poor on this airframe
        to be worth falling back to. An earlier revision of this docstring
        claimed a move to Baro + conditional range aid to work around a lidar
        dead zone on the ground. There is no dead zone (see LIDAR_MIN_VALID),
        that move was never made, and every parameter table in this repo still
        specifies Range. The analysis below therefore describes the running
        configuration exactly.

        Because the lidar reads the floor from the ground up, expect this to be
        True while parked and for the whole flight. It going False is a fault
        at any height, with no low-altitude grace period -- which is what makes
        it safe to gate arming on (position_is_usable) and to abort on
        (_still_flyable).

        NOT the same question as VehicleLocalPosition.dist_bottom_valid, which
        on PX4 up to and including v1.17 is just isTerrainEstimateValid():

            EKF2.cpp:1622   lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid();

        With EKF2_HGT_REF = 2 (Range) that flag can NEVER be true, whatever the
        sensor does, because the terrain state is not estimated at all in that
        configuration -- the ground IS the height datum, so terrain is pinned
        to zero and both terrain aiding paths are switched off by construction:

          range_height_control.cpp  the do_range_aid branch sets rng_hgt = true
                                    and then calls stopRngTerrFusion(); the only
                                    place rng_terrain is ever set true sits in
                                    the `else` of `if (rng_hgt || rng_terrain)`,
                                    which is therefore unreachable from then on.
          optical_flow_control.cpp  opt_flow_terrain = opt_flow && !(hgt_ref ==
                                    RANGE)  ->  forced false.

        So dist_bottom_valid is stuck false while the rangefinder is perfectly
        healthy and is in fact the primary height source. PX4 v1.18 fixes the
        symptom by adding `|| getHeightSensorRef() == RANGE` to that line; on
        v1.17 we have to ask the question ourselves.

        EstimatorStatusFlags answers it directly. Fall back to dist_bottom_valid
        only if those flags are not being published.

        Deliberately does NOT require cs_rng_kin_consistent. That flag compares
        the rangefinder's rate of change against the EKF's vertical velocity,
        and RangeFinderConsistencyCheck::updateConsistency() can only ever set
        it back to true while the vehicle is moving vertically:

            if ((fabsf(vz) > _min_vz_for_valid_consistency)   // 0.5 m/s, fixed
                && (_test_ratio < 1.f)
                && ((time_us - _time_last_inconsistent_us) > _consistency_hyst_time_us))
                    _is_kinematically_consistent = true;

        cs_rng_kin_consistent IS required, and that is not negotiable, because
        it is the switch that actually gates fusion:

            range_height_control.cpp:208
                if (_range_sensor.isDataHealthy()
                    && _control_status.flags.rng_kin_consistent) {
                        fuseHaglRng(...);
                }

        With it false, no range measurement is ever fused. cs_rng_hgt stays
        true -- it only means "range is the intended height source" -- so the
        vehicle looks healthy while its height estimate quietly free-runs on
        integrated accelerometer data. With EKF2_HGT_REF = 2 that estimate is
        also what VehicleLocalPosition.dist_bottom reports:

            EKF2.cpp:1623   lpos.dist_bottom = math::max(_ekf.getHagl(), 0.f);

        getHagl() is terrain + altitude, and terrain is pinned to zero in this
        configuration, so dist_bottom IS the EKF altitude. It is not a second
        opinion from the lidar. When fusion stops, dist_bottom and the altitude
        drift together, agreeing perfectly with each other and with nothing
        real -- which is exactly how a vehicle sitting on the ground reported
        2.9 m while never leaving the floor.

        The flag starts true and can only be re-earned at |vz| > 0.5 m/s, so if
        it is false on the ground the vehicle genuinely cannot be flown safely
        until PX4 is rebooted or it is flown up and down manually. Refusing to
        arm is the correct answer, not an inconvenience.
        """
        # The driver has to be alive before any of the fusion flags mean
        # anything: EKF2 can report a happy, kinematically-consistent range
        # channel for some time after the sensor itself has gone quiet.
        if not self.raw_range_is_live():
            return False

        # A sample EKF2 will not accept cannot be being fused, whatever the
        # flags say about it. Checked BEFORE the flags for that reason.
        if self.ekf_will_reject_range():
            return False

        f = self.estimator_flags
        if f is None:
            # NO silent fallback to dist_bottom_valid. That flag is derived
            # from the same estimator state we are trying to check, so using
            # it here turns a missing safety input into a green light -- which
            # is exactly how this node armed and flew a phantom 10.97 m climb
            # while the vehicle never left the floor. If the flags are not
            # being published, fix the dds_topics.yaml bridge; do not fly.
            return False
        return (f.cs_rng_hgt or f.cs_rng_terrain) and not f.cs_rng_fault \
            and not f.cs_rng_stuck and f.cs_rng_kin_consistent

    def position_is_usable(self):
        """What we need to fly at all: a height estimate and a rangefinder.

        Deliberately does NOT require xy_valid. On a flow-only airframe the
        lateral estimate cannot converge until the vehicle is off the ground
        and the flow sensor can see motion -- demanding xy_valid before arming
        is a chicken-and-egg that only passes by luck. We fly x/y as a zero
        velocity setpoint anyway, so the lateral POSITION estimate is not
        something we depend on for takeoff.
        """
        lp = self.local_position
        if lp is None or not lp.z_valid:
            return False
        # A usable lidar reading is a precondition, not something to be waived
        # while we are low: this sensor sees the floor from the ground up. The
        # bypass that used to live here -- "accept a quality-0 reading because
        # we must be in the dead zone" -- is exactly the hole the 2026-09-13
        # runaway armed through.
        if self.raw_agl() is None:
            return False
        return self.rangefinder_is_healthy()

    def flow_is_healthy(self):
        """Is the optical flow actually correcting, or just dead-reckoning?

        xy_valid alone is not enough -- EKF2 keeps it true while coasting on
        the IMU. Require a live velocity estimate and a rangefinder reading
        far enough off the ground for the flow to see anything.

        The height test is the RAW lidar, not dist_bottom. With
        EKF2_HGT_REF = 2 dist_bottom is the estimator's own altitude (see
        agl()), so checking it here asked the estimator whether it was high
        enough to trust the estimator -- and it said yes all the way through
        the phantom climb, latching flow health at a claimed 0.43 m while the
        vehicle sat on the floor.
        """
        lp = self.local_position
        agl = self.raw_agl()
        return (lp is not None and lp.xy_valid and lp.v_xy_valid
                and self.rangefinder_is_healthy()
                and agl is not None and agl > self.FLOW_MIN_AGL)

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

        This used to return VehicleLocalPosition.dist_bottom, which is NOT a
        lidar reading -- with EKF2_HGT_REF = 2 it is math::max(getHagl(), 0),
        i.e. the estimator's own height with terrain pinned to zero. Sanity-
        checking the EKF altitude against it was therefore checking a number
        against itself, and it agreed perfectly all the way to 10.97 m.

        It now returns the raw DistanceSensor reading, which is genuinely
        independent of the estimator.
        """
        return self.raw_agl()

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
        # An "n/a" that could mean either "no samples at all" or "samples we
        # threw away" is what made the runaway log ambiguous for a fortnight.
        # Distinguish them, and show the number we rejected and why.
        raw = self.raw_agl()
        if raw is not None:
            raw_str = f"{raw:.2f} m"
        elif self.raw_range_is_live():
            m = self.raw_range
            raw_str = (f"REJECTED({m.current_distance:.2f} m,"
                       f"q={m.signal_quality})")
        else:
            raw_str = "no-samples"
        self.get_logger().info(
            f"alt={alt_str} airborne={self.is_airborne()} "
            f"xy_valid={lp.xy_valid} z_valid={lp.z_valid} | "
            f"dist_bottom={lp.dist_bottom:.2f} m "
            f"lidar={raw_str} "
            f"rng_ok={self.rangefinder_is_healthy()} "
            f"vz={lp.vz:+.2f} m/s landed={self.landed} | "
            f"xy={'POS-HOLD' if self.hold_xy else 'VEL-HOLD'} "
            f"flow_ok={self.flow_is_healthy()} "
            f"vxy=({lp.vx:+.2f},{lp.vy:+.2f})",
            throttle_duration_sec=1.0)
        self.get_logger().info(self.battery_summary(),
                               throttle_duration_sec=2.0)

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

    def _begin_landing(self, reason):
        self.get_logger().warning(f"Landing: {reason}")
        self.landed_since = None
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
            if self.current_stage in (self.GROUND_WAIT, self.TAKEOFF, self.HOLD):
                self.abort_requested = False
                self._begin_landing("operator abort")
            elif self.current_stage in (self.PREPARATION, self.OFFBOARD_REQUEST,
                                        self.ARMING):
                # Nothing is flying yet, so there is nothing to descend from.
                # Make sure we are disarmed and stop.
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
                elif not self.raw_range_is_live():
                    reason = ("no /fmu/out/distance_sensor samples. FIRST check "
                              "'listener distance_sensor' on the FC console: if "
                              "that shows data, the sensor is fine and the topic "
                              "simply is not bridged -- add 'distance_sensor' to "
                              "the publications: list in the firmware's "
                              "dds_topics.yaml and reflash. Note that "
                              "/uav_2/fmu/in/distance_sensor existing is NOT the same "
                              "thing: /fmu/in/ is the direction a companion "
                              "computer feeds range data INTO PX4, it is never a "
                              "readback of the FC's own sensor, and no parameter "
                              "(EKF2_HGT_REF included) can add a topic to the "
                              "bridge. If 'listener' is silent instead, it is a "
                              "driver problem -- check SENS_TFMINI_CFG points at "
                              "the right serial port")
                elif self.raw_agl() is None:
                    m = self.raw_range
                    reason = (f"lidar is publishing but the reading is unusable: "
                              f"{m.current_distance:.2f} m, outside the "
                              f"{self.LIDAR_MIN_VALID:.2f}-{self.raw_range_top():.2f} m "
                              f"band we accept (the driver declares "
                              f"{m.min_distance:.2f}-{m.max_distance:.2f} m, "
                              f"signal_quality={m.signal_quality}, which we do not "
                              f"gate on). A reading at or near 0.00 m is a no-return: "
                              f"check the lens is clean, is looking at the floor and "
                              f"not out past the edge of the bench")
                elif self.ekf_will_reject_range():
                    m = self.raw_range
                    reason = (
                        f"the lidar reads {m.current_distance:.2f} m, past the "
                        f"{self.raw_range_top():.2f} m top of its usable range, "
                        f"so the sample is not a measurement of the floor. Check "
                        f"the sensor is pointed at the ground and not off the "
                        f"edge of the bench")
                else:
                    f = self.estimator_flags
                    if f is None:
                        reason = ("/uav_2/fmu/out/estimator_status_flags is not being "
                                  "published, so there is no way to tell whether "
                                  "EKF2 is actually fusing the rangefinder. Add it "
                                  "to the firmware's dds_topics.yaml. (This node no "
                                  "longer falls back to dist_bottom_valid: that flag "
                                  "comes from the same estimator it would be "
                                  "checking, and trusting it is what allowed a "
                                  "phantom 10.97 m climb.)")
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
                    elif not (f.cs_rng_hgt or f.cs_rng_terrain):
                        reason = ("EKF2 is not fusing the rangefinder at all "
                                  "(cs_rng_hgt and cs_rng_terrain both false) -- "
                                  "check EKF2_RNG_CTRL and that the sensor is on the bus")
            # PX4's topics start arriving at different times; for the first
            # few seconds a missing one is just not here yet, not a fault.
            if self._in_stage_for() < self.PX4_DATA_GRACE:
                self.get_logger().info(
                    f"Waiting for PX4 data: {reason}.", throttle_duration_sec=2.0)
            else:
                self.get_logger().error(
                    f"Not arming: {reason}.", throttle_duration_sec=2.0)
                self.log_flight_state()
            self.setpoint_counter = 0
            return

        # Battery is a hard gate. PX4 will refuse to arm on a CRITICAL or
        # worse pack anyway; stopping here means the refusal is explained
        # once, on the ground, instead of surfacing as an arming timeout.
        blocked = self.battery_blocks_arming()
        if blocked is not None:
            self.get_logger().error(
                f"Not arming: {blocked}.", throttle_duration_sec=5.0)
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

        # The timeout only applies when WE are the ones requesting the mode --
        # if it has not taken by now it is not going to. When a human flips the
        # switch we wait indefinitely instead, so the node can be started at
        # boot and sit there until someone is actually ready to fly.
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

        # A refusal PX4 has already explained is not worth waiting out. Once
        # it has NAKed an arm request and still says the checks do not pass,
        # the remaining seconds of the timeout buy nothing but a vaguer error.
        if (self.last_arm_denial is not None
                and self.preflight_blocks_arming()
                and self._in_stage_for() > self.ARM_DENIAL_GRACE):
            self._abort_arming(
                f"PX4 refused the arm request ({self.last_arm_denial}) and "
                f"reports pre_flight_checks_pass=false")
            return

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        if self._in_stage_for() > self.ARMING_TIMEOUT:
            self._abort_arming(
                self.last_arm_denial
                or "PX4 sent no ack -- the command may not be reaching it")

    def _abort_arming(self, detail):
        """Give up on arming, saying everything we know about why.

        The three facts that between them explain every refusal this airframe
        has produced: PX4's own ack, the battery, and the failsafe flags.
        """
        blocked = self.battery_blocks_arming()
        self.get_logger().error(
            f"Arming rejected. Aborting. PX4 said: {detail}. "
            f"{self.battery_summary()}. Failsafe: {self.failsafe_summary()}.")
        if blocked is not None:
            self.get_logger().error(f"Most likely cause: {blocked}.")
        self.kill_requested = True
        self._enter_stage(self.KILLING)

    def _capture_home(self):
        # Only z and yaw are actually flown from this. x/y are recorded for
        # logging only -- the ground x/y estimate is not trustworthy enough
        # to be a setpoint (see the module docstring).
        lp = self.local_position
        self.home_x = lp.x
        self.home_y = lp.y
        self.home_z = lp.z
        self.home_yaw = lp.heading
        self.target_z = lp.z
        self.setpoint_z = lp.z
        self.get_logger().info(
            f"Arming point captured: x={self.home_x:.2f} y={self.home_y:.2f} "
            f"z={self.home_z:.2f} yaw={self.home_yaw:+.2f} rad")

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
            self.target_z = self.home_z - self.TAKEOFF_ALTITUDE
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

        self.log_flight_state()

        # Runaway guard. The arrival test only fires INSIDE a +/-8 cm band, so
        # a vehicle that blows through the target is never "there" and would
        # otherwise keep climbing for the whole TAKEOFF_TIMEOUT.
        alt = self.relative_altitude()
        if alt is not None and alt > self.TAKEOFF_ALTITUDE + self.OVERSHOOT_ABORT:
            self._begin_landing(
                f"climb overshot: {alt:.2f} m vs {self.TAKEOFF_ALTITUDE:.2f} m target")
            return

        # The dead-zone exit guard that used to sit here is gone. It waited
        # until a claimed 0.60 m before demanding a lidar reading, because
        # below that a missing reading was supposedly legitimate. Now that the
        # lidar reads from the ground up, _still_flyable() demands one on every
        # single cycle from preflight onwards, and the runaway this guard was
        # built to catch is refused arming instead of caught at 0.60 m.

        if self._at_takeoff_altitude():
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

    def _at_takeoff_altitude(self):
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
        if alt is None or abs(alt - self.TAKEOFF_ALTITUDE) > self.ALTITUDE_TOLERANCE:
            return False

        # 3. The lidar, which knows nothing of the EKF datum, roughly agrees.
        # Wider band than the EKF check: the ground is not perfectly flat and
        # this is a cross-check, not the primary measurement.
        agl = self.agl()
        if agl is not None and abs(agl - self.TAKEOFF_ALTITUDE) > 4 * self.ALTITUDE_TOLERANCE:
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
            self._begin_landing("hold complete")
            return

        self.get_logger().info(f"Holding, {remaining:.1f} s remaining...",
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

    def _handle_landing(self):
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Disarmed during descent. Done.")
            self._stand_down()
            return

        # If PX4 or the pilot has taken the aircraft off us mid-descent, let
        # go of it completely. Carrying on would mean racing the pilot for the
        # setpoint and, once LANDING_TIMEOUT expired, disarming an aircraft
        # somebody else is flying.
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().error(
                "Offboard lost during descent; PX4 has control now "
                f"(nav_state {nav_state_name(self.nav_state)}). Standing down. "
                f"PX4 failsafe: {self.failsafe_summary()}.")
            self._stand_down()
            return

        # Ground truth, which this stage had none of. _still_flyable() is not
        # called here, so nothing cross-checked the estimate during a descent:
        # the 2026-09-13 log shows a commanded landing sitting on the floor at
        # a true 0.11 m while the altitude walked from 0.31 m to 3.95 m
        # unchallenged for fifteen seconds. If the lidar can see the floor
        # under us, we are down, and there is nothing left to descend.
        if self.on_the_ground_per_lidar():
            alt = self.relative_altitude()
            alt_str = f"{alt:.2f}" if alt is not None else "n/a"
            self.get_logger().warning(
                f"Lidar reads {self.raw_agl():.2f} m -- we are on the floor, "
                f"whatever the estimate says ({alt_str} m). Disarming rather "
                "than 'descending' from an altitude we do not have.")
            self._enter_stage(self.DISARMING)
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
            # own auto-land, which has a better height estimate than we do,
            # and get out of its way.
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
        """Land detector if we have one, otherwise altitude + descent stall."""
        if self.land_detector_seen:
            touched = self.landed
        else:
            lp = self.local_position
            alt = self.relative_altitude()
            touched = (alt is not None and alt < 0.10
                       and lp is not None and abs(lp.vz) < 0.10)

        if not touched:
            self.landed_since = None
            return False

        if self.landed_since is None:
            self.landed_since = time.monotonic()
        return time.monotonic() - self.landed_since >= self.LANDED_CONFIRM_SECONDS

    def _handle_disarming(self):
        if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.get_logger().info("Disarmed. Flight complete.")
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

        # Battery. PX4 will take the aircraft off us if this gets worse, and
        # a critical pack cannot finish a climb it has already sagged out of.
        # Land while there is still charge to land ON, rather than spending it
        # pushing a setpoint the vehicle can no longer reach.
        warning = 0
        if self.failsafe_flags is not None:
            warning = getattr(self.failsafe_flags, 'battery_warning', 0)
        if warning >= 2:
            self._begin_landing(
                f"battery_warning={warning} (2=critical, 3=emergency). "
                f"{self.battery_summary()}")
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

        lp = self.local_position
        if lp is None or not lp.z_valid:
            self._begin_landing("height estimate went invalid")
            return False

        # The height estimate is only worth anything while the rangefinder is
        # actually being fused into it. If fusion stops mid-flight the estimate
        # free-runs on the IMU and every altitude number here becomes fiction --
        # including dist_bottom, which is the same estimate. Get down now.
        #
        # Two separate ways to lose the ground -- no reading, and a reading
        # that is not being fused -- both checked at every height. There is no
        # in-flight grace period and the bar is not lower than for arming: the
        # lidar reads the floor from the ground up, so neither failure has an
        # innocent explanation at any altitude.
        if self.raw_agl() is None:
            self._begin_landing(
                "lidar stopped producing a usable reading; nothing is holding "
                "the height estimate to the ground")
            return False
        if not self.rangefinder_is_healthy():
            self._begin_landing(
                "rangefinder fusion stopped; altitude is unanchored")
            return False

        # Ground truth #1: the lidar. If the EKF height and the raw range
        # disagree by more than RANGE_DIVERGENCE_MAX the estimate is running
        # away, and every altitude number in this node is fiction. Checked in
        # EVERY stage, not just TAKEOFF -- the previous version only ran during
        # the climb, and a runaway that starts after the overshoot abort has
        # already moved us to LANDING went completely unwatched.
        if not self._divergence_is_acceptable():
            return False

        # Ground truth #2: PX4's land detector. Weaker than the lidar -- it is
        # partly fed by the same estimator -- but it costs nothing to keep.
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

    def _divergence_is_acceptable(self):
        """Cross-check the EKF height against the raw lidar. False -> bailed out.

        A brief disagreement is normal: the lidar is noisy, the vehicle tilts,
        and the EKF lags. A SUSTAINED one means the estimator is no longer
        anchored to the ground, and every metre it invents gets answered with
        more thrust. RANGE_DIVERGENCE_SECONDS of hysteresis separates the two.

        The response depends on whether we can still see the floor:

          lidar says we are on the ground -> disarm. We never took off, so
              cutting the motors costs nothing, and a "descent" is meaningless
              when the vehicle has not moved. This is the case that produced
              the 10.97 m log.
          lidar says we are genuinely up -> descend. Disarming from real
              altitude is a crash; the estimate is bad but the setpoint ramp
              can still walk us down.
        """
        div = self.estimate_divergence()
        if div is None:
            # "Unknowable" used to be lumped in with "they agree", which meant
            # this cross-check switched itself off in the one situation it
            # existed for: a lidar reading nothing while the estimate ran away.
            # _still_flyable() now bails on that before ever reaching here, so
            # this can only be a sample expiring between the two calls. Say
            # nothing rather than assert agreement.
            self.diverging_since = None
            return True
        if div <= self.RANGE_DIVERGENCE_MAX:
            self.diverging_since = None
            return True

        if self.diverging_since is None:
            self.diverging_since = time.monotonic()
            return True
        if (time.monotonic() - self.diverging_since) < self.RANGE_DIVERGENCE_SECONDS:
            return True

        raw = self.raw_agl()
        alt = self.relative_altitude()
        alt_str = f"{alt:.2f}" if alt is not None else "n/a"
        detail = (f"EKF says {alt_str} m (dist_bottom "
                  f"{self.local_position.dist_bottom:.2f} m) but the lidar reads "
                  f"{raw:.2f} m -- divergence {div:.2f} m")

        if self.on_the_ground_per_lidar():
            self.get_logger().error(
                f"HEIGHT ESTIMATE RUNAWAY, vehicle is still on the ground. {detail}. "
                "Disarming rather than 'descending' from an altitude we never had.")
            self.kill_requested = True
            self._enter_stage(self.KILLING)
            return False

        self.get_logger().error(
            f"Height estimate has come unanchored. {detail}. Descending now.")
        self._begin_landing("EKF height diverged from the rangefinder")
        return False

    # ------------------------------------------------------------ publishers

    def publish_status(self):
        """One pipe-separated line: stage|armed|altitude|flow|detail.

        Fields are always present and always in this order so the consumer
        can split on '|' without guessing. Empty detail is an empty field.
        """
        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED

        detail = ''
        if self.current_stage == self.GROUND_WAIT:
            detail = f"{max(0.0, self.GROUND_WAIT_SECONDS - self._in_stage_for()):.0f}s"
        elif self.current_stage == self.HOLD:
            detail = f"{max(0.0, self.HOLD_SECONDS - self._in_stage_for()):.0f}s"
        elif self.current_stage == self.TAKEOFF:
            detail = f"tgt{self.TAKEOFF_ALTITUDE:.2f}"
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
            msg.yaw = self.home_yaw
            self.trajectory_setpoint_pub.publish(msg)
            return

        self._step_setpoint_ramp()

        if self.hold_xy:
            # Latched in flight on a flow-corrected estimate.
            msg.position = [self.hold_x, self.hold_y, self.setpoint_z]
            msg.velocity = [nan, nan, nan]
        else:
            # "Stay still." Memoryless: a jump in the position estimate cannot
            # be flown out, and an initial tilt off uneven ground just gets
            # corrected as soon as it produces velocity.
            msg.position = [nan, nan, self.setpoint_z]
            msg.velocity = [0.0, 0.0, nan]

        msg.yaw = self.home_yaw
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
        # lurch the moment there is thrust to do it with. The same clamp stops
        # the descent from burying the setpoint metres underground if the
        # vehicle hangs up on something.
        # Only bind the leash while the vehicle is not actually following. Its
        # job is the standing start, where PX4 is still ramping thrust and the
        # ramp would otherwise walk away unopposed. Once the vehicle is moving
        # vertically, holding the setpoint a fixed distance ahead of it just
        # manufactures a constant position error for PX4's velocity integrator
        # to wind up on, and that windup is paid back as overshoot at the top.
        lp = self.local_position
        if lp is not None and lp.z_valid and abs(lp.vz) < self.LEASH_RELEASE_VZ:
            self.setpoint_z = min(max(self.setpoint_z, lp.z - self.SETPOINT_LEASH),
                                  lp.z + self.SETPOINT_LEASH)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, force=False):
        """Send a VehicleCommand, at most once every COMMAND_INTERVAL.

        Every stage that sends a command sends it from a 20 Hz timer tick.
        Unthrottled that is 20 identical commands a second into PX4's command
        queue, which overruns it and gets commands dropped -- the arm request
        and the mode request end up competing with their own repeats. One every
        250 ms is still four chances a second and leaves the queue room.
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


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy's own SIGINT handler has already shut the context down by the
        # time Ctrl-C unwinds to here; calling shutdown() again raises RCLError
        # and turns a clean abort into a traceback and exit code 1.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
