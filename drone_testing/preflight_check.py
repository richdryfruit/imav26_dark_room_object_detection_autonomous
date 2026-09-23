#!/usr/bin/env python3
"""
Read-only PX4 arming diagnostic.

Answers one question: WHY will this airframe not arm?

SAFETY: this node NEVER publishes a VehicleCommand. There is no code path
that can arm, disarm, or change mode. In the default mode it creates no
publishers at all.

Usage:
    ros2 run drone_testing preflight_check
        Pure listener. Zero publishers. Safe with props on.

    ros2 run drone_testing preflight_check --stream
        Additionally streams the Offboard heartbeat (OffboardControlMode +
        a zero-thrust VehicleRatesSetpoint) at 20 Hz while sampling, so that
        offboard_control_signal_lost clears and PX4 evaluates the arming
        checks the way it would during a real Offboard run. Still sends no
        VehicleCommand, so it cannot arm. Motors cannot spin: they are only
        driven once armed, and nothing here arms.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    BatteryStatus,
    EstimatorStatusFlags,
    FailsafeFlags,
    HealthReport,
    OffboardControlMode,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleRatesSetpoint,
    VehicleStatus,
)


# nav_state -> name, for decoding can_arm_mode_flags and nav_state itself.
NAV_STATES = {
    0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER',
    5: 'AUTO_RTL', 6: 'POSITION_SLOW', 8: 'ALTITUDE_CRUISE', 10: 'ACRO',
    12: 'DESCEND', 13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB',
    17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND', 19: 'AUTO_FOLLOW_TARGET',
    20: 'AUTO_PRECLAND', 21: 'ORBIT', 22: 'AUTO_VTOL_TAKEOFF',
}

ARM_DISARM_REASONS = {
    0: 'TRANSITION_TO_STANDBY', 1: 'STICK_GESTURE', 2: 'RC_SWITCH',
    3: 'COMMAND_INTERNAL', 4: 'COMMAND_EXTERNAL', 5: 'MISSION_START',
    6: 'SAFETY_BUTTON', 7: 'AUTO_DISARM_LAND', 8: 'AUTO_DISARM_PREFLIGHT',
    9: 'KILL_SWITCH', 10: 'LOCKDOWN', 11: 'FAILURE_DETECTOR',
    12: 'SHUTDOWN', 13: 'UNIT_TEST',
}

CMD_RESULTS = {
    0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
    3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
}

BATTERY_WARNINGS = {0: 'NONE', 1: 'LOW', 2: 'CRITICAL', 3: 'EMERGENCY', 4: 'FAILED'}

FAILURE_DETECTOR_BITS = [
    (1 << 0, 'ROLL limit exceeded'),
    (1 << 1, 'PITCH limit exceeded'),
    (1 << 2, 'ALTITUDE limit exceeded'),
    (1 << 3, 'EXTERNAL ATS trigger'),
    (1 << 4, 'ESC failed to arm'),
    (1 << 5, 'BATTERY failure'),
    (1 << 6, 'IMBALANCED PROP'),
    (1 << 7, 'MOTOR failure'),
]

# FailsafeFlags booleans that block or degrade arming, with plain-English cause.
FAILSAFE_CHECKS = [
    ('angular_velocity_invalid', 'Gyro/angular-velocity estimate invalid -- EKF not running or IMU unhealthy'),
    ('attitude_invalid', 'Attitude estimate invalid -- EKF has no valid attitude (needs level + still to initialise)'),
    ('offboard_control_signal_lost', 'Offboard setpoint stream not seen by PX4 (need >2 Hz on /fmu/in/offboard_control_mode)'),
    ('manual_control_signal_lost', 'RC / manual control lost -- blocks arming unless COM_RC_IN_MODE=4 (stick input disabled)'),
    ('battery_unhealthy', 'Battery reported unhealthy'),
    ('home_position_invalid', 'No home position'),
    ('local_position_invalid', 'Local position estimate invalid'),
    ('local_altitude_invalid', 'Local altitude estimate invalid'),
    ('local_velocity_invalid', 'Local velocity estimate invalid'),
    ('global_position_invalid', 'Global position estimate invalid (expected indoors / GPS-denied)'),
    ('geofence_breached', 'Geofence breached'),
    ('fd_critical_failure', 'Failure detector: critical failure latched'),
    ('fd_esc_arming_failure', 'Failure detector: ESC arming failure'),
    ('fd_motor_failure', 'Failure detector: motor failure'),
    ('fd_imbalanced_prop', 'Failure detector: imbalanced propeller'),
]

# Flags that genuinely prevent arming in OFFBOARD on a GPS-denied indoor rig.
# Everything else is reported but not counted as a hard blocker.
HARD_BLOCKERS = {
    'angular_velocity_invalid',
    'attitude_invalid',
    'manual_control_signal_lost',
    'battery_unhealthy',
    'fd_critical_failure',
    'fd_esc_arming_failure',
    'fd_motor_failure',
}

SAMPLE_SECONDS = 6.0


class PreflightCheck(Node):

    def __init__(self, stream=False):
        super().__init__('preflight_check')

        self.stream = stream
        self.status = None
        self.flags = None
        self.local_pos = None
        self.battery = None
        self.health = None
        self.est_flags = None
        self.acks = []

        self._status_count = 0
        self._flags_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Topic names carry a _vN suffix tied to each message's MESSAGE_VERSION,
        # and the suffix differs between PX4 releases. Discover the real names
        # instead of hardcoding them, so a rename shows up as a clear "topic
        # not found" line rather than a subscription that silently never fires.
        self.get_logger().info('Waiting for topic discovery...')
        deadline = time.monotonic() + 5.0
        available = {}
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            available = {name: types for name, types in self.get_topic_names_and_types()
                         if name.startswith('/uav_2/fmu/out/')}
            if available:
                break

        self.available = available
        # Offboard heartbeat publishers. Created ONLY with --stream, and only
        # ever carrying zero rates and zero thrust. No VehicleCommand publisher
        # exists anywhere in this node, so arming is not reachable from here.
        self.ocm_pub = None
        self.rates_pub = None
        if stream:
            self.ocm_pub = self.create_publisher(
                OffboardControlMode, '/uav_2/fmu/in/offboard_control_mode', 10)
            self.rates_pub = self.create_publisher(
                VehicleRatesSetpoint, '/uav_2/fmu/in/vehicle_rates_setpoint', 10)
            self.get_logger().warning(
                'STREAM MODE: publishing zero-thrust Offboard heartbeat at 20 Hz. '
                'No arm command will be sent.')

        self.subs = []
        self._subscribe(available, 'vehicle_status', VehicleStatus, self.on_status, qos)
        self._subscribe(available, 'failsafe_flags', FailsafeFlags, self.on_flags, qos)
        self._subscribe(available, 'vehicle_local_position', VehicleLocalPosition, self.on_local_pos, qos)
        self._subscribe(available, 'battery_status', BatteryStatus, self.on_battery, qos)
        self._subscribe(available, 'vehicle_command_ack', VehicleCommandAck, self.on_ack, qos)
        self._subscribe(available, 'estimator_status_flags', EstimatorStatusFlags,
                        self.on_est_flags, qos)
        # Optional: only present if dds_topics.yaml bridges it. This is the ONLY
        # source of the specific failing arming check; without it PX4 tells us
        # that arming failed but never which check failed.
        self._subscribe(available, 'health_report', HealthReport, self.on_health, qos,
                        optional=True)

    def _subscribe(self, available, base, msg_type, cb, qos, optional=False):
        """Match /fmu/out/<base> or /fmu/out/<base>_vN."""
        match = None
        for name in available:
            tail = name[len('/uav_2/fmu/out/'):]
            if tail == base or (tail.startswith(base + '_v') and tail[len(base) + 2:].isdigit()):
                match = name
                break
        if match is None:
            if optional:
                self.get_logger().warning(
                    f'Topic /fmu/out/{base}[_vN] not bridged (optional).')
            else:
                self.get_logger().error(
                    f'Topic /fmu/out/{base}[_vN] NOT FOUND -- PX4 is not publishing it.')
            return
        self.subs.append(self.create_subscription(msg_type, match, cb, qos_profile=qos))
        self.get_logger().info(f'subscribed: {match}')

    # ---------------------------------------------------------------- subs

    def on_status(self, msg):
        self.status = msg
        self._status_count += 1

    def on_flags(self, msg):
        self.flags = msg
        self._flags_count += 1

    def on_local_pos(self, msg):
        self.local_pos = msg

    def on_battery(self, msg):
        self.battery = msg

    def on_ack(self, msg):
        self.acks.append(msg)

    def on_health(self, msg):
        self.health = msg

    def on_est_flags(self, msg):
        self.est_flags = msg

    # -------------------------------------------------------------- report

    def _publish_heartbeat(self):
        """Zero rate, zero thrust. Never anything else."""
        now = int(self.get_clock().now().nanoseconds / 1000)

        ocm = OffboardControlMode()
        ocm.timestamp = now
        ocm.position = False
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = True
        ocm.thrust_and_torque = False
        ocm.direct_actuator = False
        self.ocm_pub.publish(ocm)

        sp = VehicleRatesSetpoint()
        sp.timestamp = now
        sp.roll = 0.0
        sp.pitch = 0.0
        sp.yaw = 0.0
        sp.thrust_body[0] = 0.0
        sp.thrust_body[1] = 0.0
        sp.thrust_body[2] = 0.0
        self.rates_pub.publish(sp)

    def run(self):
        if self.stream:
            # PX4 needs a second or so of continuous setpoints before it stops
            # reporting the offboard signal as lost, so warm up before sampling.
            self.get_logger().info('Warming up Offboard heartbeat for 3 s...')
            warm_end = time.monotonic() + 3.0
            while time.monotonic() < warm_end and rclpy.ok():
                self._publish_heartbeat()
                rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(f'Sampling for {SAMPLE_SECONDS:.0f} s...')
        end = time.monotonic() + SAMPLE_SECONDS
        while time.monotonic() < end and rclpy.ok():
            if self.stream:
                self._publish_heartbeat()
            rclpy.spin_once(self, timeout_sec=0.05)
        self.report()

    def report(self):
        p = print
        p('')
        p('=' * 72)
        p('PX4 PRE-ARM DIAGNOSTIC'.center(72))
        p('=' * 72)

        if not self.available:
            p('')
            p('NO /fmu/out/ TOPICS AT ALL.')
            p('  The uXRCE-DDS link is down. Check, in order:')
            p('   1. MicroXRCEAgent is running on this Jetson')
            p('   2. ROS_DOMAIN_ID here == XRCE_DDS_DOM_ID on the FC')
            p('   3. baudrate matches SER_TEL2_BAUD, wiring on /dev/ttyTHS1')
            p('=' * 72)
            return

        if self.status is None:
            p('')
            p('VehicleStatus never arrived (%d msgs).' % self._status_count)
            p('  Agent is up but PX4 is not publishing vehicle_status.')
            p('=' * 72)
            return

        s = self.status
        blockers = []
        warnings = []

        # ---- headline state
        armed = s.arming_state == VehicleStatus.ARMING_STATE_ARMED
        p('')
        p('STATE')
        p('  arming_state         : %s' % ('ARMED' if armed else 'DISARMED'))
        p('  nav_state            : %d (%s)' % (s.nav_state, NAV_STATES.get(s.nav_state, '?')))
        p('  nav_state_user_intent: %d (%s)' % (s.nav_state_user_intention,
                                                NAV_STATES.get(s.nav_state_user_intention, '?')))
        p('  vehicle_type         : %d' % s.vehicle_type)
        p('  last arming reason   : %s' % ARM_DISARM_REASONS.get(s.latest_arming_reason, '?'))
        p('  last disarming reason: %s' % ARM_DISARM_REASONS.get(s.latest_disarming_reason, '?'))
        p('  in failsafe          : %s' % s.failsafe)

        # ---- the single most important flag
        p('')
        p('MASTER CHECK')
        p('  pre_flight_checks_pass : %s' % s.pre_flight_checks_pass)
        if not s.pre_flight_checks_pass:
            blockers.append('pre_flight_checks_pass is FALSE -- PX4 refuses to arm in any mode')

        # ---- safety switch: the classic silent blocker
        p('')
        p('SAFETY SWITCH')
        p('  safety_button_available: %s' % s.safety_button_available)
        p('  safety_off             : %s' % s.safety_off)
        if s.safety_button_available and not s.safety_off:
            blockers.append('SAFETY SWITCH NOT PRESSED -- press and hold the button until the LED stops blinking '
                            '(or set CBRK_IO_SAFETY=22027 to bypass)')

        # ---- power / kill
        p('')
        p('POWER')
        p('  power_input_valid      : %s' % s.power_input_valid)
        p('  usb_connected          : %s  (LATCHED: set once, never cleared until FC reboot --'
          % s.usb_connected)
        p('                            this does NOT mean USB is plugged in now)')
        if not s.power_input_valid:
            blockers.append('power_input_valid is FALSE -- FC sees no valid power rail')

        # ---- battery
        if self.battery is not None:
            b = self.battery
            p('')
            p('BATTERY')
            p('  connected  : %s' % b.connected)
            p('  voltage    : %.2f V' % b.voltage_v)
            p('  remaining  : %.0f %%' % (b.remaining * 100.0))
            p('  warning    : %s' % BATTERY_WARNINGS.get(b.warning, b.warning))
            p('  cell count : %d' % b.cell_count)
            if not b.connected:
                blockers.append('Battery not detected -- PX4 will not arm without a battery reading '
                                '(check power module / BAT1_SOURCE)')
            if b.warning >= 2:
                blockers.append('Battery warning level %s' % BATTERY_WARNINGS.get(b.warning, b.warning))
        else:
            p('')
            p('BATTERY: no battery_status received')

        # ---- failure detector
        if s.failure_detector_status:
            p('')
            p('FAILURE DETECTOR: 0x%04x' % s.failure_detector_status)
            for bit, label in FAILURE_DETECTOR_BITS:
                if s.failure_detector_status & bit:
                    p('  - %s' % label)
                    blockers.append('Failure detector: %s' % label)

        # ---- failsafe flags: the detailed reasons
        p('')
        p('FAILSAFE / MODE-REQUIREMENT FLAGS')
        if self.flags is None:
            p('  failsafe_flags NOT PUBLISHED.')
            p('  Add this line to PX4 /etc/uxrce_dds_client/dds_topics.yaml, then reboot the FC:')
            p('      - topic: /fmu/out/failsafe_flags')
            p('        type: px4_msgs::msg::FailsafeFlags')
            warnings.append('failsafe_flags not published -- detailed per-check reasons unavailable')
        else:
            f = self.flags
            for field, explanation in FAILSAFE_CHECKS:
                val = getattr(f, field, None)
                if val is None:
                    continue
                if val:
                    p('  [BLOCK] %-32s %s' % (field, explanation))
                    if field in HARD_BLOCKERS:
                        blockers.append('%s -- %s' % (field, explanation))
                    else:
                        warnings.append('%s -- %s' % (field, explanation))
                else:
                    p('  [ ok  ] %-32s' % field)

            p('  battery_warning: %s' % BATTERY_WARNINGS.get(f.battery_warning, f.battery_warning))

            # Does OFFBOARD itself forbid arming?
            offboard_bit = 1 << VehicleStatus.NAVIGATION_STATE_OFFBOARD
            if f.mode_req_prevent_arming & offboard_bit:
                blockers.append('mode_req_prevent_arming has the OFFBOARD bit set -- '
                                'cannot arm while already in Offboard; arm first, then switch')

        # ---- can we even select offboard
        p('')
        p('MODE AVAILABILITY')
        offboard_bit = 1 << VehicleStatus.NAVIGATION_STATE_OFFBOARD
        p('  valid_nav_states_mask  : 0x%08x' % s.valid_nav_states_mask)
        p('  can_set_nav_states_mask: 0x%08x' % s.can_set_nav_states_mask)
        p('  OFFBOARD valid         : %s' % bool(s.valid_nav_states_mask & offboard_bit))
        p('  OFFBOARD selectable    : %s' % bool(s.can_set_nav_states_mask & offboard_bit))
        if not (s.can_set_nav_states_mask & offboard_bit):
            blockers.append('OFFBOARD is not currently selectable -- its mode requirements are unmet, '
                            'so DO_SET_MODE will be rejected')

        # ---- estimator
        p('')
        p('ESTIMATOR')
        if self.local_pos is None:
            p('  no vehicle_local_position received')
            warnings.append('No VehicleLocalPosition -- EKF may not be publishing')
        else:
            lp = self.local_pos
            p('  xy_valid=%s v_xy_valid=%s z_valid=%s v_z_valid=%s'
              % (lp.xy_valid, lp.v_xy_valid, lp.z_valid, lp.v_z_valid))
            p('  heading=%.1f deg  dist_bottom=%.2f m (valid=%s)'
              % (lp.heading * 57.2958, lp.dist_bottom, lp.dist_bottom_valid))

        # ---- rangefinder fusion. On this airframe EKF2_HGT_REF=2 (Range), so
        # dist_bottom_valid is pinned false by construction and the cs_rng_*
        # flags are the only honest answer. See offboard_takeoff.
        if self.est_flags is None:
            p('')
            p('RANGEFINDER FUSION')
            p('  estimator_status_flags NOT PUBLISHED.')
            p('  The flight nodes use it to answer "is EKF2 actually FUSING the')
            p('  rangefinder?" -- dist_bottom_valid alone cannot. Without it they')
            p('  refuse to take off whenever dist_bottom_valid is false, because')
            p('  there is no second opinion on the height.')
            p('  Add this to PX4 /etc/uxrce_dds_client/dds_topics.yaml, reboot the FC:')
            p('      - topic: /fmu/out/estimator_status_flags')
            p('        type: px4_msgs::msg::EstimatorStatusFlags')
            warnings.append('estimator_status_flags not published -- rangefinder '
                            'fusion cannot be confirmed')
        if self.est_flags is not None:
            ef = self.est_flags
            p('')
            p('RANGEFINDER FUSION (dist_bottom_valid is meaningless with EKF2_HGT_REF=2)')
            p('  cs_rng_hgt            : %s' % ef.cs_rng_hgt)
            p('  cs_rng_terrain        : %s  (expected False with HGT_REF=Range)' % ef.cs_rng_terrain)
            p('  cs_rng_fault          : %s' % ef.cs_rng_fault)
            p('  cs_rng_stuck          : %s' % ef.cs_rng_stuck)
            p('  cs_rng_kin_consistent : %s  <-- gates fusion' % ef.cs_rng_kin_consistent)
            healthy = ((ef.cs_rng_hgt or ef.cs_rng_terrain)
                       and not ef.cs_rng_fault and not ef.cs_rng_stuck
                       and ef.cs_rng_kin_consistent)
            p('  => rangefinder healthy : %s' % healthy)
            if not healthy:
                blockers.append('Rangefinder not being fused (cs_rng_*) -- '
                                'offboard_takeoff.position_is_usable() will refuse to arm')
            if ef.fs_bad_acc_vertical:
                blockers.append('fs_bad_acc_vertical -- EKF has flagged bad vertical '
                                'accelerometer data (related to the accel-bias family of checks)')
            p('  fs_bad_acc_vertical   : %s' % ef.fs_bad_acc_vertical)

        # ---- the only source of the SPECIFIC failing arming check
        p('')
        p('HEALTH REPORT (specific arming checks)')
        if self.health is None:
            p('  NOT BRIDGED. This is why PX4 can refuse to arm without telling us why.')
            p('  Add to PX4 src/modules/uxrce_dds_client/dds_topics.yaml under publications:')
            p('      - topic: /fmu/out/health_report')
            p('        type: px4_msgs::msg::HealthReport')
            p('  then rebuild + flash. Same edit you already made for distance_sensor.')
        else:
            h = self.health
            offboard_bit = 1 << VehicleStatus.NAVIGATION_STATE_OFFBOARD
            p('  arming_check_error_flags : 0x%016x' % h.arming_check_error_flags)
            p('  arming_check_warning_flags: 0x%016x' % h.arming_check_warning_flags)
            p('  health_error_flags       : 0x%016x' % h.health_error_flags)
            p('  health_warning_flags     : 0x%016x' % h.health_warning_flags)
            p('  can_arm in OFFBOARD      : %s' % bool(h.can_arm_mode_flags & offboard_bit))
            p('  can_run  in OFFBOARD     : %s' % bool(h.can_run_mode_flags & offboard_bit))
            if h.arming_check_error_flags:
                blockers.append('arming_check_error_flags = 0x%016x -- a specific arming check '
                                'is failing; QGC names it in plain text'
                                % h.arming_check_error_flags)
            if not (h.can_arm_mode_flags & offboard_bit):
                blockers.append('HealthReport says arming is not possible in OFFBOARD')

        # ---- any command acks PX4 sent (e.g. from a previous arm attempt)
        if self.acks:
            p('')
            p('COMMAND ACKS SEEN DURING SAMPLE')
            for a in self.acks[-10:]:
                p('  cmd %-6d -> %s (result_param1=%d)'
                  % (a.command, CMD_RESULTS.get(a.result, a.result), a.result_param1))

        # ---- verdict
        p('')
        p('=' * 72)
        if armed:
            p('VEHICLE IS CURRENTLY ARMED.')
        elif not blockers:
            p('VERDICT: no hard blocker found. PX4 reports it is ready to arm.')
            if warnings:
                p('')
                p('Non-blocking warnings:')
                for w in warnings:
                    p('  ! %s' % w)
        else:
            p('VERDICT: ARMING IS BLOCKED BY %d CONDITION(S):' % len(blockers))
            p('')
            for i, b in enumerate(blockers, 1):
                p('  %d. %s' % (i, b))
            if warnings:
                p('')
                p('Also worth noting:')
                for w in warnings:
                    p('  ! %s' % w)
        p('=' * 72)
        p('')


def main(args=None):
    import sys as _sys
    stream = '--stream' in _sys.argv
    rclpy.init(args=args)
    node = PreflightCheck(stream=stream)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
