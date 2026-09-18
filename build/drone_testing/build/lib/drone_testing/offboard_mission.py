#!/usr/bin/env python3
"""
Offboard arm test.

Sequence: stream offboard setpoints -> enter Offboard -> arm -> hold armed
for 5 s at zero thrust -> disarm.

The drone is NEVER commanded to move. Offboard is fed with body-rate
setpoints of zero rate and zero thrust, so the motors sit at idle for the
whole test. Press 'q' at any time for an immediate force-disarm.

RUN WITH PROPELLERS REMOVED.
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
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleRatesSetpoint,
    VehicleStatus,
)

from drone_testing.px4_topics import subscribe_versioned

# MAV_RESULT, for decoding what PX4 says back about each command we send.
CMD_RESULTS = {
    0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
    3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
}
CMD_NAMES = {
    176: 'DO_SET_MODE',
    400: 'COMPONENT_ARM_DISARM',
}


class ArmDisarmTest(Node):

    PREPARATION = "PREPARATION"
    OFFBOARD_REQUEST = "OFFBOARD_REQUEST"
    ARMING = "ARMING"
    ARMED_HOLD = "ARMED_HOLD"
    DISARMING = "DISARMING"
    ABORTING = "ABORTING"
    DONE = "DONE"

    # ---- test parameters -------------------------------------------------
    HOLD_SECONDS = 5.0          # how long to stay armed
    SETPOINT_WARMUP = 20        # setpoints streamed before requesting Offboard (@20 Hz = 1 s)
    OFFBOARD_TIMEOUT = 10.0
    ARMING_TIMEOUT = 10.0
    DISARM_TIMEOUT = 5.0
    # If you switch to Offboard from your RC transmitter instead of from
    # this node, set this to False.
    REQUEST_OFFBOARD_FROM_ROS = True
    # ----------------------------------------------------------------------

    def __init__(self):
        super().__init__('arm_disarm_test')

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
        self.rates_setpoint_pub = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', 10)

        self.vehicle_status_subs = subscribe_versioned(
            self, VehicleStatus, 'vehicle_status',
            self.vehicle_status_callback, sensor_qos)

        # Diagnostics only. Nothing in the state machine gates on this --
        # the test must run even with no position estimate at all.
        self.local_position = None
        self.local_position_subs = subscribe_versioned(
            self, VehicleLocalPosition, 'vehicle_local_position',
            self.local_position_callback, sensor_qos)

        # Without this we fire arm commands into the void and never learn why
        # PX4 refused. The ack carries the MAV_RESULT for every command we send.
        self._acked = set()
        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self.command_ack_callback, qos_profile=sensor_qos)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.status_received = False
        self._last_nav_state = None

        self.setpoint_counter = 0
        self.armed_at = None
        self.stage_enter_time = time.monotonic()
        self.current_stage = self.PREPARATION
        self.abort_requested = False

        self._stop_event = threading.Event()
        self._stdin_is_tty = False
        self._stdin_old_settings = None
        self._keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._keyboard_thread.start()

        # 20 Hz. PX4 drops Offboard if setpoints arrive slower than 2 Hz.
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().warning("PROPELLERS OFF. Press q at any time to force-disarm.")

    # ------------------------------------------------------------------ subs

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.status_received = True

        if self.nav_state != self._last_nav_state:
            self._last_nav_state = self.nav_state
            self.get_logger().info(f"nav_state -> {self.nav_state}")

    def local_position_callback(self, msg):
        self.local_position = msg

    def command_ack_callback(self, msg):
        name = CMD_NAMES.get(msg.command, str(msg.command))
        result = CMD_RESULTS.get(msg.result, str(msg.result))

        # We resend commands every tick until they take effect, so PX4 acks
        # every tick too. Log each (command, result) pair once.
        key = (msg.command, msg.result)
        if key in self._acked:
            return
        self._acked.add(key)

        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().info(f"PX4 ack: {name} -> {result}")
        else:
            self.get_logger().error(
                f"PX4 ack: {name} -> {result} (result_param1={msg.result_param1}). "
                f"Open QGroundControl for the exact pre-arm message.")

    def log_estimator_health(self):
        """Print what the EKF thinks it knows. GPS-denied debugging aid."""
        lp = self.local_position
        if lp is None:
            self.get_logger().warning("No VehicleLocalPosition being published at all.",
                                      throttle_duration_sec=2.0)
            return

        self.get_logger().info(
            f"xy_valid={lp.xy_valid} v_xy_valid={lp.v_xy_valid} "
            f"z_valid={lp.z_valid} v_z_valid={lp.v_z_valid} | "
            f"dist_bottom={lp.dist_bottom:.2f} m valid={lp.dist_bottom_valid} | "
            f"vx={lp.vx:+.2f} vy={lp.vy:+.2f} vz={lp.vz:+.2f} m/s",
            throttle_duration_sec=1.0)

    # -------------------------------------------------------------- keyboard

    def _keyboard_listener(self):
        if not sys.stdin.isatty():
            self.get_logger().warning("stdin is not a tty; q abort is unavailable.")
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
                ch = sys.stdin.read(1)
                if ch.lower() == 'q':
                    self.abort_requested = True
                    self.get_logger().warning("Abort requested from keyboard.")
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

    def timer_callback(self):
        # Heartbeat + setpoint go out on every tick, in every stage, so
        # Offboard never times out mid-test.
        self.publish_offboard_control_mode()
        self.publish_zero_rates_setpoint()

        if self.abort_requested and self.current_stage not in (self.ABORTING, self.DONE):
            self._enter_stage(self.ABORTING)

        handler = {
            self.PREPARATION: self._handle_preparation,
            self.OFFBOARD_REQUEST: self._handle_offboard_request,
            self.ARMING: self._handle_arming,
            self.ARMED_HOLD: self._handle_armed_hold,
            self.DISARMING: self._handle_disarming,
            self.ABORTING: self._handle_abort,
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

        self.log_estimator_health()

        self.setpoint_counter += 1
        if self.setpoint_counter >= self.SETPOINT_WARMUP:
            self.get_logger().info("Setpoint stream established.")
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

        if self._in_stage_for() > self.OFFBOARD_TIMEOUT:
            self.get_logger().error("Offboard mode not entered in time. Aborting.")
            self.abort_requested = True
            self._enter_stage(self.ABORTING)

    def _handle_arming(self):
        if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.armed_at = time.monotonic()
            self.get_logger().warning(
                f"ARMED. Holding for {self.HOLD_SECONDS:.0f} s at idle.")
            self._enter_stage(self.ARMED_HOLD)
            return

        # Lost Offboard before we got the chance to arm.
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().error("Dropped out of Offboard. Aborting.")
            self.abort_requested = True
            self._enter_stage(self.ABORTING)
            return

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        if self._in_stage_for() > self.ARMING_TIMEOUT:
            self.get_logger().error("Arming rejected / timed out. Aborting.")
            self.abort_requested = True
            self._enter_stage(self.ABORTING)

    def _handle_armed_hold(self):
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warning("Vehicle disarmed by PX4 during hold.")
            self._enter_stage(self.DONE)
            return

        elapsed = time.monotonic() - self.armed_at
        remaining = self.HOLD_SECONDS - elapsed
        if remaining <= 0.0:
            self.get_logger().info("Hold complete. Disarming.")
            self._enter_stage(self.DISARMING)
            return

        self.get_logger().info(f"Armed, {remaining:.1f} s remaining...",
                               throttle_duration_sec=1.0)
        self.log_estimator_health()

    def _handle_disarming(self):
        if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.get_logger().info("Disarmed. Test complete.")
            self._enter_stage(self.DONE)
            return

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

        if self._in_stage_for() > self.DISARM_TIMEOUT:
            self.get_logger().error("Normal disarm ignored. Escalating to force disarm.")
            self.abort_requested = True
            self._enter_stage(self.ABORTING)

    def _handle_abort(self):
        if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.get_logger().warning("Aborted; vehicle is disarmed.")
            self._enter_stage(self.DONE)
            return

        # param2 = 21196 is the PX4/MAVLink "force" magic number.
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0, param2=21196.0)

    def _handle_done(self):
        self.get_logger().info("Idle. Ctrl-C to exit.", throttle_duration_sec=5.0)

    # ------------------------------------------------------------ publishers

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True          # <-- rate control, not position control
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_zero_rates_setpoint(self):
        msg = VehicleRatesSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = 0.0
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = 0.0      # NED: 0 = no upward thrust, motors idle
        self.rates_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
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
    node = ArmDisarmTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()