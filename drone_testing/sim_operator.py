#!/usr/bin/env python3
"""SITL ONLY: stands in for the operator at px4_telemetry_dashboard.

On the real aircraft the flight nodes neither arm nor request Offboard nor
publish the OffboardControlMode heartbeat any more: the operator does that
from px4_telemetry_dashboard (Offboard heartbeat on -> Offboard -> Arm). An
unattended SITL run has nobody to click, so this does the same three things:

    1. OffboardControlMode, position=True, at 20 Hz -- exactly the flags the
       dashboard sends -- once a flight node is streaming trajectory setpoints
    2. DO_SET_MODE Offboard, after warmup_s of that
    3. ARM, once PX4 reports Offboard

It does NOT publish a TrajectorySetpoint. The dashboard's heartbeat does (a
frozen hold point), which on the vehicle races the flight node's own setpoints
on the same topic; the flight node is the only setpoint source here.

Refuses to start on ROS_DOMAIN_ID 0 (or unset): that is the real FC's domain,
and this node arms whatever answers on it.
"""

import os
import time

import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleStatus)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_testing.px4_topics import subscribe_versioned


class SimOperator(Node):
    def __init__(self):
        super().__init__('sim_operator')
        self.warmup = float(self.declare_parameter('warmup_s', 1.5).value)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                                 self._sp_callback, qos)
        self.subs = subscribe_versioned(self, VehicleStatus, 'vehicle_status',
                                        self._status_callback, qos)
        self.last_sp = None
        self.first_sp = None
        self.status = None
        self.last_cmd = 0.0
        self.create_timer(0.05, self._tick)
        self.get_logger().warning(
            "SIM OPERATOR: will switch Offboard and arm once a flight node "
            "streams setpoints (the dashboard's job on the real aircraft).")

    def _sp_callback(self, _msg):
        now = time.monotonic()
        if self.last_sp is None or now - self.last_sp > 1.0:
            self.first_sp = now
        self.last_sp = now

    def _status_callback(self, msg):
        self.status = msg

    def _cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = command
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def _tick(self):
        now = time.monotonic()
        streaming = self.last_sp is not None and now - self.last_sp < 0.5
        if not streaming:
            return
        ocm = OffboardControlMode()
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ocm.position = True
        self.ocm_pub.publish(ocm)

        s = self.status
        if s is None or now - self.first_sp < self.warmup or now - self.last_cmd < 1.0:
            return
        offboard = s.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        armed = s.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if armed:
            return
        self.last_cmd = now
        if not offboard:
            self.get_logger().info("Switching to Offboard.")
            self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        else:
            self.get_logger().info("Offboard; arming.")
            self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)


def main(args=None):
    if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0'):
        raise SystemExit("sim_operator: ROS_DOMAIN_ID is 0/unset -- that is the "
                         "real FC's domain. Refusing to arm anything on it.")
    rclpy.init(args=args)
    node = SimOperator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
