#!/usr/bin/env python3
"""Pixhawk telemetry reader, over uXRCE-DDS.

Prints flight mode / arming changes, battery voltage and PX4 log messages.

This used to open /dev/ttyTHS1 with pymavlink. That port is the uXRCE-DDS
link (the agent in imav_bringup owns it), and a UART has exactly one reader:
a second process on it steals bytes from the agent and corrupts the DDS
session. So this node reads the same information from the /fmu/out topics the
agent already publishes, and never touches the serial port.

    /fmu/out/vehicle_status   mode + arming   (HEARTBEAT, before)
    /fmu/out/battery_status   voltage         (SYS_STATUS, before)
    /fmu/out/log_message      PX4 log text    (STATUSTEXT, before) -- only if
                              the FC's dds_topics.yaml exports it; the default
                              one does not, and then nothing is printed.

Parameters
    rate    Hz the battery line is printed at (default 4).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import BatteryStatus, LogMessage, VehicleStatus

from drone_testing.px4_topics import subscribe_versioned


class Colors:
    HEADER, BLUE, CYAN, GREEN, YELLOW, RED, BOLD, DIM, RESET = (
        "\033[95m", "\033[94m", "\033[96m", "\033[92m", "\033[93m",
        "\033[91m", "\033[1m", "\033[2m", "\033[0m"
    )


# nav_state value -> 'OFFBOARD', 'POSCTL', ... straight from the message constants,
# so a px4_msgs update cannot leave this table stale.
NAV_STATE_NAMES = {
    getattr(VehicleStatus, n): n[len('NAVIGATION_STATE_'):]
    for n in dir(VehicleStatus)
    if n.startswith('NAVIGATION_STATE_') and n != 'NAVIGATION_STATE_MAX'
}


class PixhawkReaderNode(Node):
    def __init__(self):
        super().__init__('drone_testing')

        rate = float(self.declare_parameter('rate', 4).value)

        # PX4 publishes /fmu/out best-effort; a reliable subscriber never matches.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subs = (
            subscribe_versioned(self, VehicleStatus, 'vehicle_status',
                                self.status_callback, sensor_qos)
            + subscribe_versioned(self, BatteryStatus, 'battery_status',
                                  self.battery_callback, sensor_qos)
            + subscribe_versioned(self, LogMessage, 'log_message',
                                  self.log_callback, sensor_qos)
        )

        self.mode = None
        self.voltage = None
        self.create_timer(1.0 / rate, self.battery_timer)
        self.create_timer(5.0, self.link_check)
        self.get_logger().info(
            f"{Colors.HEADER}[CONNECT]{Colors.RESET} Reading PX4 over uXRCE-DDS (/fmu/out)...")

    def status_callback(self, msg):
        armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        mode = (NAV_STATE_NAMES.get(msg.nav_state, str(msg.nav_state)),
                'ARMED' if armed else 'DISARMED')
        if mode != self.mode:
            if self.mode is None:
                self.get_logger().info(
                    f"{Colors.GREEN}[HEARTBEAT]{Colors.RESET} Received from Sys:{msg.system_id} "
                    f"Comp:{msg.component_id}")
            self.mode = mode
            self.get_logger().info(
                f"{Colors.GREEN}[HEARTBEAT]{Colors.RESET} Mode: {Colors.BOLD}{mode[0]}{Colors.RESET} "
                f"({mode[1]})")

    def battery_callback(self, msg):
        self.voltage = msg.voltage_v

    def battery_timer(self):
        if self.voltage is not None:
            self.get_logger().info(
                f"{Colors.CYAN}[SYS_STATUS]{Colors.RESET} Battery: {self.voltage:.2f}V")

    def log_callback(self, msg):
        text = bytes(msg.text).split(b'\0', 1)[0].decode('ascii', 'replace')
        self.get_logger().info(f"{Colors.YELLOW}[STATUSTEXT]{Colors.RESET} {text}")

    def link_check(self):
        if self.mode is None:
            self.get_logger().warning(
                "No /fmu/out/vehicle_status yet: is the uXRCE-DDS agent (imav_bringup) "
                "running, and ROS_DOMAIN_ID equal to the FC's UXRCE_DDS_DOM_ID?")


def main(args=None):
    rclpy.init(args=args)
    node = PixhawkReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
