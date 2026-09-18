#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import pymavlink
from pymavlink import mavutil


mdef_path = os.path.join(os.path.dirname(pymavlink.__file__), "message_definitions")
os.environ["MDEF"] = mdef_path
os.environ["MAVLINK_DIALECT"] = "common"

class Colors:
    HEADER, BLUE, CYAN, GREEN, YELLOW, RED, BOLD, DIM, RESET = (
        "\033[95m", "\033[94m", "\033[96m", "\033[92m", "\033[93m", 
        "\033[91m", "\033[1m", "\033[2m", "\033[0m"
    )

class PixhawkReaderNode(Node):
    def __init__(self):
        super().__init__('drone_testing')
        
        # Declare Node Parameters
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('rate', 4)
        self.declare_parameter('source_system', 255)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        rate = self.get_parameter('rate').value
        source_system = self.get_parameter('source_system').value
        
        self.get_logger().info(f"{Colors.HEADER}[CONNECT]{Colors.RESET} Opening {port} at {baud} baud...")
        
        try:
            self.connection = mavutil.mavlink_connection(
                port, baud=baud, source_system=source_system, autoreconnect=True
            )
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            raise SystemExit

        # Wait for Heartbeat
        self.get_logger().info("Waiting for Pixhawk heartbeat...")
        self.connection.wait_heartbeat(blocking=True, timeout=30)
        self.get_logger().info(f"{Colors.GREEN}[HEARTBEAT]{Colors.RESET} Received from Sys:{self.connection.target_system} Comp:{self.connection.target_component}")
        
        # Request data streams
        self.request_all_data_streams(rate)

        # Timer to poll for MAVLink messages (1000Hz poll rate for low latency)
        self.timer = self.create_timer(0.001, self.recv_and_dispatch)

    def request_all_data_streams(self, rate):
        for _ in range(3):
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                rate,
                1
            )
        self.get_logger().info(f"{Colors.GREEN}[STREAMS]{Colors.RESET} Stream requests sent at {rate} Hz")

    def recv_and_dispatch(self):
        msg = self.connection.recv_match(blocking=False)
        if not msg or msg.get_type() == "BAD_DATA":
            return
            
        msg_type = msg.get_type()
        
        if msg_type == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg)
            self.get_logger().info(f"{Colors.GREEN}[HEARTBEAT]{Colors.RESET} Mode: {Colors.BOLD}{mode}{Colors.RESET}")
        elif msg_type == "SYS_STATUS":
            voltage = msg.voltage_battery / 1000.0
            self.get_logger().info(f"{Colors.CYAN}[SYS_STATUS]{Colors.RESET} Battery: {voltage:.2f}V")
        elif msg_type == "STATUSTEXT":
            self.get_logger().info(f"{Colors.YELLOW}[STATUSTEXT]{Colors.RESET} {msg.text}")
            
    def destroy_node(self):
        self.connection.close()
        self.get_logger().info(f"{Colors.GREEN}[DONE]{Colors.RESET} Connection closed.")
        super().destroy_node()

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