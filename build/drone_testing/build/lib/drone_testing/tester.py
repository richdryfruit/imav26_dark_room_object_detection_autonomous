import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleStatus

rclpy.init()
n = Node('sub_test')
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               durability=DurabilityPolicy.VOLATILE,
               history=HistoryPolicy.KEEP_LAST, depth=5)
n.create_subscription(
    VehicleStatus, '/fmu/out/vehicle_status_v1',
    lambda m: n.get_logger().info(f'got nav={m.nav_state} arm={m.arming_state}'), q)
rclpy.spin(n)