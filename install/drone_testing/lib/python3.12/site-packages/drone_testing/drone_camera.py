import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition

from drone_testing.px4_topics import subscribe_versioned
from cv_bridge import CvBridge
import cv2
import numpy as np

class MissionPlanner(Node):
    
    def __init__(self):
        super().__init__('mission_planner')
        
        # Define QoS profile for subscribers to ensure compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # --- Parameters for Control ---
        self.MAX_VELOCITY = 2.5  # m/s
        self.ACCEPTANCE_RADIUS = 0.5  # meters
        self.TAKEOFF_VELOCITY_Z = -1.0 # m/s (negative is up)
        
        # --- Publishers ---
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # --- Subscribers ---
        self.vehicle_status_subs = subscribe_versioned(self, VehicleStatus, 'vehicle_status', self.vehicle_status_callback, qos_profile)
        self.local_position_subs = subscribe_versioned(self, VehicleLocalPosition, 'vehicle_local_position', self.local_position_callback, qos_profile)
        self.image_view_sub = self.create_subscription(Image, '/depth_camera', self.image_view_callback, 10)
        self.cv_bridge = CvBridge()

        # --- State Variables ---
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.local_position = VehicleLocalPosition()

        
        # --- Timers ---
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    def local_position_callback(self, msg):
        self.local_position = msg
    
    def image_view_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            cv_img = np.array(cv_image, dtype=np.float32)
            cv_img = np.where(cv_img > 15.0, 0.0, cv_img)
            norm_image = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            cv2.imshow("Depth Camera View", norm_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


    def timer_callback(self):
        pass

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False 
        msg.velocity = True  
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx, vy, vz, yaw):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        nan_val = float('nan')
        
        # Position set to NaN
        msg.position[0] = nan_val
        msg.position[1] = nan_val
        msg.position[2] = nan_val
        
        # Velocity set
        msg.velocity[0] = vx
        msg.velocity[1] = vy
        msg.velocity[2] = vz
        
        # Yaw set
        msg.yaw = yaw 
        msg.yawspeed = nan_val

        # Acceleration set to NaN
        msg.acceleration[0] = nan_val
        msg.acceleration[1] = nan_val
        msg.acceleration[2] = nan_val
        
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    mission_planner = MissionPlanner()
    try:
        rclpy.spin(mission_planner)
    except KeyboardInterrupt:
        pass
    finally:
        mission_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()