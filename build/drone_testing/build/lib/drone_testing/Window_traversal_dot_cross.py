import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy, Image
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition, VehicleAttitude

from drone_testing.px4_topics import subscribe_versioned
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
from tf2_ros import TransformBroadcaster
from drone_controller.quaternions import Quaternions

class DroneController(Node):
    
    def __init__(self):
        super().__init__('drone_joystick_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # --- Publishers ---
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # --- Subscribers ---
        self.vehicle_status_subs = subscribe_versioned(self, VehicleStatus, 'vehicle_status', self.vehicle_status_callback, qos_profile)
        self.local_position_subs = subscribe_versioned(self, VehicleLocalPosition, 'vehicle_local_position', self.local_position_callback, qos_profile)
        self.local_orienation_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.local_orientation_callback, qos_profile=qos_profile)
        self.joystick_sub = self.create_subscription(Joy, '/joy',self.joystick_callback, 10)

        # --- State Variables ---
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.local_position = VehicleLocalPosition()
        self.local_orientation = VehicleAttitude()
        self.islanding = False

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_child_frame = 'drone_base_link'
        
        # --- Timers ---
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # --- Image & Vision Variables ---
        self.image_sub = self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.cv_bridge = CvBridge()
        self.depth_image = None
        self.normalized_depth_image = None
        self.rgb_to_depth_scale_x = 431.4 / 465.6  
        self.rgb_to_depth_scale_y = 431.4 / 620.9  
        self.cam_pitch = 0.0
        self.CAM_SENSITIVITY = 1.5       
        
        # --- Autonomous Mode Variables ---
        self.autonomous_mode = False
        self.E1_ned = None
        self.E2_ned = None
        self.E3_ned = None
        self.E4_ned = None
        self.X_w = None  
        self.Y_w = None  
        self.Z_w = None  
        self.index = 0
        self.target_color = 'BLUE'  
        self.ready_to_traverse = False
        self.TRAVERSAL_SPEED = 1.0

        # --- Velocity Variables ---
        self.MAX_VELOCITY = 3.0 
        self.BOOST = 6.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yawspeed = 0.0
        self.MAX_YAWSPEED = 1.5
        self.prev_e1 = 0
        self.prev_e2 = 0
        self.prev_e3 = 0
        self.prev_e4 = 0

        # --- Controller log ---
        self.LB = 6
        self.RB = 7
        self.LT = 5
        self.RT = 4
        self.LEFT_STICK_H = 0
        self.LEFT_STICK_V = 1
        self.RIGHT_STICK_H = 2
        self.RIGHT_STICK_V = 3
        self.Y_BUTTON = 4
        self.X_BUTTON = 3
        self.A_BUTTON = 0
        self.B_BUTTON = 1
        self.DPAD_UP_DOWN = 7
        self.axes = [0]*8
        self.buttons = [0]*15
        
        self.last_b_button = 0
        self.last_a_button = 0

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    def local_position_callback(self, msg):
        self.local_position = msg
        # try:
        #     self.publish_tf_transform()
        # except Exception as e:
        #     pass
    
    def local_orientation_callback(self, msg):
        self.local_orientation = msg

    
    def calculate_velocity(self, axes_vals):
        vx = axes_vals[self.LEFT_STICK_V] * (self.MAX_VELOCITY if axes_vals[self.DPAD_UP_DOWN] == 0 else self.BOOST)
        vy = axes_vals[self.LEFT_STICK_H] * (self.MAX_VELOCITY if axes_vals[self.DPAD_UP_DOWN] == 0 else self.BOOST)
        vz = 0.0
        if axes_vals[self.LT] < 0.0 and axes_vals[self.RT] > 0.0:
            vz = -axes_vals[self.LT] * self.MAX_VELOCITY
        elif axes_vals[self.LT] > 0.0 and axes_vals[self.RT] < 0.0:
            vz = axes_vals[self.RT] * self.MAX_VELOCITY
        angle = self.local_position.heading
        global_vx = vx * math.cos(angle) + vy * math.sin(angle)
        global_vy = vx * math.sin(angle) - vy * math.cos(angle)
        return global_vx, global_vy, vz

    def joystick_callback(self, msg):
        self.axes = msg.axes
        self.buttons = msg.buttons  
        
        if self.buttons[self.B_BUTTON] == 1 and self.last_b_button == 0:
            self.autonomous_mode = not self.autonomous_mode
            mode_str = "AUTONOMOUS" if self.autonomous_mode else "MANUAL"
            self.get_logger().info(f'--- Switched to {mode_str} MODE ---')
        self.last_b_button = self.buttons[self.B_BUTTON]

        if self.buttons[self.A_BUTTON] == 1 and self.last_a_button == 0:
            self.target_color = 'RED' if self.target_color == 'BLUE' else 'BLUE'
            self.get_logger().info(f'--- Target Color: {self.target_color} ---')
        self.last_a_button = self.buttons[self.A_BUTTON]

    def map_rgb_to_depth(self, u_rgb, v_rgb):

        center_u, center_v = 320, 240
        u_depth = int(center_u + (u_rgb - center_u) * self.rgb_to_depth_scale_x)
        v_depth = int(center_v + (v_rgb - center_v) * self.rgb_to_depth_scale_y)

        u_depth = min(max(u_depth, 0), 639)
        v_depth = min(max(v_depth, 0), 479)
        
        return u_depth, v_depth
    
    def get_valid_depth(self, depth_img, u, v, max_radius=10):
        """Spirals outward from (u, v) to find the nearest valid depth."""
        # 1. Check the exact center first
        val = depth_img[v, u]
        if val > 0 and not np.isnan(val) and not np.isinf(val):
            return val
            
        # 2. Spiral search outward
        for r in range(1, max_radius + 1):
            # Check the perimeter of a square with radius r
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    # Only check the outer edge of the current square
                    if abs(dx) == r or abs(dy) == r: 
                        nu, nv = u + dx, v + dy
                        
                        # Ensure we don't go out of image bounds
                        if 0 <= nv < depth_img.shape[0] and 0 <= nu < depth_img.shape[1]:
                            val = depth_img[nv, nu]
                            if val > 0 and not np.isnan(val) and not np.isinf(val):
                                return val
                                
        # 3. If everything fails within the radius, return 0.0
        return 0.0
    
    def lock_window_coordinates(self, R_e1, R_e2, R_e3, R_e4, theta_x1, theta_y1, theta_x2, theta_y2, theta_x3, theta_y3, theta_x4, theta_y4):

        camera_offset_frd = np.array([0.01233, 0.03000, -0.01878])

        pos_e1_body = np.array([R_e1, math.tan(math.radians(theta_x1)) * R_e1, -math.tan(math.radians(theta_y1)) * R_e1]) + camera_offset_frd
        pos_e2_body = np.array([R_e2, math.tan(math.radians(theta_x2)) * R_e2, -math.tan(math.radians(theta_y2)) * R_e2]) + camera_offset_frd
        pos_e3_body = np.array([R_e3, math.tan(math.radians(theta_x3)) * R_e3, -math.tan(math.radians(theta_y3)) * R_e3]) + camera_offset_frd
        pos_e4_body = np.array([R_e4, math.tan(math.radians(theta_x4)) * R_e4, -math.tan(math.radians(theta_y4)) * R_e4]) + camera_offset_frd

        w = self.local_orientation.q[0]
        xyz = self.local_orientation.q[1:4]
        q = Quaternions(w, xyz)
        drone_pos = np.array([self.local_position.x, self.local_position.y, self.local_position.z])

        E1_ned = q.rotate_vector(pos_e1_body) + drone_pos
        E2_ned = q.rotate_vector(pos_e2_body) + drone_pos
        E3_ned = q.rotate_vector(pos_e3_body) + drone_pos
        E4_ned = q.rotate_vector(pos_e4_body) + drone_pos

        dot_products =  np.mean(np.abs(np.array([
            np.dot(E2_ned - E1_ned, E3_ned - E2_ned),
            np.dot(E3_ned - E2_ned, E4_ned - E3_ned),
            np.dot(E4_ned - E3_ned, E1_ned - E4_ned),
            np.dot(E1_ned - E4_ned, E2_ned - E1_ned)
        ])))

        cross_products = np.mean(np.abs(np.array([
            np.cross(E2_ned - E1_ned, E4_ned - E3_ned),
            np.cross(E1_ned - E4_ned, E3_ned - E2_ned)
        ])))

        if dot_products < 0.02 and cross_products < 0.02:

            self.ready_to_traverse = True

            self.get_logger().warn(f"Ready to traverse !! Dot Product: {dot_products:.4f}, Cross Product: {cross_products:.4f}")
            self.E1_ned, self.E2_ned, self.E3_ned, self.E4_ned = E1_ned, E2_ned, E3_ned, E4_ned
            
            X_w = self.E2_ned - self.E1_ned
            self.X_w = X_w / np.linalg.norm(X_w)

            Z_w = self.E1_ned - self.E4_ned
            self.Z_w = Z_w / np.linalg.norm(Z_w)

            Y_w = np.cross(self.Z_w, self.X_w)
            self.Y_w = Y_w / np.linalg.norm(Y_w)


        self.get_logger().info(f"X_w =[{self.X_w[0]:.2f}, {self.X_w[1]:.2f}, {self.X_w[2]:.2f}]")
        self.get_logger().info(f"Y_w =[{self.Y_w[0]:.2f}, {self.Y_w[1]:.2f}, {self.Y_w[2]:.2f}]")
        self.get_logger().info(f"Z_w =[{self.Z_w[0]:.2f}, {self.Z_w[1]:.2f}, {self.Z_w[2]:.2f}]")
    
    def update_autonomous_traversal(self):

        drone_pos = np.array([self.local_position.x, self.local_position.y, self.local_position.z])

        v1_ned = self.E1_ned - drone_pos
        v2_ned = self.E2_ned - drone_pos
        v4_ned = self.E4_ned - drone_pos

        px1, py1, pz1 = np.dot(v1_ned, self.X_w), np.dot(v1_ned, self.Y_w), np.dot(v1_ned, self.Z_w)
        px2, py2, pz2 = np.dot(v2_ned, self.X_w), np.dot(v2_ned, self.Y_w), np.dot(v2_ned, self.Z_w)
        px4, py4, pz4 = np.dot(v4_ned, self.X_w), np.dot(v4_ned, self.Y_w), np.dot(v4_ned, self.Z_w)

        dist_1 = math.sqrt(px1**2 + py1**2 + pz1**2)
        beta_1 = math.atan2(py1, px1)  
        alpha_1 = math.asin(pz1 / dist_1)

        dist_2 = math.sqrt(px2**2 + py2**2 + pz2**2)
        beta_2 = math.atan2(py2, px2)
        alpha_2 = math.asin(pz2 / dist_2)

        dist_4 = math.sqrt(px4**2 + py4**2 + pz4**2)
        alpha_4 = math.asin(pz4 / dist_4)

        Dx = -0.5 * (dist_1 * math.cos(alpha_1) * math.cos(beta_1) + dist_2 * math.cos(alpha_2) * math.cos(beta_2))
        Dz = -0.5 * (dist_1 * math.sin(alpha_1) + dist_4 * math.sin(alpha_4))
        
        # Log Dx and Dz to terminal every ~0.5 seconds
        if time.time() % 0.5 < 0.1:
            self.get_logger().info(f"Trajectory Error -> Dx: {Dx:.3f}m, Dz: {Dz:.3f}m")

        if py1 < 0.1: 
            self.auto_vx = self.TRAVERSAL_SPEED * self.Y_w[0]
            self.auto_vy = self.TRAVERSAL_SPEED * self.Y_w[1]
            self.auto_vz = self.TRAVERSAL_SPEED * self.Y_w[2]
            return

        bisector_alpha = (alpha_1 + alpha_4) / 2.0
        
        S_gamma_mag = math.sqrt(abs(math.pi**2 - 4 * (bisector_alpha - np.sign(bisector_alpha)*(math.pi/2))**2)) / 4.0
        S_gamma = np.sign(bisector_alpha) * S_gamma_mag
        
        gamma_term = bisector_alpha + S_gamma
        if gamma_term > math.pi/2:
            gamma_des = math.pi - gamma_term
        elif gamma_term < -math.pi/2:
            gamma_des = -math.pi - gamma_term
        else:
            gamma_des = gamma_term

        bisector_beta = (beta_1 + beta_2) / 2.0
        S_chi = np.sign(bisector_beta - math.pi/2) * math.sqrt(abs(math.pi**2 - (bisector_beta * 2)**2)) / 4.0

        if -math.pi/2 <= gamma_term <= math.pi/2:
            chi_des = bisector_beta + S_chi
        else:
            chi_des = -bisector_beta - S_chi

        v_xw = self.TRAVERSAL_SPEED * math.cos(gamma_des) * math.cos(chi_des)
        v_yw = self.TRAVERSAL_SPEED * math.cos(gamma_des) * math.sin(chi_des)
        v_zw = self.TRAVERSAL_SPEED * math.sin(gamma_des)

        V_ned = v_xw * self.X_w + v_yw * self.Y_w + v_zw * self.Z_w

        self.auto_vx = V_ned[0]
        self.auto_vy = V_ned[1]
        self.auto_vz = V_ned[2]


    def depth_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, '32FC1')  
            image = cv_image.astype(np.float32) 
            # image = np.where(np.isinf(image), 0, image)  
            self.depth_image = image
            image_non_inf = np.where(np.isinf(image), 0, image)  
            normalized_image = (np.max(image_non_inf) - image_non_inf) / np.max(image_non_inf) * 255  
            normalized_image = ( np.bitwise_not(np.isinf(image)) * normalized_image ).astype(np.uint8)
            self.normalized_depth_image = normalized_image
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.resize(cv_image, (640, 480))
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            if self.target_color == 'BLUE':
                lower_bound = np.array([100, 150, 50])
                upper_bound = np.array([140, 255, 255])
                mask = cv2.inRange(hsv, lower_bound, upper_bound)
            else: 
                lower_red1 = np.array([0, 150, 50])
                upper_red1 = np.array([10, 255, 255])
                lower_red2 = np.array([170, 150, 50])
                upper_red2 = np.array([180, 255, 255])
                mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                mask = cv2.bitwise_or(mask1, mask2)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 1500:  
                    
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    padding = 5 

                    u1, v1 = x + padding, y + padding             
                    u2, v2 = x + w - padding, y + padding
                    u3, v3 = x + w - padding, y + h - padding         
                    u4, v4 = x + padding, y + h - padding  

                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    cv2.circle(cv_image, (u1, v1), 6, (255, 0, 0), -1) # E1
                    cv2.circle(cv_image, (u2, v2), 6, (255, 255, 0), -1) # E2
                    cv2.circle(cv_image, (u3, v3), 6, (0, 255, 255), -1) # E3
                    cv2.circle(cv_image, (u4, v4), 6, (0, 0, 255), -1) # E4

                    u1, v1 = self.map_rgb_to_depth(u1, v1)
                    u2, v2 = self.map_rgb_to_depth(u2, v2)
                    u3, v3 = self.map_rgb_to_depth(u3, v3)
                    u4, v4 = self.map_rgb_to_depth(u4, v4)

                    if self.depth_image is not None and not self.autonomous_mode:

                        v1 = min(max(v1, 0), self.depth_image.shape[0] - 1)
                        u1 = min(max(u1, 0), self.depth_image.shape[1] - 1)
                        v2 = min(max(v2, 0), self.depth_image.shape[0] - 1)
                        u2 = min(max(u2, 0), self.depth_image.shape[1] - 1)
                        u3 = min(max(u3, 0), self.depth_image.shape[1] - 1)
                        v3 = min(max(v3, 0), self.depth_image.shape[0] - 1)
                        v4 = min(max(v4, 0), self.depth_image.shape[0] - 1)
                        u4 = min(max(u4, 0), self.depth_image.shape[1] - 1)

                        R_e1 = self.get_valid_depth(self.depth_image, u1, v1)
                        if R_e1 > 0: self.prev_e1 = R_e1
                        else: R_e1 = self.prev_e1
                        R_e2 = self.get_valid_depth(self.depth_image, u2, v2)
                        if R_e2 > 0: self.prev_e2 = R_e2
                        else: R_e2 = self.prev_e2
                        R_e3 = self.get_valid_depth(self.depth_image, u3, v3) 
                        if R_e3 > 0: self.prev_e3 = R_e3
                        else: R_e3 = self.prev_e3
                        R_e4 = self.get_valid_depth(self.depth_image, u4, v4)
                        if R_e4 > 0: self.prev_e4 = R_e4
                        else: R_e4 = self.prev_e4

                        theta_x1, theta_y1 = 36.5 * (u1 - 320) / 320, -29 * (v1 - 240) / 240
                        theta_x2, theta_y2 = 36.5 * (u2 - 320) / 320, -29 * (v2 - 240) / 240
                        theta_x3, theta_y3 = 36.5 * (u3 - 320) / 320, -29 * (v3 - 240) / 240
                        theta_x4, theta_y4 = 36.5 * (u4 - 320) / 320, -29 * (v4 - 240) / 240

                        self.lock_window_coordinates(R_e1, R_e2, R_e3, R_e4, theta_x1, theta_y1, theta_x2, theta_y2, theta_x3, theta_y3, theta_x4, theta_y4)

                    # cv2.rectangle(self.normalized_depth_image, (x, y), (x + w, y + h), 0, 3)
                    cv2.circle(self.normalized_depth_image, (u1, v1), 6, 0, -1) # E1
                    cv2.circle(self.normalized_depth_image, (u2, v2), 6, 0, -1) # E2
                    cv2.circle(self.normalized_depth_image, (u3, v3), 6, 0, -1) # E3
                    cv2.circle(self.normalized_depth_image, (u4, v4), 6, 0, -1) # E4
                    

            mode_text = "AUTO" if self.autonomous_mode else "MANUAL"
            cv2.putText(cv_image, f"Mode: {mode_text}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(cv_image, f"Target: {self.target_color}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if self.target_color == 'BLUE' else (0,0,255), 2)
            cv2.imshow("Quadrotor Bearing Vision", cv_image) 
            cv2.imshow("Depth Image", self.normalized_depth_image)
            cv2.waitKey(1) 
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def timer_callback(self):

        if self.islanding:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.islanding = False
            return
        
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            if time.time() % 2 < 0.1:
                self.get_logger().info('Drone not armed... Press Y button to arm.')
            if self.buttons[self.Y_BUTTON] == 1:
                self.get_logger().info('Arming the drone...')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            return

        self.publish_offboard_control_mode()

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info('Switching to OFFBOARD mode')
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) 
            return

        if not self.autonomous_mode or not self.ready_to_traverse:
            self.vx, self.vy, self.vz = self.calculate_velocity(self.axes)
            self.yawspeed = -self.MAX_YAWSPEED * self.axes[self.RIGHT_STICK_H]
        else:
            self.yawspeed = 0.0 
            self.update_autonomous_traversal()
            self.vx, self.vy, self.vz = self.auto_vx, self.auto_vy, self.auto_vz

        self.cam_pitch += self.axes[self.RIGHT_STICK_V] * self.CAM_SENSITIVITY
        self.cam_pitch = max(-90.0, min(0.0, self.cam_pitch))

        self.publish_trajectory_setpoint(self.vx, self.vy, self.vz, self.yawspeed)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL, param1=self.cam_pitch, param2=0.0, param3=0.0, param7=2.0)
        
        if self.buttons[self.X_BUTTON] == 1:
            self.islanding = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False 
        msg.velocity = True  
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx, vy, vz, yawspeed):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        nan_val = float('nan')
        msg.position[0], msg.position[1], msg.position[2] = nan_val, nan_val, nan_val
        msg.velocity[0], msg.velocity[1], msg.velocity[2] = vx, vy, vz
        msg.yaw = nan_val
        msg.yawspeed = yawspeed
        msg.acceleration[0], msg.acceleration[1], msg.acceleration[2] = nan_val, nan_val, nan_val
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=3.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1, msg.param2, msg.param3, msg.param7 = param1, param2, param3, param7
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    finally:
        drone_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()