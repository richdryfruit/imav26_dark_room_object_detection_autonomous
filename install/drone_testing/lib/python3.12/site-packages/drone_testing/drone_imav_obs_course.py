import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition, VehicleAttitude, VehicleCommandAck
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import time
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
        # self.ack_sub = self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.ack_callback, qos_profile=qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile=qos_profile)
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_callback, qos_profile=qos_profile)
        self.local_orienation_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.local_orientation_callback, qos_profile=qos_profile)
        self.joystick_sub = self.create_subscription(Joy, '/joy',self.joystick_callback, 10)
        self.depth_data_sub = self.create_subscription(Float32MultiArray, '/depth_data', self.depth_data_callback, 10)

        # --- State Variables ---
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.local_position = VehicleLocalPosition()
        self.local_orientation = VehicleAttitude()
        self.islanding = False
        
        # --- Timers ---
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz     

        # --- Velocity Variables ---
        self.MAX_VELOCITY = 1.5 
        self.BOOST = 3.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.yawspeed = 0.0
        self.MAX_YAWSPEED = 1.5

        # --- Autonomous Traversal Variables ---
        self.autonomous_mode = False
        self.counter = 0
        self.set_points = None
        self.goal_point = None
        self.start_point = None
        self.dist_to_point = 0.0
        self.chi_d = 0.0
        self.gamma_d = 0.0
        self.k_chi = 0.3
        self.k_gamma = 0.3
        self.delta = 0.35
        self.autonomous_velocity = 0.4
        self.ned_to_gz = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]])
        self.cam_to_frd = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
        ])
        self.optimal_centre = None
        self.alpha = 0.25  # Smoothing factor. Tune this!

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
    
    # def ack_callback(self, msg):
    #     self.get_logger().info(f'Command {msg.command} result: {msg.result}')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    def local_position_callback(self, msg):
        self.local_position = msg
    
    def local_orientation_callback(self, msg):
        self.local_orientation = msg
    
    def depth_data_callback(self, msg):
        data = msg.data

        if not self.autonomous_mode or self.set_points is None:
            depth_data = np.array(data).reshape(4, 3)
            self.window_coordinates(depth_data)

    def window_coordinates(self, depth_data):
        
        camera_offset_frd = np.array([0.01233, 0.03000, -0.01878])

        e1_frd = np.array([depth_data[0][0], depth_data[0][0] * math.tan(math.radians(depth_data[0][1])), -depth_data[0][0] * math.tan(math.radians(depth_data[0][2]))]) + camera_offset_frd
        e2_frd = np.array([depth_data[1][0], depth_data[1][0] * math.tan(math.radians(depth_data[1][1])), -depth_data[1][0] * math.tan(math.radians(depth_data[1][2]))]) + camera_offset_frd
        e3_frd = np.array([depth_data[2][0], depth_data[2][0] * math.tan(math.radians(depth_data[2][1])), -depth_data[2][0] * math.tan(math.radians(depth_data[2][2]))]) + camera_offset_frd
        e4_frd = np.array([depth_data[3][0], depth_data[3][0] * math.tan(math.radians(depth_data[3][1])), -depth_data[3][0] * math.tan(math.radians(depth_data[3][2]))]) + camera_offset_frd

        centre_frd = (e1_frd + e2_frd + e3_frd + e4_frd) / 4.0
        w = self.local_orientation.q[0]
        xyz = self.local_orientation.q[1:4]
        rotate_q = Quaternions(w, xyz)
        p_drone_ned = np.array([self.local_position.x, self.local_position.y, self.local_position.z])
        centre_gz = np.dot(self.ned_to_gz, rotate_q.rotate_vector(centre_frd) + p_drone_ned)

        # width = np.linalg.norm(e2_frd - e1_frd) / 2.2
        # height = np.linalg.norm(e4_frd - e1_frd) / 2.2
        centre = centre_gz / 2.2

        if self.optimal_centre is None:
            # Initialize with the very first reading
            self.optimal_centre = centre
        else:
            # Apply the EMA formula
            self.optimal_centre = (self.alpha * centre) + ((1.0 - self.alpha) * self.optimal_centre)

        self.guidance_planner(centre)
        
        # self.get_logger().info(f'Window Width: {width:.2f} m, Window Height: {height:.2f} m')
        self.get_logger().info(f'Window Centre: x={self.optimal_centre[0]:.2f}, y={self.optimal_centre[1]:.2f}, z={self.optimal_centre[2]:.2f}')
        
    def guidance_planner(self, centre):
        # self.set_points = 2.2 * np.array([[1.7, 0.0, 1.2], [1.7, 0.9, 1.2], [1.7, 1.3, 1.2], [2.0, 1.7, 1.9],
        #                                   [2.0, 2.1, 1.9], [2.0, 2.6, 0.4], [2.0, 3.6, 0.4], [2.0, 4.5, 0.4],
        #                                   [2.1,5.35,0.75], [1.7, 5.9,0.75], [1.7, 7.5, 1.2], [1.7, 8.0, 1.2]])

        x, y, z = centre[0], centre[1] - 0.9, centre[2]
        if self.set_points is None:
            self.set_points = 2.2 * np.array([[x, y, z],
                                            [x, y + 0.9, z],
                                            [x, y + 1.3, z],
                                            [x + 0.3, y + 1.7, z + 0.7],
                                            [x + 0.3, y + 2.1, z + 0.7],
                                            [x + 0.3, y + 2.6, z - 0.8],
                                            [x + 0.3, y + 3.6, z - 0.8],
                                            [x + 0.3, y + 4.5, z - 0.8],
                                            [x + 0.4,y + 5.35,z - 0.45],
                                            [x - 0.1, y + 5.9 ,z - 0.45],
                                            [x, y + 7.5, z],
                                            [x, y + 8.0, z],
                                            [x, y + 9.0, z]])
            
            

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
        
        if self.buttons[self.A_BUTTON] == 1 and self.last_a_button == 0:
            self.autonomous_mode = not self.autonomous_mode

            if self.start_point is None:
                self.counter = 0
                start_point_ned = np.array([self.local_position.x, self.local_position.y, self.local_position.z])
                self.start_point = np.dot(start_point_ned, self.ned_to_gz)
                self.goal_point = self.set_points[self.counter, :]
                self.get_logger().info(f'Start Point in GZ Frame: x={self.start_point[0]:.2f}, y={self.start_point[1]:.2f}, z={self.start_point[2]:.2f}')
                self.get_logger().info(f'Goal Point in GZ Frame: x={self.goal_point[0]:.2f}, y={self.goal_point[1]:.2f}, z={self.goal_point[2]:.2f}')

            mode_str = "AUTONOMOUS" if self.autonomous_mode else "MANUAL"
            self.get_logger().info(f'--- Switched to {mode_str} MODE ---')
        self.last_a_button = self.buttons[self.A_BUTTON]

    
    def carrot_chasing_straight(self, W_i, W_f, delta, k_chi, k_gamma):
        
        p_ned = np.array([self.local_position.x, self.local_position.y, self.local_position.z])
        p = np.dot(p_ned, self.ned_to_gz)
        # self.get_logger().info(f'goal pose: {W_f}')
        # self.get_logger().info(f'current_pose: x={p[0]:.2f}, y={p[1]:.2f}, z={p[2]:.2f},')
        Ru = p - W_i
        d = np.linalg.norm(W_f - W_i)

        if d < 1e-6:
            self.get_logger.info("## Error dividing by zero..")
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        
        b = (W_f - W_i) / np.linalg.norm(W_f - W_i)
        R = np.dot(Ru, b)
        carrot_point = W_i + (R + delta) * b
        chi_d = np.arctan2(carrot_point[1] - p[1], carrot_point[0] - p[0])
        gamma_d = np.arcsin((carrot_point[2] - p[2]) / np.linalg.norm(carrot_point - p))
        
        chi = np.arctan2(self.local_position.vy, self.local_position.vx)
        gamma = np.arctan2(self.local_position.vz, np.linalg.norm([self.local_position.vx, self.local_position.vy]))


        chi_dot = k_chi * np.arctan2(np.sin(chi_d - chi), np.cos(chi_d - chi))
        gamma_dot = k_gamma * np.arctan2(np.sin(gamma_d - gamma), np.cos(gamma_d - gamma))
        yaw_dot = k_chi * np.arctan2(np.sin(0.0 - self.local_position.heading), np.cos(0.0 - self.local_position.heading))

        n_unit = np.array([np.cos(chi), np.sin(chi), 0])
        gamma_unit = np.cross(n_unit, np.array([0.0, 0.0, 1.0]))

        acc_chi = chi_dot * np.cross(np.array([0.0, 0.0, 1.0]), np.array([self.local_position.vx, self.local_position.vy, 0.0]))
        acc_gamma = gamma_dot * np.cross(gamma_unit, np.array([self.local_position.vx, self.local_position.vy, self.local_position.vz]))
        acc_gz = acc_chi + acc_gamma
        vel_gz = self.autonomous_velocity * np.array([np.cos(chi_d) * np.cos(gamma_d), np.sin(chi_d) * np.cos(gamma_d), np.sin(gamma_d)])

        acc_ned = np.dot(acc_gz, self.ned_to_gz.T)
        ax, ay, az = acc_ned[0], acc_ned[1], acc_ned[2]
        
        vel_ned = np.dot(vel_gz, self.ned_to_gz.T)
        vx, vy, vz = vel_ned[0], vel_ned[1], vel_ned[2]

        return vx, vy, vz, ax, ay, az, yaw_dot


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

        if not self.autonomous_mode:
            self.ax = float('nan')
            self.ay = float('nan')
            self.az = float('nan')
            self.vx, self.vy, self.vz = self.calculate_velocity(self.axes)
            self.yawspeed = -self.MAX_YAWSPEED * self.axes[self.RIGHT_STICK_H]

        else:
            dist_to_goal = np.linalg.norm(self.goal_point- np.dot(np.array([self.local_position.x, self.local_position.y, self.local_position.z]), self.ned_to_gz))
            self.vx, self.vy, self.vz, self.ax, self.ay, self.az, self.yawspeed = self.carrot_chasing_straight(self.start_point, self.goal_point, self.delta, self.k_chi, self.k_gamma)
            
            if dist_to_goal < 0.2:
                self.start_point = self.goal_point
                self.counter += 1
                if self.counter < len(self.set_points):
                    self.goal_point = self.set_points[self.counter, :]

                else:
                    self.autonomous_mode = False
                    self.get_logger().info("Done obstacle traversing using carrot chasing algorithm")

            # self.get_logger().info(f'Distance to Goal: {dist_to_goal:.2f} m')
            # self.ax = float('nan')
            # self.ay = float('nan')
            # self.az = float('nan')
            # self.vx = 0.0
            # self.vy = 0.0
            # self.vz = 0.0
            
        self.publish_trajectory_setpoint(self.vx, self.vy, self.vz, self.yawspeed, ax=self.ax, ay=self.ay, az=self.az)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL, param2=0.0, param3=0.0, param7=2.0)
        
        if self.buttons[self.X_BUTTON] == 1:
            self.islanding = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False 
        msg.velocity = True  
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = True
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx, vy, vz, yawspeed, ax=0.0, ay=0.0, az=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        nan_val = float('nan')
        msg.position[0], msg.position[1], msg.position[2] = nan_val, nan_val, nan_val
        msg.velocity[0], msg.velocity[1], msg.velocity[2] = vx, vy, vz
        msg.yaw = nan_val
        msg.yawspeed = yawspeed
        msg.acceleration[0], msg.acceleration[1], msg.acceleration[2] = ax, ay, az
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

