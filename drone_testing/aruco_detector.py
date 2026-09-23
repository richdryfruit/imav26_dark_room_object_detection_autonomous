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
from enum import Enum, auto
from drone_controller.pid_controller import PIDController


TARGET_ARUCO_ID = 1

PID_KP = 0.004              
PID_KI = 0.0001             
PID_KD = 0.002              
PID_INTEGRAL_CLAMP = 50.0   
PID_MAX_OUTPUT = 1.5        

DESCENT_SPEED       = 0.4   
CENTERING_THRESHOLD = 30    
LANDING_ALT         = 0.8   
SEARCH_HOVER_ALT    = -3.0

MARKER_LOST_ASCEND_SPEED = 0.3  
MARKER_LOST_ASCEND_SECS  = 1.5   

class LandingState(Enum):
    IDLE       = auto()   
    SEARCHING  = auto()   
    CENTERING  = auto()   
    DESCENDING = auto()   
    LANDING    = auto()  

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_joystick_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Publishers ───────────────────────────────────────────
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/uav_2/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/uav_2/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/uav_2/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/uav_2/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/uav_2/fmu/in/trajectory_setpoint', 10)

        # ── Subscribers ──────────────────────────────────────────
        self.vehicle_status_subs = subscribe_versioned(self, VehicleStatus, 'vehicle_status', self.vehicle_status_callback, qos_profile)
        self.local_position_subs = subscribe_versioned(self, VehicleLocalPosition, 'vehicle_local_position', self.local_position_callback, qos_profile)
        self.local_orientation_sub = self.create_subscription(VehicleAttitude, '/uav_2/fmu/out/vehicle_attitude',self.local_orientation_callback, qos_profile=qos_profile)
        self.joystick_sub = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)
        self.image_sub = self.create_subscription(Image,'/world/imav2026_scaled/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image',self.image_callback, 10)

        # ── State variables ───────────────────────────────────────
        self.nav_state    = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.local_position    = VehicleLocalPosition()
        self.local_orientation = VehicleAttitude()

        # ── Landing state machine ────────────────────────────────
        self.landing_state: LandingState = LandingState.IDLE
        self.marker_pixel_error: tuple[float, float] | None = None  # (ex, ey) px
        self.marker_detected_id: int | None = None

        self.pid_x = PIDController(PID_KP, PID_KI, PID_KD,PID_INTEGRAL_CLAMP, PID_MAX_OUTPUT)
        self.pid_y = PIDController(PID_KP, PID_KI, PID_KD,PID_INTEGRAL_CLAMP, PID_MAX_OUTPUT)

        self._lost_start_time: float | None = None

        # ── Camera / vision ───────────────────────────────────────
        self.cv_bridge   = CvBridge()
        self.cam_pitch   = 0.0
        self.CAM_SENSITIVITY = 1.5
        self.frame_rate  = 10.0
        self.last_image_time = 0.0
        self.IMAGE_W = 640
        self.IMAGE_H = 480
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # ── Joystick / velocity ───────────────────────────────────
        self.MAX_VELOCITY  = 3.0
        self.BOOST         = 6.0
        self.MAX_YAWSPEED  = 1.5
        self.vx = self.vy = self.vz = self.yawspeed = 0.0

        # Controller button/axis map
        self.LB = 6;  self.RB = 7
        self.LT = 5;  self.RT = 4
        self.LEFT_STICK_H  = 0;  self.LEFT_STICK_V  = 1
        self.RIGHT_STICK_H = 2;  self.RIGHT_STICK_V = 3
        self.Y_BUTTON      = 4;  self.X_BUTTON      = 3
        self.A_BUTTON      = 0;  self.B_BUTTON      = 1
        self.DPAD_UP_DOWN  = 7
        self.axes    = [0] * 8
        self.buttons = [0] * 15

        # TF broadcaster (keep for compatibility)
        self.tf_broadcaster  = TransformBroadcaster(self)
        self.tf_child_frame  = 'drone_base_link'

        # ── Timer ─────────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz


    def vehicle_status_callback(self, msg):
        self.nav_state    = msg.nav_state
        self.arming_state = msg.arming_state

    def local_position_callback(self, msg):
        self.local_position = msg

    def local_orientation_callback(self, msg):
        self.local_orientation = msg

    def joystick_callback(self, msg):
        self.axes    = msg.axes
        self.buttons = msg.buttons

    def image_callback(self, msg):
        current_time = time.time()
        if self.frame_rate > 0:
            if (current_time - self.last_image_time) < 1.0 / self.frame_rate:
                return
        self.last_image_time = current_time

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.resize(cv_image, (self.IMAGE_W, self.IMAGE_H))
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)

        self.marker_pixel_error = None
        self.marker_detected_id = None

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id != TARGET_ARUCO_ID:
                    continue

                c = corners[i][0]          
                cx = float(np.mean(c[:, 0]))
                cy = float(np.mean(c[:, 1]))

                ex = cx - self.IMAGE_W / 2.0
                ey = cy - self.IMAGE_H / 2.0

                self.marker_pixel_error = (ex, -ey)
                self.marker_detected_id = int(marker_id)

                cv2.circle(cv_image, (int(cx), int(cy)), 6, (0, 255, 0), -1)
                cv2.putText(cv_image, f'ID={marker_id} err=({ex:.0f},{ey:.0f})',
                            (int(cx) + 10, int(cy)),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                break 

        state_label = self.landing_state.name
        alt = -self.local_position.z if hasattr(self.local_position, 'z') else 0.0
        cv2.putText(cv_image, f'State: {state_label}  Alt: {alt:.2f}m',
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)
        cv2.putText(cv_image, f'Target ArUco ID: {TARGET_ARUCO_ID}',
                    (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)

        cv2.imshow('Drone Camera View', cv_image)
        cv2.waitKey(1)

    def timer_callback(self):
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            if time.time() % 2 < 0.1:
                self.get_logger().info('Not armed. Press Y to arm.')
            if self.buttons[self.Y_BUTTON] == 1:
                self.get_logger().info('Arming...')
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info('Waiting for external arm signal...')
            return

        self.publish_offboard_control_mode()

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info('Switching to OFFBOARD mode')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            return

        state = self.landing_state
        if   state == LandingState.IDLE:       self._state_idle()
        elif state == LandingState.SEARCHING:  self._state_searching()
        elif state == LandingState.CENTERING:  self._state_centering()
        elif state == LandingState.DESCENDING: self._state_descending()
        elif state == LandingState.LANDING:    self._state_landing()

    def _state_idle(self):
        vx, vy, vz = self._joystick_velocity()
        yawspeed = -self.MAX_YAWSPEED * self.axes[self.RIGHT_STICK_H]

        self.cam_pitch += self.axes[self.RIGHT_STICK_V] * self.CAM_SENSITIVITY
        self.cam_pitch = max(-90.0, min(0.0, self.cam_pitch))

        self.publish_trajectory_setpoint(vx, vy, vz, yawspeed)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_MOUNT_CONTROL,
            param1=self.cam_pitch, param2=0.0, param3=0.0, param7=2.0)

        if self.buttons[self.X_BUTTON] == 1:
            self.get_logger().info(
                f'Precision landing initiated. Searching for marker ID={TARGET_ARUCO_ID}')
            self._transition_to(LandingState.SEARCHING)

    def _state_searching(self):
        self.publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0)

        if self.marker_pixel_error is not None:
            self.get_logger().info(
                f'Marker {TARGET_ARUCO_ID} found! Centering...')
            self._reset_pids()
            self._transition_to(LandingState.CENTERING)

        if self.buttons[self.B_BUTTON] == 1:
            self.get_logger().info('Precision landing aborted.')
            self._transition_to(LandingState.IDLE)

    def _state_centering(self):
        if self.marker_pixel_error is None:
            self.get_logger().warn('Marker lost during centering. Re-searching...')
            self._transition_to(LandingState.SEARCHING)
            return

        ex, ey = self.marker_pixel_error

        body_vx =  self.pid_x.update(ey)   
        body_vy =  self.pid_y.update(ex)   

        heading    = self.local_position.heading
        global_vx  =  body_vx * math.cos(heading) - body_vy * math.sin(heading)
        global_vy  =  body_vx * math.sin(heading) + body_vy * math.cos(heading)

        self.publish_trajectory_setpoint(global_vx, global_vy, 0.0, 0.0)

        pixel_err = math.hypot(ex, ey)
        self.get_logger().info(
            f'Centering — pixel error: {pixel_err:.1f} px', throttle_duration_sec=1.0)

        if pixel_err < CENTERING_THRESHOLD:
            self.get_logger().info(
                f'Centred (err={pixel_err:.1f}px). Starting descent.')
            self._transition_to(LandingState.DESCENDING)

        if self.buttons[self.B_BUTTON] == 1:
            self._transition_to(LandingState.IDLE)

    def _state_descending(self):
        alt = -self.local_position.z   

        if alt < LANDING_ALT:
            self.get_logger().info(
                f'Altitude {alt:.2f}m < {LANDING_ALT}m. Triggering NAV_LAND.')
            self._transition_to(LandingState.LANDING)
            return

        if self.marker_pixel_error is None:
            if self._lost_start_time is None:
                self._lost_start_time = time.monotonic()
                self.get_logger().warn(
                    'Marker lost during descent. Climbing to re-acquire...')

            elapsed = time.monotonic() - self._lost_start_time
            if elapsed < MARKER_LOST_ASCEND_SECS:
                self.publish_trajectory_setpoint(
                    0.0, 0.0, -MARKER_LOST_ASCEND_SPEED, 0.0)  
            else:
                self._lost_start_time = None
                self._reset_pids()
                self._transition_to(LandingState.CENTERING)
            return

        self._lost_start_time = None

        ex, ey = self.marker_pixel_error

        body_vx = self.pid_x.update(ey)
        body_vy = self.pid_y.update(ex)

        heading   = self.local_position.heading
        global_vx =  body_vx * math.cos(heading) - body_vy * math.sin(heading)
        global_vy =  body_vx * math.sin(heading) + body_vy * math.cos(heading)

        self.publish_trajectory_setpoint(
            global_vx, global_vy, DESCENT_SPEED, 0.0)

        pixel_err = math.hypot(ex, ey)
        self.get_logger().info(
            f'Descending — alt={alt:.2f}m  pixel_err={pixel_err:.1f}px',
            throttle_duration_sec=1.0)

        if self.buttons[self.B_BUTTON] == 1:
            self.publish_trajectory_setpoint(0.0, 0.0, -0.5, 0.0)  # pull up
            self._transition_to(LandingState.IDLE)

    def _state_landing(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.get_logger().info('Landed and disarmed.')
            self._transition_to(LandingState.IDLE)

    def _transition_to(self, new_state: LandingState):
        self.get_logger().info(f'State: {self.landing_state.name} → {new_state.name}')
        self.landing_state = new_state

    def _reset_pids(self):
        self.pid_x.reset()
        self.pid_y.reset()

    def _joystick_velocity(self):
        axes = self.axes
        boost = self.BOOST if axes[self.DPAD_UP_DOWN] != 0 else self.MAX_VELOCITY
        vx = axes[self.LEFT_STICK_V] * boost
        vy = axes[self.LEFT_STICK_H] * boost
        vz = 0.0
        if axes[self.LT] < 0.0 and axes[self.RT] > 0.0:
            vz = -axes[self.LT] * self.MAX_VELOCITY
        elif axes[self.LT] > 0.0 and axes[self.RT] < 0.0:
            vz = axes[self.RT] * self.MAX_VELOCITY
        heading   = self.local_position.heading
        global_vx =  vx * math.cos(heading) + vy * math.sin(heading)
        global_vy =  vx * math.sin(heading) - vy * math.cos(heading)
        return global_vx, global_vy, vz

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position     = False
        msg.velocity     = True
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = True
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx, vy, vz, yawspeed):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        nan = float('nan')
        msg.position[0], msg.position[1], msg.position[2] = nan, nan, nan
        msg.velocity[0], msg.velocity[1], msg.velocity[2] = vx, vy, vz
        msg.yaw      = nan
        msg.yawspeed = yawspeed
        msg.acceleration[0], msg.acceleration[1], msg.acceleration[2] = nan, nan, nan
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command,param1=0.0, param2=0.0,param3=3.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command  = command
        msg.param1, msg.param2, msg.param3, msg.param7 = param1, param2, param3, param7
        msg.target_system    = 1;  msg.target_component  = 1
        msg.source_system    = 1;  msg.source_component  = 1
        msg.from_external    = True
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