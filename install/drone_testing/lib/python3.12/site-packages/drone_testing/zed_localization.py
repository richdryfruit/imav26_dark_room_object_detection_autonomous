"""
Bridge the ZED's visual odometry into PX4 as an external-vision estimate.

    /zed/zed_node/odom  (nav_msgs/Odometry, ENU world / FLU body)
        -> /fmu/in/vehicle_visual_odometry  (px4_msgs/VehicleOdometry, NED / FRD)

WHAT THIS CAMERA ACTUALLY GIVES YOU
-----------------------------------
The camera is a gen-1 ZED (`general.camera_model: 'zed'`). It has NO IMU --
the wrapper refuses to even read the sensors parameters for this model:

    zed_camera_component_main.cpp:1571
        if (sl_tools::isZED(mCamUserModel)) {
            RCLCPP_WARN(get_logger(),
                "!!! SENSORS parameters are not used with ZED !!!");
            return;
        }

so `pos_tracking.imu_fusion: true` is a no-op here. What comes out of
/zed/zed_node/odom is pure stereo VISUAL odometry at the grab rate, not
visual-INERTIAL odometry. It is still a good lateral position source -- much
better than optical flow over a featureless floor -- but it has no inertial
prior to carry it through motion blur, a fast yaw, or a frame full of
featureless wall, and it will drop out on all three. Everything below is
built on the assumption that the estimate CAN vanish mid-flight.

FRAMES
------
ROS (REP-103/105) is ENU world, FLU body. PX4 is NED world, FRD body. The
rotation between them is a fixed pair of frame changes -- a 90 deg yaw plus a
180 deg roll -- which for a Hamiltonian quaternion (w, x, y, z) works out to

    q_ned = 1/sqrt(2) * (w + z,  x + y,  x - y,  w - z)

(derived as q_ned_enu (x) q_enu (x) q_flu_frd, with q_ned_enu = (0, r, r, 0)
and q_flu_frd = (0, 1, 0, 0), r = sqrt(2)/2; the sanity check is that ENU
identity -- body forward pointing East -- must map to a +90 deg NED yaw, and
it does.)

The old version of this file used [w, y, x, -z], which is a coordinate SWAP
rather than a rotation: it is not a valid quaternion mapping, it does not
compose, and it hands PX4 an attitude that is wrong by 90 deg in yaw for any
non-trivial pose. Do not go back to it.

Position is the easy half: NED = (y_enu, x_enu, -z_enu).

WHICH FRAME TO DECLARE
----------------------
`pose_frame` defaults to FRD, not NED. The ZED's odom frame is ENU *anchored
on wherever the camera was looking at start-up* -- its x axis is the initial
heading, not true East, because nothing in this camera can observe North (no
magnetometer, no GNSS). Declaring NED would be telling PX4 the vision yaw is
absolute when it is off by an arbitrary constant, and EKF2 would fight its own
magnetometer forever. POSE_FRAME_FRD is exactly the "z is down, heading offset
from North is a constant I do not know" case.

THAT DEFAULT IS ONLY CORRECT IF THE MAGNETOMETER IS ON. Read this before you
fly with EKF2_MAG_TYPE = 5.

FRD can never align yaw. Not "will not in practice" -- cannot, by construction:

    ev_yaw_control.cpp, LOCAL_FRAME_FRD branch
        resetQuatStateYaw(...);
        _control_status.flags.yaw_align = false;   <-- explicitly false
        _control_status.flags.ev_yaw    = true;

Only the LOCAL_FRAME_NED branch sets yaw_align = true. So FRD is a declaration
that some OTHER source owns the heading, and on this airframe the only other
source is the magnetometer. Turn the magnetometer off and declare FRD and you
get a vehicle that fuses vision position and vision yaw, reports cs_ev_pos and
cs_ev_yaw both true, looks completely healthy on the ground -- and has
cs_yaw_align false forever, which PX4 reports as local_position_invalid about
a second after arming, and takes the aircraft.

With no magnetometer and no GNSS, use `pose_frame:=ned`. Nothing in the system
knows where North is, so there is no absolute heading for the vision yaw to
disagree with, and letting the ZED's start-up heading DEFINE the navigation
frame's north is self-consistent: the flight nodes capture their own reference
yaw at arming and fly everything relative to it. The only thing you give up is
that the reported heading is no longer North-referenced, which matters to a
compass rose in QGC and to nothing else indoors.

    magnetometer ON  (EKF2_MAG_TYPE = 0)  ->  pose_frame:=frd, EKF2_EV_CTRL = 1
    magnetometer OFF (EKF2_MAG_TYPE = 5)  ->  pose_frame:=ned, EKF2_EV_CTRL = 9

Do not mix the rows. Check the result before every first flight on a new
parameter set:

    ros2 topic echo /fmu/out/estimator_status_flags --once | grep cs_yaw_align

WHERE THE CAMERA IS BOLTED ON
-----------------------------
The odometry the wrapper publishes is the pose of the CAMERA, not of the
vehicle. The child frame is hardcoded:

    zed_camera_component_main.cpp:1655
        mBaseFrameId = mCameraName;
        mBaseFrameId += "_camera_link";

There is no parameter to make it report base_link, and publishing a static
base_link -> zed_camera_link transform does not change it -- the wrapper does
not consult TF for this. So the mounting offset has to be applied here.

That matters more than it sounds. A camera 10 cm forward of the CoG turns
every yaw of the airframe into a 10 cm phantom translation; a camera pitched
down 20 degrees means the vision frame's "forward" is not the vehicle's
"forward", so a commanded forward move is flown partly downward. EKF2 has
parameters for the LEVER ARM (EKF2_EV_POS_X/Y/Z) but none at all for the
camera's ROTATION, which is why both are done here instead of split across
two places.

Set `cam_x/cam_y/cam_z` and `cam_roll/cam_pitch/cam_yaw` to the pose of the
camera in the body frame, ROS convention: x forward, y LEFT, z UP, angles in
radians, applied yaw-then-pitch-then-roll. Defaults are all zero, i.e. camera
at the CoG pointing straight forward, which is a no-op. Leave EKF2_EV_POS_*
at zero when you use these.

PUBLISH RATE
------------
`publish_rate` (default 15 Hz) throttles what goes DOWN THE LINK to PX4,
independently of how fast the ZED runs. This is not a nicety -- it is the
difference between flying and not.

The uXRCE-DDS serial link is a 921600-baud UART shared by every ROS->PX4
topic. Streaming 30 Hz of odometry down it alongside the 20 Hz setpoint and
heartbeat streams saturates it, and what you get is not graceful degradation:
the ulogs from the first vision flights show PX4 receiving vision at 5-6 Hz
with 0.8 s gaps, and the offboard_control_mode heartbeat dying completely
after under a second, which PX4 reports as offboard_control_signal_lost and
acts on by taking the aircraft. Meanwhile EKF2 never rejected a single vision
sample -- innovations were millimetres. The data was good; it just was not
arriving.

EKF2 does not need 30 Hz vision. It fuses at its own delayed horizon and 10-15
Hz is ample at the speeds this vehicle flies. Set this to 0 to disable the
throttle only if the link is Ethernet, where the bandwidth argument does not
apply.

Health accounting below is deliberately measured on the INCOMING ZED rate,
not the throttled output, so `min_rate` still tells you the truth about the
camera rather than about this setting.

VELOCITY
--------
Off by default (`publish_velocity:=false`, fields left as NaN, which PX4
reads as "not provided"). The position and yaw are what you want from vision;
the twist adds little and gets the frame wrong easily -- nav_msgs/Odometry
defines twist in the child_frame_id frame (the body, FLU), so it must be published
as VELOCITY_FRAME_BODY_FRD, not as an NED velocity. If you turn it on, that
is what this node does.

See also the PX4 parameter notes in launch/sequence_vio_test.launch.py.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleOdometry

# 1/sqrt(2), the scale factor in the ENU->NED quaternion above.
_R = math.sqrt(0.5)


def quat_mul(a, b):
    """Hamilton product of two (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_rotate(q, v):
    """Rotate a 3-vector by a (w, x, y, z) quaternion."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v); v' = v + w*t + q_vec x t
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quat_from_rpy(roll, pitch, yaw):
    """(w, x, y, z) for an intrinsic yaw-pitch-roll (ZYX) rotation."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def enu_flu_to_ned_frd(w, x, y, z):
    """Rotate a ROS (ENU world, FLU body) quaternion into PX4's (NED, FRD).

    Input and output are both Hamiltonian (w, x, y, z). Re-normalised on the
    way out: the ZED's quaternion is already unit, but a NaN or a zero from a
    dropped tracking frame would otherwise be handed straight to the EKF.
    """
    qw = _R * (w + z)
    qx = _R * (x + y)
    qy = _R * (x - y)
    qz = _R * (w - z)
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if not math.isfinite(n) or n < 1e-6:
        return None
    return [qw / n, qx / n, qy / n, qz / n]


class ZedLocalization(Node):

    # Defaults, all overridable as ROS parameters.
    ODOM_TOPIC = '/zed/zed_node/odom'
    POSE_FRAME = 'frd'          # frd | ned -- see the module docstring
    PUBLISH_VELOCITY = False
    # Variance floors, m^2 and rad^2. The ZED reports a covariance, but on a
    # gen-1 camera it is optimistic: it describes the solver's fit, not the
    # drift of an unaided VO chain. Never let EKF2 be told this estimate is
    # better than these numbers.
    MIN_POSITION_VARIANCE = 0.01        # 10 cm 1-sigma
    MIN_ORIENTATION_VARIANCE = 0.0025   # ~2.9 deg 1-sigma
    MIN_VELOCITY_VARIANCE = 0.04        # 20 cm/s 1-sigma
    # A single-sample position jump larger than this is a relocalisation
    # (loop closure, tracking recovery), not motion. See _detect_reset.
    RESET_JUMP = 0.30                   # m
    # Health: the bridge reports unhealthy if the last ZED sample is older
    # than this, or if fewer than MIN_RATE samples arrived in the last second.
    MAX_AGE = 0.30                      # s
    MIN_RATE = 10.0                     # Hz
    # Hz sent to PX4. 0 = every sample. See "PUBLISH RATE" above -- 30 Hz
    # saturates the uXRCE-DDS UART and kills the offboard heartbeat.
    PUBLISH_RATE = 15.0

    def __init__(self):
        super().__init__('zed_localization')

        self.odom_topic = str(self.declare_parameter(
            'odom_topic', self.ODOM_TOPIC).value)
        pose_frame = str(self.declare_parameter(
            'pose_frame', self.POSE_FRAME).value).strip().lower()
        if pose_frame not in ('frd', 'ned'):
            self.get_logger().error(
                f"Unknown pose_frame '{pose_frame}'; expected 'frd' or 'ned'. "
                "Falling back to 'frd'.")
            pose_frame = 'frd'
        self.pose_frame = (VehicleOdometry.POSE_FRAME_NED if pose_frame == 'ned'
                           else VehicleOdometry.POSE_FRAME_FRD)
        self.publish_velocity = bool(self.declare_parameter(
            'publish_velocity', self.PUBLISH_VELOCITY).value)
        self.min_position_variance = float(self.declare_parameter(
            'min_position_variance', self.MIN_POSITION_VARIANCE).value)
        self.min_orientation_variance = float(self.declare_parameter(
            'min_orientation_variance', self.MIN_ORIENTATION_VARIANCE).value)
        self.min_velocity_variance = float(self.declare_parameter(
            'min_velocity_variance', self.MIN_VELOCITY_VARIANCE).value)
        # False (the default) sends one clock for both timestamp fields; True
        # restores the ZED's capture stamp in timestamp_sample. See the long
        # comment where the message is built before turning this on -- it is
        # the more correct-looking option and it is what stopped EKF2 fusing.
        self.use_sample_timestamp = bool(self.declare_parameter(
            'use_sample_timestamp', False).value)
        self.reset_jump = float(self.declare_parameter(
            'reset_jump', self.RESET_JUMP).value)
        self.max_age = float(self.declare_parameter('max_age', self.MAX_AGE).value)
        self.min_rate = float(self.declare_parameter('min_rate', self.MIN_RATE).value)
        publish_rate = float(self.declare_parameter(
            'publish_rate', self.PUBLISH_RATE).value)
        self.publish_interval = (1.0 / publish_rate) if publish_rate > 0.0 else 0.0

        # Pose of the camera in the body frame, ROS convention (x fwd, y left,
        # z up). See "WHERE THE CAMERA IS BOLTED ON" above. All zero = no-op.
        cam_x = float(self.declare_parameter('cam_x', 0.0).value)
        cam_y = float(self.declare_parameter('cam_y', 0.0).value)
        cam_z = float(self.declare_parameter('cam_z', 0.0).value)
        cam_roll = float(self.declare_parameter('cam_roll', 0.0).value)
        cam_pitch = float(self.declare_parameter('cam_pitch', 0.0).value)
        cam_yaw = float(self.declare_parameter('cam_yaw', 0.0).value)

        # We are handed T_body_cam and need T_cam_body to push the camera pose
        # back to the body: R_cb = R_bc^-1, p_cb = -R_bc^-1 * p_bc.
        q_bc = quat_from_rpy(cam_roll, cam_pitch, cam_yaw)
        self.q_cam_body = (q_bc[0], -q_bc[1], -q_bc[2], -q_bc[3])
        self.p_cam_body = tuple(
            -v for v in quat_rotate(self.q_cam_body, (cam_x, cam_y, cam_z)))
        self.mounted = any(abs(v) > 1e-9 for v in
                           (cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw))

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.zed_odom_callback,
            qos_profile_sensor_data)
        self.visual_odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos)

        # Health for the flight node. Reliable + transient-local so a node
        # that starts late still gets the current answer immediately instead
        # of having to wait for the next transition.
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.healthy_pub = self.create_publisher(Bool, 'vio_healthy', status_qos)
        self.status_pub = self.create_publisher(String, 'vio_status', status_qos)

        self.last_msg_time = None       # monotonic, last ZED sample received
        self.last_publish_time = None   # monotonic, last sample sent to PX4
        self.dropped = 0                # samples skipped by the rate throttle
        self.last_position = None       # NED, for the jump detector
        self.reset_counter = 0
        self.published = 0
        self.rate_window = []           # monotonic stamps, last second
        self.healthy = None             # None = not yet reported

        # Health is evaluated on a timer, not only in the callback: a bridge
        # that has stopped receiving anything must still be able to SAY so.
        self.health_timer = self.create_timer(0.1, self.publish_health)

        self.get_logger().warning(
            f"ZED VO bridge: {self.odom_topic} -> /fmu/in/vehicle_visual_odometry "
            f"as POSE_FRAME_{'NED' if pose_frame == 'ned' else 'FRD'}, "
            f"velocity {'ON' if self.publish_velocity else 'OFF (NaN)'}, "
            f"mounting offset {'APPLIED' if self.mounted else 'none (camera == body)'}, "
            f"timestamps {'CAPTURE STAMP' if self.use_sample_timestamp else 'single clock'}, "
            f"publishing at {publish_rate:.0f} Hz"
            if publish_rate > 0.0 else "publishing every sample (throttle OFF)")
        self.get_logger().warning(
            "This camera has no IMU: it is visual odometry, not visual-inertial.")

    # ---------------------------------------------------------------- bridge

    def zed_odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Still in ROS's ENU world / FLU body at this point.
        p_enu = (float(p.x), float(p.y), float(p.z))
        q_enu = (float(q.w), float(q.x), float(q.y), float(q.z))
        if not all(math.isfinite(v) for v in p_enu + q_enu):
            self.get_logger().warning("Dropped ZED odom sample: non-finite pose.",
                                      throttle_duration_sec=2.0)
            return

        if self.mounted:
            # Camera pose -> vehicle pose. T_odom_body = T_odom_cam * T_cam_body.
            p_enu = tuple(a + b for a, b in zip(
                p_enu, quat_rotate(q_enu, self.p_cam_body)))
            q_enu = quat_mul(q_enu, self.q_cam_body)

        # ENU -> NED, the easy half: N = y, E = x, D = -z.
        position = [p_enu[1], p_enu[0], -p_enu[2]]
        q_ned = enu_flu_to_ned_frd(*q_enu)
        if q_ned is None:
            self.get_logger().warning("Dropped ZED odom sample: bad quaternion.",
                                      throttle_duration_sec=2.0)
            return

        now = time.monotonic()

        # Health and relocalisation detection run on EVERY sample, before the
        # throttle. Health must describe the camera, not this setting; and a
        # jump that happened between two published samples is still a jump,
        # so reset_counter has to see the ones we drop.
        self.last_msg_time = now
        self.rate_window.append(now)
        self.rate_window = [t for t in self.rate_window if now - t <= 1.0]
        self._detect_reset(position)

        # Rate throttle. Keeping the link under its budget is what stops the
        # offboard heartbeat being starved -- see "PUBLISH RATE" above.
        if (self.publish_interval > 0.0 and self.last_publish_time is not None
                and now - self.last_publish_time < self.publish_interval * 0.98):
            self.dropped += 1
            return
        self.last_publish_time = now

        out = VehicleOdometry()
        # BOTH timestamps are "now", on this machine's clock, and that is
        # deliberate -- see use_sample_timestamp.
        #
        # The obvious thing is to put the ZED's capture stamp in
        # timestamp_sample so EKF2 can fuse the sample at the right point in
        # its history buffer. It does not work, and the failure is silent and
        # vicious. PX4's uXRCE-DDS client applies its timesync offset when it
        # converts an inbound message onto the flight controller's hrt clock,
        # which counts microseconds since PX4 booted. Our two stamps are on
        # the Jetson's clock, which counts microseconds since 1970. If the
        # client translates `timestamp` and leaves `timestamp_sample` alone,
        # EKF2 is handed a sample whose measurement time is decades in the
        # future, places it outside its fusion time horizon, and cannot fuse
        # it. It then does what it does whenever vision aiding times out: it
        # re-anchors, resetHorizontalPositionToVision(), over and over.
        #
        # From the outside that looks exactly like the failure in
        # test_debugs.txt -- a stream of "EKF2 lateral reset: delta_xy=(+0.00,
        # +0.00)" at several hertz, ekf_fusing reporting True the whole time
        # because cs_ev_pos really is set, and PX4 raising
        # local_position_invalid the moment arming makes it enforce validity.
        #
        # Sending one consistent clock for both fields costs the ~60 ms of
        # capture-to-publish latency as a fixed lag, which is what EKF2_EV_DELAY
        # exists to absorb. Set it to that measured latency (40 ms is the
        # starting point in the launch header) rather than trying to be exact
        # here.
        out.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        if self.use_sample_timestamp:
            out.timestamp_sample = int(
                rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1000)
            if out.timestamp_sample <= 0:
                out.timestamp_sample = out.timestamp
        else:
            out.timestamp_sample = out.timestamp

        out.pose_frame = self.pose_frame
        out.position = position
        out.q = q_ned

        pc = msg.pose.covariance
        out.position_variance = [
            max(float(pc[7]), self.min_position_variance),   # ENU y -> N
            max(float(pc[0]), self.min_position_variance),   # ENU x -> E
            max(float(pc[14]), self.min_position_variance),  # ENU z -> D
        ]
        out.orientation_variance = [
            max(float(pc[21]), self.min_orientation_variance),
            max(float(pc[28]), self.min_orientation_variance),
            max(float(pc[35]), self.min_orientation_variance),
        ]

        if self.publish_velocity:
            # nav_msgs/Odometry defines twist in child_frame_id -- the body,
            # FLU -- so this is a body velocity, and FLU -> FRD is just
            # (x, -y, -z). Declaring it as an NED velocity here would be a
            # silent, heading-dependent error.
            t = msg.twist.twist
            v_flu = (float(t.linear.x), float(t.linear.y), float(t.linear.z))
            w_flu = (float(t.angular.x), float(t.angular.y), float(t.angular.z))
            if self.mounted:
                # Rotate the camera-frame twist into the body frame. The lever
                # arm term (w x r) is deliberately NOT added: the twist is the
                # camera's motion, and on a vehicle that is mostly translating
                # the correction is small, while getting its sign wrong is not.
                # This is another reason velocity is off by default.
                v_flu = quat_rotate(self.q_cam_body, v_flu)
                w_flu = quat_rotate(self.q_cam_body, w_flu)
            # FLU -> FRD is (x, -y, -z).
            out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
            out.velocity = [v_flu[0], -v_flu[1], -v_flu[2]]
            out.angular_velocity = [w_flu[0], -w_flu[1], -w_flu[2]]
            tc = msg.twist.covariance
            out.velocity_variance = [
                max(float(tc[0]), self.min_velocity_variance),
                max(float(tc[7]), self.min_velocity_variance),
                max(float(tc[14]), self.min_velocity_variance),
            ]
        else:
            nan = float('nan')
            out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            out.velocity = [nan, nan, nan]
            out.angular_velocity = [nan, nan, nan]
            out.velocity_variance = [nan, nan, nan]

        out.reset_counter = self.reset_counter % 256
        # 0 means "unknown" to EKF2. It is only actually consulted when
        # EKF2_EV_QMIN > 0, but saying "good" on a sample we have just health-
        # checked is both honest and one less thing that can silently gate
        # fusion off if that parameter is ever raised.
        out.quality = 100

        self.visual_odom_pub.publish(out)

        self.published += 1

    def _detect_reset(self, position):
        """Bump reset_counter when the ZED relocalises instead of moving.

        `reset_odom_with_loop_closure` is on in the stock ZED config, so the
        odometry can teleport by metres in one sample when the camera
        recognises somewhere it has been. To EKF2 an unannounced teleport is a
        gigantic innovation and it will either reject the vision entirely or
        yank the vehicle. Incrementing reset_counter is how the message says
        "this is a new datum, not a measurement" -- PX4 then re-anchors
        instead of fusing the jump.
        """
        if self.last_position is not None:
            jump = math.dist(position, self.last_position)
            if jump > self.reset_jump:
                self.reset_counter += 1
                self.get_logger().warning(
                    f"ZED odometry jumped {jump:.2f} m in one sample -- treating "
                    f"it as a relocalisation (reset_counter={self.reset_counter}).")
        self.last_position = position

    # ---------------------------------------------------------------- health

    def measured_rate(self):
        return float(len(self.rate_window))

    def is_healthy(self):
        if self.last_msg_time is None:
            return False, "no ZED odometry received yet"
        age = time.monotonic() - self.last_msg_time
        if age > self.max_age:
            return False, f"ZED odometry is {age * 1000:.0f} ms stale"
        rate = self.measured_rate()
        if rate < self.min_rate:
            return False, f"ZED odometry only {rate:.0f} Hz"
        return True, "ok"

    def publish_health(self):
        healthy, reason = self.is_healthy()

        if healthy != self.healthy:
            self.healthy = healthy
            if healthy:
                self.get_logger().warning(
                    f"VIO healthy: {self.measured_rate():.0f} Hz from {self.odom_topic}.")
            else:
                self.get_logger().error(f"VIO UNHEALTHY: {reason}.")

        self.healthy_pub.publish(Bool(data=healthy))

        age_ms = (0.0 if self.last_msg_time is None
                  else (time.monotonic() - self.last_msg_time) * 1000.0)
        status = String()
        # in_hz is what the camera is giving us; out is what the link is
        # actually carrying. They differ by the publish_rate throttle, and
        # seeing both is how you tell a dead camera from a throttled one.
        out_hz = (0.0 if self.publish_interval <= 0.0
                  else min(1.0 / self.publish_interval, self.measured_rate()))
        status.data = "|".join([
            'OK' if healthy else 'BAD',
            f"{self.measured_rate():.0f}",
            f"{out_hz if self.publish_interval > 0.0 else self.measured_rate():.0f}",
            f"{age_ms:.0f}",
            f"{self.reset_counter}",
            reason,
        ])
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ZedLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
