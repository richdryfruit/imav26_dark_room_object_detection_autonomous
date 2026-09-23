"""
Sequence test flown on RealSense D435i + RTAB-Map visual-inertial odometry.

    offboard_sequence_vio, flown on the rtabmap_realsense_vio stack.

The uXRCE-DDS agent and the VIO stack (RealSense + Madgwick + rgbd_odometry +
PX4 bridge) are NOT started here: imav_bringup (bringup.launch.py) owns both.
The camera mounting (cam_x ... cam_yaw) and the emitter are set there, in
imav_bringup's config/params.yaml under vio.extra_args.

Same mission and the same arguments as sequence_test.launch.py -- read that
file for what the `sequence` string means and for the flight parameters. The
only difference here is where x and y come from.

    ros2 launch drone_testing sequence_vio_realsense.launch.py
    ros2 run drone_testing offboard_sequence_vio --ros-args \
        -p takeoff_altitude:=1.0 \
        -p sequence:="forward 1.0, yaw 30, up 0.5, right 1.0"

As with the flow version, `agent_only` defaults to true: nothing is started
here and you run the flight node by hand in a second pane so stdin
stays a tty and the q/k aborts keep working. Your RC kill switch is the real
safety net either way.


HOW THIS DIFFERS FROM THE OLD ZED VERSION
=========================================
That flew a gen-1 ZED, which has no IMU: the wrapper hard-disables its
sensor stack, so what EKF2 receives is pure stereo VISUAL odometry and the
inertial half of "VIO" happens inside EKF2 on the Pixhawk's own IMU.

This file flies a D435i, which does have an IMU, and RTAB-Map fuses it
BEFORE the estimate ever reaches PX4:

    rs_launch.py (infra1 + depth + gyro 200 Hz / accel 63 Hz)
      -> imu_filter_madgwick        -> /rtabmap/imu     (orientation, gravity)
      -> rtabmap_odom/rgbd_odometry -> /rtabmap/odom    (frame_id=base_link)
      -> vio_to_px4_bridge.py       -> /fmu/in/vehicle_visual_odometry

That is a genuinely inertial estimate, and it is more robust through motion
blur and short featureless patches than the ZED path was. It is also SLOWER:
rgbd_odometry runs frame-to-map on the Orin at roughly 9-13 Hz, against the
ZED's 15-30. That is fine -- a stable rate matters far more than a high one,
because EKF2_EV_DELAY is tuned once and a drifting rate means a drifting
latency -- but it means the odometry is the slowest thing in the loop.

Three consequences worth knowing before you fly it:

  1. NO publish_rate ARGUMENT. The ZED version had to throttle to 15 Hz or
     the uXRCE-DDS UART starved the offboard heartbeat and PX4 took the
     aircraft. At 9-13 Hz there is nothing to throttle.

  2. NO /vio_healthy TOPIC. That topic is published by zed_localization;
     vio_to_px4_bridge.py does not publish it. offboard_sequence_vio handles
     this -- ALLOW_MISSING_BRIDGE_STATUS defaults true and it falls back to
     EKF2's own cs_ev_pos / cs_ev_fault flags, which are the authoritative
     answer to "is this being fused" anyway. You lose the bridge's freshness
     heartbeat, so hold_xy_from_ground latches on EKF2's word alone. Watch
     the bridge's own "tracking: in=N out=N resets=R" line instead; it prints
     every 5 s and `resets` must stop incrementing before you arm.

  3. STARTUP IS SLOW. initial_reset:=true power-cycles the USB device and
     wait_imu_to_init:=true holds rgbd_odometry until Madgwick has a gravity
     alignment. Budget 25-30 s, and KEEP THE AIRFRAME STILL for all of it --
     the gravity alignment is taken while it sits there. That is why the
     flight node here is on a 30 s timer rather than the ZED file's 10 s.


================================================================
PX4 PARAMETERS -- set these in QGC BEFORE the first flight
================================================================

Read this whole block: the numbers you actually type, and why.

--- 1. Height stays on the lidar -------------------------------------------
    EKF2_HGT_REF     = 2   (Range)
    EKF2_RNG_CTRL    = 1   (enabled)

Unchanged, and for the same reason: depth error scales with the square of
range, z is stereo's worst axis, and the 1D lidar is a direct, absolute,
drift-free measurement of the one axis that kills aircraft. The D435i having
an IMU does not change this -- an IMU bounds z over seconds, not minutes.

--- 2. Vision owns x/y ------------------------------------------------------
    bit 0 (1) = horizontal position   bit 2 (4) = velocity
    bit 1 (2) = VERTICAL position     bit 3 (8) = yaw

    EKF2_EV_CTRL = 1   (horizontal position only) -- and leave it there.

This airframe takes its HEADING FROM THE PX4 MAGNETOMETER, deliberately.
Vision supplies x and y only. That means bit 0 is the whole configuration:
no vertical position, no velocity, no yaw.

Leave bit 1 clear or you have handed height back to the camera and undone
item 1. Do not set bit 2 -- note that unlike zed_localization, THIS bridge
defaults publish_velocity to True, so real velocity data is being sent; with
bit 2 clear PX4 correctly ignores it. Turn it on only after position alone
has flown clean, and only deliberately.

    cs_yaw_align MUST be true before you arm. Check it, every time, on any
    new parameter set:

        ros2 topic echo /fmu/out/estimator_status_flags --once | grep cs_yaw_align

    Because the magnetometer is the only absolute heading source here, that
    flag is really a MAG HEALTH CHECK. EKF2 will not align yaw without it,
    and until it does the horizontal estimate is anchored to nothing: PX4
    invalidates the local position and takes the aircraft about a second
    after arming, while cs_ev_pos and xy_valid both sit there looking
    perfectly healthy. A compass sitting next to four ESCs indoors is the
    failure mode to watch. Keep EKF2_MAG_TYPE at its automatic setting, do a
    compass calibration on the actual airframe, and if cs_yaw_align is false
    fix the magnetometer -- do not reach for vision yaw.

    And in case anyone is tempted later: EKF2_EV_CTRL = 9 does NOT work with
    this bridge. vio_to_px4_bridge.py publishes POSE_FRAME_FRD
    unconditionally (correct -- the VIO has no magnetometer, so its yaw datum
    has no relation to North), and EKF2 will not align yaw from an FRD
    estimate: ev_yaw_control.cpp sets yaw_align = false on the FRD branch
    explicitly, and only the NED branch sets it true. You would get cs_ev_yaw
    true, cs_yaw_align false, and an aircraft PX4 takes away. The ZED path
    had pose_frame:=ned as an escape hatch; this one does not.

--- 3. Latency and noise ----------------------------------------------------
    EKF2_EV_DELAY    = MEASURE IT. Do not use the ZED file's 40 ms.

This is the single most important number here, and this pipeline is slower
than the ZED one. rgbd_odometry reports its own figure on every frame
("delay=0.158367s"), and observed values on this Orin ran 90-200 ms. With
the stack up, measure it properly:

        ros2 topic delay /rtabmap/odom

and use the mean, in milliseconds. A sane reading is 50-250 ms. If it comes
back huge, negative, or wildly unstable, STOP -- the RealSense warns at
startup that "frame's time domain is HARDWARE_CLOCK, timestamps may reset
periodically", and the bridge copies header.stamp straight into
timestamp_sample. A bad clock domain puts every measurement in the wrong
slot in EKF2's buffer and no amount of delay tuning will save it.

    EKF2_EVP_NOISE   = 0.1   m
    EKF2_EVV_NOISE   = 0.1   m/s
    EKF2_EV_NOISE_MD = 0     use the message covariance where it is sane.
                             rgbd_odometry reports ~3e-4 m when tracking well
                             and 9999 when lost; the bridge withholds the
                             whole message on 9999 rather than passing it on.

--- 4. Where the camera is on the airframe ---------------------------------
    EKF2_EV_POS_X / _Y / _Z  = 0    leave these at zero

Set the geometry with the cam_x / cam_y / cam_z / cam_roll / cam_pitch /
cam_yaw arguments of the VIO stack instead (vio.extra_args in imav_bringup's
config/params.yaml). They become a static
base_link -> camera_link TF, and because rgbd_odometry runs with
frame_id:=base_link, RTAB-Map applies the extrinsic itself and publishes the
pose of the VEHICLE. Setting it in both places double-counts it.

MEASURE FROM THE PIXHAWK, NOT FROM THE CG. EKF2 estimates the state at the
IMU, so base_link is the FCU origin. On this airframe that is the 21.5 cm
between the RealSense and the Pixhawk -- not the 9.5 cm from the CG, which
does not appear anywhere in this pipeline.

Why it matters: a lever arm is invisible during pure translation (it is a
constant offset) but turns every YAW into a phantom sideways step of
2*r*sin(theta/2). At r = 0.215 m that is 11 cm for a 30 degree yaw and 30 cm
for a 90 degree one -- and the default sequence below contains a `yaw 30`.

ROS convention: x forward, y LEFT, z UP, metres and radians, as the pose of
the camera in the body frame. cam_pitch POSITIVE = lens down.

--- 5. What to do with the ARK Flow ----------------------------------------
    EKF2_OF_CTRL     = 0   (disabled)  <-- recommended
    EKF2_RNG_CTRL    = 1   (keep! the lidar is still the height reference)

Do not fuse optical flow and vision simultaneously as your normal
configuration. EKF2 will accept both, and that is the problem: they are two
independent, differently-scaled, differently-delayed measurements of the same
lateral state, and where they disagree the filter splits the difference and
the vehicle drifts toward whichever one is lying. Flow's scale depends on the
rangefinder being right about height; vision's on the stereo baseline. You
cannot tell which failed.

Turning EKF2_OF_CTRL off does NOT turn off the ARK Flow's rangefinder -- that
is EKF2_RNG_CTRL, a separate parameter, and it stays on. You give up the
flow, not the lidar.

The D435i has one advantage over the ZED here that is worth knowing: its IR
projector puts texture into scenes that have none, so the "bare white arena
wall" case that argued for keeping flow is weaker than it was. It is kept
OFF for VIO, though (the VIO stack's `emitter` argument, default 0).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')

    # 30 s, not the ZED file's 10 s. Two things have to finish first: the
    # uXRCE-DDS session (PX4 creates its /fmu/out writers LAST, ~7 s after the
    # agent comes up) and the VIO stack (initial_reset power-cycles the USB
    # device, then wait_imu_to_init holds odometry until Madgwick has gravity).
    # Starting the flight node early means its first setpoints are dropped and
    # its first health check runs against an estimate that does not exist yet.
    sequence_node = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='drone_testing',
                executable='offboard_sequence_vio',
                name='offboard_sequence_vio',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'sequence': LaunchConfiguration('sequence'),
                    'direction_frame': LaunchConfiguration('direction_frame'),
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'step_hold_seconds': LaunchConfiguration('step_hold_seconds'),
                    'post_hold_seconds': LaunchConfiguration('post_hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'move_speed': LaunchConfiguration('move_speed'),
                    'yaw_rate': LaunchConfiguration('yaw_rate'),
                    'min_altitude': LaunchConfiguration('min_altitude'),
                    'max_altitude': LaunchConfiguration('max_altitude'),
                    'request_offboard_from_ros': LaunchConfiguration(
                        'request_offboard_from_ros'),
                    'hold_xy_from_ground': LaunchConfiguration('hold_xy_from_ground'),
                    'vio_settle_seconds': LaunchConfiguration('vio_settle_seconds'),
                    'allow_missing_bridge_status': LaunchConfiguration(
                        'allow_missing_bridge_status'),
                }],
            )
        ],
        condition=UnlessCondition(agent_only),
    )

    lcd_node = Node(
        package='drone_testing',
        executable='lcd_status',
        name='lcd_status',
        output='screen',
        emulate_tty=True,
        parameters=[{'port': LaunchConfiguration('lcd_port')}],
        condition=IfCondition(LaunchConfiguration('lcd')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_only', default_value='true',
            description='Start only the support stack (LCD); run the '
                        'flight node manually so q/k stay available.'),

        # ---- the mission, identical to sequence_test.launch.py ----
        DeclareLaunchArgument(
            'sequence', default_value='forward 1.0, yaw 30, up 0.5, right 1.0',
            description='Comma-separated motions: forward|backward|left|right <m>, '
                        'up|down <m>, yaw <deg, + = clockwise>.'),
        DeclareLaunchArgument(
            'direction_frame', default_value='home',
            description="home = forward always means the heading held at arming; "
                        "current = each move follows the yaw commanded at that point."),
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='1.0',
            description='Metres above the arming point. Start lower (0.3) on first flights.'),
        DeclareLaunchArgument(
            'hold_seconds', default_value='5.0',
            description='Station-keeping time at altitude before the first step.'),
        DeclareLaunchArgument(
            'step_hold_seconds', default_value='3.0',
            description="Station-keeping time between steps, so each one starts from "
                        "a settled vehicle instead of compounding the last step's "
                        'overshoot.'),
        DeclareLaunchArgument(
            'post_hold_seconds', default_value='5.0',
            description='Station-keeping time after the last step, before the descent.'),
        DeclareLaunchArgument(
            'ground_wait_seconds', default_value='5.0',
            description='Time spent armed on the ground before the climb starts. With '
                        'vision this is also when the x/y hold latches.'),
        DeclareLaunchArgument(
            'climb_speed', default_value='0.35',
            description='m/s the climb setpoint is ramped at.'),
        DeclareLaunchArgument(
            'land_speed', default_value='0.15',
            description='Keep MPC_LAND_SPEED at about 0.2 so PX4 agrees this is a '
                        'descent -- same caveat as the flow launch file.'),
        DeclareLaunchArgument(
            'move_speed', default_value='0.25',
            description='m/s. Lower than the ZED default on purpose: rgbd_odometry '
                        'runs at 9-13 Hz here, so a given speed is a larger pixel '
                        'displacement between frames. Raise it one flight at a time.'),
        DeclareLaunchArgument(
            'yaw_rate', default_value='0.35',
            description='rad/s. Keep this slow. Fast yaw is the most reliable way to '
                        'break any visual odometry, and with a 21.5 cm lever arm it '
                        'is also the motion most sensitive to a wrong cam_x.'),
        DeclareLaunchArgument(
            'min_altitude', default_value='0.4',
            description='m above the arming point that a down step may not go below.'),
        DeclareLaunchArgument(
            'max_altitude', default_value='3.0',
            description='m above the arming point that an up step may not exceed.'),
        DeclareLaunchArgument(
            'request_offboard_from_ros', default_value='false',
            description='false = you flip the Offboard switch on the TX yourself.'),

        # ---- vision ----
        DeclareLaunchArgument(
            'hold_xy_from_ground', default_value='true',
            description='Latch x/y position hold before the climb instead of after. '
                        'This is the main advantage vision has over flow.'),
        DeclareLaunchArgument(
            'vio_settle_seconds', default_value='2.0',
            description='Seconds the vision estimate must be continuously healthy '
                        'before x/y hold is latched onto it.'),
        DeclareLaunchArgument(
            'allow_missing_bridge_status', default_value='true',
            description='true = fall back to EKF2 cs_ev_* flags when /vio_healthy is '
                        'absent, which it always is here: that topic belongs to '
                        'zed_localization and vio_to_px4_bridge.py does not publish '
                        'it. Setting false makes the topic mandatory and WILL refuse '
                        'to arm on this stack.'),

        # ---- LCD ----
        DeclareLaunchArgument(
            'lcd', default_value='true',
            description='Start the Arduino LCD status node.'),
        DeclareLaunchArgument(
            'lcd_port', default_value='',
            description='Arduino serial port; empty = auto-detect ttyACM*/ttyUSB*.'),

        lcd_node,
        sequence_node,
    ])
