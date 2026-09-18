"""
Sequence test flown on ZED visual odometry.

    uXRCE-DDS agent + zed_wrapper + zed_localization bridge + offboard_sequence_vio

Same mission and the same arguments as sequence_test.launch.py -- read that
file for what the `sequence` string means and for the flight parameters. The
only difference here is where x and y come from.

    ros2 launch drone_testing sequence_vio_test.launch.py
    ros2 run drone_testing offboard_sequence_vio --ros-args \
        -p takeoff_altitude:=1.0 \
        -p sequence:="forward 1.0, yaw 30, up 0.5, right 1.0"

As with the flow version, `agent_only` defaults to true: launch the support
stack here and run the flight node by hand in a second pane so stdin stays a
tty and the q/k aborts keep working. Your RC kill switch is the real safety
net either way.


================================================================
PX4 PARAMETERS -- set these in QGC BEFORE the first flight
================================================================

Read this whole block. Getting EKF2_EV_CTRL right and EKF2_HGT_REF wrong is
how you end up with perfect lateral hold and no idea how high you are.

THE CAMERA HAS NO IMU. `general.camera_model: 'zed'` is the gen-1 ZED; the
wrapper hard-disables its entire sensors stack, so `imu_fusion: true` in the
ZED config does nothing and what you are fusing is stereo VISUAL odometry.
The inertial half of "VIO" is the Pixhawk's own IMU, inside EKF2. That is a
perfectly good architecture -- it is what EKF2 is for -- but it means the
vision estimate arrives at 15-30 Hz with no inertial smoothing of its own and
WILL drop out on motion blur and on featureless walls. Plan for the dropout.

--- 1. Height stays on the lidar -------------------------------------------
    EKF2_HGT_REF     = 2   (Range)
    EKF2_RNG_CTRL    = 1   (enabled)
    EKF2_EV_CTRL     : do NOT set the vertical-position bit (see below)

The 1D lidar is a direct, absolute, drift-free measurement of the one axis
that kills aircraft. Stereo VO's z is its worst axis -- depth error scales
with the square of range and there is nothing to anchor it. Do not hand
height to the camera. This also keeps the inherited flight logic honest: it
gates arming and flight on rangefinder fusion, and treats a dead height
estimate as an emergency and dead vision as a skipped step.

--- 2. Vision owns x/y ------------------------------------------------------
Be careful with this bitmask, it is the one people get wrong:
    bit 0 (1)  = horizontal position
    bit 1 (2)  = vertical position
    bit 2 (4)  = velocity
    bit 3 (8)  = yaw

    EKF2_EV_CTRL = 1   <-- start here (horizontal position only)

Note that bit 1 is VERTICAL POSITION: leave it clear, or you have just given
height back to the camera and undone item 1 above.
That is the whole recommendation for a first flight: position only, no
velocity, no yaw, no height. It is the smallest change that gets you lateral
hold, and every additional bit is another way for the estimate to fight
itself.

BUT position-only assumes SOMETHING ELSE IS SUPPLYING THE HEADING, and on this
airframe that can only be the magnetometer. EKF2 will not align yaw without an
absolute heading source, and until it does (cs_yaw_align) the horizontal
estimate is not anchored to anything, so PX4 invalidates the local position and
takes the aircraft roughly a second after arming -- while cs_ev_pos and
xy_valid both sit there looking perfectly healthy. If you have turned the
magnetometer off for indoor flight, EKF2_EV_CTRL = 1 is NOT a valid
configuration; you must enable vision yaw as well (see below). Check with

    ros2 topic echo /fmu/out/estimator_status_flags --once | grep cs_yaw_align

before every first flight on a new parameter set. README section 10.3.

    EKF2_EV_CTRL = 9 (position + yaw) only once position alone has flown
    clean, and only if the magnetometer is unusable indoors. Vision yaw and
    the magnetometer are two absolute heading sources; enabling both means
    EKF2 arbitrating between a compass sitting next to four ESCs and a VO
    heading that drifts. Pick one. If you enable vision yaw, set
    EKF2_MAG_TYPE = 5 (None) AND pass pose_frame:=ned at the same time.

    That last part is not optional and it is not obvious. EKF2 will not align
    yaw from a POSE_FRAME_FRD estimate no matter how well vision yaw is
    fusing -- ev_yaw_control.cpp sets yaw_align = false on the FRD branch
    explicitly, and only the NED branch sets it true. EKF2_EV_CTRL = 9 with
    the default pose_frame:=frd gets you cs_ev_yaw true, cs_yaw_align false,
    and an aircraft PX4 takes away a second after arming. See the frame table
    in drone_testing/zed_localization.py.

    Do NOT set bit 2 (velocity) unless you have also set publish_velocity:=true
    on the bridge -- and you probably should not. The bridge leaves velocity
    as NaN by default and PX4 correctly reads that as "not provided".

--- 3. Latency and noise ----------------------------------------------------
    EKF2_EV_DELAY    = 40    ms, starting point. This is the single most
                             important number here. Too small and the fusion
                             oscillates; too large and it lags. Tune it by
                             flying a slow lateral translation and comparing
                             the EV innovation in the log. On a Jetson at
                             HD720/30 with the wrapper in-process, 30-60 ms is
                             the realistic range.
    EKF2_EVP_NOISE   = 0.1   m. The bridge already floors the reported
    EKF2_EVV_NOISE   = 0.1   m/s. covariance (min_position_variance etc), so
                             these are the backstop, not the only defence.
    EKF2_EV_NOISE_MD = 0     use the noise from the message where it is sane.

--- 4. Where the camera is on the airframe ---------------------------------
    EKF2_EV_POS_X / _Y / _Z  = 0    leave these at zero

Set the mounting geometry with the cam_x / cam_y / cam_z and
cam_roll / cam_pitch / cam_yaw arguments of THIS launch file instead. They go
to the bridge node, which applies the full rigid transform before publishing.

The reason it has to be done there and not in EKF2: the wrapper publishes the
pose of the CAMERA, not of the vehicle, and its child frame is hardcoded to
`<camera_name>_camera_link` (zed_camera_component_main.cpp:1655). No parameter
makes it report base_link, and a static base_link -> zed_camera_link TF does
not change it -- the wrapper never consults TF for this. Meanwhile EKF2 has
parameters for the lever arm but NONE for the camera's rotation, so a camera
that is pitched down cannot be corrected on the PX4 side at all.

Measure it in the ROS convention -- x forward, y LEFT, z UP, radians -- as
the pose of the camera in the body frame. Getting the rotation wrong tilts
every commanded translation; getting the lever arm wrong turns each yaw into
a phantom sideways step.

--- 5. What to do with the ARK Flow ----------------------------------------
    EKF2_OF_CTRL     = 0   (disabled)  <-- recommended
    EKF2_RNG_CTRL    = 1   (keep! the lidar is still the height reference)

This is the question you asked, so here is the direct answer: DO NOT fuse
optical flow and vision simultaneously as your normal configuration. EKF2
will accept both, and that is exactly the problem -- they are two
independent, differently-scaled, differently-delayed measurements of the same
lateral state, and where they disagree the filter splits the difference and
the vehicle drifts toward whichever one is lying. Flow's scale depends on the
rangefinder being right about height; vision's depends on the stereo
baseline. When they disagree you cannot tell which failed, and you have
built a system with two single points of failure instead of a redundant one.

Note that turning EKF2_OF_CTRL off does NOT turn off the ARK Flow's
rangefinder -- that is EKF2_RNG_CTRL, a separate parameter, and it stays on.
You are giving up the flow, not the lidar.

The exception, and it is a real one: keep flow enabled if you are flying
somewhere the ZED will be blind -- a bare white arena floor with the camera
pitched down, low light, a wall filling the frame during a yaw. Flow over a
textured floor works where stereo VO does not. If you do run both, fly the
vehicle slowly and expect the two to argue during fast translations.

Recommended progression, and it is worth doing all three:
    1. Flow only, EKF2_EV_CTRL = 0.        Today's configuration. Fly the
                                           sequence, keep the log.
    2. Vision only, EKF2_OF_CTRL = 0,      Fly the identical sequence with
       EKF2_EV_CTRL = 1.                   this launch file. Compare.
    3. Both, only if step 2 showed a       Nothing else changed.
       dropout you actually need covered.

You cannot pick between them from first principles -- it depends on your
arena's floor and walls. Two flights and two logs will tell you in an hour.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')

    microxrce_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_xrce_dds_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
    )

    # The ZED wrapper's own launch file: it does the camera-model config
    # loading and the camera_link TF tree, which are not worth reproducing.
    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'camera_name': LaunchConfiguration('camera_name'),
            # The ZED must not own the odom -> base_link TF: PX4 is the
            # navigation authority on this vehicle and the two would publish
            # the same edge with different answers.
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('zed')),
    )

    # The bridge. Started a little after the camera so its first health report
    # is about a camera that has had a chance to open.
    bridge_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_testing',
                executable='zed_localization',
                name='zed_localization',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'pose_frame': LaunchConfiguration('pose_frame'),
                    'publish_velocity': LaunchConfiguration('publish_velocity'),
                    'publish_rate': LaunchConfiguration('publish_rate'),
                    # Pose of the camera in the body frame, ROS convention.
                    # The wrapper reports the CAMERA's pose, so this is what
                    # turns it into the vehicle's -- see the header.
                    'cam_x': LaunchConfiguration('cam_x'),
                    'cam_y': LaunchConfiguration('cam_y'),
                    'cam_z': LaunchConfiguration('cam_z'),
                    'cam_roll': LaunchConfiguration('cam_roll'),
                    'cam_pitch': LaunchConfiguration('cam_pitch'),
                    'cam_yaw': LaunchConfiguration('cam_yaw'),
                }],
                # vio_healthy / vio_status are node-relative; the flight node
                # subscribes to them absolutely, so pin them to the root.
                remappings=[('vio_healthy', '/vio_healthy'),
                            ('vio_status', '/vio_status')],
            )
        ],
    )

    # Same 10 s delay as the flow version: the DDS session has to be up and
    # the PX4 topics have to exist before the first setpoint, or it is dropped.
    sequence_node = TimerAction(
        period=10.0,
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
            description='Start only the support stack (agent, camera, bridge, LCD); '
                        'run the flight node manually so q/k stay available.'),

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
            description='Station-keeping time between steps, so each one starts from '
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
            'move_speed', default_value='0.30',
            description='m/s. Can go higher than the flow version once vision is '
                        'trusted, but raise it one flight at a time: fast '
                        'translation is what breaks stereo VO.'),
        DeclareLaunchArgument(
            'yaw_rate', default_value='0.35',
            description='rad/s. Keep this slow. A fast yaw is the single most '
                        'reliable way to make an IMU-less stereo camera lose '
                        'tracking, and it is worse here than it was with flow.'),
        DeclareLaunchArgument(
            'min_altitude', default_value='0.4',
            description='m above the arming point that a down step may not go below.'),
        DeclareLaunchArgument(
            'max_altitude', default_value='3.0',
            description='m above the arming point that an up step may not exceed.'),
        DeclareLaunchArgument(
            'request_offboard_from_ros', default_value='true',
            description='false = you flip the Offboard switch on the TX yourself.'),

        # ---- vision ----
        DeclareLaunchArgument(
            'zed', default_value='true',
            description='Start the ZED wrapper here. false if you already run it '
                        'from another launch file.'),
        DeclareLaunchArgument(
            'camera_model', default_value='zed',
            description='ZED SDK model name. Leave at zed for the gen-1 camera; note '
                        'that model has no IMU, so this is visual odometry only.'),
        DeclareLaunchArgument(
            'camera_name', default_value='zed',
            description='Sets the topic prefix and the odom child frame '
                        '(<camera_name>_camera_link). Change odom_topic to match.'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/zed/zed_node/odom',
            description='ZED odometry the bridge converts into PX4 external vision.'),
        DeclareLaunchArgument(
            'pose_frame', default_value='frd',
            description="frd = the vision heading has an unknown constant offset "
                        "from North and EKF2 estimates it (correct for this camera: "
                        "it cannot see North). ned = only if the vision frame is "
                        "genuinely North-aligned."),
        DeclareLaunchArgument(
            'publish_rate', default_value='15.0',
            description='Hz sent to PX4, independently of the ZED frame rate. The '
                        'uXRCE-DDS UART cannot carry 30 Hz of odometry alongside the '
                        'setpoint streams -- it starves the offboard heartbeat and PX4 '
                        'takes the aircraft. 0 = no throttle, only safe on Ethernet.'),
        DeclareLaunchArgument(
            'publish_velocity', default_value='false',
            description='Send the ZED twist as a body-FRD velocity. Leave false '
                        'unless EKF2_EV_CTRL bit 2 is also set.'),
        DeclareLaunchArgument(
            'hold_xy_from_ground', default_value='true',
            description='Latch x/y position hold before the climb instead of after. '
                        'This is the main advantage vision has over flow.'),
        DeclareLaunchArgument(
            'vio_settle_seconds', default_value='2.0',
            description='Seconds the vision estimate must be continuously healthy '
                        'before x/y hold is latched onto it.'),

        # ---- camera mounting: pose of the camera IN THE BODY FRAME, ROS
        # convention (x fwd, y LEFT, z UP), metres and radians. MEASURE THESE
        # on the actual airframe -- the defaults are "camera at the CoG,
        # pointing straight forward", which is a no-op and is almost certainly
        # not where yours is. See item 4 in the header.
        DeclareLaunchArgument(
            'cam_x', default_value='0.0',
            description='Metres the camera sits FORWARD of the vehicle CoG.'),
        DeclareLaunchArgument(
            'cam_y', default_value='0.0',
            description='Metres the camera sits to the LEFT of the CoG (ROS sign).'),
        DeclareLaunchArgument(
            'cam_z', default_value='0.0',
            description='Metres the camera sits ABOVE the CoG.'),
        DeclareLaunchArgument(
            'cam_roll', default_value='0.0',
            description='Radians, positive = right side down.'),
        DeclareLaunchArgument(
            'cam_pitch', default_value='0.0',
            description='Radians, POSITIVE = nose down. A camera angled down '
                        '20 deg for window detection is cam_pitch:=0.35.'),
        DeclareLaunchArgument(
            'cam_yaw', default_value='0.0',
            description='Radians, positive = camera pointed to the LEFT of straight '
                        'ahead. A sideways-mounted camera MUST have this set.'),

        # ---- LCD ----
        DeclareLaunchArgument(
            'lcd', default_value='true',
            description='Start the Arduino LCD status node.'),
        DeclareLaunchArgument(
            'lcd_port', default_value='',
            description='Arduino serial port; empty = auto-detect ttyACM*/ttyUSB*.'),

        microxrce_node,
        zed_wrapper,
        lcd_node,
        bridge_node,
        sequence_node,
    ])
