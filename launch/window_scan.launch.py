"""
Window scan launch: uXRCE-DDS agent + RealSense + window detection + the flight.

    arm -> climb to takeoff_altitude -> hold -> sweep the nose through a
    90 degree arc until the D435i sees the window -> lock the yaw and the
    position -> land 40 s after the climb started.

WHAT TO RUN

Bench test, no props, camera only -- this is how you check the detection
and the topic names before anything spins:

    ros2 launch drone_testing window_scan.launch.py flight:=false

    ros2 topic echo /window_detected
    ros2 run rqt_image_view rqt_image_view /window_detection/image

Flight. The default starts the agent, the camera and the detector but NOT the
flight node, so you can run that one by hand and keep the q/k keyboard
aborts (a node started by launch has no tty, so those keys are dead):

    ros2 launch drone_testing window_scan.launch.py
    ros2 run drone_testing window_scan --ros-args \
        -p takeoff_altitude:=1.0 -p flight_seconds:=40.0

Everything in one shot, no keyboard abort (your RC kill switch still
works, and it is the one that matters):

    ros2 launch drone_testing window_scan.launch.py agent_only:=false

PORTED FROM THE ZED TO THE REALSENSE D435i. If realsense2_camera is already
running from another launch file (the RTAB-Map stack, say), add camera:=false
so this one does not start a second copy -- librealsense will refuse the
device rather than share it, and the failure looks like a dead camera.

CHECK THE IMAGE TOPIC FIRST. The defaults are the realsense2_camera names
under the default /camera/camera namespace:

    ros2 topic list | grep camera

and if yours differ, pass image_topic:=/your/topic (and depth_topic:=...).
The depth one must be the ALIGNED variant -- see the argument's description
below for why that is not a preference.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')
    flight = LaunchConfiguration('flight')

    # The uXRCE-DDS agent.
    #
    # ExecuteProcess on the STANDALONE BINARY, not a Node on the
    # micro_ros_agent package. There is no micro_ros_agent binary package for
    # ROS 2 Jazzy (it existed for Humble), so `package='micro_ros_agent'`
    # fails at launch time with "package not found" on this Jetson -- and it
    # fails only on a flight launch, because flight:=false never starts the
    # agent. The agent built from source lives at /usr/local/bin/MicroXRCEAgent
    # and takes the same arguments.
    #
    # agent_cmd / agent_dev / agent_baud are launch arguments so a machine
    # that DOES have the ROS package, or a different port, needs no edit.
    microxrce_node = ExecuteProcess(
        cmd=[LaunchConfiguration('agent_cmd'), 'serial',
             '--dev', LaunchConfiguration('agent_dev'),
             '-b', LaunchConfiguration('agent_baud')],
        name='micro_xrce_dds_agent',
        output='screen',
    )

    # The camera driver. Skipped with camera:=false if you already have one up.
    #
    # align_depth.enable is required, not optional: without it the
    # aligned_depth_to_color topic does not exist and the detector reports a
    # window with no distances. enable_color is required because the detection
    # is an HSV threshold and the infra streams are monochrome. See the same
    # block in window_traverse.launch.py for the full reasoning.
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_namespace': LaunchConfiguration('camera_namespace'),
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'depth_module.emitter_enabled': '1',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'pointcloud.enable': 'false',
            'publish_tf': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('camera')),
    )

    # Detection. Delayed a little: librealsense takes a few seconds to open the
    # camera, and starting the detector into a topic that does not exist yet
    # just fills the log with "no frames" before the first frame arrives.
    detect_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_testing',
                executable='window_detect',
                name='window_detect',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'image_topic': LaunchConfiguration('image_topic'),
                    'depth_topic': LaunchConfiguration('depth_topic'),
                    'show_windows': LaunchConfiguration('show_windows'),
                    'publish_image': LaunchConfiguration('publish_image'),
                    'publish_mask': LaunchConfiguration('publish_mask'),
                    'color': LaunchConfiguration('color'),
                    'min_area': LaunchConfiguration('min_area'),
                    'stream_port': LaunchConfiguration('stream_port'),
                    'stream_scale': LaunchConfiguration('stream_scale'),
                    'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('detect')),
    )

    # Reboot the FC if EKF2 came up without the rangefinder. The ARK Flow's
    # DroneCAN node enumerates after PX4 boots, so EKF2 anchors on the baro and
    # the flight node then (correctly) refuses to arm. This exits immediately
    # and touches nothing when the rangefinder is already fused.
    reboot_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='drone_testing',
                executable='fc_reboot',
                name='fc_reboot',
                output='screen',
                emulate_tty=True,
            )
        ],
        condition=IfCondition(LaunchConfiguration('reboot_fc')),
    )

    # The flight. Held back until the DDS session is up and PX4's topics
    # exist, or the first setpoints are dropped. The delay also covers the
    # fc_reboot node above: if it does reboot the FC, the link has to come
    # back before this starts asking for Offboard.
    scan_node = TimerAction(
        period=LaunchConfiguration('flight_node_delay'),
        actions=[
            Node(
                package='drone_testing',
                executable='window_scan',
                name='window_scan',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'flight_seconds': LaunchConfiguration('flight_seconds'),
                    'scan_span_deg': LaunchConfiguration('scan_span_deg'),
                    'scan_yaw_rate': LaunchConfiguration('scan_yaw_rate'),
                    'scan_direction': LaunchConfiguration('scan_direction'),
                    'yaw_rate': LaunchConfiguration('yaw_rate'),
                    'detect_seconds': LaunchConfiguration('detect_seconds'),
                    'relock_on_loss': LaunchConfiguration('relock_on_loss'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'min_altitude': LaunchConfiguration('min_altitude'),
                    'max_altitude': LaunchConfiguration('max_altitude'),
                    'request_offboard_from_ros': LaunchConfiguration(
                        'request_offboard_from_ros'),
                }],
            )
        ],
        condition=UnlessCondition(agent_only),
    )

    # Status on the Arduino TFT. Started with the agent so the screen is alive
    # from boot; it also shows the window detection on row 4 by itself.
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
            description='Start the agent, the camera and the detector but not the '
                        'flight node, so you can run that by hand and keep the '
                        'q/k keyboard aborts. false = fly the whole thing.'),
        DeclareLaunchArgument(
            'flight', default_value='true',
            description='false = camera and detection only: no DDS agent, no '
                        'flight node. Use this for the bench test.'),
        DeclareLaunchArgument(
            'detect', default_value='true',
            description='Start the window_detect node.'),
        DeclareLaunchArgument(
            'agent_cmd', default_value='MicroXRCEAgent',
            description='The uXRCE-DDS agent binary. Default is the one built '
                        'from source at /usr/local/bin/MicroXRCEAgent -- there '
                        'is no micro_ros_agent ROS package for Jazzy. Check '
                        'with `which MicroXRCEAgent`.'),
        DeclareLaunchArgument(
            'agent_dev', default_value='/dev/ttyTHS1',
            description='Serial port the Pixhawk is on.'),
        DeclareLaunchArgument(
            'agent_baud', default_value='921600',
            description='Must match SER_TEL2_BAUD (or your port) on PX4.'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Start realsense2_camera. false if it is already '
                        'running.'),
        DeclareLaunchArgument(
            'zed', default_value='true',
            description='DEPRECATED alias, ignored. The camera is a RealSense '
                        'D435i now; use camera:=false.'),
        DeclareLaunchArgument(
            'camera_name', default_value='camera',
            description='realsense2_camera node name. With camera_namespace '
                        'this is what makes the topics /camera/camera/...'),
        DeclareLaunchArgument(
            'camera_namespace', default_value='camera',
            description='Namespace the driver publishes under.'),
        DeclareLaunchArgument(
            'color_profile', default_value='1280x720x30',
            description='D435i colour stream, WxHxFPS.'),
        DeclareLaunchArgument(
            'depth_profile', default_value='848x480x30',
            description='D435i depth stream, WxHxFPS. 848x480 is the depth '
                        'imager\'s native resolution.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/camera/color/image_raw',
            description='Colour image, rgb8 on the D435i. Check yours with '
                        '`ros2 topic list | grep camera`.'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='Depth registered to image_topic, 16UC1 in '
                        'MILLIMETRES on the RealSense. It MUST be the '
                        'aligned_depth_to_color topic: the D435i\'s depth '
                        'imager is a different lens in a different place, so '
                        'depth/image_rect_raw is not pixel-registered to the '
                        'colour frame and sampling a window corner out of it '
                        'reads the wrong part of the scene.'),
        DeclareLaunchArgument(
            'show_windows', default_value='false',
            description='cv2.imshow the frame and the mask. Needs a display; '
                        'leave false on a headless Jetson.'),
        DeclareLaunchArgument(
            'publish_image', default_value='true',
            description='Publish the annotated frame on /window_detection/image.'),
        DeclareLaunchArgument(
            'publish_mask', default_value='false',
            description='Also publish the HSV mask, for tuning the thresholds.'),
        DeclareLaunchArgument(
            'color', default_value='green',
            description='Which HSV range to look for: green, blue or red.'),
        DeclareLaunchArgument(
            'min_area', default_value='1500.0',
            description='px^2 the contour must exceed to count as a window.'),
        DeclareLaunchArgument(
            'stream_port', default_value='8080',
            description='Port for the browser MJPEG stream of the annotated '
                        'frame: http://<jetson-ip>:8080/. 0 disables it.'),
        DeclareLaunchArgument(
            'stream_scale', default_value='0.5',
            description='Downscale before JPEG encoding. 0.5 quarters the '
                        'bytes and a window is still easy to judge.'),
        DeclareLaunchArgument(
            'jpeg_quality', default_value='60',
            description='JPEG quality for the stream and the compressed topic.'),
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='1.0',
            description='Metres above the arming point. Start lower (0.5) on '
                        'the first flights.'),
        DeclareLaunchArgument(
            'flight_seconds', default_value='40.0',
            description='Seconds from the START OF THE CLIMB to the descent, '
                        'window found or not.'),
        DeclareLaunchArgument(
            'scan_span_deg', default_value='20.0',
            description='Total width of the yaw sweep, centred on the takeoff '
                        'heading: 20 = 10 deg either side. Kept narrow on '
                        'purpose -- a wide sweep swings the airframe far off '
                        'the takeoff heading and smears the optical flow the '
                        'position hold stands on.'),
        DeclareLaunchArgument(
            'scan_yaw_rate', default_value='0.05',
            description='rad/s the yaw setpoint is walked at DURING THE SWEEP '
                        'only (~3 deg/s), slower than yaw_rate. Restored to '
                        'yaw_rate once the window is locked.'),
        DeclareLaunchArgument(
            'scan_direction', default_value='right',
            description="Which way the first half-leg turns, 'right' or "
                        "'left'. Point it at the side the window is expected "
                        'on.'),
        DeclareLaunchArgument(
            'yaw_rate', default_value='0.35',
            description='rad/s the yaw setpoint is walked at (~20 deg/s). Keep '
                        'it slow: a fast yaw smears the optical flow the '
                        'position hold depends on, and blurs the camera.'),
        DeclareLaunchArgument(
            'detect_seconds', default_value='0.4',
            description='How long /window_detected must stay true before the '
                        'sweep stops. On top of the detector own debounce.'),
        DeclareLaunchArgument(
            'relock_on_loss', default_value='false',
            description='true = go back to sweeping if the window is lost '
                        'after the lock. false = stay put.'),
        DeclareLaunchArgument(
            'hold_seconds', default_value='5.0',
            description='Station keeping at altitude before the sweep starts. '
                        'The optical flow x/y latch has to happen in here.'),
        DeclareLaunchArgument(
            'ground_wait_seconds', default_value='5.0',
            description='Time armed on the ground before the climb.'),
        DeclareLaunchArgument(
            'climb_speed', default_value='0.35',
            description='m/s the climb setpoint is ramped at.'),
        DeclareLaunchArgument(
            'land_speed', default_value='0.15',
            description='m/s the descent setpoint is ramped at. PX4 only counts '
                        'it as a descent if it is at least 0.9 * MPC_LAND_SPEED, '
                        'so set MPC_LAND_SPEED to about 0.2 to match.'),
        DeclareLaunchArgument(
            'min_altitude', default_value='0.4',
            description='m above the arming point the flight may not go below.'),
        DeclareLaunchArgument(
            'max_altitude', default_value='3.0',
            description='m above the arming point the flight may not exceed.'),
        DeclareLaunchArgument(
            'request_offboard_from_ros', default_value='true',
            description='false = you flip the Offboard switch on the TX.'),
        DeclareLaunchArgument(
            'reboot_fc', default_value='false',
            description='Run fc_reboot first: reboots the flight controller '
                        'over the DDS link IF EKF2 is not fusing the '
                        'rangefinder, so the ARK Flow is on the bus before '
                        'EKF2 picks its height source. Does nothing when the '
                        'rangefinder is already fused.'),
        DeclareLaunchArgument(
            'flight_node_delay', default_value='8.0',
            description='s before the flight node starts. Raise it to about '
                        '75 when reboot_fc is true, so a reboot has time to '
                        'complete and the DDS session to come back.'),
        DeclareLaunchArgument(
            'lcd', default_value='true',
            description='Start the Arduino TFT status node.'),
        DeclareLaunchArgument(
            'lcd_port', default_value='',
            description='Arduino serial port; empty = auto-detect ttyACM*/ttyUSB*.'),

        # flight:=false leaves just the camera side running, which is what you
        # want when you are tuning HSV thresholds indoors. The PX4-facing
        # actions are grouped rather than given a condition directly, because
        # two of them already carry one of their own and an action's condition
        # is fixed when it is built.
        GroupAction([microxrce_node, lcd_node, reboot_node, scan_node],
                    condition=IfCondition(flight)),
        camera_launch,
        detect_node,
    ])
