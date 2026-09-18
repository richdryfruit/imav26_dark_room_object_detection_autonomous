"""
Take off, find the ArUco ring board, fly through the ring, come back, land.

    # 1. TROUBLESHOOTING FIRST. No agent, no flight node, camera only.
    ros2 launch drone_testing ring_grab.launch.py flight:=false
    ros2 run drone_testing ring_grab --ros-args -p mode:=pose

    # 2. THE FLIGHT. agent + camera + detector from launch, flight by hand.
    ros2 launch drone_testing ring_grab.launch.py
    ros2 run drone_testing ring_grab --ros-args -p mode:=grab

    # 3. Everything from launch, including the flight node.
    ros2 launch drone_testing ring_grab.launch.py agent_only:=false mode:=grab

agent_only defaults to true, the same as every other launch file here, so the
support stack comes up from launch and the flight node is run by hand in a
second pane. That keeps stdin a tty, which is the only way the q / k aborts
work. Your RC kill switch is the real safety net regardless.

THE TWO MODES
-------------
mode:=pose is the default and it is the troubleshooting one. It arms nothing,
commands nothing and never talks to the flight controller; it prints where the
drone is in the BOARD's frame -- depth, right, up and yaw -- with the standard
deviation of each over a two-second window, and publishes the same on
/ring/board_pose, /ring/board_yaw and /ring/board_pose_info. Do this on a
bench, with a tape measure, before anything flies. See ring_grab.py.

mode:=grab flies:

    arm -> climb -> hold -> sweep +/-45 deg about the takeoff heading looking
    for the board -> lock -> COURSE (line up at a standoff, without ever
    flying backwards) -> FINE (proportional pose setpoints until the error is
    inside align_tolerance) -> GRAB (open loop, 20 cm past the grab marker)
    -> BACKOUT (straight back to where FINE finished) -> land.

WHAT THIS STARTS
----------------
    uXRCE-DDS agent     as a bare process, not the micro_ros_agent node --
                        same as window_traverse.launch.py, because the ROS
                        package is not installed on this Jetson.
    realsense2_camera   COLOUR ONLY. See below.
    ring_detect         the ArUco pipeline. Browser view on port 8080.
    ring_grab           the flight, unless agent_only:=true (the default).

COLOUR ONLY, AND WHY THERE IS NO DEPTH HERE
--------------------------------------------
window_traverse needs align_depth because an HSV blob has no scale. A marker
has one: the edge length is known, so four corners plus the camera matrix are
a complete range measurement and solvePnP returns metres directly. So depth,
the aligned-depth topic, the pointcloud, the infra streams and the IMU are all
off. On a USB3 bus shared with the flight controller that is bandwidth spent
on nobody, and the IR emitter is left off for the same reason -- it does
nothing for a passive colour detection.

If you turn depth on for some other reason, note that align_depth is still not
needed by anything in this mission.

THE NUMBERS THAT MUST BE RIGHT BEFORE THIS FLIES
-------------------------------------------------
marker_size     The edge of the black square, in metres, measured with a tape
                on the printed board -- not the number on the PDF. It is the
                metric scale of the entire pose. 10 % out here is 10 % out on
                the standoff AND on the 20 cm grab depth, in the same
                direction.
marker_gap      Blank space between two markers, so marker_size + marker_gap
                is the centre-to-centre pitch. Wrong pitch puts the grab
                target on the wrong marker whenever the grab marker itself is
                out of frame -- which is most of the approach.
marker_ids      Bottom to top. The default 4/5/6 is the sim board's.
grab_marker_id  The marker the ring hangs on. Default 4, the bottom one.
aruco_dict      Must match the board. The sim textures were DICT_4X4_50.
cam_x/y/z       Camera position in the body frame, ROS convention (x forward,
cam_roll/       y LEFT, z UP), and its mounting rotation. THE SAME SIX NUMBERS
  pitch/yaw     window_traverse.launch.py takes. If the D435i is on the same
                mount, copy them across; if it is not, measure them.
grab_behind     0.20 m. Positive is further THROUGH the ring.

PX4 SIDE
--------
Nothing here changes the PX4 configuration, and nothing here feeds the EKF.
This flies on the same PMW3901 + TFmini Plus + EKF2 stack as the window
mission -- no VIO bridge, no external vision, no EV_CTRL. The board produces
setpoints and nothing else. The usual pre-flight applies: reboot the flight
controller first, or cs_rng_kin_consistent will refuse the arm.
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

    # Started as a bare process rather than a micro_ros_agent Node: the ROS
    # package is not installed on this Jetson, the agent built from source
    # lives at /usr/local/bin/MicroXRCEAgent, and it takes the same arguments.
    microxrce_node = ExecuteProcess(
        cmd=[LaunchConfiguration('agent_cmd'), 'serial',
             '--dev', LaunchConfiguration('agent_dev'),
             '-b', LaunchConfiguration('agent_baud')],
        name='micro_xrce_dds_agent',
        output='screen',
    )

    # Scoped, non-forwarding, for the same reason window_traverse.launch.py
    # does it: rs_launch.py warns about every launch configuration it can see
    # that is not one of its own, and IncludeLaunchDescription forwards the
    # parent's by default, which buries the startup log. forwarding=False
    # means anything the include needs has to be mapped in here explicitly.
    realsense = GroupAction(
        scoped=True,
        forwarding=False,
        launch_configurations={
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_namespace': LaunchConfiguration('camera_namespace'),
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
        },
        condition=IfCondition(LaunchConfiguration('camera')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch', 'rs_launch.py'])),
                launch_arguments={
                    'enable_color': 'true',
                    # Everything else off. solvePnP needs one calibrated
                    # colour image and nothing more; see the header.
                    'enable_depth': 'false',
                    'align_depth.enable': 'false',
                    'enable_infra1': 'false',
                    'enable_infra2': 'false',
                    'enable_gyro': 'false',
                    'enable_accel': 'false',
                    'pointcloud.enable': 'false',
                    # PX4 is the navigation authority on this vehicle; the
                    # camera must not publish a competing odom -> base_link.
                    'publish_tf': 'false',
                }.items(),
            ),
        ],
    )

    # The detector. Ahead of the flight node so the camera has opened and
    # /ring_detected is already being published by the time the sweep asks it
    # a question. 6 s, not 3: librealsense takes the better part of ten
    # seconds to reach "RealSense Node Is Up!" on this Jetson, and the node
    # simply waits on the topic until then.
    detect_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='drone_testing',
                executable='ring_detect',
                name='ring_detect',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'image_topic': LaunchConfiguration('image_topic'),
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'marker_ids': LaunchConfiguration('marker_ids'),
                    'marker_size': LaunchConfiguration('marker_size'),
                    'marker_gap': LaunchConfiguration('marker_gap'),
                    'grab_marker_id': LaunchConfiguration('grab_marker_id'),
                    'aruco_dict': LaunchConfiguration('aruco_dict'),
                    'max_fps': LaunchConfiguration('max_fps'),
                    'depth_min': LaunchConfiguration('depth_min'),
                    'depth_max': LaunchConfiguration('depth_max'),
                    'max_reproj_px': LaunchConfiguration('max_reproj_px'),
                    'min_markers': LaunchConfiguration('min_markers'),
                    'detect_frames': LaunchConfiguration('detect_frames'),
                    'lost_frames': LaunchConfiguration('lost_frames'),
                    'hfov_deg': LaunchConfiguration('hfov_deg'),
                    'image_rotate': LaunchConfiguration('image_rotate'),
                    'stream_port': LaunchConfiguration('stream_port'),
                    'stream_scale': LaunchConfiguration('stream_scale'),
                    'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('detect')),
    )

    # The flight. Held back until the DDS session is up and PX4's topics
    # exist, or the first setpoints are dropped on the floor.
    grab_node = TimerAction(
        period=LaunchConfiguration('flight_node_delay'),
        actions=[
            Node(
                package='drone_testing',
                executable='ring_grab',
                name='ring_grab',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'mode': LaunchConfiguration('mode'),

                    # ---- the flight ----
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'min_altitude': LaunchConfiguration('min_altitude'),
                    'max_altitude': LaunchConfiguration('max_altitude'),
                    'flight_seconds': LaunchConfiguration('flight_seconds'),
                    'request_offboard_from_ros': LaunchConfiguration(
                        'request_offboard_from_ros'),

                    # ---- the search ----
                    'scan_span_deg': LaunchConfiguration('scan_span_deg'),
                    'scan_yaw_rate': LaunchConfiguration('scan_yaw_rate'),
                    'scan_direction': LaunchConfiguration('scan_direction'),
                    'detect_seconds': LaunchConfiguration('detect_seconds'),
                    'detect_topic': LaunchConfiguration('detect_topic'),
                    'yaw_cone_deg': LaunchConfiguration('yaw_cone_deg'),

                    # ---- the approach ----
                    'fine_standoff': LaunchConfiguration('fine_standoff'),
                    'min_standoff': LaunchConfiguration('min_standoff'),
                    'grab_behind': LaunchConfiguration('grab_behind'),
                    'backout_extra': LaunchConfiguration('backout_extra'),
                    'approach_speed': LaunchConfiguration('approach_speed'),
                    'grab_speed': LaunchConfiguration('grab_speed'),
                    'backout_speed': LaunchConfiguration('backout_speed'),
                    'coarse_tolerance': LaunchConfiguration('coarse_tolerance'),
                    'align_gain': LaunchConfiguration('align_gain'),
                    'align_tolerance': LaunchConfiguration('align_tolerance'),
                    'align_settle_seconds': LaunchConfiguration('align_settle_seconds'),
                    'yaw_tolerance_deg': LaunchConfiguration('yaw_tolerance_deg'),
                    'grab_tolerance': LaunchConfiguration('grab_tolerance'),
                    'grab_hold_seconds': LaunchConfiguration('grab_hold_seconds'),

                    # ---- the estimate ----
                    'buffer_seconds': LaunchConfiguration('buffer_seconds'),
                    'pose_min_samples': LaunchConfiguration('pose_min_samples'),
                    'pose_max_age': LaunchConfiguration('pose_max_age'),
                    'pose_lost_timeout': LaunchConfiguration('pose_lost_timeout'),
                    'gate_metres': LaunchConfiguration('gate_metres'),
                    'gate_yaw_deg': LaunchConfiguration('gate_yaw_deg'),
                    'max_tilt_deg': LaunchConfiguration('max_tilt_deg'),

                    # ---- the camera mounting ----
                    'cam_x': LaunchConfiguration('cam_x'),
                    'cam_y': LaunchConfiguration('cam_y'),
                    'cam_z': LaunchConfiguration('cam_z'),
                    'cam_roll': LaunchConfiguration('cam_roll'),
                    'cam_pitch': LaunchConfiguration('cam_pitch'),
                    'cam_yaw': LaunchConfiguration('cam_yaw'),
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
            description='Start the agent, the camera and the detector but not '
                        'the flight node, so you can run that by hand and keep '
                        'the q/k keyboard aborts. false = fly the whole thing.'),
        DeclareLaunchArgument(
            'flight', default_value='true',
            description='false = camera and detector only: no DDS agent, no '
                        'flight node. This is the bench setup.'),
        DeclareLaunchArgument(
            'detect', default_value='true',
            description='Start ring_detect.'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Start realsense2_camera. false if it is already up -- '
                        'librealsense will refuse a second claim on the device.'),

        # ---- mode ----
        DeclareLaunchArgument(
            'mode', default_value='pose',
            description='pose = report the drone pose in the board frame and '
                        'command NOTHING (the troubleshooting mode; run this '
                        'first). grab = fly the mission.'),

        # ---- the flight ----
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='1.50',
            description='m above the arming point. Should put the camera near '
                        'the height of the grab marker, so the board is '
                        'centred in frame during the sweep.'),
        DeclareLaunchArgument(
            'hold_seconds', default_value='5.0',
            description='Station keeping at altitude before the sweep. The '
                        'optical flow x/y latch has to happen in here.'),
        DeclareLaunchArgument('ground_wait_seconds', default_value='5.0'),
        DeclareLaunchArgument('climb_speed', default_value='0.50'),
        DeclareLaunchArgument('land_speed', default_value='0.15'),
        DeclareLaunchArgument(
            'min_altitude', default_value='0.40',
            description='m above the arming point the flight may not go below. '
                        'Also clamps the grab marker altitude.'),
        DeclareLaunchArgument('max_altitude', default_value='3.00'),
        DeclareLaunchArgument(
            'flight_seconds', default_value='180.0',
            description='s from the START OF THE CLIMB to a forced descent, '
                        'whatever else is happening. The backstop that '
                        'outranks every stage.'),
        DeclareLaunchArgument(
            'request_offboard_from_ros', default_value='true',
            description='false = you flip the Offboard switch on the TX.'),

        # ---- the search ----
        DeclareLaunchArgument(
            'scan_span_deg', default_value='90.0',
            description='Total sweep arc, centred on the takeoff heading, so '
                        '90 is the +/-45 deg the mission calls for. 0 disables '
                        'the sweep and stares straight ahead.'),
        DeclareLaunchArgument(
            'scan_yaw_rate', default_value='0.05',
            description='rad/s while sweeping (~3 deg/s). Slow on purpose: the '
                        'detector needs several consecutive frames on the '
                        'board, and a fast yaw sweeps past markers it '
                        'technically saw. Restored to the cruise rate on lock.'),
        DeclareLaunchArgument(
            'scan_direction', default_value='right',
            description='Which half of the arc is swept first. Point it at the '
                        'side the board is expected on.'),
        DeclareLaunchArgument(
            'detect_seconds', default_value='0.4',
            description='s /ring_detected must stay true before the lock, on '
                        'top of ring_detect\'s own frame debounce.'),
        DeclareLaunchArgument('detect_topic', default_value='ring_detected'),
        DeclareLaunchArgument(
            'yaw_cone_deg', default_value='70.0',
            description='Hard limit on how far the nose may turn from the '
                        'takeoff heading in ANY stage. Wider than the sweep so '
                        'a board found at the edge of the arc can still be '
                        'squared up to.'),

        # ---- the approach ----
        DeclareLaunchArgument(
            'fine_standoff', default_value='1.00',
            description='m in front of the board that the fine alignment is '
                        'flown at. The course never flies FURTHER out than '
                        'this -- if the sweep ended closer, it aligns where it '
                        'already is. See the node header.'),
        DeclareLaunchArgument(
            'min_standoff', default_value='0.35',
            description='m. Closer than this there is not enough board left in '
                        'frame to align on, so the course does back out to '
                        'here. The one case it flies backwards.'),
        DeclareLaunchArgument(
            'grab_behind', default_value='0.20',
            description='m PAST the grab marker, along the board normal. This '
                        'is the point the mission is about. Positive is '
                        'further through the ring.'),
        DeclareLaunchArgument(
            'backout_extra', default_value='0.0',
            description='m beyond the fine-alignment point to retreat to. '
                        '0 = come back exactly where the alignment finished.'),
        DeclareLaunchArgument('approach_speed', default_value='0.30'),
        DeclareLaunchArgument(
            'grab_speed', default_value='0.35',
            description='m/s through the ring. Committed and open loop, so '
                        'brisk enough not to linger but slow enough that a '
                        'frozen target 10 cm out is a graze, not an impact.'),
        DeclareLaunchArgument('backout_speed', default_value='0.30'),
        DeclareLaunchArgument(
            'coarse_tolerance', default_value='0.25',
            description='m from the standoff point that ends the course.'),
        DeclareLaunchArgument(
            'align_gain', default_value='0.6',
            description='Fraction of the measured error commanded each cycle. '
                        'Below 1 guarantees monotone convergence and gives '
                        'phase margin against camera latency. If the approach '
                        'is too fussy the answer is a larger align_tolerance, '
                        'not a larger gain.'),
        DeclareLaunchArgument(
            'align_tolerance', default_value='0.08',
            description='m. The number that decides whether the aircraft goes '
                        'through the ring or catches its shoulder on it.'),
        DeclareLaunchArgument('align_settle_seconds', default_value='1.5'),
        DeclareLaunchArgument(
            'yaw_tolerance_deg', default_value='6.0',
            description='deg of heading error allowed before the run-in. A yaw '
                        'error walks the aircraft sideways by '
                        'standoff*sin(err) as it runs in, and nothing is '
                        'watching by then.'),
        DeclareLaunchArgument('grab_tolerance', default_value='0.12'),
        DeclareLaunchArgument(
            'grab_hold_seconds', default_value='2.0',
            description='s parked at the grab point before backing out.'),

        # ---- the estimate ----
        DeclareLaunchArgument(
            'buffer_seconds', default_value='1.5',
            description='Length of the rolling window the median is taken over.'),
        DeclareLaunchArgument(
            'pose_min_samples', default_value='5',
            description='Accepted samples needed before the board is flown to.'),
        DeclareLaunchArgument(
            'pose_max_age', default_value='1.0',
            description='s after which the newest sample stops being evidence '
                        'about where the board is now.'),
        DeclareLaunchArgument(
            'pose_lost_timeout', default_value='6.0',
            description='s without a usable pose during the course or the fine '
                        'alignment before the attempt is abandoned into a '
                        'landing. NOT applied during the run-in, where losing '
                        'sight of the board is expected.'),
        DeclareLaunchArgument(
            'gate_metres', default_value='0.60',
            description='m a new sample may sit from the current estimate '
                        'before it is rejected as an outlier.'),
        DeclareLaunchArgument('gate_yaw_deg', default_value='35.0'),
        DeclareLaunchArgument(
            'max_tilt_deg', default_value='30.0',
            description='deg off vertical the board may be before a detection '
                        'is rejected as the floor, a ceiling or a bad solve.'),

        # ---- the board ----
        DeclareLaunchArgument(
            'marker_ids', default_value='[4, 5, 6]',
            description='Board marker ids, BOTTOM TO TOP. Evenly spaced.'),
        DeclareLaunchArgument(
            'marker_size', default_value='0.10',
            description='m, edge of the black square. MEASURE IT WITH A TAPE. '
                        'This is the metric scale of the entire pose: 10 % out '
                        'here is 10 % out on the 20 cm grab depth.'),
        DeclareLaunchArgument(
            'marker_gap', default_value='0.10',
            description='m of blank between two markers, so marker_size + '
                        'marker_gap is the centre-to-centre pitch.'),
        DeclareLaunchArgument(
            'grab_marker_id', default_value='4',
            description='The marker the ring hangs on. Everything downstream '
                        'is measured from its centre.'),
        DeclareLaunchArgument(
            'aruco_dict', default_value='DICT_4X4_50',
            description='Must match the printed board. The sim textures were '
                        'DICT_4X4_50.'),

        # ---- the detector ----
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='auto',
            description="auto = image_topic with the last segment swapped for "
                        "camera_info, which is what picks the COLOUR "
                        "intrinsics rather than the depth module's."),
        DeclareLaunchArgument(
            'max_fps', default_value='15.0',
            description='Cap on frames actually processed. The driver running '
                        'faster than the detector costs USB bandwidth, not CPU.'),
        DeclareLaunchArgument('depth_min', default_value='0.25'),
        DeclareLaunchArgument('depth_max', default_value='8.0'),
        DeclareLaunchArgument(
            'max_reproj_px', default_value='4.0',
            description='px. Worst single corner of the accepted solution '
                        'against the corners it was fitted to. Catches a '
                        'misdescribed board -- wrong marker_gap, wrong id '
                        'order -- rather than noise.'),
        DeclareLaunchArgument(
            'min_markers', default_value='1',
            description='Markers needed for an accepted frame. 2 is the '
                        'stronger setting: the board is solved as one rigid '
                        'body, and the vertical baseline between two markers '
                        'is what makes the pose well conditioned. 1 still '
                        'works, and is what keeps the fix alive as the board '
                        'fills the frame on the run-in.'),
        DeclareLaunchArgument('detect_frames', default_value='3'),
        DeclareLaunchArgument('lost_frames', default_value='5'),
        DeclareLaunchArgument(
            'hfov_deg', default_value='69.0',
            description='D435i colour. ONLY used if camera_info never arrives, '
                        'and then distances carry whatever error it has.'),
        DeclareLaunchArgument(
            'image_rotate', default_value='0',
            description='0|90|180|270, applied before detection. Use it if the '
                        'camera is bolted on rotated.'),
        DeclareLaunchArgument(
            'stream_port', default_value='8080',
            description='Browser view: http://<jetson-ip>:8080/. 0 = off.'),
        DeclareLaunchArgument('stream_scale', default_value='0.6'),
        DeclareLaunchArgument('jpeg_quality', default_value='70'),

        # ---- the camera, and where it is bolted ----
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument(
            'camera_namespace', default_value='camera',
            description='With camera_name this is what makes the topics '
                        '/camera/camera/...'),
        DeclareLaunchArgument(
            'color_profile', default_value='1280,720,30',
            description="The D435i's native colour mode. Anything else makes "
                        'librealsense rescale for nothing.'),
        DeclareLaunchArgument(
            'cam_x', default_value='0.0',
            description='Camera position in the body frame, ROS convention '
                        '(x FORWARD, y LEFT, z UP), metres. THE SAME NUMBERS '
                        'window_traverse.launch.py takes.'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.0'),
        DeclareLaunchArgument(
            'cam_roll', default_value='0.0',
            description='Camera mounting rotation, radians, ROS convention, '
                        'yaw then pitch then roll.'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),

        # ---- the agent ----
        DeclareLaunchArgument(
            'agent_cmd', default_value='MicroXRCEAgent',
            description='The uXRCE-DDS agent binary. Built from source on this '
                        'Jetson at /usr/local/bin/MicroXRCEAgent.'),
        DeclareLaunchArgument('agent_dev', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('agent_baud', default_value='921600'),

        # ---- the rest ----
        DeclareLaunchArgument(
            'flight_node_delay', default_value='12.0',
            description='s before the flight node starts, so the DDS session '
                        'is up and PX4 topics exist first.'),
        DeclareLaunchArgument('lcd', default_value='false'),
        DeclareLaunchArgument(
            'lcd_port', default_value='',
            description='Arduino serial port; empty = auto-detect.'),

        # flight:=false leaves the camera side running on its own, which is the
        # bench setup. Grouped rather than conditioned individually because two
        # of these already carry a condition of their own.
        GroupAction([microxrce_node, lcd_node, grab_node],
                    condition=IfCondition(flight)),
        realsense,
        detect_node,
    ])
