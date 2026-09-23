"""
Precision landing on a downward ArUco marker.

    arm -> climb to takeoff_altitude -> hold -> search for the marker ->
    align over it to within align_tolerance -> hold -> land on it.

WHAT TO RUN, IN THIS ORDER. Do not skip a rung.

1. BENCH, no propellers, no flight controller needed. This is the axis
   sign check and it is the default mode, so this is also what you get if
   you forget to pass anything:

       ros2 launch drone_testing precision_land.launch.py flight:=false
       ros2 run drone_testing precision_land --ros-args -p mode:=bench

   Watch the log. Put the marker under the camera and move it around:

       marker is FORWARD 0.20 m, RIGHT 0.30 m

   Move the marker to the drone's RIGHT -- it must say RIGHT. Towards the
   NOSE -- it must say FORWARD. If either is inverted or swapped, STOP and
   fix image_rotate (or the mounting) before anything flies. A sign error
   here does not wobble; it flies the aircraft away from the pad and
   accelerates, because every new frame says "further".

   The browser view is on http://<jetson-ip>:8080/ while this runs.

2. INSPECT, in the air. Flies the climb, the hold and the search, then
   prints the point it WOULD fly to and holds. Nothing lateral is
   published:

       ros2 launch drone_testing precision_land.launch.py
       ros2 run drone_testing precision_land --ros-args \
           -p mode:=inspect -p takeoff_altitude:=2.0

3. ALIGN WITHOUT LANDING. Closes the loop with the vehicle still at
   altitude, so you can watch it converge and abort with q:

       ros2 run drone_testing precision_land --ros-args \
           -p mode:=align -p land_after_align:=false -p takeoff_altitude:=2.0

4. THE REAL THING:

       ros2 run drone_testing precision_land --ros-args \
           -p mode:=align -p takeoff_altitude:=2.0

agent_only defaults to true, the same as every other launch file here, so
the support stack comes up from launch and you run the flight node by hand
in a second pane. That keeps stdin a tty, which is the only way the q / k
aborts work. Your RC kill switch is the real safety net regardless.

ALTITUDE, AND WHY 2 m IS A GOOD CHOICE
--------------------------------------
The capture basket is the camera footprint, and it grows with height; the
alignment has to be FINISHED before the marker leaves the frame on the way
down, and it grows harder to hold the lower you go. At 2 m with a 78 deg
horizontal FOV in 4:3 the footprint is about 3.2 m across and 2.4 m
fore-aft, so an 0.80 m marker is fully visible while its centre is within
roughly +/-1.2 m left/right and +/-0.8 m fore/aft of the vehicle. That is
the basket you have to arrive inside.

At the other end, the 0.80 m marker stops fitting in the frame below about
0.66 m, so everything under blind_commit_altitude (1.0 m by default) is an
open-loop descent on the last held point. MEASURE the real number on the
bench -- walk the airframe down over the marker and note where
/aruco/detected goes false -- and set blind_commit_altitude to it.

PX4 SIDE
--------
Nothing here changes the PX4 configuration. This flies on the same ARK
Flow + lidar + IMU stack as the other tests, and the same pre-flight
checklist applies -- in particular reboot the flight controller first, or
cs_rng_kin_consistent will refuse the arm. See README section 11.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')
    flight = LaunchConfiguration('flight')

    # The detector. Started a little ahead of the flight node so the camera
    # has opened and /aruco/detected is already being published by the time
    # anything asks it a question.
    aruco_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='drone_testing',
                executable='aruco_pose',
                name='aruco_pose',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'camera_index': LaunchConfiguration('camera_index'),
                    'width': LaunchConfiguration('width'),
                    'height': LaunchConfiguration('height'),
                    'fourcc': LaunchConfiguration('fourcc'),
                    'camera_fps': LaunchConfiguration('camera_fps'),
                    'marker_id': LaunchConfiguration('marker_id'),
                    'marker_size': LaunchConfiguration('marker_size'),
                    'aruco_dict': LaunchConfiguration('aruco_dict'),
                    'hfov_deg': LaunchConfiguration('hfov_deg'),
                    'image_rotate': LaunchConfiguration('image_rotate'),
                    'detect_rate': LaunchConfiguration('detect_rate'),
                    'stream_port': LaunchConfiguration('stream_port'),
                    'stream_scale': LaunchConfiguration('stream_scale'),
                    'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                    'show_gui': LaunchConfiguration('show_gui'),
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('detect')),
    )

    # The flight. Held back until the DDS session is up and PX4's topics
    # exist, or the first setpoints are dropped.
    land_node = TimerAction(
        period=LaunchConfiguration('flight_node_delay'),
        actions=[
            Node(
                package='drone_testing',
                executable='precision_land',
                name='precision_land',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'mode': LaunchConfiguration('mode'),
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'move_speed': LaunchConfiguration('move_speed'),
                    'min_altitude': LaunchConfiguration('min_altitude'),
                    'max_altitude': LaunchConfiguration('max_altitude'),
                    'align_tolerance': LaunchConfiguration('align_tolerance'),
                    'align_settle_seconds': LaunchConfiguration('align_settle_seconds'),
                    'align_gain': LaunchConfiguration('align_gain'),
                    'aligned_hold_seconds': LaunchConfiguration('aligned_hold_seconds'),
                    'land_after_align': LaunchConfiguration('land_after_align'),
                    'search_seconds': LaunchConfiguration('search_seconds'),
                    'align_timeout': LaunchConfiguration('align_timeout'),
                    'flight_seconds': LaunchConfiguration('flight_seconds'),
                    'marker_max_age': LaunchConfiguration('marker_max_age'),
                    'marker_lost_seconds': LaunchConfiguration('marker_lost_seconds'),
                    'precision_descent': LaunchConfiguration('precision_descent'),
                    'blind_commit_altitude': LaunchConfiguration('blind_commit_altitude'),
                    'on_fail': LaunchConfiguration('on_fail'),
                    'request_offboard_from_ros': LaunchConfiguration(
                        'request_offboard_from_ros'),
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
            description='Start the detector but not the flight '
                        'node, so you can run that by hand and keep the q/k '
                        'keyboard aborts. false = fly the whole thing.'),
        DeclareLaunchArgument(
            'flight', default_value='true',
            description='false = detection only: no '
                        'flight node. Use this for the bench sign check.'),
        DeclareLaunchArgument(
            'detect', default_value='true',
            description='Start the aruco_pose detector node.'),

        # ---- mode ----
        DeclareLaunchArgument(
            'mode', default_value='bench',
            description='bench = print the marker direction, command NOTHING '
                        '(the axis sign check; run this first). '
                        'inspect = fly and report the target without '
                        'publishing it. align = close the loop and land.'),

        # ---- the flight ----
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='2.0',
            description='Metres above the arming point. 2.0 gives a ~3.2 x 2.4 m '
                        'capture footprint; going lower shrinks the basket.'),
        DeclareLaunchArgument(
            'hold_seconds', default_value='5.0',
            description='Station keeping at altitude before the search. The '
                        'optical flow x/y latch has to happen in here.'),
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
            'move_speed', default_value='0.30',
            description='m/s the lateral correction carrot is walked at. Keep '
                        'it slow: the optical flow is what measures it.'),
        DeclareLaunchArgument(
            'min_altitude', default_value='0.40',
            description='m above the arming point the flight may not go below.'),
        DeclareLaunchArgument(
            'max_altitude', default_value='3.00',
            description='m above the arming point the flight may not exceed.'),
        DeclareLaunchArgument(
            'request_offboard_from_ros', default_value='false',
            description='false = you flip the Offboard switch on the TX.'),

        # ---- alignment ----
        DeclareLaunchArgument(
            'align_tolerance', default_value='0.15',
            description='m radius that counts as centred over the marker.'),
        DeclareLaunchArgument(
            'align_settle_seconds', default_value='1.5',
            description='s the vehicle must stay inside that radius before the '
                        'alignment is believed. Stops corner noise triggering it.'),
        DeclareLaunchArgument(
            'align_gain', default_value='0.6',
            description='Fraction of the measured offset commanded each cycle. '
                        'Below 1 guarantees monotone convergence and gives phase '
                        'margin against camera latency. Raise slowly if at all.'),
        DeclareLaunchArgument(
            'aligned_hold_seconds', default_value='10.0',
            description='s held over the marker before the descent starts.'),
        DeclareLaunchArgument(
            'land_after_align', default_value='true',
            description='false = align and stay up. Worth one flight before '
                        'you let it land itself.'),
        DeclareLaunchArgument(
            'search_seconds', default_value='20.0',
            description='s hovering and looking before giving up. The search is '
                        'stationary on purpose: the camera looks straight down, '
                        'so yawing would not sweep new ground.'),
        DeclareLaunchArgument(
            'align_timeout', default_value='45.0',
            description='s trying to centre before giving up.'),
        DeclareLaunchArgument(
            'flight_seconds', default_value='120.0',
            description='s from the START OF THE CLIMB to a forced descent, '
                        'whatever else is happening. The backstop.'),
        DeclareLaunchArgument(
            'on_fail', default_value='land',
            description='What a search/align timeout does: land | hold.'),
        DeclareLaunchArgument(
            'marker_max_age', default_value='0.5',
            description='s after which the last pose is not evidence of '
                        'anything. A dead camera goes quiet, and quiet must '
                        'read as "no marker".'),
        DeclareLaunchArgument(
            'marker_lost_seconds', default_value='5.0',
            description='s without a marker during alignment before it goes '
                        'back to searching. It holds position meanwhile.'),
        DeclareLaunchArgument(
            'precision_descent', default_value='true',
            description='Keep the aligned x/y hold through the descent instead '
                        'of dropping to zero-velocity hold. false = the '
                        'inherited behaviour, which drifts.'),
        DeclareLaunchArgument(
            'blind_commit_altitude', default_value='1.0',
            description='m below which the marker is expected to be out of '
                        'frame. Logged, not enforced. MEASURE IT on the bench.'),

        # ---- the camera ----
        DeclareLaunchArgument(
            'camera_index', default_value='0',
            description='cv2.VideoCapture index of the down-facing camera.'),
        DeclareLaunchArgument(
            'width', default_value='800',
            description='4:3 on purpose: the taller vertical FOV keeps the '
                        'marker in frame ~20 cm lower than 16:9 does.'),
        DeclareLaunchArgument('height', default_value='600', description='See width.'),
        DeclareLaunchArgument(
            'fourcc', default_value='MJPG',
            description='MJPG gets 30 fps at 800x600 on this camera; YUYV does not.'),
        DeclareLaunchArgument('camera_fps', default_value='30.0', description='Requested fps.'),
        DeclareLaunchArgument(
            'detect_rate', default_value='20.0',
            description='Hz the newest frame is processed at.'),
        DeclareLaunchArgument(
            'marker_id', default_value='0', description='ArUco id to track.'),
        DeclareLaunchArgument(
            'marker_size', default_value='0.80',
            description='Marker edge length in metres. Must be right: it sets '
                        'the metric scale of the whole pose.'),
        DeclareLaunchArgument(
            'aruco_dict', default_value='DICT_5X5_50',
            description='cv2.aruco predefined dictionary name.'),
        DeclareLaunchArgument(
            'hfov_deg', default_value='78.0',
            description='Horizontal field of view. Scales the reported HEIGHT, '
                        'which nothing uses; x and y are unaffected by it.'),
        DeclareLaunchArgument(
            'image_rotate', default_value='0',
            description='0|90|180|270, applied before detection. Use this if '
                        'the camera is bolted on rotated, so that image-up '
                        'still means the nose.'),
        DeclareLaunchArgument(
            'stream_port', default_value='8080',
            description='Browser MJPEG view: http://<jetson-ip>:8080/. 0 off.'),
        DeclareLaunchArgument('stream_scale', default_value='0.6', description='Downscale before JPEG.'),
        DeclareLaunchArgument('jpeg_quality', default_value='70', description='Stream JPEG quality.'),
        DeclareLaunchArgument(
            'show_gui', default_value='false',
            description='cv2.imshow the frame. Needs a display; leave false '
                        'on a headless Jetson.'),

        # ---- misc ----
        DeclareLaunchArgument(
            'flight_node_delay', default_value='8.0',
            description='s before the flight node starts, so the DDS session '
                        'is up and PX4 topics exist first.'),
        DeclareLaunchArgument(
            'lcd', default_value='true',
            description='Start the Arduino TFT status node.'),
        DeclareLaunchArgument(
            'lcd_port', default_value='',
            description='Arduino serial port; empty = auto-detect ttyACM*/ttyUSB*.'),

        # flight:=false leaves just the camera side running, which is the bench
        # sign check. Grouped rather than given a condition directly, because
        # two of these already carry one of their own.
        GroupAction([lcd_node, land_node],
                    condition=IfCondition(flight)),
        aruco_node,
    ])
