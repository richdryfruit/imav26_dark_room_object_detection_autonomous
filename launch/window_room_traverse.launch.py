"""
Window traversal -> box pattern inside the room -> traversal back out, with
geotagged doll counting and a TFT readout.

    ros2 launch drone_testing window_room_traverse.launch.py agent_only:=false

WHAT THIS STARTS, AND WHERE IT COMES FROM
-----------------------------------------
The support stack is NOT re-implemented here. This file INCLUDES
window_traverse.launch.py -- the one that works -- with agent_only:=true, so
that file brings up exactly what it always brings up:

    window_detect with the same HSV/geometry configuration

(the uXRCE-DDS agent and the RealSense D435i -- colour + aligned depth -- come
from imav_bringup, which must already be running)

and does NOT bring up its own flight node. Every camera and detector argument
below is forwarded into that include, so there is one definition of each and
the two files cannot drift apart. If the traversal behaviour changes there,
it changes here.

What this file adds is three nodes:

    window_room_traverse   the flight. A subclass of window_traverse: same
                           sweep, same lock, same approach, same clearance
                           arithmetic, plus the room pattern and the second
                           traversal. See window_room_traverse.py.
    doll_detect            the TensorRT doll model, gated on the flight node's
                           /doll_detect_enable, counting dolls by WHERE THEY
                           ARE IN THE ROOM rather than by track id. See
                           doll_detect.py.
    room_display           the Arduino TFT bridge. Needs
                           arduino/room_status/room_status.ino flashed on the
                           board -- NOT the older tft_status.ino, the two
                           sketches take different protocols.

THE FLIGHT, IN ORDER
--------------------
    climb -> hold -> find the blue window -> lock -> line up -> through it and
    inside_distance (1.20 m) past the plane -> hold -> 30 cm left -> 30 cm
    forward -> 90 deg right -> 30 cm -> 90 deg right -> 30 cm -> find the
    window again (now facing the wall it came through) -> line up -> back out
    through it -> hold -> land.

Doll detection runs from the moment the aircraft commits to the inbound
traverse until it is clear of the window on the way back out. Nothing else in
the flight is gated on it: if the engine will not load, the flight is
unaffected and the count is zero.

BEFORE THE FIRST FLIGHT
-----------------------
Everything window_traverse.launch.py's header says still applies -- the camera
mounting measurement, the CPU margin, the flow health check, walking the
estimate before flying it. Read it. In addition:

1. THE ROOM HAS TO FIT THE PATTERN, AND THEN THE STANDOFF. 30 cm left, 30 cm
   forward, then two right-angle turns and two more 30 cm legs puts the
   aircraft roughly 0.3 m to the left of and 0.3 m behind where it entered,
   facing back at the window wall from about inside_distance minus
   room_forward_3 -- call it 0.9 m with the defaults.

   That is NOT the deepest point of the flight. The return approach flies to
   the standoff point, which is standoff_distance (2.0 m, and further if the
   window is big -- see effective_standoff) in FRONT of the window on its
   axis, i.e. INSIDE the room. So the room must be at least
   standoff_distance plus the airframe deep, measured from the window wall,
   or the aircraft will try to back into a wall it cannot see. Pace it out
   with the props off before flying it. A room that cannot give that depth
   needs standoff_distance lowered, and lowering it costs the return
   approach its view of the whole window -- which is the trade that
   return_through_window:=false exists for.

2. THE WINDOW HAS TO BE VISIBLE FROM INSIDE. RELOCK does not sweep: it stands
   still facing wherever the pattern left it and rebuilds the pose. If the
   two turns do not leave the window in frame, nothing else will find it and
   the aircraft lands in the room after relock_timeout. Check by standing
   where the pattern ends, facing where it ends up facing, and looking at
   /window_detected.

3. THE CAMERA MOUNTING IS SHARED. cam_x/cam_y/cam_z/cam_roll/cam_pitch/cam_yaw
   go to BOTH the flight node and the doll node, in the same ROS convention
   (x forward, y left, z up). Get them wrong and the window is misplaced by
   the offset AND every doll is, which merges dolls that are not the same one.

4. MERGE RADIUS. doll_merge_radius (0.20 m) is what makes two sightings the
   same doll. It must be smaller than the smallest gap between two real dolls
   and bigger than the position error. Measure the gap in your arena; if the
   dolls are closer together than about a metre, lower it and expect the count
   to be more sensitive to depth noise.

USEFUL VARIATIONS
-----------------
    agent_only:=false             fly it (the default is true: support stack
                                  only, so the flight node can be run by hand
                                  with `ros2 run` and keep the q/k aborts)
    return_through_window:=false  fly in, do the pattern, land inside. This is
                                  the first flight in a new room.
    dolls:=false                  no doll model at all. Bench the flight first.
    require_enable:=false         run the doll model the whole time, on the
                                  ground included. This is how you bench the
                                  model and the geotagging without flying.
    display:=false                no Arduino.
    flight:=false                 detector only, no flight node: the
                                  bench test.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')
    flight = LaunchConfiguration('flight')

    # The camera mounting. Shared by the flight node and the doll node, which
    # is the point of having it in one dict -- see item 3 in the header.
    camera_mounting = {
        'cam_x': LaunchConfiguration('cam_x'),
        'cam_y': LaunchConfiguration('cam_y'),
        'cam_z': LaunchConfiguration('cam_z'),
        'cam_roll': LaunchConfiguration('cam_roll'),
        'cam_pitch': LaunchConfiguration('cam_pitch'),
        'cam_yaw': LaunchConfiguration('cam_yaw'),
    }

    # ---- the support stack, from the file that already works --------------
    #
    # agent_only is pinned to 'true' HERE rather than forwarded, and that is
    # load-bearing: it is what stops the include from also starting its own
    # window_traverse node. Two flight nodes publishing setpoints at the same
    # PX4 is not a race this file would win.
    #
    # Forwarding is left ON (the default) so every camera, detector and flight
    # argument declared below reaches the include without being listed twice.
    # The include's own DeclareLaunchArgument defaults lose to a forwarded
    # value, so the defaults in this file are the ones that apply.
    support = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('drone_testing'),
            'launch', 'window_traverse.launch.py'])),
        launch_arguments={
            'agent_only': 'true',
            'flight': LaunchConfiguration('flight'),
            'detect': LaunchConfiguration('detect'),
            # The old six-row sketch's bridge. Off: this mission's board runs
            # room_status.ino and room_display below. Pinned rather than
            # forwarded so `lcd:=true` cannot start a node that would fight
            # room_display for the same serial port.
            'lcd': 'false',
        }.items(),
    )

    # ---- the flight -------------------------------------------------------
    flight_node = TimerAction(
        period=LaunchConfiguration('flight_node_delay'),
        actions=[
            Node(
                package='drone_testing',
                executable='window_room_traverse',
                name='window_room_traverse',
                output='screen',
                emulate_tty=True,
                parameters=[dict(camera_mounting, **{
                    # ---- the climb and the sweep (inherited) ----
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'yaw_rate': LaunchConfiguration('yaw_rate'),
                    'takeoff_return_to_pad': LaunchConfiguration(
                        'takeoff_return_to_pad'),
                    'min_altitude': LaunchConfiguration('min_altitude'),
                    'max_altitude': LaunchConfiguration('max_altitude'),
                    'scan_span_deg': LaunchConfiguration('scan_span_deg'),
                    'scan_yaw_rate': LaunchConfiguration('scan_yaw_rate'),
                    'scan_direction': LaunchConfiguration('scan_direction'),
                    'yaw_cone_deg': LaunchConfiguration('yaw_cone_deg'),
                    'detect_seconds': LaunchConfiguration('detect_seconds'),
                    'relock_on_loss': LaunchConfiguration('relock_on_loss'),
                    'flight_seconds': LaunchConfiguration('flight_seconds'),
                    'request_offboard_from_ros': LaunchConfiguration(
                        'request_offboard_from_ros'),
                    # ---- the two traversals ----
                    # exit_distance is NOT passed: this node has two of them
                    # and reads inside_distance / outside_distance instead.
                    'standoff_distance': LaunchConfiguration('standoff_distance'),
                    'inside_distance': LaunchConfiguration('inside_distance'),
                    'outside_distance': LaunchConfiguration('outside_distance'),
                    'altitude_offset': LaunchConfiguration('altitude_offset'),
                    'gear_below_camera': LaunchConfiguration('gear_below_camera'),
                    'drone_height': LaunchConfiguration('drone_height'),
                    'drone_width': LaunchConfiguration('drone_width'),
                    'vertical_clearance': LaunchConfiguration('vertical_clearance'),
                    'lateral_clearance': LaunchConfiguration('lateral_clearance'),
                    'hard_clearance': LaunchConfiguration('hard_clearance'),
                    'sill_bias': LaunchConfiguration('sill_bias'),
                    'align_alt_tolerance': LaunchConfiguration('align_alt_tolerance'),
                    'approach_speed': LaunchConfiguration('approach_speed'),
                    'traverse_speed': LaunchConfiguration('traverse_speed'),
                    'align_tolerance': LaunchConfiguration('align_tolerance'),
                    'align_cross_tolerance': LaunchConfiguration(
                        'align_cross_tolerance'),
                    'align_along_tolerance': LaunchConfiguration(
                        'align_along_tolerance'),
                    'recentre_clear_seconds': LaunchConfiguration(
                        'recentre_clear_seconds'),
                    'recentre_yaw_step_deg': LaunchConfiguration(
                        'recentre_yaw_step_deg'),
                    'recentre_yaw_limit_deg': LaunchConfiguration(
                        'recentre_yaw_limit_deg'),
                    'recentre_timeout': LaunchConfiguration('recentre_timeout'),
                    'recentre_backoff_seconds': LaunchConfiguration(
                        'recentre_backoff_seconds'),
                    'recentre_backoff': LaunchConfiguration('recentre_backoff'),
                    'recentre_max_backoffs': LaunchConfiguration(
                        'recentre_max_backoffs'),
                    'align_yaw_tolerance_deg': LaunchConfiguration(
                        'align_yaw_tolerance_deg'),
                    'align_settle_seconds': LaunchConfiguration('align_settle_seconds'),
                    'align_timeout': LaunchConfiguration('align_timeout'),
                    'traverse_timeout': LaunchConfiguration('traverse_timeout'),
                    'clear_seconds': LaunchConfiguration('clear_seconds'),
                    'blind_traverse_seconds': LaunchConfiguration(
                        'blind_traverse_seconds'),
                    # ---- the room pattern ----
                    'return_through_window': LaunchConfiguration(
                        'return_through_window'),
                    # Free-form in-room plan. Empty = the six-leg box
                    # built from the six parameters below.
                    'room_sequence': LaunchConfiguration('room_sequence'),
                    'room_strafe': LaunchConfiguration('room_strafe'),
                    'room_forward_1': LaunchConfiguration('room_forward_1'),
                    'room_turn_1_deg': LaunchConfiguration('room_turn_1_deg'),
                    'room_forward_2': LaunchConfiguration('room_forward_2'),
                    'room_turn_2_deg': LaunchConfiguration('room_turn_2_deg'),
                    'room_forward_3': LaunchConfiguration('room_forward_3'),
                    'room_speed': LaunchConfiguration('room_speed'),
                    'room_hold_seconds': LaunchConfiguration('room_hold_seconds'),
                    'room_move_timeout': LaunchConfiguration('room_move_timeout'),
                    'room_turn_timeout': LaunchConfiguration('room_turn_timeout'),
                    'relock_timeout': LaunchConfiguration('relock_timeout'),
                    'relock_settle_seconds': LaunchConfiguration(
                        'relock_settle_seconds'),
                    # ---- the estimator ----
                    'depth_min': LaunchConfiguration('depth_min'),
                    'depth_max': LaunchConfiguration('depth_max'),
                    'corner_spread': LaunchConfiguration('corner_spread'),
                    'corner_spread_frac': LaunchConfiguration('corner_spread_frac'),
                    'plane_tolerance': LaunchConfiguration('plane_tolerance'),
                    'window_min_size': LaunchConfiguration('window_min_size'),
                    'window_max_size': LaunchConfiguration('window_max_size'),
                    'max_tilt_deg': LaunchConfiguration('max_tilt_deg'),
                    'buffer_seconds': LaunchConfiguration('buffer_seconds'),
                    'pose_min_samples': LaunchConfiguration('pose_min_samples'),
                    'pose_max_age': LaunchConfiguration('pose_max_age'),
                    'pose_lost_timeout': LaunchConfiguration('pose_lost_timeout'),
                    'gate_metres': LaunchConfiguration('gate_metres'),
                    'gate_yaw_deg': LaunchConfiguration('gate_yaw_deg'),
                    'gate_reset_count': LaunchConfiguration('gate_reset_count'),
                    'side_mismatch': LaunchConfiguration('side_mismatch'),
                })],
            )
        ],
        condition=UnlessCondition(agent_only),
    )

    # ---- the doll model ---------------------------------------------------
    #
    # Started on the SAME delay as the flight node, not earlier. Loading the
    # TensorRT engine is several seconds of GPU work and the node defers it to
    # the first frame it is enabled for anyway, but starting the process while
    # librealsense is still enumerating the camera is spending the startup
    # margin that item 2 of window_traverse.launch.py's header is about.
    #
    # It reads the camera through the SAME topics window_detect does. There is
    # no second camera and no cv2.VideoCapture here: the D435i is already
    # streaming colour and aligned depth for the window, the doll model needs
    # exactly those two, and opening the device twice on one USB3 bus is how
    # librealsense refuses the second one.
    doll_node = TimerAction(
        period=LaunchConfiguration('flight_node_delay'),
        actions=[
            Node(
                package='drone_testing',
                executable='doll_detect',
                name='doll_detect',
                output='screen',
                emulate_tty=True,
                parameters=[dict(camera_mounting, **{
                    'model_path': LaunchConfiguration('doll_model'),
                    'tracker_path': LaunchConfiguration('doll_tracker'),
                    # The dolls are on the floor: the DOWNWARD C920
                    # (imav_bringup's usb_cam), not the level RealSense. Its
                    # own mounting, overriding camera_mounting above.
                    'image_topic': LaunchConfiguration('doll_image_topic'),
                    'depth_topic': LaunchConfiguration('depth_topic'),
                    'camera_info_topic': LaunchConfiguration('doll_camera_info_topic'),
                    'depth_source': LaunchConfiguration('doll_depth_source'),
                    'target_height': LaunchConfiguration('doll_target_height'),
                    'hfov_deg': LaunchConfiguration('doll_hfov_deg'),
                    'geotag_frame': LaunchConfiguration('doll_geotag_frame'),
                    'cam_x': LaunchConfiguration('doll_cam_x'),
                    'cam_y': LaunchConfiguration('doll_cam_y'),
                    'cam_z': LaunchConfiguration('doll_cam_z'),
                    'cam_roll': 0.0,
                    'cam_pitch': LaunchConfiguration('doll_cam_pitch'),
                    'cam_yaw': 0.0,
                    'confidence': LaunchConfiguration('doll_confidence'),
                    'min_frames_to_confirm': LaunchConfiguration(
                        'doll_min_frames'),
                    'max_fps': LaunchConfiguration('doll_max_fps'),
                    'merge_radius': LaunchConfiguration('doll_merge_radius'),
                    'depth_min': LaunchConfiguration('doll_depth_min'),
                    'depth_max': LaunchConfiguration('doll_depth_max'),
                    'depth_patch': LaunchConfiguration('doll_depth_patch'),
                    'min_depth_pixels': LaunchConfiguration('doll_min_depth_pixels'),
                    'publish_image': LaunchConfiguration('doll_publish_image'),
                    'require_enable': LaunchConfiguration('require_enable'),
                })],
                # WHERE torch AND ultralytics COME FROM.
                #
                # Ubuntu 24.04 marks its Python as externally managed (PEP
                # 668), so torch and ultralytics are installed in a venv built
                # with --system-site-packages rather than into the system or
                # user site. That venv inherits this machine's numpy, cv2 and
                # tensorrt instead of shipping its own, so there is exactly one
                # of each on the path and nothing shadows the working install.
                #
                # Putting its site-packages on PYTHONPATH for THIS NODE ONLY is
                # what lets the plain `doll_detect` entry point import them.
                # It is prepended to the inherited PYTHONPATH, not substituted
                # for it: rclpy, px4_msgs and drone_testing itself all arrive
                # on that variable from the ROS setup files, and a node that
                # replaced it would not start at all.
                #
                # A path that does not exist is ignored by Python, so this is
                # harmless on a machine where the venv was never created -- the
                # node then fails the way it already failed, by logging that it
                # cannot load the model and counting nothing.
                additional_env={'PYTHONPATH': [
                    LaunchConfiguration('doll_venv'), ':',
                    EnvironmentVariable('PYTHONPATH', default_value=''),
                ]},
            )
        ],
        condition=IfCondition(LaunchConfiguration('dolls')),
    )

    # ---- the screen -------------------------------------------------------
    display_node = Node(
        package='drone_testing',
        executable='room_display',
        name='room_display',
        output='screen',
        emulate_tty=True,
        parameters=[{'port': LaunchConfiguration('display_port')}],
        condition=IfCondition(LaunchConfiguration('display')),
    )

    # ---- the count in QGC -------------------------------------------------
    # The same numbers as the TFT, but over the network to QGroundControl, so
    # the count is readable with no Arduino and no screen on the aircraft.
    qgc_node = Node(
        package='drone_testing',
        executable='qgc_doll_status',
        name='qgc_doll_status',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'qgc_host': LaunchConfiguration('qgc_host'),
            'qgc_port': LaunchConfiguration('qgc_port'),
            'system_id': LaunchConfiguration('qgc_sysid'),
        }],
        condition=IfCondition(LaunchConfiguration('qgc')),
    )

    return LaunchDescription([
        # ---- what to start ----
        DeclareLaunchArgument(
            'agent_only', default_value='true',
            description='Start the support stack but not the flight node, so '
                        'it can be run by hand and keep the q/k keyboard '
                        'aborts. false = fly the whole mission from here.'),
        DeclareLaunchArgument(
            'flight', default_value='true',
            description='false = detector only: no flight node, '
                        'no doll node. The bench test.'),
        DeclareLaunchArgument('detect', default_value='true',
                              description='Start window_detect.'),
        DeclareLaunchArgument('dolls', default_value='true',
                              description='Start the doll detector.'),
        DeclareLaunchArgument('qgc', default_value='true',
                              description='Send the doll count to QGC as '
                                          'STATUSTEXT and NAMED_VALUE_INT. '
                                          'No Arduino needed.'),
        DeclareLaunchArgument('qgc_host', default_value='255.255.255.255',
                              description='Where QGC is. The default '
                                          'broadcasts on the subnet; set the '
                                          "laptop's IP if broadcast is "
                                          'blocked on the WiFi.'),
        DeclareLaunchArgument('qgc_port', default_value='14550'),
        DeclareLaunchArgument('qgc_sysid', default_value='1',
                              description="The vehicle's MAV_SYS_ID, so QGC "
                                          'files the count under the aircraft '
                                          'instead of a second vehicle.'),
        DeclareLaunchArgument('display', default_value='true',
                              description='Start the Arduino TFT bridge. '
                                          'Needs room_status.ino flashed.'),
        DeclareLaunchArgument(
            'display_port', default_value='',
            description='Arduino serial port; empty = auto-detect (first '
                        '/dev/ttyACM*, then /dev/ttyUSB*).'),
        DeclareLaunchArgument('flight_node_delay', default_value='12.0',
                              description='s after launch before the flight '
                                           'and doll nodes start. The camera '
                                           'takes ~9 s to come up.'),

        # ---- camera and detector (forwarded into window_traverse.launch.py) ----
        DeclareLaunchArgument('image_topic',
                              default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='Must be the ALIGNED topic: both the window geometry '
                        'and the doll geotag index it by colour-image pixel.'),
        DeclareLaunchArgument('camera_info_topic', default_value='auto'),
        DeclareLaunchArgument('color', default_value='blue',
                              description='HSV range for the window.'),

        # ---- the camera mounting, in the body frame, ROS convention ----
        DeclareLaunchArgument(
            'cam_x', default_value='0.105',
            description='m forward of the CoG. Shared by the flight node and '
                        'the doll node -- measure it once, properly.'),
        DeclareLaunchArgument('cam_y', default_value='0.0',
                              description='m LEFT of the CoG.'),
        DeclareLaunchArgument('cam_z', default_value='-0.04',
                              description='m UP from the CoG (negative = below).'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0',
                              description='rad, positive = nose up.'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),

        # ---- the climb and the sweep ----
        DeclareLaunchArgument('takeoff_altitude', default_value='1.2'),
        DeclareLaunchArgument('hold_seconds', default_value='5.0'),
        DeclareLaunchArgument('ground_wait_seconds', default_value='5.0'),
        DeclareLaunchArgument('climb_speed', default_value='0.35'),
        DeclareLaunchArgument('land_speed', default_value='0.15'),
        DeclareLaunchArgument('yaw_rate', default_value='0.35',
                              description='rad/s. Also the rate the room '
                                          'pattern turns at.'),
        DeclareLaunchArgument('takeoff_return_to_pad', default_value='false'),
        DeclareLaunchArgument('min_altitude', default_value='0.4'),
        DeclareLaunchArgument('max_altitude', default_value='3.0'),
        DeclareLaunchArgument('scan_span_deg', default_value='20.0'),
        DeclareLaunchArgument('scan_yaw_rate', default_value='0.05'),
        DeclareLaunchArgument('scan_direction', default_value='right'),
        DeclareLaunchArgument(
            'yaw_cone_deg', default_value='50.0',
            description='How far off the reference heading a window may be '
                        'and still be flown at. The cone is RE-CENTRED on the '
                        'heading the room pattern ends at, so the way out is '
                        'protected about the right axis; this is its width, '
                        'both times.'),
        DeclareLaunchArgument('detect_seconds', default_value='0.4'),
        DeclareLaunchArgument('relock_on_loss', default_value='false'),
        DeclareLaunchArgument(
            'flight_seconds', default_value='300.0',
            description='Hard clock from the start of the climb. TWICE the '
                        'single-traversal default: two approaches, two '
                        'traversals and a six-leg pattern do not fit in 150 s.'),
        DeclareLaunchArgument('request_offboard_from_ros', default_value='false'),

        # ---- the two traversals ----
        DeclareLaunchArgument('standoff_distance', default_value='2.0'),
        DeclareLaunchArgument(
            'inside_distance', default_value='1.20',
            description='m past the window plane the INBOUND run ends: how '
                        'far into the room the aircraft goes before the room '
                        'pattern starts. This is also what decides how much '
                        'room the pattern needs and whether the window is '
                        'still in frame from inside.'),
        DeclareLaunchArgument(
            'outside_distance', default_value='1.20',
            description='m past the window plane the OUTBOUND run ends.'),
        DeclareLaunchArgument('altitude_offset', default_value='0.0'),
        DeclareLaunchArgument('gear_below_camera', default_value='0.120'),
        DeclareLaunchArgument('drone_height', default_value='0.260'),
        DeclareLaunchArgument('drone_width', default_value='0.260'),
        DeclareLaunchArgument('vertical_clearance', default_value='0.150'),
        DeclareLaunchArgument('lateral_clearance', default_value='0.150'),
        DeclareLaunchArgument('hard_clearance', default_value='0.030'),
        DeclareLaunchArgument('sill_bias', default_value='0.100'),
        DeclareLaunchArgument('align_alt_tolerance', default_value='0.08'),
        DeclareLaunchArgument('approach_speed', default_value='0.30'),
        DeclareLaunchArgument('traverse_speed', default_value='0.45'),
        DeclareLaunchArgument('align_tolerance', default_value='0.18'),
        DeclareLaunchArgument('align_cross_tolerance', default_value='0.06'),
        DeclareLaunchArgument('align_along_tolerance', default_value='0.25'),
        DeclareLaunchArgument('recentre_clear_seconds', default_value='0.6'),
        DeclareLaunchArgument('recentre_yaw_step_deg', default_value='4.0'),
        DeclareLaunchArgument('recentre_yaw_limit_deg', default_value='20.0'),
        DeclareLaunchArgument('recentre_timeout', default_value='20.0'),
        DeclareLaunchArgument('recentre_backoff_seconds', default_value='7.0'),
        DeclareLaunchArgument('recentre_backoff', default_value='0.60'),
        DeclareLaunchArgument('recentre_max_backoffs', default_value='2'),
        DeclareLaunchArgument('align_yaw_tolerance_deg', default_value='8.0'),
        DeclareLaunchArgument('align_settle_seconds', default_value='1.5'),
        DeclareLaunchArgument('align_timeout', default_value='60.0'),
        DeclareLaunchArgument('traverse_timeout', default_value='25.0'),
        DeclareLaunchArgument('clear_seconds', default_value='4.0'),
        DeclareLaunchArgument('blind_traverse_seconds', default_value='3.0'),

        # ---- the room pattern ----
        DeclareLaunchArgument(
            'return_through_window', default_value='true',
            description='false = fly in, fly the pattern, land inside. Fly a '
                        'new room this way first.'),
        DeclareLaunchArgument(
            'room_sequence', default_value='',
            description='In-room moves as offboard_sequence\'s grammar, e.g. '
                        '"forward 0.5, yaw 90, left 0.3". Empty = the '
                        'six-leg box from room_strafe/room_forward_N/'
                        'room_turn_N_deg. forward/backward/left/right/yaw '
                        'only; up/down are rejected.'),
        DeclareLaunchArgument('room_strafe', default_value='0.30',
                              description='m LEFT, the first leg.'),
        DeclareLaunchArgument('room_forward_1', default_value='0.30',
                              description='m forward, the second leg.'),
        DeclareLaunchArgument('room_turn_1_deg', default_value='90.0',
                              description='deg, positive = RIGHT.'),
        DeclareLaunchArgument('room_forward_2', default_value='0.30',
                              description='m straight ahead on the new heading.'),
        DeclareLaunchArgument('room_turn_2_deg', default_value='90.0'),
        DeclareLaunchArgument(
            'room_forward_3', default_value='0.30',
            description='m straight ahead again. This one is flown TOWARDS '
                        'the window wall, so it is the leg to shorten if the '
                        'aircraft ends up too close to see the whole window.'),
        DeclareLaunchArgument('room_speed', default_value='0.20',
                              description='m/s inside the room. The legs are '
                                          '30 cm; faster only buys overshoot.'),
        DeclareLaunchArgument('room_hold_seconds', default_value='2.0'),
        DeclareLaunchArgument('room_move_timeout', default_value='20.0'),
        DeclareLaunchArgument('room_turn_timeout', default_value='25.0'),
        DeclareLaunchArgument(
            'relock_timeout', default_value='45.0',
            description='s of standing inside looking for the window before '
                        'the return leg is given up and the aircraft lands '
                        'in the room.'),
        DeclareLaunchArgument('relock_settle_seconds', default_value='2.0'),

        # ---- the window estimator ----
        DeclareLaunchArgument('depth_min', default_value='0.35'),
        DeclareLaunchArgument('depth_max', default_value='8.0'),
        DeclareLaunchArgument('corner_spread', default_value='0.25'),
        DeclareLaunchArgument('corner_spread_frac', default_value='0.15'),
        DeclareLaunchArgument('plane_tolerance', default_value='0.15'),
        DeclareLaunchArgument('window_min_size', default_value='0.35'),
        DeclareLaunchArgument('window_max_size', default_value='3.0'),
        DeclareLaunchArgument('max_tilt_deg', default_value='35.0'),
        DeclareLaunchArgument('buffer_seconds', default_value='2.5'),
        DeclareLaunchArgument('pose_min_samples', default_value='6'),
        DeclareLaunchArgument('pose_max_age', default_value='1.5'),
        DeclareLaunchArgument('pose_lost_timeout', default_value='6.0'),
        DeclareLaunchArgument('gate_metres', default_value='1.0'),
        DeclareLaunchArgument('gate_yaw_deg', default_value='40.0'),
        DeclareLaunchArgument('gate_reset_count', default_value='25'),
        DeclareLaunchArgument('side_mismatch', default_value='0.40'),

        # ---- the doll detector ----
        DeclareLaunchArgument(
            'doll_model',
            default_value='/home/ark-jetson-orin-2/Downloads/DroneImpl_v7/db.engine',
            description='The TensorRT engine. Built for THIS Jetson and this '
                        'TensorRT version -- an engine copied from another '
                        'machine will not load.'),
        DeclareLaunchArgument(
            'doll_tracker',
            default_value='/home/ark-jetson-orin-2/Downloads/DroneImpl_v7/'
                          'custom_bytetrack.yaml',
            description='ByteTrack config. The long track_buffer in it is '
                        'what keeps a doll on one track id across a brief '
                        'occlusion; the geotag is what covers the rest.'),
        DeclareLaunchArgument(
            'doll_venv',
            default_value='/home/ark-jetson-orin-2/venvs/dolls/lib/'
                          'python3.12/site-packages',
            description='site-packages of the venv holding torch and '
                        'ultralytics, prepended to PYTHONPATH for the doll '
                        'node only. See the comment on doll_node. A '
                        'non-existent path is ignored by Python.'),
        DeclareLaunchArgument('doll_confidence', default_value='0.55'),
        DeclareLaunchArgument(
            'doll_min_frames', default_value='5',
            description='Frames a track must survive before it can be '
                        'counted. Kills flickery false positives.'),
        DeclareLaunchArgument(
            'doll_max_fps', default_value='6.0',
            description='Inference rate cap. The thing that must not be '
                        'starved on this Jetson is the flight node\'s 20 Hz '
                        'setpoint timer, and a doll does not move.'),
        DeclareLaunchArgument('doll_image_topic', default_value='/image_raw',
                              description='The downward C920 (usb_cam).'),
        DeclareLaunchArgument('doll_camera_info_topic', default_value='/camera_info'),
        DeclareLaunchArgument(
            'doll_depth_source', default_value='rangefinder',
            description='rangefinder: z-depth = camera height (TFmini minus '
                        'the lens offset) minus doll_target_height.'),
        DeclareLaunchArgument(
            'doll_target_height', default_value='0.10',
            description='m. Height above the FLOOR of the part of the doll the '
                        'detector box centres on (about half its height as it '
                        'lies/stands). Only scales range: 5 cm wrong at 1.75 m '
                        'moves a doll 1 m off-centre by ~3 cm.'),
        DeclareLaunchArgument(
            'doll_hfov_deg', default_value='70.4',
            description='C920 horizontal FOV, used when its CameraInfo is '
                        'uncalibrated (usb_cam without a calibration file).'),
        DeclareLaunchArgument(
            'doll_geotag_frame', default_value='arena',
            description='arena = tag in the lidar room frame (/lidar/odom_kf); '
                        'ned = EKF2 local frame.'),
        DeclareLaunchArgument('doll_cam_x', default_value='0.0'),
        DeclareLaunchArgument('doll_cam_y', default_value='0.0'),
        DeclareLaunchArgument(
            'doll_cam_z', default_value='-0.075',
            description='m, ROS z-up: the C920 is 7-8 cm BELOW the body centre.'),
        DeclareLaunchArgument(
            'doll_cam_pitch', default_value='1.5708',
            description='+90 deg = optical axis straight down, image-up = nose.'),
        DeclareLaunchArgument(
            'doll_merge_radius', default_value='0.20',
            description='m. Two detections this close together IN THE ROOM '
                        'are the same doll. Smaller than the smallest gap '
                        'between two real dolls, bigger than the position '
                        'error. See item 4 in the header.'),
        DeclareLaunchArgument('doll_depth_min', default_value='0.30'),
        DeclareLaunchArgument('doll_depth_max', default_value='8.0'),
        DeclareLaunchArgument(
            'doll_depth_patch', default_value='0.30',
            description='Fraction of the box the depth median is taken over, '
                        'centred. The middle of a doll is doll; the edges are '
                        'the floor behind it.'),
        DeclareLaunchArgument('doll_min_depth_pixels', default_value='12'),
        DeclareLaunchArgument(
            'doll_publish_image', default_value='false',
            description='Publish the annotated frames on /doll_image. Costs '
                        'the CPU margin the setpoint timer needs; for the '
                        'bench, not for a flight.'),
        DeclareLaunchArgument(
            'require_enable', default_value='true',
            description='true = the model only runs between the inbound '
                        'commit and the outbound clear, which is what the '
                        'flight node publishes on /doll_detect_enable. false '
                        '= run it always, which is how you bench it.'),

        support,
        GroupAction([flight_node, doll_node], condition=IfCondition(flight)),
        display_node,
        qgc_node,
    ])
