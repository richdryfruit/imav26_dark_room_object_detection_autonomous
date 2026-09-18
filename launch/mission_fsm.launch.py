"""
THE WHOLE RUN, FROM ONE COMMAND.

    ros2 launch drone_testing mission_fsm.launch.py agent_only:=false \
        outbound_sequence:="forward 3.0, right 1.5" \
        pad_sequence:="backward 2.0, left 4.0"

    arm -> climb -> OUTBOUND leg -> window -> dark room (dolls counted and
    shown) -> back out -> TRANSIT leg -> ArUco pad -> precision landing.

ONE FLIGHT, NOT FIVE
--------------------
Everything above happens in a SINGLE armed Offboard session. The aircraft
does not touch the ground between segments, and that is a safety property,
not a flourish: README section 10.1 records that `cs_rng_kin_consistent` is
sticky -- EKF2 only updates it while `in_air` is true, so once it latches
false nothing on the ground clears it but a flight controller reboot. A
mission that lands and re-arms four times is a mission that can refuse to
take off again with the clock running. See the mission_fsm.py header.

WHAT THIS STARTS, AND WHERE IT COMES FROM
-----------------------------------------
The support stack is NOT re-implemented here. This file INCLUDES
window_room_traverse.launch.py with agent_only:=true, so that file brings up
exactly what it always brings up --

    the uXRCE-DDS agent, the RealSense D435i, the emitter parameter, the
    window detector, the doll model (gated on /doll_detect_enable), the
    room_display TFT bridge and the QGC doll status stream

-- and does NOT bring up its own flight node. Every camera, detector, window,
room and doll argument below is forwarded into that include, so there is one
definition of each and the two files cannot drift apart.

What this file adds is two nodes:

    aruco_pose   the DOWN-facing camera and the landing marker. A separate
                 USB camera from the RealSense the window uses, so the two
                 detectors never contend. Its arguments are prefixed `pad_`
                 because `detect`, `width`, `height` and `camera_fps` are
                 already taken by the RealSense stack in the include, and a
                 forwarded name would land in both.
    mission_fsm  the flight. A subclass of window_room_traverse: same climb,
                 same sweep, same lock, same traversals, same room pattern,
                 plus the two navigation legs and the pad approach.

THE ORDER OF THE ARGUMENTS BELOW
---------------------------------
The mission-specific ones first (the two legs and the pad), because those are
the ones you actually set on the day. Everything after them is the room
mission's own argument list, forwarded unchanged.

BEFORE YOU FLY THIS
-------------------
Fly the pieces first. This file is the last rung of the ladder, not the
first:

    1. `precision_land.launch.py flight:=false` with `mode:=bench` -- the
       axis sign check on the down-facing camera. A sign error here does not
       wobble, it flies the aircraft off the pad and accelerates.
    2. `window_room_traverse.launch.py agent_only:=false` -- the dark room
       mission on its own, in the actual room.
    3. this file with `outbound_sequence:=""` and `pad_sequence:=""`, which
       degrades it to exactly (2) plus the pad landing.
    4. this file with the real legs.

MEASURE THE LEGS. They are flown on optical flow alone, which is the least
referenced part of this flight; `pad_sequence` in particular has to put the
aircraft over the marker, because the pad search does not sweep -- the
capture basket is just the down-camera footprint at that altitude.

agent_only defaults to true, as in every other launch file here, so the
support stack comes up from launch and you run the flight node by hand in a
second pane. That keeps stdin a tty, which is the only way the q / k aborts
work. Your RC kill switch is the real safety net regardless.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')

    camera_mounting = {
        'cam_x': LaunchConfiguration('cam_x'),
        'cam_y': LaunchConfiguration('cam_y'),
        'cam_z': LaunchConfiguration('cam_z'),
        'cam_roll': LaunchConfiguration('cam_roll'),
        'cam_pitch': LaunchConfiguration('cam_pitch'),
        'cam_yaw': LaunchConfiguration('cam_yaw'),
    }

    # ---- the support stack, the doll model and the displays ---------------
    #
    # agent_only is PINNED true: this include must bring up everything except
    # a flight node, because the flight node it would start is the one this
    # file is replacing. Forwarding is left on (the default) so every window,
    # room, camera and doll argument declared below reaches it without being
    # listed twice.
    support = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('drone_testing'),
            'launch', 'window_room_traverse.launch.py'])),
        launch_arguments={
            'agent_only': 'true',
        }.items(),
    )

    # ---- the landing marker ------------------------------------------------
    #
    # Started well ahead of the flight node so the camera has opened and
    # /aruco/detected is already being published by the time anything asks it
    # a question -- and, more to the point, long before the aircraft arrives
    # over the pad several minutes later.
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
                    'camera_index': LaunchConfiguration('pad_camera_index'),
                    'width': LaunchConfiguration('pad_width'),
                    'height': LaunchConfiguration('pad_height'),
                    'fourcc': LaunchConfiguration('pad_fourcc'),
                    'camera_fps': LaunchConfiguration('pad_camera_fps'),
                    'marker_id': LaunchConfiguration('pad_marker_id'),
                    'marker_size': LaunchConfiguration('pad_marker_size'),
                    'aruco_dict': LaunchConfiguration('pad_aruco_dict'),
                    'hfov_deg': LaunchConfiguration('pad_hfov_deg'),
                    'image_rotate': LaunchConfiguration('pad_image_rotate'),
                    'detect_rate': LaunchConfiguration('pad_detect_rate'),
                    'stream_port': LaunchConfiguration('pad_stream_port'),
                    'stream_scale': LaunchConfiguration('pad_stream_scale'),
                    'jpeg_quality': LaunchConfiguration('pad_jpeg_quality'),
                    'show_gui': LaunchConfiguration('pad_show_gui'),
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('pad_detect')),
    )

    # ---- the flight -------------------------------------------------------
    flight_node = TimerAction(
        period=LaunchConfiguration('flight_node_delay'),
        actions=[
            Node(
                package='drone_testing',
                executable='mission_fsm',
                name='mission_fsm',
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
                    # ---- the two navigation legs ----
                    'outbound_sequence': LaunchConfiguration('outbound_sequence'),
                    'pad_sequence': LaunchConfiguration('pad_sequence'),
                    'leg_speed': LaunchConfiguration('leg_speed'),
                    'leg_hold_seconds': LaunchConfiguration('leg_hold_seconds'),
                    # ---- the landing pad ----
                    'pad_align_tolerance': LaunchConfiguration('pad_align_tolerance'),
                    'pad_align_settle_seconds': LaunchConfiguration(
                        'pad_align_settle_seconds'),
                    'pad_align_gain': LaunchConfiguration('pad_align_gain'),
                    'pad_hold_seconds': LaunchConfiguration('pad_hold_seconds'),
                    'pad_search_seconds': LaunchConfiguration('pad_search_seconds'),
                    'pad_align_timeout': LaunchConfiguration('pad_align_timeout'),
                    'pad_on_fail': LaunchConfiguration('pad_on_fail'),
                    'marker_max_age': LaunchConfiguration('marker_max_age'),
                    'marker_lost_seconds': LaunchConfiguration('marker_lost_seconds'),
                    'precision_descent': LaunchConfiguration('precision_descent'),
                    'blind_commit_altitude': LaunchConfiguration(
                        'blind_commit_altitude'),
                })],
            )
        ],
        condition=UnlessCondition(agent_only),
    )

    return LaunchDescription([
        # ---- what to start ----
        DeclareLaunchArgument(
            'agent_only', default_value='true',
            description='Start the support stack but not the flight node, so '
                        'it can be run by hand and keep the q/k keyboard '
                        'aborts. false = fly the whole mission from here.'),

        # ---- THE TWO NAVIGATION LEGS ----
        #
        # Body-frame, relative to the yaw held since arming, in exactly the
        # grammar offboard_sequence parses. Empty skips the leg, which is how
        # this file degrades into plain window_room_traverse plus a pad
        # landing for a rehearsal.
        DeclareLaunchArgument(
            'outbound_sequence', default_value='',
            description='Arming point -> in front of the window, e.g. '
                        '"forward 3.0, right 1.5, yaw 15". The window sweep '
                        'centres its yaw cone on the heading it STARTS from, '
                        'so this leg has to finish facing the window wall. '
                        'forward/backward/left/right/yaw only; up/down are '
                        'rejected (use altitude_offset). Empty = skip, and '
                        'sweep from the arming point.'),
        DeclareLaunchArgument(
            'pad_sequence', default_value='',
            description='Outside the window -> OVER the landing pad, e.g. '
                        '"backward 2.0, left 4.0". Must put the marker inside '
                        'the down-camera footprint: the pad search does not '
                        'sweep. Empty = skip, and look for the marker from '
                        'wherever the outbound traversal ended.'),
        DeclareLaunchArgument(
            'leg_speed', default_value='0.30',
            description='m/s the carrot is walked at on a leg. Keep it slow: '
                        'these are the longest translations in the flight and '
                        'optical flow is the only thing measuring them.'),
        DeclareLaunchArgument(
            'leg_hold_seconds', default_value='3.0',
            description='s of settling at the end of a leg, before the window '
                        'sweep or the marker search starts.'),

        # ---- THE LANDING PAD ----
        DeclareLaunchArgument(
            'pad_detect', default_value='true',
            description='Start aruco_pose (the down-facing camera). false if '
                        'you are running it by hand.'),
        DeclareLaunchArgument(
            'pad_align_tolerance', default_value='0.15',
            description='m radius that counts as being over the marker.'),
        DeclareLaunchArgument('pad_align_settle_seconds', default_value='1.5',
                              description='s inside the radius before it is '
                                          'believed. One sample inside 15 cm '
                                          'is corner noise, not an arrival.'),
        DeclareLaunchArgument(
            'pad_align_gain', default_value='0.6',
            description='Fraction of the measured offset commanded per cycle. '
                        'Must be in (0, 1]: below 1 the loop is monotone, '
                        'above 1 it overshoots by design.'),
        DeclareLaunchArgument('pad_hold_seconds', default_value='5.0',
                              description='s station keeping over the marker '
                                          'before the descent commits.'),
        DeclareLaunchArgument('pad_search_seconds', default_value='25.0',
                              description='s looking for the marker before '
                                          'giving up.'),
        DeclareLaunchArgument('pad_align_timeout', default_value='45.0',
                              description='s trying to centre before giving up.'),
        DeclareLaunchArgument(
            'pad_on_fail', default_value='land',
            description="land | hold. What a pad timeout does. 'land' means "
                        'land here, OFF the pad, which beats hovering until '
                        'the battery decides where to land for you.'),
        DeclareLaunchArgument('marker_max_age', default_value='0.5',
                              description='s. Older than this reads as "no '
                                          'marker": a dead camera goes quiet, '
                                          'and quiet must never read as a lock.'),
        DeclareLaunchArgument('marker_lost_seconds', default_value='5.0',
                              description='s without a marker mid-align '
                                          'before falling back to the search.'),
        DeclareLaunchArgument(
            'precision_descent', default_value='true',
            description='Hold the aligned x/y point all the way down instead '
                        'of dropping to zero-velocity hold. false reverts to '
                        'the generic descent.'),
        DeclareLaunchArgument(
            'blind_commit_altitude', default_value='1.0',
            description='m below which the marker is expected to leave the '
                        'frame and the descent is open loop. MEASURE IT: walk '
                        'the airframe down over the marker and note where '
                        '/aruco/detected goes false.'),

        # ---- the down-facing camera (aruco_pose) ----
        #
        # `pad_` prefixed throughout: the bare names collide with the
        # RealSense arguments in the include, and a forwarded name would land
        # in both cameras at once.
        DeclareLaunchArgument('pad_camera_index', default_value='0',
                              description='cv2.VideoCapture index of the '
                                          'down-facing camera.'),
        DeclareLaunchArgument('pad_width', default_value='800',
                              description='4:3 on purpose: the taller vertical '
                                          'FOV keeps the marker in frame ~20 cm '
                                          'lower than 16:9 does.'),
        DeclareLaunchArgument('pad_height', default_value='600'),
        DeclareLaunchArgument('pad_fourcc', default_value='MJPG',
                              description='MJPG gets 30 fps at 800x600 on this '
                                          'camera; YUYV does not.'),
        DeclareLaunchArgument('pad_camera_fps', default_value='30.0'),
        DeclareLaunchArgument('pad_detect_rate', default_value='20.0',
                              description='Hz the newest frame is processed at.'),
        DeclareLaunchArgument('pad_marker_id', default_value='0'),
        DeclareLaunchArgument('pad_marker_size', default_value='0.80',
                              description='Marker edge length in metres. Must '
                                          'be right: it sets the metric scale '
                                          'of the whole pose.'),
        DeclareLaunchArgument('pad_aruco_dict', default_value='DICT_5X5_50'),
        DeclareLaunchArgument('pad_hfov_deg', default_value='78.0'),
        DeclareLaunchArgument('pad_image_rotate', default_value='0',
                              description='0|90|180|270, applied before '
                                          'detection. Use this if the camera '
                                          'is bolted on rotated, so that '
                                          'image-up still means the nose.'),
        DeclareLaunchArgument('pad_stream_port', default_value='8080',
                              description='Browser MJPEG view: '
                                          'http://<jetson-ip>:8080/. 0 off.'),
        DeclareLaunchArgument('pad_stream_scale', default_value='0.6'),
        DeclareLaunchArgument('pad_jpeg_quality', default_value='70'),
        DeclareLaunchArgument('pad_show_gui', default_value='false',
                              description='Needs a display; leave false on a '
                                          'headless Jetson.'),

        # ---- everything below is the room mission's own argument list, ----
        # ---- forwarded into the include unchanged.                     ----
        DeclareLaunchArgument(
            'flight', default_value='true',
            description='false = camera side only: no DDS agent, no flight '
                        'node, no doll node. The bench test.'),
        DeclareLaunchArgument('detect', default_value='true',
                              description='Start window_detect.'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Start realsense2_camera. false if '
                                          'it is already running.'),
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
            'flight_seconds', default_value='420.0',
            description='Hard clock from the start of the climb. TWICE the '
                        'single-traversal default: two approaches, two '
                        'traversals and a six-leg pattern do not fit in 150 s.'),
        DeclareLaunchArgument('request_offboard_from_ros', default_value='true'),
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
        DeclareLaunchArgument(
            'doll_merge_radius', default_value='0.60',
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
        aruco_node,
        flight_node,
    ])
