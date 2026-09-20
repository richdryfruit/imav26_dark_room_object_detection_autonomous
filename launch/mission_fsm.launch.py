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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
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
    # SCOPED, and this is not cosmetic.
    #
    # IncludeLaunchDescription does NOT push a scope. Its `launch_arguments`
    # are implemented as bare SetLaunchConfiguration actions emitted into the
    # SHARED context, so pinning agent_only:='true' for the include also
    # rewrites agent_only for THIS file -- and the flight node below, which is
    # visited after the include and gated UnlessCondition(agent_only), then
    # reads 'true' and is silently skipped. You pass agent_only:=false, the
    # support stack comes up, and nothing ever flies, with no error printed.
    #
    # GroupAction(scoped=True, forwarding=True) pushes the configurations
    # before the include and pops them after: everything declared here is
    # still visible to the include (forwarding), but the include's own sets
    # die with the group.
    #
    # NOTE: window_room_traverse.launch.py has this same bug against
    # window_traverse.launch.py -- `support` is listed before its flight node,
    # which is also UnlessCondition(agent_only) -- so `agent_only:=false` does
    # not start a flight node there either. It goes unnoticed because the
    # documented workflow is to run the flight node by hand in a second pane.
    support = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('drone_testing'),
                'launch', 'window_room_traverse.launch.py'])),
            launch_arguments={
                'agent_only': 'true',
            }.items(),
        ),
    ], scoped=True, forwarding=True)

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

    # ---- the doll count, in this terminal ---------------------------------
    #
    # doll_count_gui is the OTHER display and is deliberately not started
    # here: it is a curses application, so it owns a terminal and needs a real
    # tty, and launch hands its processes a pipe. Run it yourself in a second
    # pane when you want the big number:
    #
    #     ros2 run drone_testing doll_count_gui
    #
    # Both read /doll_count and apply the same default offset, so the two
    # never disagree.
    doll_text_node = Node(
        package='drone_testing',
        executable='doll_report_text',
        name='doll_report_text',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'offset': LaunchConfiguration('doll_offset'),
            'period': LaunchConfiguration('doll_report_period'),
        }],
        condition=IfCondition(LaunchConfiguration('doll_text')),
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
                    # ---- the altitude schedule ----
                    'cruise_altitude': LaunchConfiguration('cruise_altitude'),
                    'window_altitude_m': LaunchConfiguration('window_altitude_m'),
                    'room_altitude': LaunchConfiguration('room_altitude'),
                    'alt_change_timeout': LaunchConfiguration('alt_change_timeout'),
                    # ---- the window search failsafe ----
                    'window_search_seconds': LaunchConfiguration(
                        'window_search_seconds'),
                    'window_sweep_deg': LaunchConfiguration('window_sweep_deg'),
                    'window_backoff': LaunchConfiguration('window_backoff'),
                    'window_max_backoffs': LaunchConfiguration(
                        'window_max_backoffs'),
                    # ---- the missed-marker failsafe ----
                    'marker_retry_altitude': LaunchConfiguration(
                        'marker_retry_altitude'),
                    'marker_retry_seconds': LaunchConfiguration(
                        'marker_retry_seconds'),
                    'marker_retry_distance': LaunchConfiguration(
                        'marker_retry_distance'),
                    'marker_max_retries': LaunchConfiguration(
                        'marker_max_retries'),
                    # ---- the markers ----
                    'window_marker_id': LaunchConfiguration('window_marker_id'),
                    'turn_marker_id': LaunchConfiguration('turn_marker_id'),
                    'pad_marker_id': LaunchConfiguration('pad_marker_id'),
                    # ---- the four marker-terminated legs ----
                    'outbound_distance': LaunchConfiguration('outbound_distance'),
                    'window_offset': LaunchConfiguration('window_offset'),
                    'window_offset_direction': LaunchConfiguration(
                        'window_offset_direction'),
                    'return_distance': LaunchConfiguration('return_distance'),
                    'turn_distance': LaunchConfiguration('turn_distance'),
                    'pad_distance': LaunchConfiguration('pad_distance'),
                    'leg_speed': LaunchConfiguration('leg_speed'),
                    'leg_settle_seconds': LaunchConfiguration('leg_settle_seconds'),
                    'leg_latch_timeout': LaunchConfiguration('leg_latch_timeout'),
                    'leg_timeout_margin': LaunchConfiguration('leg_timeout_margin'),
                    # ---- centring on a marker ----
                    'marker_tolerance': LaunchConfiguration('marker_tolerance'),
                    'marker_settle_seconds': LaunchConfiguration(
                        'marker_settle_seconds'),
                    'marker_gain': LaunchConfiguration('marker_gain'),
                    'marker_hold_seconds': LaunchConfiguration('marker_hold_seconds'),
                    'marker_align_timeout': LaunchConfiguration(
                        'marker_align_timeout'),
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

        # ---- THE ALTITUDE SCHEDULE ----
        #
        # Two heights, and the whole flight is at one or the other. NOTE that
        # takeoff_altitude is IGNORED by this node: the climb goes to
        # cruise_altitude, because there is one height for the marker phases
        # and this is it.
        DeclareLaunchArgument(
            'cruise_altitude', default_value='2.50',
            description='m for the climb, every leg and the landing. The down '
                        'camera basket is +/- h*tan(39 deg), so 2.50 m gives '
                        'about +/-2.0 m of capture width for the markers.'),
        DeclareLaunchArgument(
            'window_altitude_m', default_value='1.75',
            description='m dropped to on the FIRST sighting of the window '
                        'marker, and climbed back out of on the second. The '
                        'dark room is NOT on this schedule -- its heights come '
                        'from the measured window pose via window_altitude().'),
        DeclareLaunchArgument(
            'room_altitude', default_value='1.75',
            description='m the dark-room box pattern is flown at. NOT the '
                        'traverse height -- the traversal is lined up on the '
                        'measured window pose and goes through wherever the '
                        'aperture is. This is the height it levels off at '
                        'once inside.'),
        DeclareLaunchArgument('alt_change_timeout', default_value='25.0',
                              description='s before a climb or descent gives '
                                          'up and carries on from where it '
                                          'got to. The height is a preference; '
                                          'the mission outranks it.'),

        # ---- THE WINDOW SEARCH FAILSAFE ----
        #
        # Normally the strafe puts the aircraft on the axis and the window is
        # simply there. This is the ladder for when it is not.
        DeclareLaunchArgument(
            'window_search_seconds', default_value='12.0',
            description='s of looking before climbing a rung of the failsafe.'),
        DeclareLaunchArgument(
            'window_sweep_deg', default_value='30.0',
            description='deg either side for rung 1 -- yaw left, centre, '
                        'right. This is SCAN\'s own sweep, which '
                        'window_traverse disables by setting the span to zero.'),
        DeclareLaunchArgument(
            'window_backoff', default_value='0.30',
            description='m straight back per rung after the sweep. Seeing '
                        'NOTHING (as opposed to a truncated quad, which is '
                        'RECENTRE\'s job) usually means standing too close '
                        'for the aperture to fit the frame -- about 1.26 x '
                        'the window height is needed. Yawing cannot fix that; '
                        'backing off is the only thing that can. The traversal '
                        'needs NO compensation for it: approach_points() '
                        'builds both ends from the window pose, so backing off '
                        'moves where the approach starts, not where it ends.'),
        DeclareLaunchArgument(
            'window_max_backoffs', default_value='2',
            description='rungs of backoff before the attempt is abandoned. '
                        'Two is enough because the aperture already fits from '
                        'the marker: a 0.60 m window needs 0.76 m of depth '
                        'and the marker stands 1.00 m from the wall.'),

        # ---- THE MISSED-MARKER FAILSAFE ----
        #
        # A leg that runs out of distance without its marker has almost
        # certainly drifted -- ten metres on optical flow with nothing
        # correcting it. The down camera basket is +/- h*tan(39 deg), so the
        # cheapest fix is height:
        #
        #     1.75 m -> +/-1.42 m    2.50 m -> +/-2.02 m    3.50 m -> +/-2.83 m
        #
        # Runs before any per-leg fallback, so it covers the course legs and
        # the pad leg alike. It matters most for the pad: without it one
        # missed detection is a landing off the pad and nothing left to try.
        DeclareLaunchArgument(
            'marker_retry_altitude', default_value='3.50',
            description='m climbed to for a second look. The MAXIMUM height '
                        'this mission flies at -- keep max_altitude above it.'),
        DeclareLaunchArgument('marker_retry_seconds', default_value='8.0',
                              description='s hovering at that height before '
                                          'retracing.'),
        DeclareLaunchArgument(
            'marker_retry_distance', default_value='4.0',
            description='m retraced BACK along the leg at the retry altitude, '
                        'still watching. Backwards because a leg that ran out '
                        'of distance overshot or drifted, so the marker is '
                        'behind where it stopped. Bounded rather than the '
                        'whole leg: a full retrace costs the flight clock '
                        'twice over.'),
        DeclareLaunchArgument(
            'marker_max_retries', default_value='0',
            description='Elevated searches per leg. ZERO by default: a leg '
                        'that reaches its limit without its marker STOPS '
                        'THERE and moves on -- the roll stops at '
                        'turn_distance, and the pad leg lands. Set to 1 to '
                        're-enable the climb-and-look failsafe described by '
                        'the three arguments above, which is built and '
                        'tested but off.'),

        # ---- THE MARKERS ----
        #
        # id 0 is the takeoff pad and is not used by this node. These three
        # MUST differ -- the flight node refuses to construct otherwise, and
        # the reason is in its header: the left strafe looks for the turn
        # marker while the window marker is still under the camera, so a
        # shared id ends that leg before it starts.
        DeclareLaunchArgument(
            'window_marker_id', default_value='2',
            description='ArUco id of the marker in front of the window. '
                        'Ends the outbound leg AND the return leg.'),
        DeclareLaunchArgument(
            'turn_marker_id', default_value='3',
            description='ArUco id of the marker the LEFT strafe ends on.'),
        DeclareLaunchArgument(
            'pad_marker_id', default_value='1',
            description='ArUco id of the landing pad. The only required '
                        'marker: without it the aircraft lands off the pad.'),

        # ---- THE FOUR MARKER-TERMINATED LEGS ----
        #
        # Each distance is a LIMIT, not a target. A leg ends when its marker
        # appears; the distance is how far it will go before giving up and
        # holding. Set them a little LONGER than the real spacing, so flow
        # error cannot stop the aircraft short of a marker it would have seen.
        DeclareLaunchArgument(
            'outbound_distance', default_value='10.0',
            description='m FORWARD from the arming point, looking for '
                        'window_marker_id. If it never appears the leg ends '
                        'here and the WINDOW SEARCH starts anyway -- the '
                        'window is the real landmark and the marker only '
                        'refines the approach to it.'),
        DeclareLaunchArgument(
            'window_offset', default_value='0.25',
            description='m sideways from the window marker to the window\'s '
                        'CENTRE-LINE. MEASURE IT. The aircraft strafes this '
                        'far onto the axis before the sweep, and mirrors it '
                        'on the way out -- the marker is outside the down '
                        'camera footprint (about +/-0.95 m at 1.2 m) from the '
                        'axis, so without the mirror the return leg never '
                        'sees it. 0.0 disables both strafes and approaches '
                        'obliquely from the marker. MEASURED: 0.25 m.'),
        DeclareLaunchArgument(
            'window_offset_direction', default_value='left',
            description='left | right, in the TAKEOFF frame. Which way the '
                        'window centre-line lies from its marker. The mirror '
                        'strafe on the way out is the opposite of this.'),
        DeclareLaunchArgument(
            'return_distance', default_value='10.0',
            description='m BACKWARD from the window, looking for '
                        'window_marker_id again. Backward because directions '
                        'are in the TAKEOFF frame: the airframe is facing '
                        'back down the course after the room, but that is a '
                        'heading, not a frame, and forward here would fly it '
                        'straight back into the room. Only ~3 m is ever '
                        'flown; the rest is margin.'),
        DeclareLaunchArgument(
            'turn_distance', default_value='5.4',
            description='m LEFT (takeoff frame) from the window marker, '
                        'looking for '
                        'turn_marker_id. MEASURE THIS ONE -- it is the only '
                        'leg with no natural 11 m to fall back on.'),
        DeclareLaunchArgument(
            'pad_distance', default_value='10.0',
            description='m BACKWARD (takeoff frame) from the turn marker, '
                        'looking for pad_marker_id. This lands the aircraft '
                        'level with the takeoff pad and turn_distance to its '
                        'left.'),
        DeclareLaunchArgument(
            'leg_speed', default_value='0.30',
            description='m/s on a leg. Slow on purpose: the down camera has '
                        'to have time to see a marker pass beneath it, and '
                        'flow is the only thing measuring these translations.'),
        DeclareLaunchArgument('leg_settle_seconds', default_value='2.0',
                              description='s of settling at the end of a leg '
                                          'that ran out of distance.'),
        DeclareLaunchArgument('leg_latch_timeout', default_value='20.0',
                              description='s waiting for a flow-healthy x/y '
                                          'latch before a leg is given up.'),
        DeclareLaunchArgument('leg_timeout_margin', default_value='30.0',
                              description='s allowed on top of distance/speed '
                                          'before a leg is called stuck.'),

        # ---- THE DOLL COUNT IN THIS TERMINAL ----
        DeclareLaunchArgument(
            'doll_text', default_value='true',
            description='Print the geotagged count and doll positions as log '
                        'lines. Works under launch, unlike doll_count_gui, '
                        'which is curses and needs a tty (second pane).'),
        DeclareLaunchArgument(
            'doll_offset', default_value='2',
            description='Display-side subtraction, the same default '
                        'doll_count_gui uses so the two agree. The raw count '
                        'is printed alongside and nothing upstream changes.'),
        DeclareLaunchArgument('doll_report_period', default_value='10.0',
                              description='s between heartbeat prints.'),

        # ---- CENTRING ON A MARKER ----
        # ---- THE LANDING PAD ----
        DeclareLaunchArgument(
            'pad_detect', default_value='true',
            description='Start aruco_pose (the down-facing camera). false if '
                        'you are running it by hand.'),
        DeclareLaunchArgument(
            'marker_tolerance', default_value='0.15',
            description='m radius that counts as being over the marker.'),
        DeclareLaunchArgument('marker_settle_seconds', default_value='1.5',
                              description='s inside the radius before it is '
                                          'believed. One sample inside 15 cm '
                                          'is corner noise, not an arrival.'),
        DeclareLaunchArgument(
            'marker_gain', default_value='0.6',
            description='Fraction of the measured offset commanded per cycle. '
                        'Must be in (0, 1]: below 1 the loop is monotone, '
                        'above 1 it overshoots by design.'),
        DeclareLaunchArgument('marker_hold_seconds', default_value='5.0',
                              description='s station keeping over the marker '
                                          'before the descent commits.'),
        DeclareLaunchArgument('marker_align_timeout', default_value='45.0',
                              description='s trying to centre before giving up.'),
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
        DeclareLaunchArgument('max_altitude', default_value='4.0',
                              description='m ceiling. Raised above the 3.0 m default because\n                                          '
                                          'the missed-marker failsafe deliberately climbs to\n                                          '
                                          'marker_retry_altitude (3.50 m).'),
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
        doll_text_node,
        flight_node,
    ])
