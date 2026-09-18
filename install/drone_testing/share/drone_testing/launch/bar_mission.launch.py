"""
The obstacle-course BAR, flown as its own mission. ARK Flow localisation.

    uXRCE-DDS agent + zed_wrapper (CAMERA ONLY)
    + bar_detect (detection AND geometry) + bar_cross (the flight)

    arm -> climb -> hold -> find the bar -> measure how high it is -> climb or
    descend onto the crossing altitude -> fly across it -> hold -> land.

Read window_traverse.launch.py first. Everything it says about the
localisation stack, the PX4 parameters, MPC_LAND_SPEED, the CPU budget and
why there is no external vision here applies unchanged and is not repeated.

RED BAR (over) AND BLUE BARS (under) ARE THE SAME MISSION

    They differ in a colour and a sign, so they are one node and one launch
    file. The default here is the red bar, flown OVER and MEASURED:

        ros2 launch drone_testing bar_mission.launch.py

    The red bar flown BLIND at the 1600 mm setting, which is what I would
    actually fly -- see BLIND OR MEASURED below:

        ros2 launch drone_testing bar_mission.launch.py \\
            assume_bar_height:=1.6 assume_bar_distance:=3.0

    The blue bar at the 800 mm setting, flown UNDER and measured:

        ros2 launch drone_testing bar_mission.launch.py \\
            color:=blue pass_mode:=under max_bar_height:=1.4

    Nothing else changes. Do not fork this file for the blue bar.

BLIND OR MEASURED

    assume_bar_height decides it. The default is 0 -- measured -- because
    flying blind should be a decision you typed, not one you inherited from a
    default. But for the RED bar, blind is very likely what you want:

    the rules publish the height (1200 / 1600 / 1980 mm red, 400 / 800 / 1200
    blue), so there is nothing for the camera to discover about it. Making a
    vision measurement the gate on a number you were given in advance only
    adds a way to fail -- and on an arena whose floor is the same colour as
    the red bar, a fairly likely one.

    Blind skips SEARCH and LOCK: climb, hold, then fly the geometry from
    assume_bar_height and assume_bar_distance. bar_detect still runs and
    still publishes; the flight just is not waiting on it. What it sees comes
    out in the log as a CROSS-CHECK line against the assumed height, which is
    the cheapest way to discover that assume_bar_height is set to the wrong
    rules setting -- without having had to trust it in the air.

    Set assume_bar_height:=0.0 to measure instead.

    The two are not equally safe, and the asymmetry is what should decide
    which obstacle uses which:

        OVER  -- assuming 30 cm too low means flying 30 cm higher than
                 necessary. Nothing happens.
        UNDER -- assuming 30 cm too high means hitting the bar.

    So: blind is the right default for the red bar. For the blue bars it is a
    deliberate choice, and if you make it, measure the distance and the
    setting carefully first.

THE 400 mm BLUE BAR IS NOT FLYABLE ON THIS STACK

    Worth knowing before the arena rather than in it. Going under, the
    commanded altitude is bar_height - bar_radius - cross_clearance -
    body_above. With the standard mounting (body_above = 0.10 m):

        1200 mm  ->  0.85 m   comfortable
         800 mm  ->  0.45 m   tight, but above the flow floor
         400 mm  ->  0.05 m   impossible

    Even at zero clearance the 400 mm setting needs 0.25 m, and FLOW_MIN_AGL
    is 0.30: below that EKF2 is not fusing optical flow and there is no
    horizontal estimate at all. Flying under a bar on dead reckoning is not a
    thing to attempt. 800 mm is the aggressive-but-real target; if you want
    more margin there, cross_clearance:=0.10 puts it at 0.55 m.

THE FLOOR IS THE SAME COLOUR AS THE RED BAR

    This is the whole problem with the red obstacle, and it is worth being
    explicit about because it will look like a detector bug the first time you
    watch the overlay.

    The arena floor is red between the textured strips. bar_detect WILL draw
    boxes around floor strips -- from one image a long red horizontal region
    is a long red horizontal region, and no amount of HSV tuning distinguishes
    them. The floor is rejected one layer later, in bar_cross, by height:
    every candidate is placed in NED using the vehicle attitude, and anything
    less than min_bar_height above the arming plane is refused as floor.

    So the thing to watch on the bench is NOT "does the overlay ever box the
    floor" -- it will. It is:

        ros2 topic echo /bar_pose

    which is empty until an accepted, gated, above-the-floor bar exists, and
    then reports x|y|height|angle|length|samples|age. And the rejection tally
    in bar_cross's shutdown line, where "too low to be the bar (floor?)"
    counting into the hundreds while the aircraft sits on the pad is the
    system working, not failing.

    If you can raise min_bar_height, do. The rules put the bar at 1200, 1600
    or 1980 mm, so 0.60 m is already a factor of two of margin at the lowest
    setting; there is no reason to run it tighter and every reason not to.

BEFORE THE FIRST FLIGHT

1. BENCH THE DETECTOR, ON THE ARENA FLOOR. Hold the airframe at hover height
   pointing at the bar, with real floor in the bottom of the frame:

        ros2 launch drone_testing bar_mission.launch.py flight:=false
        ros2 topic echo /bar_detected
        ros2 topic echo /bar_geometry
        # or point a browser at http://<jetson>:8081/

   You want /bar_detected true when the bar is in view and the reported
   depth to match a tape measure.

2. THEN BENCH THE HEIGHT TEST. With the flight node running but not armed,
   carry the airframe around and watch /bar_pose. The height should read the
   bar's real height above where you started, and should NOT change as you
   walk. That is the test that catches a wrong cam_pitch, and it is the same
   test that the window mission needed.

3. MEASURE THE CAMERA MOUNTING. cam_x/cam_y/cam_z and cam_roll/cam_pitch/
   cam_yaw are the pose of the camera in the body frame, ROS convention
   (x forward, y LEFT, z UP). They are the same numbers window_traverse
   takes. A wrong cam_pitch shows up here as a bar height that changes when
   the aircraft pitches, which is exactly the error that matters.

4. CHECK max_altitude AGAINST THE BAR. At the 1980 mm setting the crossing
   altitude is about 2.35 m, so max_altitude must be comfortably above that
   or the node abandons the attempt rather than flying at the ceiling. The
   default is 3.0.

WHAT TO RUN

Bench, no props, camera side only:

    ros2 launch drone_testing bar_mission.launch.py flight:=false

Flight, with the keyboard aborts (a node started by launch has no tty, so
q/k are dead there -- run the flight node yourself in a second pane):

    ros2 launch drone_testing bar_mission.launch.py
    ros2 run drone_testing bar_cross --ros-args \\
        -p takeoff_altitude:=1.2 -p cam_pitch:=0.0 -p cam_x:=0.10

Everything in one shot, no keyboard abort (the RC kill switch still works):

    ros2 launch drone_testing bar_mission.launch.py agent_only:=false
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
    flight = LaunchConfiguration('flight')
    agent_only = LaunchConfiguration('agent_only')

    camera_mounting = {
        'cam_x': LaunchConfiguration('cam_x'),
        'cam_y': LaunchConfiguration('cam_y'),
        'cam_z': LaunchConfiguration('cam_z'),
        'cam_roll': LaunchConfiguration('cam_roll'),
        'cam_pitch': LaunchConfiguration('cam_pitch'),
        'cam_yaw': LaunchConfiguration('cam_yaw'),
    }

    microxrce_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_xrce_dds_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
    )

    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'camera_name': LaunchConfiguration('camera_name'),
            # PX4 is the navigation authority; the ZED must not publish a
            # competing odom -> base_link edge.
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('zed')),
    )

    detect_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_testing',
                executable='bar_detect',
                name='bar_detect',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'image_topic': LaunchConfiguration('image_topic'),
                    'depth_topic': LaunchConfiguration('depth_topic'),
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'color': LaunchConfiguration('color'),
                    'min_area': LaunchConfiguration('min_area'),
                    'min_aspect': LaunchConfiguration('min_aspect'),
                    'max_tilt_deg': LaunchConfiguration('max_tilt_deg'),
                    'min_elevation_deg': LaunchConfiguration('min_elevation_deg'),
                    'samples_along': LaunchConfiguration('samples_along'),
                    'border_margin': LaunchConfiguration('border_margin'),
                    'detect_frames': LaunchConfiguration('detect_frames'),
                    'lost_frames': LaunchConfiguration('lost_frames'),
                    'max_fps': LaunchConfiguration('max_fps'),
                    'fallback_hfov_deg': LaunchConfiguration('fallback_hfov_deg'),
                    'publish_image': LaunchConfiguration('publish_image'),
                    'publish_mask': LaunchConfiguration('publish_mask'),
                    'publish_compressed': LaunchConfiguration('publish_compressed'),
                    'stream_port': LaunchConfiguration('stream_port'),
                    'stream_scale': LaunchConfiguration('stream_scale'),
                    'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('detect')),
    )

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

    cross_node = TimerAction(
        period=LaunchConfiguration('flight_node_delay'),
        actions=[
            Node(
                package='drone_testing',
                executable='bar_cross',
                name='bar_cross',
                output='screen',
                emulate_tty=True,
                parameters=[dict(camera_mounting, **{
                    # ---- the climb (inherited from OffboardSequence) ----
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'yaw_rate': LaunchConfiguration('yaw_rate'),
                    'takeoff_return_to_pad': LaunchConfiguration('takeoff_return_to_pad'),
                    'min_altitude': LaunchConfiguration('min_altitude'),
                    'max_altitude': LaunchConfiguration('max_altitude'),
                    'yaw_cone_deg': LaunchConfiguration('yaw_cone_deg'),
                    'flight_seconds': LaunchConfiguration('flight_seconds'),

                    # ---- the crossing ----
                    'pass_mode': LaunchConfiguration('pass_mode'),
                    'assume_bar_height': LaunchConfiguration('assume_bar_height'),
                    'assume_bar_distance': LaunchConfiguration('assume_bar_distance'),
                    'assume_bar_length': LaunchConfiguration('assume_bar_length'),
                    'standoff_distance': LaunchConfiguration('standoff_distance'),
                    'exit_distance': LaunchConfiguration('exit_distance'),
                    'cross_clearance': LaunchConfiguration('cross_clearance'),
                    'bar_radius': LaunchConfiguration('bar_radius'),
                    'approach_speed': LaunchConfiguration('approach_speed'),
                    'cross_speed': LaunchConfiguration('cross_speed'),
                    'set_alt_tolerance': LaunchConfiguration('set_alt_tolerance'),
                    'align_cross_tolerance': LaunchConfiguration('align_cross_tolerance'),
                    'align_along_tolerance': LaunchConfiguration('align_along_tolerance'),
                    'end_margin': LaunchConfiguration('end_margin'),

                    # ---- the airframe ----
                    'gear_below_camera': LaunchConfiguration('gear_below_camera'),
                    'drone_height': LaunchConfiguration('drone_height'),
                    'drone_width': LaunchConfiguration('drone_width'),

                    # ---- the estimate ----
                    'min_bar_height': LaunchConfiguration('min_bar_height'),
                    'max_bar_height': LaunchConfiguration('max_bar_height'),
                    'min_length': LaunchConfiguration('min_length'),
                    'max_length': LaunchConfiguration('max_length'),
                    'max_slope_deg': LaunchConfiguration('max_slope_deg'),
                    'depth_min': LaunchConfiguration('depth_min'),
                    'depth_max': LaunchConfiguration('depth_max'),
                    'pose_min_samples': LaunchConfiguration('pose_min_samples'),
                    'pose_lost_timeout': LaunchConfiguration('pose_lost_timeout'),
                    'buffer_seconds': LaunchConfiguration('buffer_seconds'),
                    'gate_metres': LaunchConfiguration('gate_metres'),
                    'gate_yaw_deg': LaunchConfiguration('gate_yaw_deg'),
                    'detect_seconds': LaunchConfiguration('detect_seconds'),
                })],
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
            description='Start the support stack but not the flight node, so '
                        'you can run that by hand and keep the q/k keyboard '
                        'aborts. false = fly the whole thing from this launch '
                        'file. Same meaning and same default as in '
                        'window_traverse.launch.py -- these must not differ.'),
        DeclareLaunchArgument(
            'flight', default_value='true',
            description='false = camera side only: no DDS agent and no flight '
                        'node. This is the bench test.'),
        DeclareLaunchArgument('detect', default_value='true'),
        DeclareLaunchArgument(
            'zed', default_value='true',
            description='Start zed_wrapper. false if it is already running.'),

        # ---- camera ----
        DeclareLaunchArgument('camera_model', default_value='zed'),
        DeclareLaunchArgument('camera_name', default_value='zed'),
        DeclareLaunchArgument(
            'image_topic', default_value='/zed/zed_node/rgb/color/rect/image'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/zed/zed_node/depth/depth_registered',
            description='Depth registered to image_topic, 32FC1 in metres.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='auto',
            description='"auto" takes the sibling of image_topic, which is '
                        'where image_transport always puts it. Without real '
                        'CameraInfo every angle -- and so every bar HEIGHT -- '
                        'is scaled by a guessed field of view.'),
        DeclareLaunchArgument('fallback_hfov_deg', default_value='90.0'),

        # ---- camera mounting (the same numbers window_traverse takes) ----
        DeclareLaunchArgument(
            'cam_x', default_value='0.105',
            description='Metres the camera sits FORWARD of the CoG.'),
        DeclareLaunchArgument(
            'cam_y', default_value='0.0',
            description='Metres the camera sits to the LEFT of the CoG.'),
        DeclareLaunchArgument(
            'cam_z', default_value='-0.04',
            description='Metres the camera sits ABOVE the CoG.'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument(
            'cam_pitch', default_value='0.0',
            description='Radians, POSITIVE = camera pointed DOWN. Get this '
                        'wrong and the measured bar height changes as the '
                        'aircraft pitches, which is the one error that turns '
                        'a clean crossing into a collision.'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),

        # ---- the detector ----
        DeclareLaunchArgument(
            'color', default_value='red',
            description='red for the first bar (flown OVER), blue for the '
                        'pair after it (flown UNDER).'),
        DeclareLaunchArgument(
            'min_area', default_value='800.0',
            description='px^2. Smaller than the window detector uses: a bar '
                        'several metres away is a thin sliver, and it is the '
                        'shape gate that keeps the noise out, not area.'),
        DeclareLaunchArgument(
            'min_aspect', default_value='4.0',
            description='Long side / short side of the fitted rectangle. A '
                        'bar is long and thin. Raise it if blobs of floor are '
                        'getting as far as the height test; lower it if a bar '
                        'seen nearly end-on is being missed.'),
        DeclareLaunchArgument(
            'max_tilt_deg', default_value='25.0',
            description='How far the bar may lie off horizontal IN THE IMAGE. '
                        'Generous, because the aircraft rolls a few degrees '
                        'while translating and the bar rolls with it.'),
        DeclareLaunchArgument(
            'min_elevation_deg', default_value='-20.0',
            description='Cheap floor prefilter: candidates whose centre is '
                        'further below the optical axis than this are '
                        'discarded without measuring. Kept permissive on '
                        'purpose -- the test that actually separates floor '
                        'from bar is min_bar_height, which needs the '
                        'attitude and therefore lives in the flight node.'),
        DeclareLaunchArgument('samples_along', default_value='7'),
        DeclareLaunchArgument('border_margin', default_value='12.0'),
        DeclareLaunchArgument('detect_frames', default_value='3'),
        DeclareLaunchArgument('lost_frames', default_value='5'),
        DeclareLaunchArgument('max_fps', default_value='10.0'),
        DeclareLaunchArgument('publish_image', default_value='true'),
        DeclareLaunchArgument('publish_mask', default_value='false'),
        DeclareLaunchArgument('publish_compressed', default_value='true'),
        DeclareLaunchArgument(
            'stream_port', default_value='8081',
            description='MJPEG debug stream. NOT 8080 -- window_detect owns '
                        'that, and both may be up during a full course run.'),
        DeclareLaunchArgument('stream_scale', default_value='0.5'),
        DeclareLaunchArgument('jpeg_quality', default_value='60'),

        # ---- the climb ----
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='1.2',
            description='m above the arming point. Only has to get the camera '
                        'looking at the bar; the crossing then flies at '
                        'whatever height the bar turns out to be.'),
        DeclareLaunchArgument('hold_seconds', default_value='5.0'),
        DeclareLaunchArgument('ground_wait_seconds', default_value='5.0'),
        DeclareLaunchArgument('climb_speed', default_value='0.35'),
        DeclareLaunchArgument(
            'land_speed', default_value='0.15',
            description='Keep MPC_LAND_SPEED at about 0.2 so PX4 agrees this '
                        'is a descent.'),
        DeclareLaunchArgument('yaw_rate', default_value='0.35'),
        DeclareLaunchArgument(
            'takeoff_return_to_pad', default_value='false',
            description='See window_traverse.launch.py. Leave it false: the '
                        'ground x/y estimate is not trustworthy and flying '
                        'back to it is what makes a takeoff look like it '
                        'pitched over and went backwards.'),
        DeclareLaunchArgument('min_altitude', default_value='0.4'),
        DeclareLaunchArgument(
            'max_altitude', default_value='3.0',
            description='m above the arming point, and a HARD clamp on the '
                        'crossing altitude. The 1980 mm bar needs about 2.35 '
                        'm, so this must stay well above that or the node '
                        'abandons rather than flying at the ceiling.'),
        DeclareLaunchArgument(
            'yaw_cone_deg', default_value='50.0',
            description='Hard limit on how far the nose may turn from the '
                        'arming heading. Point the aircraft at the bar before '
                        'you arm and this never engages.'),
        DeclareLaunchArgument(
            'flight_seconds', default_value='120.0',
            description='Seconds from the START OF THE CLIMB to the descent. '
                        'Never fires during the crossing itself.'),

        # ---- the crossing ----
        DeclareLaunchArgument(
            'pass_mode', default_value='over',
            description='over = the red bar, under = the blue ones. This is '
                        'the ONLY logical difference between the two '
                        'missions; do not fork the node for it.'),
        DeclareLaunchArgument(
            'assume_bar_height', default_value='0.0',
            description='FLY BLIND at this bar height, in metres, instead of '
                        'measuring it. 0 (the default) = measure it with the '
                        'camera.\n'
                        'The default is 0 for two reasons: flying blind is a '
                        'decision that should be typed rather than inherited, '
                        'and it keeps this node behaving the same whether it '
                        'is started from here or with a bare `ros2 run`. Opt '
                        'in explicitly: assume_bar_height:=1.6\n'
                        'The rules publish the height -- 1.2 / 1.6 / 1.98 for '
                        'the red bar, 0.4 / 0.8 / 1.2 for the blue -- so for '
                        'the red bar there is nothing for the camera to '
                        'discover, and making a vision measurement the gate '
                        'on a number you were given in advance only adds a '
                        'way to fail. SEARCH and LOCK are skipped; bar_detect '
                        'still runs and logs a CROSS-CHECK line comparing '
                        'what it sees with what was assumed.\n'
                        'The default 1.6 is the middle red setting. Set it to '
                        'the setting you actually chose. Note the asymmetry '
                        'before using this on the blue bars: assuming wrong '
                        'going OVER costs you some altitude, assuming wrong '
                        'going UNDER hits the bar.'),
        DeclareLaunchArgument(
            'assume_bar_distance', default_value='3.0',
            description='m ahead of the aircraft the assumed bar sits, along '
                        'the takeoff heading. Measured from where the '
                        'aircraft IS at the end of the hold, not from the '
                        'arming point -- the ground x/y estimate is not '
                        'trustworthy and the climb may have drifted. Only '
                        'used when assume_bar_height > 0.'),
        DeclareLaunchArgument(
            'assume_bar_length', default_value='3.0',
            description='m. Only feeds the end-margin check on a blind '
                        'crossing. Set it to the real bar length if it is '
                        'short enough that crossing near an end is a risk.'),
        DeclareLaunchArgument(
            'standoff_distance', default_value='1.20',
            description='m short of the bar the crossing starts from.'),
        DeclareLaunchArgument(
            'exit_distance', default_value='1.20',
            description='m past the bar the crossing ends.'),
        DeclareLaunchArgument(
            'cross_clearance', default_value='0.20',
            description='m of air between the airframe and the bar. Over a '
                        'bar there is only sky on the other side, so this is '
                        'nearly free -- be generous. Going UNDER, it is the '
                        'gap between the top of the aircraft and the bar, and '
                        'the floor is what limits how much you can spend.'),
        DeclareLaunchArgument(
            'bar_radius', default_value='0.05',
            description='m, half the bar thickness. The estimate measures the '
                        'centreline of what the mask saw, so the surface is '
                        'this much nearer than the measurement.'),
        DeclareLaunchArgument('approach_speed', default_value='0.30'),
        DeclareLaunchArgument('cross_speed', default_value='0.40'),
        DeclareLaunchArgument('set_alt_tolerance', default_value='0.08'),
        DeclareLaunchArgument(
            'align_cross_tolerance', default_value='0.20',
            description='m off the crossing centreline. Looser than the '
                        "window's, because a bar has no jambs -- being off to "
                        'one side costs nothing as long as it is not off an '
                        'END, which end_margin covers.'),
        DeclareLaunchArgument('align_along_tolerance', default_value='0.30'),
        DeclareLaunchArgument(
            'end_margin', default_value='0.35',
            description='m of bar wanted either side of the airframe. The '
                        'crossing goes through the measured CENTRE, so this '
                        'is really a check that the bar was seen end to end.'),

        # ---- the airframe ----
        DeclareLaunchArgument('gear_below_camera', default_value='0.120'),
        DeclareLaunchArgument('drone_height', default_value='0.260'),
        DeclareLaunchArgument('drone_width', default_value='0.260'),

        # ---- the estimate ----
        DeclareLaunchArgument(
            'min_bar_height', default_value='0.60',
            description='m above the arming plane. THE gate that separates '
                        'the bar from the red floor: the floor is at zero by '
                        'construction and cannot pass. The rules put the bar '
                        'at 1.2 m at the lowest, so this has a factor of two '
                        'of margin. Raise it, never lower it.'),
        DeclareLaunchArgument(
            'max_bar_height', default_value='2.60',
            description='m. Covers the 1980 mm setting with room for '
                        'measurement error. Raise for a blue bar hung high.'),
        DeclareLaunchArgument('min_length', default_value='0.60'),
        DeclareLaunchArgument('max_length', default_value='6.00'),
        DeclareLaunchArgument(
            'max_slope_deg', default_value='20.0',
            description='How far the reconstructed bar may be off level in '
                        'NED. This is the second floor gate: a floor strip '
                        'seen in perspective has its near end metres closer '
                        'than its far end, so once both ends are in NED it '
                        'climbs away at a slope no real bar has.'),
        DeclareLaunchArgument('depth_min', default_value='0.35'),
        DeclareLaunchArgument('depth_max', default_value='9.0'),
        DeclareLaunchArgument('pose_min_samples', default_value='6'),
        DeclareLaunchArgument('pose_lost_timeout', default_value='8.0'),
        DeclareLaunchArgument('buffer_seconds', default_value='2.5'),
        DeclareLaunchArgument('gate_metres', default_value='1.0'),
        DeclareLaunchArgument('gate_yaw_deg', default_value='35.0'),
        DeclareLaunchArgument('detect_seconds', default_value='0.4'),

        # ---- the rest ----
        DeclareLaunchArgument('reboot_fc', default_value='false'),
        DeclareLaunchArgument('flight_node_delay', default_value='12.0'),
        DeclareLaunchArgument('lcd', default_value='false'),
        DeclareLaunchArgument('lcd_port', default_value=''),

        GroupAction([microxrce_node, lcd_node, reboot_node, cross_node],
                    condition=IfCondition(flight)),
        zed_wrapper,
        detect_node,
    ])
