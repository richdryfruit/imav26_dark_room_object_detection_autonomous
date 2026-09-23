"""
Window traversal on ARK Flow localisation.

    window_detect (detection AND geometry) + window_traverse (the flight)

    The uXRCE-DDS agent and the RealSense D435i (colour + aligned depth, via
    the VIO stack) are NOT started here: imav_bringup (bringup.launch.py)
    owns them. Bring that up first.

    arm -> climb -> hold -> sweep for the window -> lock and build a pose
    estimate -> turn onto the window normal -> fly to a point in front of it
    -> fly through it -> hold on the far side -> land.

This is window_scan.launch.py's mission continued past the lock, flown on
the localisation stack from sequence_test.launch.py: PMW3901 optical flow
plus a Benewake TFmini Plus rangefinder plus the IMU, fused in PX4. Read BOTH
of those files first -- everything they say about PX4 parameters and about
MPC_LAND_SPEED applies here unchanged and is not repeated.

PORTED FROM ZED + ARK FLOW TO REALSENSE D435i + PMW3901 + TFmini PLUS.
The camera half of that is topics and field of view and lives in
window_detect.py and in the camera topics below. The sensor half is a PX4
parameter exercise and touches NO ROS code at all, because every horizontal
gate in the inherited OffboardSequence is written against EKF2's own flags,
never against a sensor topic. What it does need on the flight controller:

    SENS_EN_PMW3901 = 1     the flow driver (SPI). EKF2_OF_CTRL = 1.
    SENS_EN_TFMINI  = 1     the lidar on its serial port, and
    SENS_TFMINI_CFG = <port>. EKF2_RNG_CTRL = 1, EKF2_HGT_REF = 2.
    EKF2_OF_QMIN            the PMW3901 reports a coarser quality number than
                            the ARK Flow's PAW3902 did; if the flow never
                            latches, this is the first parameter to look at.
    EKF2_EV_CTRL    = 0     no external vision. See below.

THE ONE THING THE ARK FLOW HAD THAT THE PMW3901 DOES NOT is an integrated
rangefinder. The TFmini Plus is now a separate device on its own port, so it
is a separate way for the flight to be un-armable: rangefinder_is_healthy()
is a hard arming gate and it does not care why the lidar is quiet. Check it
on the ground, every session, exactly as section 11 of the README says.

EXTERNAL VISION IS NOT THIS FILE'S BUSINESS. The D435i frames used here are
for window_detect to find and measure the window. Position from the camera,
if any, comes from the VIO stack imav_bringup starts (RTAB-Map ->
vio_to_px4_bridge), and whether EKF2 fuses it is EKF2_EV_CTRL on the flight
controller -- nothing in this file changes either.

Why: on this airframe the ZED's visual odometry was making EKF2 reset its
horizontal position six to seven times a second, continuously, even sitting
still and disarmed. PX4 only enforces position validity once armed, so that
surfaced as local_position_invalid about a second after arming and looked
like an Offboard problem. tools/ekf_reset_rate.py measures it in 25 s without
arming; a healthy vehicle reads ~0.00 resets/s. sequence_vio_realsense.launch.py
and offboard_sequence_vio.py hold the vision path.

WHAT TO RUN

Bench, no props: the detector only, no flight node. This is
where you check that the geometry topic is alive and sane before anything
spins, and you can do it holding the airframe in your hands:

    ros2 launch drone_testing window_traverse.launch.py flight:=false

    ros2 topic echo /window_detected
    ros2 topic echo /window_geometry      # 15 floats: 5 points x (d, az, el)
    ros2 run rqt_image_view rqt_image_view /window_detection/image

Flight. The default starts everything EXCEPT the flight node, so you run
that in a second pane and keep the q/k keyboard aborts (a node started by
launch has no tty, so those keys are dead):

    ros2 launch drone_testing window_traverse.launch.py
    ros2 run drone_testing window_traverse --ros-args \
        -p takeoff_altitude:=1.2 -p standoff_distance:=1.6 \
        -p cam_pitch:=0.0 -p cam_x:=0.10

    ros2 topic echo /window_pose          # x|y|z|yaw|w|h|samples|age, NED

Everything in one shot, no keyboard abort (the RC kill switch still works,
and it is the one that matters):

    ros2 launch drone_testing window_traverse.launch.py agent_only:=false

BEFORE THE FIRST FLIGHT

1. MEASURE THE CAMERA MOUNTING. cam_x/cam_y/cam_z and
   cam_roll/cam_pitch/cam_yaw below go to window_traverse, which uses them to
   turn a pixel into a point in front of the VEHICLE rather than in front of
   the lens. Get them wrong and the window is estimated in the wrong place by
   exactly the offset you failed to measure. They are the pose of the camera
   in the body frame in the ROS convention: x forward, y LEFT, z UP, metres
   and radians, and the defaults ("at the CoG, pointing straight forward")
   are almost certainly not yours. Only one node reads them now, so unlike
   the VIO version there is nothing here that can disagree with itself.

2. WATCH THE CPU. The failure this stack dies of on a Jetson is not a flight
   bug: it is the companion falling behind, PX4 seeing a late setpoint and
   taking the aircraft back with offboard_control_signal_lost about a second
   after arming. The detection pipeline is capped at max_fps (10 Hz) and the
   flight node now runs its setpoint timer on a thread of its own, but if you
   turn publish_image and the MJPEG stream on for a real flight you are
   spending that margin. Bench first with:

        ros2 launch drone_testing window_traverse.launch.py \
            publish_image:=false stream_port:=0

3. CHECK THE FLOW, NOT THE VISION FLAGS. The gate that matters here is
   cs_opt_flow, and the estimate is not trustworthy on the ground: optical
   flow needs ground texture and a rangefinder reading above FLOW_MIN_AGL, so
   the node holds zero velocity until it has been healthy for a few seconds
   after the climb and only then latches an x/y point. That is the inherited
   behaviour from sequence_test.launch.py and it is why the status line reads
   VEL-HOLD on the ground and POS-HOLD in the air.

        ros2 topic echo /fmu/out/estimator_status_flags --once | grep cs_opt_flow
        python3 src/drone_testing/tools/ekf_reset_rate.py   # want ~0.00/s

   The magnetometer stays ON for this configuration (EKF2_MAG_TYPE=0): with
   no external vision there is no other absolute heading source, and without
   one EKF2 never aligns yaw and PX4 takes the aircraft shortly after arming.
   That is README 10.3, and it is the trap in the opposite direction from the
   VIO version.

4. WALK THE ESTIMATE BEFORE YOU FLY IT. With the props off, carry the
   airframe to where you expect it to hover and watch /window_pose. The
   centre should sit still in NED to within a few centimetres while you move
   the airframe around -- that is the whole point of the estimate being in
   NED rather than in the camera frame, and it is the one test that catches a
   wrong cam_pitch before it costs you an airframe.

5. MEASURE THE WINDOW. /window_pose reports the width and height the
   estimator reconstructs. If they do not match a tape measure, the
   intrinsics or the depth scale are wrong, and every distance in the
   approach is wrong by the same factor.

GEOMETRY OF THE APPROACH

    standoff_distance   where the aircraft lines up: this far in front of the
                        window plane, ON THE WINDOW'S AXIS, at the window's
                        height, nose along the normal. Bigger is a longer,
                        better-conditioned look at the window; smaller is a
                        shorter run through. 1.6 m keeps a 1 m window fully in
                        frame on a 90 degree lens.
    exit_distance       how far past the window plane the run ends.
    approach_speed      m/s for the line-up.
    traverse_speed      m/s for the run through. The estimate is frozen by
                        then, so this is only limited by how much control
                        authority you want in the aperture.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')
    flight = LaunchConfiguration('flight')

    # The mounting pose of the camera. Only window_traverse reads it now that
    # there is no VIO bridge -- see item 1 in the header.
    camera_mounting = {
        'cam_x': LaunchConfiguration('cam_x'),
        'cam_y': LaunchConfiguration('cam_y'),
        'cam_z': LaunchConfiguration('cam_z'),
        'cam_roll': LaunchConfiguration('cam_roll'),
        'cam_pitch': LaunchConfiguration('cam_pitch'),
        'cam_yaw': LaunchConfiguration('cam_yaw'),
    }

    # Detection and geometry. Same node and the same delay as window_scan; the
    # only addition is camera_info_topic, which is what makes /window_geometry
    # metric rather than a guess from an assumed field of view.
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
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'publish_geometry': True,
                    'show_windows': LaunchConfiguration('show_windows'),
                    'publish_image': LaunchConfiguration('publish_image'),
                    'publish_mask': LaunchConfiguration('publish_mask'),
                    'color': LaunchConfiguration('color'),
                    'min_area': LaunchConfiguration('min_area'),
                    'border_margin': LaunchConfiguration('border_margin'),
                    'max_fps': LaunchConfiguration('max_fps'),
                    'use_depth': LaunchConfiguration('use_depth'),
                    'depth_scale': LaunchConfiguration('depth_scale'),
                    'fallback_hfov_deg': LaunchConfiguration('fallback_hfov_deg'),
                    'detect_frames': LaunchConfiguration('detect_frames'),
                    'lost_frames': LaunchConfiguration('lost_frames'),
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

    traverse_node = TimerAction(
        period=LaunchConfiguration('flight_node_delay'),
        actions=[
            Node(
                package='drone_testing',
                executable='window_traverse',
                name='window_traverse',
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
                    # ---- the traversal ----
                    'standoff_distance': LaunchConfiguration('standoff_distance'),
                    'exit_distance': LaunchConfiguration('exit_distance'),
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
            description='Start the support stack but not the flight node, so you '
                        'can run that by hand and keep the q/k keyboard aborts. '
                        'false = fly the whole thing from this launch file.'),
        DeclareLaunchArgument(
            'flight', default_value='true',
            description='false = detector only: no flight node. This is the '
                        'bench test.'),
        DeclareLaunchArgument(
            'detect', default_value='true',
            description='Start window_detect.'),
        # ---- camera topics. The RealSense itself (colour + aligned depth,
        # emitter, filters) is started by imav_bringup's VIO stack. ----
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/camera/color/image_raw',
            description='Colour image. rgb8 on the D435i.'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='Depth registered to image_topic, 16UC1 in '
                        'MILLIMETRES on the RealSense (the ZED published '
                        '32FC1 metres; window_detect handles both). This must '
                        'be the aligned_depth_to_color topic, not '
                        'depth/image_rect_raw -- the latter is in the depth '
                        'imager\'s own frame and sampling window corners out '
                        'of it reads the wrong part of the scene.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='auto',
            description='CameraInfo for image_topic. "auto" (the default) '
                        'takes the sibling of image_topic, which is where '
                        'image_transport always puts it -- so the two cannot '
                        'drift apart. This is where the geometry gets its '
                        'intrinsics; without it the node falls back to a '
                        'guessed field of view and every angle, and therefore '
                        'every window size and position, is scaled wrong. It '
                        'says so in the log once a second if it does.'),
        DeclareLaunchArgument('show_windows', default_value='false'),
        DeclareLaunchArgument('publish_image', default_value='true'),
        DeclareLaunchArgument('publish_mask', default_value='false'),
        DeclareLaunchArgument(
            'color', default_value='blue',
            description='HSV range to look for: green, blue or red.'),
        DeclareLaunchArgument(
            'border_margin', default_value='12.0',
            description='px. A detected quad with a corner closer than this to '
                        'the image edge is reported TRUNCATED, and the traversal '
                        'node refuses to measure the window from it -- the quad '
                        'is the visible PART of the window, so its centre is not '
                        'the window centre and its width is not the window '
                        'width. Flying at that centre is what put a prop into a '
                        'window frame. Raise it if the detector flags windows '
                        'that are plainly complete; lower it only if you are '
                        'certain the aperture can never leave the frame.'),
        DeclareLaunchArgument(
            'min_area', default_value='1500.0',
            description='px^2, smallest contour taken seriously. Too high and a '
                        'window first seen from across the room is ignored until '
                        'the aircraft is nearly on top of it.'),
        DeclareLaunchArgument(
            'use_depth', default_value='true',
            description='false disables the depth path entirely. /window_geometry '
                        'needs depth, so the traversal CANNOT run without it -- '
                        'this is for debugging the colour detection alone.'),
        DeclareLaunchArgument(
            'depth_scale', default_value='1.0',
            description='Multiplier applied AFTER the depth image has been '
                        'converted to metres, for display only. Leave at 1.0. '
                        'The RealSense 16UC1 millimetre -> metre conversion is '
                        'NOT this: window_detect does that from the message '
                        'encoding, so a D435i needs no change here. 100.0 if '
                        'you want the centimetres the old standalone script '
                        'printed.'),
        DeclareLaunchArgument(
            'fallback_hfov_deg', default_value='69.0',
            description='Horizontal FOV assumed ONLY while no CameraInfo has '
                        'arrived on camera_info_topic. It is a guess and it '
                        'scales every angle in /window_geometry; if you see the '
                        'node warn that it is using this, fix the topic name '
                        'rather than tuning this number. CHANGED 90 -> 69 for '
                        'the D435i: that is its COLOUR sensor (measured 70.4 '
                        'on this unit), not the 87 deg depth/infra pair.'),
        DeclareLaunchArgument(
            'detect_frames', default_value='3',
            description='Consecutive hits before the detection latches true. '
                        'Debounce against a single frame of noise.'),
        DeclareLaunchArgument(
            'lost_frames', default_value='5',
            description='Consecutive misses before it latches false. Higher '
                        'rides out a brief occlusion; too high and the flight '
                        'node keeps flying at a window that is gone.'),
        DeclareLaunchArgument(
            'publish_compressed', default_value='true',
            description='false drops the compressed image topic. Worth turning '
                        'off with publish_image for a real flight -- it is CPU '
                        'spent on something nobody is watching.'),
        DeclareLaunchArgument(
            'max_fps', default_value='10.0',
            description='Cap on how often the detection pipeline runs. It is '
                        'the biggest CPU consumer on the Jetson and the '
                        'aircraft closes at 0.3-0.45 m/s, so 10 Hz loses '
                        'nothing the estimator can use and leaves the CPU the '
                        'offboard heartbeat needs. 0 = every frame.'),
        DeclareLaunchArgument('stream_port', default_value='8080'),
        DeclareLaunchArgument('stream_scale', default_value='0.5'),
        DeclareLaunchArgument('jpeg_quality', default_value='60'),

        # ---- camera mounting: pose of the camera IN THE BODY FRAME, ROS
        # convention (x fwd, y LEFT, z UP), metres and radians. Fed to BOTH the
        # window_traverse -- see item 1 in the header. MEASURE THESE.
        DeclareLaunchArgument(
            'cam_x', default_value='0.105',
            description='Metres the camera sits FORWARD of the CoG.'),
        DeclareLaunchArgument(
            'cam_y', default_value='0.0',
            description='Metres the camera sits to the LEFT of the CoG.'),
        DeclareLaunchArgument(
            'cam_z', default_value='-0.04',
            description='Metres the camera sits ABOVE the CoG.'),
        DeclareLaunchArgument(
            'cam_roll', default_value='0.0',
            description='Radians, positive = right side down.'),
        DeclareLaunchArgument(
            'cam_pitch', default_value='0.0',
            description='Radians, POSITIVE = camera pointed DOWN. A window is '
                        'looked at level, so this is usually 0 here even if the '
                        'airframe carries the camera tilted for landing.'),
        DeclareLaunchArgument(
            'cam_yaw', default_value='0.0',
            description='Radians, positive = camera pointed to the LEFT.'),

        # ---- the climb and the sweep ----
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='1.2',
            description='m above the arming point. This only has to get the '
                        'camera looking at the window; the traverse then flies '
                        'at the window centre height, whatever that turns out '
                        'to be, clamped into min/max_altitude.'),
        DeclareLaunchArgument('hold_seconds', default_value='5.0'),
        DeclareLaunchArgument('ground_wait_seconds', default_value='5.0'),
        DeclareLaunchArgument('climb_speed', default_value='0.35'),
        DeclareLaunchArgument(
            'land_speed', default_value='0.15',
            description='Keep MPC_LAND_SPEED at about 0.2 so PX4 agrees this is '
                        'a descent.'),
        DeclareLaunchArgument(
            'yaw_rate', default_value='0.35',
            description='rad/s the yaw setpoint is walked at, for the sweep and '
                        'for the turn onto the window normal. Keep it slow: a '
                        'fast yaw is the most reliable way to make an IMU-less '
                        'stereo camera lose tracking.'),
        DeclareLaunchArgument(
            'takeoff_return_to_pad', default_value='false',
            description='After the climb anchors on optical flow, fly back to '
                        'the x/y captured at arming. FALSE by default: the '
                        'ground x/y estimate is not trustworthy (no flow '
                        'below FLOW_MIN_AGL, and it drifts through the ground '
                        'wait), so flying to it is a translation to a number '
                        'of unknown quality -- and it happens right after the '
                        'climb, which is what makes a takeoff look like it '
                        'pitched over and went backwards. Leave it false '
                        'unless you specifically need the aircraft back over '
                        'the pad; the drift is logged either way.'),
        DeclareLaunchArgument('min_altitude', default_value='0.4'),
        DeclareLaunchArgument('max_altitude', default_value='3.0'),
        DeclareLaunchArgument(
            'scan_span_deg', default_value='20.0',
            description='Total width of the yaw sweep about the takeoff '
                        'heading. 0 (the default) means NO SWEEP AT ALL: the '
                        'vehicle climbs, holds, and then just stares straight '
                        'ahead on its takeoff heading until the detector calls '
                        'a window, so point it at the window before you arm. '
                        'Set it to e.g. 20 (= 10 deg either side) to bring the '
                        'sweep back; keep it narrow, a wide sweep swings the '
                        'airframe far off heading and smears the optical '
                        'flow.'),
        DeclareLaunchArgument(
            'scan_yaw_rate', default_value='0.05',
            description='rad/s the yaw setpoint is walked at DURING THE SWEEP '
                        'only (~3 deg/s). Deliberately slower than yaw_rate: a '
                        'fast sweep smears the optical flow and can cross a '
                        'window in fewer frames than the detector needs to '
                        'call it. Restored to yaw_rate once the window is '
                        'locked, so the approach is not slowed down.'),
        DeclareLaunchArgument(
            'yaw_cone_deg', default_value='50.0',
            description='HARD LIMIT on how far the nose may turn from the '
                        'heading the aircraft armed on, in degrees either '
                        'side. Nothing in the flight commands a yaw outside '
                        'it, and a window estimate whose bearing is outside '
                        'it is refused rather than flown at -- a "window" 90 '
                        'or 180 degrees off the takeoff heading is a '
                        'reflection, a doorway behind the aircraft, or a bad '
                        'pose, never the one you pointed it at. Widen it only '
                        'if the window really is that far round. 0 disables '
                        'the check.'),
        DeclareLaunchArgument(
            'scan_direction', default_value='right',
            description="Which way the first half-leg of the sweep turns, "
                        "'right' or 'left'. Point it at the side the window is "
                        'expected on so the usual case is found in a few '
                        'degrees instead of after crossing the far half of the '
                        'arc.'),
        DeclareLaunchArgument('detect_seconds', default_value='0.4'),
        DeclareLaunchArgument(
            'relock_on_loss', default_value='false',
            description='true = go back to sweeping if the detection is lost '
                        'while still locked. Has no effect once the approach '
                        'has started.'),
        DeclareLaunchArgument(
            'flight_seconds', default_value='150.0',
            description='Hard limit from the START OF THE CLIMB to the descent. '
                        'Fires from every stage EXCEPT the traverse itself -- '
                        'the aircraft is never landed from inside a window.'),
        DeclareLaunchArgument('request_offboard_from_ros', default_value='false'),

        # ---- the traversal ----
        DeclareLaunchArgument(
            'standoff_distance', default_value='2.0',
            description='m in front of the window plane the approach lines up '
                        'on, along the window normal. RAISED 1.6 -> 2.0 for '
                        'the D435i: at 43 deg of VERTICAL field of view the '
                        '"whole window still in frame" distance is 1.26x the '
                        'window height, against 0.87x on the ZED, so 1.6 m '
                        'had no margin left for a 1.2 m aperture. The node '
                        'also pushes this out further at run time if the '
                        'aperture it measures needs it -- see '
                        'effective_standoff() in window_traverse.py.'),
        DeclareLaunchArgument(
            'exit_distance', default_value='1.5',
            description='m beyond the window plane the run through ends.'),
        DeclareLaunchArgument(
            'altitude_offset', default_value='0.0',
            description='m added to the estimated window centre height. Positive '
                        'is higher. Use it if the detected quad sits off-centre '
                        'on the real aperture.'),
        DeclareLaunchArgument(
            'gear_below_camera', default_value='0.120',
            description='m from the camera down to the bottom of the landing '
                        'gear. With cam_z this is what tells the traverse how '
                        'far the airframe hangs below the point PX4 flies -- '
                        'get it wrong low and the gear catches the sill.'),
        DeclareLaunchArgument(
            'drone_height', default_value='0.260',
            description='m, landing gear bottom to the highest point.'),
        DeclareLaunchArgument(
            'drone_width', default_value='0.260',
            description='m across the widest point.'),
        DeclareLaunchArgument(
            'vertical_clearance', default_value='0.150',
            description='m of air wanted between the gear and the sill, and '
                        'between the top and the lintel. The traverse height '
                        'is solved for this; a window too small to give it '
                        'gets a warning and the sill is favoured.'),
        DeclareLaunchArgument(
            'lateral_clearance', default_value='0.150',
            description='m wanted either side. Advisory only -- it warns, '
                        'nothing steers off it.'),
        DeclareLaunchArgument(
            'hard_clearance', default_value='0.030',
            description='m. An aperture leaving less than this around the '
                        'airframe is abandoned rather than flown.'),
        DeclareLaunchArgument(
            'sill_bias', default_value='0.100',
            description='m of extra height above the airframe-centred '
                        'solution, spent only if the aperture affords it. '
                        'Buys margin against altitude sag at the sill, where '
                        'a strike flips the aircraft.'),
        DeclareLaunchArgument(
            'align_alt_tolerance', default_value='0.08',
            description='m of altitude error tolerated before committing to '
                        'the traverse.'),
        DeclareLaunchArgument('approach_speed', default_value='0.30'),
        DeclareLaunchArgument(
            'traverse_speed', default_value='0.45',
            description='m/s through the window. The estimate is frozen by then.'),
        DeclareLaunchArgument(
            'align_tolerance', default_value='0.18',
            description='m radius around the approach point that counts as '
                        'lined up. Tighten it for a small window, but every '
                        'centimetre costs settling time against VO noise.'),
        DeclareLaunchArgument(
            'align_cross_tolerance', default_value='0.06',
            description='m PERPENDICULAR to the approach line that the aircraft '
                        'may be off the window centreline before the traverse is '
                        'allowed to start. This is the tight one on purpose: '
                        'every centimetre of it comes straight out of the lateral '
                        'clearance, which on a 0.6 m window is only ~0.17 m a '
                        'side to begin with.'),
        DeclareLaunchArgument(
            'align_along_tolerance', default_value='0.25',
            description='m ALONG the approach line. Loose, because being 20 cm '
                        'early or late on the standoff point changes nothing '
                        'except the length of the run through.'),
        DeclareLaunchArgument(
            'recentre_clear_seconds', default_value='0.6',
            description='s the detection must stay untruncated before RECENTRE '
                        'hands over to AIM.'),
        DeclareLaunchArgument(
            'recentre_timeout', default_value='20.0',
            description='s of RECENTRE before the attempt is abandoned. A window '
                        'that never comes fully into view was never measured, and '
                        'flying at an unmeasured aperture is the thing this whole '
                        'stage exists to prevent.'),
        DeclareLaunchArgument(
            'recentre_backoff_seconds', default_value='7.0',
            description='s of fruitless yawing before RECENTRE concludes the '
                        'window is simply too wide for the field of view from '
                        'here and moves away from it instead.'),
        DeclareLaunchArgument(
            'recentre_backoff', default_value='0.60',
            description='m to retreat, backwards along the current heading, on '
                        'each back-off.'),
        DeclareLaunchArgument(
            'recentre_max_backoffs', default_value='2',
            description='how many back-offs before giving up on the window.'),
        DeclareLaunchArgument(
            'recentre_yaw_step_deg', default_value='4.0',
            description='deg of yaw RECENTRE commands per tick while the '
                        'window is clipped by the frame edge. A nudge, not a '
                        'slew: enough to tell whether the window edge really '
                        'is the frame edge, small enough that the window '
                        'cannot swing out of view on the other side.'),
        DeclareLaunchArgument(
            'recentre_yaw_limit_deg', default_value='20.0',
            description='deg of cumulative yaw RECENTRE is allowed either side '
                        'of the heading an attempt started at. Past this the '
                        'window does not fit in the field of view from here '
                        'and it backs off instead of turning further.'),
        DeclareLaunchArgument('align_yaw_tolerance_deg', default_value='8.0'),
        DeclareLaunchArgument(
            'align_settle_seconds', default_value='1.5',
            description='How long position, altitude and heading must ALL be in '
                        'tolerance together before the traverse commits.'),
        DeclareLaunchArgument('align_timeout', default_value='60.0'),
        DeclareLaunchArgument('traverse_timeout', default_value='25.0'),
        DeclareLaunchArgument(
            'clear_seconds', default_value='4.0',
            description='Station keeping on the far side before the descent.'),
        DeclareLaunchArgument(
            'blind_traverse_seconds', default_value='3.0',
            description='If vision dies mid-traverse, how long the aircraft may '
                        'push on open-loop along the committed heading before it '
                        'lands. This exists so it does not stop inside the '
                        'aperture. 0 disables it -- do not, unless you have a '
                        'reason.'),

        # ---- the estimator. The defaults are the ones the rejection logic was
        # tuned against; raise a threshold only after the log tells you which
        # test is doing the rejecting.
        DeclareLaunchArgument(
            'depth_min', default_value='0.35',
            description='m. Corner depths below this are not believed. Well '
                        'clear of the D435i\'s ~0.17 m minimum at 848x480, '
                        'which is deliberate -- a window corner that close is '
                        'a misdetection, not a measurement.'),
        DeclareLaunchArgument(
            'depth_max', default_value='8.0',
            description='m. Above this the depth is not believed. A D435i is '
                        'specified to about 3 m at 2%% error and degrades from '
                        'there; 8 m is a generous outer bound meant to reject '
                        'nonsense, not a working range.'),
        DeclareLaunchArgument(
            'corner_spread', default_value='0.25',
            description='m the four corner depths may differ from their median. '
                        'The primary outlier filter: a sample box that landed on '
                        'the wall behind the frame fails here.'),
        DeclareLaunchArgument(
            'corner_spread_frac', default_value='0.15',
            description='The same allowance as a fraction of range; the larger '
                        'of the two is used, so an obliquely-seen window at '
                        'distance is not rejected for being oblique.'),
        DeclareLaunchArgument(
            'plane_tolerance', default_value='0.15',
            description='m the reconstructed corners may sit off their own '
                        'best-fit plane. Backstop to corner_spread.'),
        DeclareLaunchArgument('window_min_size', default_value='0.35'),
        DeclareLaunchArgument(
            'window_max_size', default_value='3.0',
            description='m. With window_min_size, the sanity check on the '
                        'reconstructed aperture: a quad that comes out 6 m wide '
                        'is a bad depth, not a big window.'),
        DeclareLaunchArgument(
            'max_tilt_deg', default_value='35.0',
            description='How far off horizontal the window normal may be. A '
                        'window is vertical; a horizontal normal means the '
                        'detection is the floor or a ceiling light.'),
        DeclareLaunchArgument(
            'buffer_seconds', default_value='2.5',
            description='Length of the rolling window the median is taken over.'),
        DeclareLaunchArgument(
            'pose_min_samples', default_value='6',
            description='Accepted samples needed before the pose is flown to.'),
        DeclareLaunchArgument(
            'pose_max_age', default_value='1.5',
            description='s after which the newest accepted sample stops counting '
                        'as evidence about where the window is now.'),
        DeclareLaunchArgument(
            'pose_lost_timeout', default_value='6.0',
            description='s without a usable pose during the aim or the approach '
                        'before the attempt is abandoned into a landing.'),
        DeclareLaunchArgument(
            'gate_metres', default_value='1.0',
            description='m a new sample centre may sit from the current estimate '
                        'before it is rejected as an outlier.'),
        DeclareLaunchArgument('gate_yaw_deg', default_value='40.0'),
        DeclareLaunchArgument(
            'gate_reset_count', default_value='25',
            description='Consecutive gated samples before the buffer is thrown '
                        'away and the estimate rebuilt. If every new sample '
                        'disagrees with the estimate, the estimate is the '
                        'minority opinion -- this is what stops the aircraft '
                        'flying confidently at nothing.'),
        DeclareLaunchArgument(
            'side_mismatch', default_value='0.40',
            description='Fraction by which opposite sides of the reconstructed '
                        'quad may differ. A real window seen obliquely still '
                        'has matching opposite sides; a bad corner does not.'),

        # ---- the rest ----
        DeclareLaunchArgument(
            'reboot_fc', default_value='false',
            description='Reboot the FC first if EKF2 came up without the '
                        'rangefinder. Raise flight_node_delay to ~75 with this.'),
        DeclareLaunchArgument('flight_node_delay', default_value='12.0'),
        DeclareLaunchArgument('lcd', default_value='false'),
        DeclareLaunchArgument(
            'lcd_port', default_value='',
            description='Arduino serial port; empty = auto-detect.'),

        # flight:=false leaves the detector running on its own, which is the
        # bench test. Grouped rather than conditioned individually because two
        # of these already carry a condition of their own.
        GroupAction([lcd_node, reboot_node, traverse_node],
                    condition=IfCondition(flight)),
        detect_node,
    ])
