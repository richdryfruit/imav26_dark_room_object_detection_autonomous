"""
PX4 SITL + Gazebo (gz sim 8) for mission_fsm_part1/2, on a laptop.

    ros2 launch drone_testing sitl.launch.py sitl_src:=<imav_indoor_2026_sitl checkout>

World: imav2026_scaled, the real arena x2.2 (the only world with every sensor
plugin PX4 needs -- baro, mag, navsat, flow). In it:
    takeoff pad   id 0   (-4.4, -14.3)     <- default spawn
    window marker id 2   (-4.4,   7.15)    21.45 m ahead, 0.88 m marker
    window centre        (-5.83,  9.856, z 3.85), 1.2 x 1.2 m

Starts: gz sim on the imav_indoor_2026 world, the x500 with a down camera,
PX4 SITL (standalone, attaches to the spawned model), the uXRCE-DDS agent
(UDP 8888), the ROS<->gz bridge and aruco_pose reading the sim down camera.
The flight node is run by hand in a second terminal, as on the real drone.

SENSORS AND ESTIMATOR, matched to the aircraft rather than PX4's SITL default:
    x/y     PMW3901 optical flow (the sim flow camera is its 42 deg FOV)
    height  baro reference + TFmini Plus (conditional): the sim's single-beam
            lidar clipped to 0.1-12 m, 2 cm noise. Range as the height REF
            stops EKF2 ever starting flow fusion -- see params below.
    GPS     simulated but NOT fused (EKF2_GPS_CTRL 0)

Gazebo's yaw 0 is +X; the course runs along +Y, hence yaw 1.5708 (facing
the window marker from the takeoff pad).
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    sitl_src = os.path.expanduser(arg('sitl_src'))
    px4_dir = os.path.expanduser(arg('px4_dir'))
    world_name = arg('world')
    world_file = os.path.join(sitl_src, 'world', world_name + '.sdf.world')
    share = get_package_share_directory('drone_testing')
    urdf = xacro.process_file(
        os.path.join(share, 'sim', 'sim_drone.urdf.xacro')).toxml()
    # TFmini Plus: 0.1-12 m, ~2 cm noise. The only <max>100.0</max> in the
    # model is the downward lidar's range; the noise goes right after it.
    i = urdf.find('<max>100.0</max>')
    j = urdf.find('</range>', i)
    if i > 0 and j > 0:
        j += len('</range>')
        urdf = (urdf[:i] + '<max>12.0</max>' + urdf[i + len('<max>100.0</max>'):j]
                + '<noise><type>gaussian</type><mean>0</mean>'
                  '<stddev>0.02</stddev></noise>' + urdf[j:])

    plugins = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'src',
                           'modules', 'simulation', 'gz_plugins')
    plugin_dirs = [plugins] + sorted(
        os.path.join(plugins, d) for d in (os.listdir(plugins)
                                           if os.path.isdir(plugins) else [])
        if os.path.isdir(os.path.join(plugins, d)))

    env = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join([
            os.path.dirname(get_package_share_directory('imav_indoor_2026')),
            os.path.join(sitl_src, 'world'),
            os.path.join(sitl_src, 'world', 'models')])),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                  ':'.join(plugin_dirs)),
    ]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch',
            'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v2 {world_file}',
                          'on_exit_shutdown': 'true'}.items())
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description': urdf}])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'x500_drone', '-topic', 'robot_description',
                            '-x', arg('x'), '-y', arg('y'), '-z', '0.35',
                            '-Y', arg('yaw')])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=[
                      '/down_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo'
                      '[gz.msgs.CameraInfo',
                      '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                  ])
    # Flow for x/y, GPS off; arm with no RC/GCS. Height ref is BARO with the
    # rangefinder CONDITIONAL: EKF2 only starts flow fusion with a valid
    # terrain estimate, and with the rangefinder as the height reference
    # (HGT_REF 2) terrain is never estimated -- flow never starts, local
    # position goes invalid at arming and PX4 disarms. This is PX4's
    # recommended flow setup; the rangefinder still gives height above ground.
    # Set twice: as PX4_PARAM_* (read by rcS at boot on PX4 >= 1.14) and
    # again with px4-param once it is up, for builds that ignore the env.
    params = {
        'EKF2_GPS_CTRL': 0, 'EKF2_OF_CTRL': 1, 'EKF2_RNG_CTRL': 1,
        'EKF2_HGT_REF': 1, 'EKF2_MIN_RNG': 0.1, 'COM_ARM_WO_GPS': 1,
        'NAV_RCL_ACT': 0, 'NAV_DLL_ACT': 0, 'COM_RCL_EXCEPT': 4,
        # No barometer, as on the aircraft (lidar = height). The sim baro is
        # flagged faulty on the pad ("height estimate not stable", arming
        # denied); with it off EKF2 falls back to the rangefinder for height
        # and still estimates terrain from it, which is what flow needs.
        'EKF2_BARO_CTRL': 0,
        # No RC in SITL: without this PX4 raises manual_control_signal_lost
        # and drops out of Offboard into Hold.
        'COM_RC_IN_MODE': 4,
        # ONE EKF on one IMU and one compass, as on the aircraft. The sim
        # exposes 3 IMUs and 2 mags, PX4 runs an EKF per pair, and at arming
        # the selector switched to an instance 147 deg out in heading and 2 m
        # out in position -> local position invalid -> auto-disarm. Boot-time
        # only (read at ekf2 start), hence the PX4_PARAM_* route.
        'EKF2_MULTI_IMU': 0, 'SENS_IMU_MODE': 1,
        'EKF2_MULTI_MAG': 0, 'SENS_MAG_MODE': 1,
        # No external vision. The SITL model publishes Gazebo odometry, which
        # PX4's gz_bridge forwards as vehicle_visual_odometry in a different
        # frame (ev_hpos test ratio ~787). Whenever EKF2 tried to start on it
        # it reset position/heading to it -- the 2 m / 147 deg jump at arming
        # and the flapping local_position_invalid. Flow is the only x/y here.
        'EKF2_EV_CTRL': 0,
        # No power module in SITL ("system power unavailable").
        'CBRK_SUPPLY_CHK': 894281,
    }
    env_params = ' '.join(f'PX4_PARAM_{k}={v}' for k, v in params.items())
    set_params = '; '.join(f'bin/px4-param set {k} {v}' for k, v in params.items())
    px4 = ExecuteProcess(
        cmd=['bash', '-c',
             f'cd {px4_dir} && rm -f build/px4_sitl_default/rootfs/*.bson && '
             f'{env_params} PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 '
             f'PX4_GZ_MODEL_NAME=x500_drone PX4_GZ_WORLD={world_name} '
             'build/px4_sitl_default/bin/px4 -d'],
        output='screen')
    px4_params = ExecuteProcess(
        cmd=['bash', '-c',
             f'cd {px4_dir}/build/px4_sitl_default && {set_params}; '
             'echo "SITL PARAMS SET: flow x/y, rangefinder height, no baro, no GPS, no EV, no RC"; '
             # One-shot health report after EKF2 has had time to settle, so a
             # refused arm or a flapping position is explained in this log.
             'sleep 15; echo "===== SITL HEALTH REPORT ====="; '
             'for p in ' + ' '.join(params) + '; do bin/px4-param show $p | grep -E "^ *[x+*]"; done; '
             'bin/px4-commander check; bin/px4-ekf2 status; '
             'bin/px4-listener estimator_status_flags -n 1 | grep -E "cs_(opt_flow|rng_hgt|rng_terrain|baro|mag_hdg|yaw_align|valid_fake|constant)|fs_bad|reject"; '
             'for i in 1 2 3 4 5; do echo "--- sample $i"; '
             'bin/px4-listener estimator_status -n 1 | grep -E "pre_flt_fail|test_ratio|pos_horiz_acc"; '
             'bin/px4-listener estimator_innovation_test_ratios -n 1 | grep -E "flow|heading|gps_h|ev_h|rng|hagl"; '
             'bin/px4-listener vehicle_local_position -n 1 | grep -E " (xy_valid|v_xy_valid|eph|evh|heading_good_for_control):"; '
             'sleep 1; done; '
             'echo "===== END HEALTH REPORT ====="; '
             f'find {plugins} -name libOpticalFlowSystem.so | grep -q . '
             '&& echo "optical flow plugin: found" '
             '|| echo "WARNING: libOpticalFlowSystem.so NOT BUILT -- no optical '
             'flow. sudo apt install libopencv-dev, then make px4_sitl again."'],
        output='screen')
    agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                           output='screen')
    aruco = Node(package='drone_testing', executable='aruco_pose',
                 name='aruco_pose', output='screen',
                 additional_env={'PYTHONFAULTHANDLER': '1'},
                 parameters=[{'image_topic': '/down_cam/image',
                              'width': 800, 'height': 600,
                              'hfov_deg': 78.0,
                              'marker_id': int(arg('marker_id')),
                              'marker_size': float(arg('marker_size')),
                              'aruco_dict': arg('aruco_dict'),
                              'stream_port': 8080}])
    return env + [gazebo, rsp, spawn, bridge,
                  TimerAction(period=8.0, actions=[px4]),
                  TimerAction(period=20.0, actions=[px4_params]),
                  agent, TimerAction(period=5.0, actions=[aruco])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sitl_src',
                              default_value='~/ros2_ws/src/imav_indoor_2026_sitl',
                              description='Source checkout of imav_indoor_2026_sitl '
                                          '(the worlds and models are not installed).'),
        DeclareLaunchArgument('px4_dir', default_value='~/PX4-Autopilot'),
        DeclareLaunchArgument('world', default_value='imav2026_scaled'),
        DeclareLaunchArgument('x', default_value='-4.4'),
        DeclareLaunchArgument('y', default_value='-14.3'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        DeclareLaunchArgument('marker_id', default_value='2'),
        DeclareLaunchArgument('marker_size', default_value='0.88',
                              description='Edge of the printed marker in the '
                                          'sim world, not the real 0.80.'),
        DeclareLaunchArgument('aruco_dict', default_value='DICT_5X5_50'),
        OpaqueFunction(function=_setup),
    ])
