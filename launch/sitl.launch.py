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

    env = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join([
            os.path.dirname(get_package_share_directory('imav_indoor_2026')),
            os.path.join(sitl_src, 'world'),
            os.path.join(sitl_src, 'world', 'models')])),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.path.join(
            px4_dir, 'build', 'px4_sitl_default', 'src', 'modules',
            'simulation', 'gz_plugins')),
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
    px4 = ExecuteProcess(
        cmd=['bash', '-c',
             f'cd {px4_dir} && rm -f build/px4_sitl_default/rootfs/*.bson && '
             f'PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 '
             f'PX4_GZ_MODEL_NAME=x500_drone PX4_GZ_WORLD={world_name} '
             'build/px4_sitl_default/bin/px4 -d'],
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
