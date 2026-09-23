"""The 2D lidar stack on the real aircraft, seeded from the pad heading.

    ros2 launch drone_testing lidar_real.launch.py

Start it with the aircraft ALREADY on the pad, pointing exactly at the window
(perpendicular to its wall) -- the same pose it is armed in. It:

  1. reads EKF2's heading for ~1 s and sets wall_localizer's seed_yaw_offset
     to -(that heading), so the pad pose is arena yaw +90 deg (facing into
     the room). See drone_testing/yaw_seed.py for the arithmetic;
  2. starts lidar_loc's hw_lidar.launch.py (LDS-01 driver + scan_leveler),
     the seeded wall_localizer and pose_kf (/lidar/odom_kf, which the room
     scan and the doll geotags fly on), all on the real-room config
     (lidar_arena.yaml: 2.5 x 2.5 m, the blue + red windows).

seed_yaw_offset:=<rad> overrides step 1 (e.g. from `ros2 run drone_testing
yaw_seed`). If no attitude arrives it refuses to start rather than guessing.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    lidar_loc = get_package_share_directory('lidar_loc')
    config = arg('config') or os.path.join(lidar_loc, 'config', 'lidar_arena.yaml')

    given = arg('seed_yaw_offset').strip()
    if given:
        seed = float(given)
        how = 'given on the command line'
    else:
        from drone_testing.yaw_seed import read_pad_heading, seed_from_heading
        heading = read_pad_heading(timeout=float(arg('seed_timeout')))
        if heading is None:
            return [LogInfo(msg='lidar_real: NO /fmu/out/vehicle_attitude -- '
                                'cannot seed the arena heading. Is the agent '
                                'up, and ROS_DOMAIN_ID right? Not starting.'),
                    Shutdown(reason='no attitude for the yaw seed')]
        seed = seed_from_heading(heading)
        how = f'pad heading {math.degrees(heading):+.1f} deg NED'

    msg = (f'lidar_real: seed_yaw_offset {seed:+.4f} rad ({how}). The pad pose '
           'is arena yaw +90 deg: aircraft pointing at the window.')
    hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_loc, 'launch', 'hw_lidar.launch.py')),
        launch_arguments={'config': config, 'use_sim_time': 'false',
                          'mount_xyz': arg('mount_xyz')}.items())
    localizer = Node(
        package='lidar_loc', executable='wall_localizer', name='wall_localizer',
        output='screen',
        parameters=[config, {'use_sim_time': False, 'yaw_source': 'imu',
                             'seed_yaw_offset': seed, 'origin_height': 0.0}])
    kf = Node(package='lidar_loc', executable='pose_kf.py', name='pose_kf',
              output='screen', parameters=[{'use_sim_time': False}])
    # lidar_loc subscribes PX4 at /fmu/out/... (no namespace). The aircraft
    # publishes under the imav_bringup vehicle_namespace (/uav_2/fmu/...),
    # so without these the leveler has no attitude and the localizer no
    # height. Same prefix as every drone_testing node (px4_topics).
    from drone_testing.px4_topics import versioned_names
    prefix = versioned_names('x')[0][:-len('x')]            # '/<ns>/fmu/out/'
    remaps = [SetRemap(src=f'/fmu/out/{t}', dst=f'{prefix}{t}')
              for t in ('vehicle_attitude', 'vehicle_odometry',
                        'vehicle_local_position_v1', 'timesync_status')]
    return [LogInfo(msg=msg + f' PX4 topics: {prefix}*'),
            GroupAction(remaps + [hw, localizer, kf], scoped=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='',
                              description='lidar_loc config; default the '
                              'real room, lidar_arena.yaml'),
        DeclareLaunchArgument('seed_yaw_offset', default_value='',
                              description='rad; empty = read from the pad'),
        DeclareLaunchArgument('seed_timeout', default_value='20.0'),
        DeclareLaunchArgument('mount_xyz', default_value='[0.0, 0.0, 0.135]',
                              description='lidar above the CG, measured'),
        OpaqueFunction(function=_setup),
    ])
