"""
DARK ROOM BACKUP: pad -> climb 2.50 m -> window marker (the second ArUco) ->
1.75 m -> strafe 0.15 m left -> window, aimed traverse_centre_offset above ->
slow landing inside. See drone_testing/mission_fsm_darkroom_backup.py.
The leg follows the carpet track (floor_line, started by mission_fsm.launch.py).
The pilot switches Offboard and arms (transmitter or dashboard).

    ros2 launch drone_testing mission_fsm_darkroom_backup.launch.py
    ros2 run drone_testing mission_fsm_darkroom_backup --ros-args --params-file \
      $(ros2 pkg prefix drone_testing)/share/drone_testing/config/hw_darkroom_backup.yaml

No lidar_loc needed: nothing is flown on the room lidar. (The inbound window
alignment still uses /lidar/scan_level if it is there.)

This is mission_fsm.launch.py with the defaults below, so every one of its
arguments still works and still overrides. agent_only defaults to true: the
stack comes up and you run the flight node by hand so the q/k keyboard aborts
work. agent_only:=false flies it from here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Declared here FIRST, so they win over mission_fsm.launch.py's defaults but
# still lose to anything given on the command line.
BACKUP_DEFAULTS = {
    'fsm_executable': 'mission_fsm_darkroom_backup',
    # aruco_pose's own id, as mission_fsm_full.launch.py. The FLIGHT node's
    # pad_marker_id comes from hw_darkroom_backup.yaml (1), which loads after
    # this and must differ from window_marker_id or MissionFSM will not start.
    'pad_marker_id': '0',
    'stream_port': '8081',    # window_detect's browser stream; aruco has 8080
    'window_marker_id': '0',
    'cruise_altitude': '2.50',
    'outbound_distance': '9.3',
    'window_altitude_m': '1.75',
    'room_altitude': '1.75',
    'window_offset': '0.15',
    'window_offset_direction': 'left',
    'standoff_distance': '1.67',
    'inside_distance': '0.80',
    'flight_seconds': '360.0',
}


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v)
         for k, v in BACKUP_DEFAULTS.items()]
        + [DeclareLaunchArgument('hw_params', default_value=PathJoinSubstitution([
            FindPackageShare('drone_testing'), 'config',
            'hw_darkroom_backup.yaml']))]
        + [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('drone_testing'),
                'launch', 'mission_fsm.launch.py'])))])
