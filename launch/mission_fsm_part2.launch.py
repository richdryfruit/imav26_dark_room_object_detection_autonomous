"""
MISSION PART 2: from the window marker, strafe 0.15 m left, window in, dark
room at 1.90 m (dolls geotagged and counted), window out, strafe back and land
on the marker. See drone_testing/mission_fsm_part2.py.

    ros2 launch drone_testing mission_fsm_part2.launch.py
    ros2 run drone_testing mission_fsm_part2            # second pane

Also needs the lidar stack for the room:
    ros2 launch lidar_loc localize.launch.py

This is mission_fsm.launch.py with the part-2 defaults below, so every one of
its arguments still works and still overrides. agent_only defaults to true:
the stack comes up and you run the flight node by hand so the q/k keyboard
aborts work. agent_only:=false flies it from here.

The doll count is printed in this terminal by doll_report_text (count minus
doll_offset) and by the flight node (raw /doll_count).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Declared here FIRST, so they win over mission_fsm.launch.py's defaults but
# still lose to anything given on the command line.
PART2_DEFAULTS = {
    'fsm_executable': 'mission_fsm_part2',
    'window_marker_id': '0',
    'cruise_altitude': '1.90',
    'window_altitude_m': '1.90',
    'room_altitude': '1.90',
    'window_offset': '0.15',
    'window_offset_direction': 'left',
    'standoff_distance': '1.67',
    'inside_distance': '0.80',
    'outside_distance': '1.67',
    'room_mode': 'lidar',
    'room_scan_sequence':
        'forward 0.8, right 0.8, backward 0.8, left 0.8, yaw 180',
    'flight_seconds': '420.0',
}


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v)
         for k, v in PART2_DEFAULTS.items()]
        + [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('drone_testing'),
                'launch', 'mission_fsm.launch.py'])))])
