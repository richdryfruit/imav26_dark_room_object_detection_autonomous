"""
MISSION PART 1: takeoff pad -> window marker at 2.10 m, land on it (or at
9.3 m if it is never seen). See drone_testing/mission_fsm_part1.py.

    ros2 launch drone_testing mission_fsm_part1.launch.py
    ros2 run drone_testing mission_fsm_part1            # second pane

This is mission_fsm.launch.py with the part-1 defaults below, so every one of
its arguments still works and still overrides. agent_only defaults to true,
as everywhere else here: the stack comes up and you run the flight node by
hand so the q/k keyboard aborts work. agent_only:=false flies it from here.

The window detector and the doll model are off: this flight never sees the
window. The RealSense stays up for the VIO; the down camera (aruco_pose) is
what this flight actually uses.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Declared here FIRST, so they win over mission_fsm.launch.py's defaults but
# still lose to anything given on the command line.
PART1_DEFAULTS = {
    'fsm_executable': 'mission_fsm_part1',
    'window_marker_id': '0',
    'cruise_altitude': '2.10',
    'outbound_distance': '9.3',
    'flight_seconds': '240.0',
    'detect': 'false',
    'dolls': 'false',
    'doll_text': 'false',
}


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v)
         for k, v in PART1_DEFAULTS.items()]
        + [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('drone_testing'),
                'launch', 'mission_fsm.launch.py'])))])
