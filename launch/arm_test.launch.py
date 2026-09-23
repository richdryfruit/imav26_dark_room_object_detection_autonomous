"""
Bench arm test launch. No ZED, no localization -- fewest moving parts.

The uXRCE-DDS agent is NOT started here: imav_bringup (bringup.launch.py)
owns it, and ROS_DOMAIN_ID comes from that environment too.

NOTE: launching the mission node this way means stdin is not a tty, so the
'q' keyboard abort will be DISABLED. For the first arm tests, leave
agent_only:=true (the default: nothing is started) and run the mission node
by hand in a second tmux pane so you keep the abort key.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')

    # PX4 finishes creating its publishers ~7 s after the agent comes up:
    # ~6 s for the session handshake, then ~1 s to enumerate all 65 topics.
    # The /fmu/out writers are created LAST, so anything shorter than that
    # starts the node before VehicleStatus exists. 12 s leaves margin.
    mission_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='drone_testing',
                executable='offboard_mission',
                name='arm_disarm_test',
                output='screen',
                emulate_tty=True,
            )
        ],
        condition=UnlessCondition(agent_only),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_only', default_value='true',
            description='Do not start the mission node; run it manually.'),
        mission_node,
    ])
