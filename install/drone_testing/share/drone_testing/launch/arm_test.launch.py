"""
Bench arm test launch. No ZED, no localization -- fewest moving parts.

NOTE: launching the mission node this way means stdin is not a tty, so the
'q' keyboard abort will be DISABLED. For the first arm tests, launch only
the agent (agent_only:=true) and run the mission node by hand in a second
tmux pane so you keep the abort key.

DOMAIN ID: PX4's uxrce_dds_client tells the agent which DDS domain to put
its participant in -- the agent's own ROS_DOMAIN_ID is ignored. That domain
is PX4's XRCE_DDS_DOM_ID parameter, which is 0 on this airframe. If your
shell exports a different ROS_DOMAIN_ID (this Jetson's .bashrc exports 25),
the agent creates all 65 /fmu topics in domain 0 and your node sits in
domain 25 seeing none of them -- the symptom is 'Waiting for VehicleStatus
from PX4...' forever while the agent terminal looks perfectly healthy.
So we pin ROS_DOMAIN_ID for everything this launch starts. Override with
px4_domain_id:= if you change XRCE_DDS_DOM_ID on the flight controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')

    # The PX4 uXRCE-DDS agent is MicroXRCEAgent, not micro_ros_agent -- the
    # latter is the micro-ROS agent and is not what PX4 speaks to.
    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial',
             '--dev', LaunchConfiguration('serial_dev'),
             '-b', LaunchConfiguration('baudrate')],
        name='micro_xrce_dds_agent',
        output='screen',
    )

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
            description='Start only the uXRCE-DDS agent; run the mission node manually.'),
        DeclareLaunchArgument(
            'px4_domain_id', default_value='0',
            description="DDS domain PX4's XRCE_DDS_DOM_ID puts the agent participant in."),
        DeclareLaunchArgument(
            'serial_dev', default_value='/dev/ttyTHS1',
            description='Serial port the flight controller is on.'),
        DeclareLaunchArgument(
            'baudrate', default_value='921600',
            description='Serial baudrate; must match PX4 SER_TEL2_BAUD.'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('px4_domain_id')),
        microxrce_agent,
        mission_node,
    ])
