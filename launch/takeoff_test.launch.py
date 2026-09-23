"""
Takeoff test launch: the offboard_takeoff node (the uXRCE-DDS agent
comes from imav_bringup).

No ZED, no localization -- the flight relies only on ARK Flow + lidar +
IMU fused in PX4, so there is nothing else to go wrong.

NOTE: launching the takeoff node this way means stdin is not a tty, so the
'q' / 'k' keyboard aborts will be DISABLED. For every test where the props
are on, launch the support stack only (the default) and run the node by hand in a
second tmux pane so you keep the abort keys:

    ros2 launch drone_testing takeoff_test.launch.py
    ros2 run drone_testing offboard_takeoff --ros-args -p takeoff_altitude:=0.3

Your RC kill switch is the real safety net regardless -- the keyboard is a
convenience, not a substitute.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')

    # PX4 finishes creating its publishers ~7 s after the agent comes up:
    # ~6 s for the session handshake, then ~1 s to enumerate all 65 topics.
    # The /fmu/out writers are created LAST, so anything shorter starts the
    # node before VehicleStatus exists. 12 s leaves margin.
    takeoff_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_testing',
                executable='offboard_takeoff',
                name='offboard_takeoff',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'request_offboard_from_ros': LaunchConfiguration(
                        'request_offboard_from_ros'),
                }],
            )
        ],
        condition=UnlessCondition(agent_only),
    )

    # Status readout on the Arduino-driven LCD. Started at launch, not
    # with the flight node, so the display is alive from boot and can show
    # "NO TAKEOFF NODE" while you are still getting set up.
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
            description='Do not start the takeoff node; run it manually '
                        'so the q/k keyboard aborts stay available.'),
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='0.80',
            description='Metres above the arming point. Start lower (0.3) on first flights.'),
        DeclareLaunchArgument(
            'hold_seconds', default_value='15.0',
            description='Station-keeping time once the altitude is reached.'),
        DeclareLaunchArgument(
            'ground_wait_seconds', default_value='5.0',
            description='Time spent armed on the ground before the climb starts.'),
        DeclareLaunchArgument(
            'climb_speed', default_value='0.35',
            description='m/s the climb setpoint is ramped at.'),
        DeclareLaunchArgument(
            'land_speed', default_value='0.15',
            description='m/s the descent setpoint is ramped at.'),
        DeclareLaunchArgument(
            'request_offboard_from_ros', default_value='false',
            description='false = you flip the Offboard switch on the TX yourself.'),
        DeclareLaunchArgument(
            'lcd', default_value='false',
            description='Start the Arduino LCD status node.'),
        DeclareLaunchArgument(
            'lcd_port', default_value='',
            description='Arduino serial port; empty = auto-detect ttyACM*/ttyUSB*.'),
        lcd_node,
        takeoff_node,
    ])
