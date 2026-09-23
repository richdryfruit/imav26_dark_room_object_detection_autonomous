"""
Translate test launch: the offboard_translate node (the uXRCE-DDS agent
comes from imav_bringup).

Arm -> climb to 1.0 m -> hold -> move 1.0 m (forward by default) -> hold
-> land. No ZED, no localization: the flight relies only on ARK Flow +
lidar + IMU fused in PX4.

NOTE: launching the flight node this way means stdin is not a tty, so the
'q' / 'k' keyboard aborts will be DISABLED. For every test where the props
are on, launch the support stack only (the default) and run the node by hand in a
second tmux pane so you keep the abort keys:

    ros2 launch drone_testing translate_test.launch.py
    ros2 run drone_testing offboard_translate --ros-args \
        -p takeoff_altitude:=1.0 -p move_distance:=1.0 -p move_direction:=forward

Directions are body-frame relative to the yaw held since arming:
forward | backward | left | right.

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

    # Delay so the DDS session is up and PX4 topics exist before the node
    # starts publishing. Without this the first setpoints are dropped.
    translate_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_testing',
                executable='offboard_translate',
                name='offboard_translate',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'post_hold_seconds': LaunchConfiguration('post_hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'move_distance': LaunchConfiguration('move_distance'),
                    'move_direction': LaunchConfiguration('move_direction'),
                    'move_speed': LaunchConfiguration('move_speed'),
                    'request_offboard_from_ros': LaunchConfiguration(
                        'request_offboard_from_ros'),
                }],
            )
        ],
        condition=UnlessCondition(agent_only),
    )

    # Status readout on the Arduino-driven LCD. Started at launch, not
    # with the flight node, so the display is alive from boot and can show
    # "NO TAKEOFF NODE" while you are still getting set up. The flight node
    # publishes the same /takeoff_status format, so this needs no changes.
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
            description='Do not start the flight node; run it manually '
                        'so the q/k keyboard aborts stay available.'),
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='1.0',
            description='Metres above the arming point. Start lower (0.3) on first flights.'),
        DeclareLaunchArgument(
            'hold_seconds', default_value='5.0',
            description='Station-keeping time at altitude before the move starts. '
                        'The optical flow x/y latch has to happen in here.'),
        DeclareLaunchArgument(
            'post_hold_seconds', default_value='5.0',
            description='Station-keeping time after the move, before the descent.'),
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
            'move_distance', default_value='1.0',
            description='Metres to travel horizontally.'),
        DeclareLaunchArgument(
            'move_direction', default_value='forward',
            description='Body-frame direction: forward | backward | left | right.'),
        DeclareLaunchArgument(
            'move_speed', default_value='0.30',
            description='m/s the horizontal setpoint is walked at. Keep it slow: '
                        'the optical flow is the only thing measuring this move.'),
        DeclareLaunchArgument(
            'request_offboard_from_ros', default_value='false',
            description='false = you flip the Offboard switch on the TX yourself.'),
        DeclareLaunchArgument(
            'lcd', default_value='true',
            description='Start the Arduino LCD status node.'),
        DeclareLaunchArgument(
            'lcd_port', default_value='',
            description='Arduino serial port; empty = auto-detect ttyACM*/ttyUSB*.'),
        lcd_node,
        translate_node,
    ])
