"""
Sequence test launch: the offboard_sequence node (the uXRCE-DDS agent
comes from imav_bringup).

Arm -> climb to 1.0 m -> hold -> run four commanded motions one at a time,
holding between each -> hold -> land. No ZED, no localization: the flight
relies only on ARK Flow + lidar + IMU fused in PX4.

The mission is one string:

    sequence:="forward 1.0, yaw 30, up 0.5, right 1.0"

Each item is a motion and a number, comma separated:

    forward | backward | left | right   metres
    up | down                           metres
    yaw                                 degrees, + = clockwise from above

Directions are body-frame relative to the heading held AT ARMING, and stay
that way for the whole flight: a `yaw 30` step rotates the airframe but
does not rotate what `forward` means, so the move after it flies the same
ground track it would have without the yaw (this is the same convention as
a velocity setpoint in SITL -- the setpoint frame is NED, not the body).
Pass direction_frame:=current for the other convention.

NOTE: launching the flight node this way means stdin is not a tty, so the
'q' / 'k' keyboard aborts will be DISABLED. For every test where the props
are on, launch the support stack only (the default) and run the node by hand in a
second tmux pane so you keep the abort keys:

    ros2 launch drone_testing sequence_test.launch.py
    ros2 run drone_testing offboard_sequence --ros-args \
        -p takeoff_altitude:=1.0 \
        -p sequence:="forward 1.0, yaw 30, up 0.5, right 1.0"

Your RC kill switch is the real safety net regardless -- the keyboard is a
convenience, not a substitute.

BEFORE THE FIRST FLIGHT, check MPC_LAND_SPEED against land_speed below.
PX4's land detector will not declare touchdown unless the commanded sink
rate is at least 0.9 * MPC_LAND_SPEED, which is why the last flight sat on
the ground reporting airborne. Setting MPC_LAND_SPEED to 0.2 makes PX4
agree with us; the node also confirms touchdown independently either way.
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
    sequence_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='drone_testing',
                executable='offboard_sequence',
                name='offboard_sequence',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'sequence': LaunchConfiguration('sequence'),
                    'direction_frame': LaunchConfiguration('direction_frame'),
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'step_hold_seconds': LaunchConfiguration('step_hold_seconds'),
                    'post_hold_seconds': LaunchConfiguration('post_hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'move_speed': LaunchConfiguration('move_speed'),
                    'yaw_rate': LaunchConfiguration('yaw_rate'),
                    'min_altitude': LaunchConfiguration('min_altitude'),
                    'max_altitude': LaunchConfiguration('max_altitude'),
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
            'sequence', default_value='forward 1.0, yaw 30, up 0.5, right 1.0',
            description='The four motions, comma separated. Each is a name and a '
                        'number: forward|backward|left|right <m>, up|down <m>, '
                        'yaw <deg, + = clockwise>.'),
        DeclareLaunchArgument(
            'direction_frame', default_value='home',
            description="home = forward always means the heading held at arming "
                        "(a yaw step does not rotate it); current = each move is "
                        "resolved against the yaw commanded at that point."),
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='1.0',
            description='Metres above the arming point. Start lower (0.3) on first flights.'),
        DeclareLaunchArgument(
            'hold_seconds', default_value='5.0',
            description='Station-keeping time at altitude before the first step. '
                        'The optical flow x/y latch has to happen in here.'),
        DeclareLaunchArgument(
            'step_hold_seconds', default_value='3.0',
            description='Station-keeping time between steps, so each one starts '
                        'from a settled vehicle instead of compounding the last '
                        "step's overshoot."),
        DeclareLaunchArgument(
            'post_hold_seconds', default_value='5.0',
            description='Station-keeping time after the last step, before the descent.'),
        DeclareLaunchArgument(
            'ground_wait_seconds', default_value='5.0',
            description='Time spent armed on the ground before the climb starts.'),
        DeclareLaunchArgument(
            'climb_speed', default_value='0.35',
            description='m/s the climb setpoint is ramped at.'),
        DeclareLaunchArgument(
            'land_speed', default_value='0.15',
            description='m/s the descent setpoint is ramped at. PX4 only counts '
                        'this as a descent for its land detector if it is at '
                        'least 0.9 * MPC_LAND_SPEED, so set MPC_LAND_SPEED to '
                        'about 0.2 to match.'),
        DeclareLaunchArgument(
            'move_speed', default_value='0.30',
            description='m/s the horizontal setpoint is walked at. Keep it slow: '
                        'the optical flow is the only thing measuring these moves.'),
        DeclareLaunchArgument(
            'yaw_rate', default_value='0.35',
            description='rad/s the yaw setpoint is walked at (~20 deg/s). Keep it '
                        'slow: a fast yaw smears the optical flow the position '
                        'hold depends on.'),
        DeclareLaunchArgument(
            'min_altitude', default_value='0.4',
            description='m above the arming point that a down step may not go below.'),
        DeclareLaunchArgument(
            'max_altitude', default_value='3.0',
            description='m above the arming point that an up step may not exceed.'),
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
        sequence_node,
    ])
