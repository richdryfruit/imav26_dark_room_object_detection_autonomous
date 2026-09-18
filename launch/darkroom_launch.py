"""
Dark room: in through one window -> a sequence you supply -> out through
ANOTHER window.

    ros2 launch drone_testing darkroom_launch.py agent_only:=false \
        room_sequence:="forward 0.8, yaw 90, right 0.4, backward 0.5, yaw 90"

WHY THIS FILE IS THIN
---------------------
The mission is already implemented. window_room_traverse.py is a subclass of
window_traverse.py that flies the traversal TWICE with a pattern in between,
and it already takes the in-room plan as a free-form string (`room_sequence`,
the same grammar offboard_sequence parses). So nothing here re-implements a
flight stage, and this file does NOT declare the ~90 traversal, estimator and
clearance arguments: it INCLUDES window_room_traverse.launch.py with
forwarding on, so every one of those keeps exactly one definition, in the file
that already flies. See that file's header for the full stack.

What this file adds is the two things a DARK room changes:

    1. Camera exposure. Auto-exposure hunts badly when the only bright thing
       in frame is the window itself, which is precisely the situation here:
       the room is dark, the window is a hole with light behind it. The colour
       sensor is pinned to a fixed exposure and gain after the driver is up
       (item 2 below), so the frame the HSV threshold sees does not change
       brightness every time the aircraft yaws.

    2. The IR emitter matters more, not less. Passive stereo has nothing to
       correlate in an unlit room, so the projector is what produces depth at
       all -- and depth is what the corner sampling and the whole pose
       estimate are built on. It is ON by default here and you should leave it
       on; `emitter:=0` in a dark room is how the window pose stops existing.

HOW THE WAY OUT FINDS A *DIFFERENT* WINDOW
------------------------------------------
This is the part worth understanding before flying it, because it is not a
new code path -- it is an existing one used deliberately.

At the end of the room sequence, window_room_traverse._begin_relock() throws
away everything it knew about the window it came in through: the estimator
samples, the last-good pose, the latched traverse geometry. It then re-centres
the yaw cone on WHATEVER HEADING THE SEQUENCE ENDED ON and rebuilds a window
pose from scratch out of the frames it is looking at right then.

So the exit window is simply "the window in front of the aircraft when the
sequence finishes". Nothing anywhere records that the entry window and the
exit window are the same hole. To leave through a different one:

    >>> END YOUR SEQUENCE FACING THE WINDOW YOU WANT TO LEAVE THROUGH. <<<

RELOCK does NOT sweep to search -- deliberately, because yawing to search from
a metre inside a room is how it locks onto a doorway instead of a window. It
stands still and looks. If the exit window is not in the camera's view when
the last step of your sequence completes, RELOCK times out
(`relock_timeout`, 45 s) and the aircraft lands inside the room.

Two consequences:

    - Budget a final `yaw` step to point at the exit window. If the exit
      window is in the wall to the aircraft's right when the pattern ends,
      the sequence must end with `yaw 90`.
    - `yaw_cone_deg` is re-centred on that final heading, so the cone does not
      need widening for the second window. It needs your sequence to be right.

THE SEQUENCE GRAMMAR
--------------------
Comma-separated `name number` pairs, parsed by parse_sequence() in
offboard_sequence.py. Accepted names (with aliases):

    forward / fwd / f        metres
    backward / back / b      metres
    left / l                 metres        (strafe, airframe stays pointed)
    right / r                metres        (strafe)
    yaw / turn / rotate      DEGREES, positive = right/clockwise from above

Distances are always positive -- use the opposite word rather than a negative
number; a negative distance is a fatal parse error, not a guess.

NOT accepted inside the room, and each for a reason:

    roll            Not a motion in this grammar at all, and not a thing a
                    multirotor on position setpoints can be commanded to do
                    independently: roll IS how it translates sideways. If you
                    want the aircraft to move sideways, that is `left` or
                    `right`. A literal `roll 20` is a fatal parse error
                    ("unknown motion 'roll'").
    up / down       Rejected by window_room_traverse specifically. The room
                    stages dispatch only to _handle_room_move and
                    _handle_room_turn; there is no altitude handler among
                    them, and changing height inside the room changes the
                    height the RETURN APPROACH lines up at. Use
                    `altitude_offset` for that instead.

A malformed sequence is FATAL at node construction, before the motors spin --
a typo in a flight plan is not quietly guessed at.

An empty `room_sequence` (the default) falls back to the inherited six-leg box
(0.30 left, 0.30 forward, yaw 90 right, 0.30 forward, yaw 90 right, 0.30
forward), which ends facing the wall it came in through -- i.e. it leaves
through the SAME window. That is the safe default, not the mission described
above; set `room_sequence` to get a different exit.

USEFUL COMBINATIONS
-------------------
    agent_only:=false            fly it. The default (true) starts the agent,
                                 camera and detector but NOT the flight node,
                                 so you can run that by hand and keep the
                                 keyboard abort.
    flight:=false                camera + detector only. No agent, no flight
                                 node: the bench test, and how you check the
                                 darkroom exposure values below by eye before
                                 trusting them in the air.
    return_through_window:=false fly in, run the sequence, land inside. Fly
                                 this FIRST in a new room: it answers "does
                                 the sequence fit" without also asking "can it
                                 find the second window".
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- the whole mission, from the file that already flies it -----------
    #
    # Forwarding is left ON (the default), so every argument declared below --
    # and every argument NOT declared below, which is most of them -- resolves
    # in that file against its own defaults. This file overrides nothing it
    # does not have a darkroom-specific reason to override.
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('drone_testing'),
            'launch', 'window_room_traverse.launch.py'])),
    )

    # ---- the darkroom camera --------------------------------------------
    #
    # Set by `ros2 param set` AFTER the driver is up, for the same reason the
    # emitter is in window_scan.launch.py / window_traverse.launch.py: these
    # are runtime ROS parameters on the camera node, not rs_launch.py launch
    # arguments, so passing them at include time sets nothing and only earns
    # an "unsupported parameter" warning.
    #
    # The delay must clear the driver's own startup. librealsense took 8.7 s
    # to reach "RealSense Node Is Up!" on this Jetson and `ros2 param set`
    # fails outright against a node that does not exist yet -- and it fails
    # SILENTLY as far as the flight is concerned, because the camera simply
    # keeps its default auto-exposure and the first you know is a washed-out
    # window. 14 s: two seconds more margin than the 12 s used elsewhere,
    # because this file sets four parameters in series rather than one.
    camera_delay = LaunchConfiguration('darkroom_camera_delay')
    cam_node = ['/', LaunchConfiguration('camera_namespace'),
                '/', LaunchConfiguration('camera_name')]

    def param_set(name, value):
        return ExecuteProcess(
            cmd=['ros2', 'param', 'set', cam_node, name, value],
            output='screen',
        )

    darkroom_camera = TimerAction(
        period=camera_delay,
        actions=[
            # Colour first: this is the stream the HSV threshold runs on, and
            # the one auto-exposure gets wrong here. With a bright window in a
            # dark room, auto-exposure meters for the whole frame, opens up to
            # expose the dark walls, and blows the window out to white -- at
            # which point the frame has no hue in the region that matters and
            # the threshold finds nothing. Pinning it exposes FOR the window
            # and lets the room go black, which is the correct trade: the
            # detector only ever cares about the window.
            param_set('rgb_camera.enable_auto_exposure',
                      LaunchConfiguration('rgb_auto_exposure')),
            param_set('rgb_camera.exposure', LaunchConfiguration('rgb_exposure')),
            param_set('rgb_camera.gain', LaunchConfiguration('rgb_gain')),
            # Depth auto-exposure is left ALONE by default (auto), because the
            # projector gives the depth sensor its own light and the usual
            # dark-room failure is on the colour side. Exposed as an argument
            # anyway: if the depth image is noisy at the window edge, this is
            # the knob, not the colour one.
            param_set('depth_module.enable_auto_exposure',
                      LaunchConfiguration('depth_auto_exposure')),
        ],
        condition=IfCondition(LaunchConfiguration('darkroom_camera')),
    )

    return LaunchDescription([
        # ---- the mission ------------------------------------------------
        DeclareLaunchArgument(
            'room_sequence', default_value='',
            description='The in-room plan, comma separated: '
                        '"forward 0.8, yaw 90, right 0.4, backward 0.5". '
                        'forward/backward/left/right in metres, yaw in '
                        'degrees (positive = right). No roll (use left/right) '
                        'and no up/down (use altitude_offset). MUST END '
                        'FACING THE WINDOW YOU WANT TO LEAVE THROUGH -- '
                        'RELOCK does not sweep to search. Empty = the '
                        'inherited six-leg box, which exits through the '
                        'window it entered by.'),

        # ---- the darkroom camera ----------------------------------------
        DeclareLaunchArgument(
            'darkroom_camera', default_value='true',
            description='Apply the fixed-exposure darkroom camera settings '
                        'after the driver is up. false = leave the camera on '
                        'its own auto-exposure, i.e. behave exactly like '
                        'window_room_traverse.launch.py.'),
        DeclareLaunchArgument(
            'rgb_auto_exposure', default_value='false',
            description='Colour auto-exposure. OFF for a dark room: a bright '
                        'window in a dark frame is the case auto-exposure '
                        'gets worst, and it blows the window to white.'),
        DeclareLaunchArgument(
            'rgb_exposure', default_value='120',
            description='Colour exposure in units of 100 us (120 = 12 ms) '
                        'when rgb_auto_exposure is false. CHECK THIS BY EYE '
                        'with flight:=false before trusting it: the right '
                        'value depends on how much light is actually coming '
                        'through your window, and it is the single most '
                        'likely thing in this file to be wrong for your room.'),
        DeclareLaunchArgument(
            'rgb_gain', default_value='64',
            description='Colour sensor gain. Raise before raising exposure: '
                        'a longer exposure motion-blurs the frame during the '
                        'sweep, and a blurred window edge moves the corners '
                        'the pose is built from.'),
        DeclareLaunchArgument(
            'depth_auto_exposure', default_value='true',
            description='Depth auto-exposure. Left ON: the IR projector gives '
                        'the depth sensor its own light, so the dark room is '
                        'not its problem.'),
        DeclareLaunchArgument(
            'darkroom_camera_delay', default_value='14.0',
            description='Seconds to wait for the camera node before setting '
                        'its parameters. `ros2 param set` fails outright if '
                        'the node is not up yet.'),

        # These two only exist here so the `ros2 param set` target above can
        # be built. They are also declared by the include, and forwarding
        # means these defaults are the ones that apply -- so they must match
        # window_traverse.launch.py's, or the param set aims at a node that
        # does not exist while the camera itself comes up under another name.
        DeclareLaunchArgument(
            'camera_name', default_value='camera',
            description="RealSense node name; must match the include's."),
        DeclareLaunchArgument(
            'camera_namespace', default_value='camera',
            description="RealSense namespace; must match the include's."),

        mission,
        darkroom_camera,
    ])
