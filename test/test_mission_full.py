"""mission_fsm_full: part 1's leg, no landing on the marker, then part 2."""
import math

import pytest
import rclpy

from drone_testing.mission_fsm_full import MissionFSMFull


class _LP:
    xy_valid = z_valid = v_xy_valid = True
    dist_bottom = 2.10
    dist_bottom_valid = True
    x = y = 0.0
    z = -2.10
    heading = 0.0


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def fsm():
    n = MissionFSMFull()
    n._still_flyable = lambda: True
    n._try_latch_xy_hold = lambda: None
    n.log_flight_state = lambda: None
    n.publish_status = lambda: None
    n._publish_mission_phase = lambda: None
    n.stream_setpoints = False
    n.local_position = _LP()
    n.hold_xy = True
    n.hold_x = n.hold_y = 0.0
    n.home_yaw = 0.0
    n.home_z = 0.0
    n.relative_altitude = lambda: n.home_z - n.local_position.z
    yield n
    n.destroy_node()


def test_plan_is_part1s_leg_then_part2s(fsm):
    assert [leg.name for leg in fsm.legs][:2] == ['to_marker', 'offset']
    assert {'offset_back', 'final', 'exit'} <= set(fsm.leg_by_name)
    leg = fsm.legs[0]
    assert leg.direction == 'forward'
    assert leg.distance == pytest.approx(9.3)
    assert leg.marker_id == 0 and leg.retry is False
    assert fsm.CRUISE_ALTITUDE == pytest.approx(2.50)


def test_hold_starts_the_straight_leg_not_the_strafe(fsm):
    fsm._enter_stage(fsm.HOLD)
    fsm.stage_enter_time -= fsm.HOLD_SECONDS + 1
    fsm._handle_hold()
    assert fsm.current_leg().name == 'to_marker'
    assert fsm._phase_label() == 'TO_MARKER'


@pytest.mark.parametrize('on_marker', [True, False])
def test_marker_is_not_landed_on_it_drops_to_the_window_height(fsm, on_marker):
    landed = []
    fsm._begin_landing = landed.append
    fsm._dispatch_after('window_alt', on_marker=on_marker)
    assert not landed                      # the whole point
    assert fsm.current_stage == fsm.ALT_CHANGE
    assert fsm.alt_target == pytest.approx(1.75)
    assert fsm.alt_next == 'offset'
    fsm._dispatch_after('offset')          # what ALT_CHANGE does when done
    assert fsm.current_leg().name == 'offset'


def test_part2_behaviour_survives(fsm):
    assert fsm.EXIT_MODE == 'detect'
    assert fsm.TRAVERSE_CENTRE_OFFSET == pytest.approx(0.07)   # 7 cm above centre
    assert fsm.WINDOW_OFFSET == pytest.approx(0.15)
    assert fsm.LIDAR_CLOSED_LOOP is True
    # the final leg still lands on the marker
    assert fsm.legs[fsm.leg_by_name['final']].marker_id == 0


def test_climb_is_held_on_the_pad_marker(fsm):
    """Flow is useless below flow_min_agl; the marker under us is not."""
    fsm.hold_xy = False
    fsm.marker_offset_ned = lambda: (0.22, -0.13, 2.0)   # 26 cm away
    import drone_testing.mission_fsm as mf
    mro = type(fsm).__mro__
    base = mro[mro.index(mf.MissionFSM) + 1]          # whatever super() is
    called = []
    orig = base._handle_takeoff
    base._handle_takeoff = lambda self: called.append(1)
    try:
        fsm._handle_takeoff()
    finally:
        base._handle_takeoff = orig
    assert fsm.hold_xy and called
    assert fsm.hold_x == pytest.approx(0.22) and fsm.hold_y == pytest.approx(-0.13)
    assert fsm.target_marker_id == fsm.WINDOW_MARKER_ID
    # a same-id pad across the cage is NOT this one
    fsm.hold_xy = False
    fsm.marker_offset_ned = lambda: (4.0, 0.0, 2.0)
    base._handle_takeoff = lambda self: None
    try:
        fsm._handle_takeoff()
    finally:
        base._handle_takeoff = orig
    assert not fsm.hold_xy


def test_strafe_is_flown_slowly(fsm):
    fsm.LEG_SPEED = 0.6
    fsm._begin_leg(fsm.leg_by_name['offset'])
    assert fsm.MOVE_SPEED == pytest.approx(fsm.STRAFE_SPEED)
    fsm._begin_leg(fsm.leg_by_name['to_marker'])
    assert fsm.MOVE_SPEED == pytest.approx(0.6)




def test_first_metre_is_climbed_gently(fsm):
    """Ground effect is worst and the flow useless in the first metre."""
    import drone_testing.mission_fsm as mf
    mro = type(fsm).__mro__
    base = mro[mro.index(mf.MissionFSM) + 1]
    orig = base._handle_takeoff
    base._handle_takeoff = lambda self: None
    fsm.marker_offset_ned = lambda: None
    fsm.CLIMB_SPEED = 0.8
    try:
        fsm.local_position.z = -0.4          # 0.4 m up
        fsm._handle_takeoff()
        assert fsm.CLIMB_SPEED == pytest.approx(fsm.GENTLE_CLIMB_SPEED)
        fsm.local_position.z = -1.6          # past the first metre
        fsm._handle_takeoff()
        assert fsm.CLIMB_SPEED == pytest.approx(0.8)
    finally:
        base._handle_takeoff = orig


def test_a_second_flight_node_is_reported(fsm):
    """Two setpoint publishers = a race, and a start that looks hung."""
    said = []
    fsm.get_logger().error = lambda m, **k: said.append(m)
    fsm.count_publishers = lambda t: 2          # someone else, plus us
    fsm._check_for_a_second_flight_node()
    assert said and 'ANOTHER FLIGHT NODE' in said[0]
    assert '/uav_2/fmu/in/trajectory_setpoint' in said[0]
    said.clear()
    fsm.count_publishers = lambda t: 1          # just us
    fsm._dup_timer = fsm.create_timer(3.0, lambda: None)
    fsm._check_for_a_second_flight_node()
    assert not said


def test_the_pilot_owns_mode_and_arming(fsm):
    """Offboard and the arm come from the TX (or the dashboard), never here."""
    from px4_msgs.msg import VehicleCommand, VehicleStatus
    assert fsm.REQUEST_OFFBOARD_FROM_ROS is False
    assert fsm.ARM_FROM_ROS is False
    sent = []
    fsm.publish_vehicle_command = lambda cmd, **kw: sent.append((cmd, kw))
    fsm.arming_state = VehicleStatus.ARMING_STATE_DISARMED
    fsm.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    fsm._enter_stage(fsm.ARMING)
    fsm.stage_enter_time -= 60           # long past any arming timeout
    for _ in range(3):
        fsm._handle_arming()
    assert not sent                      # no ARM command, ever
    assert fsm.current_stage == fsm.ARMING and not fsm.kill_requested
    # and it proceeds the moment the pilot arms
    fsm.arming_state = VehicleStatus.ARMING_STATE_ARMED
    fsm._capture_home = lambda: None
    fsm._handle_arming()
    assert fsm.current_stage == fsm.GROUND_WAIT


def _lane_node(width_norm, cross=0.0, height=2.5):
    import time
    from drone_testing.mission_fsm_full import MissionFSMFull
    n = MissionFSMFull()
    n.local_position = _LP()
    n.local_position.dist_bottom = height
    n.local_position.dist_bottom_valid = True
    n._still_flyable = lambda: True
    n._try_latch_xy_hold = lambda: None
    n.log_flight_state = lambda: None
    n.hold_xy = True
    n.marker_offset_ned = lambda: None
    n._begin_cruise_move = lambda leg: None
    n.leg_index = n.leg_by_name['to_marker']
    n.leg_started = True
    n.moving = True
    n.move_start_x = n.move_start_y = 0.0
    n.move_target_x, n.move_target_y = 9.3, 0.0      # due north
    n.local_position.x, n.local_position.y = 1.0, cross   # cross = east offset
    n.line_fix = (time.monotonic(), 0.0, 0.0, width_norm)
    return n


def _width_norm(width_m, height=2.5, hfov=70.4):
    frame = 2.0 * height * math.tan(math.radians(hfov) / 2.0)
    return width_m / frame


def test_our_lane_is_followed(fsm=None):
    n = _lane_node(_width_norm(0.60))          # our 0.60 m lane
    try:
        for _ in range(4):
            n._handle_cruise()
        assert n._track_cmd is not None
    finally:
        n.destroy_node()


def test_the_wider_centre_lane_is_refused():
    """The centre lane carries the obstacles: 1.20 m against our 0.60 m."""
    n = _lane_node(_width_norm(1.20))
    try:
        for _ in range(4):
            n._handle_cruise()
        assert n._track_cmd is None
    finally:
        n.destroy_node()


def test_a_lane_spacing_off_the_leg_line_is_refused():
    """Right width, but a lane spacing (1.50 m) off the line we started on."""
    n = _lane_node(_width_norm(0.60), cross=1.50)
    try:
        for _ in range(4):
            n._handle_cruise()
        assert n._track_cmd is None
    finally:
        n.destroy_node()


def test_a_climb_that_drifted_to_the_next_lane_is_caught():
    """The leg line is the PAD's lane, not wherever the climb ended up."""
    n = _lane_node(_width_norm(0.60))          # right width: width check blind
    try:
        n.pad_marker_ned = (0.0, 0.0)          # the pad, on our lane
        n.local_position.x, n.local_position.y = 1.0, 1.50   # a lane over
        for _ in range(4):
            n._handle_cruise()
        assert n._track_cmd is None            # refused: not our lane
        assert n._leg_line[:2] == (0.0, 0.0)
    finally:
        n.destroy_node()


def test_without_a_pad_fix_the_line_falls_back_to_the_leg_start():
    n = _lane_node(_width_norm(0.60))
    try:
        n.pad_marker_ned = None
        n.local_position.x, n.local_position.y = 1.0, 1.50
        # the leg really does start from where the drifted climb left it
        n.move_start_x, n.move_start_y = 1.0, 1.50
        n.move_target_x, n.move_target_y = 10.3, 1.50
        for _ in range(4):
            n._handle_cruise()
        # anchored at the (drifted) start, so cross-track is zero and only
        # the width check is left -- which this frame passes
        assert n._track_cmd is not None
    finally:
        n.destroy_node()
