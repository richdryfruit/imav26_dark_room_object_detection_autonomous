"""mission_fsm_part2's 'retrace' exit: no camera relock from inside."""
import time

import pytest
import rclpy

from drone_testing.mission_fsm_part2 import MissionFSMPart2


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class _LP:
    xy_valid = z_valid = v_xy_valid = True
    dist_bottom = 1.75
    dist_bottom_valid = True
    x = 0.0
    y = 0.0
    z = -1.75
    heading = 3.14159


@pytest.fixture
def fsm():
    n = MissionFSMPart2()
    n._still_flyable = lambda: True
    n._try_latch_xy_hold = lambda: None
    n.log_flight_state = lambda: None
    n.publish_status = lambda: None
    n._publish_mission_phase = lambda: None
    n.stream_setpoints = False
    n.flow_is_healthy = lambda: True
    n.local_position = _LP()
    n.hold_xy = True
    n.hold_x = n.hold_y = 0.0
    n.home_yaw = 0.0
    n.home_z = 0.0
    n.relative_altitude = lambda: n.home_z - n.local_position.z
    yield n
    n.destroy_node()


def test_exit_mode_defaults_to_retrace(fsm):
    assert fsm.EXIT_MODE == 'retrace'
    leg = fsm.legs[fsm.leg_by_name['exit']]
    assert leg.direction == 'backward' and leg.marker_id is None
    assert leg.distance == pytest.approx(fsm.INSIDE_DISTANCE + fsm.OUTSIDE_DISTANCE)


def test_retrace_returns_to_the_inbound_height_then_flies_out(fsm):
    fsm.target_z = -1.95                 # inbound traverse height (NED)
    fsm._begin_room = lambda: None       # only the capture matters here
    fsm.inbound_z = -1.95
    fsm.inbound_hagl = 1.95                 # TFmini floor distance inbound
    fsm.local_position.dist_bottom = 1.75   # floor distance now (EKF says 1.75)
    fsm._begin_relock()
    assert fsm.phase == fsm.PHASE_OUT
    assert fsm.current_stage == fsm.ALT_CHANGE
    # 1.75 (EKF now) + (1.95 - 1.75) on the rangefinder
    assert fsm.alt_target == pytest.approx(1.95)
    assert fsm.alt_next == 'exit'
    fsm._dispatch_after('exit')
    assert fsm.current_leg().name == 'exit'


def test_exit_leg_ends_in_the_outside_hold_then_the_mirrored_strafe(fsm):
    fsm.phase = fsm.PHASE_OUT
    fsm._begin_leg_named('exit')
    fsm._handle_cruise()                 # starts the move
    fsm.local_position.x, fsm.local_position.y = fsm.move_target_x, fsm.move_target_y
    fsm._handle_cruise()
    fsm.move_in_band_since = time.monotonic() - 100
    fsm._handle_cruise()
    assert fsm.current_stage == fsm.CLEAR
    fsm.stage_enter_time = time.monotonic() - 100
    fsm._handle_clear()
    assert fsm.current_leg().name == 'offset_back'


def test_relock_mode_keeps_the_inherited_camera_relock(fsm):
    fsm.EXIT_MODE = 'relock'
    fsm._begin_relock()
    assert fsm.current_stage == fsm.RELOCK


class _Msg:
    def __init__(self):
        self.position = [1.0, 2.0, -1.75]
        self.velocity = [float('nan')] * 3


def test_wall_crossing_flies_z_on_velocity_then_reanchors(fsm):
    sent = []
    fsm._publish_sp_raw = sent.append
    fsm._enter_stage(fsm.TRAVERSE)
    fsm._update_crossing()
    assert fsm._crossing
    m = _Msg()
    fsm._publish_sp(m)
    assert m.position[2] != m.position[2]          # NaN: z not position-held
    assert m.velocity[2] == 0.0
    assert m.position[:2] == [1.0, 2.0]            # x/y untouched
    # a height blip mid-crossing gets the wide grace, not an instant landing
    assert fsm.HEIGHT_INVALID_GRACE < fsm.CROSSING_GRACE
    # past the wall: needs crossing_settle_s of valid height, then re-anchors
    fsm._enter_stage(fsm.CLEAR)
    fsm.local_position.z = -1.42
    fsm._update_crossing()
    assert fsm._crossing
    fsm._crossing_valid_since = time.monotonic() - 10
    fsm._update_crossing()
    assert not fsm._crossing
    assert fsm.target_z == pytest.approx(-1.42)
    m2 = _Msg()
    fsm._publish_sp(m2)
    assert m2.position[2] == -1.75                 # normal again


def test_a_landing_during_a_crossing_releases_the_hold(fsm):
    fsm._enter_stage(fsm.TRAVERSE)
    fsm._update_crossing()
    assert fsm._crossing
    fsm._enter_stage(fsm.LANDING)
    fsm._update_crossing()
    assert not fsm._crossing


def test_blind_push_is_bounded_by_the_remaining_distance(fsm):
    fsm.TRAVERSE_SPEED = 0.6
    fsm.BLIND_TRAVERSE_SECONDS = 8.0
    fsm.traverse_standoff, fsm.EXIT_DISTANCE = 2.71, 1.76
    fsm._distance_along_traverse = lambda: 3.15      # 1.32 m left
    fsm._enter_stage(fsm.TRAVERSE)
    fsm.blind_traverse_since = None
    fsm._handle_blind_traverse()
    assert fsm.blind_traverse_limit == pytest.approx(1.32 / 0.6 + 0.5)
    assert fsm.blind_traverse_limit < 8.0


def test_blind_push_done_inbound_goes_on_into_the_room_not_down(fsm):
    fsm.phase = fsm.PHASE_IN
    fsm.blind_traverse_left = 1.3
    fsm.blind_traverse_limit = 2.7
    landed = []
    fsm._begin_landing = landed.append
    fsm._blind_push_done(4.47)
    assert fsm.current_stage == fsm.CLEAR and not landed


def test_exit_height_is_range_relative_not_the_reset_ekf_datum(fsm):
    """After the sill resets EKF2's height, its altitude is offset; the exit
    must still come back to the inbound height ABOVE THE FLOOR."""
    fsm.inbound_hagl = 3.78
    fsm.local_position.z = -3.21            # EKF: 3.21 m (datum shifted)
    fsm.local_position.dist_bottom = 3.60   # truth on the TFmini: 3.60 m
    fsm._begin_relock()
    assert fsm.alt_target == pytest.approx(3.21 + (3.78 - 3.60))


def test_inbound_floor_height_is_captured_at_the_crossing(fsm):
    fsm.phase = fsm.PHASE_IN
    fsm.local_position.dist_bottom = 3.78
    fsm._enter_stage(fsm.TRAVERSE)
    fsm._update_crossing()
    assert fsm.inbound_hagl == pytest.approx(3.78)
