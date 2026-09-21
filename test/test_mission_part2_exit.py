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
    fsm._begin_relock()
    assert fsm.phase == fsm.PHASE_OUT
    assert fsm.current_stage == fsm.ALT_CHANGE
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
