"""mission_fsm_darkroom_backup: plan, the second ArUco, the sill-safe landing."""
import time

import pytest
import rclpy

from drone_testing.mission_fsm_darkroom_backup import MissionFSMDarkroomBackup
from drone_testing.mission_fsm_full import MissionFSMFull


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class _LP:
    xy_valid = z_valid = v_xy_valid = True
    dist_bottom = 1.80
    dist_bottom_valid = True
    x = 0.0
    y = 0.0
    z = -1.80
    heading = 0.0


class _DS:
    def __init__(self, d, q=100):
        self.current_distance = d
        self.signal_quality = q
        self.min_distance = 0.1
        self.max_distance = 12.0


@pytest.fixture
def fsm():
    n = MissionFSMDarkroomBackup()
    n._still_flyable = lambda: True
    n._try_latch_xy_hold = lambda: None
    n.log_flight_state = lambda: None
    n.publish_status = lambda: None
    n.stream_setpoints = False
    n.rangefinder_is_healthy = lambda: True
    n.local_position = _LP()
    n.hold_xy = True
    n.hold_x = n.hold_y = 0.0
    n.home_z = 0.0
    yield n
    n.destroy_node()


def _inside_clear(n):
    n.phase = n.PHASE_IN
    n._enter_stage(n.CLEAR)
    n.stage_start_time = time.monotonic() - n.CLEAR_SECONDS - 1.0
    n.CLEAR_SECONDS = 0.0


def test_plan_and_defaults(fsm):
    assert [l.name for l in fsm.legs] == ['to_marker', 'offset']
    assert fsm.legs[0].marker_id == fsm.WINDOW_MARKER_ID
    assert fsm.legs[0].nxt == 'window_alt'
    assert fsm.legs[1].direction == 'left'
    assert fsm.legs[1].distance == pytest.approx(0.15)
    # Everything flown is the main mission's, not a copy of it.
    for name in ('CRUISE_ALTITUDE', 'WINDOW_ALTITUDE', 'TRAVERSE_CENTRE_OFFSET',
                 'STRAFE_SPEED', 'TRACK_SPEED', 'FOLLOW_LINE'):
        assert getattr(fsm, name) == getattr(MissionFSMFull, name), name
    for name in ('_handle_takeoff', '_handle_cruise', '_handle_traverse',
                 'publish_position_setpoint', '_begin_leg', '_dispatch_after',
                 '_handle_hold', '_still_flyable'):
        assert name not in MissionFSMDarkroomBackup.__dict__, name


def test_offset_leg_gets_mains_slow_strafe(fsm):
    fsm.MOVE_SPEED = 0.6
    fsm._begin_leg(fsm.leg_by_name['offset'])
    assert fsm.MOVE_SPEED <= fsm.STRAFE_SPEED


def test_pad_marker_ignored_near_the_start(fsm):
    fsm.leg_index = 0
    fsm.leg_started = True
    fsm.current_stage = fsm.CRUISE
    fsm.move_start_x, fsm.move_start_y = 0.0, 0.0
    fsm.marker_is_fresh = lambda: True
    fsm.target_marker_id = fsm.WINDOW_MARKER_ID
    called = []
    import drone_testing.mission_fsm as mf
    orig = mf.MissionFSM.marker_offset_ned
    mf.MissionFSM.marker_offset_ned = lambda self: called.append(1) or (0.1, 0.0)
    try:
        fsm.local_position.x = 1.0
        assert fsm.marker_offset_ned() is None and not called
        fsm.local_position.x = 3.5
        assert fsm.marker_offset_ned() == (0.1, 0.0)
    finally:
        mf.MissionFSM.marker_offset_ned = orig


def test_waits_while_the_tfmini_reads_the_sill(fsm):
    _inside_clear(fsm)
    fsm._distance_sensor_callback(_DS(0.10))     # the sill, not the floor
    fsm._handle_clear()
    ok, why = fsm._floor_check()
    assert not ok and 'raw TFmini' in why
    assert fsm.current_stage == fsm.CLEAR


def test_waits_while_crossing_hold_is_on(fsm):
    _inside_clear(fsm)
    fsm._crossing = True
    fsm._distance_sensor_callback(_DS(1.80))
    assert fsm._floor_check()[0] is False
    fsm._handle_clear()
    assert fsm.current_stage == fsm.CLEAR


def test_lands_slowly_once_the_floor_is_confirmed(fsm):
    _inside_clear(fsm)
    fsm._distance_sensor_callback(_DS(1.80))
    fsm._handle_clear()                          # starts the confirm window
    assert fsm.current_stage == fsm.CLEAR
    fsm.floor_ok_since -= fsm.FLOOR_CONFIRM_S + 0.1
    fsm._distance_sensor_callback(_DS(1.81))
    fsm._handle_clear()
    assert fsm.current_stage == fsm.LANDING
    assert fsm.LAND_SPEED == pytest.approx(0.10)
    assert fsm.hold_xy and fsm.precision_descent_active


def test_lands_anyway_after_the_wait(fsm):
    _inside_clear(fsm)
    fsm._handle_clear()                          # no raw range at all
    fsm.floor_wait_since -= fsm.FLOOR_WAIT_S + 1.0
    fsm._handle_clear()
    assert fsm.current_stage == fsm.LANDING


def test_landing_releases_the_vz_hold(fsm):
    fsm._rng_hold = True
    fsm._rng_bob = time.monotonic()
    fsm._crossing = True
    fsm._begin_landing('test')
    assert not (fsm._rng_hold or fsm._crossing) and fsm._rng_bob is None
    sent = []
    fsm._publish_sp_raw = sent.append

    class _Sp:
        position = [0.0, 0.0, 0.0]
        velocity = [0.0, 0.0, 0.10]
    fsm._rng_hold = True                         # even if something re-sets it
    fsm._publish_sp(_Sp())
    assert sent[0].velocity[2] == pytest.approx(0.10)
