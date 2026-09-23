"""mission_fsm_part2's 'detect' exit and lidar-closed room moves.

Arena transform fixed at identity (theta 0): arena +X = east, +Y = north, so
NED (n, e) = (arena y, arena x). The window wall is the south wall, y = -1.25
in a 2.5 m room; facing it from inside is NED heading pi.
"""
import math
import time

import pytest
import rclpy

from drone_testing.mission_fsm_part2 import MissionFSMPart2

WALL_Y = -1.25


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class _LP:
    xy_valid = z_valid = v_xy_valid = True
    dist_bottom = 1.85
    dist_bottom_valid = True
    x = -0.25       # NED north = arena y
    y = 0.10        # NED east  = arena x
    z = -1.85
    heading = math.pi


class _Scan:
    def __init__(self, fix, x0, x1):
        fx, fy, psi = fix
        self.angle_min = -math.pi
        self.angle_increment = 2 * math.pi / 360
        self.ranges = []
        for i in range(360):
            d = psi + self.angle_min + i * self.angle_increment
            s = math.sin(d)
            if s >= -1e-6:
                self.ranges.append(2.0)
                continue
            t = (WALL_Y - fy) / s
            hx = fx + t * math.cos(d)
            self.ranges.append(float('inf') if x0 <= hx <= x1 else t)


@pytest.fixture
def fsm():
    n = MissionFSMPart2()
    n._still_flyable = lambda: True
    n._try_latch_xy_hold = lambda: None
    n.log_flight_state = lambda: None
    n.publish_status = lambda: None
    n._publish_mission_phase = lambda: None
    n.stream_setpoints = False
    n.local_position = _LP()
    n.hold_xy = True
    n.hold_x, n.hold_y = n.local_position.x, n.local_position.y
    n.home_yaw = 0.0
    n.home_z = 0.0
    n.yaw_setpoint = math.pi
    n.ROOM_X = n.ROOM_Y = 2.5
    n.update_transform = lambda: None
    n.arena_tf = (0.0, 0.0, 0.0, 0.0)
    n.lidar_fix = (0.10, -0.25, 1.9, -math.pi / 2)
    n.lidar_fix_time = n._now()
    n.lidar_fix_ekf = (n.local_position.x, n.local_position.y)
    n.arena_entry = (0.10, -0.45)
    n.inbound_hagl = 1.85
    n.inbound_window = (0.60, 0.60)
    n.DRONE_WIDTH = 0.26
    n.RELOCK_STANDOFF = 1.0
    yield n
    n.destroy_node()


def test_lidar_goal_cancels_ekf_drift(fsm):
    # EKF2 has wandered 5 m; the lidar says we are at arena (0.10, -0.25).
    fsm.lidar_fix_ekf = (5.0, 5.0)
    n, e, _ = fsm.arena_to_ned(0.30, -0.25)
    assert (n, e) == pytest.approx((5.0, 5.2))
    fsm.LIDAR_CLOSED_LOOP = False
    n, e, _ = fsm.arena_to_ned(0.30, -0.25)
    assert (n, e) == pytest.approx((-0.25, 0.30))      # transform only


def test_facing_the_window_from_inside(fsm):
    assert fsm._window_facing_ned() == pytest.approx(math.pi)


def _run_find(fsm, cues, ticks):
    fsm.exit_cues_now = lambda: cues
    for _ in range(ticks):
        fsm.exit_last_eval = 0.0
        fsm._handle_exit_find()


def test_two_agreeing_cues_start_the_align(fsm):
    fsm._begin_exit_find()
    assert fsm.current_stage == fsm.EXIT_FIND
    cues = {'lidar': {'x': 0.12, 'width': 0.60}, 'depth': {'x': 0.16, 'width': 0.62},
            'bright': None}
    _run_find(fsm, cues, fsm.EXIT_CONFIRM - 1)
    assert fsm.current_stage == fsm.EXIT_FIND
    _run_find(fsm, cues, 1)
    assert fsm.current_stage == fsm.EXIT_ALIGN
    assert fsm.exit_gap_x == pytest.approx(0.12)       # the lidar's value
    assert sorted(fsm.exit_cues) == ['depth', 'lidar']
    # same height above the floor as the way in
    assert fsm.target_z == pytest.approx(-1.85)


def test_one_cue_is_not_enough_until_the_entry_axis_backs_it(fsm):
    fsm._begin_exit_find()
    cues = {'lidar': {'x': 0.12, 'width': 0.60}, 'depth': None, 'bright': None}
    _run_find(fsm, cues, 10)
    assert fsm.current_stage == fsm.EXIT_FIND
    fsm.stage_enter_time = time.monotonic() - fsm.EXIT_FIND_TIMEOUT - 1
    _run_find(fsm, cues, 1)
    assert fsm.current_stage == fsm.EXIT_ALIGN
    assert fsm.exit_cues == ['lidar', 'entry axis']


def test_align_waits_for_lateral_height_and_yaw(fsm):
    fsm._begin_exit_find()
    fsm._begin_exit_align(0.30, 0.60, ['lidar', 'depth'], {})
    fsm.exit_scan = None
    fsm._handle_exit_align()             # 20 cm off laterally
    assert fsm.exit_in_band_since is None
    fsm.lidar_fix = (0.30, -0.25, 1.9, -math.pi / 2)
    fsm._handle_exit_align()
    fsm.exit_in_band_since = time.monotonic() - 10
    fsm._handle_exit_align()
    assert fsm.current_stage == fsm.EXIT_TRAVERSE
    assert fsm.exit_total == pytest.approx(1.0 + fsm.OUTSIDE_DISTANCE)


def test_traverse_stops_if_the_live_gap_does_not_fit_the_airframe(fsm):
    fsm._begin_exit_find()
    fsm._begin_exit_align(0.10, 0.60, ['lidar', 'depth'], {})
    fsm._begin_exit_traverse()
    # The real gap is centred at x = 0.45: we are 35 cm off, 17 cm of air.
    fsm.exit_scan = (time.monotonic(), _Scan((0.10, -0.25, -math.pi / 2), 0.15, 0.75))
    fsm._handle_exit_traverse()
    assert fsm.current_stage == fsm.EXIT_ALIGN
    assert fsm.exit_gap_x == pytest.approx(0.45, abs=0.03)


def test_traverse_steers_on_lidar_then_freezes_then_clears(fsm):
    fsm._begin_exit_find()
    fsm._begin_exit_align(0.10, 0.60, ['lidar', 'depth'], {})
    fsm._begin_exit_traverse()
    fsm.exit_scan = (time.monotonic(), _Scan((0.10, -0.25, -math.pi / 2), -0.20, 0.40))
    fsm._handle_exit_traverse()
    assert fsm.current_stage == fsm.EXIT_TRAVERSE and fsm.exit_frozen is None
    assert fsm._crossing_now()           # z on velocity through the wall
    # at the wall plane: lidar steering stops, the line is frozen
    fsm.lidar_fix = (0.10, WALL_Y + 0.3, 1.9, -math.pi / 2)
    fsm._handle_exit_traverse()
    assert fsm.exit_frozen is not None
    lp = fsm.local_position
    lp.x = fsm.exit_start[0] - fsm.exit_total
    fsm._handle_exit_traverse()
    assert fsm.current_stage == fsm.CLEAR


def test_half_turn_only_after_reaching_the_station(fsm):
    fsm.room_entry_heading = 0.0
    fsm._enter_stage(fsm.TILE_RETURN)
    fsm.lidar_fix = (0.10, 0.60, 1.9, math.pi / 2)    # far from the station
    fsm.yaw_setpoint = 0.0
    fsm._handle_tile_return()
    assert not fsm._return_turning and fsm.yaw_remaining == 0.0
    sx, sy = fsm.relock_station_arena()
    fsm.lidar_fix = (sx, sy, 1.9, math.pi / 2)
    fsm._handle_tile_return()
    assert fsm._return_turning
    fsm._handle_tile_return()
    assert abs(fsm.yaw_remaining) == pytest.approx(math.pi)


def test_backoffs_share_one_budget_and_go_slowly(fsm):
    fsm.window_backoffs = 1
    fsm.recentre_backoffs = 1
    reasons = []
    fsm._abandon = reasons.append
    fsm._begin_window_backoff()
    assert reasons and 'budget 2' in reasons[0]
    fsm.window_backoffs = 0
    fsm.recentre_backoffs = 0
    fsm._handle_hold = lambda: None
    fsm._begin_window_backoff()
    assert fsm.MOVE_SPEED == pytest.approx(fsm.BACKOFF_SPEED)
    assert fsm.APPROACH_SPEED != fsm.BACKOFF_SPEED


def test_room_keeps_the_traverse_height(fsm):
    fsm.ROOM_MODE = 'lidar'
    fsm.target_z = -1.95
    fsm._begin_room()
    assert fsm.current_stage == fsm.TILE_SETTLE
    assert fsm.target_z == pytest.approx(-1.95)


# ------------------------------------------------ rangefinder dropout

def _armed_offboard(fsm):
    from px4_msgs.msg import VehicleStatus
    fsm.arming_state = VehicleStatus.ARMING_STATE_ARMED
    fsm.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD


def test_rangefinder_dropout_inside_holds_and_bobs_never_lands(fsm):
    _armed_offboard(fsm)
    del fsm._still_flyable                      # the real one
    landed = []
    fsm._begin_landing = landed.append
    fsm._flyable_base = lambda: True
    fsm.rangefinder_is_healthy = lambda: False
    fsm._enter_stage(fsm.TILE_DWELL)
    fsm.moving = True
    assert fsm._still_flyable() is False        # hold: the stage pauses
    assert fsm._rng_hold and not fsm.moving and not landed
    fsm._rng_bad_since -= 2.0                   # after a second: bob
    fsm._still_flyable()
    assert fsm._bob_vz() == pytest.approx(fsm.RNG_BOB_SPEED)
    fsm._rng_bob -= fsm.RNG_BOB_TIME + 0.01
    assert fsm._bob_vz() == pytest.approx(-fsm.RNG_BOB_SPEED)
    # published: z on velocity
    m = type('M', (), {})()
    m.position, m.velocity = [0.0, 0.0, -1.8], [0.0, 0.0, 0.0]
    sent = []
    fsm._publish_sp_raw = sent.append
    fsm._publish_sp(m)
    assert m.position[2] != m.position[2]
    # never recovers: heads for the way out, still no landing
    fsm._rng_bad_since -= fsm.RNG_RECOVER_S
    fsm._still_flyable()
    assert fsm.current_stage == fsm.TILE_RETURN and not landed
    assert fsm._still_flyable() is True         # flying on, z on velocity
    # comes back: altitude re-anchored where it is, hold released
    fsm.rangefinder_is_healthy = lambda: True
    fsm.local_position.z = -1.62
    fsm._still_flyable()
    assert not fsm._rng_hold and fsm.target_z == pytest.approx(-1.62)


def test_rangefinder_dropout_mid_traverse_carries_on(fsm):
    _armed_offboard(fsm)
    del fsm._still_flyable
    fsm._flyable_base = lambda: True
    fsm.rangefinder_is_healthy = lambda: False
    fsm._enter_stage(fsm.TRAVERSE)
    fsm.moving = True
    assert fsm._still_flyable() is True and fsm.moving and fsm._rng_hold


def test_rangefinder_dropout_outside_lands_after_the_timeout(fsm):
    _armed_offboard(fsm)
    del fsm._still_flyable
    landed = []
    fsm._begin_landing = landed.append
    fsm._flyable_base = lambda: True
    fsm.rangefinder_is_healthy = lambda: False
    fsm.phase = fsm.PHASE_OUTSIDE
    fsm._enter_stage(fsm.HOLD)
    assert fsm._still_flyable() is False and not landed
    fsm._rng_bad_since -= fsm.RNG_RECOVER_S + 1
    fsm._still_flyable()
    assert landed


# ------------------------------------------------ the window on the lidar

def _body_scan(d, lo, hi):
    inc = 2 * math.pi / 360

    class M:
        pass
    m = M()
    m.angle_min, m.angle_increment, m.ranges = -math.pi, inc, []
    for i in range(360):
        a = -math.pi + i * inc
        c = math.cos(a)
        if c <= 1e-3:
            m.ranges.append(float('inf'))
            continue
        t = d / c
        s = t * math.sin(a)
        m.ranges.append(float('inf') if lo <= s <= hi else t)
    return m


def test_camera_window_corrected_by_the_lidar_gap(fsm):
    import numpy as np
    fsm.phase = fsm.PHASE_OUTSIDE
    fsm._enter_stage(fsm.ALIGN)
    lp = fsm.local_position
    lp.x, lp.y, lp.heading = 0.0, 0.0, 0.0           # facing north
    # camera: window 2.5 m north, 15 cm EAST (right) of the truth, 3 deg off
    cam = {'centre': np.array([2.5, 0.15, -1.9]),
           'normal': np.array([-math.cos(0.05), -math.sin(0.05), 0.0]),
           'width': 0.60, 'height': 0.60, 'samples': 10, 'age': 0.0}
    import drone_testing.mission_fsm as mf
    orig = mf.MissionFSM.window_estimate
    mf.MissionFSM.window_estimate = lambda self: dict(cam)
    try:
        fsm.exit_scan = (time.monotonic(), _body_scan(2.5, -0.30, 0.30))
        est = fsm.window_estimate()
    finally:
        mf.MissionFSM.window_estimate = orig
    assert est['lidar']
    assert est['centre'][0] == pytest.approx(2.5, abs=0.03)
    assert est['centre'][1] == pytest.approx(0.0, abs=0.03)   # on the gap
    assert est['centre'][2] == pytest.approx(-1.9)            # camera height
    assert est['normal'][0] == pytest.approx(-1.0, abs=1e-3)  # square wall


def test_inbound_traverse_line_follows_the_lidar_gap(fsm):
    import numpy as np
    fsm.phase = fsm.PHASE_IN
    fsm.inbound_window = (0.60, 0.60)
    fsm._enter_stage(fsm.TRAVERSE)
    lp = fsm.local_position
    lp.x, lp.y, lp.heading = 0.0, 0.0, 0.0
    fsm.traverse_heading = 0.0
    fsm.traverse_entry = np.array([0.0, 0.0, 0.0])
    fsm.traverse_exit = np.array([4.0, 0.0, 0.0])
    fsm.traverse_standoff = 2.0
    fsm.exit_scan = (time.monotonic(), _body_scan(2.0, -0.40, 0.20))  # gap 10 cm right
    for _ in range(30):
        fsm._steer_last = 0.0
        fsm.exit_scan = (time.monotonic(), fsm.exit_scan[1])
        fsm._handle_traverse()
        # the aircraft stays on the old line, so the gap stays 10 cm right of it
        fsm.traverse_entry[1] = 0.0 + 0.0
    assert fsm.traverse_shift == pytest.approx(fsm.TRAVERSE_STEER_MAX) or \
        fsm.traverse_shift > 0.05
    assert fsm.traverse_exit[1] > 0.05                        # moved right (east)


def test_exit_stages_send_the_offboard_heartbeat(fsm):
    """No heartbeat = PX4 takes the aircraft back mid-exit (seen in SITL)."""
    sent = []
    fsm.publish_offboard_control_mode = lambda: sent.append('ocm')
    fsm.publish_position_setpoint = lambda: sent.append('sp')
    fsm.stream_setpoints = True
    fsm._check_flight_clock = lambda: False
    fsm._handle_exit_find = lambda: None
    fsm._begin_exit_find()
    fsm.timer_callback()
    assert sent == ['ocm', 'sp']


# ------------------------------------------ losing the room

def test_lidar_dropout_holds_still_then_flies_out(fsm):
    """Most dropouts are a second or two: hold, do not abandon."""
    import time
    landed, gave_up = [], []
    fsm._begin_landing = landed.append
    fsm._tile_scan_give_up = gave_up.append
    fsm.transform_is_healthy = lambda: True
    fsm.lidar_fix_time = None                       # stale
    assert fsm._tile_health_ok() is False
    assert fsm._room_lost_since is not None and not fsm.moving
    assert not landed and not gave_up               # still waiting
    # it comes back within the grace: carry on
    fsm.lidar_fix_time = fsm._now()
    assert fsm._tile_health_ok() is True
    assert fsm._room_lost_since is None and not landed and not gave_up
    # and if it does not come back, the heading is good, so fly OUT
    fsm.lidar_fix_time = None
    fsm._tile_health_ok()
    fsm._room_lost_since -= fsm.ROOM_LIDAR_RECOVER + 1
    fsm._tile_health_ok()
    assert gave_up and not landed


def test_bad_yaw_lands_in_the_room_rather_than_flying_out(fsm):
    """With the heading gone, 'out' points somewhere else."""
    import math as m
    landed, gave_up = [], []
    fsm._begin_landing = landed.append
    fsm._tile_scan_give_up = gave_up.append
    fsm.transform_is_healthy = lambda: True
    fsm._enter_stage(fsm.TILE_MOVE)                 # inside the room
    for _ in range(3):                              # 3 x 8 deg = 24 > 20
        fsm._on_heading_reset(m.radians(8.0))
    assert fsm._yaw_suspect
    fsm.lidar_fix_time = None
    fsm._tile_health_ok()
    fsm._room_lost_since -= fsm.ROOM_LIDAR_RECOVER + 1
    fsm._tile_health_ok()
    assert landed and not gave_up
    assert fsm.LAND_SPEED == pytest.approx(fsm.SLOW_LAND_SPEED)


def test_the_exit_is_refused_on_an_untrusted_yaw(fsm):
    landed = []
    fsm._begin_landing = landed.append
    fsm._yaw_suspect = True
    fsm._begin_exit_find()
    assert landed and fsm.current_stage != fsm.EXIT_FIND


def test_heading_resets_outside_the_room_do_not_count(fsm):
    import math as m
    fsm.phase = fsm.PHASE_OUTSIDE
    fsm._enter_stage(fsm.HOLD)
    for _ in range(5):
        fsm._on_heading_reset(m.radians(10.0))
    assert not fsm._yaw_suspect and fsm._yaw_reset_total == 0.0
