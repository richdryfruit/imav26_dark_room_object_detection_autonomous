"""The marker-terminated legs mission_fsm adds to window_room_traverse.

WHAT THIS IS FOR
----------------
mission_fsm splices two forks into a state machine it inherits -- the
outbound leg between the hold and the window sweep, and the three legs home
in place of the landing at the outbound CLEAR -- and adds one primitive used
four times: a leg that ends on ITS OWN marker rather than on a distance.

Both forks replace a parent behaviour that was "land", and the leg primitive
has a failure mode that no amount of reading catches: acting on a fix from
the wrong marker. A silent regression in either is not a wrong number in a
log, it is an aircraft descending mid-run, or flying the final leg from the
wrong place because the strafe ended on the marker it started over.

So these are the tests worth having: that each fork goes the new way when it
should and the PARENT's way when it should, that a leg ends on its marker and
ignores every other id, that the distance limit is a limit and not a target,
and that nothing which was under the flight clock fell out of it.

WHAT IT DOES NOT TEST
---------------------
Anything requiring an aircraft: the climb, the window estimator, the
traversal geometry, the flow latch. `_still_flyable`, the setpoint publishers
and the position feed are stubbed, so this says the wiring is right, not that
the flight is. It is a bench check, and stands in the same relation to a
flight as `precision_land mode:=bench` does.

    python3 -m pytest test/test_mission_fsm.py -q

Run it from the WORKSPACE root, not the package root: at the package root the
local drone_testing/ directory shadows the installed package. Needs a sourced
ROS environment; does NOT need PX4, a camera or an agent.
"""

import math
import time

import numpy as np

import pytest
import rclpy
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleAttitude
from std_msgs.msg import Float32MultiArray

from drone_testing.mission_fsm import MissionFSM


WINDOW_ID, TURN_ID, PAD_ID = 2, 3, 1


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class _LocalPosition:
    xy_valid = True
    z_valid = True
    v_xy_valid = True
    dist_bottom = 1.0
    dist_bottom_valid = True
    x = 0.0
    y = 0.0
    z = -1.5
    heading = 0.0


def _stubbed_node():
    """A MissionFSM with every PX4 dependency stubbed out."""
    node = MissionFSM()

    node._still_flyable = lambda: True
    node._try_latch_xy_hold = lambda: None
    node.log_flight_state = lambda: None
    node.publish_status = lambda: None
    node._publish_mission_phase = lambda: None
    node.publish_offboard_control_mode = lambda: None
    node.publish_position_setpoint = lambda: None
    node.stream_setpoints = False
    node.flow_is_healthy = lambda: True
    node.relative_altitude = lambda: 1.5
    node.local_position = _LocalPosition()
    node.hold_xy = True
    node.hold_x, node.hold_y = 0.0, 0.0
    node.home_yaw = 0.0

    node.window_calls = []
    node._begin_scan = lambda: node.window_calls.append('scan')
    node._lock_on_window = lambda reason: node.window_calls.append('lock')
    node.window_is_confirmed = lambda: False
    node.landings = []
    node._begin_landing = lambda reason: (
        node.landings.append(reason), node._enter_stage(node.LANDING))

    yield node
    node.destroy_node()


@pytest.fixture
def fsm_factory():
    """Build a stubbed MissionFSM, for fixtures that vary its parameters."""
    def make():
        yield from _stubbed_node()
    return make


@pytest.fixture
def fsm():
    """A MissionFSM with every PX4 dependency stubbed.

    Fresh per test on purpose: the thing under test is a state machine, and a
    node carried between tests would let one test's stage leak into the next
    and quietly turn a failure into a pass.
    """
    yield from _stubbed_node()


def expire(node):
    """Make the current stage's clock read as long elapsed."""
    node.stage_enter_time = time.monotonic() - 10000.0


def see(node, marker_id, x=0.40, y=0.30, z=1.50):
    """Deliver one /aruco/marker_points message for `marker_id`."""
    msg = PointStamped()
    msg.header.frame_id = f'aruco:{marker_id}'
    msg.point.x, msg.point.y, msg.point.z = x, y, z
    node.attitude_q = [1.0, 0.0, 0.0, 0.0]      # level
    node.marker_points_callback(msg)


def arrive(node):
    """Teleport the stub to wherever the leg is actually aiming.

    Computed from move_target_*, never from a hand-written coordinate: the
    legs home run BACKWARD in the takeoff frame, so a literal +distance would
    put the aircraft on the far side of its own start and the leg would never
    finish.
    """
    node.local_position.x = node.move_target_x
    node.local_position.y = node.move_target_y


def cruising(node, name):
    """Put the node in CRUISE on leg `name`, already under way."""
    node._begin_leg_named(name)
    node._handle_cruise()          # first tick starts the move
    assert node.leg_started
    return node.current_leg()


# ----------------------------------------------------------- the marker feed

def test_only_the_targeted_marker_reaches_the_control_loop(fsm):
    """The whole reason /aruco/marker_points carries the id."""
    fsm._begin_leg_named('outbound')                       # outbound, targets WINDOW_ID
    see(fsm, TURN_ID)
    assert fsm.marker_offset_ned() is None, "a non-target id must be discarded"
    assert fsm.marker_point is None

    see(fsm, WINDOW_ID)
    assert fsm.marker_offset_ned() is not None


def test_a_malformed_frame_id_is_ignored(fsm):
    fsm._begin_leg_named('outbound')
    msg = PointStamped()
    msg.header.frame_id = 'camera_body'     # what /aruco/point publishes
    fsm.marker_points_callback(msg)
    msg.header.frame_id = 'aruco:notanint'
    fsm.marker_points_callback(msg)
    assert fsm.marker_point is None


def test_retargeting_clears_the_previous_marker(fsm):
    """A stale fix would end the next leg where the last one ended."""
    fsm._begin_leg_named('outbound')
    see(fsm, WINDOW_ID)
    assert fsm.marker_offset_ned() is not None

    fsm._begin_leg_named('turn')                       # turn leg, targets TURN_ID
    assert fsm.marker_point is None
    assert fsm.marker_offset_ned() is None


def test_marker_offset_is_tilt_compensated_and_correctly_mapped(fsm):
    fsm._begin_leg_named('outbound')
    see(fsm, WINDOW_ID, x=0.40, y=0.30, z=1.50)
    north, east, _ = fsm.marker_offset_ned()
    # marker_body re-labels (x, y, z) as (forward, right, down) = (y, x, -z),
    # and level attitude makes body == NED.
    assert north == pytest.approx(0.30)
    assert east == pytest.approx(0.40)


def test_no_attitude_means_no_offset(fsm):
    """Never fall back to a heading-only rotation: that reintroduces the tilt
    error the quaternion is there to remove."""
    fsm._begin_leg_named('outbound')
    see(fsm, WINDOW_ID)
    fsm.attitude_q = None
    assert fsm.marker_offset_ned() is None


def test_a_stale_marker_is_not_a_marker(fsm):
    fsm._begin_leg_named('outbound')
    see(fsm, WINDOW_ID)
    fsm.marker_time = time.monotonic() - 10.0
    assert fsm.marker_offset_ned() is None, "silence must read as 'no marker'"


# ------------------------------------------------------------------ the legs

def test_hold_flies_the_outbound_leg_before_the_sweep(fsm):
    fsm._enter_stage(fsm.HOLD)
    expire(fsm)
    fsm._handle_hold()

    assert fsm.current_stage == fsm.CRUISE
    assert fsm.current_leg().name == 'outbound'
    assert fsm.target_marker_id == WINDOW_ID
    assert fsm.window_calls == [], "the sweep must wait for the leg"
    assert fsm.MOVE_SPEED == pytest.approx(fsm.LEG_SPEED)
    assert fsm._phase_label() == 'TO_WINDOW'


def test_the_outbound_leg_is_not_flown_twice(fsm):
    fsm.legs_done.add('outbound')
    fsm._enter_stage(fsm.HOLD)
    expire(fsm)
    fsm._handle_hold()

    assert fsm.current_stage != fsm.CRUISE
    assert fsm.window_calls == ['scan'], "this is the parent's hold now"


def test_a_leg_aims_at_its_distance_limit(fsm):
    leg = cruising(fsm, 'outbound')
    # forward at home_yaw 0 is +x in NED.
    assert fsm.move_target_x == pytest.approx(leg.distance)
    assert fsm.move_target_y == pytest.approx(0.0)
    assert fsm.moving


def test_a_backward_leg_aims_behind_the_start(fsm):
    """The concrete form of the takeoff-frame bug: backward must be -x."""
    leg = cruising(fsm, 'pad')
    assert leg.direction == 'backward'
    assert fsm.move_target_x == pytest.approx(-leg.distance)
    assert fsm.move_target_y == pytest.approx(0.0)


def test_the_left_leg_goes_left(fsm):
    leg = cruising(fsm, 'turn')
    # DIRECTIONS['left'](cos0, sin0) = (sin0, -cos0) = (0, -1): -y in NED.
    assert leg.direction == 'left'
    assert fsm.move_target_x == pytest.approx(0.0)
    assert fsm.move_target_y == pytest.approx(-leg.distance)


def test_seeing_the_marker_ends_the_leg_early(fsm):
    cruising(fsm, 'outbound')
    fsm.local_position.x = 4.0          # only part way along
    see(fsm, WINDOW_ID)
    fsm._handle_cruise()

    assert fsm.current_stage == fsm.MARKER_ALIGN
    assert not fsm.moving, "stop pushing towards the distance limit"


def test_the_wrong_marker_does_not_end_the_leg(fsm):
    cruising(fsm, 'outbound')
    fsm.local_position.x = 4.0
    see(fsm, PAD_ID)
    see(fsm, TURN_ID)
    fsm._handle_cruise()

    assert fsm.current_stage == fsm.CRUISE, "only this leg's marker ends it"


def test_running_out_of_distance_holds_and_carries_on(fsm):
    """The distance is a limit, not a target, and hitting it is not an abort.

    The outbound leg deliberately does NOT fly the elevated marker retry: the
    window is a better landmark than its marker, so a miss hands straight over
    to the window search rather than spending clock climbing.
    """
    fsm.home_z = 0.0        # without it _begin_alt_change short-circuits
    cruising(fsm, 'outbound')
    fsm.local_position.x = fsm.legs[fsm.leg_by_name['outbound']].distance     # arrived at the limit
    fsm._handle_cruise()                            # enters the settle band
    fsm.move_in_band_since = time.monotonic() - 99.0
    fsm._handle_cruise()

    assert fsm.landings == [], "a missing course marker is not fatal"
    assert 'outbound' in fsm.legs_done
    # With a strafe configured the mission carries on via the descent and the
    # strafe rather than straight to the sweep.
    assert fsm.current_stage == fsm.ALT_CHANGE
    assert fsm.alt_next == 'offset'


def test_losing_the_flow_latch_mid_leg_abandons_it(fsm):
    cruising(fsm, 'outbound')
    fsm.hold_xy = False
    fsm._handle_cruise()

    assert not fsm.moving, "do not coast on an estimate we no longer believe"
    assert 'outbound' in fsm.legs_done


def test_a_stuck_leg_times_out(fsm):
    cruising(fsm, 'outbound')
    expire(fsm)
    fsm._handle_cruise()
    assert 'outbound' in fsm.legs_done


# ------------------------------------------------------------- the CLEAR fork

def test_outbound_clear_flies_the_legs_home_instead_of_landing(fsm):
    fsm.phase = fsm.PHASE_OUT
    fsm.dolls_armed = True
    fsm._enter_stage(fsm.CLEAR)
    expire(fsm)
    fsm._handle_clear()

    assert fsm.current_stage == fsm.CRUISE
    assert fsm.current_leg().name == 'offset_back', "the mirror strafe first"
    assert fsm.landings == [], "the parent would have landed here"
    assert not fsm.dolls_armed, "the model has nothing to look at outside"
    assert fsm._phase_label() == 'OFF_AXIS', 'the mirror strafe, not the leg'


def test_inbound_clear_still_starts_the_room_pattern(fsm):
    fsm.phase = fsm.PHASE_IN
    fsm._enter_stage(fsm.CLEAR)
    expire(fsm)
    fsm._handle_clear()

    assert fsm.current_stage in (fsm.ROOM_MOVE, fsm.ROOM_TURN)


# ---------------------------------------------------------------- centring

def test_align_drives_the_carrot_by_the_gain(fsm):
    fsm._begin_leg_named('outbound')
    fsm._enter_stage(fsm.MARKER_ALIGN)
    see(fsm, WINDOW_ID, x=0.40, y=0.30)
    fsm.moving = False
    fsm._handle_marker_align()

    assert fsm.moving
    assert fsm.move_target_x == pytest.approx(fsm.MARKER_GAIN * 0.30)
    assert fsm.move_target_y == pytest.approx(fsm.MARKER_GAIN * 0.40)


def test_one_sample_inside_tolerance_is_not_an_arrival(fsm):
    fsm._begin_leg_named('outbound')
    fsm._enter_stage(fsm.MARKER_ALIGN)
    see(fsm, WINDOW_ID, x=0.02, y=0.01)
    fsm._handle_marker_align()
    assert fsm.current_stage == fsm.MARKER_ALIGN, "corner noise is not arrival"


def test_settled_inside_tolerance_parks_on_the_marker(fsm):
    fsm._begin_leg_named('outbound')
    fsm._enter_stage(fsm.MARKER_ALIGN)
    see(fsm, WINDOW_ID, x=0.02, y=0.01)
    fsm._handle_marker_align()
    fsm.align_in_band_since = time.monotonic() - 99.0
    fsm._handle_marker_align()

    assert fsm.current_stage == fsm.MARKER_HOLD
    assert not fsm.moving, "the carrot is released on arrival"


def test_a_lost_marker_freezes_the_carrot_then_resumes_cruising(fsm):
    fsm._begin_leg_named('outbound')
    fsm._enter_stage(fsm.MARKER_ALIGN)
    fsm.moving = True
    fsm._handle_marker_align()
    assert not fsm.moving, "do not walk towards a measurement we lost"
    assert fsm.current_stage == fsm.MARKER_ALIGN

    fsm.marker_lost_since = time.monotonic() - fsm.MARKER_LOST_SECONDS - 1.0
    fsm._handle_marker_align()
    assert fsm.current_stage == fsm.CRUISE, "go back to flying the leg"


def test_drift_during_the_hold_earns_another_correction(fsm):
    fsm._begin_leg_named('outbound')
    fsm._enter_stage(fsm.MARKER_HOLD)
    see(fsm, WINDOW_ID, x=0.90, y=0.90)
    fsm._handle_marker_hold()
    assert fsm.current_stage == fsm.MARKER_ALIGN


def test_noise_inside_the_released_band_does_not_flap(fsm):
    fsm._begin_leg_named('outbound')
    fsm._enter_stage(fsm.MARKER_HOLD)
    edge = fsm.MARKER_TOLERANCE * 1.2       # inside release, outside tolerance
    see(fsm, WINDOW_ID, x=edge, y=0.0)
    fsm._handle_marker_hold()
    assert fsm.current_stage == fsm.MARKER_HOLD


# --------------------------------------------------------- the leg sequence

@pytest.mark.parametrize('name,nxt', [('return', 'turn'), ('turn', 'pad')])
def test_each_leg_home_starts_the_next(fsm, name, nxt):
    fsm._begin_leg_named(name)
    fsm._enter_stage(fsm.MARKER_HOLD)
    see(fsm, fsm.legs[fsm.leg_by_name[name]].marker_id, x=0.01, y=0.01)
    fsm.marker_aligned_error = 0.02
    expire(fsm)
    fsm._handle_marker_hold()

    assert fsm.current_leg().name == nxt
    assert fsm.current_stage == fsm.CRUISE
    assert fsm.landings == []


def test_the_pad_leg_ends_in_a_precision_descent(fsm):
    fsm._begin_leg_named('pad')
    fsm._enter_stage(fsm.MARKER_HOLD)
    see(fsm, PAD_ID, x=0.01, y=0.01)
    fsm.marker_aligned_error = 0.02
    expire(fsm)
    fsm._handle_marker_hold()

    assert fsm.current_stage == fsm.LANDING
    assert fsm.precision_descent_active, "the aligned point is kept for the drop"
    assert fsm.hold_xy
    assert 'pad' in fsm.landings[-1]


def test_the_pad_leg_lands_off_pad_when_its_marker_never_appears(fsm):
    """The one required marker. No pad means land here, not hover forever.

    Only AFTER the elevated retry, which is tested separately -- the pad leg
    is the one the retry matters most for.
    """
    fsm.home_z = 0.0
    cruising(fsm, 'pad')
    arrive(fsm)
    fsm._handle_cruise()
    fsm.move_in_band_since = time.monotonic() - 99.0
    fsm._handle_cruise()

    assert fsm.landings, "a missing pad marker must land the aircraft"
    assert not fsm.precision_descent_active, "there is no aligned point to keep"


def test_only_the_pad_leg_is_required(fsm):
    for name in ('outbound', 'return', 'turn'):
        assert not fsm.legs[fsm.leg_by_name[name]].required
    assert fsm.legs[fsm.leg_by_name['pad']].required


# ------------------------------------------------------- clock and naming

def test_every_new_airborne_stage_is_under_the_flight_clock(fsm):
    stages = fsm._clock_stages()
    for stage in (fsm.CRUISE, fsm.MARKER_ALIGN, fsm.MARKER_HOLD):
        assert stage in stages, f"{stage} can postpone the descent"


def test_the_inherited_clock_membership_is_unchanged(fsm):
    stages = fsm._clock_stages()
    for stage in (fsm.SCAN, fsm.LOCK, fsm.ROOM_MOVE, fsm.ROOM_TURN,
                  fsm.ROOM_HOLD, fsm.RELOCK):
        assert stage in stages
    # Deliberately exempt, and adding stages must not have changed it: you do
    # not abandon a run halfway through an aperture because a timer expired.
    assert fsm.TRAVERSE not in stages


def test_no_two_stages_share_a_name(fsm):
    """The reason this class does not inherit PrecisionLand -- see the header.

    A stage name is a class attribute, so two branches that both call a stage
    ALIGN cannot coexist on one object: the dispatch of whichever loses the
    MRO silently routes to the other one's handler.
    """
    stages = [
        fsm.PREPARATION, fsm.OFFBOARD_REQUEST, fsm.ARMING, fsm.GROUND_WAIT,
        fsm.TAKEOFF, fsm.HOLD, fsm.STEP, fsm.STEP_HOLD, fsm.POST_HOLD,
        fsm.SCAN, fsm.LOCK, fsm.RECENTRE, fsm.AIM, fsm.ALIGN, fsm.TRAVERSE,
        fsm.CLEAR, fsm.ROOM_MOVE, fsm.ROOM_TURN, fsm.ROOM_HOLD, fsm.RELOCK,
        fsm.CRUISE, fsm.MARKER_ALIGN, fsm.MARKER_HOLD,
        fsm.LANDING, fsm.DISARMING, fsm.KILLING, fsm.DONE,
    ]
    assert len(stages) == len(set(stages))
    assert fsm.ALIGN != fsm.MARKER_ALIGN


def test_the_legs_home_run_BACKWARD_in_the_takeoff_frame(fsm):
    """The bug this exists to catch.

    DIRECTION_FRAME is 'home', so _reference_yaw() returns the yaw held at
    ARMING. Coming out of the window the airframe is facing back down the
    course, but that is a heading, not a frame: 'forward' out there still
    points at the window, and a forward return leg flies straight back into
    the room.
    """
    assert fsm.DIRECTION_FRAME == 'home', "these directions assume the home frame"
    d = {leg.name: leg.direction for leg in fsm.legs}
    assert d['outbound'] == 'forward'
    assert d['return'] == 'backward'
    assert d['turn'] == 'left'
    assert d['pad'] == 'backward'


def test_the_successor_chain_is_the_mission_order(fsm):
    order, name = [], 'outbound'
    for _ in range(10):
        order.append(name)
        nxt = fsm.legs[fsm.leg_by_name[name]].nxt
        if nxt in ('window', 'descend'):
            order.append(nxt)
            break
        name = nxt
    # With the measured strafe configured, the outbound leg hands over to it
    # and the strafe hands over to the sweep. The chain home is picked up by
    # _handle_clear rather than by a successor.
    assert order == ['outbound', 'offset', 'window']
    assert fsm.legs[fsm.leg_by_name['return']].nxt == 'turn'
    assert fsm.legs[fsm.leg_by_name['turn']].nxt == 'pad'
    assert fsm.legs[fsm.leg_by_name['pad']].nxt == 'descend'


def test_no_strafe_legs_when_window_offset_is_zero(plain_fsm):
    fsm = plain_fsm
    assert fsm.WINDOW_OFFSET == 0.0
    assert 'offset' not in fsm.leg_by_name
    assert 'offset_back' not in fsm.leg_by_name
    assert fsm.legs[fsm.leg_by_name['outbound']].nxt == 'window'


def test_outbound_clear_goes_straight_to_the_return_leg_without_a_strafe(plain_fsm):
    fsm = plain_fsm
    fsm.phase = fsm.PHASE_OUT
    fsm._enter_stage(fsm.CLEAR)
    expire(fsm)
    fsm._handle_clear()
    assert fsm.current_leg().name == 'return'


# --------------------------------------------------- the window-axis strafes
#
# The default fixture leaves window_offset at 0, which is the "approach
# obliquely from the marker" configuration. These build a node with the
# strafes enabled instead. The class attribute is the declared parameter's
# default, so setting it before construction is how a launch-time override is
# reproduced in-process -- feeding a real `-p` override needs the value on the
# command line at rclpy.init(), which a test cannot arrange per case.

@pytest.fixture
def plain_fsm(monkeypatch, fsm_factory):
    """window_offset = 0: no strafes, outbound hands straight to the window.

    The default is now the MEASURED 0.25 m, so this is the explicit
    'approach obliquely from the marker' configuration rather than the
    default one.
    """
    monkeypatch.setattr(MissionFSM, 'WINDOW_OFFSET', 0.0, raising=False)
    yield from fsm_factory()


@pytest.fixture
def offset_fsm(monkeypatch, fsm_factory):
    monkeypatch.setattr(MissionFSM, 'WINDOW_OFFSET', 1.40, raising=False)
    monkeypatch.setattr(MissionFSM, 'WINDOW_OFFSET_DIRECTION', 'left',
                        raising=False)
    yield from fsm_factory()


def test_the_strafes_exist_and_mirror_each_other(offset_fsm):
    node = offset_fsm
    out = node.legs[node.leg_by_name['offset']]
    back = node.legs[node.leg_by_name['offset_back']]

    assert out.direction == 'left' and back.direction == 'right', \
        "the way out and the way back must be opposites in the takeoff frame"
    assert out.distance == pytest.approx(back.distance) == pytest.approx(1.40)
    assert out.marker_id is None and back.marker_id is None, \
        "there is no marker on the window axis to aim at"
    assert not out.required and not back.required


def test_the_outbound_leg_hands_over_to_the_strafe_not_the_window(offset_fsm):
    node = offset_fsm
    assert node.legs[node.leg_by_name['outbound']].nxt == 'offset'
    assert node.legs[node.leg_by_name['offset']].nxt == 'window'
    assert node.legs[node.leg_by_name['offset_back']].nxt == 'return'


def test_a_pure_distance_leg_ignores_every_marker(offset_fsm):
    node = offset_fsm
    cruising(node, 'offset')
    for mid in (WINDOW_ID, TURN_ID, PAD_ID):
        see(node, mid)
    node._handle_cruise()

    assert node.current_stage == node.CRUISE, \
        "a strafe has no marker and must run to its distance"
    assert node.target_marker_id is None


def test_the_strafe_ends_on_its_distance_and_starts_the_window(offset_fsm):
    node = offset_fsm
    cruising(node, 'offset')
    arrive(node)
    node._handle_cruise()
    node.move_in_band_since = time.monotonic() - 99.0
    node._handle_cruise()

    assert node.window_calls == ['scan']
    assert node.landings == []


def test_the_strafe_goes_left_in_the_takeoff_frame(offset_fsm):
    node = offset_fsm
    cruising(node, 'offset')
    # DIRECTIONS['left'](cos0, sin0) = (0, -1): -y in NED.
    assert node.move_target_x == pytest.approx(0.0)
    assert node.move_target_y == pytest.approx(-1.40)


def test_outbound_clear_runs_the_mirror_strafe_before_the_return(offset_fsm):
    node = offset_fsm
    node.phase = node.PHASE_OUT
    node._enter_stage(node.CLEAR)
    expire(node)
    node._handle_clear()

    assert node.current_leg().name == 'offset_back', \
        "without the mirror the return leg passes the marker outside the " \
        "down camera footprint and never sees it"
    assert node.landings == []


# ------------------------------------------------------- the attitude clash

def test_attitude_feeds_BOTH_halves_of_the_chain(fsm):
    """One topic, one callback name, two attributes -- and both are load bearing.

    Found by tools/fsm_sim.py, not by reading: with only PrecisionLand's
    version running, self.attitude stays None, window_traverse's
    geometry_callback refuses EVERY corner sample, the window pose is never
    built, and the flight locks onto a window it can then never approach. It
    does not error. It sits in LOCK until the flight clock lands it.
    """
    msg = VehicleAttitude()
    msg.q = [1.0, 0.0, 0.0, 0.0]
    fsm.attitude_callback(msg)

    assert fsm.attitude is not None, \
        "window_traverse needs self.attitude or the estimator gets nothing"
    assert fsm.attitude_q is not None, \
        "the marker maths needs the normalised self.attitude_q"


def test_the_window_estimator_actually_receives_samples(fsm):
    """The end-to-end form of the same thing, through geometry_callback."""
    att = VehicleAttitude()
    att.q = [1.0, 0.0, 0.0, 0.0]
    fsm.attitude_callback(att)
    fsm.local_position.xy_valid = True
    fsm.local_position.z_valid = True

    before = fsm.estimator.accepted_total
    msg = Float32MultiArray()
    # Four corners of a 1 m window 3 m ahead, plus the centre row and the
    # not-truncated row. Angles in degrees, depth along the optical axis.
    half = math.degrees(math.atan(0.5 / 3.0))
    msg.data = [float(v) for v in (
        3.0, -half, +half,   3.0, +half, +half,
        3.0, +half, -half,   3.0, -half, -half,
        3.0, 0.0, 0.0,
        0.0, 999.0, 0.0)]
    fsm.geometry_callback(msg)

    assert fsm.estimator.accepted_total > before, (
        "geometry_callback bailed before reaching the estimator "
        f"(rejections: {fsm.estimator.rejection_summary()})")


# ------------------------------------------------------ the altitude schedule

def test_the_climb_goes_to_the_cruise_altitude(fsm):
    """takeoff_altitude is deliberately ignored: one height for the markers."""
    assert fsm.TAKEOFF_ALTITUDE == pytest.approx(fsm.CRUISE_ALTITUDE)
    assert fsm.CRUISE_ALTITUDE == pytest.approx(2.50)
    assert fsm.WINDOW_ALTITUDE == pytest.approx(1.75)


def test_the_outbound_leg_drops_to_the_window_altitude(fsm):
    fsm.home_z = 0.0
    fsm._begin_leg_named('outbound')
    fsm._enter_stage(fsm.MARKER_HOLD)
    see(fsm, WINDOW_ID, x=0.01, y=0.01)
    fsm.marker_aligned_error = 0.02
    expire(fsm)
    fsm._handle_marker_hold()

    assert fsm.current_stage == fsm.ALT_CHANGE
    assert fsm.alt_target == pytest.approx(fsm.WINDOW_ALTITUDE)
    assert fsm.alt_next == 'offset', "the strafe follows the descent"
    assert fsm.legs[fsm.leg_by_name['offset']].nxt == 'window'


def test_the_return_leg_climbs_back_to_cruise(fsm):
    fsm.home_z = 0.0
    fsm._begin_leg_named('return')
    fsm._enter_stage(fsm.MARKER_HOLD)
    see(fsm, WINDOW_ID, x=0.01, y=0.01)
    fsm.marker_aligned_error = 0.02
    expire(fsm)
    fsm._handle_marker_hold()

    assert fsm.current_stage == fsm.ALT_CHANGE
    assert fsm.alt_target == pytest.approx(fsm.CRUISE_ALTITUDE)
    assert fsm.alt_next == 'turn'


def test_reaching_the_altitude_runs_what_follows(fsm):
    fsm.home_z = 0.0
    fsm.relative_altitude = lambda: fsm.WINDOW_ALTITUDE
    fsm._begin_alt_change(fsm.WINDOW_ALTITUDE, 'window', 'test')
    fsm._handle_alt_change()

    assert fsm.window_calls == ['scan']


def test_a_stuck_altitude_change_carries_on_anyway(fsm):
    """The height is a preference; the mission outranks it."""
    fsm.home_z = 0.0
    fsm.relative_altitude = lambda: 0.2         # never arrives
    fsm._begin_alt_change(fsm.WINDOW_ALTITUDE, 'window', 'test')
    expire(fsm)
    fsm._handle_alt_change()

    assert fsm.window_calls == ['scan']
    assert fsm.landings == [], "a missed height must not end the flight"


# ------------------------------------------------- the window search failsafe

def test_the_first_rung_is_a_yaw_sweep(fsm):
    fsm._enter_stage(fsm.SCAN)
    expire(fsm)
    fsm._handle_scan()

    assert fsm.window_rung == 1
    # SCAN_SPAN is the TOTAL arc, so +/-30 deg is a 60 deg span.
    assert fsm.SCAN_SPAN == pytest.approx(math.radians(2 * fsm.WINDOW_SWEEP_DEG))
    assert fsm.window_backoffs == 0, "rung 1 must not move the aircraft"


def test_later_rungs_back_off(fsm):
    fsm.window_rung = 1                         # sweep already tried
    fsm._enter_stage(fsm.SCAN)
    expire(fsm)
    fsm._handle_scan()

    assert fsm.current_stage == fsm.WINDOW_BACKOFF
    assert fsm.window_backoffs == 1
    assert fsm.backoff_total == pytest.approx(fsm.WINDOW_BACKOFF_M)
    # Backward in the takeoff frame is -x with home_yaw 0.
    assert fsm.move_target_x == pytest.approx(-fsm.WINDOW_BACKOFF_M)


def test_a_window_seen_while_backing_off_ends_the_retreat(fsm):
    # Nothing visible during the sweep -- that is why it backs off at all.
    fsm.window_rung = 1
    fsm._enter_stage(fsm.SCAN)
    expire(fsm)
    fsm._handle_scan()
    assert fsm.current_stage == fsm.WINDOW_BACKOFF
    assert fsm.window_calls == []

    # ...and the retreat is what brings it into frame.
    fsm.window_is_confirmed = lambda: True
    fsm._handle_window_backoff()
    assert fsm.window_calls == ['lock'], "the point of backing off was to see it"
    assert not fsm.moving


def test_the_ladder_gives_up_after_the_backoff_budget(fsm):
    fsm.window_rung = 1
    fsm.window_backoffs = fsm.WINDOW_MAX_BACKOFFS
    fsm._enter_stage(fsm.SCAN)
    expire(fsm)
    fsm._handle_scan()

    assert fsm.landings, "out of rungs, the attempt is abandoned"


def test_the_ladder_does_not_fire_before_its_timer(fsm):
    fsm._enter_stage(fsm.SCAN)
    fsm._handle_scan()          # stage clock just started

    assert fsm.window_rung == 0
    assert fsm.SCAN_SPAN == pytest.approx(0.0), "no sweep until it escalates"


def test_the_legs_are_ten_metres_not_eleven(fsm):
    d = {leg.name: leg.distance for leg in fsm.legs}
    assert d['outbound'] == pytest.approx(10.0)
    assert d['return'] == pytest.approx(10.0)
    assert d['pad'] == pytest.approx(10.0)


# --------------------------------------------- the missed-marker retry

def test_a_leg_without_its_marker_climbs_for_a_wider_look(fsm):
    """The basket is +/- h*tan(39 deg), so height is the cheapest fix.

    OFF by default -- see test_a_missed_marker_just_moves_on. This is the
    behaviour when marker_max_retries is turned back up.
    """
    fsm.MARKER_MAX_RETRIES = 1
    fsm.home_z = 0.0
    cruising(fsm, 'turn')
    arrive(fsm)
    fsm._handle_cruise()
    fsm.move_in_band_since = time.monotonic() - 99.0
    fsm._handle_cruise()

    assert fsm.current_stage == fsm.ALT_CHANGE
    assert fsm.alt_target == pytest.approx(3.50)
    assert fsm.alt_next == 'retry'
    assert fsm.leg_retries == 1
    assert fsm.current_leg().name == 'turn', "the leg is not finished yet"
    assert fsm.landings == []


def test_the_pad_leg_retries_before_it_gives_up(fsm):
    """With the retry enabled, the pad leg tries from height before landing."""
    fsm.MARKER_MAX_RETRIES = 1
    fsm.home_z = 0.0
    cruising(fsm, 'pad')
    arrive(fsm)
    fsm._handle_cruise()
    fsm.move_in_band_since = time.monotonic() - 99.0
    fsm._handle_cruise()

    assert fsm.landings == [], "the pad leg must try from height first"
    assert fsm.alt_next == 'retry'


def test_the_retry_hovers_then_retraces_backwards(fsm):
    fsm.home_z = 0.0
    fsm._begin_leg_named('pad')          # pad runs BACKWARD in the home frame
    fsm.hold_x, fsm.hold_y = -10.0, 0.0
    fsm._begin_retry_search()

    fsm._handle_marker_retry()
    assert not fsm.retry_moving, "it looks from the spot first"

    expire(fsm)
    fsm._handle_marker_retry()
    assert fsm.retry_moving
    # The leg flew backward (-x), so the retrace is forward (+x) of the stop.
    assert fsm.move_target_x == pytest.approx(-10.0 + fsm.MARKER_RETRY_DISTANCE)


def test_a_marker_found_during_the_retry_descends_and_centres(fsm):
    fsm.home_z = 0.0
    fsm._begin_leg_named('turn')
    fsm._begin_retry_search()
    see(fsm, TURN_ID)
    fsm._handle_marker_retry()

    assert fsm.current_stage == fsm.ALT_CHANGE
    assert fsm.alt_target == pytest.approx(fsm.CRUISE_ALTITUDE)
    assert fsm.alt_next == 'align'

    fsm.relative_altitude = lambda: fsm.CRUISE_ALTITUDE
    fsm._handle_alt_change()
    assert fsm.current_stage == fsm.MARKER_ALIGN


def test_an_exhausted_retry_falls_back_to_the_leg_rule(fsm):
    """Course legs carry on; the pad leg lands. Only after the retry."""
    fsm.home_z = 0.0
    fsm._begin_leg_named('pad')
    fsm.leg_retries = fsm.MARKER_MAX_RETRIES     # already used
    cruising(fsm, 'pad')
    fsm.leg_retries = fsm.MARKER_MAX_RETRIES
    arrive(fsm)
    fsm._handle_cruise()
    fsm.move_in_band_since = time.monotonic() - 99.0
    fsm._handle_cruise()

    assert fsm.landings, "out of retries, the pad leg lands"


def test_the_retry_altitude_is_the_missions_ceiling(fsm):
    assert fsm.MARKER_RETRY_ALTITUDE == pytest.approx(3.50)
    assert fsm.MARKER_RETRY_ALTITUDE > fsm.CRUISE_ALTITUDE > fsm.WINDOW_ALTITUDE


def test_the_outbound_leg_does_not_spend_clock_retrying(fsm):
    """It has a better fallback than height: the window itself."""
    assert not fsm.legs[fsm.leg_by_name['outbound']].retry
    for name in ('return', 'turn', 'pad'):
        assert fsm.legs[fsm.leg_by_name[name]].retry, \
            f"{name} has only its marker to go on"


def test_a_leg_that_lost_the_flow_latch_does_not_retry(fsm):
    """Climbing to search on an estimate we do not trust searches the wrong
    patch of floor, more widely."""
    fsm.home_z = 0.0
    cruising(fsm, 'turn')
    fsm.hold_xy = False
    fsm._handle_cruise()

    assert fsm.current_stage != fsm.ALT_CHANGE
    assert fsm.leg_retries == 0
    assert 'turn' in fsm.legs_done


# ------------------------------------------ stop at the limit and move on

def test_a_missed_marker_just_moves_on(fsm):
    """The roll stops at turn_distance and the mission continues.

    marker_max_retries is 0 by instruction: a leg that reaches its limit
    without its marker STOPS THERE. The climb-and-look failsafe is built and
    tested but off.
    """
    assert fsm.MARKER_MAX_RETRIES == 0
    fsm.home_z = 0.0
    cruising(fsm, 'turn')
    arrive(fsm)
    fsm._handle_cruise()
    fsm.move_in_band_since = time.monotonic() - 99.0
    fsm._handle_cruise()

    assert fsm.current_stage != fsm.ALT_CHANGE, "no climb, no search"
    assert 'turn' in fsm.legs_done
    assert fsm.current_leg().name == 'pad', "straight on to the next step"
    assert fsm.landings == []


def test_the_descent_to_the_window_altitude_happens_either_way(fsm):
    """Coming down is how the approach is set up, not a reward for the marker.

    'at 10 m, if you dont detect the aruco ... come down to 1.75 m and then
    only detect the windows'.
    """
    fsm.home_z = 0.0
    cruising(fsm, 'outbound')
    arrive(fsm)
    fsm._handle_cruise()
    fsm.move_in_band_since = time.monotonic() - 99.0
    fsm._handle_cruise()          # ran out of distance, NO marker seen

    assert fsm.current_stage == fsm.ALT_CHANGE
    assert fsm.alt_target == pytest.approx(fsm.WINDOW_ALTITUDE) == 1.75
    # ...and only then the strafe, and only then the window search.
    assert fsm.alt_next == 'offset'
    assert fsm.legs[fsm.leg_by_name['offset']].nxt == 'window'


def test_the_traverse_does_not_depend_on_where_the_aircraft_stands(fsm):
    """Why the 30 cm backoff needs no compensation, pinned as a test.

    approach_points() does not TAKE the aircraft position -- both ends are
    built from the window pose alone, and _handle_align recomputes them from
    the live estimate every tick. Backing off changes where the approach
    starts, never where it ends. Adding the backoff to the traversal would
    move the entry point that far further from the window than intended.
    """
    est = {'centre': np.array([3.0, 0.0, -1.8]),
           'normal': np.array([-1.0, 0.0, 0.0]),
           'width': 1.0, 'height': 1.0}

    fsm.local_position.x = 1.00
    entry_a, exit_a, head_a = fsm.approach_points(est)
    fsm.local_position.x = 0.70          # backed off 30 cm
    entry_b, exit_b, head_b = fsm.approach_points(est)

    assert np.allclose(entry_a, entry_b)
    assert np.allclose(exit_a, exit_b)
    assert head_a == pytest.approx(head_b)


# ------------------------------------------------- the lateral estimate (VIO)

class _Flags:
    """Minimal EstimatorStatusFlags stand-in."""
    cs_ev_pos = True
    cs_ev_vel = False
    cs_ev_yaw_fault = False
    cs_yaw_align = True
    cs_rng_hgt = True
    cs_rng_kin_consistent = True
    cs_rng_fault = False
    cs_rng_stuck = False
    cs_rng_terrain = False


def vio_ok(node, age=0.0, cov=1.0):
    """Make the VIO stream look healthy `age` seconds ago.

    Also drops the fixture's flow_is_healthy stub, so these tests exercise the
    REAL predicate rather than the always-true stand-in the other tests want.
    """
    node.__dict__.pop('flow_is_healthy', None)
    node.vio_odom_time = time.monotonic() - age
    node.vio_covariance = cov
    node.estimator_flags = _Flags()
    node.local_position.xy_valid = True
    node.local_position.v_xy_valid = True


def test_the_legs_fly_on_vio_not_optical_flow(fsm):
    assert fsm.LATERAL_SOURCE == 'vio'
    assert fsm.VIO_ODOM_TOPIC == '/rtabmap/odom'


def test_a_fresh_vio_fix_is_healthy(fsm):
    vio_ok(fsm)
    assert fsm.flow_is_healthy()


def test_a_stale_vio_stream_is_not_healthy(fsm):
    """rtabmap going quiet must read as 'no fix', not as a coasting estimate."""
    vio_ok(fsm, age=fsm.VIO_MAX_AGE + 1.0)
    assert not fsm.flow_is_healthy()


def test_lost_tracking_covariance_is_not_healthy(fsm):
    """rtabmap_odom signals lost tracking with a huge covariance, not silence."""
    vio_ok(fsm, cov=9999.0)
    assert not fsm.flow_is_healthy()


def test_ekf_not_fusing_vision_is_not_healthy(fsm):
    """xy_valid alone is EKF2 coasting on the IMU, not a corrected estimate."""
    vio_ok(fsm)
    f = _Flags()
    f.cs_ev_pos = False
    f.cs_ev_vel = False
    fsm.estimator_flags = f
    assert not fsm.flow_is_healthy()


def test_lost_yaw_alignment_is_not_healthy(fsm):
    """An unanchored heading makes the latched x/y point meaningless."""
    vio_ok(fsm)
    f = _Flags()
    f.cs_yaw_align = False
    fsm.estimator_flags = f
    assert not fsm.flow_is_healthy()


def test_no_estimator_flags_falls_back_to_the_stream(fsm):
    """Weaker, but refusing to fly a healthy aircraft is worse."""
    vio_ok(fsm)
    fsm.estimator_flags = None
    assert fsm.flow_is_healthy()


def test_the_agl_floor_is_NOT_applied_to_vio(fsm):
    """FLOW_MIN_AGL exists because flow cannot see a floor 15 cm away.

    The RealSense looks out across a room, so that gate is meaningless here --
    and keeping it would block the legs at exactly the low altitudes this
    mission flies.
    """
    vio_ok(fsm)
    fsm.local_position.dist_bottom = 0.05      # far below FLOW_MIN_AGL (0.30)
    assert fsm.flow_is_healthy()


def test_flow_can_be_restored_by_parameter(fsm):
    """lateral_source:=flow must hand straight back to the inherited gate."""
    vio_ok(fsm)
    fsm.LATERAL_SOURCE = 'flow'
    fsm.local_position.dist_bottom = 0.05
    assert not fsm.flow_is_healthy(), "the inherited AGL floor should bite again"
    fsm.local_position.dist_bottom = 1.0
    assert fsm.flow_is_healthy()


# ------------------------------------------- the real window's clearance

def test_the_real_window_leaves_a_workable_margin():
    """The REAL aperture is 0.50 x 0.60 m and it is the WIDTH that binds.

    hard_clearance (0.030 m) is the abandon threshold, and what matters is
    what is left at the worst alignment the gate still PERMITS -- not at
    perfect centring, which the aircraft never achieves:

        swept width at yaw e : 0.260 * (cos e + sin e)
        margin per side      : (width - swept)/2 - cross_tolerance

    At the stock 0.06 m / 8 deg that is 43 mm: thirteen millimetres over the
    hard clearance. The launch file tightens it to 0.04 m / 6 deg.
    """
    width, drone_w, hard = 0.50, 0.260, 0.030

    def margin(cross_tol, yaw_deg):
        e = math.radians(yaw_deg)
        swept = drone_w * (abs(math.cos(e)) + abs(math.sin(e)))
        return 0.5 * (width - swept) - cross_tol

    stock = margin(0.06, 8.0)
    tightened = margin(0.04, 6.0)

    assert stock > hard, "even the stock gate does not refuse outright"
    assert stock - hard < 0.020, "...but it leaves under 20 mm, which is why it was tightened"
    assert tightened - hard > 0.035, "the tightened gate must leave a real margin"
    assert tightened > stock

    # The vertical is not the binding constraint on this window.
    assert 0.5 * (0.60 - 0.260) > tightened, "height is generous; width binds"
