"""
The whole competition run, as ONE continuous flight, from one command.

    arm -> climb -> hold -> OUTBOUND leg -> find the blue window -> lock ->
    line up -> through it -> box pattern in the dark room (dolls counted and
    shown) -> relock on the window from inside -> back out through it ->
    TRANSIT leg to the landing pad -> find the ArUco marker -> centre on it ->
    precision descent -> disarm.

    ros2 launch drone_testing mission_fsm.launch.py agent_only:=false \
        outbound_sequence:="forward 3.0, right 1.5" \
        pad_sequence:="backward 2.0, left 4.0"

WHY THIS IS ONE NODE AND NOT FIVE LAUNCH FILES IN A ROW
-------------------------------------------------------
The obvious way to build this is to run takeoff_test, then translate_test,
then window_room_traverse, then translate_test again, then precision_land,
waiting for each to exit. Every one of those is a COMPLETE flight: it arms,
climbs, does its job, lands and disarms. Chaining them means four landings
and four re-arms in the middle of a competition run.

That is not merely inelegant, it is the documented way to get stuck on the
ground. README section 10.1: `cs_rng_kin_consistent` is sticky. EKF2 only
updates it while `in_air` is true, so once it latches false in flight
NOTHING ON THE GROUND CLEARS IT except a flight controller reboot. A run
that lands mid-mission is a run that can refuse to take off again, with the
clock going, and no fix available from the Jetson.

So this node never touches the ground between segments. One arm, one
Offboard session, one continuous stream of setpoints, all the way from the
pad to the pad.

HOW IT IS ASSEMBLED
-------------------
Nothing here re-implements a flight. The class inherits WindowRoomTraverse,
which is where the whole window-and-room mission already lives:

    OffboardSequence      arm / climb / hold / STEP / land / disarm, the
                          keyboard aborts, the x/y latch, the setpoint ramps
      WindowScan          SCAN, LOCK -- sweeping for the blue window
        WindowTraverse    RECENTRE, AIM, ALIGN, TRAVERSE, CLEAR -- the run
                          through the aperture
          WindowRoomTraverse  ROOM_MOVE/TURN/HOLD, RELOCK -- the box pattern,
                              the doll gating, the second traversal out

and this class adds exactly two things to it:

    the two navigation legs, flown on the base class's OWN step machinery
    (STEP / STEP_HOLD / POST_HOLD, the same code offboard_sequence flies, so
    the legs get the flow-gated position control and the leashed carrot for
    free); and

    PAD_SEARCH / PAD_ALIGN / PAD_HOLD, the descent onto the marker.

WHY THE PAD STAGES ARE WRITTEN OUT HERE RATHER THAN INHERITED
--------------------------------------------------------------
PrecisionLand is the other child of OffboardSequence, so
`class MissionFSM(WindowRoomTraverse, PrecisionLand)` looks like it should
work, and the MRO is in fact legal. It does not work, for two reasons that
are worth writing down so nobody tries it again:

  * `ALIGN` means two different things. WindowTraverse.ALIGN == "ALIGN" is
    lining up in front of a window; PrecisionLand.ALIGN == "ALIGN" is
    centring over a marker. A stage name is a class attribute, and one
    object cannot hold two values for `self.ALIGN` -- whichever class is
    earlier in the MRO wins and the other branch's dispatch silently routes
    to the wrong handler.

  * four parameters are declared by both branches -- `align_tolerance`,
    `align_settle_seconds`, `align_timeout`, `flight_seconds` -- and the
    second `declare_parameter` of a name raises
    ParameterAlreadyDeclaredException at construction.

What IS reused from precision_land is the part that is genuinely subtle and
must not be retyped: the marker maths. `marker_offset_ned` applies the full
attitude quaternion, so the roll and pitch the airframe is carrying in order
to make a correction do not themselves read as more offset to correct. Those
helpers touch nothing but `self.marker_*` and `self.attitude_q`, so they are
called unbound -- `PrecisionLand.marker_offset_ned(self)` -- and there is
one copy of that arithmetic in the package, in the file that already flies.
The stage handlers below are the thin control loop around it, under PAD_
names that collide with nothing.

WHERE THE MISSION IS SPLICED IN
--------------------------------
Three overrides, each one a fork the parent already had a branch for:

    _handle_hold        end of the settle at altitude. The parent goes to
                        SCAN. We fly the outbound leg first, THEN let it.
    _handle_post_hold   end of a leg. The parent lands ("sequence complete").
                        We go to the window, or to the pad.
    _handle_clear       far side of a traversal. Outbound, the parent lands.
                        We fly the transit leg instead.

Every other stage is the parent's, reached through the same call it always
was. If the traversal behaviour changes in window_traverse.py, it changes
here too, which is the point.

THE FLIGHT CLOCK
----------------
`flight_seconds` is the backstop that outranks every stage handler: when it
expires the vehicle descends from wherever it is, whatever it was doing.
WindowRoomTraverse sets it to 300 s for two traversals and a room pattern.
This flight adds two navigation legs and a marker search on top, so the
default here is 420 s. It is a BACKSTOP, not a schedule -- set it from the
battery, and leave margin for the descent it is supposed to guarantee.

Keys:  q -> abort into a controlled descent from wherever we are.
       k -> force-disarm immediately (motors cut, vehicle drops).
"""

import math
import time

import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleStatus
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped

from drone_testing.offboard_sequence import parse_sequence, spin_node
from drone_testing.precision_land import PrecisionLand, body_words
from drone_testing.window_room_traverse import WindowRoomTraverse


class MissionFSM(WindowRoomTraverse):
    """The room mission, with a leg to it, a leg away and a landing on a pad."""

    # ---- the stages this class adds ---------------------------------------
    # PAD_ rather than SEARCH/ALIGN/ALIGNED_HOLD: see the module header. The
    # inherited chain already owns ALIGN, and a stage name is a class
    # attribute, not something a subclass can hold a second value for.
    PAD_SEARCH = "PAD_SEARCH"   # stationary, watching for the marker
    PAD_ALIGN = "PAD_ALIGN"     # closing the loop on the marker offset
    PAD_HOLD = "PAD_HOLD"       # settled over it, about to descend

    PAD_STAGES = (PAD_SEARCH, PAD_ALIGN, PAD_HOLD)

    # ---- the legs ---------------------------------------------------------
    # Body-frame, relative to the yaw held since arming, in exactly the
    # grammar offboard_sequence parses: "forward 3.0, right 1.5, yaw 20".
    # Empty means "skip this leg", which is what makes the node degrade
    # cleanly into plain window_room_traverse for a rehearsal.
    OUTBOUND_SEQUENCE = ''      # arming point -> in front of the window
    PAD_SEQUENCE = ''           # outside the window -> over the landing pad
    LEG_SPEED = 0.30            # m/s the carrot is walked at on a leg. Slow:
                                # the optical flow is the only thing measuring
                                # these moves, and they are the longest
                                # unreferenced translations in the flight.
    LEG_HOLD_SECONDS = 3.0      # settle after a leg, before the window search
                                # or the marker search starts. A search that
                                # begins while the vehicle is still stopping
                                # is a search through smeared frames.

    # ---- the pad ----------------------------------------------------------
    PAD_ALIGN_TOLERANCE = 0.15   # m radius that counts as "over the marker"
    PAD_ALIGN_SETTLE_SECONDS = 1.5   # s inside that radius before we believe
                                     # it. One sample inside 15 cm is corner
                                     # noise, not an arrival.
    PAD_ALIGN_GAIN = 0.6         # fraction of the measured offset commanded
                                 # per cycle. < 1 is what makes the loop
                                 # monotone instead of oscillatory.
    PAD_ALIGN_RELEASE = 1.5      # multiple of the tolerance at which a settled
                                 # vehicle goes back to aligning. Hysteresis,
                                 # so it does not flap on the boundary.
    PAD_HOLD_SECONDS = 5.0       # station keeping over the marker before the
                                 # descent commits
    PAD_SEARCH_SECONDS = 25.0    # s hovering and looking before giving up
    PAD_ALIGN_TIMEOUT = 45.0     # s trying to centre before giving up
    MARKER_MAX_AGE = 0.5         # s. Older than this is not evidence: a dead
                                 # camera goes quiet, and quiet must read as
                                 # "no marker", never as a lock.
    MARKER_LOST_SECONDS = 5.0    # s without a marker while aligning before
                                 # falling back to the search
    PRECISION_DESCENT = True     # keep the aligned x/y hold through the drop
    BLIND_COMMIT_ALTITUDE = 1.0  # m. Below this the marker leaves the frame.
                                 # MEASURE IT on the bench; see the
                                 # precision_land header.
    PAD_ON_FAIL = 'land'         # land | hold -- what a pad timeout does.
                                 # 'land' means "land here, off the pad",
                                 # which beats hovering until the battery
                                 # decides where to land for you.

    # Two traversals, a room pattern, two navigation legs and a marker search.
    # The parent's 300 s does not cover it. A BACKSTOP, not a schedule.
    FLIGHT_SECONDS = 420.0

    def __init__(self):
        super().__init__()

        # ---- the legs ----
        self.outbound_steps = self._declare_leg(
            'outbound_sequence', self.OUTBOUND_SEQUENCE)
        self.pad_steps = self._declare_leg('pad_sequence', self.PAD_SEQUENCE)
        self.LEG_SPEED = float(self._declare_number('leg_speed', self.LEG_SPEED))
        self.LEG_HOLD_SECONDS = float(self._declare_number(
            'leg_hold_seconds', self.LEG_HOLD_SECONDS))

        # ---- the pad ----
        self.PAD_ALIGN_TOLERANCE = float(self._declare_number(
            'pad_align_tolerance', self.PAD_ALIGN_TOLERANCE))
        self.PAD_ALIGN_SETTLE_SECONDS = float(self._declare_number(
            'pad_align_settle_seconds', self.PAD_ALIGN_SETTLE_SECONDS))
        self.PAD_ALIGN_GAIN = float(self._declare_number(
            'pad_align_gain', self.PAD_ALIGN_GAIN))
        self.PAD_HOLD_SECONDS = float(self._declare_number(
            'pad_hold_seconds', self.PAD_HOLD_SECONDS))
        self.PAD_SEARCH_SECONDS = float(self._declare_number(
            'pad_search_seconds', self.PAD_SEARCH_SECONDS))
        self.PAD_ALIGN_TIMEOUT = float(self._declare_number(
            'pad_align_timeout', self.PAD_ALIGN_TIMEOUT))
        self.MARKER_MAX_AGE = float(self._declare_number(
            'marker_max_age', self.MARKER_MAX_AGE))
        self.MARKER_LOST_SECONDS = float(self._declare_number(
            'marker_lost_seconds', self.MARKER_LOST_SECONDS))
        self.PRECISION_DESCENT = bool(self.declare_parameter(
            'precision_descent', self.PRECISION_DESCENT).value)
        self.BLIND_COMMIT_ALTITUDE = float(self._declare_number(
            'blind_commit_altitude', self.BLIND_COMMIT_ALTITUDE))
        self.PAD_ON_FAIL = str(self.declare_parameter(
            'pad_on_fail', self.PAD_ON_FAIL).value).strip().lower()
        if self.PAD_ON_FAIL not in ('land', 'hold'):
            self.PAD_ON_FAIL = 'land'

        # A gain outside (0, 1] does not converge: at 0 nothing moves, above 1
        # every correction overshoots by design and the loop can walk away.
        if self.PAD_ALIGN_GAIN <= 0.0 or self.PAD_ALIGN_GAIN > 1.0:
            self.get_logger().error(
                f"pad_align_gain {self.PAD_ALIGN_GAIN} is outside (0, 1]; "
                "clamping to 0.6. Above 1 the alignment overshoots and can "
                "diverge.")
            self.PAD_ALIGN_GAIN = 0.6

        # ---- the marker feed ----
        # Same topics aruco_pose publishes, same names precision_land reads.
        self.marker_detected = False
        self.marker_point = None        # (x, y, z) in the camera's frame
        self.marker_time = None         # monotonic, when it arrived
        self.marker_ever_seen = False
        self.marker_info = ''
        self.marker_lost_since = None
        self._warned_no_attitude = False
        self._warned_blind = False
        self.pad_in_band_since = None
        self.pad_error = None
        self.pad_aligned_error = None

        self.create_subscription(Bool, '/aruco/detected',
                                 self.marker_detected_callback, 10)
        self.create_subscription(PointStamped, '/aruco/point',
                                 self.marker_point_callback, 10)
        self.create_subscription(String, '/aruco/info',
                                 self.marker_info_callback, 10)

        # The full quaternion, not just the heading: that is what makes the
        # marker vector tilt-compensated. Both topic names, for the same
        # reason the base class subscribes to both land-detector names -- it
        # is _v1 on some PX4 builds.
        self.attitude_q = None
        self.attitude_time = None
        attitude_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.attitude_subs = [
            self.create_subscription(VehicleAttitude, topic,
                                     self.attitude_callback,
                                     qos_profile=attitude_qos)
            for topic in ('/fmu/out/vehicle_attitude',
                          '/fmu/out/vehicle_attitude_v1')
        ]

        # ---- where we are in the run ----
        # None, 'outbound' or 'transit'. Which leg the shared STEP machinery
        # is currently flying, and therefore what POST_HOLD means when it
        # finishes. Without this the base class's "sequence complete -> land"
        # is the only reading available.
        self.leg = None
        self.outbound_done = not self.outbound_steps
        self.transit_done = not self.pad_steps
        self.pad_outcome = 'not attempted'

        self.get_logger().warning(self._plan_summary())

    # ------------------------------------------------------------ parameters

    def _declare_leg(self, name, default):
        """Parse a navigation leg, fatally.

        A typo in a flight plan must not be quietly guessed at -- the same
        rule, and the same failure mode, as `sequence` in offboard_sequence
        and `room_sequence` in window_room_traverse. Empty is not a typo, it
        is "skip this leg", and is the only reason this returns [].

        Altitude steps are rejected. The legs are flown at the traversal
        altitude, and a leg that changes height changes the height the window
        approach lines up at, which is what `altitude_offset` is for.
        """
        text = str(self.declare_parameter(name, default).value).strip()
        if not text:
            return []
        try:
            parsed = parse_sequence(text)
        except ValueError as exc:
            raise SystemExit(f"{name}: {exc}")
        if not parsed:
            raise SystemExit(f"{name}: no steps in '{text}'")
        for step in parsed:
            if step.kind == 'alt':
                raise SystemExit(
                    f"{name}: '{step.name}' changes altitude, which is not "
                    "flown on a navigation leg (use altitude_offset to set "
                    "the height the mission is flown at)")
        if len(parsed) > self.MAX_STEPS:
            raise SystemExit(
                f"{name}: {len(parsed)} steps, the cap is {self.MAX_STEPS}")
        return [(step.kind, step.name, step.arg) for step in parsed]

    def _plan_summary(self):
        def leg(steps):
            return ' -> '.join(f"{n} {a:.2f}" for _, n, a in steps) or '(skipped)'
        return (
            "MISSION FSM, one continuous flight:\n"
            f"  1. climb {self.TAKEOFF_ALTITUDE:.2f} m, hold "
            f"{self.HOLD_SECONDS:.0f} s\n"
            f"  2. outbound leg: {leg(self.outbound_steps)}\n"
            "  3. find the window, line up, traverse in\n"
            f"  4. room pattern, {len(self.room_steps)} legs, dolls counted\n"
            "  5. relock on the window from inside, traverse out\n"
            f"  6. transit leg: {leg(self.pad_steps)}\n"
            f"  7. find the marker ({self.PAD_SEARCH_SECONDS:.0f} s), centre "
            f"to {self.PAD_ALIGN_TOLERANCE * 100:.0f} cm, hold "
            f"{self.PAD_HOLD_SECONDS:.0f} s\n"
            f"  8. precision descent and disarm\n"
            f"  Hard descent at {self.FLIGHT_SECONDS:.0f} s airborne, whatever "
            "stage we are in.\n"
            "  q aborts into a descent, k force-disarms.")

    # ------------------------------------------------- the marker feed (subs)
    #
    # Thin forwarders onto precision_land's implementations. Called unbound so
    # there is one copy of the marker arithmetic in the package -- in the file
    # that already flies -- rather than a second one here that can drift.

    def marker_detected_callback(self, msg):
        PrecisionLand.marker_detected_callback(self, msg)

    def marker_point_callback(self, msg):
        PrecisionLand.marker_point_callback(self, msg)

    def marker_info_callback(self, msg):
        PrecisionLand.marker_info_callback(self, msg)

    def attitude_callback(self, msg):
        PrecisionLand.attitude_callback(self, msg)

    def marker_is_fresh(self):
        return PrecisionLand.marker_is_fresh(self)

    def marker_body(self):
        return PrecisionLand.marker_body(self)

    def marker_offset_ned(self):
        """(north, east, down) from the vehicle to the marker, tilt-compensated."""
        return PrecisionLand.marker_offset_ned(self)

    def marker_summary(self):
        return PrecisionLand.marker_summary(self)

    # ------------------------------------------------------------ the clock

    def _clock_stages(self):
        """Every airborne stage this class adds goes under the flight clock.

        A stage the clock does not know about is a stage that can hold the
        aircraft up past the descent the clock exists to guarantee. The legs
        included: a leg flown on a bad flow estimate is exactly the case where
        the vehicle is somewhere unplanned and the battery is the only thing
        still counting.
        """
        return super()._clock_stages() + self.PAD_STAGES + (
            self.STEP, self.STEP_HOLD, self.POST_HOLD)

    # ------------------------------------------------------------ the machine

    def timer_callback(self):
        if self.current_stage not in self.PAD_STAGES:
            # Everything else is the room node's, the traversal's or the base
            # class's, reached through the same call it always was.
            super().timer_callback()
            return

        self._publish_mission_phase()

        # The clock outranks the stage handlers below, so a stage that is
        # stuck hunting for a marker cannot postpone the landing.
        if self._check_flight_clock():
            return

        # The same preamble every other timer_callback in this chain runs.
        # Getting it wrong is how the Offboard heartbeat stops and PX4 takes
        # the aircraft back mid-flight.
        if self.stream_setpoints:
            self.publish_offboard_control_mode()
            self.publish_position_setpoint()
        self.publish_status()

        if self.kill_requested:
            self._enter_stage(self.KILLING)
            return
        if self.abort_requested:
            self.abort_requested = False
            self._begin_landing("operator abort")
            return

        {
            self.PAD_SEARCH: self._handle_pad_search,
            self.PAD_ALIGN: self._handle_pad_align,
            self.PAD_HOLD: self._handle_pad_hold,
        }[self.current_stage]()

    # ------------------------------------------------------------- the legs

    def _begin_leg(self, which, steps):
        """Hand the leg to the base class's own step machinery.

        `self.steps` / `step_index` / STEP / STEP_HOLD / POST_HOLD is the
        offboard_sequence flight plan interface, so a leg gets the flow-gated
        x/y latch, the ramped carrot leashed to the measured position, and the
        EKF2 reset handling without any of it being restated here. All this
        method does is load the plan and say which leg it is, so POST_HOLD
        knows what comes next.
        """
        self.leg = which
        self.steps = [_Step(kind, name, arg) for kind, name, arg in steps]
        self.step_index = 0
        self.step_started = False
        self.step_results = []
        self.moving = False
        self.yaw_remaining = 0.0
        self.MOVE_SPEED = self.LEG_SPEED
        self.POST_HOLD_SECONDS = self.LEG_HOLD_SECONDS
        plan = ' -> '.join(f"{n} {a:.2f}" for _, n, a in steps)
        self.get_logger().warning(
            f"{which.upper()} LEG: {len(steps)} steps at "
            f"{self.LEG_SPEED:.2f} m/s: {plan}.")
        self._enter_stage(self.STEP)

    def _handle_hold(self):
        """End of the settle at altitude.

        The parent goes straight to the window sweep. If an outbound leg was
        given, it is flown first: the sweep has a yaw cone around the heading
        it starts from, so it must start from in front of the window, not from
        the arming point.
        """
        if self.outbound_done or not self.outbound_steps:
            super()._handle_hold()
            return

        if not self._still_flyable():
            return

        # The same settle the parent does. The x/y latch has to happen in
        # here: a leg is a position move, and there is no frame to express its
        # target in until the flow is healthy and latched.
        self._try_latch_xy_hold()

        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_leg('outbound', self.outbound_steps)
            return

        self.get_logger().info(
            f"Holding, {remaining:.1f} s to the outbound leg...",
            throttle_duration_sec=1.0)
        self.log_flight_state()

    def _handle_post_hold(self):
        """A leg has finished settling. Which leg decides what happens next.

        The inherited behaviour is to land -- "sequence complete" is the end
        of the mission for offboard_sequence. Here a leg is the START of
        something, so both branches are taken over. The fall-through is kept
        for the case where neither leg is configured and POST_HOLD was reached
        some other way: landing is the right thing to do then.
        """
        if self.leg is None:
            super()._handle_post_hold()
            return

        if not self._still_flyable():
            return

        self._try_latch_xy_hold()

        remaining = self.POST_HOLD_SECONDS - self._in_stage_for()
        if remaining > 0.0:
            self.get_logger().info(
                f"Settling after the {self.leg} leg, {remaining:.1f} s...",
                throttle_duration_sec=1.0)
            self.log_flight_state()
            return

        finished, self.leg = self.leg, None
        self.get_logger().warning(
            f"{finished.upper()} leg complete: "
            + ('; '.join(self.step_results) or 'no steps'))
        # The legs borrowed MOVE_SPEED; the window approach owns it.
        self.MOVE_SPEED = self.APPROACH_SPEED
        self.moving = False

        if finished == 'outbound':
            self.outbound_done = True
            # Exactly the fork window_scan takes at the end of its hold: if
            # the window is already in front of us there is nothing to search
            # for, and sweeping would turn away from a window we can see.
            if self.window_is_confirmed():
                self._lock_on_window("already in sight after the outbound leg")
            else:
                self._begin_scan()
            return

        self.transit_done = True
        self.get_logger().warning(
            f"Over the pad area. Searching for the marker for up to "
            f"{self.PAD_SEARCH_SECONDS:.0f} s. {self.marker_summary()}.")
        self.marker_lost_since = None
        self.pad_in_band_since = None
        self._enter_stage(self.PAD_SEARCH)

    # --------------------------------------------------------------- CLEAR

    def _handle_clear(self):
        """The far side of a traversal. On the way OUT, the pad leg replaces
        the landing.

        Inbound this is not our business at all -- it is the start of the room
        pattern and the parent handles it. Outbound the parent lands where it
        is, which is correct for window_room_traverse and wrong here: the
        vehicle is outside the window with a landing pad somewhere else.
        """
        outbound_clear = (self.phase == self.PHASE_OUT
                          and self.pad_steps and not self.transit_done)
        if not outbound_clear:
            super()._handle_clear()
            return

        if not self._still_flyable():
            return

        # Idempotent, and the model has nothing left to look at out here.
        self._disarm_dolls("cleared the window on the way out")

        self._try_latch_xy_hold()
        self.log_flight_state()

        remaining = self.CLEAR_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_leg('transit', self.pad_steps)
            return

        self.get_logger().info(
            f"OUT: holding, {remaining:.1f} s to the transit leg.",
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------- the pad

    def _handle_pad_search(self):
        """Hold station and watch for the marker.

        Deliberately NOT a yaw sweep, for the reason precision_land gives:
        the window camera looks forward, where a yaw sweeps new ground; this
        one looks down, where a yaw rotates the footprint and barely changes
        which patch of floor is in it. Turning would cost tracking quality and
        buy almost no coverage, so the capture basket is simply the footprint
        at this altitude -- which is why the transit leg has to put the
        vehicle over the pad, not near it.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        if self.marker_offset_ned() is not None:
            self.get_logger().warning("Marker found. Centring on the pad.")
            self.marker_lost_since = None
            self.pad_in_band_since = None
            self._enter_stage(self.PAD_ALIGN)
            return

        remaining = self.PAD_SEARCH_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._pad_give_up("no marker found in the search window")
            return

        self.get_logger().info(
            f"Searching for the pad, {remaining:.1f} s left. "
            f"{self.marker_summary()}.", throttle_duration_sec=1.0)

    def _handle_pad_align(self):
        """Walk the x/y carrot onto the marker until it is inside tolerance."""
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        offset = self.marker_offset_ned()
        if offset is None:
            # Freeze the carrot. Walking on towards a target derived from a
            # measurement we no longer have is the coasting failure a position
            # setpoint is chosen to avoid in the first place.
            self.moving = False
            if self.marker_lost_since is None:
                self.marker_lost_since = time.monotonic()
                self.get_logger().warning(
                    f"Marker lost mid-align ({self.marker_summary()}); "
                    "holding position.")
            elif (time.monotonic() - self.marker_lost_since
                    >= self.MARKER_LOST_SECONDS):
                self.get_logger().warning("Marker gone. Back to searching.")
                self._enter_stage(self.PAD_SEARCH)
            self.pad_in_band_since = None
            self._check_pad_align_timeout()
            return

        self.marker_lost_since = None
        north, east = offset[0], offset[1]
        error = math.hypot(north, east)
        self.pad_error = error

        # Arrival is tested first, so a vehicle that is already centred is not
        # given one more nudge before it is allowed to settle.
        if error <= self.PAD_ALIGN_TOLERANCE:
            if self.pad_in_band_since is None:
                self.pad_in_band_since = time.monotonic()
            elif (time.monotonic() - self.pad_in_band_since
                    >= self.PAD_ALIGN_SETTLE_SECONDS):
                self._on_pad_aligned(error)
                return
        else:
            self.pad_in_band_since = None

        if not self._pad_drive_towards(north, east):
            self.get_logger().info(
                "Waiting for a flow-healthy x/y hold before correcting...",
                throttle_duration_sec=1.0)
        else:
            body = self.marker_body()
            self.get_logger().info(
                f"Centring: marker {body_words(body[0], body[1])}, err "
                f"{error:.2f} m (want {self.PAD_ALIGN_TOLERANCE:.2f}).",
                throttle_duration_sec=1.0)

        self._check_pad_align_timeout()

    def _pad_drive_towards(self, north, east):
        """Point the base class's x/y carrot at the marker.

        Nothing here talks to PX4: setting move_target_* and `moving` is
        precisely the interface _step_xy_ramp() consumes, so the target is
        walked at MOVE_SPEED, leashed to the measured position, and shifted
        automatically on an EKF2 lateral reset. False means there is no
        latched lateral estimate to express a target in yet.
        """
        if not self.hold_xy or self.local_position is None:
            return False
        lp = self.local_position
        self.move_target_x = lp.x + self.PAD_ALIGN_GAIN * north
        self.move_target_y = lp.y + self.PAD_ALIGN_GAIN * east
        self.moving = True
        return True

    def _check_pad_align_timeout(self):
        if self._in_stage_for() > self.PAD_ALIGN_TIMEOUT:
            self._pad_give_up(
                f"could not centre within {self.PAD_ALIGN_TIMEOUT:.0f} s"
                + ('' if self.pad_error is None
                   else f" (best {self.pad_error:.2f} m)"))

    def _on_pad_aligned(self, error):
        """Stop correcting and park on the point we actually reached.

        Inside the tolerance there is nothing left to win by chasing, and
        re-commanding on every frame would only inject corner noise into the
        position setpoint.
        """
        self.moving = False
        self.pad_aligned_error = error
        lp = self.local_position
        if self.hold_xy and lp is not None:
            self.hold_x = lp.x
            self.hold_y = lp.y
        self.get_logger().warning(
            f"ON THE PAD to {error * 100:.0f} cm. Holding "
            f"{self.PAD_HOLD_SECONDS:.0f} s, then landing.")
        self._enter_stage(self.PAD_HOLD)

    def _handle_pad_hold(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        # Deadband with hysteresis: drift back outside the released band earns
        # another correction, noise inside it does not.
        offset = self.marker_offset_ned()
        if offset is not None:
            error = math.hypot(offset[0], offset[1])
            self.pad_error = error
            if error > self.PAD_ALIGN_TOLERANCE * self.PAD_ALIGN_RELEASE:
                self.get_logger().warning(
                    f"Drifted {error:.2f} m off the pad; correcting again.")
                self.pad_in_band_since = None
                self._enter_stage(self.PAD_ALIGN)
                return

        remaining = self.PAD_HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_precision_descent()
            return

        self.get_logger().info(
            f"On the pad, holding {remaining:.1f} s. {self.marker_summary()}"
            + ('' if self.pad_error is None else f", err {self.pad_error:.2f} m"),
            throttle_duration_sec=1.0)

    def _pad_give_up(self, reason):
        self.pad_outcome = f"FAILED: {reason}"
        if self.PAD_ON_FAIL == 'hold':
            self.get_logger().error(
                f"{reason}. pad_on_fail is 'hold': staying at altitude. "
                "q to descend, k to force-disarm.")
            self.moving = False
            return
        self.get_logger().error(
            f"{reason}. Landing here, OFF THE PAD.")
        self._begin_landing(f"pad approach failed -- {reason}")

    # ----------------------------------------------------------- the descent

    def _begin_precision_descent(self):
        """Descend WITHOUT throwing away the alignment.

        _begin_landing drops to a zero-velocity hold, which is right for a
        generic flight and wrong here: from 2 m at land_speed 0.15 that is
        over ten seconds of unheld descent, and flow bias alone would drift
        further than the tolerance just achieved. So the latched point is put
        back immediately afterwards.

        Every OTHER caller of _begin_landing -- operator abort, a dead height
        estimate, the flight clock, a failed traverse -- keeps the inherited
        behaviour untouched. Those are emergencies, and in an emergency "do
        not translate" is the correct horizontal command.
        """
        self.pad_outcome = (
            'landed on the pad'
            + ('' if self.pad_aligned_error is None
               else f" to {self.pad_aligned_error * 100:.0f} cm"))
        hold_x, hold_y, had_hold = self.hold_x, self.hold_y, self.hold_xy
        self._begin_landing(
            "centred on the pad"
            + ('' if self.pad_aligned_error is None
               else f" to {self.pad_aligned_error * 100:.0f} cm"))
        if had_hold and self.PRECISION_DESCENT:
            self.hold_x, self.hold_y = hold_x, hold_y
            self.hold_xy = True
            self.get_logger().warning(
                f"Precision descent: holding ({hold_x:.2f}, {hold_y:.2f}) NED "
                "all the way down. The marker is expected to leave the frame "
                f"below about {self.BLIND_COMMIT_ALTITUDE:.2f} m -- everything "
                "under that is open loop on this point.")

    def _handle_landing(self):
        # The aligned hold is only worth keeping while the lateral estimate
        # underneath it is still being corrected. The base class never
        # re-checks during a descent because it has already dropped to
        # velocity hold; since a precision descent did not, we have to.
        if (self.pad_aligned_error is not None and self.hold_xy
                and not self.flow_is_healthy()):
            self.hold_xy = False
            self.get_logger().warning(
                "Flow no longer healthy: dropping the aligned point and "
                "finishing the descent on zero-velocity hold.")

        alt = self.relative_altitude()
        if (self.pad_aligned_error is not None and not self._warned_blind
                and alt is not None and alt <= self.BLIND_COMMIT_ALTITUDE):
            self._warned_blind = True
            self.get_logger().warning(
                f"Below {self.BLIND_COMMIT_ALTITUDE:.2f} m: the marker is out "
                "of frame from here. Descent is open loop.")

        super()._handle_landing()

    # --------------------------------------------------------------- status

    def _phase_label(self):
        """The coarse phase, for the TFT and anything logging.

        The inherited version knows nothing about the legs or the pad and
        would label all of them 'OUTSIDE', which on the way home is exactly
        backwards.
        """
        if self.current_stage in self.PAD_STAGES:
            return 'PAD'
        if self.leg == 'outbound':
            return 'TO_WINDOW'
        if self.leg == 'transit':
            return 'TO_PAD'
        return super()._phase_label()

    def publish_status(self):
        """stage|armed|altitude|xy|detail -- the format the display reads.

        The inherited version covers every stage it knows about; the pad
        stages would fall through to it with no detail of their own and leave
        the last traversal's text frozen on the screen.
        """
        if self.current_stage not in self.PAD_STAGES:
            super().publish_status()
            return

        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        left = max(0.0, self.FLIGHT_SECONDS - self.flight_time())

        if self.current_stage == self.PAD_SEARCH:
            detail = f"srch {left:.0f}s"
        elif self.current_stage == self.PAD_ALIGN:
            err = '--' if self.pad_error is None else f"{self.pad_error:.2f}"
            detail = f"err{err} {left:.0f}s"
        else:
            remaining = max(0.0, self.PAD_HOLD_SECONDS - self._in_stage_for())
            detail = f"pad {remaining:.0f}s"

        msg = String()
        msg.data = "|".join([
            self.current_stage,
            'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('FLO' if self.flow_is_healthy() else '---'),
            detail,
        ])
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().warning(
            f"Mission FSM: outbound leg "
            f"{'flown' if self.outbound_done else 'not flown'}; "
            f"transit leg {'flown' if self.transit_done else 'not flown'}; "
            f"pad {self.pad_outcome}.")
        super().destroy_node()


class _Step:
    """The (kind, name, arg) triple the base class's step machinery expects.

    offboard_sequence.parse_sequence returns objects of its own Step class and
    the handlers read `.kind`, `.name` and `.arg` off them. The legs are stored
    here as plain triples -- the same shape window_room_traverse stores its
    room pattern in -- so this is the adapter back. Not imported from
    offboard_sequence because its Step also carries the parse text, which a
    leg rebuilt from a triple does not have.
    """

    __slots__ = ('kind', 'name', 'arg')

    def __init__(self, kind, name, arg):
        self.kind = kind
        self.name = name
        self.arg = arg

    def __str__(self):
        if self.kind == 'yaw':
            return f"yaw {math.degrees(self.arg):+.0f} deg"
        return f"{self.name} {self.arg:.2f} m"


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionFSM()
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        # `node is None` covers a fatal SystemExit out of the constructor -- a
        # malformed leg string -- and the rclpy.ok() guard covers the ordinary
        # path, where the executor has already brought the context down and a
        # second shutdown() raises over the top of whatever really happened.
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
