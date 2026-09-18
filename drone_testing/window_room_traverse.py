#!/usr/bin/env python3
"""
Window traversal, a box pattern inside the room, and a traversal back out.

    arm -> climb -> hold -> find the blue window -> lock -> line up ->
    FLY THROUGH IT and 1.20 m past the plane -> hold ->
    30 cm left -> 30 cm forward -> yaw 90 right -> 30 cm straight ->
    yaw 90 right -> 30 cm straight ->
    find the window AGAIN (the aircraft is now facing the wall it came
    through) -> line up -> fly back out through it -> hold -> land.

EVERYTHING ABOUT THE TRAVERSAL ITSELF IS window_traverse.py's, UNCHANGED.
This module is a subclass and nothing else: the sweep, the lock, the
estimator, RECENTRE/AIM/ALIGN/TRAVERSE/CLEAR, the airframe clearance
arithmetic, the yaw cone, the blind-traverse fallback and every PX4 gate come
straight from WindowTraverse, which is the node that has actually flown. The
only new flight code here is

    * the room pattern (ROOM_MOVE / ROOM_TURN / ROOM_HOLD), which is the
      inherited carrot and yaw ramp driven from a small list of steps, and
    * RELOCK, which throws away everything the estimator believes about the
      first window and lets the SAME approach stages run a second time on the
      far side of it.

Read window_traverse.py first. Nothing it says is repeated here.

NOTE ON THE NODE NAME. Run bare with `ros2 run drone_testing
window_room_traverse` the node still calls itself `window_traverse` -- the
name is hard-coded in the base class's constructor and this file does not
touch that file. The launch file gives it its own name. It changes nothing
except which name the logs and `ros2 node list` show, but it is confusing the
first time, so: if you are looking for the room mission's parameters under
/window_room_traverse and it is not there, you started it by hand.

THE THREE THINGS THAT ARE GENUINELY DIFFERENT ON THE WAY OUT
------------------------------------------------------------
1. THE ESTIMATOR IS CLEARED AT RELOCK. It is not an optimisation. The
   estimator's innovation gate refuses any sample whose normal has swung more
   than gate_yaw_deg from what it already believes, and the normal is always
   resolved to point back at the aircraft -- so from inside the room the same
   physical window produces a normal 180 degrees from the one on file and
   EVERY return sample would be gated out as "normal swung". Clearing the
   buffer is what makes the second approach possible at all.

2. THE YAW CONE IS RE-CENTRED. WindowTraverse refuses to believe in, or turn
   towards, anything more than yaw_cone_deg off the heading the aircraft
   ARMED on -- which is the correct rule going in and exactly backwards coming
   out, where the window is ~180 degrees from that heading. The cone is kept,
   with the same width; only its centre moves, to the heading the aircraft
   holds at the end of the room pattern. So the return leg is still protected
   against locking onto a doorway behind it; it is just protected about the
   right axis.

3. THE ROOM MOVES ARE IN THE CURRENT HEADING FRAME, not the takeoff frame
   OffboardSequence's own steps use. "Go straight" after "yaw 90 right" means
   along the new heading -- that is what makes the pattern a box rather than a
   dog-leg. There is one reference yaw per step, sampled when the step starts.

THE DOLL DETECTOR
-----------------
This node does not do any doll detection. It publishes two things the
detector and the display node consume, and it publishes them regardless of
whether anything is listening:

    /doll_detect_enable   Bool, latched behaviour: True from the moment the
                          aircraft is committed to the inbound traverse,
                          False again once it has cleared the window on the
                          way back out. That is the window of the flight the
                          dolls can possibly be in, and running the model
                          outside it is Jetson time spent on the wall.
    /mission_phase        String, PHASE|STAGE|detail. What the TFT shows.

Phases, in order: OUTSIDE, WINDOW_IN, ENTERING, INSIDE, ROOM, WINDOW_OUT,
EXITING, OUT, LANDING.

GEOMETRY OF THE ROOM PATTERN
----------------------------
Distances are small on purpose -- 30 cm is inside one MOVE_TOLERANCE of the
inherited step logic doubled, so each leg is flown and then SETTLED before
the next starts, and the pattern is about pointing the camera into the room
rather than about covering it. All six legs are parameters; the defaults are
the pattern asked for:

    room_strafe        0.30  m left, immediately after the traverse hold
    room_forward_1     0.30  m forward
    room_turn_1_deg   +90.0  deg (positive = to the RIGHT, NED convention)
    room_forward_2     0.30  m straight ahead on the new heading
    room_turn_2_deg   +90.0  deg
    room_forward_3     0.30  m straight ahead again

After the second turn the nose is 180 degrees from the entry heading, i.e.
pointing back at the wall the window is in, which is the geometry RELOCK
needs. Change the turns and you change that; the node will still fly it, and
it will still refuse to fly at a window outside the cone about wherever the
pattern left it pointing.
"""

import math
import time

import rclpy
from px4_msgs.msg import VehicleStatus
from std_msgs.msg import Bool, String

from drone_testing.offboard_sequence import (DIRECTIONS, parse_sequence,
                                             spin_node, wrap_pi)
from drone_testing.window_traverse import WindowTraverse


class WindowRoomTraverse(WindowTraverse):
    """WindowTraverse, flown twice, with a box pattern in between."""

    # ---- the new stages ---------------------------------------------------
    ROOM_MOVE = "ROOM_MOVE"     # flying one leg of the box
    ROOM_TURN = "ROOM_TURN"     # yawing on the spot between legs
    ROOM_HOLD = "ROOM_HOLD"     # settling after a leg, so the next one starts
                                # from a stationary vehicle and the errors do
                                # not compound down the pattern
    RELOCK = "RELOCK"           # stationary, facing the wall, rebuilding a
                                # window pose from nothing

    ROOM_STAGES = (ROOM_MOVE, ROOM_TURN, ROOM_HOLD, RELOCK)

    # ---- the mission phases, as published on /mission_phase ---------------
    PHASE_OUTSIDE = 'OUTSIDE'
    PHASE_IN = 'IN'             # committed to the inbound traverse, or inside
    PHASE_ROOM = 'ROOM'
    PHASE_OUT = 'OUT'           # looking for the window from inside, or leaving

    # ---- the traversals ---------------------------------------------------
    # How far past the window plane the INBOUND run ends: the "120 cm inside"
    # of the mission. This is WindowTraverse's exit_distance under a name that
    # says which side of the wall it is on, because there are now two of them.
    INSIDE_DISTANCE = 1.20
    # ... and the outbound one. Far enough out that the aircraft is clear of
    # the wall before it stops, which is all this one has to be.
    OUTSIDE_DISTANCE = 1.20

    # ---- the room pattern -------------------------------------------------
    ROOM_STRAFE = 0.30          # m left
    ROOM_FORWARD_1 = 0.30       # m forward
    ROOM_TURN_1_DEG = 90.0      # deg, positive = right (clockwise from above)
    ROOM_FORWARD_2 = 0.30       # m straight on the new heading
    ROOM_TURN_2_DEG = 90.0      # deg
    ROOM_FORWARD_3 = 0.30       # m straight on the new heading

    ROOM_SPEED = 0.20           # m/s the carrot is walked at inside the room.
                                # Slower than the approach: the legs are 30 cm
                                # long, so the vehicle spends all of each one
                                # accelerating and decelerating, and a higher
                                # carrot speed just buys overshoot. It is also
                                # the speed the doll detector sees the room at.
    ROOM_HOLD_SECONDS = 2.0     # s of station keeping between legs
    ROOM_MOVE_TIMEOUT = 20.0    # s for one 30 cm leg before it is given up on
    ROOM_TURN_TIMEOUT = 25.0    # s for one 90 degree turn

    # ---- finding the window again -----------------------------------------
    RELOCK_TIMEOUT = 45.0       # s of standing still looking at the wall
                                # before the attempt to leave through the
                                # window is given up and the aircraft lands
                                # where it is. Generous, because this is the
                                # one stage with nothing else to fall back on.
    RELOCK_SETTLE_SECONDS = 2.0 # s of holding still before the estimate is
                                # believed at all. The pattern ends with a
                                # turn, the turn smears both the flow and the
                                # depth, and a pose built during the settle is
                                # built on the worst frames of the flight.

    # Two traversals, a box pattern and two lots of settling do not fit in the
    # single-traversal clock.
    FLIGHT_SECONDS = 300.0

    def __init__(self):
        super().__init__()

        self.INSIDE_DISTANCE = float(self._declare_number(
            'inside_distance', self.INSIDE_DISTANCE))
        self.OUTSIDE_DISTANCE = float(self._declare_number(
            'outside_distance', self.OUTSIDE_DISTANCE))
        # The inbound traversal is flown by the inherited code, which reads
        # EXIT_DISTANCE. Point it at the inbound number now and swap it for the
        # outbound one at RELOCK; nothing else has to know there are two.
        self.EXIT_DISTANCE = self.INSIDE_DISTANCE

        self.ROOM_SPEED = float(self._declare_number('room_speed', self.ROOM_SPEED))
        self.ROOM_HOLD_SECONDS = float(self._declare_number(
            'room_hold_seconds', self.ROOM_HOLD_SECONDS))
        self.ROOM_MOVE_TIMEOUT = float(self._declare_number(
            'room_move_timeout', self.ROOM_MOVE_TIMEOUT))
        self.ROOM_TURN_TIMEOUT = float(self._declare_number(
            'room_turn_timeout', self.ROOM_TURN_TIMEOUT))
        self.RELOCK_TIMEOUT = float(self._declare_number(
            'relock_timeout', self.RELOCK_TIMEOUT))
        self.RELOCK_SETTLE_SECONDS = float(self._declare_number(
            'relock_settle_seconds', self.RELOCK_SETTLE_SECONDS))

        # The six-leg box, always DECLARED so a launch file may pass
        # room_strafe / room_forward_N / room_turn_N_deg whichever form is
        # in use -- an undeclared override is an error at startup, and a
        # launch file cannot know which branch below will be taken.
        default_room_steps = [
            ('move', 'left',
             float(self._declare_number('room_strafe', self.ROOM_STRAFE))),
            ('move', 'forward',
             float(self._declare_number('room_forward_1', self.ROOM_FORWARD_1))),
            ('yaw', 'right',
             math.radians(float(self._declare_number(
                 'room_turn_1_deg', self.ROOM_TURN_1_DEG)))),
            ('move', 'forward',
             float(self._declare_number('room_forward_2', self.ROOM_FORWARD_2))),
            ('yaw', 'right',
             math.radians(float(self._declare_number(
                 'room_turn_2_deg', self.ROOM_TURN_2_DEG)))),
            ('move', 'forward',
             float(self._declare_number('room_forward_3', self.ROOM_FORWARD_3))),
        ]

        # The pattern, as (kind, name, argument).
        #
        # `room_sequence` is the free-form form: the SAME grammar
        # offboard_sequence takes, so the in-room moves are whatever you type
        # rather than the shape this class was written around --
        #
        #     -p room_sequence:="forward 0.5, yaw 90, left 0.3, backward 0.4"
        #
        # Empty (the default) keeps the six-leg box above, built from its own
        # parameters, so nothing that already flies changes.
        #
        # Only translations and yaws are accepted here. `up`/`down` are not:
        # the room stages dispatch to _handle_room_move / _handle_room_turn and
        # there is no altitude handler among them, and an altitude change
        # inside a room is a change to the height the return approach lines up
        # at -- altitude_offset is where that belongs. A malformed string is
        # FATAL at construction, exactly as in offboard_sequence: a typo in a
        # flight plan must not be quietly guessed at.
        room_sequence = str(self.declare_parameter('room_sequence', '').value).strip()
        if room_sequence:
            try:
                parsed = parse_sequence(room_sequence)
            except ValueError as exc:
                raise SystemExit(f"room_sequence: {exc}")
            if not parsed:
                raise SystemExit("room_sequence: no steps in "
                                 f"'{room_sequence}'")
            for step in parsed:
                if step.kind == 'alt':
                    raise SystemExit(
                        f"room_sequence: '{step.name}' is not a room move; "
                        "only forward/backward/left/right/yaw are flown "
                        "inside the room (use altitude_offset to change the "
                        "height the traversal is flown at)")
            # The yaw sign convention is offboard_sequence's and this class's
            # alike -- positive is to the RIGHT, seen from above -- so the
            # radians parse_sequence produced go straight in. The name is only
            # ever used for the log line and the four-character TFT field.
            self.room_steps = [(step.kind, step.name, step.arg)
                               for step in parsed]
        else:
            self.room_steps = default_room_steps

        # Whether to attempt the way out at all. false = fly in, do the room
        # pattern, land inside. Useful for a first flight in a new room, where
        # the question is whether the pattern fits before whether it can be
        # reversed.
        self.return_through_window = bool(self.declare_parameter(
            'return_through_window', True).value)

        # The yaw cone's centre. The inherited code hard-codes home_yaw; this
        # node moves it once, at RELOCK. See _clamp_to_cone and _bearing_to.
        self.cone_centre = None     # None = "use home_yaw", i.e. inbound

        self.phase = self.PHASE_OUTSIDE
        self.room_index = 0
        self.room_started = False
        self.room_results = []
        self.relock_started = None
        self.dolls_armed = False
        self.inbound_outcome = 'not attempted'

        self.phase_pub = self.create_publisher(String, 'mission_phase', 10)
        # Latched-ish: republished every tick, so a detector that starts late
        # still gets told to run rather than waiting for an edge it missed.
        self.doll_enable_pub = self.create_publisher(Bool, 'doll_detect_enable', 10)

        self.get_logger().warning(
            f"ROOM MISSION: through the window and {self.INSIDE_DISTANCE:.2f} m "
            f"in, then {self._room_plan_text()}, then "
            + ("find the window again and fly back out "
               f"{self.OUTSIDE_DISTANCE:.2f} m, then land. "
               if self.return_through_window else
               "land inside (return_through_window:=false). ") +
            f"Doll detection runs from the inbound commit to the outbound "
            f"clear. Hard clock {self.FLIGHT_SECONDS:.0f} s.")

    def _room_plan_text(self):
        """The room pattern as one human-readable line, any length.

        Used only for the startup WARN, which is the one place the whole plan
        is printed before it is flown -- so it must read back whatever
        room_sequence was given, not the shape this class happens to default
        to.
        """
        parts = []
        for kind, name, arg in self.room_steps:
            if kind == 'yaw':
                parts.append(f"yaw {math.degrees(arg):+.0f} deg")
            else:
                parts.append(f"{arg:.2f} m {name}")
        return ", ".join(parts)

    # ------------------------------------------------------------ the cone

    def _cone_centre(self):
        """The heading the yaw cone is measured about.

        home_yaw until the room pattern has turned the aircraft round, then
        whatever heading it ended on. Both the "do not turn further than this"
        clamp and the "do not believe a window further off than this" test go
        through here, so the two cannot disagree about which way the aircraft
        is supposed to be looking.
        """
        return self.home_yaw if self.cone_centre is None else self.cone_centre

    def _clamp_to_cone(self, heading):
        if self.YAW_CONE <= 0.0 or self.home_z is None:
            return heading
        centre = self._cone_centre()
        off = wrap_pi(heading - centre)
        if abs(off) <= self.YAW_CONE:
            return heading
        clamped = wrap_pi(centre + math.copysign(self.YAW_CONE, off))
        self.get_logger().warning(
            f"Yaw cone: {math.degrees(heading):+.0f} deg is "
            f"{math.degrees(abs(off)):.0f} deg off the "
            f"{math.degrees(centre):+.0f} deg cone centre; commanding "
            f"{math.degrees(clamped):+.0f} deg instead.",
            throttle_duration_sec=2.0)
        return clamped

    def _bearing_to(self, point):
        """Bearing from the vehicle to a NED point, off the cone centre."""
        lp = self.local_position
        if lp is None:
            return 0.0
        return wrap_pi(math.atan2(point[1] - lp.y, point[0] - lp.x)
                       - self._cone_centre())

    def _on_heading_reset(self, delta):
        super()._on_heading_reset(delta)
        if self.cone_centre is not None:
            self.cone_centre = wrap_pi(self.cone_centre + delta)

    # ----------------------------------------------------------- the clock

    def _clock_stages(self):
        """The room stages are under the hard clock; TRAVERSE still is not.

        RELOCK especially: it is a stationary stage with a timeout of its own,
        and an aircraft that has been hunting for the window for 45 s has
        spent battery the landing still needs.
        """
        return super()._clock_stages() + self.ROOM_STAGES

    # ---------------------------------------------------------- the machine

    def timer_callback(self):
        self._publish_mission_phase()

        if self.current_stage not in self.ROOM_STAGES:
            # Everything else is WindowTraverse's or the base class's, reached
            # through the same call it always was.
            super().timer_callback()
            return

        self.publish_window_pose()

        if self._check_flight_clock():
            return

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

        self._track_pose_health()

        {
            self.ROOM_MOVE: self._handle_room_move,
            self.ROOM_TURN: self._handle_room_turn,
            self.ROOM_HOLD: self._handle_room_hold,
            self.RELOCK: self._handle_relock,
        }[self.current_stage]()

    # ------------------------------------------------- arming the detector

    def _arm_dolls(self, reason):
        if self.dolls_armed:
            return
        self.dolls_armed = True
        self.get_logger().warning(f"DOLL DETECTION ON: {reason}.")

    def _disarm_dolls(self, reason):
        if not self.dolls_armed:
            return
        self.dolls_armed = False
        self.get_logger().warning(f"DOLL DETECTION OFF: {reason}.")

    def _begin_traverse(self):
        """Commit, and turn the model on as the aircraft enters the aperture.

        Armed HERE rather than at the far side of the window: the commit is
        the last moment the flight is certainly still going to go through, and
        the few seconds of aperture and near wall the model sees on the way in
        cost nothing -- there are no dolls in them to double count, and the
        geotagging would reject them anyway.
        """
        super()._begin_traverse()
        if self.current_stage == self.TRAVERSE and self.phase == self.PHASE_OUTSIDE:
            self.phase = self.PHASE_IN
            self._arm_dolls("committed to the inbound traverse")

    # --------------------------------------------------------------- CLEAR

    def _handle_clear(self):
        """The far side of a traversal. Which one decides what happens next.

        Inbound: the hold is the end of the traverse and the start of the room
        pattern. Outbound (or with return_through_window false): the inherited
        behaviour, which is to land.
        """
        if self.phase != self.PHASE_IN or not self.return_through_window:
            if self.phase == self.PHASE_OUT:
                self._disarm_dolls("cleared the window on the way out")
            super()._handle_clear()
            return

        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        remaining = self.CLEAR_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_room()
            return

        self.get_logger().info(
            f"INSIDE: holding, {remaining:.1f} s to the room pattern.",
            throttle_duration_sec=1.0)

    # --------------------------------------------------------- room pattern

    def _begin_room(self):
        self.inbound_outcome = self.outcome
        self.phase = self.PHASE_ROOM
        self.room_index = 0
        self.room_results = []
        self.MOVE_SPEED = self.ROOM_SPEED
        self.get_logger().warning(
            f"ROOM: inside and {self.INSIDE_DISTANCE:.2f} m past the window. "
            f"Flying the {len(self.room_steps)}-leg pattern at "
            f"{self.ROOM_SPEED:.2f} m/s, {self.ROOM_HOLD_SECONDS:.0f} s of "
            "settling between legs.")
        self._start_room_step()

    def _start_room_step(self):
        if self.room_index >= len(self.room_steps):
            self._finish_room()
            return
        kind, name, arg = self.room_steps[self.room_index]
        self.room_started = False
        self.moving = False
        self.yaw_remaining = 0.0
        self.move_in_band_since = None
        self.yaw_in_band_since = None
        self._enter_stage(self.ROOM_MOVE if kind == 'move' else self.ROOM_TURN)

    def _finish_room_step(self, outcome):
        kind, name, arg = self.room_steps[self.room_index]
        self.room_results.append(f"{name} {arg:.2f} -> {outcome}")
        self.get_logger().warning(
            f"Room leg {self.room_index + 1}/{len(self.room_steps)} "
            f"({name}): {outcome}")
        self.moving = False
        self.yaw_remaining = 0.0
        self.room_started = False
        self._enter_stage(self.ROOM_HOLD)

    def _handle_room_hold(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        remaining = self.ROOM_HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self.room_index += 1
            self._start_room_step()
            return

        self.get_logger().info(
            f"ROOM: settling, {remaining:.1f} s to the next leg.",
            throttle_duration_sec=1.0)

    def _handle_room_move(self):
        """One translation leg, in the CURRENT heading frame.

        The same shape as OffboardSequence._run_move_step -- latch gate, carrot
        target, tolerance, settle, timeout -- but resolved against the yaw the
        aircraft is actually holding rather than the takeoff heading, because
        "go straight" after a turn has to mean the new straight.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        _, name, distance = self.room_steps[self.room_index]

        if not self.hold_xy:
            # No latched frame, so no point to fly to. Exactly the base class's
            # verdict: skip the leg rather than dead-reckon it on velocity.
            if self.room_started:
                self.moving = False
                self._finish_room_step("ABANDONED, flow lost mid-leg")
                return
            if self._in_stage_for() > self.MOVE_LATCH_TIMEOUT:
                self._finish_room_step("SKIPPED, flow never latched x/y")
            else:
                self.get_logger().info(
                    "ROOM: waiting for a flow-healthy x/y hold before moving...",
                    throttle_duration_sec=1.0)
            return

        if not self.room_started:
            ref_yaw = self.yaw_setpoint
            ux, uy = DIRECTIONS[name](math.cos(ref_yaw), math.sin(ref_yaw))
            self.move_start_x = self.hold_x
            self.move_start_y = self.hold_y
            self.move_target_x = self.hold_x + ux * distance
            self.move_target_y = self.hold_y + uy * distance
            self.move_in_band_since = None
            self.moving = True
            self.room_started = True
            self._restart_stage_clock()
            self.get_logger().warning(
                f"Room leg {self.room_index + 1}/{len(self.room_steps)}: "
                f"{distance:.2f} m {name} at {self.MOVE_SPEED:.2f} m/s in the "
                f"{math.degrees(ref_yaw):+.0f} deg frame: "
                f"({self.move_start_x:.2f}, {self.move_start_y:.2f}) -> "
                f"({self.move_target_x:.2f}, {self.move_target_y:.2f}) NED.")
            return

        lp = self.local_position
        remaining = math.hypot(self.move_target_x - lp.x, self.move_target_y - lp.y)

        if remaining <= self.MOVE_TOLERANCE:
            if self.move_in_band_since is None:
                self.move_in_band_since = time.monotonic()
            elif time.monotonic() - self.move_in_band_since >= self.MOVE_SETTLE_SECONDS:
                travelled = math.hypot(lp.x - self.move_start_x,
                                       lp.y - self.move_start_y)
                self.hold_x = self.move_target_x
                self.hold_y = self.move_target_y
                self._finish_room_step(
                    f"done, {travelled:.2f} m of {distance:.2f} m")
            return

        self.move_in_band_since = None
        self.get_logger().info(
            f"ROOM: {remaining:.2f} m to go on the {name} leg.",
            throttle_duration_sec=1.0)

        if self._in_stage_for() > self.ROOM_MOVE_TIMEOUT:
            self.moving = False
            self.hold_x = lp.x
            self.hold_y = lp.y
            self._finish_room_step(f"TIMED OUT {remaining:.2f} m short")

    def _handle_room_turn(self):
        """One yaw leg, on the spot, ramped.

        Deliberately NOT through _aim_yaw_at: that clamps to the yaw cone, and
        the whole point of these two turns is to leave the cone the inbound
        approach was flown in. The cone is re-centred at the end of the
        pattern instead, which is the honest place to do it -- the aircraft has
        by then actually turned round.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        _, _, angle = self.room_steps[self.room_index]

        if not self.room_started:
            self.yaw_remaining = angle
            self.yaw_in_band_since = None
            self.room_started = True
            self._restart_stage_clock()
            target = math.degrees(wrap_pi(self.yaw_setpoint + angle))
            self.get_logger().warning(
                f"Room leg {self.room_index + 1}/{len(self.room_steps)}: yaw "
                f"{math.degrees(angle):+.0f} deg at "
                f"{math.degrees(self.YAW_RATE):.0f} deg/s, to a heading of "
                f"{target:+.0f} deg. Holding position through the turn.")
            return

        lp = self.local_position
        error = abs(wrap_pi(self.yaw_setpoint - lp.heading))

        if abs(self.yaw_remaining) < 1e-3 and error <= self.YAW_TOLERANCE:
            if self.yaw_in_band_since is None:
                self.yaw_in_band_since = time.monotonic()
            elif time.monotonic() - self.yaw_in_band_since >= self.YAW_SETTLE_SECONDS:
                self._finish_room_step(
                    f"done, heading {math.degrees(lp.heading):+.0f} deg")
            return

        self.yaw_in_band_since = None
        self.get_logger().info(
            f"ROOM: yawing, {math.degrees(abs(self.yaw_remaining)):.0f} deg of "
            f"setpoint left, airframe {math.degrees(error):.0f} deg behind it.",
            throttle_duration_sec=1.0)

        if self._in_stage_for() > self.ROOM_TURN_TIMEOUT:
            self.yaw_remaining = 0.0
            self.yaw_setpoint = lp.heading
            self._finish_room_step(
                f"TIMED OUT at {math.degrees(lp.heading):+.0f} deg, "
                f"{math.degrees(error):.0f} deg short")

    def _finish_room(self):
        self.get_logger().warning(
            "ROOM pattern complete: " + "; ".join(self.room_results) + ".")
        if not self.return_through_window:
            self.outcome = "room pattern flown; landing inside (no return leg)"
            self._disarm_dolls("landing inside, no return through the window")
            self._begin_landing("room pattern complete, return leg disabled")
            return
        self._begin_relock()

    # -------------------------------------------------------------- RELOCK

    def _begin_relock(self):
        """Forget the first window and start looking for it from the inside.

        The three things that have to be reset are the estimator's samples
        (their normals point the wrong way -- see the module header), the
        "last good" pose the commit is allowed to fall back on (it describes
        an approach from the other side of the wall and committing on it would
        fly the aircraft backwards through the window it is standing behind),
        and the cone.
        """
        lp = self.local_position
        heading = self.yaw_setpoint if lp is None else lp.heading

        # Under the estimator's own lock: geometry_callback runs on the sensor
        # thread and clearing the deque from under an in-flight _add() is the
        # one race this node could have.
        with self.estimator._lock:
            self.estimator.samples.clear()
            self.estimator.consecutive_gated = 0
        self.last_good_est = None
        self.last_good_est_time = 0.0
        self.last_detection = None
        self.pose_ok_since = None
        self.pose_lost_since = None
        self.traverse_window = None
        self.traverse_entry = None
        self.traverse_exit = None
        self.traverse_heading = None
        self.traverse_standoff = None
        self.recentre_backoffs = 0
        self.recentre_backoff_target = None
        self.recentre_untruncated_since = None

        self.cone_centre = heading
        self.EXIT_DISTANCE = self.OUTSIDE_DISTANCE
        self.MOVE_SPEED = self.APPROACH_SPEED
        self.moving = False
        self.phase = self.PHASE_OUT
        self.relock_started = time.monotonic()
        self._enter_stage(self.RELOCK)

        self.get_logger().warning(
            f"RELOCK: facing {math.degrees(heading):+.0f} deg, which is now the "
            f"centre of the {math.degrees(self.YAW_CONE):.0f} deg yaw cone. "
            "The window pose has been cleared -- from this side the same window "
            "has the opposite normal and every sample would be gated out "
            "against the old one. Rebuilding it from scratch, then flying back "
            f"out to {self.OUTSIDE_DISTANCE:.2f} m beyond the plane.")

    def _handle_relock(self):
        """Stand still facing the wall until there is a pose worth flying at.

        The same job LOCK does on the way in, and it hands over to exactly the
        same stages: _begin_recentre if the window is hanging off the frame
        edge, _begin_aim once there is a pose. What it does not do is sweep --
        the aircraft is a metre from the wall in a room it has just flown a
        pattern in, and yawing to search from here is how it finds a doorway
        instead of the window.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        settling = self._in_stage_for() < self.RELOCK_SETTLE_SECONDS
        if settling:
            self.get_logger().info(
                f"RELOCK: settling after the turn, "
                f"{self.RELOCK_SETTLE_SECONDS - self._in_stage_for():.1f} s.",
                throttle_duration_sec=1.0)
            return

        if self._in_stage_for() > self.RELOCK_TIMEOUT:
            self.outcome = (
                f"ABANDONED: no window pose from inside the room within "
                f"{self.RELOCK_TIMEOUT:.0f} s")
            self._disarm_dolls("giving up on the return leg")
            self._begin_landing(
                f"no usable window pose from inside after "
                f"{self.RELOCK_TIMEOUT:.0f} s -- landing in the room")
            return

        est = self.window_estimate()
        if est is None:
            det = self.fresh_detection()
            if det is not None and det['truncated']:
                self._begin_recentre(det)
                return
            self.get_logger().info(
                f"RELOCK: rebuilding the window pose from inside. "
                f"{self.pose_summary()}", throttle_duration_sec=1.0)
            return

        if not self.hold_xy:
            self.get_logger().warning(
                "RELOCK: window pose is ready but the flow estimate is not "
                "healthy enough to fly to a point. Waiting.",
                throttle_duration_sec=2.0)
            return

        self.get_logger().warning(
            f"RELOCK: window found from inside. {self.pose_summary()}.")
        self._begin_aim(est)

    # ------------------------------------------------------------- landing

    def _begin_landing(self, reason):
        # Whatever brought the flight down -- a completed mission, an abort, a
        # failsafe -- the model has nothing left to look at and the Jetson has
        # a descent to keep setpoints flowing through.
        self._disarm_dolls(f"landing ({reason})")
        super()._begin_landing(reason)

    # ------------------------------------------------------------ publishing

    def _phase_label(self):
        """The coarse mission phase, for the TFT and for anything logging.

        Derived from the stage and the phase flag rather than stored, so it
        cannot drift out of step with what the aircraft is actually doing.
        """
        stage = self.current_stage
        if stage in (self.LANDING, self.DISARMING, self.KILLING, self.DONE):
            return 'LANDING' if stage == self.LANDING else stage
        if stage in self.ROOM_STAGES:
            return 'ROOM' if stage != self.RELOCK else (
                'WINDOW_OUT' if self.window_estimate() is not None else 'SEARCH_OUT')
        if stage == self.TRAVERSE:
            return 'EXITING' if self.phase == self.PHASE_OUT else 'ENTERING'
        if stage == self.CLEAR:
            return 'OUT' if self.phase == self.PHASE_OUT else 'INSIDE'
        if stage in (self.RECENTRE, self.AIM, self.ALIGN):
            return 'WINDOW_OUT' if self.phase == self.PHASE_OUT else 'WINDOW_IN'
        if stage in (self.SCAN, self.LOCK):
            return 'WINDOW_IN' if self.window_estimate() is not None else 'SEARCH'
        return 'OUTSIDE'

    def _publish_mission_phase(self):
        """PHASE|STAGE|detail, plus the detector's enable flag.

        Published from the timer on every tick and in every stage, including
        the ones this class does not otherwise touch, so a display or a
        detector that starts after the flight does still learns the phase
        within 50 ms instead of waiting for the next transition.
        """
        msg = String()
        msg.data = "|".join([
            self._phase_label(),
            self.current_stage,
            f"{self.room_index + 1}/{len(self.room_steps)}"
            if self.current_stage in (self.ROOM_MOVE, self.ROOM_TURN,
                                      self.ROOM_HOLD) else '',
        ])
        self.phase_pub.publish(msg)

        enable = Bool()
        enable.data = self.dolls_armed
        self.doll_enable_pub.publish(enable)

    def publish_status(self):
        """stage|armed|altitude|xy|detail -- the format the display node reads.

        The inherited version covers every stage it knows about; the four new
        ones would otherwise fall through to the base class's, which has no
        detail for them and would leave the last traversal's text frozen on
        the screen.
        """
        if self.current_stage not in self.ROOM_STAGES:
            super().publish_status()
            return

        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        n = f"{self.room_index + 1}/{len(self.room_steps)}"

        if self.current_stage == self.ROOM_MOVE:
            lp = self.local_position
            if self.moving and lp is not None and self.move_target_x is not None:
                left = math.hypot(self.move_target_x - lp.x,
                                  self.move_target_y - lp.y)
                detail = f"{n} {self.room_steps[self.room_index][1][:3]}{left:.2f}"
            else:
                detail = f"{n} wait"
        elif self.current_stage == self.ROOM_TURN:
            detail = f"{n} yaw{math.degrees(abs(self.yaw_remaining)):.0f}"
        elif self.current_stage == self.ROOM_HOLD:
            detail = f"{n} {max(0.0, self.ROOM_HOLD_SECONDS - self._in_stage_for()):.0f}s"
        else:
            detail = f"relock {self._in_stage_for():.0f}s"

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
            f"Room mission: inbound {self.inbound_outcome}; room "
            + ("; ".join(self.room_results) if self.room_results else "not flown")
            + f"; outbound {self.outcome}.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WindowRoomTraverse()
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
