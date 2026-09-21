"""
MISSION, PART 2 OF 2: WINDOW MARKER -> DARK ROOM -> BACK ONTO THE MARKER.

    (placed on the window marker, FACING THE WINDOW)
    arm -> climb to 1.75 m -> hold
      -> strafe LEFT 0.15 m onto the window's axis
      -> find the window, line up, traverse in to 0.80 m past the window plane.
         The doll detector is switched on at the first window lock.
      -> dark room at 1.75 m, flown on the lidar wall fix:
             forward 0.8, right 0.8, backward 0.8, left 0.8, yaw 180
         dolls geotagged and counted (/doll_count, printed in the terminal)
      -> relock on the window from inside, traverse out to 1.67 m past the
         window plane, which is the marker's line
      -> strafe RIGHT 0.15 m back off the axis, over the marker. Call it P.
      -> hover at P for up to 10 s, looking for the marker:
           seen     -> centre on it -> PLAIN descent -> disarm
           centring fails -> fly back to P, hover and look for another 10 s
                             (it may try centring once more)
           not seen -> SLOW descent at P, holding P -> disarm

    ros2 run drone_testing mission_fsm_part2
    ros2 launch drone_testing mission_fsm_part2.launch.py      (support stack)

Part 1 (mission_fsm_part1.py) is what puts the aircraft on that marker.

EVERYTHING IS mission_fsm's
---------------------------
The strafes, the window search ladder, both traversals, the lidar room
scan, the doll gating and the relock are MissionFSM's, reached through the
same calls it always makes. What this file changes:

  * the plan: 'offset' -> window -> room -> out -> 'offset_back' -> 'final'.
    No outbound leg, no return/turn/pad legs.
  * one altitude, 1.75 m, for the climb, the strafes, the window and the
    room. Lowered from the window centre (1.90 m) on instruction.
  * 'final' is a zero-length marker leg with its own handler: hover at P,
    look, centre if seen, and come back to P if centring fails. It reuses
    MARKER_ALIGN / MARKER_HOLD unchanged, so the centring loop and its
    tolerances are the full mission's.

DIRECTIONS ARE IN THE TAKEOFF FRAME. Coming out of the room the airframe
faces back down the course (it yawed 180 inside), but 'right' still means
right of the heading it was ARMED at, so the strafe out mirrors the strafe in.
The window search also centres its yaw cone on the armed heading: place the
aircraft on the marker pointing at the window.

THE SLOW DESCENT AND PX4's LAND DETECTOR
----------------------------------------
PX4 only declares ground contact once the descent setpoint is at least
0.9 * MPC_LAND_SPEED (see OffboardSequence._touchdown_confirmed). If
slow_land_speed is below that, touchdown is confirmed by the node's own
stalled-descent test instead, which takes a few seconds longer on the ground.
To avoid that, set MPC_LAND_SPEED at or below slow_land_speed / 0.9.

Keys:  q -> abort into a controlled descent from wherever we are.
       k -> force-disarm immediately (motors cut, vehicle drops).
"""

import math
import time

import rclpy
from std_msgs.msg import Int32

from drone_testing.mission_fsm import Leg, MissionFSM
from drone_testing.offboard_sequence import spin_node


class MissionFSMPart2(MissionFSM):
    """Window marker -> window -> dark room -> window -> back onto the marker."""

    # ---- one height for the whole flight --------------------------------
    CRUISE_ALTITUDE = 1.75      # m. Climb, strafes and the window approach.
    WINDOW_ALTITUDE = 1.75
    ROOM_ALTITUDE = 1.75

    # ---- the course, measured -------------------------------------------
    WINDOW_OFFSET = 0.15        # m LEFT, window marker -> window axis
    WINDOW_OFFSET_DIRECTION = 'left'
    STANDOFF_DISTANCE = 1.67    # m, marker to the window plane
    INSIDE_DISTANCE = 0.80      # m past the window plane on the way in
    OUTSIDE_DISTANCE = 1.67     # m past it on the way out = the marker line
    ROOM_SEQUENCE = 'forward 0.8, right 0.8, backward 0.8, left 0.8, yaw 180'

    # ---- landing back on the marker -------------------------------------
    FINAL_LOOK_SECONDS = 10.0   # s hovering at P looking for the marker
    FINAL_CENTRING_ATTEMPTS = 2 # centring tries before landing at P anyway
    SLOW_LAND_SPEED = 0.10      # m/s for the fallback descent at P

    # ---- the way out ------------------------------------------------------
    # 'retrace' (default): the window is UNMARKED on the inside, so it cannot
    # be re-found from inside with the camera. The aircraft instead returns to
    # the height it went in at and flies straight back out along the takeoff
    # heading -- it entered on the window axis and the room hold (lidar) or
    # the turn on the spot keeps it there. 'relock' is the inherited camera
    # relock, for a window that is marked on both faces.
    EXIT_MODE = 'retrace'

    WINDOW_MARKER_ID = 0        # the ArUco in front of the window
    FLIGHT_SECONDS = 420.0      # two traversals and the room. A backstop.

    def __init__(self):
        super().__init__()

        self.FINAL_LOOK_SECONDS = float(self._declare_number(
            'final_look_seconds', self.FINAL_LOOK_SECONDS))
        self.FINAL_CENTRING_ATTEMPTS = int(self.declare_parameter(
            'final_centring_attempts', self.FINAL_CENTRING_ATTEMPTS).value)
        self.SLOW_LAND_SPEED = float(self._declare_number(
            'slow_land_speed', self.SLOW_LAND_SPEED))

        self.EXIT_MODE = str(self.declare_parameter(
            'exit_mode', self.EXIT_MODE).value).strip().lower()
        if self.EXIT_MODE not in ('retrace', 'relock'):
            raise SystemExit(
                f"exit_mode must be 'retrace' or 'relock'; got '{self.EXIT_MODE}'.")
        self.inbound_z = None           # target_z of the inbound traverse

        # Both strafes are always in the plan, even at window_offset 0: a
        # zero-length pure-distance leg just settles for leg_settle_seconds,
        # and MissionFSM._handle_clear hands over to 'offset_back' by name.
        back = 'right' if self.WINDOW_OFFSET_DIRECTION == 'left' else 'left'
        self.legs = [
            Leg('offset', self.WINDOW_OFFSET_DIRECTION, self.WINDOW_OFFSET,
                None, 'window'),
            Leg('offset_back', back, self.WINDOW_OFFSET, None, 'final'),
            # Zero length: _handle_final drives it, not the cruise handler.
            Leg('final', back, 0.0, self.WINDOW_MARKER_ID, 'land',
                retry=False),
            # exit_mode 'retrace': straight back out, takeoff-frame BACKWARD
            # (the window is ahead of the armed heading), pure distance.
            Leg('exit', 'backward', self.INSIDE_DISTANCE + self.OUTSIDE_DISTANCE,
                None, 'exit_clear'),
        ]
        self.leg_by_name = {leg.name: i for i, leg in enumerate(self.legs)}

        self.final_point = None         # P: (x, y) NED where the strafe ended
        self.final_look_since = None    # monotonic, start of this look at P
        self.final_centring = False     # in MARKER_ALIGN/HOLD from a look
        self.final_attempts = 0         # centring tries that failed
        self.final_returning = False    # flying back to P
        self.final_return_since = None

        # The detector's running total, for the log. doll_report_text prints
        # the same topic, minus its display offset.
        self.create_subscription(Int32, 'doll_count', self.doll_count_callback, 10)

    def _plan_summary(self):
        # Called from MissionFSM.__init__ before self.legs is replaced, so it
        # reads the parameters rather than the leg list.
        back = 'right' if self.WINDOW_OFFSET_DIRECTION == 'left' else 'left'
        return (
            "MISSION FSM PART 2, one flight. Directions in the TAKEOFF frame:\n"
            f"  1. climb {self.CRUISE_ALTITUDE:.2f} m, hold "
            f"{self.HOLD_SECONDS:.0f} s\n"
            f"  2. strafe {self.WINDOW_OFFSET_DIRECTION} "
            f"{self.WINDOW_OFFSET:.2f} m onto the window axis\n"
            f"  3. find the window, traverse in to {self.INSIDE_DISTANCE:.2f} m "
            "past it; doll detection on at the first lock\n"
            f"  4. room at {self.ROOM_ALTITUDE:.2f} m on the "
            f"{self.ROOM_MODE.upper()}: '{self.ROOM_SEQUENCE}', dolls "
            "geotagged and counted\n"
            f"  5. relock, traverse out to {self.OUTSIDE_DISTANCE:.2f} m past "
            "the window\n"
            f"  6. strafe {back} {self.WINDOW_OFFSET:.2f} m back over the "
            f"marker (id {self.WINDOW_MARKER_ID})\n"
            f"  7. hover {self.FINAL_LOOK_SECONDS:.0f} s looking: centred -> "
            "plain descent; otherwise back to the strafe end, look again, "
            f"slow descent at {self.SLOW_LAND_SPEED:.2f} m/s\n"
            f"  Hard descent at {self.FLIGHT_SECONDS:.0f} s airborne.\n"
            "  q aborts into a descent, k force-disarms.")

    # ------------------------------------------------------------ the dolls

    def doll_count_callback(self, msg):
        if getattr(self, 'doll_count', None) != msg.data:
            self.get_logger().warning(f"DOLL COUNT: {msg.data} (raw /doll_count)")
        self.doll_count = int(msg.data)

    def _doll_words(self):
        n = getattr(self, 'doll_count', None)
        return 'no /doll_count received' if n is None else f"{n} (raw)"

    def _finish_tile_scan(self, outcome):
        self.get_logger().warning(
            f"ROOM DONE. DOLLS COUNTED: {self._doll_words()}.")
        super()._finish_tile_scan(outcome)

    # ------------------------------------------------------------ the way out

    def _begin_room(self):
        # First call is straight after the inbound CLEAR, before the room
        # height is commanded: target_z is still the traverse height.
        if self.inbound_z is None:
            self.inbound_z = self.target_z
        super()._begin_room()

    def _begin_relock(self):
        if self.EXIT_MODE != 'retrace':
            super()._begin_relock()
            return
        self.moving = False
        self.yaw_remaining = 0.0
        self.phase = self.PHASE_OUT
        self.EXIT_DISTANCE = self.OUTSIDE_DISTANCE
        self.MOVE_SPEED = self.APPROACH_SPEED
        leg = self.legs[self.leg_by_name['exit']]
        if self.inbound_z is not None and self.home_z is not None:
            alt = self.home_z - self.inbound_z
            self.get_logger().warning(
                f"EXIT (retrace): back to the inbound traverse height "
                f"{alt:.2f} m, then {leg.distance:.2f} m straight back out "
                "along the takeoff heading. The window is unmarked inside, so "
                "no camera relock.")
            self._begin_alt_change(alt, 'exit', 'inbound traverse height')
        else:
            self._begin_leg_named('exit')

    # ------------------------------------------------------------ the start

    def _handle_hold(self):
        """End of the settle at altitude: strafe onto the window axis."""
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()

        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_leg_named('offset')
            return

        self.get_logger().info(
            f"Holding, {remaining:.1f} s to the strafe...",
            throttle_duration_sec=1.0)
        self.log_flight_state()

    # ------------------------------------------------ back onto the marker

    def _handle_cruise(self):
        leg = self.current_leg()
        if leg is None or leg.name != 'final':
            super()._handle_cruise()
            return
        self._handle_final()

    def _handle_final(self):
        """Hover at P and look; come back to P after a failed centring."""
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        now = time.monotonic()
        lp = self.local_position

        if self.final_point is None:
            if not self.hold_xy or lp is None:
                self._final_land_slowly(
                    "no x/y hold at the end of the strafe, nothing to hold at")
                return
            self.final_point = (self.hold_x, self.hold_y)
            self.final_look_since = now
            self.get_logger().warning(
                f"At the strafe end P ({self.final_point[0]:.2f}, "
                f"{self.final_point[1]:.2f}) NED. Looking for marker id "
                f"{self.WINDOW_MARKER_ID} for {self.FINAL_LOOK_SECONDS:.0f} s.")

        # MARKER_ALIGN hands back here by entering CRUISE when it has lost
        # the marker for marker_lost_seconds.
        if self.final_centring:
            self._final_attempt_failed(
                f"lost marker id {self.WINDOW_MARKER_ID} while centring")
            return

        px, py = self.final_point
        if self.final_returning:
            if not self.hold_xy or lp is None:
                self._final_land_slowly("lost the x/y hold flying back to P")
                return
            self.move_target_x, self.move_target_y = px, py
            self.moving = True
            remaining = math.hypot(px - lp.x, py - lp.y)
            if remaining <= self.MOVE_TOLERANCE:
                self.moving = False
                self.hold_x, self.hold_y = px, py
                self.final_returning = False
                self.final_look_since = now
                self.get_logger().warning(
                    f"Back at P. Hovering and looking for "
                    f"{self.FINAL_LOOK_SECONDS:.0f} s.")
            elif now - self.final_return_since > self.MOVE_TIMEOUT:
                self._final_land_slowly(
                    f"could not get back to P, {remaining:.2f} m short")
            else:
                self.get_logger().info(
                    f"Flying back to P, {remaining:.2f} m to go.",
                    throttle_duration_sec=1.0)
            return

        can_centre = self.final_attempts < self.FINAL_CENTRING_ATTEMPTS
        if can_centre and self.marker_offset_ned() is not None:
            self.get_logger().warning(
                f"Marker id {self.WINDOW_MARKER_ID} in sight. Centring "
                f"(attempt {self.final_attempts + 1}/"
                f"{self.FINAL_CENTRING_ATTEMPTS}).")
            self.final_centring = True
            self.moving = False
            self.marker_lost_since = None
            self.align_in_band_since = None
            self.marker_aligned_error = None
            self._enter_stage(self.MARKER_ALIGN)
            return

        left = self.FINAL_LOOK_SECONDS - (now - self.final_look_since)
        if left <= 0.0:
            self._final_land_slowly(
                f"no centring on marker id {self.WINDOW_MARKER_ID} after "
                f"{self.final_attempts} failed attempt(s)" if self.final_attempts
                else f"marker id {self.WINDOW_MARKER_ID} not seen in "
                     f"{self.FINAL_LOOK_SECONDS:.0f} s")
            return

        self.get_logger().info(
            f"Hovering at P, {left:.1f} s left. {self.marker_summary()}.",
            throttle_duration_sec=1.0)

    def _final_attempt_failed(self, why):
        self.final_attempts += 1
        self.final_centring = False
        self.marker_aligned_error = None
        self.final_returning = True
        self.final_return_since = time.monotonic()
        self.MOVE_SPEED = self.LEG_SPEED
        self.get_logger().warning(
            f"Centring attempt {self.final_attempts}/"
            f"{self.FINAL_CENTRING_ATTEMPTS} failed: {why}. Flying back to P.")
        if self.current_stage != self.CRUISE:
            self._enter_stage(self.CRUISE)

    def _final_land_slowly(self, why):
        self.final_centring = False
        self._complete_leg(self.current_leg(), why, False)

    def _finish_leg(self, outcome, retryable=True):
        """MARKER_HOLD done (centred) or MARKER_ALIGN timed out, on 'final'."""
        leg = self.current_leg()
        if leg is None or leg.name != 'final':
            super()._finish_leg(outcome, retryable)
            return
        if self.marker_aligned_error is not None:
            self.final_centring = False
            self._complete_leg(leg, outcome, True)
            return
        self._final_attempt_failed(outcome)

    def _dispatch_after(self, nxt, on_marker=False):
        if nxt == 'exit_clear':
            # Outside again: the inherited far-side hold, which with phase OUT
            # hands over to the mirrored strafe (MissionFSM._handle_clear).
            self.alt_next = None
            self.get_logger().warning("EXIT: out of the window. Holding, then "
                                      "the mirrored strafe.")
            self._enter_stage(self.CLEAR)
            return
        if nxt != 'land':
            super()._dispatch_after(nxt, on_marker)
            return
        self.alt_next = None
        self.get_logger().warning(f"DOLLS COUNTED: {self._doll_words()}.")

        if on_marker:
            self._begin_landing(
                f"centred on window marker id {self.WINDOW_MARKER_ID}")
            return

        # The fallback: slowly, and holding P rather than drifting off it.
        self.LAND_SPEED = self.SLOW_LAND_SPEED
        point, had_hold = self.final_point, self.hold_xy
        why = self.leg_outcomes[-1] if self.leg_outcomes else 'no marker'
        self._begin_landing(
            f"slow descent at the strafe end ({self.SLOW_LAND_SPEED:.2f} m/s), "
            f"{why}")
        if had_hold and point is not None:
            self.hold_x, self.hold_y = point
            self.hold_xy = True
            # Makes MissionFSM._handle_landing drop the hold if the VIO dies
            # on the way down. The blind-altitude warning is about the pad
            # marker and does not apply here.
            self.precision_descent_active = True
            self._warned_blind = True

    def _phase_label(self):
        leg = self.current_leg()
        if (leg is not None and leg.name == 'exit' and self.current_stage in
                (self.CRUISE, self.MARKER_ALIGN, self.MARKER_HOLD)):
            return 'EXITING'
        if (leg is not None and leg.name == 'final' and self.current_stage in
                (self.CRUISE, self.MARKER_ALIGN, self.MARKER_HOLD)):
            return 'TO_MARKER'
        return super()._phase_label()

    def destroy_node(self):
        self.get_logger().warning(f"DOLLS COUNTED: {self._doll_words()}.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionFSMPart2()
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
