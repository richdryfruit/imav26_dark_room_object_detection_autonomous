"""
MISSION, PART 1 OF 2: THE STRAIGHT LEG, TAKEOFF PAD -> WINDOW MARKER.

    arm -> climb to 2.10 m -> hold
      -> forward, slowly, up to 9.3 m, watching the down camera for the
         WINDOW marker
           seen     -> centre on it -> hold -> slow descent onto it, holding
                       the centred point -> disarm
           not seen -> stop at 9.3 m -> the same slow descent there -> disarm

    ros2 run drone_testing mission_fsm_part1
    ros2 launch drone_testing mission_fsm_part1.launch.py      (support stack)

Part 2 (mission_fsm_part2.py) takes off from where this one lands.

EVERYTHING IS mission_fsm's
---------------------------
This is MissionFSM with a one-leg flight plan. The leg is the same
marker-terminated CRUISE -> MARKER_ALIGN -> MARKER_HOLD primitive the full
mission flies to the window marker, on the same VIO lateral estimate, with the
same marker filter and the same tilt-compensated marker maths. Only three
things are different:

  * the plan is one leg, 'to_marker', whose successor is 'land';
  * the leg ends in a SLOW descent (slow_land_speed, 0.10 m/s) that keeps
    the x/y point it was holding -- the centred point over the marker, or the
    9.3 m point without one. It is not a precision landing: the marker is
    not tracked on the way down, the latched VIO point is. If the VIO stops
    being healthy mid-descent the hold is dropped for zero-velocity;
  * the window marker is id 0;
  * cruise height is 2.10 m, where the down camera's basket is about
    +/-1.70 m (h * tan 39 deg).

PX4 only declares touchdown once the descent setpoint is at least
0.9 * MPC_LAND_SPEED. Below that, the node's own stalled-descent test
confirms it, a few seconds later. Set MPC_LAND_SPEED <= slow_land_speed / 0.9
to avoid that.

The leg is deliberately NOT named 'outbound'. MissionFSM keys its altitude
schedule on that name and would drop to window_altitude_m before handing on,
losing the "was it on the marker?" answer through ALT_CHANGE.

Keys:  q -> abort into a controlled descent from wherever we are.
       k -> force-disarm immediately (motors cut, vehicle drops).
"""

import rclpy

from drone_testing.mission_fsm import Leg, MissionFSM
from drone_testing.offboard_sequence import spin_node


class MissionFSMPart1(MissionFSM):
    """Fly forward to the window marker and land on it."""

    CRUISE_ALTITUDE = 2.10      # m. Takeoff and the whole leg.
    OUTBOUND_DISTANCE = 9.3     # m FORWARD. A LIMIT: the leg ends on the
                                # marker if it appears, here if it does not.
    WINDOW_MARKER_ID = 0        # the ArUco in front of the window
    SLOW_LAND_SPEED = 0.10      # m/s for the final descent
    FLIGHT_SECONDS = 240.0      # climb + 31 s of leg + centring + descent,
                                # with margin. A backstop, not a schedule.

    def __init__(self):
        super().__init__()

        self.SLOW_LAND_SPEED = float(self._declare_number(
            'slow_land_speed', self.SLOW_LAND_SPEED))

        # retry=False: marker_max_retries is 0 by default anyway, and a missed
        # marker here has an answer that needs no climbing -- land at 9.3 m.
        self.legs = [
            Leg('to_marker', 'forward', self.OUTBOUND_DISTANCE,
                self.WINDOW_MARKER_ID, 'land', retry=False),
        ]
        self.leg_by_name = {leg.name: i for i, leg in enumerate(self.legs)}

    def _plan_summary(self):
        # Called from MissionFSM.__init__ before self.legs is replaced, so it
        # reads the parameters rather than the leg list.
        return (
            "MISSION FSM PART 1, one flight. Directions in the TAKEOFF frame:\n"
            f"  1. climb {self.CRUISE_ALTITUDE:.2f} m, hold "
            f"{self.HOLD_SECONDS:.0f} s\n"
            f"  2. forward up to {self.OUTBOUND_DISTANCE:.1f} m at "
            f"{self.LEG_SPEED:.2f} m/s, watching for window marker id "
            f"{self.WINDOW_MARKER_ID}\n"
            f"  3a. marker seen: centre to "
            f"{self.MARKER_TOLERANCE * 100:.0f} cm, hold "
            f"{self.MARKER_HOLD_SECONDS:.0f} s, descend at "
            f"{self.SLOW_LAND_SPEED:.2f} m/s onto it holding that point\n"
            f"  3b. no marker: stop at {self.OUTBOUND_DISTANCE:.1f} m, the same "
            "slow descent there\n"
            f"  Hard descent at {self.FLIGHT_SECONDS:.0f} s airborne.\n"
            "  q aborts into a descent, k force-disarms.")

    def _handle_hold(self):
        """End of the settle at altitude: fly the one leg.

        Replaces MissionFSM's version outright. That one falls through to the
        window sweep once 'outbound' is done, and this flight has no window.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()

        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_leg_named('to_marker')
            return

        self.get_logger().info(
            f"Holding, {remaining:.1f} s to the leg...",
            throttle_duration_sec=1.0)
        self.log_flight_state()

    def _dispatch_after(self, nxt, on_marker=False):
        if nxt != 'land':
            super()._dispatch_after(nxt, on_marker)
            return
        self.alt_next = None
        # Hold, drop slowly, land: keep the point held over the marker (or at
        # the end of the leg) and walk the height down at slow_land_speed.
        self.LAND_SPEED = self.SLOW_LAND_SPEED
        hold_x, hold_y, had_hold = self.hold_x, self.hold_y, self.hold_xy
        self._begin_landing(
            f"centred on window marker id {self.WINDOW_MARKER_ID}, slow descent"
            if on_marker else
            f"no window marker within {self.OUTBOUND_DISTANCE:.1f} m, slow "
            "descent at the end of the leg")
        if had_hold:
            self.hold_x, self.hold_y = hold_x, hold_y
            self.hold_xy = True
            # Makes MissionFSM._handle_landing drop the hold if the VIO dies
            # on the way down. Its blind-altitude warning is about the pad
            # marker and does not apply here.
            self.precision_descent_active = True
            self._warned_blind = True

    def _phase_label(self):
        if (self.current_leg() is not None and self.current_stage in
                (self.CRUISE, self.MARKER_ALIGN, self.MARKER_HOLD)):
            return 'TO_MARKER'
        return super()._phase_label()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionFSMPart1()
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
