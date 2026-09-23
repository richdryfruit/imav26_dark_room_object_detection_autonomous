"""
DARK ROOM BACKUP: TAKEOFF PAD -> WINDOW MARKER -> THROUGH THE WINDOW -> LAND
INSIDE. No room pattern, no exit.

    (on the takeoff pad, FACING THE COURSE; the PILOT switches Offboard and
     arms, from the transmitter or the imav_bringup dashboard)
    climb to 2.50 m on the pad marker (first metre gently) -> hold
      -> forward up to 9.3 m ON THE CARPET TRACK, watching the down camera
         for the SECOND ArUco -- the window marker. Anything seen in the
         first marker_ignore_distance (3.0 m) is the pad's marker (same id)
         and is ignored.
           seen     -> centre on it (15 cm) -> hold
           not seen -> carry on from the end of the leg
      -> down to 1.75 m
      -> strafe LEFT 0.15 m (slowly) onto the window's axis
      -> find the blue window, line up on its measured centre +
         traverse_centre_offset and fly through it to 0.80 m past the window
         plane
      -> hold inside until the TFmini is proven to be looking at the FLOOR
         again (see below), then a SLOW descent (0.10 m/s) holding that point
         -> disarm, inside the dark room

    ros2 launch drone_testing mission_fsm_darkroom_backup.launch.py  (support)
    ros2 run drone_testing mission_fsm_darkroom_backup --ros-args \
      --params-file .../config/hw_darkroom_backup.yaml               (flight)

ONLY THE PART IT FLIES, AND THAT PART IS mission_fsm_full's
-----------------------------------------------------------
Everything up to the inbound CLEAR is inherited from MissionFSMFull /
MissionFSMPart2 / MissionFSM unchanged, so a fix there is a fix here:
the pad-marker hold on the climb, the carpet-track leg, the slow strafe, the
window search / lidar-corrected align / traverse, the aim height
(traverse_centre_offset, main's value), and the sill crossing. This file only:

  * cuts the plan to 'to_marker' -> 'window_alt' -> 'offset' -> window;
  * ignores markers for the first marker_ignore_distance of the leg, because
    the pad marker has the window marker's id and the leg starts over it;
  * lands, slowly, from the inbound CLEAR, after the floor check below.

THE RANGEFINDER AT THE WINDOWSILL -- THE FIX
--------------------------------------------
Height is the downward TFmini alone (EKF2_HGT_REF = 2). Crossing the wall it
reads the sill for a fraction of a second. EKF2 then either resets its height
to the sill (z_valid blips, the datum jumps), or rejects range as
kinematically inconsistent and only takes it back while |vz| > 0.5 m/s.

  1. CROSSING (inherited): through TRAVERSE, z is flown on VERTICAL VELOCITY
     (hold 0), the z-invalid grace is widened, and z goes back on position
     only after the height has been valid for crossing_settle_s, re-anchored
     where the aircraft actually is.
  2. DROPOUT (inherited): range out of EKF2 inside is a HOLD with a +/-0.3 m
     bob so EKF2 re-accepts it, never a landing; a dropout mid-TRAVERSE
     carries on through.
  3. HERE -- the descent does not start on the estimator's word. After the
     CLEAR hold it waits until, continuously for floor_confirm_s: the
     crossing hold has ended, EKF2 is fusing range, the RAW TFmini
     (/fmu/out/distance_sensor) agrees with EKF2's dist_bottom to
     floor_agree_m, and the raw reading is steady (floor_spread_m). If that
     never happens inside floor_wait_s it lands anyway, slowly.
  4. HERE -- once LANDING starts the vertical-velocity hold is released;
     otherwise a leftover hold pins vz at 0 and the aircraft hovers in
     LANDING until the landing timeout.

Keys:  q -> abort into a controlled descent from wherever we are.
       k -> force-disarm immediately (motors cut, vehicle drops).
"""

import math
import time

import rclpy
from px4_msgs.msg import DistanceSensor
from rclpy.qos import qos_profile_sensor_data

from drone_testing.mission_fsm import Leg
from drone_testing.mission_fsm_full import MissionFSMFull
from drone_testing.offboard_sequence import spin_node


class MissionFSMDarkroomBackup(MissionFSMFull):
    """Pad -> window marker -> through the window -> slow landing inside."""

    SLOW_LAND_SPEED = 0.10      # m/s for the landing inside
    FLIGHT_SECONDS = 360.0      # climb + leg + centring + window + landing,
                                # with margin. A backstop, not a schedule.

    # ---- "the SECOND aruco" -----------------------------------------------
    MARKER_IGNORE_DISTANCE = 3.0    # m of the leg in which markers are
                                    # ignored: at 2.5 m the down camera sees
                                    # +/-2.0 m, so the pad's marker is out of
                                    # frame before this.

    # ---- landing inside: the TFmini must be on the floor --------------------
    FLOOR_CONFIRM_S = 1.0       # s all floor checks must hold continuously
    FLOOR_AGREE_M = 0.15        # m raw TFmini vs EKF2 dist_bottom
    FLOOR_SPREAD_M = 0.05       # m max-min of the raw reading over the window
    FLOOR_WAIT_S = 20.0         # s after the CLEAR hold before landing anyway
    RAW_RANGE_MAX_AGE = 0.3     # s. Older is not a reading.

    def __init__(self):
        super().__init__()
        num = self._declare_number
        self.MARKER_IGNORE_DISTANCE = float(num(
            'marker_ignore_distance', self.MARKER_IGNORE_DISTANCE))
        self.FLOOR_CONFIRM_S = float(num('floor_confirm_s', self.FLOOR_CONFIRM_S))
        self.FLOOR_AGREE_M = float(num('floor_agree_m', self.FLOOR_AGREE_M))
        self.FLOOR_SPREAD_M = float(num('floor_spread_m', self.FLOOR_SPREAD_M))
        self.FLOOR_WAIT_S = float(num('floor_wait_s', self.FLOOR_WAIT_S))

        # The whole plan. The window follows 'offset' through _dispatch_after.
        self.legs = [
            Leg('to_marker', 'forward', self.OUTBOUND_DISTANCE,
                self.WINDOW_MARKER_ID, 'window_alt', retry=False),
            Leg('offset', self.WINDOW_OFFSET_DIRECTION, self.WINDOW_OFFSET,
                None, 'window'),
        ]
        self.leg_by_name = {leg.name: i for i, leg in enumerate(self.legs)}

        # The raw TFmini, straight off the driver: the one height number the
        # estimator has not already touched.
        self.raw_range = []             # [(monotonic, metres)], newest last
        self.create_subscription(
            DistanceSensor, '/uav_2/fmu/out/distance_sensor',
            self._distance_sensor_callback, qos_profile_sensor_data)

        self.floor_ok_since = None
        self.floor_wait_since = None

    def _plan_summary(self):
        # Called from MissionFSM.__init__ before self.legs is replaced, so it
        # reads the parameters rather than the leg list.
        return (
            "DARK ROOM BACKUP, one flight. Directions in the TAKEOFF frame:\n"
            f"  1. climb {self.CRUISE_ALTITUDE:.2f} m, hold "
            f"{self.HOLD_SECONDS:.0f} s\n"
            f"  2. forward up to {self.OUTBOUND_DISTANCE:.1f} m on the carpet "
            "track to the second "
            f"ArUco (window marker id {self.WINDOW_MARKER_ID}; markers ignored "
            f"for the first {getattr(self, 'MARKER_IGNORE_DISTANCE', 3.0):.1f} m); "
            "centre on it if seen\n"
            f"  3. down to {self.WINDOW_ALTITUDE:.2f} m, strafe "
            f"{self.WINDOW_OFFSET_DIRECTION} {self.WINDOW_OFFSET:.2f} m\n"
            f"  4. find the window, traverse at centre "
            f"{self.TRAVERSE_CENTRE_OFFSET:+.2f} m to "
            f"{self.INSIDE_DISTANCE:.2f} m past it (z on vertical velocity "
            "across the sill)\n"
            "  5. hold until the TFmini is on the floor, then descend at "
            f"{self.SLOW_LAND_SPEED:.2f} m/s INSIDE the dark room\n"
            f"  Hard descent at {self.FLIGHT_SECONDS:.0f} s airborne.\n"
            "  q aborts into a descent, k force-disarms.")

    # ------------------------------------------------- the second ArUco

    def marker_offset_ned(self):
        """The pad's marker is not the leg's: nothing counts until the leg
        has put marker_ignore_distance behind it."""
        leg = self.current_leg()
        if (leg is not None and leg.name == 'to_marker'
                and self.current_stage == self.CRUISE):
            lp = self.local_position
            if not self.leg_started or lp is None:
                return None
            travelled = math.hypot(lp.x - self.move_start_x,
                                   lp.y - self.move_start_y)
            if travelled < self.MARKER_IGNORE_DISTANCE:
                return None
        return super().marker_offset_ned()

    # --------------------------------------------- the raw TFmini

    def _distance_sensor_callback(self, msg):
        now = time.monotonic()
        d = float(msg.current_distance)
        valid = (math.isfinite(d) and msg.signal_quality != 0
                 and (msg.min_distance <= 0.0 or d >= msg.min_distance)
                 and (msg.max_distance <= 0.0 or d <= msg.max_distance))
        if valid:
            self.raw_range.append((now, d))
        horizon = now - max(self.FLOOR_CONFIRM_S, self.RAW_RANGE_MAX_AGE)
        self.raw_range = [s for s in self.raw_range if s[0] >= horizon]

    def _floor_check(self):
        """(ok, why): is the TFmini looking at the floor, steadily?"""
        lp = self.local_position
        if self._crossing:
            return False, "crossing hold not ended (height not re-anchored)"
        if lp is None or not lp.z_valid:
            return False, "height estimate invalid"
        if not self.rangefinder_is_healthy():
            return False, "range not fused by EKF2"
        now = time.monotonic()
        if not self.raw_range or now - self.raw_range[-1][0] > self.RAW_RANGE_MAX_AGE:
            return False, "no live /fmu/out/distance_sensor reading"
        raw = self.raw_range[-1][1]
        ekf = float(lp.dist_bottom)
        if abs(raw - ekf) > self.FLOOR_AGREE_M:
            return False, f"raw TFmini {raw:.2f} m vs EKF2 {ekf:.2f} m"
        vals = [d for _, d in self.raw_range]
        spread = max(vals) - min(vals)
        if spread > self.FLOOR_SPREAD_M:
            return False, f"raw TFmini unsteady ({spread * 100:.0f} cm spread)"
        return True, f"raw {raw:.2f} m = EKF2 {ekf:.2f} m, {spread * 100:.0f} cm spread"

    # --------------------------------------------- inside: land slowly

    def _handle_clear(self):
        if self.phase != self.PHASE_IN:
            super()._handle_clear()
            return
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        remaining = self.CLEAR_SECONDS - self._in_stage_for()
        if remaining > 0.0:
            self.get_logger().info(
                f"INSIDE: holding, {remaining:.1f} s before the floor check.",
                throttle_duration_sec=1.0)
            return

        now = time.monotonic()
        if self.floor_wait_since is None:
            self.floor_wait_since = now
        if self._rng_gave_up:
            self._land_inside("rangefinder never came back into EKF2; landing "
                              "inside anyway, on the velocity descent")
            return
        ok, why = self._floor_check()
        if ok:
            if self.floor_ok_since is None:
                self.floor_ok_since = now
            elif now - self.floor_ok_since >= self.FLOOR_CONFIRM_S:
                self._land_inside(f"TFmini on the floor ({why})")
            return
        self.floor_ok_since = None
        if now - self.floor_wait_since > self.FLOOR_WAIT_S:
            self._land_inside(
                f"floor not confirmed in {self.FLOOR_WAIT_S:.0f} s ({why}); "
                "landing inside anyway")
            return
        self.get_logger().info(f"INSIDE: waiting for the floor: {why}.",
                               throttle_duration_sec=1.0)

    def _land_inside(self, why):
        self._disarm_dolls("landing inside")
        self.LAND_SPEED = self.SLOW_LAND_SPEED
        hold_x, hold_y, had_hold = self.hold_x, self.hold_y, self.hold_xy
        self._begin_landing(
            f"inside the dark room, slow descent at "
            f"{self.SLOW_LAND_SPEED:.2f} m/s: {why}")
        if had_hold:
            # Keep the point just inside the window rather than drifting back
            # towards the wall. MissionFSM._handle_landing drops it for
            # zero-velocity if the flow dies on the way down.
            self.hold_x, self.hold_y = hold_x, hold_y
            self.hold_xy = True
            self.precision_descent_active = True
            self._warned_blind = True

    def _begin_landing(self, reason):
        # Whatever the sill left behind, the landing owns z from here.
        self._crossing = False
        self._crossing_valid_since = None
        self._rng_hold = False
        self._rng_bob = None
        super()._begin_landing(reason)

    def _publish_sp(self, msg):
        if self.current_stage in (self.LANDING, self.DISARMING,
                                  self.KILLING, self.DONE):
            self._publish_sp_raw(msg)
            return
        super()._publish_sp(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionFSMDarkroomBackup()
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
