"""
MISSION, PART 2 OF 2: WINDOW MARKER -> DARK ROOM -> BACK ONTO THE MARKER.

    (placed on the window marker, FACING THE WINDOW)
    arm -> climb to 1.75 m -> hold
      -> strafe LEFT 0.15 m onto the window's axis
      -> find the blue window with the WHOLE aperture in frame. If it is not
         fully in view: up to 2 slow 0.30 m backoffs (shared between the
         search and the recentre). The traverse is rebuilt from the measured
         window pose, so the backoff only lengthens the approach.
      -> line up 5 cm below the window centre (lidar plane inside the opening) and traverse in to 0.80 m past
         the window plane. That height ABOVE THE FLOOR (TFmini) is held for
         the rest of the flight. The doll detector is on from the first lock.
      -> dark room, every move closed on the 2D lidar's wall fix:
             forward 0.8, right 0.8, backward 0.8, left 0.8
         then back to the relock station and the half turn IN PLACE.
         Dolls geotagged and counted (/doll_count, printed in the terminal).
      -> exit_mode 'detect': re-find the (unmarked) window from inside with
         the lidar gap, the depth hole and the brightness -- 2 of 3 must
         agree -- align on its axis at the inbound height (lidar lateral,
         TFmini height), fly out steered on the lidar to the wall plane and
         straight on to 1.67 m past it: the marker's line
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

import numpy as np
import rclpy
from px4_msgs.msg import VehicleStatus
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Int32

from drone_testing.mission_fsm import Leg, MissionFSM
from drone_testing.offboard_sequence import spin_node
from drone_testing.room_lidar_scan import wrap_pi
from drone_testing.window_exit import (bright_region, depth_hole, lidar_gap,
                                       vote, wall_gap_body)


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
    ROOM_SEQUENCE = 'forward 0.5, right 0.5, backward 0.5, left 0.5, yaw 180'

    # ---- landing back on the marker -------------------------------------
    FINAL_LOOK_SECONDS = 10.0   # s hovering at P looking for the marker
    FINAL_CENTRING_ATTEMPTS = 2 # centring tries before landing at P anyway
    SLOW_LAND_SPEED = 0.10      # m/s for the fallback descent at P

    # ---- the way out ------------------------------------------------------
    # 'detect' (default): the window is UNMARKED inside, so the colour
    # detector cannot see it. It is re-found from three other cues (see
    # window_exit.py), and the way out is steered on the lidar.
    # 'retrace': no detection -- back to the inbound height and straight out
    # along the takeoff heading (the fallback detect uses without a lidar
    # frame). 'relock': the inherited colour relock, for a window marked on
    # both faces.
    EXIT_MODE = 'detect'

    LIDAR_CLOSED_LOOP = True    # room goals closed on the lidar fix

    # ---- one height, set by the window ------------------------------------
    TRAVERSE_CENTRE_OFFSET = 0.07   # m ABOVE the measured window centre, as
                                    # flown. That height above the floor
                                    # (TFmini) is then held for the room and
                                    # the way out.
    # THE LIDAR PLANE IS THE CONSTRAINT. The 2D lidar sits 0.135 m above the
    # body, so its plane is offset + 0.135 above the window centre and has to
    # stay inside the 0.60 m opening (half-height 0.30) for the window-gap
    # fix, in and out:
    #     +0.10 -> +0.235: 6.5 cm to the top edge; 2 deg of pitch at 1.67 m
    #     +0.07 -> +0.205: 9.5 cm, about 3 deg at 1.67 m  <-- flown
    #     -0.05 -> +0.085: 21.5 cm, about 7 deg (most robust, gear 13 cm
    #                      over the sill) -- traverse_centre_offset:=-0.05
    # Above +0.07 the gap cue is the first thing lost on the approach.
                                    # = below). That height ABOVE THE FLOOR
                                    # (TFmini) is held for the room and the
                                    # way out.
    # WHY BELOW, NOT ABOVE. The 2D lidar sits 0.135 m above the body and its
    # plane has to pass THROUGH the 0.60 m opening (half-height 0.30) for the
    # window-gap fix, inbound and outbound. Plane = offset + 0.135:
    #     +0.10 -> +0.235: 6.5 cm to the top edge; 2 deg of pitch at 1.67 m
    #              lifts it 6 cm -> the gap vanishes on the approach.
    #     -0.05 -> +0.085: 21.5 cm to the top (~7 deg of pitch at 1.67 m),
    #              and the body (0.12 below / 0.14 above the aim) keeps 13 cm
    #              over the sill and 21 cm under the top edge.
    # Best overall is where the plane and the body margins balance; with a
    # different lidar height, re-solve: plane margin = 0.30 - (offset + h).

    # ---- finding the window before going in -------------------------------
    BACKOFF_BUDGET = 2          # 30 cm backoffs, window search + recentre
    BACKOFF_SPEED = 0.15        # m/s: small, slow

    # ---- the strafe onto the window axis ----------------------------------
    # 0.15 m at leg_speed (0.6 m/s) is over in a quarter of a second and the
    # airframe is still accelerating when it should be stopping: it overshoots
    # the axis it just measured. Flown slowly it stops where it is told.
    STRAFE_SPEED = 0.25         # m/s for 'offset' and 'offset_back'

    # ---- in the room: every move closed on the lidar ----------------------
    ROOM_YAW_RATE = 0.12        # rad/s (~7 deg/s): the half turn is
                                # slow on purpose, it is what drifts

    # ---- losing the lidar, or the yaw, INSIDE the room --------------------
    # Two failures make every in-room waypoint meaningless, and they want
    # different answers:
    #
    #   LIDAR LOST   the fix goes stale or degenerate. The room is still
    #                where it was, so HOLD STILL and wait: most dropouts are
    #                a second or two of a reference wall out of view. Only if
    #                it does not come back is the flight over, and then the
    #                way out is still flyable because the heading is good.
    #   YAW UNTRUSTED  EKF2's heading has been reset repeatedly, or the lidar
    #                and EKF2 disagree about it. Now the way out is NOT
    #                flyable: 'forward' points somewhere else, and flying it
    #                means flying into a wall. Land where it is, slowly.
    #
    # Landing inside the dark room is a last resort and says so in the log.
    ROOM_LIDAR_RECOVER = 15.0   # s of holding still for the lidar to return
    ROOM_YAW_RESET_MAX = 20.0   # deg of EKF2 heading reset, summed, inside
                                # the room before the yaw is not trusted
    ROOM_LOST_ACTION = 'exit'   # exit | land, when the lidar does not return

    # ---- the way out: re-find the unmarked window from inside -------------
    EXIT_NEED = 2               # cues that must agree (of lidar/depth/bright)
    EXIT_CONFIRM = 5            # consecutive agreeing evaluations
    EXIT_FIND_TIMEOUT = 25.0    # s before falling back (see _handle_exit_find)
    EXIT_ALIGN_TOL = 0.05       # m lateral, lidar-measured
    EXIT_HEIGHT_TOL = 0.07      # m, TFmini against the inbound height
    EXIT_YAW_TOL = math.radians(3.0)
    EXIT_ALIGN_HOLD = 1.5       # s inside all three tolerances
    EXIT_ALIGN_TIMEOUT = 30.0
    EXIT_SPEED = 0.30           # m/s out through the window
    EXIT_FREEZE = 0.50          # m before the wall plane: stop steering on
                                # the lidar (it loses the room as it passes)
    EXIT_SIDE_MARGIN = 0.05     # m of air each side the live gap must leave
    EXIT_CAM_X = 0.215          # m, front camera ahead of the lidar axis
    EXIT_STALL_S = 10.0         # s without 5 cm of progress = stalled

    # ---- the window on the lidar, from OUTSIDE too ------------------------
    # The lidar plane crosses the window, so the wall ahead has a gap in it.
    # Its lateral position and the wall's angle are measured to ~1-2 cm / <1
    # deg, far tighter than the camera's corner-depth pose (which wandered
    # 20 cm and 3 deg during one SITL alignment and committed a traverse
    # 15 cm right of centre). The camera still gives the height.
    GAP_MIN = 0.80              # x the camera-measured width. The red 40 cm
    GAP_MAX = 1.30              # window next to the blue 60 cm one is 0.67x.
    LIDAR_WINDOW_AGREE = 0.25   # m camera vs lidar centre to accept the fix
    LIDAR_REQUIRED_S = 15.0     # s of ALIGN without any lidar gap before the
                                # commit falls back to camera-only stability
    CAMERA_STABLE = 0.04        # m spread of the camera centre over the
                                # settle, for that fallback
    ALIGN_CROSS_TOLERANCE = 0.04
    TRAVERSE_STEER_GAIN = 0.3   # per scan, lateral line correction inbound
    TRAVERSE_STEER_MAX = 0.35   # m total the line may be moved by the lidar
                                # (0.20 saturated in SITL: the camera commit
                                # was 20 cm out and the cap hid the rest)

    # ---- the TFmini dropping out of EKF2 (the sill) -----------------------
    # EKF2 stops fusing range when it is kinematically inconsistent (the sill
    # step) and only re-accepts it while |vz| > 0.5 m/s. Inside the room this
    # must never become a landing: hold, bob, carry on; out if it persists.
    RNG_BOB_SPEED = 0.6         # m/s, just over EKF2's 0.5 m/s
    RNG_BOB_TIME = 0.5          # s each way (down, then up): +/-0.3 m
    RNG_RECOVER_S = 20.0        # s before giving up on it and heading out
                                # (inside) or landing (outside)

    EXIT_FIND = 'EXIT_FIND'         # holding, voting the three cues
    EXIT_ALIGN = 'EXIT_ALIGN'       # onto the gap axis at the inbound height
    EXIT_TRAVERSE = 'EXIT_TRAVERSE' # out, steered on the lidar to the plane
    EXIT_STAGES = (EXIT_FIND, EXIT_ALIGN, EXIT_TRAVERSE)

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
        if self.EXIT_MODE not in ('detect', 'retrace', 'relock'):
            raise SystemExit(
                "exit_mode must be 'detect', 'retrace' or 'relock'; got "
                f"'{self.EXIT_MODE}'.")
        num = self._declare_number
        self.BACKOFF_BUDGET = int(num('backoff_budget', self.BACKOFF_BUDGET))
        self.BACKOFF_SPEED = float(num('backoff_speed', self.BACKOFF_SPEED))
        self.ROOM_YAW_RATE = float(num('room_yaw_rate', self.ROOM_YAW_RATE))
        self.STRAFE_SPEED = float(num('strafe_speed', self.STRAFE_SPEED))
        self.EXIT_NEED = int(num('exit_need', self.EXIT_NEED))
        self.EXIT_CONFIRM = int(num('exit_confirm', self.EXIT_CONFIRM))
        self.EXIT_FIND_TIMEOUT = float(num('exit_find_timeout', self.EXIT_FIND_TIMEOUT))
        self.EXIT_ALIGN_TOL = float(num('exit_align_tol', self.EXIT_ALIGN_TOL))
        self.EXIT_HEIGHT_TOL = float(num('exit_height_tol', self.EXIT_HEIGHT_TOL))
        self.EXIT_YAW_TOL = math.radians(float(num(
            'exit_yaw_tol_deg', math.degrees(self.EXIT_YAW_TOL))))
        self.EXIT_ALIGN_HOLD = float(num('exit_align_hold', self.EXIT_ALIGN_HOLD))
        self.EXIT_ALIGN_TIMEOUT = float(num('exit_align_timeout', self.EXIT_ALIGN_TIMEOUT))
        self.EXIT_SPEED = float(num('exit_speed', self.EXIT_SPEED))
        self.EXIT_FREEZE = float(num('exit_freeze', self.EXIT_FREEZE))
        self.EXIT_SIDE_MARGIN = float(num('exit_side_margin', self.EXIT_SIDE_MARGIN))
        self.EXIT_CAM_X = float(num('exit_cam_x', self.EXIT_CAM_X))
        self.ROOM_LIDAR_RECOVER = float(num('room_lidar_recover',
                                            self.ROOM_LIDAR_RECOVER))
        self.ROOM_YAW_RESET_MAX = float(num('room_yaw_reset_max_deg',
                                            self.ROOM_YAW_RESET_MAX))
        self.ROOM_LOST_ACTION = str(self.declare_parameter(
            'room_lost_action', self.ROOM_LOST_ACTION).value).strip().lower()
        if self.ROOM_LOST_ACTION not in ('exit', 'land'):
            raise SystemExit("room_lost_action must be 'exit' or 'land'; got "
                             f"'{self.ROOM_LOST_ACTION}'.")
        self._room_lost_since = None
        self._yaw_reset_total = 0.0
        self._yaw_suspect = False
        self.EXIT_STALL_S = float(num('exit_stall_s', self.EXIT_STALL_S))
        # NaN = size the gap checks off the window measured on the way in.
        self.EXIT_WINDOW_WIDTH = float(num('exit_window_width', float('nan')))
        self.GAP_MIN = float(num('gap_min', self.GAP_MIN))
        self.GAP_MAX = float(num('gap_max', self.GAP_MAX))
        self.LIDAR_WINDOW_AGREE = float(num('lidar_window_agree', self.LIDAR_WINDOW_AGREE))
        self.LIDAR_REQUIRED_S = float(num('lidar_required_s', self.LIDAR_REQUIRED_S))
        self.CAMERA_STABLE = float(num('camera_stable', self.CAMERA_STABLE))
        self.TRAVERSE_STEER_GAIN = float(num('traverse_steer_gain', self.TRAVERSE_STEER_GAIN))
        self.TRAVERSE_STEER_MAX = float(num('traverse_steer_max', self.TRAVERSE_STEER_MAX))
        self.RNG_BOB_SPEED = float(num('rng_bob_speed', self.RNG_BOB_SPEED))
        self.RNG_BOB_TIME = float(num('rng_bob_time', self.RNG_BOB_TIME))
        self.RNG_RECOVER_S = float(num('rng_recover_s', self.RNG_RECOVER_S))
        self.lidar_window = None        # newest lidar gap seen from outside
        self.lidar_window_time = None
        self.camera_centres = []        # (t, centre) during ALIGN
        self.traverse_shift = 0.0
        self._rng_bad_since = None
        self._rng_hold = False
        self._rng_bob = None            # (start time) of the current bob
        self._rng_gave_up = False
        self.inbound_window = None      # (width, height) measured outside
        self._return_turning = False
        self._init_exit()
        self.inbound_z = None           # target_z of the inbound traverse
        self.inbound_hagl = None        # TFmini height above floor, inbound

        # ---- crossing the wall on the TFmini alone ----------------------
        # The only height source is the downward rangefinder. For the
        # instant it reads the window SILL instead of the floor, EKF2 resets
        # its height, and a position setpoint on z then chases a height that
        # is wrong by metres (seen in SITL: a stall in the aperture, then a
        # climb to "6 m" in a 5.5 m room). So while crossing -- the inbound
        # TRAVERSE and the 'exit' leg -- z is flown on VELOCITY (hold 0), the
        # height-invalid grace is widened, and position control on z resumes
        # only once the height has been valid again for crossing_settle_s,
        # re-anchored where the aircraft actually is.
        self.CROSSING_VZ_HOLD = bool(self.declare_parameter(
            'crossing_vz_hold', True).value)
        self.CROSSING_GRACE = float(self._declare_number(
            'crossing_height_grace', 5.0))
        self.CROSSING_SETTLE = float(self._declare_number(
            'crossing_settle_s', 1.0))
        self._crossing = False
        self._crossing_valid_since = None
        self._publish_sp_raw = self.trajectory_setpoint_pub.publish
        self.trajectory_setpoint_pub.publish = self._publish_sp

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
            f"  3. find the window (<= {self.BACKOFF_BUDGET} slow backoffs), "
            f"traverse at centre {self.TRAVERSE_CENTRE_OFFSET:+.2f} m to "
            f"{self.INSIDE_DISTANCE:.2f} m past it; doll detection on at the "
            "first lock\n"
            f"  4. room at the traverse height on the {self.ROOM_MODE.upper()}"
            f" (closed loop): '{self.ROOM_SEQUENCE}', half turn in place, "
            "dolls geotagged and counted\n"
            f"  5. exit '{self.EXIT_MODE}': window re-found from inside "
            f"({self.EXIT_NEED} of lidar gap / depth hole / brightness), out "
            f"to {self.OUTSIDE_DISTANCE:.2f} m past it at the same height\n"
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

    # ------------------------------------------------ crossing the wall

    def _crossing_now(self):
        """True while the rangefinder may be looking at the sill."""
        if not self.CROSSING_VZ_HOLD:
            return False
        if self.current_stage == self.TRAVERSE:
            return True
        if self.current_stage == self.EXIT_TRAVERSE:
            return True
        leg = self.current_leg()
        return (leg is not None and leg.name == 'exit'
                and self.current_stage == self.CRUISE)

    def _update_crossing(self):
        lp = self.local_position
        if self.current_stage in (self.LANDING, self.DISARMING, self.KILLING,
                                  self.DONE):
            # A landing owns z (its own descent logic); never hold it at 0.
            self._crossing = False
            self._crossing_valid_since = None
            return
        if self._crossing_now():
            if not self._crossing:
                # The inbound height ABOVE THE FLOOR, measured by the TFmini
                # before it reaches the sill. The exit flies back to this --
                # not to an EKF2 altitude, whose datum every crossing resets.
                if (self.inbound_hagl is None and self.phase != self.PHASE_OUT
                        and lp is not None and lp.dist_bottom_valid):
                    self.inbound_hagl = float(lp.dist_bottom)
                self._crossing = True
                self.get_logger().warning(
                    "CROSSING the wall: altitude on vertical velocity (hold 0) "
                    f"and a {self.CROSSING_GRACE:.1f} s height grace until the "
                    "rangefinder sees the floor again.")
            self._crossing_valid_since = None
            return
        if not self._crossing:
            return
        # Past the wall. Stay on velocity until the height has been valid for
        # crossing_settle_s, then re-anchor z where the aircraft actually is.
        if lp is None or not lp.z_valid:
            self._crossing_valid_since = None
            return
        now = time.monotonic()
        if self._crossing_valid_since is None:
            self._crossing_valid_since = now
            return
        if now - self._crossing_valid_since < self.CROSSING_SETTLE:
            return
        self._crossing = False
        self._crossing_valid_since = None
        self.setpoint_z = float(lp.z)
        self.target_z = float(lp.z)
        self.get_logger().warning(
            f"CROSSED: height valid again ({self.relative_altitude() or 0.0:.2f} m); "
            "altitude back on position, held where it is.")

    def _publish_sp(self, msg):
        if self._crossing or self._rng_hold:
            msg.position[2] = float('nan')
            msg.velocity[2] = self._bob_vz()
        self._publish_sp_raw(msg)

    def timer_callback(self):
        self._update_crossing()
        if self.current_stage not in self.EXIT_STAGES:
            super().timer_callback()
            return
        # The same preamble as MissionFSM's tile stages.
        self._publish_mission_phase()
        if self._check_flight_clock():
            return
        if self.stream_setpoints:
            # BOTH, as every other stage does: a setpoint without the
            # OffboardControlMode heartbeat is offboard_control_signal_lost
            # a few seconds later, and PX4 takes the aircraft back -- which
            # is exactly what happened in the SITL exit.
            if hasattr(self, 'publish_offboard_control_mode'):
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
            self.EXIT_FIND: self._handle_exit_find,
            self.EXIT_ALIGN: self._handle_exit_align,
            self.EXIT_TRAVERSE: self._handle_exit_traverse,
        }[self.current_stage]()

    def _clock_stages(self):
        return super()._clock_stages() + self.EXIT_STAGES

    # ------------------------------------------ losing the room

    def _on_heading_reset(self, delta):
        """EKF2 corrected the heading. Inside the room, count it.

        One small reset is normal. Several, or a big one, means the yaw the
        whole room frame is built on has been moving, and the way out cannot
        be flown on it.
        """
        super()._on_heading_reset(delta)
        if not self._inside_room():
            return
        self._yaw_reset_total += abs(math.degrees(delta))
        if (not self._yaw_suspect
                and self._yaw_reset_total > self.ROOM_YAW_RESET_MAX):
            self._yaw_suspect = True
            self.get_logger().error(
                f"YAW NOT TRUSTED inside the room: EKF2 has reset the heading "
                f"by {self._yaw_reset_total:.0f} deg in total (limit "
                f"{self.ROOM_YAW_RESET_MAX:.0f}). The way out is flown on that "
                "heading, so it is no longer flyable.")

    def _room_land(self, why):
        """Last resort: land where it is, inside the room, slowly."""
        self.LAND_SPEED = self.SLOW_LAND_SPEED
        lp = self.local_position
        if lp is not None and self.hold_xy:
            self.hold_x, self.hold_y = float(lp.x), float(lp.y)
        self.moving = False
        self.yaw_remaining = 0.0
        self.get_logger().error(
            f"LANDING INSIDE THE DARK ROOM -- last resort: {why}. Descending "
            f"at {self.SLOW_LAND_SPEED:.2f} m/s where it is, rather than "
            "flying a heading that cannot be trusted.")
        self._begin_landing(f"inside the room: {why}")

    def _tile_health_ok(self):
        """Hold still and wait for the lidar; escalate only if it stays gone."""
        if self.lidar_fix_is_fresh() and self.transform_is_healthy():
            if self._room_lost_since is not None:
                self.get_logger().warning(
                    f"Lidar back after "
                    f"{time.monotonic() - self._room_lost_since:.1f} s. "
                    "Carrying on with the room.")
                self._room_lost_since = None
            return True

        now = time.monotonic()
        if self._room_lost_since is None:
            self._room_lost_since = now
            lp = self.local_position
            self.moving = False
            if lp is not None and self.hold_xy:
                self.hold_x, self.hold_y = float(lp.x), float(lp.y)
            self.get_logger().error(
                "LIDAR FIX LOST inside the room "
                f"({self.lidar_status or 'no status'}). Holding still for up "
                f"to {self.ROOM_LIDAR_RECOVER:.0f} s -- the room has not "
                "moved, so waiting beats flying on a stale frame.")
            return False

        waited = now - self._room_lost_since
        if waited < self.ROOM_LIDAR_RECOVER:
            self.get_logger().warning(
                f"Holding for the lidar, {self.ROOM_LIDAR_RECOVER - waited:.1f} "
                "s left.", throttle_duration_sec=1.0)
            return False

        why = (f"no lidar fix for {waited:.0f} s "
               f"({self.lidar_status or 'no status'})")
        if self._yaw_suspect or self.ROOM_LOST_ACTION == 'land':
            self._room_land(why + (" and the yaw is not trusted"
                                   if self._yaw_suspect else ""))
        else:
            self.get_logger().error(
                f"{why}. The heading is still good, so flying OUT beats "
                "landing in here.")
            self._tile_scan_give_up(why)
        return False

    # ------------------------------------------ the TFmini out of EKF2

    def _inside_room(self):
        if self.current_stage in ((self.TRAVERSE,) + self.TILE_STAGES
                                  + self.EXIT_STAGES):
            return True
        if self.current_stage in (self.LANDING, self.DISARMING, self.KILLING,
                                  self.DONE):
            return False
        return self.phase == self.PHASE_IN or (
            self.phase == self.PHASE_OUT and self.current_leg() is None
            and self.current_stage != self.CLEAR)

    def _bob_vz(self):
        """Down then up at just over 0.5 m/s: what EKF2 needs to re-accept
        the rangefinder. Zero outside a bob."""
        if self._rng_bob is None:
            return 0.0
        t = time.monotonic() - self._rng_bob
        if t < self.RNG_BOB_TIME:
            return self.RNG_BOB_SPEED
        if t < 2.0 * self.RNG_BOB_TIME:
            return -self.RNG_BOB_SPEED
        return 0.0

    def _rangefinder_dropout(self):
        """True while the flight should HOLD instead of doing its stage."""
        now = time.monotonic()
        if self._rng_bad_since is None:
            self._rng_bad_since = now
            self._rng_hold = True
            self._rng_bob = None
            self.get_logger().error(
                "RANGEFINDER OUT OF EKF2 (kinematic consistency, e.g. the sill). "
                + ("Inside the room: NOT landing. " if self._inside_room() else "")
                + "Holding (altitude on vertical velocity) and bobbing "
                f"+/-{self.RNG_BOB_SPEED * self.RNG_BOB_TIME:.2f} m at "
                f"{self.RNG_BOB_SPEED:.1f} m/s so EKF2 takes it back.")
        waited = now - self._rng_bad_since
        if self._rng_bob is None or now - self._rng_bob > 2.0 * self.RNG_BOB_TIME + 1.5:
            if waited > 1.0:
                self._rng_bob = now
        if waited > self.RNG_RECOVER_S and not self._rng_gave_up:
            self._rng_gave_up = True
            if self._inside_room():
                self.get_logger().error(
                    f"Rangefinder still out after {self.RNG_RECOVER_S:.0f} s. "
                    "Inside: skipping to the way out (hover, half turn, exit), "
                    "altitude held on vertical velocity.")
                if self.current_stage in self.TILE_STAGES[:3]:
                    self._next_tile_to_end()
                return False
            self._begin_landing(
                f"rangefinder out of EKF2 for {self.RNG_RECOVER_S:.0f} s, outside")
            return True
        if self._rng_gave_up and self._inside_room():
            return False
        self.get_logger().warning(
            f"Rangefinder out of EKF2 {waited:.1f} s: holding.",
            throttle_duration_sec=1.0)
        return True

    def _next_tile_to_end(self):
        self.tile_index = len(self.tile_centres_arena)
        self.tile_arrived_since = None
        self._enter_stage(self.TILE_RETURN)

    def _rng_recovered(self):
        if self._rng_bad_since is None:
            return
        lp = self.local_position
        self.get_logger().warning(
            f"RANGEFINDER BACK in EKF2 after "
            f"{time.monotonic() - self._rng_bad_since:.1f} s. Carrying on.")
        self._rng_bad_since = None
        self._rng_hold = False
        self._rng_bob = None
        self._rng_gave_up = False
        if lp is not None and lp.z_valid and not self._crossing:
            if self.current_stage in self.EXIT_STAGES and self.exit_z is not None \
                    and self.inbound_hagl is not None and lp.dist_bottom_valid:
                self.exit_z = float(lp.z) - (self.inbound_hagl - float(lp.dist_bottom))
                self.target_z = self.exit_z
            else:
                self.setpoint_z = float(lp.z)
                self.target_z = float(lp.z)

    def _flyable_base(self):
        if not self._crossing:
            return super()._still_flyable()
        normal = self.HEIGHT_INVALID_GRACE
        self.HEIGHT_INVALID_GRACE = max(normal, self.CROSSING_GRACE)
        try:
            return super()._still_flyable()
        finally:
            self.HEIGHT_INVALID_GRACE = normal

    def _still_flyable(self):
        """The base checks, except that a rangefinder dropout (or a height
        estimate gone invalid) is a HOLD, never a landing inside the room."""
        if (self.arming_state != VehicleStatus.ARMING_STATE_ARMED
                or self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                or self.current_stage in (self.LANDING, self.DISARMING,
                                          self.KILLING, self.DONE)
                or self.local_position is None):
            return self._flyable_base()
        lp = self.local_position
        if lp.z_valid and self.rangefinder_is_healthy():
            self._rng_recovered()
            return self._flyable_base()
        if self.current_stage in (self.TRAVERSE, self.EXIT_TRAVERSE):
            # Never stop in the aperture: altitude on vertical velocity and
            # carry on through; the hold starts on the far side.
            if self._rng_bad_since is None:
                self._rng_bad_since = time.monotonic()
                self.get_logger().error(
                    "RANGEFINDER OUT OF EKF2 mid-traverse: carrying on through "
                    "with altitude on vertical velocity.")
            self._rng_hold = True
            return True
        if self._rng_bob is None and self._rng_bad_since is None:
            self.moving = False
            self.hold_x, self.hold_y = float(lp.x), float(lp.y)
        return not self._rangefinder_dropout()

    def _blind_push_done(self, total):
        """Past the wall on the blind push: carry on into the room.

        The push covered the whole remaining traverse (it is distance-
        bounded), so the aircraft is through. Landing here would be landing
        in the dark room; instead hold on the far side and let the room hold
        -- on the 2D lidar, which does not need the flow -- take over.
        """
        if self.phase in (self.PHASE_OUTSIDE, self.PHASE_IN):
            self.get_logger().warning(
                f"TRAVERSE: blind push covered the remaining "
                f"{self.blind_traverse_left:.2f} m; treating the window as "
                "passed and handing over to the room hold (lidar).")
            self._begin_clear(f"through on the blind push ({total:.2f} m)")
            return
        super()._blind_push_done(total)

    # ------------------------------------------------------------ the way out

    def _begin_leg(self, index):
        super()._begin_leg(index)
        if self.legs[index].name in ('offset', 'offset_back'):
            self.MOVE_SPEED = min(self.MOVE_SPEED, self.STRAFE_SPEED)

    def _begin_room(self):
        # First call is straight after the inbound CLEAR, before the room
        # height is commanded: target_z is still the traverse height.
        if self.inbound_z is None:
            self.inbound_z = self.target_z
        # One height for the whole flight: the one the window set (centre +
        # traverse_centre_offset). No room-altitude change.
        self.room_alt_set = True
        super()._begin_room()

    def _begin_relock(self):
        if self.EXIT_MODE == 'detect':
            self._begin_exit_find()
            return
        if self.EXIT_MODE != 'retrace':
            super()._begin_relock()
            return
        self.moving = False
        self.yaw_remaining = 0.0
        self.phase = self.PHASE_OUT
        self.EXIT_DISTANCE = self.OUTSIDE_DISTANCE
        self.MOVE_SPEED = self.APPROACH_SPEED
        leg = self.legs[self.leg_by_name['exit']]
        lp = self.local_position
        rel = self.relative_altitude()
        if (self.inbound_hagl is not None and lp is not None
                and lp.dist_bottom_valid and rel is not None):
            # Range-relative: move by (inbound floor height - floor height
            # now), applied to wherever the EKF thinks it is now.
            alt = rel + (self.inbound_hagl - float(lp.dist_bottom))
            self.get_logger().warning(
                f"EXIT (retrace): back to the inbound height above the floor "
                f"({self.inbound_hagl:.2f} m on the TFmini; now "
                f"{float(lp.dist_bottom):.2f} m), then {leg.distance:.2f} m "
                "straight back out along the takeoff heading. The window is "
                "unmarked inside, so no camera relock.")
            self._begin_alt_change(alt, 'exit', 'inbound height above the floor')
        else:
            self.get_logger().warning(
                "EXIT (retrace): no inbound TFmini height recorded; flying out "
                "at the current height.")
            self._begin_leg_named('exit')

    # ------------------------------------------- finding the window, outside

    def _backoffs_used(self):
        return (getattr(self, 'window_backoffs', 0)
                + getattr(self, 'recentre_backoffs', 0))

    def _slow_backoff(self, begin, what):
        """Both backoff ladders share one budget and one slow speed."""
        if self._backoffs_used() >= self.BACKOFF_BUDGET:
            self._abandon(
                f"{what}: the window is still not fully in view after "
                f"{self._backoffs_used()} backoffs (budget {self.BACKOFF_BUDGET})")
            return
        speed = self.APPROACH_SPEED
        self.APPROACH_SPEED = self.BACKOFF_SPEED
        try:
            begin()
        finally:
            self.APPROACH_SPEED = speed

    def _begin_window_backoff(self):
        # No compensation needed afterwards: the traverse is rebuilt from the
        # measured window pose, so the backoff just lengthens the approach.
        self._slow_backoff(super()._begin_window_backoff, 'window search')

    def _begin_backoff(self):
        self._slow_backoff(super()._begin_backoff, 'recentre')

    # ------------------------------------ the window on the lidar, inbound

    def _body_to_ned(self, x, y):
        """FLU body offset -> NED offset at the current heading."""
        h = self.local_position.heading
        return (x * math.cos(h) + y * math.sin(h),
                x * math.sin(h) - y * math.cos(h))

    def lidar_window_ahead(self, width):
        """The gap in the wall ahead from the levelled scan, in NED, or None.

        {'centre': (n, e) of the gap centre on the wall line, 'normal':
        (n, e) unit, window -> aircraft, 'd', 'width'}.
        """
        lp = self.local_position
        if lp is None or not self._fresh(self.exit_scan) or width is None:
            return None
        m = self.exit_scan[1]
        g = wall_gap_body(m.ranges, m.angle_min, m.angle_increment,
                          min_width=self.GAP_MIN * width,
                          max_width=self.GAP_MAX * width)
        if g is None:
            return None
        dn, de = self._body_to_ned(*g['centre'])
        wn, we = self._body_to_ned(math.cos(g['phi']), math.sin(g['phi']))
        return {'centre': (lp.x + dn, lp.y + de), 'normal': (-wn, -we),
                'd': g['d'], 'width': g['width'], 'lateral': g['lateral']}

    def window_estimate(self):
        """The camera's estimate, laterally and in angle corrected by the lidar.

        Outside, before the commit. The camera keeps the height (the lidar is a
        horizontal plane); the lidar's gap centre on the fitted wall line and
        that line's normal replace the camera's horizontal centre and normal
        when the two agree to lidar_window_agree. Agreement is required: a gap
        30 cm from where the camera sees the blue window is some other hole.
        """
        est = super().window_estimate()
        if (est is None or self.phase != self.PHASE_OUTSIDE
                or self.current_stage == self.TRAVERSE):
            return est
        lw = self.lidar_window_ahead(float(est['width']))
        if lw is None:
            return est
        c = est['centre']
        if math.hypot(lw['centre'][0] - c[0], lw['centre'][1] - c[1]) \
                > self.LIDAR_WINDOW_AGREE:
            self.get_logger().warning(
                f"Lidar gap {math.hypot(lw['centre'][0] - c[0], lw['centre'][1] - c[1]):.2f} m "
                "from the camera's window: not the same opening, ignored.",
                throttle_duration_sec=2.0)
            return est
        est = dict(est)
        est['camera_centre'] = np.array(c, dtype=float)
        est['centre'] = np.array([lw['centre'][0], lw['centre'][1], c[2]])
        est['normal'] = np.array([lw['normal'][0], lw['normal'][1], 0.0])
        est['lidar'] = True
        self.lidar_window = lw
        self.lidar_window_time = time.monotonic()
        return est

    def _aligned(self):
        """Inherited tolerances, AND the pose it is aligned to must be the
        lidar-corrected one -- or, if the lidar never sees the gap, a camera
        pose that has held still over the settle."""
        if not super()._aligned():
            return False
        if self.phase != self.PHASE_OUTSIDE:
            return True
        est = self.window_estimate()
        if est is None:
            return False
        now = time.monotonic()
        if est.get('lidar'):
            return True
        if (self.lidar_window_time is not None
                and now - self.lidar_window_time < 3.0):
            return False            # it was seeing it: wait for it
        if self._in_stage_for() < self.LIDAR_REQUIRED_S:
            self.get_logger().info(
                "ALIGN: waiting for the lidar to see the window gap.",
                throttle_duration_sec=2.0)
            return False
        c = np.asarray(est['centre'], dtype=float)
        self.camera_centres = [(t, p) for t, p in self.camera_centres
                               if now - t <= max(self.ALIGN_SETTLE_SECONDS, 1.0)]
        self.camera_centres.append((now, c))
        pts = np.array([p[:2] for _, p in self.camera_centres])
        spread = float(np.max(np.linalg.norm(pts - pts.mean(axis=0), axis=1)))
        if len(pts) < 5 or spread > self.CAMERA_STABLE:
            self.get_logger().warning(
                f"ALIGN: no lidar gap; camera centre spread {spread * 100:.0f} cm, "
                f"need <= {self.CAMERA_STABLE * 100:.0f} cm to commit on it.",
                throttle_duration_sec=2.0)
            return False
        return True

    def _begin_traverse(self):
        self.traverse_shift = 0.0
        if self.phase == self.PHASE_OUTSIDE and self.inbound_window is None:
            est = self.window_estimate()
            if est is not None:
                self.inbound_window = (float(est['width']), float(est['height']))
                self.get_logger().warning(
                    "TRAVERSE on the "
                    + ("LIDAR-corrected window (lateral + wall angle), camera "
                       "height" if est.get('lidar') else "CAMERA window only")
                    + ".")
        super()._begin_traverse()

    def _handle_traverse(self):
        """Inbound: keep the line on the lidar gap until close to the wall."""
        # Inbound only: the commit has already set phase IN (the exit is
        # EXIT_TRAVERSE, and 'relock' mode flies TRAVERSE with phase OUT).
        if (self.phase == self.PHASE_IN and self.inbound_window is not None
                and self.traverse_exit is not None and self.local_position is not None
                and time.monotonic() - getattr(self, '_steer_last', 0.0) >= 0.18):
            self._steer_last = time.monotonic()
            lw = self.lidar_window_ahead(self.inbound_window[0])
            if lw is not None and lw['d'] > self.EXIT_FREEZE:
                h = self.traverse_heading
                perp = (-math.sin(h), math.cos(h))          # NED, to the right
                off = ((lw['centre'][0] - self.traverse_entry[0]) * perp[0]
                       + (lw['centre'][1] - self.traverse_entry[1]) * perp[1])
                step = self.TRAVERSE_STEER_GAIN * off
                new = max(-self.TRAVERSE_STEER_MAX,
                          min(self.TRAVERSE_STEER_MAX, self.traverse_shift + step))
                step = new - self.traverse_shift
                if abs(step) > 1e-4:
                    self.traverse_shift = new
                    for pt in (self.traverse_entry, self.traverse_exit):
                        pt[0] += step * perp[0]
                        pt[1] += step * perp[1]
                    self._set_target(self.traverse_exit[0], self.traverse_exit[1])
                self.get_logger().info(
                    f"TRAVERSE on the lidar: gap {off * 100:+.0f} cm off the line "
                    f"(+ right), line moved {self.traverse_shift * 100:+.0f} cm, "
                    f"{lw['d']:.2f} m to the wall.", throttle_duration_sec=0.5)
        super()._handle_traverse()

    # ------------------------------------------- the room: turn in place

    def _window_facing_ned(self):
        """NED heading square-on to the window wall from inside (arena -Y)."""
        if self.arena_tf is None:
            return wrap_pi(self.room_entry_heading + math.pi)
        # arena yaw a -> ENU yaw theta + a -> NED heading pi/2 - that.
        return wrap_pi(math.pi / 2.0 - (self.arena_tf[0] - math.pi / 2.0))

    def _handle_tile_return(self):
        """Back to the relock station, THEN the half turn, on the spot.

        Translating and yawing together is what walked the aircraft sideways
        in the room. Here the heading is frozen until the station is reached
        (lidar-closed), then it turns slowly while the lidar holds the point.
        """
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.update_transform()
        self.log_flight_state()
        if self.arena_tf is None:
            self._finish_tile_scan('no transform for the return')
            return

        sx, sy = self.relock_station_arena()
        n, e, _ = self.arena_to_ned(sx, sy)
        self._set_target(n, e)
        lp = self.local_position
        err = self.lidar_error(sx, sy)
        remaining = err if err is not None else math.hypot(n - lp.x, e - lp.y)
        facing = self._window_facing_ned()

        if not self._return_turning:
            self.yaw_remaining = 0.0
            if remaining <= self.TILE_ARRIVE_EPS:
                self._return_turning = True
                self.YAW_RATE = self.ROOM_YAW_RATE
                self.tile_arrived_since = None
                self.get_logger().warning(
                    f"At the relock station ({remaining * 100:.0f} cm on the "
                    "lidar). Turning to face the window, in place, at "
                    f"{math.degrees(self.ROOM_YAW_RATE):.0f} deg/s.")
            elif self._in_stage_for() > self.TILE_MOVE_TIMEOUT:
                self._finish_tile_scan(
                    f"relock station not reached, {remaining:.2f} m short")
            else:
                self.get_logger().info(
                    f"Returning to the relock station, {remaining:.2f} m.",
                    throttle_duration_sec=1.0)
            return

        self.yaw_remaining = wrap_pi(facing - self.yaw_setpoint)
        turned = self._heading_error(facing) <= self.EXIT_YAW_TOL
        if turned and remaining <= self.TILE_ARRIVE_EPS:
            if self.tile_arrived_since is None:
                self.tile_arrived_since = time.monotonic()
            elif time.monotonic() - self.tile_arrived_since >= self.TILE_SETTLE_SECONDS:
                self._return_turning = False
                self.YAW_RATE = self.cruise_yaw_rate
                self._finish_tile_scan('all tiles visited')
            return
        self.tile_arrived_since = None
        if self._in_stage_for() > self.TILE_MOVE_TIMEOUT + 30.0:
            self._return_turning = False
            self.YAW_RATE = self.cruise_yaw_rate
            self._finish_tile_scan(
                f"half turn incomplete, "
                f"{math.degrees(self._heading_error(facing)):.0f} deg off")
            return
        self.get_logger().info(
            f"Turning in place: {math.degrees(self._heading_error(facing)):.0f} "
            f"deg to go, {remaining * 100:.0f} cm off the station (lidar).",
            throttle_duration_sec=1.0)

    # ------------------------------------ the way out: find, align, go

    def _init_exit(self):
        self.exit_scan = None
        self.exit_gray = None
        self.exit_depth = None
        self.exit_K = None
        self.exit_hist = []
        self.exit_last_eval = 0.0
        self.exit_gap_x = None
        self.exit_gap_width = None
        self.exit_cues = []
        self.exit_z = None
        self.exit_in_band_since = None
        self.exit_goal = None
        self.exit_frozen = None
        self.exit_start = None
        self.exit_total = None
        topic = lambda name, default: str(self.declare_parameter(name, default).value)
        q = qos_profile_sensor_data
        self.create_subscription(
            LaserScan, topic('exit_scan_topic', '/lidar/scan_level'),
            lambda m: setattr(self, 'exit_scan', (time.monotonic(), m)), q)
        self.create_subscription(
            Image, topic('exit_color_topic', '/camera/camera/color/image_raw'),
            lambda m: setattr(self, 'exit_gray', (time.monotonic(), m)), q)
        self.create_subscription(
            Image, topic('exit_depth_topic',
                         '/camera/camera/aligned_depth_to_color/image_raw'),
            lambda m: setattr(self, 'exit_depth', (time.monotonic(), m)), q)
        self.create_subscription(
            CameraInfo, topic('exit_info_topic', '/camera/camera/color/camera_info'),
            self._exit_info_callback, q)

    def _exit_info_callback(self, msg):
        k = msg.k
        if k[0] > 0.0:
            self.exit_K = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))

    def _exit_width(self):
        if math.isfinite(self.EXIT_WINDOW_WIDTH):
            return self.EXIT_WINDOW_WIDTH
        if self.inbound_window is not None:
            return self.inbound_window[0]
        return 0.60

    def _arena_pose(self):
        """(x, y, yaw) in the arena: the lidar fix, else through the transform."""
        if self.lidar_fix is not None and self.lidar_fix_is_fresh():
            return self.lidar_fix[0], self.lidar_fix[1], self.lidar_fix[3]
        lp = self.local_position
        if self.arena_tf is None or lp is None:
            return None
        ax, ay = self.ned_to_arena(lp.x, lp.y)
        yaw = wrap_pi(math.pi / 2.0 - lp.heading - self.arena_tf[0])
        return ax, ay, yaw

    def _fresh(self, item, max_age=0.5):
        return item is not None and time.monotonic() - item[0] <= max_age

    def exit_cues_now(self):
        """Evaluate the three cues on the newest frames. {name: result|None}."""
        from drone_testing.window_detect import imgmsg_to_bgr, imgmsg_to_depth
        import cv2
        pose = self._arena_pose()
        cues = {'lidar': None, 'depth': None, 'bright': None}
        if pose is None:
            return cues
        wall_y = -self.ROOM_Y / 2.0
        w = self._exit_width()
        lo, hi = self.GAP_MIN * w, self.GAP_MAX * w
        if self._fresh(self.exit_scan):
            m = self.exit_scan[1]
            cues['lidar'] = lidar_gap(
                m.ranges, m.angle_min, m.angle_increment, pose, wall_y,
                fov=math.radians(75.0), min_width=lo, max_width=hi)
        if self.exit_K is not None:
            x, y, yaw = pose
            cam = (x + self.EXIT_CAM_X * math.cos(yaw),
                   y + self.EXIT_CAM_X * math.sin(yaw), yaw)
            try:
                if self._fresh(self.exit_depth):
                    cues['depth'] = depth_hole(
                        imgmsg_to_depth(self.exit_depth[1]), self.exit_K, cam,
                        wall_y, min_width=lo, max_width=hi)
                if self._fresh(self.exit_gray):
                    gray = cv2.cvtColor(imgmsg_to_bgr(self.exit_gray[1]),
                                        cv2.COLOR_BGR2GRAY)
                    cues['bright'] = bright_region(
                        gray, self.exit_K, cam, wall_y, min_width=lo, max_width=hi)
            except Exception as exc:  # a bad frame must not stop the hold
                self.get_logger().warning(f"EXIT: camera cue failed: {exc}",
                                          throttle_duration_sec=2.0)
        return cues

    def _cue_words(self, cues):
        parts = []
        for k in ('lidar', 'depth', 'bright'):
            c = cues.get(k)
            parts.append(f"{k} -" if c is None else
                         f"{k} x={c['x']:+.2f} w={c['width']:.2f}")
        return ', '.join(parts)

    def _hold_arena(self, ax, ay, facing):
        n, e, _ = self.arena_to_ned(ax, ay)
        self._set_target(n, e)
        self.yaw_remaining = wrap_pi(facing - self.yaw_setpoint)
        err = self.lidar_error(ax, ay)
        if err is None:
            lp = self.local_position
            err = math.hypot(n - lp.x, e - lp.y)
        return err

    def _begin_exit_find(self):
        if self._yaw_suspect:
            self._room_land("the yaw is not trusted, so the way out cannot be "
                            "flown")
            return
        if self.arena_tf is None:
            self.get_logger().error(
                "EXIT: no lidar arena frame (the room scan never got one), so "
                "the window cannot be re-found on the lidar. Falling back to "
                "the retrace exit.")
            self.EXIT_MODE = 'retrace'
            self._begin_relock()
            return
        self.phase = self.PHASE_OUT
        self.moving = False
        self.yaw_remaining = 0.0
        self.YAW_RATE = self.ROOM_YAW_RATE
        self.MOVE_SPEED = self.TILE_SPEED
        self.exit_hist = []
        self.exit_station = self.relock_station_arena()
        self.get_logger().warning(
            "EXIT: the window is unmarked inside. Looking for it with the lidar "
            f"gap, the depth hole and the brightness; need {self.EXIT_NEED} "
            f"to agree {self.EXIT_CONFIRM} times. Holding at arena "
            f"({self.exit_station[0]:+.2f}, {self.exit_station[1]:+.2f}) on the lidar.")
        self._enter_stage(self.EXIT_FIND)

    def _handle_exit_find(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.update_transform()
        self.log_flight_state()
        self._hold_arena(*self.exit_station, self._window_facing_ned())

        now = time.monotonic()
        if now - self.exit_last_eval < 0.2:
            return
        self.exit_last_eval = now
        cues = self.exit_cues_now()
        tol = max(0.10, 0.25 * self._exit_width())
        x, names = vote(cues, tol, need=self.EXIT_NEED)
        self.get_logger().info(f"EXIT cues: {self._cue_words(cues)}",
                               throttle_duration_sec=1.0)
        if x is not None and self.exit_hist and abs(x - self.exit_hist[-1][0]) > tol:
            self.exit_hist = []
        if x is None:
            self.exit_hist = []
        else:
            width = (cues['lidar'] or cues['depth'] or cues['bright'])['width']
            self.exit_hist.append((x, width, names, cues))
        if len(self.exit_hist) >= self.EXIT_CONFIRM:
            xs = sorted(h[0] for h in self.exit_hist)
            self._begin_exit_align(xs[len(xs) // 2],
                                   sorted(h[1] for h in self.exit_hist)[len(xs) // 2],
                                   self.exit_hist[-1][2], self.exit_hist[-1][3])
            return

        if self._in_stage_for() > self.EXIT_FIND_TIMEOUT:
            # Not two cues. The entry axis is itself a lidar measurement (where
            # the aircraft stood after coming through), so a live lidar gap
            # that agrees with it is accepted as the second cue. Otherwise fly
            # out on the entry axis: better than landing in the dark room.
            entry = None if self.arena_entry is None else self.arena_entry[0]
            g = cues.get('lidar')
            if g is not None and entry is not None and abs(g['x'] - entry) <= tol:
                self._begin_exit_align(g['x'], g['width'], ['lidar', 'entry axis'], cues)
            elif entry is not None:
                self.get_logger().error(
                    f"EXIT: no {self.EXIT_NEED} agreeing cues in "
                    f"{self.EXIT_FIND_TIMEOUT:.0f} s ({self._cue_words(cues)}). "
                    "Flying out on the ENTRY AXIS measured on the way in.")
                self._begin_exit_align(entry, self._exit_width(), ['entry axis'], cues)
            else:
                self._abandon("window not re-found from inside and no entry axis")

    def _begin_exit_align(self, x, width, names, cues):
        self.exit_gap_x = float(x)
        self.exit_gap_width = float(width)
        self.exit_cues = list(names)
        self.exit_in_band_since = None
        lp = self.local_position
        # Same height above the floor as the way in: range-relative.
        if self.inbound_hagl is not None and lp is not None and lp.dist_bottom_valid:
            self.exit_z = float(lp.z) - (self.inbound_hagl - float(lp.dist_bottom))
        else:
            self.exit_z = self.inbound_z if self.inbound_z is not None else self.target_z
        self.target_z = self.exit_z
        body = self.DRONE_WIDTH
        air = 0.5 * (self.exit_gap_width - body)
        self.get_logger().warning(
            f"EXIT WINDOW at arena x {x:+.2f} m, {width:.2f} m wide, from "
            f"{' + '.join(names)} ({self._cue_words(cues)}). "
            f"{air:.2f} m of air each side. Aligning on its axis at the "
            f"inbound height ({self.inbound_hagl or 0.0:.2f} m above the floor).")
        vert = [c['up'] for k, c in cues.items()
                if c is not None and k != 'lidar' and 'up' in c]
        if vert:
            self.get_logger().info(
                f"EXIT: camera sees the window centre {sum(vert) / len(vert):+.2f} m "
                "from its axis (vertical), for the record.")
        self._enter_stage(self.EXIT_ALIGN)

    def _handle_exit_align(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.update_transform()
        self.log_flight_state()
        facing = self._window_facing_ned()
        sx, sy = self.exit_gap_x, self.exit_station[1]
        err = self._hold_arena(sx, sy, facing)
        self.target_z = self.exit_z

        # Keep refining the axis on the live lidar gap while standing still.
        if time.monotonic() - self.exit_last_eval >= 0.2:
            self.exit_last_eval = time.monotonic()
            pose = self._arena_pose()
            if pose is not None and self._fresh(self.exit_scan):
                m = self.exit_scan[1]
                w = self._exit_width()
                g = lidar_gap(m.ranges, m.angle_min, m.angle_increment, pose,
                              -self.ROOM_Y / 2.0, fov=math.radians(75.0),
                              min_width=self.GAP_MIN * w, max_width=self.GAP_MAX * w)
                if g is not None and abs(g['x'] - self.exit_gap_x) <= 0.10:
                    self.exit_gap_x += 0.2 * (g['x'] - self.exit_gap_x)

        lp = self.local_position
        dz = (abs(float(lp.dist_bottom) - self.inbound_hagl)
              if self.inbound_hagl is not None and lp.dist_bottom_valid
              else abs(lp.z - self.exit_z))
        yaw_err = self._heading_error(facing)
        ok = (err <= self.EXIT_ALIGN_TOL and dz <= self.EXIT_HEIGHT_TOL
              and yaw_err <= self.EXIT_YAW_TOL)
        if ok:
            if self.exit_in_band_since is None:
                self.exit_in_band_since = time.monotonic()
            elif time.monotonic() - self.exit_in_band_since >= self.EXIT_ALIGN_HOLD:
                self._begin_exit_traverse()
            return
        self.exit_in_band_since = None
        if self._in_stage_for() > self.EXIT_ALIGN_TIMEOUT:
            if err <= 2.0 * self.EXIT_ALIGN_TOL and dz <= 2.0 * self.EXIT_HEIGHT_TOL:
                self.get_logger().warning(
                    f"EXIT ALIGN: {err * 100:.0f} cm / {dz * 100:.0f} cm after "
                    f"{self.EXIT_ALIGN_TIMEOUT:.0f} s, inside twice the "
                    "tolerance. Going.")
                self._begin_exit_traverse()
                return
        self.get_logger().info(
            f"EXIT ALIGN: {err * 100:.0f} cm lateral (lidar), {dz * 100:.0f} cm "
            f"height, {math.degrees(yaw_err):.1f} deg yaw.",
            throttle_duration_sec=1.0)

    def _begin_exit_traverse(self):
        lp = self.local_position
        pose = self._arena_pose()
        wall_y = -self.ROOM_Y / 2.0
        to_wall = (pose[1] - wall_y) if pose is not None else self.RELOCK_STANDOFF
        self.exit_total = to_wall + self.OUTSIDE_DISTANCE
        self.exit_start = (float(lp.x), float(lp.y))
        self.exit_frozen = None
        self.exit_progress = None
        self.MOVE_SPEED = self.EXIT_SPEED
        self.exit_goal = (self.exit_gap_x, wall_y - self.OUTSIDE_DISTANCE)
        self.get_logger().warning(
            f"EXIT TRAVERSE: {to_wall:.2f} m to the window plane, then "
            f"{self.OUTSIDE_DISTANCE:.2f} m beyond, at {self.EXIT_SPEED:.2f} m/s. "
            f"Steered on the lidar until {self.EXIT_FREEZE:.2f} m from the "
            "wall, straight on from there.")
        self._enter_stage(self.EXIT_TRAVERSE)

    def _handle_exit_traverse(self):
        if not self._still_flyable():
            return
        self.update_transform()
        self.log_flight_state()
        lp = self.local_position
        wall_y = -self.ROOM_Y / 2.0
        self.yaw_remaining = wrap_pi(self._window_facing_ned() - self.yaw_setpoint) \
            if self.exit_frozen is None else 0.0

        if self.exit_frozen is None:
            fix_ok = self.lidar_fix is not None and self.lidar_fix_is_fresh()
            to_wall = self.lidar_fix[1] - wall_y if fix_ok else None
            if fix_ok and to_wall > self.EXIT_FREEZE:
                # The live gap must still hold the whole airframe with air
                # either side. If it does not, stop and re-align.
                if self._fresh(self.exit_scan):
                    m = self.exit_scan[1]
                    w = self._exit_width()
                    g = lidar_gap(m.ranges, m.angle_min, m.angle_increment,
                                  self._arena_pose(), wall_y,
                                  fov=math.radians(80.0),
                                  min_width=self.GAP_MIN * w, max_width=self.GAP_MAX * w)
                    body = self.DRONE_WIDTH
                    if g is not None:
                        half_air = 0.5 * (g['width'] - body)
                        off = abs(self.lidar_fix[0] - g['x'])
                        if off > half_air - self.EXIT_SIDE_MARGIN:
                            self.moving = False
                            self.hold_x, self.hold_y = lp.x, lp.y
                            self.get_logger().error(
                                f"EXIT: {off * 100:.0f} cm off the live gap "
                                f"centre, only {half_air * 100:.0f} cm of air. "
                                "Stopping and re-aligning.")
                            self.exit_gap_x = g['x']
                            self.exit_in_band_since = None
                            self._enter_stage(self.EXIT_ALIGN)
                            return
                n, e, _ = self.arena_to_ned(*self.exit_goal)
                self._set_target(n, e)
            else:
                # Lidar about to lose the room: fly the rest in a straight
                # line on EKF2, along the last lidar-steered heading.
                n, e, _ = self.arena_to_ned(*self.exit_goal)
                self.exit_frozen = (n, e)
                self._set_target(n, e)
                self.get_logger().warning(
                    "EXIT: at the window plane"
                    + ("" if fix_ok else " (lidar fix gone)")
                    + ", holding the line on the flow.")

        travelled = math.hypot(lp.x - self.exit_start[0], lp.y - self.exit_start[1])
        left = self.exit_total - travelled
        if left <= self.MOVE_TOLERANCE:
            self.moving = False
            self.hold_x, self.hold_y = lp.x, lp.y
            self._dispatch_after('exit_clear')
            return
        # Stall test on PROGRESS, not a wall-clock budget (a slow simulator
        # or a slow approach is not a failure; not moving is).
        now = time.monotonic()
        if self.exit_progress is None or travelled > self.exit_progress[1] + 0.05:
            self.exit_progress = (now, travelled)
        elif now - self.exit_progress[0] > self.EXIT_STALL_S:
            self.moving = False
            self.hold_x, self.hold_y = lp.x, lp.y
            if self.exit_frozen is None:
                self.get_logger().error(
                    f"EXIT: no progress for {self.EXIT_STALL_S:.0f} s before the "
                    "window plane. Re-aligning on the gap.")
                self.exit_in_band_since = None
                self._enter_stage(self.EXIT_ALIGN)
            else:
                self.get_logger().error(
                    f"EXIT: no progress for {self.EXIT_STALL_S:.0f} s, {left:.2f} m "
                    "short but past the window plane; treating as out.")
                self._dispatch_after('exit_clear')
            return
        self.get_logger().info(f"EXIT TRAVERSE: {left:.2f} m to go.",
                               throttle_duration_sec=1.0)

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
            self.YAW_RATE = self.cruise_yaw_rate
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
        if self.current_stage in self.EXIT_STAGES:
            return 'EXITING'
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
