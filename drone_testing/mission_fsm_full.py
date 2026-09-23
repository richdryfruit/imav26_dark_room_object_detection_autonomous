"""
THE WHOLE MISSION IN ONE FLIGHT: pad -> window marker -> dark room -> marker.

    arm -> climb to 2.50 m -> hold
      -> forward up to 9.3 m ON THE CARPET TRACK (down camera, velocity
         loop at 0.6 m/s), watching for the WINDOW marker
         (id 0). Seen: centre on it and hold. NOT SEEN: carry on anyway from
         the end of the leg.
      -> NO LANDING HERE. Descend to 1.75 m and run part 2 unchanged:
         strafe 0.15 m left, the blue window (lidar-corrected), 0.80 m into
         the dark room, the square on the lidar, the half turn in place,
         the window re-found from inside, out, mirrored strafe
      -> hover over the window marker, centre, land on it.

    ros2 run drone_testing mission_fsm_full            # + its launch file

EVERYTHING IS mission_fsm_part1's AND mission_fsm_part2's
---------------------------------------------------------
This is MissionFSMPart2 with one extra leg in front of its plan and one
altitude change after it. Part 1's leg is the same marker-terminated
CRUISE -> MARKER_ALIGN -> MARKER_HOLD primitive, flown at the same 2.10 m;
what it does NOT get is part 1's landing, because this flight continues.

WHY CENTRE ON THE MARKER AT ALL IF IT IS NOT LANDING ON IT
-----------------------------------------------------------
The marker is the one absolute position fix between the pad and the window.
Centring on it is what makes the strafe, the standoff and the window search
start from a known point instead of from 9.3 m of accumulated flow drift.
Missing it is survivable (the leg ends at outbound_distance and the window
search widens from there), which is why the leg does not retry.

Keys:  q -> abort into a controlled descent.  k -> force-disarm.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

from px4_msgs.msg import TrajectorySetpoint

from drone_testing.commons import track_control
from drone_testing.mission_fsm import Leg
from drone_testing.mission_fsm_part2 import MissionFSMPart2
from drone_testing.offboard_sequence import spin_node


class MissionFSMFull(MissionFSMPart2):
    """Part 1 and part 2 as one flight, with no landing in between."""

    CRUISE_ALTITUDE = 2.50      # m for the climb and the track leg
                                # (track_traversal_node's altitude)
    OUTBOUND_DISTANCE = 9.3     # m FORWARD: a LIMIT, not a target
    WINDOW_ALTITUDE = 1.75      # m from the marker on, as part 2
    ROOM_ALTITUDE = 1.75
    FLIGHT_SECONDS = 660.0      # part 1 (240) + part 2 (420). A backstop.

    # ---- the carpet track under the leg ---------------------------------
    # Flown the way track_traversal_node flies it: a VELOCITY loop on the
    # track itself, not a position line. Lateral velocity nulls the track's
    # offset, yaw rate nulls its tilt so the nose follows the track, and
    # forward speed is allowed only while the track is near-vertical in the
    # image. Optical flow drifts over 9.3 m; the track does not move.
    FOLLOW_LINE = True
    TRACK_SPEED = 0.6           # m/s body +x, and the cap on |(vx, vy)|
    TRACK_ACCEL = 0.15          # m/s^2 ramp up
    TRACK_DECEL = 0.60          # m/s^2 ramp down
    TRACK_KP_LAT = 0.6          # (m/s) per unit of offset_norm  \  track_
    TRACK_MAX_LAT = 0.30        # m/s of lateral correction       |  traversal
    TRACK_KP_YAW = 0.8          # (rad/s) per radian of tilt      |  _node's
    TRACK_MAX_YAW = 0.50        # rad/s                           |  own
    TRACK_TOL = 10.0            # deg: forward only inside this   |  gains
    TRACK_HYST = 3.0            # deg of hysteresis on it         /
    LINE_MAX_AGE = 0.7          # s: older than this is not a fix
    LINE_PERIOD = 0.0           # every tick: this is a control loop now
    LINE_AGREE = 0.25           # of a half-frame: two fixes in a row must
                                # agree, or it is a DIFFERENT track (the cage
                                # has three parallel ones)
    # ---- and it must be OUR lane ----------------------------------------
    # The cage has three parallel lanes 1.50 m apart, and the CENTRE one
    # carries the obstacles. Drift at takeoff, or a lane change mid-leg, puts
    # the wrong one under the camera -- and the detector is perfectly happy
    # with it: a wrong lane centred in frame looks exactly like the right one.
    # Two independent things say otherwise:
    #   WIDTH     ours is track_width_m; the centre lane is about twice that,
    #             and width in the frame is measurable against the height.
    #   POSITION  following the wrong lane means being about a lane-spacing
    #             off the line this leg started on, which EKF2 knows even
    #             while it drifts, because the error is metres, not
    #             centimetres.
    # MEASURED CAGE GEOMETRY:
    #   our lane        0.60 m wide
    #   centre lane     1.20 m wide, and it carries the obstacles
    #   lane spacing    1.50 m between centres (the drawing's '1502,5', and
    #                   the strip centres scaled against its 7000 mm width)
    # which leaves 0.60 m of bare floor between our lane's edge and the
    # centre lane's. The width tolerance has to separate 0.60 from 1.20, and
    # the lane guard has to trip before the aircraft can reach the next lane
    # centre, i.e. well under half of 1.50 m.
    TRACK_WIDTH_M = 0.60        # m, our lane. 0 disables the width check.
    TRACK_WIDTH_TOL = 0.25      # fraction: accepts 0.45-0.75, rejects 1.20
    TRACK_HFOV_DEG = 70.4       # of the down camera, for width in metres
    TRACK_LANE_GUARD = 0.60     # m off the leg's original line before the
                                # track is not ours to follow. Under half the
                                # 1.50 m spacing, and it also trips before
                                # the 0.60 m of bare floor has been crossed.

    LINE_TRUST = 0.80           # and it must be OURS: the detector reports
                                # +/-1 across the frame, and the aircraft is
                                # corrected continuously, so a track out at
                                # the frame edge is the next one over.

    def __init__(self):
        super().__init__()
        # OUTBOUND_DISTANCE is MissionFSM's own parameter, already declared.
        # In front of part 2's plan. 'window_alt' is handled in
        # _dispatch_after: drop to the window height, then part 2's 'offset'.
        self.legs.insert(0, Leg('to_marker', 'forward', self.OUTBOUND_DISTANCE,
                                self.WINDOW_MARKER_ID, 'window_alt',
                                retry=False))
        self.leg_by_name = {leg.name: i for i, leg in enumerate(self.legs)}

        self.FOLLOW_LINE = bool(self.declare_parameter(
            'follow_line', self.FOLLOW_LINE).value)
        num = self._declare_number
        self.TRACK_SPEED = float(num('track_speed', self.TRACK_SPEED))
        self.TRACK_ACCEL = float(num('track_accel', self.TRACK_ACCEL))
        self.TRACK_DECEL = float(num('track_decel', self.TRACK_DECEL))
        self.TRACK_KP_LAT = float(num('track_kp_lat', self.TRACK_KP_LAT))
        self.TRACK_MAX_LAT = float(num('track_max_lat', self.TRACK_MAX_LAT))
        self.TRACK_KP_YAW = float(num('track_kp_yaw', self.TRACK_KP_YAW))
        self.TRACK_MAX_YAW = float(num('track_max_yaw', self.TRACK_MAX_YAW))
        self.TRACK_TOL = float(num('track_vertical_tol_deg', self.TRACK_TOL))
        self.TRACK_HYST = float(num('track_vertical_hyst_deg', self.TRACK_HYST))
        self.LINE_MAX_AGE = float(num('line_max_age', self.LINE_MAX_AGE))
        self.LINE_AGREE = float(num('line_agree', self.LINE_AGREE))
        self.LINE_TRUST = float(num('line_trust', self.LINE_TRUST))
        self.TRACK_WIDTH_M = float(num('track_width_m', self.TRACK_WIDTH_M))
        self.TRACK_WIDTH_TOL = float(num('track_width_tol', self.TRACK_WIDTH_TOL))
        self.TRACK_HFOV_DEG = float(num('track_hfov_deg', self.TRACK_HFOV_DEG))
        self.TRACK_LANE_GUARD = float(num('track_lane_guard',
                                          self.TRACK_LANE_GUARD))
        self._leg_line = None       # (start_x, start_y, heading) before steering
        self.line_fix = None        # (t, offset_norm, angle_deg, width_norm)
        self._line_prev = None
        self._track_cmd = None          # (vx, vy, yawspeed) body FRD
        self._track_vx = 0.0            # slew-limited forward speed
        self._track_go = False          # the forward gate's state
        self.line_enable_pub = self.create_publisher(
            Bool, str(self.declare_parameter(
                'floor_line_enable_topic', '/floor_line_enable').value), 10)
        self._line_on = None
        self.create_subscription(
            PointStamped, str(self.declare_parameter(
                'floor_line_topic', '/floor_line').value),
            self._line_callback, 10)

    def _line_wanted(self, on):
        if self._line_on != on:
            self._line_on = on
            self.line_enable_pub.publish(Bool(data=on))
            self._line_prev = None

    def _begin_leg(self, index):
        super()._begin_leg(index)
        self._leg_line = None
        self._line_wanted(self.FOLLOW_LINE
                          and self.legs[index].name == 'to_marker')

    def _line_callback(self, msg):
        if msg.header.frame_id == 'track':   # floor_line's confirmed fix
            self.line_fix = (time.monotonic(), float(msg.point.x),
                             float(msg.point.y), float(msg.point.z))

    def _line_is_fresh(self):
        return (self.line_fix is not None
                and time.monotonic() - self.line_fix[0] <= self.LINE_MAX_AGE)

    def _track_is_ours(self):
        """Is the track under the camera OUR lane, or the next one over?

        Width first: ours is TRACK_WIDTH_M, the centre lane about twice that,
        and the detector reports width as a fraction of the frame, which the
        height turns into metres. Then position: having wandered a whole
        lane-spacing off the line this leg started on is not a track to
        follow, it is a track to get off.
        """
        lp = self.local_position
        if self.TRACK_WIDTH_M > 0.0 and lp is not None and lp.dist_bottom_valid:
            frame_m = 2.0 * float(lp.dist_bottom) * math.tan(
                math.radians(self.TRACK_HFOV_DEG) / 2.0)
            width_m = self.line_fix[3] * frame_m
            if abs(width_m - self.TRACK_WIDTH_M) > \
                    self.TRACK_WIDTH_TOL * self.TRACK_WIDTH_M:
                self.get_logger().error(
                    f"Track under the camera is {width_m:.2f} m wide; ours is "
                    f"{self.TRACK_WIDTH_M:.2f} m. That is a DIFFERENT LANE "
                    "(the centre one carries the obstacles). Not following "
                    "it; flying the leg's own line instead.",
                    throttle_duration_sec=2.0)
                return False
        if self._leg_line is not None and lp is not None and self.TRACK_LANE_GUARD > 0.0:
            x0, y0, heading = self._leg_line
            right = (-math.sin(heading), math.cos(heading))
            cross = (lp.x - x0) * right[0] + (lp.y - y0) * right[1]
            if abs(cross) > self.TRACK_LANE_GUARD:
                self.get_logger().error(
                    f"{cross:+.2f} m off the line this leg started on -- about "
                    "a lane spacing. Whatever is under the camera is not our "
                    "lane. Flying the leg's own line back onto it.",
                    throttle_duration_sec=2.0)
                return False
        return True

    def _handle_cruise(self):
        """The straight leg, flown ON the carpet track while it is in view.

        track_traversal_node's law: lateral velocity nulls the offset, yaw
        rate nulls the tilt, forward only while the track is near-vertical.
        The leg's own bookkeeping (the marker that ends it, the distance
        limit, the timeouts) is the base class's and is untouched -- this
        only replaces what is PUBLISHED while the track is visible, which is
        why losing the track simply hands the leg back to the carrot.
        """
        leg = self.current_leg()
        self._track_cmd = None
        if (self.FOLLOW_LINE and leg is not None and leg.name == 'to_marker'
                and self.leg_started and self.local_position is not None
                and self._line_is_fresh()):
            if self._leg_line is None and self.moving:
                # The line this leg is measured against. ANCHORED ON THE PAD
                # MARKER when the climb saw it: the pad sits on our lane, so
                # that is the lane itself. Anchoring on where the leg happens
                # to start instead would accept a climb that had already
                # drifted onto the next lane -- cross-track would read zero
                # while the aircraft sat over the wrong carpet.
                heading = math.atan2(self.move_target_y - self.move_start_y,
                                     self.move_target_x - self.move_start_x)
                if self.pad_marker_ned is not None:
                    x0, y0 = self.pad_marker_ned
                    self.get_logger().info(
                        f"Leg line anchored on the pad marker "
                        f"({x0:+.2f}, {y0:+.2f}) NED -- our lane.")
                else:
                    x0, y0 = self.move_start_x, self.move_start_y
                    self.get_logger().warning(
                        "No pad marker fix from the climb: the leg line is "
                        "anchored where the leg started, so a drift during "
                        "the climb cannot be told from our lane. The width "
                        "check is the only lane guard now.")
                self._leg_line = (x0, y0, heading)
            if not self._track_is_ours():
                super()._handle_cruise()
                return
            det = {'offset_norm': self.line_fix[1],
                   'angle_deg': self.line_fix[2],
                   'track_width_norm': self.line_fix[3]}
            offset = det['offset_norm']
            prev, self._line_prev = self._line_prev, offset
            agrees = prev is not None and abs(offset - prev) <= self.LINE_AGREE
            if abs(offset) > self.LINE_TRUST:
                self.get_logger().warning(
                    f"Track {offset:+.2f} of a half-frame away: that is the "
                    "NEXT track, not ours. Not steering on it.",
                    throttle_duration_sec=2.0)
            elif not agrees:
                self.get_logger().info(
                    f"Track {offset:+.2f}: waiting for a second reading that "
                    "agrees.", throttle_duration_sec=2.0)
            else:
                go = track_control.forward_gate(det, self._track_go,
                                                self.TRACK_TOL, self.TRACK_HYST)
                if go != self._track_go:
                    self.get_logger().warning(
                        f"Track {det['angle_deg']:+.0f} deg: "
                        + ("vertical, going forward." if go else
                           "tilted -- holding forward, yawing onto it."))
                    self._track_go = go
                vy, yawspeed = track_control.track_correction(
                    det, self.TRACK_KP_LAT, self.TRACK_MAX_LAT,
                    self.TRACK_KP_YAW, self.TRACK_MAX_YAW)
                self._track_vx = track_control.slew(
                    self._track_vx, self.TRACK_SPEED if go else 0.0,
                    self.TRACK_ACCEL, self.TRACK_DECEL, 0.05)
                self._track_cmd = track_control.limit_command(
                    self._track_vx, vy, yawspeed, self.TRACK_SPEED,
                    self.TRACK_MAX_LAT, self.TRACK_SPEED, self.TRACK_MAX_YAW)
                self.get_logger().info(
                    f"ON THE TRACK: {offset:+.2f} off centre, "
                    f"{det['angle_deg']:+.0f} deg, flying "
                    f"{self._track_cmd[0]:.2f} m/s fwd, "
                    f"{self._track_cmd[1]:+.2f} m/s lateral, "
                    f"{math.degrees(self._track_cmd[2]):+.0f} deg/s yaw.",
                    throttle_duration_sec=1.0)
        if self._track_cmd is None:
            self._track_vx = 0.0
        super()._handle_cruise()

    def publish_position_setpoint(self):
        """Velocity on the track, position everywhere else.

        Altitude stays a POSITION setpoint (the z ramp the whole mission
        flies); only the horizontal axes become velocities, and only while
        the track loop has a command. Yaw is handed over as a RATE, because
        following the track means turning with it.
        """
        if self._track_cmd is None or self.home_z is None:
            super().publish_position_setpoint()
            return
        vx, vy, yawspeed = self._track_cmd
        lp = self.local_position
        heading = lp.heading if lp is not None else self.yaw_setpoint
        c, s_ = math.cos(heading), math.sin(heading)
        vn, ve = vx * c - vy * s_, vx * s_ + vy * c
        self._step_setpoint_ramp()
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [nan, nan, self.setpoint_z]
        msg.velocity = [vn, ve, nan]
        msg.yaw = nan
        msg.yawspeed = yawspeed
        self.trajectory_setpoint_pub.publish(msg)
        # Keep the inherited ramps tracking the aircraft, so the moment the
        # track is lost the carrot picks up from where it actually is.
        if lp is not None:
            self.hold_x, self.hold_y = float(lp.x), float(lp.y)
            self.yaw_setpoint = float(heading)

    def _plan_summary(self):
        back = 'right' if self.WINDOW_OFFSET_DIRECTION == 'left' else 'left'
        return (
            "MISSION FSM FULL, one flight. Directions in the TAKEOFF frame:\n"
            f"  1. climb {self.CRUISE_ALTITUDE:.2f} m, hold "
            f"{self.HOLD_SECONDS:.0f} s\n"
            f"  2. forward up to {self.OUTBOUND_DISTANCE:.1f} m watching for "
            f"window marker id {self.WINDOW_MARKER_ID}; centre on it if seen, "
            "carry on if not -- NO LANDING HERE\n"
            f"  3. down to {self.WINDOW_ALTITUDE:.2f} m, strafe "
            f"{self.WINDOW_OFFSET_DIRECTION} {self.WINDOW_OFFSET:.2f} m\n"
            f"  4. find the window (<= {self.BACKOFF_BUDGET} slow backoffs), "
            f"traverse at centre {self.TRAVERSE_CENTRE_OFFSET:+.2f} m to "
            f"{self.INSIDE_DISTANCE:.2f} m past it\n"
            f"  5. room on the {self.ROOM_MODE.upper()} (closed loop): "
            f"'{self.ROOM_SEQUENCE}', dolls geotagged and counted\n"
            f"  6. exit '{self.EXIT_MODE}': window re-found from inside, out "
            f"to {self.OUTSIDE_DISTANCE:.2f} m past it\n"
            f"  7. strafe {back} {self.WINDOW_OFFSET:.2f} m, hover "
            f"{self.FINAL_LOOK_SECONDS:.0f} s, centre and land on the marker\n"
            f"  Hard descent at {self.FLIGHT_SECONDS:.0f} s airborne.\n"
            "  q aborts into a descent, k force-disarms.")

    def _handle_hold(self):
        """End of the settle at altitude: fly the straight leg, not the strafe.

        Part 2's version starts at 'offset' (it takes off already on the
        marker). This flight has to get there first.
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
        if nxt != 'window_alt':
            super()._dispatch_after(nxt, on_marker)
            return
        # The one thing part 1 did here was land. This drops to the window
        # height instead and hands straight over to part 2's first leg.
        self._line_wanted(False)        # the strip has done its job
        self.alt_next = None
        self.get_logger().warning(
            ("Centred on window marker id "
             f"{self.WINDOW_MARKER_ID}" if on_marker else
             f"No window marker within {self.OUTBOUND_DISTANCE:.1f} m")
            + f" -- NOT landing. Down to {self.WINDOW_ALTITUDE:.2f} m for the "
            "window.")
        self._begin_alt_change(self.WINDOW_ALTITUDE, 'offset',
                               'the window height, carrying on into part 2')

    def _phase_label(self):
        leg = self.current_leg()
        if (leg is not None and leg.name == 'to_marker'
                and self.current_stage in (self.CRUISE, self.MARKER_ALIGN,
                                           self.MARKER_HOLD)):
            return 'TO_MARKER'
        return super()._phase_label()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionFSMFull()
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
