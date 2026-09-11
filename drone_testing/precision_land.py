#!/usr/bin/env python3
"""
Precision landing on a downward-facing ArUco marker.

    arm -> ground wait -> climb to takeoff_altitude -> hold -> SEARCH for
    the marker -> ALIGN over it to within align_tolerance -> hold there
    aligned_hold_seconds -> land on it.

Everything that can hurt somebody -- arming, the estimator health gates,
the climb ramp and its leash, the descent, the touchdown detection, the
failsafe reporting -- is inherited unchanged from
offboard_sequence.OffboardSequence. This node only adds what happens
between the post-takeoff hold and the landing, exactly as window_scan.py
does. The `sequence` parameter the parent declares is parsed and then
ignored; the flow never enters the STEP stage.

THREE MODES, AND THEY ARE A LADDER -- WALK UP IT
------------------------------------------------
    mode:=bench     Nothing is armed, no Offboard is requested, and NOT A
                    SINGLE setpoint or vehicle command is published. The
                    node only reads the detector and prints where the
                    marker is in vehicle terms:

                        marker is FORWARD 0.20 m, RIGHT 0.30 m
                        -> the drone would move FORWARD 0.20 m, RIGHT 0.30 m

                    This is the axis sign check, and it is the default,
                    because a launch you forgot to configure should do
                    nothing at all. RUN IT FIRST, ON THE GROUND, PROPS OFF.

    mode:=inspect   Flies the climb and the hold, searches, and then
                    reports the point it WOULD fly to -- current position,
                    target position, and the error -- but never sets a
                    lateral waypoint. The vehicle just holds. This is the
                    dry run with the aircraft in the air.

    mode:=align     The real thing: drives the x/y hold point onto the
                    marker, and once inside align_tolerance for
                    align_settle_seconds, holds aligned_hold_seconds and
                    then lands. Pass land_after_align:=false to align and
                    hold without landing -- worth one flight on its own.

WHY x/y IS A POSITION SETPOINT AND NOT A VELOCITY
--------------------------------------------------
Because the marker offset is re-measured every tick, a scale error in the
vision is a LOOP GAIN, not a bias: commanding a correction of s*e leaves
(1-s)*e, which converges for any 0 < s < 2. It does not accumulate. So
scale is not what picks between position and velocity here, and two other
things do:

  * If the marker is lost, a latched position setpoint means the vehicle
    PARKS. A velocity setpoint means it COASTS until something explicitly
    zeroes it, which near the ground is exactly the wrong default.
  * The parent already owns a leashed, EKF2-reset-aware x/y carrot
    (hold_x/hold_y walked towards move_target_x/y by _step_xy_ramp, capped
    at MOVE_LEASH). Driving that is reusing a flown control path instead
    of inventing a new one.

align_gain (0.6) commands only a fraction of the measured offset each
cycle. That is not for the scale -- it is phase margin against camera and
link latency, and it guarantees monotone convergence even if the vision
scale is off by a third in the wrong direction.

THE FRAME, AND THE ONE ERROR THAT MATTERS
------------------------------------------
aruco_pose publishes the marker centre in the CAMERA BODY frame: +x right
in the image, +y up in the image, +z up (so a marker below has negative
z). With the camera mounted image-up towards the NOSE and image-right to
the vehicle's RIGHT, that becomes body FRD:

    forward = y      right = x      down = -z

and the whole vector is then rotated into NED by the vehicle's ATTITUDE
QUATERNION, not just its heading. Using the full quaternion is what makes
this tilt-compensated: the camera rolls and pitches with the airframe, and
at 2 m a 10 degree pitch is a 0.35 m phantom lateral offset. Worse, it is
correlated with motion -- you pitch in order to move -- so uncorrected it
becomes an oscillation. Rotating the full 3D vector by the attitude
removes it exactly.

If a sign in that mapping is wrong the vehicle flies AWAY from the marker
and keeps accelerating, because every new measurement says "further".
That is what mode:=bench exists to rule out. Do not skip it.

THE HARDWARE UNDER THIS, AFTER THE 2026 SWAP
---------------------------------------------
The airframe went from a ZED + ARK Flow to a RealSense D435i + PMW3901 +
Benewake TFmini Plus. NEITHER HALF OF THAT REACHES THIS FILE, and it is
worth being explicit about why, because "the camera changed" is the kind
of thing that gets a precision-landing node re-tuned for no reason:

  * The camera here was never the ZED. aruco_pose opens the DOWN-FACING
    USB camera directly with cv2.VideoCapture -- see its header -- so the
    D435i swap changes nothing in this chain. If you ever do point this
    at the D435i, note that its colour stream is 70x43 deg where the old
    down-camera is 78 deg, which SHRINKS the capture basket in the table
    in the README and would need blind_commit_altitude re-measured.

  * The flow sensor is not read here either. flow_is_healthy() lives in
    the parent and is written against EKF2's flags and
    VehicleLocalPosition, never a sensor topic, so PMW3901-vs-ARK-Flow is
    a PX4 parameter question (EKF2_OF_CTRL, EKF2_OF_QMIN) and not a code
    one.

  * The rangefinder IS now a separate device (the ARK Flow had one built
    in). It is the same fused rangefinder as far as this node is
    concerned, but it is a new independent way for the flight to be
    un-armable, and it is the one the paragraph below depends on
    absolutely.

The one thing genuinely worth re-checking on the bench after the swap is
the PMW3901's noise floor against ALIGN_TOLERANCE: the alignment is only
ever as tight as the x/y estimate the correction is written in, and a
15 cm tolerance on a noisier flow sensor may simply never settle. If
mode:=align keeps timing out just outside tolerance, that is what it is,
and the answer is a larger align_tolerance, not a larger align_gain.

HEIGHT COMES FROM THE LIDAR, NEVER FROM THE MARKER
---------------------------------------------------
solvePnP's z is scaled by whatever error hfov_deg carries. x and y are
not -- the inflated range and deflated bearing cancel. So this node takes
only x and y from the vision, and every altitude decision stays on the
rangefinder the parent class already gates arming on -- the TFmini Plus
now, on its own serial port, rather than the ARK Flow's integrated one.

THE BLIND LAST METRE
--------------------
An 0.80 m marker under a 78 degree lens no longer fits in the frame below
about 0.66 m (4:3) or 0.88 m (16:9), and higher than that when it is
off-centre. There is no nested inner marker, so the last stretch of the
descent is open loop on the last held point. Two consequences:

  * alignment must be FINISHED by roughly 1 m, which is why the align
    stage runs at takeoff_altitude (2 m is a good choice) and not on the
    way down;
  * the x/y hold is deliberately KEPT through the descent, unlike the
    parent's _begin_landing which drops to zero-velocity hold. Ten seconds
    of unheld descent from 2 m would drift further than the tolerance we
    just worked to achieve. It still falls back to zero-velocity hold the
    moment the flow stops being healthy, which is the parent's own gate.

Keys:  q -> abort into a controlled descent.  k -> force-disarm.
"""

import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleAttitude, VehicleStatus

from drone_testing.offboard_sequence import OffboardSequence


def quat_rotate(q, v):
    """Rotate a 3-vector by a (w, x, y, z) quaternion.

    Deliberately a local copy rather than an import from the VIO bridge
    module: this node must not fail to load because of an unrelated
    module's dependencies, and the vision-odometry path is not part of
    this flight.
    """
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def body_words(forward, right):
    """'FORWARD 0.20 m, RIGHT 0.30 m' -- the sign check a human can read."""
    return (f"{'FORWARD' if forward >= 0 else 'BACK'} {abs(forward):.2f} m, "
            f"{'RIGHT' if right >= 0 else 'LEFT'} {abs(right):.2f} m")


class PrecisionLand(OffboardSequence):

    BENCH = "BENCH"
    SEARCH = "SEARCH"
    ALIGN = "ALIGN"
    ALIGNED_HOLD = "ALIGNED_HOLD"

    MODE = 'bench'              # bench | inspect | align -- see the header

    # ---- alignment --------------------------------------------------------
    ALIGN_TOLERANCE = 0.15      # m radius that counts as "over the marker"
    ALIGN_SETTLE_SECONDS = 1.5  # time inside that radius before we believe it.
                                # A single sample inside 15 cm is corner noise,
                                # not an arrival.
    ALIGN_GAIN = 0.6            # fraction of the measured offset commanded per
                                # cycle. < 1 guarantees monotone convergence.
    ALIGN_RELEASE = 1.5         # multiple of the tolerance at which a settled
                                # vehicle goes back to ALIGN. Hysteresis, so it
                                # does not flap on the boundary.
    ALIGNED_HOLD_SECONDS = 10.0 # station keeping over the marker before landing
    LAND_AFTER_ALIGN = True

    # ---- the marker feed --------------------------------------------------
    MARKER_MAX_AGE = 0.5        # s. Older than this is not evidence of
                                # anything: a dead camera goes quiet, and quiet
                                # must read as "no marker", never as a lock.
    MARKER_LOST_SECONDS = 5.0   # s without a marker in ALIGN before giving up
                                # and going back to SEARCH

    # ---- timeouts ---------------------------------------------------------
    SEARCH_SECONDS = 20.0       # s hovering and looking before we give up
    ALIGN_TIMEOUT = 45.0        # s trying to centre before we give up
    FLIGHT_SECONDS = 120.0      # s from the START OF THE CLIMB to a forced
                                # descent, whatever else is happening. The
                                # backstop that outranks every stage.
    ON_FAIL = 'land'            # land | hold -- what a timeout does

    # ---- descent ----------------------------------------------------------
    PRECISION_DESCENT = True    # keep the aligned x/y hold through the descent
    BLIND_COMMIT_ALTITUDE = 1.0 # m. Below this the marker is expected to leave
                                # the frame; logged so the ulog says where the
                                # closed loop ended. MEASURE THIS ON THE BENCH:
                                # walk the airframe down over the marker and
                                # note where /aruco/detected goes false.

    def __init__(self):
        super().__init__('precision_land')

        self.MODE = str(self.declare_parameter(
            'mode', self.MODE).value).strip().lower()
        if self.MODE not in ('bench', 'inspect', 'align'):
            self.get_logger().error(
                f"Unknown mode '{self.MODE}'; expected bench | inspect | align. "
                "Falling back to 'bench', which commands nothing.")
            self.MODE = 'bench'

        self.ALIGN_TOLERANCE = float(self._declare_number(
            'align_tolerance', self.ALIGN_TOLERANCE))
        self.ALIGN_SETTLE_SECONDS = float(self._declare_number(
            'align_settle_seconds', self.ALIGN_SETTLE_SECONDS))
        self.ALIGN_GAIN = float(self._declare_number(
            'align_gain', self.ALIGN_GAIN))
        self.ALIGNED_HOLD_SECONDS = float(self._declare_number(
            'aligned_hold_seconds', self.ALIGNED_HOLD_SECONDS))
        self.LAND_AFTER_ALIGN = bool(self.declare_parameter(
            'land_after_align', self.LAND_AFTER_ALIGN).value)
        self.MARKER_MAX_AGE = float(self._declare_number(
            'marker_max_age', self.MARKER_MAX_AGE))
        self.MARKER_LOST_SECONDS = float(self._declare_number(
            'marker_lost_seconds', self.MARKER_LOST_SECONDS))
        self.SEARCH_SECONDS = float(self._declare_number(
            'search_seconds', self.SEARCH_SECONDS))
        self.ALIGN_TIMEOUT = float(self._declare_number(
            'align_timeout', self.ALIGN_TIMEOUT))
        self.FLIGHT_SECONDS = float(self._declare_number(
            'flight_seconds', self.FLIGHT_SECONDS))
        self.PRECISION_DESCENT = bool(self.declare_parameter(
            'precision_descent', self.PRECISION_DESCENT).value)
        self.BLIND_COMMIT_ALTITUDE = float(self._declare_number(
            'blind_commit_altitude', self.BLIND_COMMIT_ALTITUDE))
        self.ON_FAIL = str(self.declare_parameter(
            'on_fail', self.ON_FAIL).value).strip().lower()
        if self.ON_FAIL not in ('land', 'hold'):
            self.ON_FAIL = 'land'

        if self.ALIGN_GAIN <= 0.0 or self.ALIGN_GAIN > 1.0:
            self.get_logger().error(
                f"align_gain {self.ALIGN_GAIN} is outside (0, 1]; clamping to 0.6. "
                "Above 1 the loop overshoots and can diverge.")
            self.ALIGN_GAIN = 0.6

        # ---- the marker feed ----
        self.marker_detected = False
        self.marker_point = None        # (x, y, z) camera body frame
        self.marker_time = None         # monotonic, when it arrived
        self.marker_ever_seen = False
        self.marker_info = ''
        self.marker_lost_since = None

        self.create_subscription(Bool, '/aruco/detected',
                                 self.marker_detected_callback, 10)
        self.create_subscription(PointStamped, '/aruco/point',
                                 self.marker_point_callback, 10)
        self.create_subscription(String, '/aruco/info',
                                 self.marker_info_callback, 10)

        # ---- attitude ----
        # The full quaternion, not just heading: this is what makes the marker
        # vector tilt-compensated. Both topic names, same reason the parent
        # subscribes to both land-detector names -- it is _v1 on some builds.
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

        self.align_in_band_since = None
        self.flight_start = None
        self.aligned_error = None
        self.last_error = None
        self._warned_no_attitude = False
        self._warned_blind = False

        if self.MODE == 'bench':
            # Completely inert: no heartbeat, no setpoints, no commands. The
            # parent's timer still runs, but timer_callback below sends it
            # straight to the bench handler and never touches the publishers.
            self.stream_setpoints = False
            self.current_stage = self.BENCH
            self.get_logger().warning(
                "BENCH MODE. Nothing will be armed and nothing will be "
                "published to PX4. Put the marker under the camera and check "
                "the FORWARD/BACK and RIGHT/LEFT words below against reality: "
                "move the marker to the drone's RIGHT and it must say RIGHT. "
                "If any axis is inverted, fix it before flying -- a sign error "
                "flies the aircraft away from the pad, accelerating.")
            return

        self.get_logger().warning(
            f"Precision landing, mode '{self.MODE}': climb "
            f"{self.TAKEOFF_ALTITUDE:.2f} m, hold {self.HOLD_SECONDS:.0f} s, "
            f"search {self.SEARCH_SECONDS:.0f} s, align to "
            f"{self.ALIGN_TOLERANCE * 100:.0f} cm at gain {self.ALIGN_GAIN:.2f}, "
            f"hold {self.ALIGNED_HOLD_SECONDS:.0f} s, then "
            f"{'LAND' if self.LAND_AFTER_ALIGN else 'stay up'}. Hard descent at "
            f"{self.FLIGHT_SECONDS:.0f} s. Press q to abort into a descent, "
            "k to force-disarm.")
        if self.MODE == 'inspect':
            self.get_logger().warning(
                "INSPECT MODE: the target will be computed and logged but NO "
                "lateral setpoint will be published. The vehicle will hold.")

    # ------------------------------------------------------------------ subs

    def marker_detected_callback(self, msg):
        self.marker_detected = bool(msg.data)
        self.marker_ever_seen = True

    def marker_point_callback(self, msg):
        self.marker_point = (float(msg.point.x), float(msg.point.y),
                             float(msg.point.z))
        self.marker_time = time.monotonic()

    def marker_info_callback(self, msg):
        self.marker_info = msg.data

    def attitude_callback(self, msg):
        q = [float(v) for v in msg.q]
        if len(q) == 4 and all(math.isfinite(v) for v in q):
            n = math.sqrt(sum(v * v for v in q))
            if n > 1e-6:
                self.attitude_q = [v / n for v in q]
                self.attitude_time = time.monotonic()

    # ------------------------------------------------------------ the marker

    def marker_is_fresh(self):
        """A live, debounced detection. Silence reads as 'no marker'."""
        if not self.marker_detected or self.marker_point is None:
            return False
        if self.marker_time is None:
            return False
        return time.monotonic() - self.marker_time <= self.MARKER_MAX_AGE

    def marker_body(self):
        """(forward, right, down) of the marker relative to the vehicle.

        The raw camera vector re-labelled for the mounting this package
        assumes: image up is the nose, image right is the vehicle's right.
        NOT tilt-compensated -- it is what the camera sees, which is the
        right thing for a human sign check on a level bench and the wrong
        thing to fly on. marker_offset_ned() is the one that flies.
        """
        if self.marker_point is None:
            return None
        x, y, z = self.marker_point
        return (y, x, -z)

    def marker_offset_ned(self):
        """(north, east, down) from the vehicle to the marker, or None.

        The full attitude quaternion is applied, so the roll and pitch the
        airframe is carrying to make the correction do not themselves appear
        as extra offset. Without attitude there is no honest answer, and we
        deliberately do NOT fall back to a heading-only rotation: that would
        silently reintroduce the tilt error it is here to remove.
        """
        if not self.marker_is_fresh():
            return None
        if self.attitude_q is None:
            if not self._warned_no_attitude:
                self._warned_no_attitude = True
                self.get_logger().error(
                    "No VehicleAttitude is being published: the marker vector "
                    "cannot be tilt-compensated, so it will not be used. Check "
                    "that vehicle_attitude is in the PX4 DDS topic list.")
            return None
        return quat_rotate(self.attitude_q, self.marker_body())

    def marker_summary(self):
        if not self.marker_ever_seen:
            return "no /aruco/detected messages -- is aruco_pose running?"
        if self.marker_time is None:
            return "no marker seen yet"
        if time.monotonic() - self.marker_time > self.MARKER_MAX_AGE:
            return "marker not visible"
        return "marker IN SIGHT" if self.marker_detected else "marker unconfirmed"

    # ------------------------------------------------------------- the clock

    def flight_time(self):
        if self.flight_start is None:
            return 0.0
        return time.monotonic() - self.flight_start

    def _check_flight_clock(self):
        """Descend at flight_seconds, from whichever airborne stage we are in."""
        if self.flight_start is None:
            return False
        if self.current_stage not in (self.TAKEOFF, self.HOLD, self.SEARCH,
                                      self.ALIGN, self.ALIGNED_HOLD):
            return False
        if self.flight_time() < self.FLIGHT_SECONDS:
            return False
        self._begin_landing(
            f"{self.FLIGHT_SECONDS:.0f} s airborne (stage {self.current_stage})")
        return True

    # -------------------------------------------------------- state machine

    def timer_callback(self):
        if self.MODE == 'bench':
            self._handle_bench()
            return

        # The clock outranks every stage handler, so a stuck stage cannot
        # postpone the landing.
        if self._check_flight_clock():
            return

        if self.current_stage not in (self.SEARCH, self.ALIGN, self.ALIGNED_HOLD):
            super().timer_callback()
            return

        # The same preamble the parent runs. Getting this wrong is how the
        # offboard heartbeat stops and PX4 takes the aircraft.
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

        {self.SEARCH: self._handle_search,
         self.ALIGN: self._handle_align,
         self.ALIGNED_HOLD: self._handle_aligned_hold}[self.current_stage]()

    # -------------------------------------------------------------- bench

    def _handle_bench(self):
        body = self.marker_body()
        if not self.marker_is_fresh() or body is None:
            self.get_logger().info(
                f"BENCH: {self.marker_summary()}.", throttle_duration_sec=1.0)
            return

        fwd, right, down = body
        line = (f"BENCH: marker is {body_words(fwd, right)}, "
                f"{down:.2f} m below  ->  the drone would move "
                f"{body_words(fwd, right)}")
        ned = self.marker_offset_ned()
        if ned is not None:
            lp = self.local_position
            hdg = ('n/a' if lp is None
                   else f"{math.degrees(lp.heading):+.0f} deg")
            line += f"  |  NED delta N={ned[0]:+.2f} E={ned[1]:+.2f} (hdg {hdg})"
        self.get_logger().info(line, throttle_duration_sec=1.0)

    # ------------------------------------------------------------- takeoff

    def _handle_takeoff(self):
        if self.flight_start is None:
            self.flight_start = time.monotonic()
            self.get_logger().warning(
                f"Flight clock started: forced descent in "
                f"{self.FLIGHT_SECONDS:.0f} s.")
        super()._handle_takeoff()

    def _handle_hold(self):
        """Settle at altitude, then start looking.

        If the marker is already underneath at the end of the hold there is
        nothing to search for, so go straight to aligning on it.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()

        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            if self.marker_offset_ned() is not None:
                self.get_logger().warning(
                    "Marker already in sight at the end of the hold.")
                self._enter_stage(self.ALIGN)
            else:
                self.get_logger().warning(
                    f"Searching for the marker for up to "
                    f"{self.SEARCH_SECONDS:.0f} s.")
                self._enter_stage(self.SEARCH)
            return

        self.get_logger().info(
            f"Holding, {remaining:.1f} s to the search... ({self.marker_summary()})",
            throttle_duration_sec=1.0)
        self.log_flight_state()

    # -------------------------------------------------------------- search

    def _handle_search(self):
        """Hold station and watch.

        Note this does NOT sweep the yaw the way window_scan does. That node
        has a forward camera, where a yaw sweeps new ground; this one looks
        straight down, where a yaw rotates the footprint but barely changes
        which patch of floor is in it. Turning would cost tracking quality
        and buy almost no coverage, so the search is stationary and the
        capture basket is simply the footprint at this altitude.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        if self.marker_offset_ned() is not None:
            self.get_logger().warning("Marker found. Aligning.")
            self.marker_lost_since = None
            self.align_in_band_since = None
            self._enter_stage(self.ALIGN)
            return

        remaining = self.SEARCH_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._give_up("no marker found in the search window")
            return

        self.get_logger().info(
            f"Searching, {remaining:.1f} s left. {self.marker_summary()}.",
            throttle_duration_sec=1.0)

    # --------------------------------------------------------------- align

    def _handle_align(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        offset = self.marker_offset_ned()
        if offset is None:
            # Freeze the carrot where it is. Walking on towards a target
            # derived from a measurement we no longer have is exactly the
            # coasting failure a position setpoint is chosen to avoid.
            self.moving = False
            if self.marker_lost_since is None:
                self.marker_lost_since = time.monotonic()
                self.get_logger().warning(
                    f"Marker lost mid-align ({self.marker_summary()}); holding "
                    "position.")
            elif time.monotonic() - self.marker_lost_since >= self.MARKER_LOST_SECONDS:
                self.get_logger().warning("Marker gone. Back to searching.")
                self._enter_stage(self.SEARCH)
            self.align_in_band_since = None
            self._check_align_timeout()
            return

        self.marker_lost_since = None
        north, east = offset[0], offset[1]
        error = math.hypot(north, east)
        self.last_error = error
        body = self.marker_body()

        # Arrival first, so a vehicle that is already centred does not get one
        # more nudge before being allowed to settle.
        if error <= self.ALIGN_TOLERANCE:
            if self.align_in_band_since is None:
                self.align_in_band_since = time.monotonic()
            elif (time.monotonic() - self.align_in_band_since
                    >= self.ALIGN_SETTLE_SECONDS):
                self._on_aligned(error)
                return
        else:
            self.align_in_band_since = None

        if self.MODE == 'inspect':
            lp = self.local_position
            self.get_logger().info(
                f"INSPECT: marker {body_words(body[0], body[1])} | err "
                f"{error:.2f} m | would move from "
                f"({lp.x:+.2f}, {lp.y:+.2f}) to "
                f"({lp.x + self.ALIGN_GAIN * north:+.2f}, "
                f"{lp.y + self.ALIGN_GAIN * east:+.2f}) NED. "
                "Nothing published.",
                throttle_duration_sec=1.0)
        elif not self._drive_towards(north, east):
            self.get_logger().info(
                "Waiting for a flow-healthy x/y hold before correcting...",
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(
                f"Aligning: marker {body_words(body[0], body[1])}, "
                f"err {error:.2f} m (want {self.ALIGN_TOLERANCE:.2f}).",
                throttle_duration_sec=1.0)

        self._check_align_timeout()

    def _drive_towards(self, north, east):
        """Point the parent's x/y carrot at the marker. Returns False if we
        have no latched lateral estimate to express a target in.

        Nothing here talks to PX4 directly: setting move_target_* and moving
        is precisely the interface _step_xy_ramp() consumes, so the target is
        walked at MOVE_SPEED, capped at MOVE_LEASH ahead of the measured
        position, and shifted automatically on an EKF2 lateral reset.
        """
        if not self.hold_xy or self.local_position is None:
            return False
        lp = self.local_position
        self.move_target_x = lp.x + self.ALIGN_GAIN * north
        self.move_target_y = lp.y + self.ALIGN_GAIN * east
        self.moving = True
        return True

    def _check_align_timeout(self):
        if self._in_stage_for() > self.ALIGN_TIMEOUT:
            self._give_up(
                f"could not centre within {self.ALIGN_TIMEOUT:.0f} s"
                + ('' if self.last_error is None
                   else f" (best {self.last_error:.2f} m)"))

    def _on_aligned(self, error):
        """Stop correcting and park.

        The carrot is released and the hold point is pinned to where the
        vehicle actually is. Inside the tolerance there is nothing left to
        win by chasing, and continuing to re-command on every frame would
        only be injecting corner noise into the position setpoint.
        """
        self.moving = False
        self.aligned_error = error
        lp = self.local_position
        if self.hold_xy and lp is not None:
            self.hold_x = lp.x
            self.hold_y = lp.y
        self.get_logger().warning(
            f"ALIGNED to {error * 100:.0f} cm. Holding "
            f"{self.ALIGNED_HOLD_SECONDS:.0f} s, then "
            f"{'landing' if self.LAND_AFTER_ALIGN else 'staying up'}.")
        self._enter_stage(self.ALIGNED_HOLD)

    # -------------------------------------------------------- aligned hold

    def _handle_aligned_hold(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        # Deadband, with hysteresis. Drift back outside the band earns another
        # correction; noise inside it does not.
        offset = self.marker_offset_ned()
        if offset is not None:
            error = math.hypot(offset[0], offset[1])
            self.last_error = error
            if error > self.ALIGN_TOLERANCE * self.ALIGN_RELEASE:
                self.get_logger().warning(
                    f"Drifted to {error:.2f} m off the marker; correcting again.")
                self.align_in_band_since = None
                self._enter_stage(self.ALIGN)
                return

        remaining = self.ALIGNED_HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            if self.LAND_AFTER_ALIGN:
                self._begin_precision_landing()
            else:
                self.get_logger().info(
                    "Aligned and holding (land_after_align is false). "
                    "q to descend, k to force-disarm.",
                    throttle_duration_sec=5.0)
            return

        self.get_logger().info(
            f"Aligned, holding {remaining:.1f} s. {self.marker_summary()}"
            + ('' if self.last_error is None else f", err {self.last_error:.2f} m"),
            throttle_duration_sec=1.0)

    # -------------------------------------------------------------- landing

    def _begin_precision_landing(self):
        """Descend WITHOUT throwing away the alignment.

        The parent's _begin_landing drops to a zero-velocity hold, which is
        right for a generic flight and wrong here: from 2 m at land_speed
        0.15 that is over ten seconds of unheld descent, and flow bias alone
        would drift further than the tolerance we just achieved. So the
        latched point is restored immediately afterwards.

        Every OTHER caller of _begin_landing -- operator abort, a dead height
        estimate, a lost rangefinder, an overshoot, the flight clock -- keeps
        the inherited behaviour untouched. Those are emergencies, and in an
        emergency "do not translate" is the correct horizontal command.
        """
        hold_x, hold_y, had_hold = self.hold_x, self.hold_y, self.hold_xy
        self._begin_landing(
            "aligned on the marker"
            + ('' if self.aligned_error is None
               else f" to {self.aligned_error * 100:.0f} cm"))
        if had_hold and self.PRECISION_DESCENT:
            self.hold_x, self.hold_y = hold_x, hold_y
            self.hold_xy = True
            self.get_logger().warning(
                f"Precision descent: holding ({hold_x:.2f}, {hold_y:.2f}) NED "
                f"all the way down. The marker is expected to leave the frame "
                f"below about {self.BLIND_COMMIT_ALTITUDE:.2f} m -- everything "
                "under that is open loop on this point.")

    def _handle_landing(self):
        # The aligned hold is only worth keeping while the lateral estimate
        # underneath it is still being corrected. The parent never re-checks
        # during a descent because it has already dropped to velocity hold;
        # since we did not, we have to.
        if self.hold_xy and not self.flow_is_healthy():
            self.hold_xy = False
            self.get_logger().warning(
                "Flow no longer healthy: dropping the aligned point and "
                "finishing the descent on zero-velocity hold.")

        alt = self.relative_altitude()
        if (not self._warned_blind and alt is not None
                and alt <= self.BLIND_COMMIT_ALTITUDE):
            self._warned_blind = True
            self.get_logger().warning(
                f"Below {self.BLIND_COMMIT_ALTITUDE:.2f} m: the marker is out "
                "of frame from here. Descent is open loop.")

        super()._handle_landing()

    def _give_up(self, reason):
        if self.ON_FAIL == 'hold':
            self.get_logger().error(
                f"{reason}. on_fail is 'hold': staying at altitude. "
                "q to descend, k to force-disarm.")
            self.moving = False
            return
        self._begin_landing(reason)

    # --------------------------------------------------------------- status

    def publish_status(self):
        """stage|armed|altitude|flow|detail -- the format the LCD node reads."""
        if self.current_stage not in (self.SEARCH, self.ALIGN, self.ALIGNED_HOLD):
            super().publish_status()
            return

        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        left = max(0.0, self.FLIGHT_SECONDS - self.flight_time())

        if self.current_stage == self.SEARCH:
            detail = f"srch {left:.0f}s"
        elif self.current_stage == self.ALIGN:
            err = '--' if self.last_error is None else f"{self.last_error:.2f}"
            detail = f"err{err} {left:.0f}s"
        else:
            remaining = max(0.0, self.ALIGNED_HOLD_SECONDS - self._in_stage_for())
            detail = f"algn {remaining:.0f}s"

        msg = String()
        msg.data = "|".join([
            self.current_stage,
            'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('FLO' if self.flow_is_healthy() else '---'),
            detail,
        ])
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PrecisionLand()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
