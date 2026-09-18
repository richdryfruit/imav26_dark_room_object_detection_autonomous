"""
Takeoff -> yaw sweep -> lock onto the window -> land after a fixed time.

The flight:

    arm -> sit on the ground -> climb to takeoff_altitude -> hold -> sweep
    the nose left and right through a 90 degree arc centred on the heading
    it took off with, until window_detect reports a window -> stop yawing,
    lock the position, and hold it -> land flight_seconds after the climb
    started.

Everything about arming, the climb, the estimator health gates, the
descent and the touchdown detection is inherited unchanged from
offboard_sequence.OffboardSequence -- this node only replaces what happens
between the post-takeoff hold and the landing. That is deliberate: the
parts that can hurt somebody have already been flown, and forking them to
add a yaw sweep would mean maintaining two copies of them.

    q -> abort into a controlled descent.   k -> force-disarm.

THE SWEEP
    From the takeoff heading the nose goes half a span towards the side the
    window is expected on (scan_direction, right by default), then a full
    span back the other way, then a full span again, and so on, so the whole
    arc is covered every leg. It is walked as a ramped yaw setpoint by the
    inherited yaw ramp, which is leashed to the measured heading: the
    vehicle is never asked to spin faster than it is actually turning.

    Slow is the point, and the sweep therefore has its own rate --
    scan_yaw_rate, ~3 deg/s, not the ~20 deg/s the rest of the flight yaws
    at. Two reasons. A fast yaw smears the optical flow the position hold is
    standing on. And the sweep is a SEARCH: the detector needs several
    consecutive frames on a window before it will call it, and at 20 deg/s a
    window can cross the frame in fewer frames than that, so the aircraft
    sweeps past a window it technically saw. The rate is restored to the
    normal one the moment the window is locked, so the approach that follows
    is not slowed down by it.

THE LOCK
    "Detected" means window_detect's DEBOUNCED /window_detected, which is
    already several consecutive frames, and this node additionally
    requires it to have been continuously true for detect_seconds and to
    be no older than DETECT_MAX_AGE. A camera that dies goes quiet, and
    quiet is treated as "no window", never as a lock.

    On the lock the commanded yaw is frozen at the heading the airframe
    actually has (not the one being commanded, which may be a few degrees
    ahead of it) and the x/y hold point is re-latched to where the vehicle
    is right now, so it parks looking at the window.

    Losing the window afterwards does NOT restart the sweep by default --
    the vehicle stays where it is. Pass relock_on_loss:=true if you would
    rather it goes back to scanning.

THE 40 SECONDS
    flight_seconds is measured from the START OF THE CLIMB, not from
    arming and not from reaching altitude, and it fires from every
    airborne stage. Whatever else is or is not happening -- window found,
    never found, still climbing -- the vehicle starts its descent at that
    mark. The descent itself takes as long as it takes on top of that.
"""

import math
import time

import rclpy
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleStatus

from drone_testing.offboard_sequence import OffboardSequence, spin_node, wrap_pi


class WindowScan(OffboardSequence):

    SCAN = "SCAN"
    LOCK = "LOCK"

    # ---- the sweep --------------------------------------------------------
    SCAN_SPAN = math.radians(20.0)   # total arc, centred on the takeoff heading
    SCAN_LEG_TIMEOUT_MARGIN = 6.0    # s added to the theoretical leg duration
                                     # before a stuck leg is abandoned and the
                                     # sweep turns around anyway
    # The sweep gets its OWN yaw rate, slower than the one the rest of the
    # flight uses. Everywhere else a yaw is a manoeuvre to be got over with;
    # here it is a search, and the search is rate-limited by the camera and the
    # detector, not by the airframe. At the inherited 20 deg/s a 1 m window at
    # 3 m crosses the frame in well under a second, which is fewer frames than
    # window_detect's debounce needs to call it -- so the aircraft can sweep
    # straight past a window it technically saw. Slower also keeps the optical
    # flow clean, which is what the position hold is standing on.
    SCAN_YAW_RATE = 0.05             # rad/s (~3 deg/s) while sweeping
    SCAN_FIRST_DIRECTION = 'right'   # which way the first half-leg goes

    # ---- the window -------------------------------------------------------
    DETECT_TOPIC = 'window_detected'
    DETECT_SECONDS = 0.4     # s /window_detected must stay true before we act
                             # on it. On top of window_detect's own debounce.
    DETECT_MAX_AGE = 1.0     # s after which the last message is not evidence
                             # of anything. window_detect publishes at camera
                             # rate, so this is ~15 missed frames.

    # ---- the clock --------------------------------------------------------
    FLIGHT_SECONDS = 40.0    # s from the start of the climb to the descent

    def __init__(self, node_name='window_scan'):
        # node_name is a parameter for the same reason it is one on the base
        # class: window_traverse reuses this whole sweep-and-lock under its own
        # name rather than forking it.
        super().__init__(node_name)

        self.SCAN_SPAN = math.radians(float(self._declare_number(
            'scan_span_deg', math.degrees(self.SCAN_SPAN))))
        self.SCAN_YAW_RATE = float(self._declare_number(
            'scan_yaw_rate', self.SCAN_YAW_RATE))
        direction = str(self.declare_parameter(
            'scan_direction', self.SCAN_FIRST_DIRECTION).value).strip().lower()
        if direction not in ('right', 'left'):
            self.get_logger().warning(
                f"scan_direction '{direction}' is not 'right' or 'left'; "
                f"using '{self.SCAN_FIRST_DIRECTION}'.")
            direction = self.SCAN_FIRST_DIRECTION
        # +1 is clockwise seen from above, i.e. the camera swings to the right.
        self.scan_first_sign = 1.0 if direction == 'right' else -1.0
        self.FLIGHT_SECONDS = float(self._declare_number(
            'flight_seconds', self.FLIGHT_SECONDS))
        self.DETECT_SECONDS = float(self._declare_number(
            'detect_seconds', self.DETECT_SECONDS))
        self.relock_on_loss = bool(self.declare_parameter('relock_on_loss', False).value)
        detect_topic = str(self.declare_parameter('detect_topic', self.DETECT_TOPIC).value)

        # Detection input. Plain default QoS (reliable, depth 10): this is a
        # low-rate boolean, not a sensor stream, and window_detect publishes it
        # the same way.
        self.create_subscription(Bool, detect_topic, self.window_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.create_subscription(String, 'window_info', self.window_info_callback, 10,
                                 callback_group=self.sensor_cbg)

        self.window_flag = False        # last value received
        self.window_msg_time = 0.0      # when it arrived
        self.window_true_since = None   # when it last became true
        self.window_info = ''
        self.window_ever_seen = False   # any message at all on the topic

        # Where the sweep is. scan_center is the heading it swings about,
        # captured at the moment scanning starts.
        self.scan_center = None
        self.scan_leg = 0
        self.scan_leg_deadline = None
        self.cruise_yaw_rate = self.YAW_RATE  # restored when the sweep stops

        self.flight_start = None        # monotonic time the climb began
        self.locked_heading = None

        if self.__class__ is not WindowScan:
            return

        self.get_logger().warning(
            f"Window scan: climb {self.TAKEOFF_ALTITUDE:.2f} m, hold "
            f"{self.HOLD_SECONDS:.0f} s, then sweep +/-"
            f"{math.degrees(self.SCAN_SPAN) / 2:.0f} deg at "
            f"{math.degrees(self.YAW_RATE):.0f} deg/s looking for the window; "
            f"lock on it when found. Landing {self.FLIGHT_SECONDS:.0f} s after "
            "the climb starts, window or no window.")

    # ---------------------------------------------------------- EKF2 resets

    def _on_heading_reset(self, delta):
        """The base class has shifted the commanded yaw; shift ours too.

        scan_center and locked_heading are absolute NED headings latched
        before the reset, so they name the wrong direction afterwards for
        exactly the same reason yaw_setpoint did.
        """
        super()._on_heading_reset(delta)
        if self.scan_center is not None:
            self.scan_center = wrap_pi(self.scan_center + delta)
        if self.locked_heading is not None:
            self.locked_heading = wrap_pi(self.locked_heading + delta)

    # ------------------------------------------------------------------ subs

    def window_callback(self, msg):
        self.window_msg_time = time.monotonic()
        self.window_ever_seen = True
        if msg.data:
            if not self.window_flag:
                self.window_true_since = self.window_msg_time
        else:
            self.window_true_since = None
        self.window_flag = msg.data

    def window_info_callback(self, msg):
        self.window_info = msg.data

    def window_is_confirmed(self):
        """True only for a live, sustained detection.

        Three separate ways to say no: the topic has gone quiet, the flag is
        false, or it has not been true for long enough yet. A silent camera
        must read as "keep looking", never as "found it".
        """
        if not self.window_flag or self.window_true_since is None:
            return False
        now = time.monotonic()
        if now - self.window_msg_time > self.DETECT_MAX_AGE:
            return False
        return now - self.window_true_since >= self.DETECT_SECONDS

    def window_summary(self):
        if not self.window_ever_seen:
            return "no /window_detected messages -- is window_detect running?"
        if time.monotonic() - self.window_msg_time > self.DETECT_MAX_AGE:
            return "window_detect has gone quiet"
        return "window IN SIGHT" if self.window_flag else "no window"

    # ------------------------------------------------------------ the clock

    def flight_time(self):
        if self.flight_start is None:
            return 0.0
        return time.monotonic() - self.flight_start

    def _clock_stages(self):
        """The airborne stages the flight clock is allowed to land from.

        A method rather than a literal so a subclass that adds stages of its
        own -- window_traverse does -- can put them under the same clock
        without reimplementing it.
        """
        return (self.TAKEOFF, self.HOLD, self.SCAN, self.LOCK)

    def _check_flight_clock(self):
        """Land at flight_seconds, from whichever airborne stage we are in."""
        if self.flight_start is None:
            return False
        if self.current_stage not in self._clock_stages():
            return False
        if self.flight_time() < self.FLIGHT_SECONDS:
            return False
        self._begin_landing(
            f"{self.FLIGHT_SECONDS:.0f} s airborne "
            f"({'window locked' if self.current_stage == self.LOCK else 'no lock'})")
        return True

    # ------------------------------------------------------- state machine

    def timer_callback(self):
        # The clock outranks everything below it, including the stage handlers,
        # so a stage that is stuck cannot postpone the landing.
        if self._check_flight_clock():
            return

        if self.current_stage not in (self.SCAN, self.LOCK):
            # Every other stage is the parent's, unchanged.
            super().timer_callback()
            return

        # Same preamble the parent runs: heartbeat, setpoint, status, aborts.
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

        if self.current_stage == self.SCAN:
            self._handle_scan()
        else:
            self._handle_lock()

    def _handle_takeoff(self):
        # The flight clock starts with the climb, so it measures time in the
        # air rather than time spent waiting for a mode switch on the ground.
        if self.flight_start is None:
            self.flight_start = time.monotonic()
            self.get_logger().warning(
                f"Flight clock started: landing in {self.FLIGHT_SECONDS:.0f} s.")
        super()._handle_takeoff()

    def _handle_hold(self):
        """Settle at altitude, then start sweeping.

        If the window is already in front of us when the hold ends there is
        nothing to search for -- go straight to the lock rather than sweeping
        away from a window we can already see.
        """
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()

        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            if self.window_is_confirmed():
                self._lock_on_window("already in sight at the end of the hold")
            else:
                self._begin_scan()
            return

        self.get_logger().info(
            f"Holding, {remaining:.1f} s to the sweep... ({self.window_summary()})",
            throttle_duration_sec=1.0)
        self.log_flight_state()

    # ------------------------------------------------------------- scanning

    def _begin_scan(self):
        lp = self.local_position
        # Sweep about the heading we are actually holding now, not the one at
        # arming: the climb can leave a degree or two of yaw error, and the arc
        # should be centred on where the camera is really pointing.
        self.scan_center = self.yaw_setpoint if lp is None else lp.heading
        self.yaw_setpoint = self.scan_center
        self.yaw_remaining = 0.0
        self.scan_leg = 0
        self.scan_leg_deadline = None
        # Slow down for the search. cruise_yaw_rate was captured in __init__
        # and is NOT re-read here: a second pass through this function (a
        # relock_on_loss sweep) would otherwise save the scan rate as the
        # cruise rate and the flight would never yaw at flying speed again.
        self.YAW_RATE = self.SCAN_YAW_RATE
        self._enter_stage(self.SCAN)
        if self.SCAN_SPAN <= 0.0:
            # Sweep disabled (scan_span_deg = 0): the vehicle stares straight
            # ahead on the heading it climbed on and waits for the detector.
            # Point the airframe at the window before you arm.
            self.get_logger().warning(
                f"Searching without a sweep: holding "
                f"{math.degrees(self.scan_center):+.0f} deg and waiting for "
                "the window.")
            return
        self.get_logger().warning(
            f"Scanning: sweeping +/-{math.degrees(self.SCAN_SPAN) / 2:.0f} deg "
            f"about {math.degrees(self.scan_center):+.0f} deg at "
            f"{math.degrees(self.SCAN_YAW_RATE):.0f} deg/s, "
            f"{'right' if self.scan_first_sign > 0 else 'left'} first, "
            "looking for the window.")

    def _next_leg(self):
        """Aim the yaw ramp at the other end of the arc.

        The first leg is a half span, from the centre out to one edge; every
        leg after it is a full span, edge to edge. Written as a delta rather
        than an absolute target because that is what the inherited yaw ramp
        consumes, and it keeps the sign (which way we are turning) explicit.
        """
        half = self.SCAN_SPAN / 2.0
        if self.scan_leg == 0:
            # Out to the side the window is expected on first, so the common
            # case is found in a few degrees of turn instead of after a full
            # traverse of the far half of the arc.
            delta = half * self.scan_first_sign
        else:
            # Alternate: leg 1 goes the other way by a full span, leg 2 back,
            # and so on.
            sign = -self.scan_first_sign if self.scan_leg % 2 == 1 else self.scan_first_sign
            delta = self.SCAN_SPAN * sign
        self.scan_leg += 1
        self.yaw_remaining = delta

        duration = abs(delta) / max(self.YAW_RATE, 1e-3)
        self.scan_leg_deadline = time.monotonic() + duration + self.SCAN_LEG_TIMEOUT_MARGIN
        self.get_logger().info(
            f"Sweep leg {self.scan_leg}: {math.degrees(delta):+.0f} deg "
            f"(~{duration:.0f} s).")

    def _handle_scan(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        if self.window_is_confirmed():
            self._lock_on_window("window detected during the sweep")
            return

        if self.SCAN_SPAN <= 0.0:
            # No sweep: hold the heading and keep looking until the flight
            # clock runs out.
            self.get_logger().info(
                f"Searching on a fixed heading. {self.window_summary()}.",
                throttle_duration_sec=1.0)
            return

        if abs(self.yaw_remaining) < 1e-3:
            self._next_leg()
        elif self.scan_leg_deadline is not None and time.monotonic() > self.scan_leg_deadline:
            # The ramp is leashed to the measured heading, so a leg that runs
            # long means the airframe is not following. Turning around beats
            # pushing against whatever is holding it.
            self.get_logger().warning(
                f"Sweep leg {self.scan_leg} did not finish in time "
                f"({math.degrees(abs(self.yaw_remaining)):.0f} deg left); "
                "turning around.")
            self.yaw_remaining = 0.0
            if self.local_position is not None:
                self.yaw_setpoint = self.local_position.heading
            self._next_leg()

        left = math.degrees(abs(self.yaw_remaining))
        self.get_logger().info(
            f"Scanning: leg {self.scan_leg}, {left:.0f} deg of setpoint left. "
            f"{self.window_summary()}.",
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------- the lock

    def _lock_on_window(self, reason):
        """Stop the sweep and park here.

        Freezing at the MEASURED heading rather than the commanded one matters:
        the commanded yaw leads the airframe by up to YAW_LEASH, and locking on
        to it would make the vehicle keep turning past the window after we said
        stop.
        """
        lp = self.local_position
        self.yaw_remaining = 0.0
        self.YAW_RATE = self.cruise_yaw_rate
        if lp is not None:
            self.yaw_setpoint = wrap_pi(lp.heading)
            self.locked_heading = lp.heading
            # Re-latch the hold point to right here. It is already being held,
            # but the turn will have pushed it around a little.
            if self.hold_xy:
                self.hold_x = lp.x
                self.hold_y = lp.y

        self._enter_stage(self.LOCK)
        heading = ('unknown' if self.locked_heading is None
                   else f"{math.degrees(self.locked_heading):+.0f} deg")
        self.get_logger().warning(
            f"WINDOW LOCKED ({reason}). Yaw stopped at {heading}, holding "
            f"position. {self.window_info or 'no detail yet'}. Landing in "
            f"{max(0.0, self.FLIGHT_SECONDS - self.flight_time()):.0f} s.")

    def _handle_lock(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        if not self.window_is_confirmed():
            if self.relock_on_loss:
                self.get_logger().warning("Window lost: resuming the sweep.")
                self._begin_scan()
                return
            self.get_logger().warning(
                f"Window lost ({self.window_summary()}), but staying locked "
                "-- pass relock_on_loss:=true to sweep again.",
                throttle_duration_sec=2.0)

        self.get_logger().info(
            f"Locked, holding. {self.window_summary()}. "
            f"{max(0.0, self.FLIGHT_SECONDS - self.flight_time()):.0f} s to landing.",
            throttle_duration_sec=1.0)

    # ------------------------------------------------------------- status

    def publish_status(self):
        """stage|armed|altitude|flow|detail -- the format the LCD node reads.

        Overridden rather than extended because the parent's detail column
        talks about sequence steps, which this node does not have.
        """
        if self.current_stage not in (self.SCAN, self.LOCK, self.HOLD, self.TAKEOFF):
            super().publish_status()
            return

        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        left = max(0.0, self.FLIGHT_SECONDS - self.flight_time())

        if self.current_stage == self.SCAN:
            detail = f"scan{math.degrees(abs(self.yaw_remaining)):.0f} {left:.0f}s"
        elif self.current_stage == self.LOCK:
            hdg = 0.0 if self.locked_heading is None else math.degrees(self.locked_heading)
            detail = f"lock{hdg:+.0f} {left:.0f}s"
        elif self.current_stage == self.HOLD:
            detail = f"{max(0.0, self.HOLD_SECONDS - self._in_stage_for()):.0f}s"
        else:
            detail = f"tgt{self.commanded_altitude:.2f}"

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
    node = WindowScan()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
