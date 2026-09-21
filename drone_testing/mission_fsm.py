"""
THE WHOLE RUN, AS ONE CONTINUOUS FLIGHT, FROM ONE COMMAND.

    arm -> climb -> hold
      -> LEG 1  forward, slowly, up to 11 m, watching the down camera
                for the WINDOW marker; centre on it and hover
      -> the window: sweep, lock, line up, through it
      -> the dark room: box pattern, dolls detected, geotagged, counted
                        and shown on the TFT, in QGC and in the terminal
      -> relock on the window from inside, back out through it
      -> LEG 2  forward, up to 11 m, back to the WINDOW marker; centre
      -> LEG 3  LEFT, up to turn_distance, to the TURN marker; centre
      -> LEG 4  forward, up to 11 m, to the PAD marker; centre
      -> precision descent onto the pad -> disarm

    ros2 launch drone_testing mission_fsm.launch.py agent_only:=false \
        window_marker_id:=2 turn_marker_id:=3 pad_marker_id:=1

ONE FLIGHT, NOT FIVE
--------------------
All of the above happens in a SINGLE armed Offboard session. The aircraft
never touches the ground between segments, and that is a safety property,
not a flourish: README section 10.1 records that `cs_rng_kin_consistent` is
sticky -- EKF2 only updates it while `in_air` is true, so once it latches
false nothing on the ground clears it but a flight controller reboot. A
mission that lands and re-arms four times is a mission that can refuse to
take off again with the clock running.

THE LEGS END ON A MARKER, NOT ON A DISTANCE
--------------------------------------------
This is the thing to understand about this file. A leg is NOT "fly 11 m". It
is "fly in this direction, slowly, watching the down camera, and stop when
you see THIS marker -- and if you have gone 11 m without seeing it, stop
anyway". The distance is a LIMIT, not a target.

That matters because the legs are flown on optical flow, which is the least
referenced part of the flight: 11 m of dead reckoning accumulates error that
nothing corrects. The marker is the correction. Arriving "11 m along" means
arriving wherever the flow thinks 11 m is; arriving "over the marker" means
arriving over the marker, and every leg that ends on one hands the next leg
a position that is right rather than one that is merely old.

A leg that reaches its limit without ever seeing its marker is NOT an abort.
It holds where it is and the mission carries on -- the window sweep does not
need the marker, and neither does the leg after it. The one exception is the
final leg: no pad marker means there is no pad, so the aircraft lands where
it is rather than hovering until the battery decides for it.

FOUR MARKERS, FOUR IDS, AND WHY THAT IS LOAD BEARING
-----------------------------------------------------
    id 0    the takeoff pad. Not used by this node at all.
    id 1    the landing pad             (pad_marker_id)
    id ?    in front of the window      (window_marker_id)
    id ?    the left turn point         (turn_marker_id)

They must be DIFFERENT ids, and the flight depends on it. Leg 3 strafes left
off the window marker looking for the turn marker; if the two shared an id,
the window marker would still be under the camera as the strafe began and the
leg would "arrive" instantly without having moved. Leg 2 flies back to the
window marker specifically, not to whichever marker happens to be nearest.

aruco_pose publishes every marker it can see on /aruco/marker_points, one
message per marker, with the id in header.frame_id as "aruco:<id>". This node
subscribes to that and DISCARDS everything that is not the current leg's
target, so a fix from the wrong pad can never reach the control loop. Its
three original topics -- /aruco/detected, /aruco/point, /aruco/info -- are
untouched and still describe the single configured marker_id, which is what
precision_land reads.

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

and this class adds one primitive, used four times: a marker-terminated leg,
as the three stages CRUISE -> MARKER_ALIGN -> MARKER_HOLD.

WHY THE MARKER STAGES ARE WRITTEN OUT HERE RATHER THAN INHERITED
-----------------------------------------------------------------
PrecisionLand is the other child of OffboardSequence, so
`class MissionFSM(WindowRoomTraverse, PrecisionLand)` looks like it should
work, and the MRO is in fact legal. It does not work, for two reasons worth
writing down so nobody tries it again:

  * `ALIGN` means two different things. WindowTraverse.ALIGN == "ALIGN" is
    lining up in front of a window; PrecisionLand.ALIGN == "ALIGN" is
    centring over a marker. A stage name is a class attribute, and one object
    cannot hold two values for `self.ALIGN` -- whichever class is earlier in
    the MRO wins and the other branch's dispatch silently routes to the wrong
    handler.

  * four parameters are declared by both branches -- `align_tolerance`,
    `align_settle_seconds`, `align_timeout`, `flight_seconds` -- and the
    second `declare_parameter` of a name raises
    ParameterAlreadyDeclaredException at construction.

What IS reused from precision_land is the part that is genuinely subtle and
must not be retyped: the marker maths. `marker_offset_ned` applies the full
attitude quaternion, so the roll and pitch the airframe is carrying in order
to make a correction do not themselves read as more offset to correct. Those
helpers touch nothing but `self.marker_*` and `self.attitude_q`, so the
per-leg filter below writes the CURRENT TARGET's fix into exactly those
fields and the helpers are then called unbound --
`PrecisionLand.marker_offset_ned(self)` -- leaving one copy of that
arithmetic in the package, in the file that already flies.

WHERE THE MISSION IS SPLICED INTO THE INHERITED MACHINE
--------------------------------------------------------
Three overrides, each a fork the parent already had a branch for, and in two
of the three the behaviour being replaced is "land":

    _handle_hold        end of the settle at altitude. The parent goes to
                        SCAN. We fly leg 1 first, THEN let it.
    _handle_clear       far side of a traversal. Outbound, the parent lands.
                        We fly legs 2, 3 and 4 instead.
    _handle_landing     keep the aligned point through a precision descent.

Every other stage is the parent's, reached through the same call it always
was. If the traversal behaviour changes in window_traverse.py, it changes
here too, which is the point.

THE FLIGHT CLOCK
----------------
`flight_seconds` is the backstop that outranks every stage handler: when it
expires the vehicle descends from wherever it is, whatever it was doing. Two
traversals, a room pattern, FOUR navigation legs of up to 11 m each at
leg_speed, and three marker acquisitions do not fit in the room mission's
300 s, so the default here is 600. It is a BACKSTOP, NOT A SCHEDULE -- set it
from the battery, and leave margin for the descent it exists to guarantee.

Keys:  q -> abort into a controlled descent from wherever we are.
       k -> force-disarm immediately (motors cut, vehicle drops).
"""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude, VehicleStatus
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String

from drone_testing.offboard_sequence import DIRECTIONS, spin_node
from drone_testing.offboard_sequence_vio import OffboardSequenceVio
from drone_testing.room_lidar_scan import LidarRoomScan, wrap_pi
from drone_testing.precision_land import PrecisionLand, body_words
from drone_testing.window_room_traverse import WindowRoomTraverse


class Leg:
    """One navigation leg.

    `distance` is the LIMIT, not the target, whenever there is a marker: the
    leg ends on the marker if it appears and on the distance if it does not.

    `marker_id` None makes it a PURE DISTANCE move -- the two strafes that put
    the aircraft on the window axis and take it off again are exactly that,
    because there is no marker on the axis to aim at. A pure-distance leg
    watches for nothing and always ends on its distance.

    `required` says which ending is acceptable. Only the pad leg insists on
    its marker, because only the pad leg has nothing sensible to do without
    one.

    `nxt` is what runs when this leg finishes: another leg's name, 'window'
    (hand over to the sweep) or 'descend'. Naming the successor rather than
    taking the next index keeps the plan correct when the optional strafes
    are absent.
    """

    __slots__ = ('name', 'direction', 'distance', 'marker_id', 'required',
                 'nxt', 'retry')

    def __init__(self, name, direction, distance, marker_id, nxt,
                 required=False, retry=True):
        self.name = name
        self.direction = direction
        self.distance = distance
        self.marker_id = marker_id
        self.nxt = nxt
        self.required = required
        # `retry` = is an elevated second look worth the flight clock here?
        # Only where the marker is the ONLY landmark. The outbound leg has a
        # better fallback than any amount of climbing -- the window itself --
        # so it hands over to the window search instead.
        self.retry = retry

    def __str__(self):
        target = ('no marker, pure distance' if self.marker_id is None
                  else f"marker id {self.marker_id}")
        limit = 'up to ' if self.marker_id is not None else ''
        return (f"{self.name}: {self.direction} {limit}{self.distance:.1f} m, "
                f"{target}" + (" (required)" if self.required else ""))


class MissionFSM(WindowRoomTraverse, LidarRoomScan):
    """The room mission, with marker-terminated legs to it, away and to the pad."""

    # ---- the stages this class adds ---------------------------------------
    # CRUISE/MARKER_* rather than STEP/ALIGN/HOLD: see the module header. The
    # inherited chain already owns ALIGN and STEP, and a stage name is a class
    # attribute, not something a subclass can hold a second value for.
    CRUISE = "CRUISE"               # flying a leg, watching for its marker
    MARKER_ALIGN = "MARKER_ALIGN"   # closing the loop on the marker offset
    MARKER_HOLD = "MARKER_HOLD"     # settled over it
    ALT_CHANGE = "ALT_CHANGE"       # climbing or descending between phases
    WINDOW_BACKOFF = "WINDOW_BACKOFF"   # retreating to fit the aperture in frame
    MARKER_RETRY = "MARKER_RETRY"   # high and looking again for a missed marker
    TILE_SETTLE = "TILE_SETTLE"     # stabilising on the lidar before scanning
    TILE_MOVE = "TILE_MOVE"         # flying to a tile centre in the arena frame
    TILE_DWELL = "TILE_DWELL"       # stationary while the detector works
    TILE_RETURN = "TILE_RETURN"     # back to the window wall for the relock

    LEG_STAGES = (CRUISE, MARKER_ALIGN, MARKER_HOLD, ALT_CHANGE,
                  WINDOW_BACKOFF, MARKER_RETRY)
    TILE_STAGES = (TILE_SETTLE, TILE_MOVE, TILE_DWELL, TILE_RETURN)

    # ---- the markers ------------------------------------------------------
    # id 0 is the takeoff pad and is deliberately absent: this node never
    # looks for it. The three below MUST be distinct -- see the header.
    WINDOW_MARKER_ID = 2
    TURN_MARKER_ID = 3
    PAD_MARKER_ID = 1

    # ---- the legs ---------------------------------------------------------
    # ALL DIRECTIONS ARE IN THE TAKEOFF FRAME, not the aircraft's current
    # heading: DIRECTION_FRAME is 'home', so _reference_yaw() returns the yaw
    # held at ARMING and 'forward' means the way the aircraft was facing then,
    # for the whole flight.
    #
    # This is why the two legs home are BACKWARD. Coming out of the window the
    # airframe is physically facing back down the course -- it yawed 180 in
    # the room -- but that changes the HEADING, not the frame. Telling it
    # 'forward' out there would send it straight back into the room.
    # ---- the altitude schedule ------------------------------------------
    # Two heights, and the whole flight is at one or the other.
    #
    # CRUISE is where the ArUco markers are read: the down camera's basket is
    # +/- h*tan(39 deg), so height buys capture width, and 2.5 m gives about
    # +/-2.0 m of it. Everything outside the dark room is flown here.
    #
    # WINDOW is where the window is approached and flown. Lower, because the
    # aperture's height above the floor is what the traversal has to line up
    # with, and because the approach wants the window filling more of the
    # frame than a 2.5 m stand-off would leave it.
    #
    # The dark room itself is NOT on this schedule: window_altitude() derives
    # the traverse height from the measured window pose, which is the only
    # honest source for it, and the room pattern inherits that.
    CRUISE_ALTITUDE = 2.50      # m. Takeoff, every leg, and the landing.
    WINDOW_ALTITUDE = 1.85      # m. Dropped to on first sighting of the
                                # window marker; climbed back out of after
                                # the room, on sighting it again.
    ROOM_MODE = 'lidar'         # lidar | box. 'lidar' divides the room into
                                # tiles and flies their centres off the wall
                                # localizer's drift-free fix. 'box' restores
                                # the inherited dead-reckoned leg pattern.
    TILE_MOVE_TIMEOUT = 25.0    # s to reach one tile centre before skipping it
    ROOM_ALTITUDE = 1.85        # m the tile scan is flown at. NOT the
                                # traverse height: the traversal is lined up
                                # on the measured window pose and goes through
                                # wherever the aperture actually is, but once
                                # inside, the room is flown at a height of our
                                # choosing rather than the window's.
    ALT_CHANGE_TIMEOUT = 25.0   # s before a climb or descent is given up on

    # ---- the window search failsafe -------------------------------------
    # Normally the strafe puts the aircraft on the axis and the window is
    # simply there. This is the ladder for when it is not.
    WINDOW_SEARCH_SECONDS = 12.0    # s of looking before escalating a rung
    WINDOW_SWEEP_DEG = 30.0         # deg either side. Rung 1: yaw left,
                                    # centre and right, which is exactly what
                                    # SCAN already does with a non-zero span.
    WINDOW_BACKOFF_M = 0.30         # m straight back. Rung 2+: the commonest
                                    # reason for seeing nothing at all is
                                    # being too CLOSE for the aperture to fit
                                    # in frame, and backing off is the only
                                    # thing that fixes that.
    WINDOW_MAX_BACKOFFS = 2         # rungs of backoff before giving up.
                                    # Two, because the aperture already fits
                                    # in frame from the marker: a 0.60 m tall
                                    # window needs 0.76 m of depth and the
                                    # marker stands 1.00 m from the wall. The
                                    # backoff is a failsafe for a bad day,
                                    # not the normal path, so it does not
                                    # need a long ladder.

    # ---- the missed-marker failsafe -------------------------------------
    # A leg that runs out of distance without seeing its marker has almost
    # certainly drifted: 10 m on optical flow with nothing correcting it, and
    # the marker ends up just outside a basket that is only as wide as the
    # camera's footprint. So widen the footprint.
    #
    # The down camera is 78 deg HFOV, so the basket is +/- h*tan(39 deg):
    #
    #       1.75 m  ->  +/-1.42 m          2.50 m  ->  +/-2.02 m
    #       3.50 m  ->  +/-2.83 m
    #
    # Climbing from cruise to 3.50 m is a 40% wider swath for a few seconds
    # of flight clock, and it attacks exactly the failure being seen.
    MARKER_RETRY_ALTITUDE = 3.50    # m. The max height this mission flies at.
    MARKER_RETRY_SECONDS = 8.0      # s of looking from up there before moving
    MARKER_RETRY_DISTANCE = 4.0     # m retraced back along the leg, at the
                                    # retry altitude, still watching. Bounded
                                    # rather than the whole leg: the marker is
                                    # near the END of a leg that overshot, and
                                    # a full retrace costs the flight clock
                                    # twice over.
    MARKER_MAX_RETRIES = 0          # elevated searches per leg. ZERO by
                                    # instruction: a leg that reaches its
                                    # limit without its marker STOPS THERE
                                    # and moves on -- the roll stops at
                                    # turn_distance, and the pad leg lands.
                                    # The machinery below is kept and works;
                                    # set marker_max_retries:=1 to re-enable
                                    # the climb-and-look failsafe.

    OUTBOUND_DISTANCE = 9.3     # m FORWARD, arming point -> window marker
    WINDOW_OFFSET = 0.10        # m LEFT, window marker -> the window's axis.
                                # MEASURED on the course. 0 disables both
                                # strafes and approaches obliquely instead.
    WINDOW_OFFSET_DIRECTION = 'left'
    RETURN_DISTANCE = 2.0       # m BACKWARD, a SHORT search for the window
                                # marker. The exit ends outside_distance past
                                # the window plane -- set equal to the
                                # marker's 1.0 m, so the marker is directly
                                # under the aircraft on arrival. This is a
                                # margin, not a transit: an 11 m limit here
                                # would fly a missed marker back to the net.
    TURN_DISTANCE = 5.0         # m RIGHT (east), window marker -> turn marker
    PAD_DISTANCE = 9.3          # m BACKWARD, turn marker -> landing pad.
                                # This lands the aircraft level with the
                                # takeoff pad and turn_distance to its left.
    LEG_SPEED = 0.30            # m/s. Slow on purpose: the down camera has to
                                # have a chance to see a marker pass beneath
                                # it, and flow is the only thing measuring
                                # these translations.
    LEG_SETTLE_SECONDS = 2.0    # s of holding still at the end of a leg that
                                # ended on its DISTANCE rather than a marker
    LEG_LATCH_TIMEOUT = 20.0    # s waiting for a flow-healthy x/y latch
                                # before a leg is given up on
    LEG_TIMEOUT_MARGIN = 30.0   # s added to the ideal leg time before the
                                # leg is called stuck. distance/speed is the
                                # ideal; this is the allowance on top.

    # ---- centring on a marker --------------------------------------------
    MARKER_TOLERANCE = 0.15     # m radius that counts as "over the marker"
    MARKER_SETTLE_SECONDS = 1.5 # s inside that radius before we believe it.
                                # One sample inside 15 cm is corner noise,
                                # not an arrival.
    MARKER_GAIN = 0.6           # fraction of the measured offset commanded
                                # per cycle. < 1 is what makes the loop
                                # monotone instead of oscillatory.
    MARKER_RELEASE = 1.5        # multiple of the tolerance at which a settled
                                # vehicle goes back to aligning. Hysteresis,
                                # so it does not flap on the boundary.
    MARKER_HOLD_SECONDS = 4.0   # s station keeping once centred
    MARKER_ALIGN_TIMEOUT = 45.0 # s trying to centre before giving up
    MARKER_MAX_AGE = 0.5        # s. Older than this is not evidence: a dead
                                # camera goes quiet, and quiet must read as
                                # "no marker", never as a lock.
    MARKER_LOST_SECONDS = 5.0   # s without the marker while aligning before
                                # falling back to cruising

    # ---- the descent ------------------------------------------------------
    PRECISION_DESCENT = True    # keep the aligned x/y hold through the drop
    BLIND_COMMIT_ALTITUDE = 1.0 # m. Below this the marker leaves the frame.
                                # MEASURE IT on the bench; see the
                                # precision_land header.

    # ---- where the lateral estimate comes from ---------------------------
    # The legs between the markers are the longest unreferenced translations
    # in the flight, and on this airframe they are flown on RTAB-Map VIO, not
    # on optical flow. EKF2 is configured vision=x/y, lidar=height,
    # mag=heading, flow OFF -- so the inherited flow_is_healthy() would be
    # gating on a sensor that is not even being fused.
    LATERAL_SOURCE = 'vio'      # vio | flow. 'flow' restores the inherited
                                # ARK Flow predicate unchanged.
    VIO_ODOM_TOPIC = '/rtabmap/odom'
    VIO_MAX_AGE = 0.5           # s. Older than this is not a fix.
    VIO_MAX_COVARIANCE = 100.0  # rtabmap_odom publishes ~9999 on lost
                                # tracking rather than going silent.
    ALLOW_MISSING_VIO_STATUS = True   # no /rtabmap/odom at all -> fall back
                                      # to EKF2's own cs_ev_* opinion
    FLOW_SETTLE_SECONDS = 2.0   # s the estimate must be good before the x/y
                                # hold latches. Vision settles faster than
                                # flow and is valid on the ground.

    # Two traversals, a room pattern, four legs and three marker acquisitions.
    # The parent's 300 s does not begin to cover it. A BACKSTOP, not a plan.
    FLIGHT_SECONDS = 600.0

    def __init__(self):
        super().__init__()

        # ---- the markers ----
        self.WINDOW_MARKER_ID = int(self.declare_parameter(
            'window_marker_id', self.WINDOW_MARKER_ID).value)
        self.TURN_MARKER_ID = int(self.declare_parameter(
            'turn_marker_id', self.TURN_MARKER_ID).value)
        self.PAD_MARKER_ID = int(self.declare_parameter(
            'pad_marker_id', self.PAD_MARKER_ID).value)

        ids = [self.WINDOW_MARKER_ID, self.TURN_MARKER_ID, self.PAD_MARKER_ID]
        if len(set(ids)) != len(ids):
            # Fatal, not a warning. Two legs sharing an id means the strafe
            # off the window marker "arrives" at the turn marker without
            # moving, and the aircraft then flies the final leg from the wrong
            # place. A flight plan that cannot be flown must not be taken off
            # with. See the header.
            raise SystemExit(
                f"window_marker_id, turn_marker_id and pad_marker_id must all "
                f"differ; got {ids}. Two legs sharing a marker id means the "
                "second one ends the instant it starts, on the first one's "
                "marker.")

        # ---- the legs ----
        self.LATERAL_SOURCE = str(self.declare_parameter(
            'lateral_source', self.LATERAL_SOURCE).value).strip().lower()
        if self.LATERAL_SOURCE not in ('vio', 'flow'):
            raise SystemExit(
                f"lateral_source must be 'vio' or 'flow'; got "
                f"'{self.LATERAL_SOURCE}'.")
        self.VIO_ODOM_TOPIC = str(self.declare_parameter(
            'vio_odom_topic', self.VIO_ODOM_TOPIC).value)
        self.VIO_MAX_AGE = float(self._declare_number(
            'vio_max_age', self.VIO_MAX_AGE))
        self.VIO_MAX_COVARIANCE = float(self._declare_number(
            'vio_max_covariance', self.VIO_MAX_COVARIANCE))
        self.ALLOW_MISSING_VIO_STATUS = bool(self.declare_parameter(
            'allow_missing_vio_status', self.ALLOW_MISSING_VIO_STATUS).value)

        self.CRUISE_ALTITUDE = float(self._declare_number(
            'cruise_altitude', self.CRUISE_ALTITUDE))
        self.WINDOW_ALTITUDE = float(self._declare_number(
            'window_altitude_m', self.WINDOW_ALTITUDE))
        self.ROOM_MODE = str(self.declare_parameter(
            'room_mode', self.ROOM_MODE).value).strip().lower()
        if self.ROOM_MODE not in ('lidar', 'box'):
            raise SystemExit(
                f"room_mode must be 'lidar' or 'box'; got '{self.ROOM_MODE}'.")
        self.TILE_MOVE_TIMEOUT = float(self._declare_number(
            'tile_move_timeout', self.TILE_MOVE_TIMEOUT))
        self.ROOM_ALTITUDE = float(self._declare_number(
            'room_altitude', self.ROOM_ALTITUDE))
        self.ALT_CHANGE_TIMEOUT = float(self._declare_number(
            'alt_change_timeout', self.ALT_CHANGE_TIMEOUT))
        self.WINDOW_SEARCH_SECONDS = float(self._declare_number(
            'window_search_seconds', self.WINDOW_SEARCH_SECONDS))
        self.WINDOW_SWEEP_DEG = float(self._declare_number(
            'window_sweep_deg', self.WINDOW_SWEEP_DEG))
        self.WINDOW_BACKOFF_M = float(self._declare_number(
            'window_backoff', self.WINDOW_BACKOFF_M))
        self.WINDOW_MAX_BACKOFFS = int(self.declare_parameter(
            'window_max_backoffs', self.WINDOW_MAX_BACKOFFS).value)
        self.MARKER_RETRY_ALTITUDE = float(self._declare_number(
            'marker_retry_altitude', self.MARKER_RETRY_ALTITUDE))
        self.MARKER_RETRY_SECONDS = float(self._declare_number(
            'marker_retry_seconds', self.MARKER_RETRY_SECONDS))
        self.MARKER_RETRY_DISTANCE = float(self._declare_number(
            'marker_retry_distance', self.MARKER_RETRY_DISTANCE))
        self.MARKER_MAX_RETRIES = int(self.declare_parameter(
            'marker_max_retries', self.MARKER_MAX_RETRIES).value)

        # The climb is to the cruise height, not to a separate number: there
        # is one altitude for the marker phases and this is it.
        self.TAKEOFF_ALTITUDE = self.CRUISE_ALTITUDE

        self.OUTBOUND_DISTANCE = float(self._declare_number(
            'outbound_distance', self.OUTBOUND_DISTANCE))
        self.WINDOW_OFFSET = float(self._declare_number(
            'window_offset', self.WINDOW_OFFSET))
        self.WINDOW_OFFSET_DIRECTION = str(self.declare_parameter(
            'window_offset_direction', self.WINDOW_OFFSET_DIRECTION).value
        ).strip().lower()
        if self.WINDOW_OFFSET_DIRECTION not in ('left', 'right'):
            raise SystemExit(
                f"window_offset_direction must be 'left' or 'right'; got "
                f"'{self.WINDOW_OFFSET_DIRECTION}'.")
        self.RETURN_DISTANCE = float(self._declare_number(
            'return_distance', self.RETURN_DISTANCE))
        self.TURN_DISTANCE = float(self._declare_number(
            'turn_distance', self.TURN_DISTANCE))
        self.PAD_DISTANCE = float(self._declare_number(
            'pad_distance', self.PAD_DISTANCE))
        self.LEG_SPEED = float(self._declare_number('leg_speed', self.LEG_SPEED))
        self.LEG_SETTLE_SECONDS = float(self._declare_number(
            'leg_settle_seconds', self.LEG_SETTLE_SECONDS))
        self.LEG_LATCH_TIMEOUT = float(self._declare_number(
            'leg_latch_timeout', self.LEG_LATCH_TIMEOUT))
        self.LEG_TIMEOUT_MARGIN = float(self._declare_number(
            'leg_timeout_margin', self.LEG_TIMEOUT_MARGIN))

        # ---- centring ----
        self.MARKER_TOLERANCE = float(self._declare_number(
            'marker_tolerance', self.MARKER_TOLERANCE))
        self.MARKER_SETTLE_SECONDS = float(self._declare_number(
            'marker_settle_seconds', self.MARKER_SETTLE_SECONDS))
        self.MARKER_GAIN = float(self._declare_number(
            'marker_gain', self.MARKER_GAIN))
        self.MARKER_HOLD_SECONDS = float(self._declare_number(
            'marker_hold_seconds', self.MARKER_HOLD_SECONDS))
        self.MARKER_ALIGN_TIMEOUT = float(self._declare_number(
            'marker_align_timeout', self.MARKER_ALIGN_TIMEOUT))
        self.MARKER_MAX_AGE = float(self._declare_number(
            'marker_max_age', self.MARKER_MAX_AGE))
        self.MARKER_LOST_SECONDS = float(self._declare_number(
            'marker_lost_seconds', self.MARKER_LOST_SECONDS))
        self.PRECISION_DESCENT = bool(self.declare_parameter(
            'precision_descent', self.PRECISION_DESCENT).value)
        self.BLIND_COMMIT_ALTITUDE = float(self._declare_number(
            'blind_commit_altitude', self.BLIND_COMMIT_ALTITUDE))

        if self.MARKER_GAIN <= 0.0 or self.MARKER_GAIN > 1.0:
            self.get_logger().error(
                f"marker_gain {self.MARKER_GAIN} is outside (0, 1]; clamping "
                "to 0.6. Above 1 the alignment overshoots and can diverge.")
            self.MARKER_GAIN = 0.6

        # ---- the flight plan --------------------------------------------
        #
        # Legs 'outbound' and 'offset' run BEFORE the window mission; the rest
        # run after it. Each names its own successor rather than relying on
        # the next index, because the two strafes are optional.
        #
        # The strafes exist because the window marker is NOT on the window's
        # centre-line. Approaching from the marker would mean viewing the
        # aperture obliquely, where the quad is foreshortened and the plane
        # fit, the normal and the corner depths are all at their worst -- and
        # at larger offsets the aperture truncates at the frame edge, which is
        # the failure effective_standoff() exists to avoid. So the aircraft
        # gets on the axis FIRST, and the window logic then runs in the
        # geometry it was written and flown for.
        #
        # The mirror on the way out is not symmetry for its own sake. The down
        # camera at 78 deg HFOV sees about +/-0.95 m at 1.2 m altitude, so a
        # return leg flown along the window axis would pass the marker outside
        # its own footprint and never see it.
        strafe = self.WINDOW_OFFSET > 0.0
        back = 'right' if self.WINDOW_OFFSET_DIRECTION == 'left' else 'left'

        self.legs = [
            # retry=False: if the marker never appears, the WINDOW search
            # starts anyway. The window is the real landmark on this leg and
            # the marker only refines the approach to it, so climbing to hunt
            # for the marker would spend clock on the lesser of the two.
            Leg('outbound', 'forward', self.OUTBOUND_DISTANCE,
                self.WINDOW_MARKER_ID, 'offset' if strafe else 'window',
                retry=False),
            Leg('return', 'backward', self.RETURN_DISTANCE,
                self.WINDOW_MARKER_ID, 'turn'),
            # RIGHT in the takeoff frame -- east, toward the turn marker.
            # Said as 'left' on the course because after the room the
            # aircraft faces SOUTH, and left of south is east. Coded as
            # takeoff-frame left it flew WEST: from a marker ~1.3 m in from
            # the west edge, 5.4 m west is ~4 m outside a 7 m arena, into the
            # net. The arena drawing puts the turn marker on the east side.
            Leg('turn', 'right', self.TURN_DISTANCE, self.TURN_MARKER_ID, 'pad'),
            Leg('pad', 'backward', self.PAD_DISTANCE, self.PAD_MARKER_ID,
                'descend', required=True),
        ]
        if strafe:
            self.legs += [
                Leg('offset', self.WINDOW_OFFSET_DIRECTION, self.WINDOW_OFFSET,
                    None, 'window'),
                Leg('offset_back', back, self.WINDOW_OFFSET, None, 'return'),
            ]
        self.leg_by_name = {leg.name: i for i, leg in enumerate(self.legs)}

        self.alt_next = None        # what to run once ALT_CHANGE arrives
        self.alt_target = None      # m, the altitude it is climbing to
        self.room_alt_set = False   # has the room pattern's height been
                                    # commanded yet? See _begin_room.
        self.leg_retries = 0        # elevated searches flown on this leg
        self.retry_moving = False   # retracing, as opposed to hovering high
        self.window_rung = 0        # how far up the search failsafe we are
        self.window_backoffs = 0
        self.backoff_total = 0.0    # m retreated, for the log only -- the
                                    # approach needs no compensation for it,
                                    # see _begin_window_backoff
        self.leg_index = None       # None = not flying a leg
        self.leg_started = False
        self.legs_done = set()
        self.leg_outcomes = []

        # ---- the marker feed ----
        #
        # These are exactly the fields precision_land's helpers read, and they
        # are written ONLY for the current leg's target id (see
        # marker_points_callback). That is what makes the unbound calls to
        # those helpers safe: they cannot see a marker this leg is not for.
        self.target_marker_id = None
        self.marker_detected = False
        self.marker_point = None        # (x, y, z) in the camera's frame
        self.marker_time = None         # monotonic, when it arrived
        self.marker_ever_seen = False
        self.marker_info = ''
        self.marker_lost_since = None
        self._warned_no_attitude = False
        self._warned_blind = False
        self.align_in_band_since = None
        self.marker_error = None
        self.marker_aligned_error = None
        self.ids_in_view = set()
        # Set only by _begin_precision_descent. _handle_landing keys off it so
        # that an ordinary landing -- an abort, the flight clock, a failed
        # traverse -- keeps the base class's zero-velocity descent untouched.
        self.precision_descent_active = False

        # RTAB-Map's odometry, as a liveness signal only -- see vio_is_fresh.
        self.vio_odom_time = None
        self.vio_covariance = None
        if self.LATERAL_SOURCE == 'vio':
            self.create_subscription(Odometry, self.VIO_ODOM_TOPIC,
                                     self.vio_odom_callback, 10)

        self.create_subscription(PointStamped, '/aruco/marker_points',
                                 self.marker_points_callback, 20)
        # The single-marker topics are still worth having for diagnostics --
        # they are what precision_land flies on and what the browser view
        # annotates -- but nothing in this node's control loop reads them.
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

        # The lidar tile scan's own parameters and subscriptions. Declared
        # even in 'box' mode so a launch file may pass them either way -- an
        # undeclared override is a fatal error at startup and a launch file
        # cannot know which branch is taken.
        self.init_lidar_scan(time.monotonic)

        self.get_logger().warning(self._plan_summary())

    # ------------------------------------------------------------- the plan

    def log_flight_state(self):
        super().log_flight_state()
        if self.LATERAL_SOURCE != 'vio':
            return
        fused = self.vision_is_fused()
        aligned = self.yaw_is_aligned()
        age = ('never' if self.vio_odom_time is None
               else f"{time.monotonic() - self.vio_odom_time:.2f}s")
        self.get_logger().info(
            f"vio: odom_age={age} "
            f"cov={'?' if self.vio_covariance is None else f'{self.vio_covariance:.1f}'} "
            f"ekf_fusing={'?' if fused is None else fused} "
            f"yaw_align={'?' if aligned is None else aligned} "
            f"lateral_ok={self.flow_is_healthy()} "
            f"xy={'POS-HOLD' if self.hold_xy else 'VEL-HOLD'}",
            throttle_duration_sec=1.0)

    def _plan_summary(self):
        def leg(name):
            i = self.leg_by_name.get(name)
            return '(skipped)' if i is None else str(self.legs[i])

        return (
            "MISSION FSM, one continuous flight. ALL DIRECTIONS ARE IN THE "
            "TAKEOFF FRAME:\n"
            f"  1. climb {self.TAKEOFF_ALTITUDE:.2f} m, hold "
            f"{self.HOLD_SECONDS:.0f} s\n"
            f"  2. {leg('outbound')}\n"
            f"  3. {leg('offset')}   <- onto the window axis\n"
            "  4. find the window, line up, traverse in\n"
            + (f"  5. room: {self.TILES_X}x{self.TILES_Y} tile scan on the "
               f"LIDAR wall fix, {self.TILE_DWELL_SECONDS:.0f} s per tile, "
               "dolls counted, geotagged and displayed\n"
               if self.ROOM_MODE == 'lidar' else
               f"  5. room pattern, {len(self.room_steps)} dead-reckoned legs, "
               "dolls counted, geotagged and displayed\n")
            + "  6. relock on the window from inside, traverse out\n"
            f"  7. {leg('offset_back')}   <- back onto the marker line\n"
            f"  8. {leg('return')}\n"
            f"  9. {leg('turn')}\n"
            f" 10. {leg('pad')}\n"
            " 11. precision descent onto the pad, disarm\n"
            f"  Lateral estimate: "
            + ("RTAB-Map VIO (" + self.VIO_ODOM_TOPIC + ")\n"
               if self.LATERAL_SOURCE == 'vio' else "ARK Flow optical flow\n")
            + f"  Legs fly at {self.LEG_SPEED:.2f} m/s and END ON THEIR MARKER; "
            "the distance is a limit, not a target.\n"
            f"  Centring to {self.MARKER_TOLERANCE * 100:.0f} cm, then "
            f"{self.MARKER_HOLD_SECONDS:.0f} s of hold.\n"
            f"  Hard descent at {self.FLIGHT_SECONDS:.0f} s airborne, whatever "
            "stage we are in.\n"
            "  q aborts into a descent, k force-disarms.")

    # ------------------------------------------------- the marker feed (subs)

    def marker_points_callback(self, msg):
        """Keep ONLY the current leg's marker; discard every other id.

        This is the whole reason /aruco/marker_points exists. The fields
        written here are the ones precision_land's helpers read, so filtering
        at the point of arrival is what guarantees the control loop can never
        be handed a fix from a pad this leg is not flying to.
        """
        frame = str(msg.header.frame_id)
        if not frame.startswith('aruco:'):
            return
        try:
            mid = int(frame.split(':', 1)[1])
        except ValueError:
            return

        self.ids_in_view.add(mid)
        if self.target_marker_id is None or mid != self.target_marker_id:
            return

        self.marker_point = (float(msg.point.x), float(msg.point.y),
                             float(msg.point.z))
        self.marker_time = time.monotonic()
        self.marker_detected = True
        self.marker_ever_seen = True

    def marker_info_callback(self, msg):
        self.marker_info = msg.data

    def attitude_callback(self, msg):
        """BOTH halves of the chain need this message, in different forms.

        WindowTraverse stores the message itself as self.attitude, and its
        geometry_callback refuses EVERY sample while that is None -- so with
        only PrecisionLand's version running, the window estimator never
        receives a single corner, the pose is never built, and the flight
        locks onto a window it can then never approach. It does not error; it
        just sits in LOCK until the flight clock lands it.

        PrecisionLand's version keeps the normalised quaternion as
        self.attitude_q, which is what tilt-compensates the marker vector.

        Two different attributes, one topic, one callback name. Overriding
        with either alone silently disables the other half of the mission.
        """
        super().attitude_callback(msg)
        PrecisionLand.attitude_callback(self, msg)

    def marker_is_fresh(self):
        return PrecisionLand.marker_is_fresh(self)

    def marker_body(self):
        return PrecisionLand.marker_body(self)

    def marker_offset_ned(self):
        """(north, east, down) to the CURRENT LEG's marker, tilt-compensated."""
        return PrecisionLand.marker_offset_ned(self)

    def marker_summary(self):
        if self.target_marker_id is None:
            return "no marker targeted"
        if not self.marker_is_fresh():
            others = sorted(self.ids_in_view - {self.target_marker_id})
            return (f"id {self.target_marker_id} not visible"
                    + (f" (in view: {others})" if others else ""))
        return f"id {self.target_marker_id} IN SIGHT"

    # ------------------------------------------------------- the lateral fix

    def vio_odom_callback(self, msg):
        """RTAB-Map's own odometry, used ONLY as a liveness/quality signal.

        Nothing here is flown. The estimate the aircraft actually flies is
        EKF2's, fused from what vio_to_px4_bridge forwards to
        /fmu/in/vehicle_visual_odometry. This subscription exists to answer a
        question EKF2 answers too slowly: has RTAB-Map stopped producing?

        rtabmap_odom signals lost tracking by publishing a pose with an
        enormous covariance rather than by going silent, so BOTH are checked
        -- a stale topic and a null-covariance message mean the same thing
        here, and neither should be flown on.
        """
        self.vio_odom_time = time.monotonic()
        try:
            self.vio_covariance = float(msg.pose.covariance[0])
        except (AttributeError, IndexError):
            self.vio_covariance = None

    def vio_is_fresh(self):
        """Is RTAB-Map still producing a usable fix?

        The RTAB-Map bridge has no heartbeat topic of its own -- unlike
        zed_localization, which publishes /vio_healthy -- so freshness is
        measured on its INPUT, /rtabmap/odom. That is the better place to
        measure it anyway: it distinguishes 'RTAB-Map lost tracking' from
        'the bridge died', and it is the first of the two to go.
        """
        if self.vio_odom_time is None:
            return self.ALLOW_MISSING_VIO_STATUS
        if time.monotonic() - self.vio_odom_time > self.VIO_MAX_AGE:
            return False
        if (self.vio_covariance is not None
                and self.vio_covariance > self.VIO_MAX_COVARIANCE):
            return False
        return True

    def vision_is_fused(self):
        """Is EKF2 actually fusing the external-vision estimate right now?

        Called unbound off OffboardSequenceVio so the cs_ev_* flag logic has
        one home in this package. It reads nothing but self.estimator_flags,
        which is the base class's, so it is safe to borrow this way.
        """
        return OffboardSequenceVio.vision_is_fused(self)

    def yaw_is_aligned(self):
        """Has EKF2 resolved an absolute heading? Borrowed for the same reason."""
        return OffboardSequenceVio.yaw_is_aligned(self)

    def flow_is_healthy(self):
        """Overridden: the lateral estimate is RTAB-Map VIO, not optical flow.

        KEEPS THE BASE CLASS'S NAME ON PURPOSE. Every horizontal decision in
        the inherited chain funnels through this one predicate -- the x/y
        latch, whether a leg runs or is skipped, whether the window approach
        may fly, whether the precision descent keeps its aligned point, the
        status line. Overriding it here moves all of them onto vision without
        editing a line of flight logic, and renaming it would fork the base
        class for no gain. offboard_sequence_vio.py makes the same choice for
        the same reason.

        WHAT IS DELIBERATELY NOT TESTED
        -------------------------------
        dist_bottom and FLOW_MIN_AGL. Those gates exist because optical flow
        cannot resolve a floor 15 cm from the lens, so the base class refuses
        to believe x/y until the aircraft is above FLOW_MIN_AGL. The
        RealSense is looking out across a room and works sitting on the
        ground, so that floor is meaningless here -- and keeping it would
        block the legs at exactly the low altitudes this mission flies.

        The rangefinder is NOT dropped from the aircraft, only from this
        predicate. It remains the height reference and a hard gate in
        position_is_usable(), the arming checks and the descent. Vision that
        dies costs the mission; a height estimate that dies costs the
        aircraft.
        """
        if self.LATERAL_SOURCE == 'flow':
            return super().flow_is_healthy()

        lp = self.local_position
        if lp is None or not lp.xy_valid or not lp.v_xy_valid:
            return False
        if not self.vio_is_fresh():
            return False

        fused = self.vision_is_fused()
        if fused is None:
            # No estimator flags published at all. xy_valid plus a live
            # RTAB-Map stream is the only opinion available; weaker, but the
            # alternative is refusing to fly on a healthy aircraft.
            return True
        # Losing yaw alignment unanchors the frame the latched x/y point was
        # captured in, so a position setpoint flown against it walks away.
        return bool(fused) and bool(self.yaw_is_aligned())

    # ------------------------------------------------------------ the clock

    def _clock_stages(self):
        """Every airborne stage this class adds goes under the flight clock.

        A stage the clock does not know about is a stage that can hold the
        aircraft up past the descent the clock exists to guarantee. The legs
        especially: a leg flown on a bad flow estimate is exactly the case
        where the vehicle is somewhere unplanned and the battery is the only
        thing still counting.
        """
        return super()._clock_stages() + self.LEG_STAGES + self.TILE_STAGES

    # ------------------------------------------------------------ the machine

    def timer_callback(self):
        if self.current_stage in self.TILE_STAGES:
            self._publish_mission_phase()
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
            {
                self.TILE_SETTLE: self._handle_tile_settle,
                self.TILE_MOVE: self._handle_tile_move,
                self.TILE_DWELL: self._handle_tile_dwell,
                self.TILE_RETURN: self._handle_tile_return,
            }[self.current_stage]()
            return

        if self.current_stage not in self.LEG_STAGES:
            # Everything else is the room node's, the traversal's or the base
            # class's, reached through the same call it always was.
            super().timer_callback()
            return

        self._publish_mission_phase()

        # The clock outranks the stage handlers below, so a leg that is stuck
        # hunting for a marker cannot postpone the landing.
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
            self.CRUISE: self._handle_cruise,
            self.MARKER_ALIGN: self._handle_marker_align,
            self.MARKER_HOLD: self._handle_marker_hold,
            self.ALT_CHANGE: self._handle_alt_change,
            self.WINDOW_BACKOFF: self._handle_window_backoff,
            self.MARKER_RETRY: self._handle_marker_retry,
        }[self.current_stage]()

    # --------------------------------------------------------------- the legs

    def current_leg(self):
        if self.leg_index is None:
            return None
        return self.legs[self.leg_index]

    def _begin_leg_named(self, name):
        return self._begin_leg(self.leg_by_name[name])

    def _begin_leg(self, index):
        """Start leg `index`, retargeting the marker filter onto its id.

        Clearing the marker fields is not housekeeping, it is the correctness
        step: a fix from the PREVIOUS leg's marker left in marker_point would
        be treated as this leg's marker on the very first tick, and the leg
        would end where the last one did.
        """
        self.leg_index = index
        leg = self.legs[index]
        self.leg_started = False
        self.leg_retries = 0
        self.retry_moving = False
        self.target_marker_id = leg.marker_id
        self.marker_point = None
        self.marker_time = None
        self.marker_detected = False
        self.marker_lost_since = None
        self.align_in_band_since = None
        self.marker_error = None
        self.ids_in_view = set()
        self.moving = False
        self.yaw_remaining = 0.0
        self.MOVE_SPEED = self.LEG_SPEED

        self.get_logger().warning(
            f"LEG {index + 1}/{len(self.legs)} -- {leg}. Flying "
            f"{leg.direction} at {self.LEG_SPEED:.2f} m/s, watching the down "
            f"camera for id {leg.marker_id}; stopping on the marker, or at "
            f"{leg.distance:.1f} m if it never appears.")
        self._enter_stage(self.CRUISE)

    def _begin_cruise_move(self, leg):
        """Aim the x/y carrot at the far end of the leg.

        Same arithmetic as the base class's _begin_move, and deliberately the
        same interface: setting move_target_* and `moving` is what
        _step_xy_ramp() consumes, so the target is walked at MOVE_SPEED,
        leashed to the measured position, and shifted automatically on an EKF2
        lateral reset. The difference is only that we expect to stop early.
        """
        ref_yaw = self._reference_yaw()
        ux, uy = DIRECTIONS[leg.direction](math.cos(ref_yaw), math.sin(ref_yaw))
        self.move_start_x = self.hold_x
        self.move_start_y = self.hold_y
        self.move_target_x = self.hold_x + ux * leg.distance
        self.move_target_y = self.hold_y + uy * leg.distance
        self.move_in_band_since = None
        self.moving = True
        self.leg_started = True
        self._restart_stage_clock()

        self.get_logger().info(
            f"Leg '{leg.name}': ({self.move_start_x:.2f}, "
            f"{self.move_start_y:.2f}) -> ({self.move_target_x:.2f}, "
            f"{self.move_target_y:.2f}) NED, in the "
            f"{math.degrees(ref_yaw):+.0f} deg frame.")

    def _leg_timeout(self, leg):
        """Ideal time for the leg plus an allowance, never less than the
        allowance itself."""
        ideal = leg.distance / max(self.LEG_SPEED, 0.01)
        return ideal + self.LEG_TIMEOUT_MARGIN

    def _handle_cruise(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        leg = self.current_leg()

        # A leg is a position move; without a latched lateral estimate there
        # is no frame to express its target in, and dead reckoning 11 m on
        # velocity is exactly what this mission must not do.
        if not self.hold_xy:
            if self.leg_started:
                self.moving = False
                self._finish_leg("ABANDONED, flow lost mid-leg",
                                 retryable=False)
                return
            if self._in_stage_for() > self.LEG_LATCH_TIMEOUT:
                self._finish_leg("SKIPPED, flow never healthy enough to latch x/y",
                                 retryable=False)
            else:
                self.get_logger().info(
                    "Waiting for a flow-healthy x/y hold before the leg...",
                    throttle_duration_sec=1.0)
            return

        if not self.leg_started:
            self._begin_cruise_move(leg)
            return

        self.log_flight_state()

        # The marker outranks the distance: this is what makes a leg
        # marker-terminated rather than a fixed translation.
        if self.marker_offset_ned() is not None:
            travelled = math.hypot(self.local_position.x - self.move_start_x,
                                   self.local_position.y - self.move_start_y)
            self.get_logger().warning(
                f"Leg '{leg.name}': marker id {leg.marker_id} ACQUIRED after "
                f"{travelled:.2f} m of {leg.distance:.1f} m. Centring.")
            self.moving = False
            self.marker_lost_since = None
            self.align_in_band_since = None
            self._enter_stage(self.MARKER_ALIGN)
            return

        lp = self.local_position
        remaining = math.hypot(self.move_target_x - lp.x,
                               self.move_target_y - lp.y)

        if remaining <= self.MOVE_TOLERANCE:
            if self.move_in_band_since is None:
                self.move_in_band_since = time.monotonic()
            elif (time.monotonic() - self.move_in_band_since
                    >= self.LEG_SETTLE_SECONDS):
                # Park the hold exactly on the target, so the settle is
                # station keeping and not a slow continuation of the move.
                self.hold_x = self.move_target_x
                self.hold_y = self.move_target_y
                self.moving = False
                self._finish_leg(
                    f"reached {leg.distance:.1f} m without seeing id "
                    f"{leg.marker_id}")
            return

        self.move_in_band_since = None
        self.get_logger().info(
            f"Leg '{leg.name}': {remaining:.2f} m to the limit, "
            f"{self.marker_summary()}.", throttle_duration_sec=1.0)

        if self._in_stage_for() > self._leg_timeout(leg):
            self.moving = False
            self.hold_x = lp.x
            self.hold_y = lp.y
            self._finish_leg(f"TIMED OUT {remaining:.2f} m short",
                             retryable=False)

    # ------------------------------------------------------------- centring

    def _handle_marker_align(self):
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
                    f"Marker lost while centring ({self.marker_summary()}); "
                    "holding position.")
            elif (time.monotonic() - self.marker_lost_since
                    >= self.MARKER_LOST_SECONDS):
                self.get_logger().warning(
                    "Marker gone. Back to cruising the leg.")
                self._enter_stage(self.CRUISE)
            self.align_in_band_since = None
            self._check_align_timeout()
            return

        self.marker_lost_since = None
        north, east = offset[0], offset[1]
        error = math.hypot(north, east)
        self.marker_error = error

        # Arrival is tested first, so a vehicle that is already centred is not
        # given one more nudge before it is allowed to settle.
        if error <= self.MARKER_TOLERANCE:
            if self.align_in_band_since is None:
                self.align_in_band_since = time.monotonic()
            elif (time.monotonic() - self.align_in_band_since
                    >= self.MARKER_SETTLE_SECONDS):
                self._on_marker_aligned(error)
                return
        else:
            self.align_in_band_since = None

        if not self._drive_towards(north, east):
            self.get_logger().info(
                "Waiting for a flow-healthy x/y hold before correcting...",
                throttle_duration_sec=1.0)
        else:
            body = self.marker_body()
            self.get_logger().info(
                f"Centring on id {self.target_marker_id}: marker "
                f"{body_words(body[0], body[1])}, err {error:.2f} m "
                f"(want {self.MARKER_TOLERANCE:.2f}).",
                throttle_duration_sec=1.0)

        self._check_align_timeout()

    def _drive_towards(self, north, east):
        """Point the base class's x/y carrot at the marker.

        Nothing here talks to PX4: setting move_target_* and `moving` is
        precisely the interface _step_xy_ramp() consumes. False means there is
        no latched lateral estimate to express a target in yet.
        """
        if not self.hold_xy or self.local_position is None:
            return False
        lp = self.local_position
        self.move_target_x = lp.x + self.MARKER_GAIN * north
        self.move_target_y = lp.y + self.MARKER_GAIN * east
        self.moving = True
        return True

    def _check_align_timeout(self):
        if self._in_stage_for() > self.MARKER_ALIGN_TIMEOUT:
            self.moving = False
            self._finish_leg(
                f"could not centre on id {self.target_marker_id} within "
                f"{self.MARKER_ALIGN_TIMEOUT:.0f} s"
                + ('' if self.marker_error is None
                   else f" (best {self.marker_error:.2f} m)"))

    def _on_marker_aligned(self, error):
        """Stop correcting and park on the point actually reached.

        Inside the tolerance there is nothing left to win by chasing, and
        re-commanding on every frame would only inject corner noise into the
        position setpoint.
        """
        self.moving = False
        self.marker_aligned_error = error
        lp = self.local_position
        if self.hold_xy and lp is not None:
            self.hold_x = lp.x
            self.hold_y = lp.y
        self.get_logger().warning(
            f"CENTRED on id {self.target_marker_id} to {error * 100:.0f} cm. "
            f"Holding {self.MARKER_HOLD_SECONDS:.0f} s.")
        self._enter_stage(self.MARKER_HOLD)

    def _handle_marker_hold(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        # Deadband with hysteresis: drift back outside the released band earns
        # another correction, noise inside it does not.
        offset = self.marker_offset_ned()
        if offset is not None:
            error = math.hypot(offset[0], offset[1])
            self.marker_error = error
            if error > self.MARKER_TOLERANCE * self.MARKER_RELEASE:
                self.get_logger().warning(
                    f"Drifted {error:.2f} m off id {self.target_marker_id}; "
                    "correcting again.")
                self.align_in_band_since = None
                self._enter_stage(self.MARKER_ALIGN)
                return

        remaining = self.MARKER_HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._finish_leg(
                f"centred on id {self.target_marker_id}"
                + ('' if self.marker_aligned_error is None
                   else f" to {self.marker_aligned_error * 100:.0f} cm"))
            return

        self.get_logger().info(
            f"Holding over id {self.target_marker_id}, {remaining:.1f} s. "
            f"{self.marker_summary()}"
            + ('' if self.marker_error is None
               else f", err {self.marker_error:.2f} m"),
            throttle_duration_sec=1.0)

    # ------------------------------------------------------- what comes next

    def _finish_leg(self, outcome, retryable=True):
        """Record how a leg ended and dispatch to whatever follows it.

        Every leg ends here, however it ended -- on its marker, on its
        distance, on a timeout or on a lost estimate -- so there is exactly
        one place that decides what the mission does next.

        A leg that WANTED a marker, could have used one, and did not get one
        gets one more chance first, from higher up -- see _begin_marker_retry.

        `retryable` is False for the endings where a second look cannot help:
        a lost x/y latch or a stall means the position estimate itself is not
        to be trusted, and climbing to search on a bad estimate is searching
        the wrong patch of floor more widely.
        """
        leg = self.current_leg()
        on_marker = self.marker_aligned_error is not None and self.marker_is_fresh()

        if (retryable and leg.retry and leg.marker_id is not None
                and not on_marker
                and self.leg_retries < self.MARKER_MAX_RETRIES):
            self.leg_retries += 1
            self._begin_marker_retry(leg, outcome)
            return

        self._complete_leg(leg, outcome, on_marker)

    # ----------------------------------------------- the missed-marker retry

    def _begin_marker_retry(self, leg, why):
        """Climb, and look for the marker again from a wider basket.

        The failure being handled is almost always the same one: ten metres
        flown on optical flow with nothing correcting it, and the marker
        finishing just outside a footprint only +/-2.0 m wide at cruise
        height. The fix is to make the footprint wider, which costs a climb
        and nothing else -- at 3.50 m it is +/-2.83 m, a 40% wider swath.

        Done BEFORE any of the per-leg fallbacks, so it applies to the course
        legs and the pad leg alike. It is the pad leg it matters most for:
        without it, one missed detection at the end of the course is a landing
        off the pad, with nothing else left to try.
        """
        self.moving = False
        self.get_logger().warning(
            f"Leg '{leg.name}': {why}. RETRY {self.leg_retries}/"
            f"{self.MARKER_MAX_RETRIES}: climbing to "
            f"{self.MARKER_RETRY_ALTITUDE:.2f} m for a wider look (basket "
            f"about +/-{self.MARKER_RETRY_ALTITUDE * 0.81:.2f} m, against "
            f"+/-{self.CRUISE_ALTITUDE * 0.81:.2f} m at cruise).")
        self._begin_alt_change(self.MARKER_RETRY_ALTITUDE, 'retry',
                               f"wider look for id {leg.marker_id}")

    def _begin_retry_search(self):
        """At the retry altitude: hover and look, then retrace and look."""
        self.retry_moving = False
        self.moving = False
        self._enter_stage(self.MARKER_RETRY)
        self.get_logger().warning(
            f"Looking for id {self.target_marker_id} from "
            f"{self.MARKER_RETRY_ALTITUDE:.2f} m for "
            f"{self.MARKER_RETRY_SECONDS:.0f} s, then retracing "
            f"{self.MARKER_RETRY_DISTANCE:.1f} m back along the leg.")

    def _handle_marker_retry(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        leg = self.current_leg()
        if leg is None:
            self._enter_stage(self.CRUISE)
            return

        # Found it. Drop back to cruise height and centre there: the centring
        # loop is the same at any altitude, but the marker pose is better
        # resolved closer to it.
        if self.marker_offset_ned() is not None:
            self.get_logger().warning(
                f"RETRY SUCCEEDED: id {leg.marker_id} found from "
                f"{self.MARKER_RETRY_ALTITUDE:.2f} m. Descending to centre.")
            self.moving = False
            self.marker_lost_since = None
            self.align_in_band_since = None
            self._begin_alt_change(self.CRUISE_ALTITUDE, 'align',
                                   'marker found, dropping to centre')
            return

        if not self.retry_moving:
            if self._in_stage_for() <= self.MARKER_RETRY_SECONDS:
                self.get_logger().info(
                    f"Retry: holding high, {self.marker_summary()}.",
                    throttle_duration_sec=1.0)
                return
            if not self.hold_xy or self.local_position is None:
                self._complete_leg(leg, "retry could not latch x/y", False)
                return
            # Retrace along the leg, BACKWARDS, still high. Backwards because
            # a leg that ran out of distance overshot or drifted, and the
            # marker is behind the point it stopped at, not ahead of it.
            ref_yaw = self._reference_yaw()
            ux, uy = DIRECTIONS[leg.direction](math.cos(ref_yaw),
                                               math.sin(ref_yaw))
            self.move_target_x = self.hold_x - ux * self.MARKER_RETRY_DISTANCE
            self.move_target_y = self.hold_y - uy * self.MARKER_RETRY_DISTANCE
            self.move_in_band_since = None
            self.moving = True
            self.retry_moving = True
            self.MOVE_SPEED = self.LEG_SPEED
            self._restart_stage_clock()
            self.get_logger().warning(
                f"Retry: retracing {self.MARKER_RETRY_DISTANCE:.1f} m back "
                f"along the '{leg.name}' leg at "
                f"{self.MARKER_RETRY_ALTITUDE:.2f} m.")
            return

        lp = self.local_position
        remaining = math.hypot(self.move_target_x - lp.x,
                               self.move_target_y - lp.y)
        timed_out = self._in_stage_for() > self._leg_timeout(leg)
        if remaining <= self.MOVE_TOLERANCE or timed_out:
            self.moving = False
            self.hold_x, self.hold_y = lp.x, lp.y
            self._complete_leg(
                leg,
                f"retry found nothing after {self.MARKER_RETRY_DISTANCE:.1f} m "
                f"at {self.MARKER_RETRY_ALTITUDE:.2f} m", False)
            return

        self.get_logger().info(
            f"Retry: retracing, {remaining:.2f} m to go. "
            f"{self.marker_summary()}.", throttle_duration_sec=1.0)

    # ------------------------------------------------------- what comes next

    def _complete_leg(self, leg, outcome, on_marker):
        """The teardown and the dispatch, once a leg is really finished."""
        index = self.leg_index

        self.leg_outcomes.append(f"{leg.name} -> {outcome}")
        self.get_logger().warning(f"Leg '{leg.name}': {outcome}.")

        self.legs_done.add(leg.name)
        self.leg_index = None
        self.leg_started = False
        self.target_marker_id = None
        self.moving = False
        self.marker_aligned_error = None
        # The legs borrow MOVE_SPEED; the window approach owns it.
        self.MOVE_SPEED = self.APPROACH_SPEED

        if leg.required and not on_marker:
            # Only the pad leg is required, and a pad leg with no pad has
            # nothing sensible left to do. Landing here beats hovering until
            # the battery decides where to land for us.
            self.get_logger().error(
                f"Leg '{leg.name}' needed marker id {leg.marker_id} and did "
                "not get it. Landing HERE, off the pad.")
            self._begin_landing(f"'{leg.name}' leg failed -- {outcome}")
            return

        # THE ALTITUDE SCHEDULE. Two sightings of the window marker bracket
        # the low part of the flight: drop to WINDOW_ALTITUDE on the way in,
        # climb back to CRUISE_ALTITUDE on the way out. Hooked to the LEGS
        # rather than to the window stages because it is the marker that says
        # where we are, and because the room's own heights come from the
        # window pose and must not be overridden.
        if leg.name == 'outbound':
            # UNCONDITIONAL on the marker. Whether the marker was found or
            # the leg simply ran out of distance, the next thing that happens
            # is the window, and the window is approached from
            # WINDOW_ALTITUDE. Coming down is not a reward for finding the
            # marker; it is how the approach is set up.
            self._begin_alt_change(
                self.WINDOW_ALTITUDE, leg.nxt,
                'window marker reached' if on_marker
                else f"no marker inside {leg.distance:.1f} m, dropping anyway "
                     "to look for the window")
            return
        if leg.name == 'return':
            self._begin_alt_change(
                self.CRUISE_ALTITUDE, leg.nxt,
                'back at the window marker, climbing for the markers'
                if on_marker else
                'no marker on the way back, climbing anyway for the roll')
            return

        self._dispatch_after(leg.nxt, on_marker)

    def _dispatch_after(self, nxt, on_marker=False):
        """Run whatever follows a leg or an altitude change.

        One place, so a leg and an altitude change that precedes the same
        successor cannot disagree about what it is.
        """
        self.alt_next = None
        if nxt == 'room':
            self._begin_room()
            return

        if nxt == 'retry':
            self._begin_retry_search()
            return

        if nxt == 'align':
            self._enter_stage(self.MARKER_ALIGN)
            return

        if nxt == 'window':
            # Exactly the fork window_scan takes at the end of its hold: if
            # the window is already in front of us there is nothing to search
            # for, and sweeping would turn away from a window we can see.
            if self.window_is_confirmed():
                self._lock_on_window("already in sight after the approach legs")
            else:
                self._begin_scan()
            return

        if nxt == 'descend':
            self._begin_precision_descent(on_marker)
            return

        self._begin_leg_named(nxt)

    # --------------------------------------------- the lidar tile scan

    def _begin_tile_scan(self):
        """Hand the room over to the lidar, once inside and stable.

        Called in place of the inherited box pattern. The VIO is still being
        fused by EKF2 and is still what flow_is_healthy() tests -- nothing
        about the estimator changes here. What changes is that the waypoints
        are now measured from the WALLS instead of from EKF2's origin, so
        they stop moving when EKF2 drifts.
        """
        # Waypoints are NOT built here. They are relative to the entry pose,
        # which the lidar only measures once the transform has settled --
        # see the end of _handle_tile_settle.
        self.tile_centres_arena = []
        self.tile_index = 0
        self.tiles_done = []
        self.room_entry_heading = 0.0
        self.tile_arrived_since = None
        self.tile_dwell_started = None
        self.MOVE_SPEED = self.TILE_SPEED
        # Restore the cruise yaw rate. _begin_scan dropped it to
        # SCAN_YAW_RATE (~3 deg/s) for the window sweep and nothing puts it
        # back, so the half turn before the relock would take a minute and
        # time out at 50 degrees short -- pointing the yaw cone at a wall.
        self.YAW_RATE = self.cruise_yaw_rate
        self._arm_dolls('inside the room, starting the tile scan')
        self.get_logger().warning(
            f"ROOM SCAN on the lidar: '{self.ROOM_SEQUENCE}' in a "
            f"{self.ROOM_X:.2f}x{self.ROOM_Y:.2f} m room, "
            f"{self.TILE_DWELL_SECONDS:.0f} s at each point, every point kept "
            f"{self.WALL_MARGIN:.2f} m off the walls. Arena frame: +X is the "
            f"{self.FRONT_WALL.upper()} wall, +Y to its {self.Y_AXIS.upper()}.")
        self._enter_stage(self.TILE_SETTLE)

    def _handle_tile_settle(self):
        """Stabilise on the lidar before flying anything in its frame.

        The transform needs both pose streams live and a few ticks of
        low-pass before it means anything, and a tile centre computed through
        a half-converged transform is in the wrong place. So the scan does not
        start until there is a fresh fix AND a transform, which is what the
        'stabilize it first' in the plan amounts to.
        """
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.update_transform()
        self.log_flight_state()

        if not self.lidar_fix_is_fresh():
            if self._in_stage_for() > self.LIDAR_FIX_TIMEOUT * 5.0:
                self._tile_scan_give_up(
                    'no lidar fix inside the room -- is wall_localizer running '
                    'and is the scan height inside the window band?')
                return
            self.get_logger().info(
                f"Waiting for a lidar fix... ({self.lidar_status or 'no status'})",
                throttle_duration_sec=1.0)
            return

        if self.arena_tf is None:
            self.get_logger().info("Deriving the arena transform...",
                                   throttle_duration_sec=1.0)
            return

        # The low-pass needs latch_sec to converge before the drift reference
        # means anything. This wait IS the 'stabilise first' step.
        if self._in_stage_for() < self.TRANSFORM_LATCH_SECONDS:
            self.get_logger().info(
                f"Settling the arena transform, "
                f"{self.TRANSFORM_LATCH_SECONDS - self._in_stage_for():.1f} s "
                "to go...", throttle_duration_sec=1.0)
            return

        self.latch_transform_reference()
        ax, ay = self.ned_to_arena(self.local_position.x, self.local_position.y)
        # The aircraft got in through the window, so where it is standing now
        # IS the window axis. Recorded for the return -- see
        # relock_station_arena.
        self.arena_entry = (ax, ay)
        # ...and the heading it came in on. The window is directly BEHIND that
        # heading, so the way to face it again is a half turn. The old box
        # pattern got this for free from its two 90-degree legs; this scan
        # does not yaw between points, so it has to be commanded.
        self.room_entry_heading = float(self.local_position.heading)
        # The same heading in the ARENA frame, which the room moves are
        # expressed relative to: forward = the way the aircraft flew in.
        self.arena_entry_yaw = float(self.lidar_fix[3])

        wps = self.build_room_waypoints()
        if not wps:
            self._tile_scan_give_up(
                f"room_scan_sequence '{self.ROOM_SEQUENCE}' produced no moves")
            return
        for name, d, want, got in self.waypoints_clamped:
            self.get_logger().error(
                f"CLAMPED '{name} {d:.2f}': it would have planned arena "
                f"({want[0]:+.2f}, {want[1]:+.2f}) m, less than "
                f"{self.WALL_MARGIN:.2f} m from a wall in a "
                f"{self.ROOM_X:.2f} m room. Flying ({got[0]:+.2f}, "
                f"{got[1]:+.2f}) instead. Check where the window sits in its "
                "wall -- the pattern is relative to it.")
        plan = ' -> '.join(f"({x:+.2f},{y:+.2f})" for x, y in wps)
        self.get_logger().warning(f"Room waypoints (arena): {plan}")
        self.get_logger().warning(
            f"Stable on the lidar at arena ({ax:+.2f}, {ay:+.2f}) m, transform "
            f"latched. Flying {len(self.tile_centres_arena)} tiles.")
        self._enter_stage(self.TILE_MOVE)

    def _handle_tile_move(self):
        """Fly to the current tile centre, re-derived in NED every tick."""
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.update_transform()
        self.log_flight_state()

        if not self._tile_health_ok():
            return

        tile = self.current_tile()
        if tile is None:
            self._finish_tile_scan('all tiles visited')
            return

        n, e, _ = self.arena_to_ned(tile[0], tile[1])
        self._set_target(n, e)

        lp = self.local_position
        remaining = math.hypot(n - lp.x, e - lp.y)
        if remaining <= self.TILE_ARRIVE_EPS:
            if self.tile_arrived_since is None:
                self.tile_arrived_since = time.monotonic()
            elif (time.monotonic() - self.tile_arrived_since
                    >= self.TILE_SETTLE_SECONDS):
                # Pin the hold to the tile centre so the dwell is station
                # keeping rather than a slow continuation of the approach.
                self.moving = False
                self.hold_x, self.hold_y = n, e
                self.tile_dwell_started = time.monotonic()
                self.get_logger().warning(
                    f"Tile {self.tile_progress()} at arena "
                    f"({tile[0]:+.2f}, {tile[1]:+.2f}) m. Holding "
                    f"{self.TILE_DWELL_SECONDS:.0f} s for the detector.")
                self._enter_stage(self.TILE_DWELL)
            return

        self.tile_arrived_since = None
        self.get_logger().info(
            f"Tile {self.tile_progress()}: {remaining:.2f} m to go.",
            throttle_duration_sec=1.0)

        if self._in_stage_for() > self.TILE_MOVE_TIMEOUT:
            self.get_logger().error(
                f"Tile {self.tile_progress()} not reached in "
                f"{self.TILE_MOVE_TIMEOUT:.0f} s, {remaining:.2f} m short. "
                "Skipping it.")
            self.tiles_done.append(f"{self.tile_progress()} TIMED OUT")
            self._next_tile()

    def _handle_tile_dwell(self):
        """Hold still while doll_detect works this tile.

        Stationary on purpose. The detector needs MIN_FRAMES_TO_CONFIRM
        consecutive frames on a track before it counts a doll, and a moving
        camera costs tracks to motion blur at exactly the moment they are
        being confirmed.
        """
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.update_transform()
        self.log_flight_state()

        if not self._tile_health_ok():
            return

        left = self.TILE_DWELL_SECONDS - (time.monotonic() - self.tile_dwell_started)
        if left <= 0.0:
            tile = self.current_tile()
            self.tiles_done.append(
                f"{self.tile_progress()} ({tile[0]:+.2f},{tile[1]:+.2f})")
            self.get_logger().warning(
                f"Tile {self.tile_progress()} done. Dolls so far: "
                f"{self.doll_count if hasattr(self, 'doll_count') else '?'}")
            self._next_tile()
            return

        self.get_logger().info(
            f"Tile {self.tile_progress()}: dwelling {left:.1f} s.",
            throttle_duration_sec=1.0)

    def _next_tile(self):
        self.tile_index += 1
        self.tile_arrived_since = None
        self.tile_dwell_started = None
        if self.current_tile() is None:
            self.get_logger().warning(
                f"All {len(self.tile_centres_arena)} tiles scanned. Returning "
                f"to {self.RELOCK_STANDOFF:.2f} m off the window wall for the "
                "relock.")
            self._enter_stage(self.TILE_RETURN)
            return
        self.MOVE_SPEED = self.TILE_SPEED
        self._enter_stage(self.TILE_MOVE)

    def _handle_tile_return(self):
        """Fly back to the window side before handing over to RELOCK.

        The serpentine finishes in whichever corner the pattern ends in, which
        for a 2x2 is the one FURTHEST from the window. RELOCK has to rebuild a
        window pose from scratch and needs the whole aperture in frame to do
        it, so starting it from the far corner is asking it to fail. This is
        the 'come back' step.
        """
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.update_transform()
        self.log_flight_state()

        # Deliberately NOT gated on _tile_health_ok: if the lidar has died,
        # flying back toward the window on the last good transform is still
        # better than relocking from the far corner, and RELOCK itself does
        # not need the lidar at all.
        if self.arena_tf is None:
            self._finish_tile_scan('no transform for the return')
            return

        sx, sy = self.relock_station_arena()
        n, e, _ = self.arena_to_ned(sx, sy)
        self._set_target(n, e)

        # Face back the way we came in. RELOCK latches its yaw cone on the
        # heading it STARTS from, so arriving still pointed at the far wall
        # would centre a 50-degree cone on entirely the wrong direction and
        # every window sample would be gated out.
        facing = wrap_pi(self.room_entry_heading + math.pi)
        # Set yaw_remaining DIRECTLY rather than through _aim_yaw_at. That
        # helper clamps to the +/-yaw_cone_deg cone, which exists to stop the
        # window search wandering off the wall it was pointed at -- and a
        # half turn is 180 degrees, so it would be clipped to 50 and the
        # aircraft would stop a long way short, facing nothing. The inherited
        # room turn sets it directly for the same reason. The ramp still
        # limits the rate; only the cone is bypassed.
        self.yaw_remaining = wrap_pi(facing - self.yaw_setpoint)
        turned = abs(self._heading_error(facing)) <= self.ALIGN_YAW_TOLERANCE

        lp = self.local_position
        remaining = math.hypot(n - lp.x, e - lp.y)
        arrived = remaining <= self.TILE_ARRIVE_EPS
        if (arrived and turned) or self._in_stage_for() > self.TILE_MOVE_TIMEOUT:
            self.moving = False
            self.hold_x, self.hold_y = lp.x, lp.y
            if not turned:
                self.get_logger().warning(
                    "Returned but still "
                    f"{math.degrees(abs(self._heading_error(facing))):.0f} deg "
                    "off the entry heading; relocking anyway.")
            self._finish_tile_scan('all tiles visited')
            return

        self.get_logger().info(
            f"Returning to the window wall, {remaining:.2f} m and "
            f"{math.degrees(abs(self._heading_error(facing))):.0f} deg to go.",
            throttle_duration_sec=1.0)

    def _tile_health_ok(self):
        """The two things that invalidate every waypoint at once."""
        if not self.lidar_fix_is_fresh():
            self._tile_scan_give_up(
                f"lidar fix stale for more than {self.LIDAR_FIX_TIMEOUT:.1f} s "
                f"({self.lidar_status or 'no status'})")
            return False
        if not self.transform_is_healthy():
            self._tile_scan_give_up(
                'the arena->NED transform has drifted beyond '
                f"{self.TRANSFORM_DRIFT_MAX:.2f} m / "
                f"{math.degrees(self.TRANSFORM_YAW_DRIFT_MAX):.0f} deg -- one "
                'pose stream is moving against the other, so every tile '
                'centre is now in the wrong place')
            return False
        return True

    def _tile_scan_give_up(self, why):
        """Stop scanning, keep the mission. The way out does not need tiles."""
        self.get_logger().error(f"TILE SCAN ABANDONED: {why}.")
        self.tiles_done.append(f"ABANDONED: {why}")
        self._finish_tile_scan(f"abandoned -- {why}")

    def _finish_tile_scan(self, outcome):
        """Hand back to the inherited way out: relock and traverse.

        Deliberately NOT a landing. The tiles are the scoring part, but the
        aircraft still has to leave the room, and the way out is flown on the
        window, not on the lidar -- so a failed scan still gets flown home.
        """
        self.moving = False
        self.MOVE_SPEED = self.APPROACH_SPEED
        self.get_logger().warning(
            f"Tile scan {outcome}. Scanned: {self.scan_summary()}. "
            "Turning for the way out.")
        self._begin_relock()

    # ----------------------------------------------- the doll detector

    def _lock_on_window(self, reason):
        """Start the doll detector the moment the window is FIRST found.

        The inherited node arms it at the inbound traverse commit. It is
        started earlier here, at the first lock, from outside the room: the
        TensorRT engine takes seconds to load on its first enabled frame, and
        starting it at the commit spends those seconds inside the room, which
        is the only place it is needed. Anything it sees through the aperture
        from outside is harmless -- doll_detect counts by geotagged position,
        and there are no dolls between the marker and the wall to count.

        Only on the way IN. On the way out the detector is already on and is
        switched off at the outbound clear, as before.
        """
        if self.phase == self.PHASE_OUTSIDE:
            self._arm_dolls("window first detected, before entering the room")
        super()._lock_on_window(reason)

    # ----------------------------------------------------- the room height

    def _begin_room(self):
        """Fly the box pattern at ROOM_ALTITUDE, not at the window's height.

        The traversal goes through wherever the aperture actually is -- that
        comes from the measured window pose and is not ours to choose. Once
        inside, the room is a different problem: the pattern wants a height
        that clears the floor and the ceiling with margin, and that is a
        number we pick rather than one the window hands us.

        Commanded once, on the way in. The flag matters because
        _dispatch_after comes back through here when the climb finishes, and
        without it the two would ping-pong.
        """
        if not self.room_alt_set:
            self.room_alt_set = True
            self._begin_alt_change(self.ROOM_ALTITUDE, 'room',
                                   'inside, levelling for the room pattern')
            return
        if self.ROOM_MODE == 'lidar':
            self._begin_tile_scan()
            return
        super()._begin_room()

    # ------------------------------------------------ the altitude schedule

    def _begin_alt_change(self, altitude, nxt, why):
        """Climb or descend to `altitude`, then run `nxt`.

        The height is commanded through the base class's own z ramp -- the
        same one the climb uses -- so the descent is rate-limited and leashed
        exactly as every other vertical move in this package is. Nothing here
        publishes a setpoint of its own.
        """
        if self.home_z is None:
            self._dispatch_after(nxt)
            return
        self.alt_target = float(altitude)
        self.alt_next = nxt
        self.commanded_altitude = float(altitude)
        self.target_z = self.home_z - float(altitude)
        self.moving = False
        current = self.relative_altitude()
        self.get_logger().warning(
            f"ALTITUDE: {altitude:.2f} m ({why})"
            + ('' if current is None else f", from {current:.2f} m."))
        self._enter_stage(self.ALT_CHANGE)

    def _handle_alt_change(self):
        if not self._still_flyable():
            return

        # Hold station laterally while the height changes. A climb flown with
        # the carrot still walking somewhere is two moves at once, and the
        # flow degrades on the vertical one.
        self._try_latch_xy_hold()
        self.log_flight_state()

        alt = self.relative_altitude()
        if alt is not None and abs(alt - self.alt_target) <= self.ALTITUDE_TOLERANCE:
            self.get_logger().warning(
                f"At {alt:.2f} m. Continuing.")
            self._dispatch_after(self.alt_next)
            return

        if self._in_stage_for() > self.ALT_CHANGE_TIMEOUT:
            # Not fatal. The height is a preference -- a better basket for the
            # markers, a better line on the window -- and the mission is worth
            # more than the preference. Carry on from wherever we got to.
            self.get_logger().error(
                f"Did not reach {self.alt_target:.2f} m in "
                f"{self.ALT_CHANGE_TIMEOUT:.0f} s"
                + ('' if alt is None else f" (stuck at {alt:.2f} m)")
                + ". Continuing anyway.")
            self._dispatch_after(self.alt_next)
            return

        self.get_logger().info(
            f"Changing altitude to {self.alt_target:.2f} m"
            + ('' if alt is None else f", {alt:.2f} m now") + "...",
            throttle_duration_sec=1.0)

    # --------------------------------------------- the window search ladder

    def _handle_scan(self):
        """The inherited search, with a failsafe ladder on top.

        window_traverse stares straight ahead and waits for the detector until
        the flight clock lands it. That is right for a flight that was pointed
        at the window before it armed; it is not right here, where the
        aircraft arrived by dead reckoning and may be looking slightly past
        the aperture, or standing too close for it to fit in the frame.
        """
        super()._handle_scan()

        if self.current_stage != self.SCAN:
            return                              # locked, or gave up
        if self._in_stage_for() <= self.WINDOW_SEARCH_SECONDS:
            return
        self._escalate_window_search()

    def _escalate_window_search(self):
        """One rung up the ladder. Rung 1 sweeps; the rest back off."""
        if self.window_rung == 0:
            # Rung 1: yaw left, centre, right. This is SCAN's own sweep, which
            # window_traverse disables by setting the span to zero. SCAN_SPAN
            # is the TOTAL arc, so +/-30 deg is a 60 deg span.
            self.window_rung = 1
            self.SCAN_SPAN = math.radians(2.0 * self.WINDOW_SWEEP_DEG)
            self.get_logger().warning(
                f"No window after {self.WINDOW_SEARCH_SECONDS:.0f} s. "
                f"FAILSAFE 1: sweeping +/-{self.WINDOW_SWEEP_DEG:.0f} deg.")
            self._begin_scan()
            return

        if self.window_backoffs >= self.WINDOW_MAX_BACKOFFS:
            self._abandon(
                f"no window after a sweep and {self.window_backoffs} backoffs "
                f"totalling {self.backoff_total:.2f} m")
            return

        self._begin_window_backoff()

    def _begin_window_backoff(self):
        """Retreat WINDOW_BACKOFF_M and look again.

        WHY BACKING OFF IS THE RIGHT SECOND RUNG
        ----------------------------------------
        The commonest reason for seeing nothing at all -- as opposed to seeing
        a truncated quad, which is RECENTRE's job -- is standing too close for
        the aperture to fit in the frame at all. The D435i colour sensor is
        70 x 43 deg, so a window of height H needs about 1.26 * H of depth
        before the whole of it is in the picture. Yawing cannot fix that.
        Backing off is the only thing that can.

        AND WHY THE TRAVERSAL NEEDS NO COMPENSATION FOR IT
        --------------------------------------------------
        It is tempting to think the retreat has to be added back somewhere.
        It does not, and adding it would be a bug. approach_points() builds
        both ends of the traverse from the WINDOW POSE --

            entry = centre + normal * effective_standoff(est)
            exit  = centre - normal * EXIT_DISTANCE

        -- and _handle_align recomputes that from the live estimate on every
        tick. Where the aircraft happens to be standing never enters the
        arithmetic. Backing off changes where the approach STARTS, not where
        it ends; ALIGN simply flies a little further forward to the same entry
        point. Compensating manually would move the entry point backwards by
        the backoff and put the aircraft short of the window.
        """
        self.window_backoffs += 1
        self.backoff_total += self.WINDOW_BACKOFF_M
        ref_yaw = self._reference_yaw()
        ux, uy = DIRECTIONS['backward'](math.cos(ref_yaw), math.sin(ref_yaw))
        self.move_start_x = self.hold_x
        self.move_start_y = self.hold_y
        self.move_target_x = self.hold_x + ux * self.WINDOW_BACKOFF_M
        self.move_target_y = self.hold_y + uy * self.WINDOW_BACKOFF_M
        self.move_in_band_since = None
        self.moving = True
        self.MOVE_SPEED = self.APPROACH_SPEED
        self.get_logger().warning(
            f"FAILSAFE {self.window_backoffs + 1}: backing off "
            f"{self.WINDOW_BACKOFF_M:.2f} m "
            f"({self.backoff_total:.2f} m total) to fit the aperture in frame, "
            "then sweeping again.")
        self._enter_stage(self.WINDOW_BACKOFF)

    def _handle_window_backoff(self):
        if not self._still_flyable():
            return

        self._try_latch_xy_hold()
        self.log_flight_state()

        # A window that turns up mid-retreat ends the retreat: the point of
        # backing off is to see it, and it has been seen.
        if self.window_is_confirmed():
            self.moving = False
            self.hold_x = self.local_position.x
            self.hold_y = self.local_position.y
            self._lock_on_window("window found while backing off")
            return

        lp = self.local_position
        if lp is None:
            return
        remaining = math.hypot(self.move_target_x - lp.x,
                               self.move_target_y - lp.y)
        arrived = remaining <= self.MOVE_TOLERANCE
        if arrived or self._in_stage_for() > self.MOVE_TIMEOUT:
            self.moving = False
            self.hold_x = lp.x
            self.hold_y = lp.y
            self.get_logger().warning(
                f"Backed off {self.backoff_total:.2f} m. Sweeping again.")
            self._begin_scan()
            return

        self.get_logger().info(
            f"Backing off, {remaining:.2f} m to go.", throttle_duration_sec=1.0)

    # ------------------------------------------- splicing into the inherited

    def _handle_hold(self):
        """End of the settle at altitude.

        The parent goes straight to the window sweep. The outbound leg is
        flown first: the sweep centres its yaw cone on the heading it STARTS
        from, so it must start from in front of the window, not from the
        arming point.
        """
        if 'outbound' in self.legs_done:
            super()._handle_hold()
            return

        if not self._still_flyable():
            return

        # The same settle the parent does. The x/y latch has to happen in
        # here: a leg is a position move and needs a frame to express it in.
        self._try_latch_xy_hold()

        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_leg_named('outbound')
            return

        self.get_logger().info(
            f"Holding, {remaining:.1f} s to the outbound leg...",
            throttle_duration_sec=1.0)
        self.log_flight_state()

    def _handle_clear(self):
        """The far side of a traversal. On the way OUT, the legs home replace
        the landing.

        Inbound this is not our business at all -- it is the start of the room
        pattern and the parent handles it. Outbound the parent lands where it
        is, which is right for window_room_traverse and wrong here: the
        aircraft is outside the window with three legs still to fly.
        """
        first_leg_home = ('offset_back' if 'offset_back' in self.leg_by_name
                          else 'return')
        going_home = (self.phase == self.PHASE_OUT
                      and first_leg_home not in self.legs_done)
        if not going_home:
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
            self._begin_leg_named(first_leg_home)
            return

        self.get_logger().info(
            f"OUT: holding, {remaining:.1f} s to the '{first_leg_home}' leg.",
            throttle_duration_sec=1.0)

    # ----------------------------------------------------------- the descent

    def _begin_precision_descent(self, on_marker):
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
        hold_x, hold_y, had_hold = self.hold_x, self.hold_y, self.hold_xy
        self._begin_landing(f"centred on pad id {self.PAD_MARKER_ID}")
        if had_hold and on_marker and self.PRECISION_DESCENT:
            self.hold_x, self.hold_y = hold_x, hold_y
            self.hold_xy = True
            self.precision_descent_active = True
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
        if self.precision_descent_active:
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
                    f"Below {self.BLIND_COMMIT_ALTITUDE:.2f} m: the marker is "
                    "out of frame from here. Descent is open loop.")

        super()._handle_landing()

    # --------------------------------------------------------------- status

    def _phase_label(self):
        """The coarse phase, for the TFT and anything logging.

        The inherited version knows nothing about the legs and would label
        every one of them 'OUTSIDE', which on the way home is backwards.
        """
        if self.current_stage == self.ALT_CHANGE:
            return 'ALTITUDE'
        if self.current_stage == self.WINDOW_BACKOFF:
            return 'WINDOW_RETRY'
        if self.current_stage == self.MARKER_RETRY:
            return 'MARKER_RETRY'
        if self.current_stage in self.TILE_STAGES:
            return 'ROOM'
        leg = self.current_leg()
        if leg is not None:
            return {
                'outbound': 'TO_WINDOW',
                'offset': 'ON_AXIS',
                'offset_back': 'OFF_AXIS',
                'return': 'TO_MARKER',
                'turn': 'TO_TURN',
                'pad': 'TO_PAD',
            }[leg.name]
        return super()._phase_label()

    def publish_status(self):
        """stage|armed|altitude|xy|detail -- the format the display reads.

        The inherited version covers every stage it knows about; the leg
        stages would fall through to it with no detail of their own and leave
        the last traversal's text frozen on the screen.
        """
        if self.current_stage not in self.LEG_STAGES + self.TILE_STAGES:
            super().publish_status()
            return

        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        leg = self.current_leg()
        n = '?' if leg is None else f"{self.leg_index + 1}/{len(self.legs)}"

        if self.current_stage == self.ALT_CHANGE:
            tgt = '--' if self.alt_target is None else f"{self.alt_target:.1f}"
            detail = f"alt {tgt}"
        elif self.current_stage == self.WINDOW_BACKOFF:
            detail = f"back {self.backoff_total:.2f}"
        elif self.current_stage == self.MARKER_RETRY:
            detail = ('retrace' if self.retry_moving
                      else f"high {self.MARKER_RETRY_ALTITUDE:.1f}")
        elif self.current_stage == self.TILE_SETTLE:
            detail = 'lidar?' if not self.lidar_fix_is_fresh() else 'settling'
        elif self.current_stage == self.TILE_MOVE:
            detail = f"tile {self.tile_progress()}"
        elif self.current_stage == self.TILE_DWELL:
            left = max(0.0, self.TILE_DWELL_SECONDS
                       - (time.monotonic() - (self.tile_dwell_started or 0.0)))
            detail = f"dwell {left:.0f}s"
        elif self.current_stage == self.CRUISE:
            lp = self.local_position
            if self.moving and lp is not None and self.move_target_x is not None:
                left = math.hypot(self.move_target_x - lp.x,
                                  self.move_target_y - lp.y)
                detail = f"{n} {leg.direction[:3]}{left:.1f}"
            else:
                detail = f"{n} wait"
        elif self.current_stage == self.MARKER_ALIGN:
            err = '--' if self.marker_error is None else f"{self.marker_error:.2f}"
            detail = f"{n} err{err}"
        else:
            left = max(0.0, self.MARKER_HOLD_SECONDS - self._in_stage_for())
            detail = f"{n} hold{left:.0f}s"

        msg = String()
        msg.data = "|".join([
            self.current_stage,
            'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else (
                ('VIO' if self.LATERAL_SOURCE == 'vio' else 'FLO')
                if self.flow_is_healthy() else '---'),
            detail,
        ])
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().warning(
            "Mission FSM legs: "
            + ('; '.join(self.leg_outcomes) or 'none flown'))
        super().destroy_node()


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
        # duplicate marker id -- and the rclpy.ok() guard covers the ordinary
        # path, where the executor has already brought the context down and a
        # second shutdown() raises over the top of whatever really happened.
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
