"""
Driftless tile coverage of the dark room, flown off the 2D lidar's wall fix.

    the room is divided tiles_x by tiles_y -> each tile centre is visited in
    turn -> the aircraft holds there while doll_detect works -> it returns

WHY THE LIDAR AND NOT THE VIO, INSIDE THE ROOM
-----------------------------------------------
The VIO degrades in the dark room: it is a bare, low-texture, unlit box, which
is the worst case for visual odometry. It does NOT stop being fused -- EKF2
keeps taking it, and nothing here touches that. Vision remains the lateral
source everywhere else in the mission (see mission_fsm.flow_is_healthy).

What changes inside the room is only WHAT THE WAYPOINTS ARE MEASURED FROM.
wall_localizer fits lines to the four walls on every scan INDEPENDENTLY, so it
has no state and therefore no drift; it is re-derived from observed geometry
5.5 times a second. Planning the tiles in that frame means a tile centre stays
where it is relative to the walls however far EKF2 has wandered.

THE TWO LOOPS
-------------
    OUTER   wall_localizer -> pose_kf -> /lidar/odom_kf
            An absolute, drift-free fix in the ARENA frame, smoothed by a
            constant-velocity Kalman filter. Runs on its own, asynchronously,
            and is NOT fused into EKF2.

    INNER   this file, driven by the FSM's 20 Hz timer
            Reads the current arena fix and PX4's own local pose, derives the
            transform between them, converts the next tile centre into EKF2's
            NED frame, and hands that to the FSM's existing setpoint machinery.

The transform is re-derived every tick rather than latched once, and it SHOULD
be a constant: both streams describe the same rigid vehicle, so any movement in
it is one of them drifting relative to the other. Watching it is the single
best health check available in the room, which is why exceeding
transform_drift_max aborts rather than warns.

WHY THIS IS NOT A NODE OF ITS OWN
----------------------------------
The obvious shape is a second node publishing TrajectorySetpoint while the FSM
publishes its own. That does not work: PX4 takes the last setpoint it received,
so two publishers on one stream is a race whose winner changes with scheduling.
mission_fsm owns the offboard stream for the whole flight, so this is a helper
it drives, and the only thing it produces is an NED point for the FSM's own
_set_target() -- the same ramped, leashed carrot every other stage is flown on.

FRAMES, AND THE ONE THAT BITES
-------------------------------
    arena   ENU-like and wall-fixed: +X along the front wall, +Y to its left
            or right per the config, +Z up. What the tiles are planned in.
    NED     PX4's local frame. What is published.

The conversion is arena_to_ned(). Note the axis swap AND the sign: ENU (E,N,U)
to NED (N,E,D) is not a rotation, it is a rotation plus a mirror, and getting
it wrong produces a plan that is a mirror image of the one intended -- which
looks plausible right up until the aircraft flies at a wall.

WHICH WALL IS "FRONT"
---------------------
wall_localizer identifies SOUTH from the room itself: it is the only wall with
holes in it, and that asymmetry resolves the 90-degree ambiguity a bare square
otherwise has. That is not ours to override. front_wall/y_axis only RELABEL
that result into the frame the tiles are planned in, as a quarter-turn and an
optional mirror applied after the fix arrives.
"""

import math

from nav_msgs.msg import Odometry
from std_msgs.msg import String


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


# Quarter turns that take each named wall onto arena +X. SOUTH is the one the
# localizer names, so it is the identity.
_FRONT_WALL_YAW = {
    'south': 0.0,
    'west': math.pi / 2.0,
    'north': math.pi,
    'east': -math.pi / 2.0,
}


class LidarRoomScan:
    """Tile coverage in the arena frame, converted to NED on every tick.

    Mixed into the flight node rather than owned by it: every method here
    either reads the node's own state (local_position) or state this class
    installs on it. Nothing here publishes.
    """

    # ---- the arena frame ---------------------------------------------------
    FRONT_WALL = 'south'        # which wall becomes arena +X
    Y_AXIS = 'left'             # which side of it is +Y

    # ---- the room and the tiles -------------------------------------------
    ROOM_X = 5.41               # m, nominal only -- the localizer measures it
    ROOM_Y = 5.41
    TILES_X = 2
    TILES_Y = 2
    TILE_ORDER = 'serpentine'   # serpentine | raster
    WALL_MARGIN = 0.90          # m, minimum tile-centre distance from a wall

    # ---- flying them ------------------------------------------------------
    TILE_DWELL_SECONDS = 3.0    # s held at a centre while the detector works
    TILE_ARRIVE_EPS = 0.15      # m that counts as arrived
    TILE_SETTLE_SECONDS = 0.5   # s inside that before the dwell clock starts
    TILE_SPEED = 0.35           # m/s the virtual setpoint is walked at
    RELOCK_STANDOFF = 1.60      # m out from the window wall that the scan
                                # returns to before handing back to RELOCK.
                                # The serpentine ends in the FAR corner, and
                                # RELOCK has to see the whole aperture again
                                # -- about 1.26 x the window height of depth.
                                # Ending the scan where the tiles happen to
                                # end is what makes a relock fail.

    # ---- health -----------------------------------------------------------
    TRANSFORM_DRIFT_MAX = 0.50          # m
    TRANSFORM_YAW_DRIFT_MAX = math.radians(10.0)
    LIDAR_FIX_TIMEOUT = 2.0             # s, about ten missed scans
    LIDAR_MAX_VARIANCE = 0.25           # m^2 (0.5 m sigma). A HEALTHY fix is
                                        # floored at 0.0025 (5 cm); a
                                        # DEGENERATE one -- a reference wall
                                        # out of view -- is published at
                                        # 100.0. Anything above this is
                                        # rejected as if it never arrived.
    TRANSFORM_LOWPASS = 0.02            # per-tick blend; see update_transform
    TRANSFORM_LATCH_SECONDS = 5.0       # s of settling before the drift
                                        # reference is frozen. The low-pass
                                        # needs this long to converge; latch
                                        # earlier and its own settling reads
                                        # as drift.

    LIDAR_ODOM_TOPIC = '/lidar/odom_kf'
    LIDAR_STATUS_TOPIC = '/lidar/loc_status'

    # ------------------------------------------------------------- lifecycle

    def init_lidar_scan(self, clock):
        """Declare the parameters and wire the subscriptions.

        Called from the flight node's __init__. `clock` is a callable
        returning monotonic seconds, so this class never reaches for a time
        source of its own and the simulator can drive it.
        """
        self._now = clock

        self.FRONT_WALL = str(self.declare_parameter(
            'front_wall', self.FRONT_WALL).value).strip().lower()
        if self.FRONT_WALL not in _FRONT_WALL_YAW:
            raise SystemExit(
                f"front_wall must be one of {sorted(_FRONT_WALL_YAW)}; "
                f"got '{self.FRONT_WALL}'.")
        self.Y_AXIS = str(self.declare_parameter(
            'y_axis', self.Y_AXIS).value).strip().lower()
        if self.Y_AXIS not in ('left', 'right'):
            raise SystemExit(
                f"y_axis must be 'left' or 'right'; got '{self.Y_AXIS}'.")

        self.ROOM_X = float(self._declare_number('room_x', self.ROOM_X))
        self.ROOM_Y = float(self._declare_number('room_y', self.ROOM_Y))
        self.TILES_X = int(self.declare_parameter('tiles_x', self.TILES_X).value)
        self.TILES_Y = int(self.declare_parameter('tiles_y', self.TILES_Y).value)
        if self.TILES_X < 1 or self.TILES_Y < 1:
            raise SystemExit(
                f"tiles_x and tiles_y must both be >= 1; got "
                f"{self.TILES_X}x{self.TILES_Y}.")
        self.TILE_ORDER = str(self.declare_parameter(
            'order', self.TILE_ORDER).value).strip().lower()
        self.WALL_MARGIN = float(self._declare_number(
            'wall_margin', self.WALL_MARGIN))
        self.TILE_DWELL_SECONDS = float(self._declare_number(
            'dwell_s', self.TILE_DWELL_SECONDS))
        self.TILE_ARRIVE_EPS = float(self._declare_number(
            'arrive_eps', self.TILE_ARRIVE_EPS))
        self.TILE_SETTLE_SECONDS = float(self._declare_number(
            'settle_s', self.TILE_SETTLE_SECONDS))
        self.TILE_SPEED = float(self._declare_number('speed', self.TILE_SPEED))
        self.RELOCK_STANDOFF = float(self._declare_number(
            'relock_standoff', self.RELOCK_STANDOFF))
        self.TRANSFORM_DRIFT_MAX = float(self._declare_number(
            'transform_drift_max', self.TRANSFORM_DRIFT_MAX))
        self.TRANSFORM_YAW_DRIFT_MAX = math.radians(float(self._declare_number(
            'transform_yaw_drift_max_deg',
            math.degrees(self.TRANSFORM_YAW_DRIFT_MAX))))
        self.LIDAR_FIX_TIMEOUT = float(self._declare_number(
            'fix_timeout', self.LIDAR_FIX_TIMEOUT))
        self.LIDAR_MAX_VARIANCE = float(self._declare_number(
            'lidar_max_variance', self.LIDAR_MAX_VARIANCE))
        self.lidar_rejected = 0
        self.TRANSFORM_LATCH_SECONDS = float(self._declare_number(
            'latch_sec', self.TRANSFORM_LATCH_SECONDS))
        self.LIDAR_ODOM_TOPIC = str(self.declare_parameter(
            'lidar_odom_topic', self.LIDAR_ODOM_TOPIC).value)

        # (x, y, z, yaw) in the arena frame, newest lidar fix.
        self.lidar_fix = None
        self.lidar_fix_time = None
        self.lidar_status = ''
        # (theta, tx, ty, tz): arena -> ENU, low-passed. See update_transform.
        self.arena_tf = None
        self.arena_tf_ref = None        # first accepted, for drift measurement
        self.tile_index = 0
        self.tile_centres_arena = []
        self.tile_arrived_since = None
        self.tile_dwell_started = None
        self.tiles_done = []
        self.arena_entry = None   # arena (x, y) the room was entered at

        self.create_subscription(Odometry, self.LIDAR_ODOM_TOPIC,
                                 self.lidar_odom_callback, 10)
        self.create_subscription(String, self.LIDAR_STATUS_TOPIC,
                                 self.lidar_status_callback, 10)

    # ------------------------------------------------------------------ subs

    def lidar_odom_callback(self, msg):
        """The KF-smoothed arena fix. Relabelled into the configured frame.

        REJECTED, not stored, if its covariance says it is degenerate.

        This matters more than it looks. wall_localizer does not go quiet when
        it loses a reference wall -- it keeps publishing ON TIME, with a
        variance of 100 m^2 and a position it cannot vouch for. The LDS-01's
        range is 3.50 m and the room is 5.41 m, so at a far tile both
        reference walls are 4.06 m away and that is exactly what happens.

        A freshness check on the timestamp alone accepts that fix, and every
        tile centre would then be computed through a transform built from it.
        So a degenerate fix is treated as if it never arrived: the timestamp
        is not advanced, lidar_fix_is_fresh() goes false within
        fix_timeout, and the scan is abandoned for the way out rather than
        flown on a position nobody can vouch for.
        """
        cov = msg.pose.covariance
        try:
            var = max(float(cov[0]), float(cov[7]))
        except (IndexError, TypeError, ValueError):
            var = 0.0
        if var > self.LIDAR_MAX_VARIANCE:
            self.lidar_rejected += 1
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x, y, yaw = self._relabel(float(p.x), float(p.y), yaw)
        self.lidar_fix = (x, y, float(p.z), yaw)
        self.lidar_fix_time = self._now()

    def lidar_status_callback(self, msg):
        self.lidar_status = msg.data

    def _relabel(self, x, y, yaw):
        """Rotate (and optionally mirror) the localizer's frame onto ours.

        The localizer's +X is the SOUTH wall, because that is the wall it can
        identify from the room alone. front_wall says which wall we want +X to
        be, which is a quarter turn; y_axis says which side is +Y, which is a
        mirror. The mirror is applied after the rotation and flips the yaw
        sign, because a mirrored frame is left-handed in the plane.
        """
        th = _FRONT_WALL_YAW[self.FRONT_WALL]
        c, s = math.cos(th), math.sin(th)
        rx, ry = c * x - s * y, s * x + c * y
        ryaw = wrap_pi(yaw + th)
        if self.Y_AXIS == 'right':
            ry = -ry
            ryaw = -ryaw
        return rx, ry, wrap_pi(ryaw)

    # ------------------------------------------------------------- the fix

    def lidar_fix_is_fresh(self):
        if self.lidar_fix is None or self.lidar_fix_time is None:
            return False
        return (self._now() - self.lidar_fix_time) <= self.LIDAR_FIX_TIMEOUT

    # ------------------------------------------------------- the transform

    def update_transform(self):
        """Re-derive arena -> ENU from the two pose streams. Every tick.

        Both streams describe the same rigid vehicle, so the transform between
        them is a constant. It is recomputed rather than latched so that a
        drifting EKF2 shows up as MOVEMENT here, which is what
        transform_is_healthy() tests -- and it is low-passed hard, because the
        constant is what we want and per-scan fit noise is not.
        """
        lp = getattr(self, 'local_position', None)
        if self.lidar_fix is None or lp is None:
            return
        if not (lp.xy_valid and lp.z_valid):
            return

        # EKF2 NED (N, E, D) -> ENU (E, N, U). Not a rotation: a rotation and
        # a mirror. Getting this wrong gives a mirror-image plan that looks
        # plausible until the aircraft flies at a wall.
        pe = (lp.y, lp.x, -lp.z)
        # heading is NED, clockwise from north; ENU yaw is ccw from east.
        psi_ekf = wrap_pi(math.pi / 2.0 - lp.heading)
        theta = wrap_pi(psi_ekf - self.lidar_fix[3])

        c, s = math.cos(theta), math.sin(theta)
        tx = pe[0] - (c * self.lidar_fix[0] - s * self.lidar_fix[1])
        ty = pe[1] - (s * self.lidar_fix[0] + c * self.lidar_fix[1])
        tz = pe[2] - self.lidar_fix[2]

        if self.arena_tf is None:
            self.arena_tf = (theta, tx, ty, tz)
            return
        a = self.TRANSFORM_LOWPASS
        o = self.arena_tf
        self.arena_tf = (wrap_pi(o[0] + a * wrap_pi(theta - o[0])),
                         o[1] + a * (tx - o[1]),
                         o[2] + a * (ty - o[2]),
                         o[3] + a * (tz - o[3]))

    def latch_transform_reference(self):
        """Freeze the transform the drift check measures against.

        Called at the END of the settle, never at the first sample. The
        low-pass in update_transform takes a few seconds to converge, and a
        reference captured before it has done so turns the filter's own
        settling into apparent drift -- which aborts the scan on a perfectly
        healthy aircraft. room_mission.py latches after latch_sec for exactly
        this reason; this is the same thing.
        """
        if self.arena_tf is not None:
            self.arena_tf_ref = self.arena_tf
        return self.arena_tf_ref is not None

    def transform_is_healthy(self):
        """Has the transform moved since it was latched?

        It should not have. Movement means one pose stream is drifting against
        the other, and every tile centre is computed through this -- so a
        transform that has walked half a metre is a plan that has walked half
        a metre, silently.
        """
        if self.arena_tf is None:
            return False
        if self.arena_tf_ref is None:
            # Not latched yet: still settling, nothing to compare against.
            return True
        th, tx, ty, _ = self.arena_tf
        rth, rtx, rty, _ = self.arena_tf_ref
        if math.hypot(tx - rtx, ty - rty) > self.TRANSFORM_DRIFT_MAX:
            return False
        return abs(wrap_pi(th - rth)) <= self.TRANSFORM_YAW_DRIFT_MAX

    def arena_to_ned(self, x, y, z=None):
        """Arena point -> EKF2 local NED, for a TrajectorySetpoint."""
        th, tx, ty, tz = self.arena_tf
        c, s = math.cos(th), math.sin(th)
        e = c * x - s * y + tx
        n = s * x + c * y + ty
        if z is None:
            return n, e, None
        return n, e, -(z + tz)

    def ned_to_arena(self, n, e):
        """The inverse, for reporting where the aircraft actually is."""
        th, tx, ty, _ = self.arena_tf
        c, s = math.cos(th), math.sin(th)
        dx, dy = e - tx, n - ty
        return c * dx + s * dy, -s * dx + c * dy

    # ----------------------------------------------------------- the tiles

    def build_tile_centres(self):
        """Tile centres in the arena frame, in visiting order.

        The arena origin is the room centre, so the room spans +/-room/2 and a
        tile centre is at the middle of its cell. Centres are then pulled in
        by wall_margin: with 2x2 in a 5.41 m room they are already 1.35 m off
        each wall so nothing moves, but a 4x4 would otherwise plan a centre
        0.68 m from a wall and this is what stops it.
        """
        cells = []
        for iy in range(self.TILES_Y):
            row = []
            for ix in range(self.TILES_X):
                cx = (ix + 0.5) * self.ROOM_X / self.TILES_X - self.ROOM_X / 2.0
                cy = (iy + 0.5) * self.ROOM_Y / self.TILES_Y - self.ROOM_Y / 2.0
                lim_x = max(0.0, self.ROOM_X / 2.0 - self.WALL_MARGIN)
                lim_y = max(0.0, self.ROOM_Y / 2.0 - self.WALL_MARGIN)
                row.append((max(-lim_x, min(lim_x, cx)),
                            max(-lim_y, min(lim_y, cy))))
            # Serpentine: reverse every other row so consecutive tiles are
            # adjacent. Raster order flies the full width of the room between
            # the end of one row and the start of the next.
            if self.TILE_ORDER == 'serpentine' and iy % 2 == 1:
                row.reverse()
            cells.extend(row)
        self.tile_centres_arena = cells
        return cells

    def current_tile(self):
        if 0 <= self.tile_index < len(self.tile_centres_arena):
            return self.tile_centres_arena[self.tile_index]
        return None

    def tile_progress(self):
        return f"{min(self.tile_index + 1, len(self.tile_centres_arena))}/" \
               f"{len(self.tile_centres_arena)}"

    def scan_summary(self):
        if not self.tiles_done:
            return 'no tiles scanned'
        return '; '.join(self.tiles_done)

    def relock_station_arena(self):
        """Where to sit before handing back to RELOCK.

        RELOCK_STANDOFF out from the front wall, and laterally ON THE WINDOW
        AXIS -- which is not the middle of the room.

        The window is rarely centred in its wall. Returning to the room's
        centre line leaves the aperture off to one side, and at the short
        range RELOCK needs, an aperture a little off-axis is an aperture with
        one edge outside the frame: truncated, which the estimator refuses
        outright. The scan then ends with a relock that cannot succeed.

        The entry point solves it for free. The aircraft got into this room by
        flying THROUGH the window, so the arena x it entered at IS the window
        axis, measured rather than assumed. Falling back to the room centre
        only if the entry was never recorded.

        The front wall is at -Y by construction: +X runs ALONG it, so the
        perpendicular is Y, and the room spans +/-room_y/2.
        """
        x = 0.0 if self.arena_entry is None else self.arena_entry[0]
        lim = self.ROOM_X / 2.0 - self.WALL_MARGIN
        x = max(-lim, min(lim, x))
        y = -(self.ROOM_Y / 2.0) + self.RELOCK_STANDOFF
        return x, max(-(self.ROOM_Y / 2.0), y)
