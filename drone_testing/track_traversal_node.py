#!/usr/bin/env python3
"""
Track Traversal Node
=====================
Offboard takeoff, then a forward traversal *along* the textured course
track, steered entirely by the down-camera track detector — not by a fixed
forward-velocity target along a latched heading like `forward_traversal`.

`forward_traversal` commands a constant `cruise_speed` along `locked_yaw`,
so the path it flies is only as straight (and only as on-track) as the
heading it happened to latch at arming. This node instead closes the loop
on the track itself every tick:

  * yaw     nulls the track centreline's tilt from vertical in-frame, so the
            nose follows the track, not a latched heading;
  * roll    (body +y velocity) nulls the centreline's lateral offset, so the
            drone stays over the middle of the track;
  * forward (body +x velocity) is `max_forward_speed` (default 0.5 m/s)
            while a track is detected *and* vertical in the image
            (|angle| <= `forward_vertical_tol_deg`, with
            `forward_vertical_hyst_deg` hysteresis), zero otherwise — the
            drone yaws/centres onto a tilted track before moving on, and
            doesn't fly forward at all with no track in view. With
            `align_before_traverse:=true` it is instead scaled down as the
            offset/tilt grow. Either way it is slew-limited
            (`max_forward_accel` / `max_forward_decel`).

All horizontal commands respect `max_forward_speed`, `max_roll_vel`,
`max_yaw_rate`, and an overall `max_horizontal_vel` cap on the combined
(vx, vy) norm — lateral correction takes priority, forward speed gets
whatever's left under the cap. The law itself lives in
`traversal_utils/track_control.py`, shared with the offline video replay
`src/track_detection_standalone.py`.

Detection comes from `landing_guidance.track_detector.make_track_detector`,
selected by `detector_mode`: 'grey_background' (default — track against a
greyish floor, boundaries snapped to its distinct edges) or 'red_flanks'
(track between red panels, the detector `track_centering` and
`obstacle_traversal` use). Both report the same centreline offset/angle,
and the roll/yaw law is the one those nodes use. The optional lateral search below is
`obstacle_traversal`'s ROLL_SEARCH (including its lose-then-regain debounce
for the takeoff pad's own flanking texture), generalised to either
direction.

Coordinate system: PX4 NED. Camera mounting assumption is the same as every
other track/marker node in this workspace (landing_guidance README "Camera
mounting assumption"): image top -> body +x (nose), image right -> body +y
(right). If the vehicle corrects the wrong way, flip the sign of `kp_roll`
or `kp_yaw` rather than re-deriving the geometry.

Mission profile
---------------
  INIT ─▶ TAKEOFF ─▶ START_ALIGN ─▶ TRAVERSE ─▶ END_ALIGN ─▶ HOLD | LAND
                     (id 0, if seen)     │  ▲    (id 2)
                                         ▼  │
                                      TRACK_END

  INIT        Streams position setpoints (PX4 needs a live stream before it
              accepts OFFBOARD) and waits for the external go signal: the
              vehicle ARMED while in OFFBOARD (RC/QGC). auto_arm:=true
              (SITL) requests OFFBOARD + ARM itself. Already flying when
              that happens -> skips the climb.
  TAKEOFF     Climb straight up to `altitude` (2.5 m above ground). The
              moment it is reached:
  START_ALIGN If start marker id 0 is in view (confirmed within
              `start_marker_timeout_s`): centre on it (|px|,|py| <=
              `marker_center_tol_px`) and yaw to `start_marker_yaw_deg`
              (0 deg, +/- `marker_yaw_tol_deg`), held for
              `marker_stabilise_s`. Not in view -> straight to TRAVERSE.
  TRAVERSE    Track detection + following (see above). Forward only while
              the track is vertical in the image; if no track has been
              found `blind_forward_after_s` (3 s) after TRAVERSE starts, it
              flies forward on its heading until one comes into view. Ends
              when finish marker id 2 is confirmed in view.
  TRACK_END   The track ran out (end-of-track / lost / max distance) before
              id 2 was seen: hover, keep watching for id 2.
  END_ALIGN   Centre on id 2 and yaw to `end_marker_yaw_deg` (0 deg),
              same tolerances, held for `marker_stabilise_s`; hovers if the
              marker drops out of view. Then HOLD (or LAND with
              `land_at_end`).
  (align_before_traverse:=true / search_direction left|right bring back the
  older STABILISE -> ACQUIRE -> ALIGN stages before TRAVERSE.)
  HOLD       Terminal: position hold at the latched end point.
  LAND       Terminal (`land_at_end:=true`): PX4 AUTO_LAND on the spot;
             offboard streaming stops.
  ABORTED    Terminal: the vehicle left OFFBOARD or disarmed after the
             mission started (pilot took over, or a PX4 failsafe). The node
             never re-requests OFFBOARD; it keeps streaming a hold at the
             current position so that if the pilot switches back to
             OFFBOARD the vehicle just hovers.

Altitude is held throughout ACQUIRE/ALIGN/TRAVERSE by a mixed
TrajectorySetpoint (x/y velocity, z position — see
`PX4OffboardLink.publish_altitude_held_velocity_setpoint`), so PX4's own
position controller (with integral action) owns altitude.

Hardware notes
--------------
  * The image subscription uses sensor-data QoS (best-effort), which
    connects to both best-effort hardware camera drivers and the reliable
    ros_gz_image bridge.
  * Image processing runs in its own callback group on a multi-threaded
    executor, so a slow frame can't stall the 20 Hz setpoint stream PX4
    needs to stay in OFFBOARD.
  * A camera that stops publishing counts as "track lost"
    (`image_timeout_s`) — the drone stops instead of flying on a stale
    reading.
  * Take off facing roughly along the track: the detector rejects a
    centreline tilted more than `detector_max_angle_from_vertical_deg`.
"""

import math
import os
import threading

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

from landing_guidance.aruco_utils import (
    ArucoMarkerDetector,
    UnknownDictionaryArucoDetector,
)
from landing_guidance.mjpeg_server import MjpegServer
from landing_guidance.px4_offboard_utils import (
    PX4_QOS,
    PX4OffboardLink,
    body_to_ned,
)
from landing_guidance.track_detector import (
    TRACK_DETECTOR_PARAMS,
    draw,
    make_track_detector,
)

from traversal_utils import track_control

STATE_INIT = 'INIT'
STATE_TAKEOFF = 'TAKEOFF'
STATE_STABILISE = 'STABILISE'
STATE_START_ALIGN = 'START_ALIGN'
STATE_ACQUIRE = 'ACQUIRE'
STATE_ALIGN = 'ALIGN'
STATE_TRAVERSE = 'TRAVERSE'
STATE_TRACK_END = 'TRACK_END'
STATE_END_ALIGN = 'END_ALIGN'
STATE_HOLD = 'HOLD'
STATE_LAND = 'LAND'
STATE_ABORTED = 'ABORTED'

TIMER_PERIOD = 0.05          # seconds — 20 Hz setpoint stream
SETPOINT_WARMUP_TICKS = 30   # ~1.5 s of setpoints before requesting OFFBOARD


class TrackTraversalNode(Node):
    """Takeoff, align onto the textured track, and fly along it."""

    def __init__(self):
        super().__init__('track_traversal')

        self._declare_parameters()
        self._read_parameters()

        self.bridge = CvBridge()
        self.track_detector = make_track_detector(self.detector_params)
        if self.aruco_dictionary.lower() == 'auto_5x5':
            self.marker_detector = UnknownDictionaryArucoDetector()
        else:
            self.marker_detector = ArucoMarkerDetector(self.aruco_dictionary)
        self.px4 = PX4OffboardLink(self)

        # ── PX4 telemetry state ──────────────────────────────────────────
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.current_yaw = 0.0
        self.have_local_position = False

        # ── Vision state (written by _image_cb, read by the timer) ───────
        self.latest_detection = None
        self.last_image_time = None
        self.frame_h = 0
        self.latest_debug_frame = None   # shown by main()'s window loop
        self.frame_w = 0
        self.latest_markers = {}         # id -> MarkerDetection, this frame
        self.marker_hits = {}            # id -> consecutive frames seen
        self.marker_status = None        # (ex_px, ey_px, yaw_err_deg) while aligning
        self._first_image_logged = False

        # ── FSM state ────────────────────────────────────────────────────
        self.state = STATE_INIT
        self.setpoint_ticks = 0
        self.mission_started = False
        self.home_xyz = None          # (x, y, z) latched at arming
        self.home_yaw = 0.0
        self.target_z = 0.0           # NED z to fly at (home z - altitude)
        self.hold_xyz = None
        self.hold_yaw = 0.0
        self.stable_since = None
        self.lost_ticks = 0
        self.track_found_once = False
        self.search_seen_ok = False
        self.search_lost_once = False
        self.search_started = None
        self.search_start_xy = None
        self.traverse_start_xy = None
        self.vx_cmd = 0.0
        self.forward_enabled = False
        self.blind_forward = False
        self.traverse_started = None
        self.phase_started = None
        self.marker_seen_once = False
        self.marker_lost_since = None
        self._last_arm_request = None
        self._last_land_request = None
        self._log_tick = 0

        # ── Publishers ───────────────────────────────────────────────────
        self.state_pub = self.create_publisher(String, '/track_traversal/state', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/track_traversal/debug_image', 10)

        self.stream = None
        if self.debug_stream_port > 0:
            try:
                self.stream = MjpegServer(
                    self.debug_stream_port, self.debug_stream_bind,
                    title='track_traversal debug')
                self.get_logger().info(
                    f'Debug MJPEG stream on http://{self.debug_stream_bind}:'
                    f'{self.debug_stream_port}  (laptop: ssh -L '
                    f'{self.debug_stream_port}:localhost:{self.debug_stream_port} '
                    f'<user>@<companion computer>, then open http://localhost:'
                    f'{self.debug_stream_port})')
            except OSError as e:
                self.get_logger().error(
                    f'Could not start debug stream on port '
                    f'{self.debug_stream_port}: {e}')

        # ── Subscribers ──────────────────────────────────────────────────
        # Vision gets its own callback group so a slow frame never delays
        # the setpoint timer (MultiThreadedExecutor in main()).
        control_group = MutuallyExclusiveCallbackGroup()
        vision_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb, PX4_QOS, callback_group=control_group)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._status_cb,
            PX4_QOS, callback_group=control_group)
        self.create_subscription(
            Image, self.camera_topic, self._image_cb, qos_profile_sensor_data,
            callback_group=vision_group)

        self.timer = self.create_timer(
            TIMER_PERIOD, self._timer_cb, callback_group=control_group)

        self.get_logger().info(
            f'Track Traversal Node started — camera={self.camera_topic}, '
            f'altitude={self.altitude} m, max_forward_speed={self.max_forward_speed} m/s, '
            f'max_horizontal_vel={self.max_horizontal_vel} m/s, '
            f'search_direction={self.search_direction}, auto_arm={self.auto_arm}, '
            f'land_at_end={self.land_at_end}')

    # ══════════════════════════════════════════════════════════════════════
    # Parameters
    # ══════════════════════════════════════════════════════════════════════

    def _declare_parameters(self):
        self.declare_parameter('camera_topic', '/camera_down')
        # true: SITL/bench — node requests OFFBOARD + ARM itself.
        # false: hardware — pilot arms and switches to OFFBOARD.
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('arm_retry_s', 1.0)
        self.declare_parameter('debug_stream_port', 8083)   # 0 disables
        self.declare_parameter('debug_stream_bind', '127.0.0.1')
        # Live OpenCV window of the debug image (skipped automatically when
        # there is no display, e.g. a headless companion computer).
        self.declare_parameter('show_window', True)

        # ── Takeoff / altitude ───────────────────────────────────────────
        self.declare_parameter('altitude', 2.5)             # EDIT ME — m above ground
        # Above this height at start the vehicle counts as already flying:
        # no takeoff climb, forward traversal begins immediately.
        self.declare_parameter('airborne_height_m', 0.5)
        self.declare_parameter('altitude_tolerance_m', 0.15)
        self.declare_parameter('stabilise_after_takeoff_s', 2.0)

        # ── Speed limits ─────────────────────────────────────────────────
        self.declare_parameter('max_forward_speed', 0.5)    # EDIT ME — body +x speed
        self.declare_parameter('max_forward_accel', 0.15)   # m/s^2 ramp-up
        self.declare_parameter('max_forward_decel', 0.6)    # m/s^2 ramp-down
        self.declare_parameter('max_horizontal_vel', 0.6)   # cap on |(vx, vy)|
        # false: no hover-and-align stage — traversal starts straight away
        # and flies forward at max_forward_speed whenever a track is
        # detected and vertical in the image (forward_vertical_tol_deg),
        # yawing/centring onto it otherwise. true: the original ALIGN stage
        # + alignment-scaled speed.
        self.declare_parameter('align_before_traverse', False)
        self.declare_parameter('forward_vertical_tol_deg', 10.0)
        self.declare_parameter('forward_vertical_hyst_deg', 3.0)
        # No track found this long after traversal starts -> fly forward
        # (heading held) until it appears. 0 disables.
        self.declare_parameter('blind_forward_after_s', 3.0)

        # ── ACQUIRE — optional lateral search (obstacle_traversal's
        # ROLL_SEARCH generalised to either direction) ────────────────────
        self.declare_parameter('search_direction', 'none')  # 'none' | 'right' | 'left'
        self.declare_parameter('search_speed', 0.25)
        self.declare_parameter('search_require_lose_regain', True)
        self.declare_parameter('search_timeout_s', 20.0)
        self.declare_parameter('search_max_distance_m', 4.0)  # EDIT ME — arena-safe limit

        # ── track detector (landing_guidance/track_detector.py): every
        # detector_* parameter comes from its shared table, so this node
        # and src/track_detection_standalone.py always agree. detector_mode:
        # 'grey_background' (track against a greyish floor, distinct edges)
        # or 'red_flanks' (track between red panels). EDIT ME against the
        # debug view. ────────────────────────────────────────────────────
        for name, default in TRACK_DETECTOR_PARAMS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('image_timeout_s', 0.5)

        # ── Roll / yaw — same law as track_centering / obstacle_traversal.
        # EDIT ME: flip a sign if the vehicle corrects the wrong way. ─────
        self.declare_parameter('kp_roll', 0.6)
        self.declare_parameter('max_roll_vel', 0.3)
        self.declare_parameter('kp_yaw', 0.8)
        self.declare_parameter('max_yaw_rate', 0.5)

        # ── ALIGN ────────────────────────────────────────────────────────
        self.declare_parameter('center_tolerance_norm', 0.06)
        self.declare_parameter('angle_tolerance_deg', 4.0)
        self.declare_parameter('align_time_s', 2.0)

        # ── TRAVERSE — forward speed scales 1 -> 0 as |offset| / |angle|
        # approach these cutoffs. ─────────────────────────────────────────
        self.declare_parameter('forward_offset_cutoff_norm', 0.35)
        self.declare_parameter('forward_angle_cutoff_deg', 20.0)
        self.declare_parameter('lost_grace_s', 0.5)
        self.declare_parameter('end_lost_s', 1.5)
        self.declare_parameter('end_track_top_frac', 0.5)   # 0 disables
        self.declare_parameter('min_traverse_distance_m', 1.0)
        self.declare_parameter('max_traverse_distance_m', 0.0)  # 0 = unlimited
        self.declare_parameter('land_at_end', False)

        # ── ArUco markers: start pad (align before following the track) and
        # finish marker (stop the traversal, align, hold). angle 0 = marker
        # upright in the image (drone squared up to it), 180 = facing the
        # opposite way. EDIT ME: IDs / dictionary for the arena. ─────────
        self.declare_parameter('aruco_dictionary', 'auto_5x5')   # or e.g. DICT_5X5_250
        self.declare_parameter('start_marker_id', 0)
        self.declare_parameter('start_marker_yaw_deg', 0.0)
        # Start marker not confirmed within this after reaching altitude (or
        # lost this long while aligning) -> go straight to track following.
        self.declare_parameter('start_marker_timeout_s', 0.5)
        self.declare_parameter('end_marker_id', 2)
        self.declare_parameter('end_marker_yaw_deg', 0.0)
        self.declare_parameter('marker_yaw_tol_deg', 4.0)
        self.declare_parameter('marker_center_tol_px', 20.0)
        self.declare_parameter('marker_stabilise_s', 2.0)
        self.declare_parameter('marker_confirm_frames', 3)
        self.declare_parameter('kp_marker_center', 0.5)
        self.declare_parameter('max_marker_vel', 0.3)
        self.declare_parameter('kp_marker_yaw', 0.8)

    def _read_parameters(self):
        gp = lambda name: self.get_parameter(name).value  # noqa: E731

        self.camera_topic = str(gp('camera_topic'))
        self.auto_arm = bool(gp('auto_arm'))
        self.arm_retry_s = float(gp('arm_retry_s'))
        self.debug_stream_port = int(gp('debug_stream_port'))
        self.debug_stream_bind = str(gp('debug_stream_bind'))
        self.show_window = bool(gp('show_window'))

        self.altitude = abs(float(gp('altitude')))
        self.altitude_tolerance_m = float(gp('altitude_tolerance_m'))
        self.stabilise_after_takeoff_s = float(gp('stabilise_after_takeoff_s'))
        self.airborne_height_m = float(gp('airborne_height_m'))

        self.max_forward_speed = abs(float(gp('max_forward_speed')))
        self.max_forward_accel = abs(float(gp('max_forward_accel')))
        self.max_forward_decel = abs(float(gp('max_forward_decel')))
        self.max_horizontal_vel = abs(float(gp('max_horizontal_vel')))
        self.align_before_traverse = bool(gp('align_before_traverse'))
        self.forward_vertical_tol_deg = float(gp('forward_vertical_tol_deg'))
        self.forward_vertical_hyst_deg = float(gp('forward_vertical_hyst_deg'))
        self.blind_forward_after_s = float(gp('blind_forward_after_s'))

        self.search_direction = str(gp('search_direction')).strip().lower()
        if self.search_direction not in ('none', 'right', 'left'):
            self.get_logger().warn(
                f'Unknown search_direction {self.search_direction!r} — using "none"')
            self.search_direction = 'none'
        self.search_speed = abs(float(gp('search_speed')))
        self.search_require_lose_regain = bool(gp('search_require_lose_regain'))
        self.search_timeout_s = float(gp('search_timeout_s'))
        self.search_max_distance_m = float(gp('search_max_distance_m'))

        self.detector_params = {name: gp(name) for name in TRACK_DETECTOR_PARAMS}
        self.image_timeout_s = float(gp('image_timeout_s'))

        self.kp_roll = float(gp('kp_roll'))
        self.max_roll_vel = abs(float(gp('max_roll_vel')))
        self.kp_yaw = float(gp('kp_yaw'))
        self.max_yaw_rate = abs(float(gp('max_yaw_rate')))

        self.center_tolerance_norm = float(gp('center_tolerance_norm'))
        self.angle_tolerance_deg = float(gp('angle_tolerance_deg'))
        self.align_time_s = float(gp('align_time_s'))

        self.forward_offset_cutoff_norm = max(1e-3, float(gp('forward_offset_cutoff_norm')))
        self.forward_angle_cutoff_deg = max(1e-3, float(gp('forward_angle_cutoff_deg')))
        self.lost_grace_s = float(gp('lost_grace_s'))
        self.end_lost_s = max(self.lost_grace_s, float(gp('end_lost_s')))
        self.end_track_top_frac = float(gp('end_track_top_frac'))
        self.min_traverse_distance_m = float(gp('min_traverse_distance_m'))
        self.max_traverse_distance_m = float(gp('max_traverse_distance_m'))
        self.land_at_end = bool(gp('land_at_end'))

        self.aruco_dictionary = str(gp('aruco_dictionary'))
        self.start_marker_id = int(gp('start_marker_id'))
        self.start_marker_yaw_deg = float(gp('start_marker_yaw_deg'))
        self.start_marker_timeout_s = float(gp('start_marker_timeout_s'))
        self.end_marker_id = int(gp('end_marker_id'))
        self.end_marker_yaw_deg = float(gp('end_marker_yaw_deg'))
        self.marker_yaw_tol_deg = float(gp('marker_yaw_tol_deg'))
        self.marker_center_tol_px = float(gp('marker_center_tol_px'))
        self.marker_stabilise_s = float(gp('marker_stabilise_s'))
        self.marker_confirm_frames = max(1, int(gp('marker_confirm_frames')))
        self.kp_marker_center = float(gp('kp_marker_center'))
        self.max_marker_vel = abs(float(gp('max_marker_vel')))
        self.kp_marker_yaw = float(gp('kp_marker_yaw'))

        self.lost_grace_ticks = max(1, int(round(self.lost_grace_s / TIMER_PERIOD)))
        self.end_lost_ticks = max(1, int(round(self.end_lost_s / TIMER_PERIOD)))

    # ══════════════════════════════════════════════════════════════════════
    # PX4 telemetry callbacks
    # ══════════════════════════════════════════════════════════════════════

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self.local_position = msg
        self.have_local_position = True
        if msg.heading_good_for_control:
            self.current_yaw = float(msg.heading)

    def _status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg

    @property
    def _is_offboard(self):
        return self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    @property
    def _is_armed(self):
        return self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _localisation_healthy(self):
        pos = self.local_position
        return bool(self.have_local_position and pos.xy_valid and pos.z_valid)

    def _height_agl(self):
        pos = self.local_position
        if pos.dist_bottom_valid:
            return float(pos.dist_bottom)
        return -float(pos.z)

    def _xy(self):
        return float(self.local_position.x), float(self.local_position.y)

    def _dist_from(self, xy):
        if xy is None:
            return 0.0
        x, y = self._xy()
        return math.hypot(x - xy[0], y - xy[1])

    # ══════════════════════════════════════════════════════════════════════
    # Vision
    # ══════════════════════════════════════════════════════════════════════

    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        if not self._first_image_logged:
            h, w = frame.shape[:2]
            self.get_logger().info(
                f'First image received on {self.camera_topic} ({w}x{h}) — '
                f'camera subscription OK')
            self._first_image_logged = True

        self.frame_h, self.frame_w = frame.shape[:2]
        markers = self.marker_detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        self.marker_hits = {mid: self.marker_hits.get(mid, 0) + 1 for mid in markers}
        self.latest_markers = markers
        self.latest_detection = self.track_detector(frame)
        self.last_image_time = self.get_clock().now()
        self._publish_debug_image(frame)

    def _track(self):
        """Current locked track reading, or None if there isn't one — no
        lock, not yet confirmed, or the camera has gone quiet."""
        if self.last_image_time is None:
            return None
        age = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        if age > self.image_timeout_s:
            return None
        det = self.latest_detection
        if det is None or not det['ok']:
            return None
        return det

    def _marker(self, marker_id):
        """Marker `marker_id` if seen for marker_confirm_frames consecutive
        frames and the camera is live, else None."""
        if self.last_image_time is None:
            return None
        age = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        if age > self.image_timeout_s:
            return None
        if self.marker_hits.get(marker_id, 0) < self.marker_confirm_frames:
            return None
        return self.latest_markers.get(marker_id)

    def _draw_markers(self, img):
        target = {STATE_START_ALIGN: self.start_marker_id,
                  STATE_END_ALIGN: self.end_marker_id}.get(self.state)
        h, w = img.shape[:2]
        if target is not None:   # the centring tolerance box
            t = int(self.marker_center_tol_px)
            cv2.rectangle(img, (w // 2 - t, h // 2 - t), (w // 2 + t, h // 2 + t),
                          (0, 255, 255), 1)
        for mid, m in self.latest_markers.items():
            col = (0, 255, 255) if mid == target else (200, 200, 200)
            cv2.polylines(img, [m.corners.astype(int)], True, col, 2)
            top = tuple(int(v) for v in m.corners[0])
            cv2.line(img, top, tuple(int(v) for v in m.corners[1]), (0, 0, 255), 3)
            cv2.putText(img, f'id {mid}  {m.angle_deg:+.0f} deg',
                        (int(m.center_x) + 10, int(m.center_y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, f'id {mid}  {m.angle_deg:+.0f} deg',
                        (int(m.center_x) + 10, int(m.center_y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 1, cv2.LINE_AA)
        return img

    def _publish_debug_image(self, frame):
        extra = [
            f'STATE: {self.state}',
            f'reason: {self.track_detector.reason}',
            f'vx {self.vx_cmd:.2f}/{self.max_forward_speed:.2f} m/s',
        ]
        if self.state == STATE_TRAVERSE:
            extra.append(f'traversed {self._dist_from(self.traverse_start_xy):.2f} m  '
                         f'(watching for marker id {self.end_marker_id})')
        if self.state in (STATE_START_ALIGN, STATE_END_ALIGN) and self.marker_status:
            ex, ey, yerr = self.marker_status
            held = (0.0 if self.stable_since is None else
                    (self.get_clock().now() - self.stable_since).nanoseconds / 1e9)
            extra.append(f'marker ex {ex:+.0f} ey {ey:+.0f} px '
                         f'(tol {self.marker_center_tol_px:.0f})'
                         f'  yaw err {yerr:+.1f} deg (tol {self.marker_yaw_tol_deg:.0f})'
                         f'  stable {held:.1f}/{self.marker_stabilise_s:.1f} s')
        annotated = draw(frame, self.latest_detection, detector=self.track_detector,
                         extra_lines=extra)
        self._draw_markers(annotated)
        self.latest_debug_frame = annotated
        if self.stream is not None:
            self.stream.push(annotated)
        try:
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')

    # ══════════════════════════════════════════════════════════════════════
    # Setpoint helpers
    # ══════════════════════════════════════════════════════════════════════

    def _hold_position(self, x, y, z, yaw):
        self.px4.publish_offboard_mode(position=True)
        self.px4.publish_position_setpoint(x, y, z, yaw)

    def _velocity_at_altitude(self, vx_body, vy_body, yawspeed=0.0):
        """Body-frame horizontal velocity at `target_z`, under every speed
        limit. Lateral correction has priority: forward speed is trimmed so
        the combined horizontal speed never exceeds `max_horizontal_vel`."""
        vx_body, vy_body, yawspeed = track_control.limit_command(
            vx_body, vy_body, yawspeed, self.max_forward_speed, self.max_roll_vel,
            self.max_horizontal_vel, self.max_yaw_rate)

        vx_ned, vy_ned = body_to_ned(self.current_yaw, vx_body, vy_body)
        self.px4.publish_offboard_mode(position=True, velocity=True)
        self.px4.publish_altitude_held_velocity_setpoint(
            vx_ned, vy_ned, self.target_z, yawspeed)

    def _track_correction(self, det):
        """(vy_body, yawspeed) — identical law to track_centering_node."""
        return track_control.track_correction(
            det, self.kp_roll, self.max_roll_vel, self.kp_yaw, self.max_yaw_rate)

    def _is_aligned(self, det):
        return (abs(det['offset_norm']) < self.center_tolerance_norm
                and abs(det['angle_deg']) < self.angle_tolerance_deg)

    def _alignment_factor(self, det):
        """1.0 when perfectly on the track, falling linearly to 0.0 as the
        offset or the tilt reaches its cutoff — the forward-speed scale."""
        return track_control.alignment_factor(
            det, self.forward_offset_cutoff_norm, self.forward_angle_cutoff_deg)

    def _track_end_below(self, det):
        """True once the top of the fitted track centreline (image top =
        ahead) has dropped to `end_track_top_frac` of the frame height, i.e.
        there is no more track ahead of the drone."""
        return track_control.track_end_below(det, self.end_track_top_frac, self.frame_h)

    def _slew_forward(self, target):
        self.vx_cmd = track_control.slew(
            self.vx_cmd, target, self.max_forward_accel, self.max_forward_decel, TIMER_PERIOD)
        return self.vx_cmd

    def _publish_state_str(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════
    # FSM
    # ══════════════════════════════════════════════════════════════════════

    def _timer_cb(self):
        self._publish_state_str()

        if self.state == STATE_LAND:
            self._tick_land()   # PX4 owns the vehicle; no offboard streaming
            return

        if self.mission_started and self.state != STATE_ABORTED:
            if not self._is_armed or not self._is_offboard:
                self._abort('disarmed' if not self._is_armed else
                            f'left OFFBOARD (nav_state={self.vehicle_status.nav_state})')

        handler = {
            STATE_INIT: self._tick_init,
            STATE_TAKEOFF: self._tick_takeoff,
            STATE_STABILISE: self._tick_stabilise,
            STATE_ACQUIRE: self._tick_acquire,
            STATE_ALIGN: self._tick_align,
            STATE_START_ALIGN: self._tick_start_align,
            STATE_TRAVERSE: self._tick_traverse,
            STATE_TRACK_END: self._tick_track_end,
            STATE_END_ALIGN: self._tick_end_align,
            STATE_HOLD: self._tick_hold,
            STATE_ABORTED: self._tick_aborted,
        }[self.state]
        handler()

        self._log_tick += 1
        if self._log_tick % 40 == 0:   # ~every 2 s
            det = self._track()
            track = ('none (%s)' % self.track_detector.reason if det is None else
                     f"offset={det['offset_norm']:+.2f} angle={det['angle_deg']:+.1f}deg")
            alt = self._height_agl()
            self.get_logger().info(
                f'[{self.state}] alt={alt:.2f}m yaw={math.degrees(self.current_yaw):.1f}deg '
                f'vx={self.vx_cmd:.2f} track: {track}')

    def _abort(self, why):
        self.get_logger().error(
            f'Vehicle {why} — mission ABORTED. Not re-requesting OFFBOARD; '
            f'streaming a hold at the current position only.')
        self.vx_cmd = 0.0
        self.state = STATE_ABORTED

    def _tick_init(self):
        pos = self.local_position
        self._hold_position(pos.x, pos.y, pos.z, self.current_yaw)
        self.setpoint_ticks += 1

        # The warm-up stream only matters before PX4 is asked to enter
        # OFFBOARD — if it's already armed + OFFBOARD, start right away.
        already_flying_offboard = self._is_armed and self._is_offboard
        if self.setpoint_ticks < SETPOINT_WARMUP_TICKS and not already_flying_offboard:
            return
        if not self._localisation_healthy():
            if self.setpoint_ticks % 40 == 0:
                self.get_logger().warn(
                    'Waiting for a valid local position estimate '
                    '(xy_valid/z_valid still false)')
            return

        if not (self._is_armed and self._is_offboard):
            if self.auto_arm:
                now = self.get_clock().now()
                if (self._last_arm_request is None
                        or (now - self._last_arm_request).nanoseconds / 1e9 >= self.arm_retry_s):
                    self._last_arm_request = now
                    if not self._is_offboard:
                        self.px4.engage_offboard()
                    if not self._is_armed:
                        self.px4.arm()
            elif self.setpoint_ticks % 100 == 0:
                self.get_logger().info(
                    'Ready — waiting for pilot to ARM and switch to OFFBOARD '
                    f'(armed={self._is_armed}, offboard={self._is_offboard})')
            return

        self.home_xyz = (float(pos.x), float(pos.y), float(pos.z))
        self.home_yaw = self.current_yaw
        self.mission_started = True

        # Altitude is referenced to the ground, not to wherever the node
        # happened to start: rangefinder when valid, otherwise the current z
        # if on the ground, else the EKF origin (PX4 boots on the ground).
        height = self._height_agl()
        airborne = height > self.airborne_height_m
        if pos.dist_bottom_valid:
            ground_z = float(pos.z) + float(pos.dist_bottom)
        else:
            ground_z = 0.0 if airborne else float(pos.z)
        self.target_z = ground_z - self.altitude

        if airborne:
            self.get_logger().info(
                f'Armed + OFFBOARD and already flying ({height:.2f} m) — '
                f'starting forward traversal now at {self.altitude:.2f} m')
            self._enter_start_align()
        else:
            self.state = STATE_TAKEOFF
            self.get_logger().info(
                f'Armed + OFFBOARD on the ground — climbing to {self.altitude:.2f} m, '
                'forward traversal starts on reaching it')

    def _tick_takeoff(self):
        x, y, _ = self.home_xyz
        self._hold_position(x, y, self.target_z, self.home_yaw)
        if abs(float(self.local_position.z) - self.target_z) < self.altitude_tolerance_m:
            if not self.align_before_traverse and self.search_direction == 'none':
                self.get_logger().info('Reached traversal altitude')
                self._enter_start_align()
                return
            self.get_logger().info('Reached takeoff altitude — stabilising')
            self.stable_since = None
            self.state = STATE_STABILISE

    def _tick_stabilise(self):
        x, y, _ = self.home_xyz
        self._hold_position(x, y, self.target_z, self.home_yaw)

        if abs(float(self.local_position.z) - self.target_z) >= self.altitude_tolerance_m:
            self.stable_since = None
            return
        now = self.get_clock().now()
        if self.stable_since is None:
            self.stable_since = now
        elif (now - self.stable_since).nanoseconds / 1e9 >= self.stabilise_after_takeoff_s:
            self.get_logger().info('Stabilised after takeoff')
            self._enter_start_align()

    def _start_traversal_phase(self):
        if self.align_before_traverse or self.search_direction != 'none':
            self.get_logger().info('Acquiring the track')
            self._enter_acquire()
        else:
            self.get_logger().info(
                f'Moving ahead at {self.max_forward_speed:.2f} m/s, yawing onto the '
                'track centreline')
            self._enter_traverse()

    # ── ACQUIRE ──────────────────────────────────────────────────────────

    def _enter_acquire(self):
        self.vx_cmd = 0.0
        self.search_seen_ok = False
        self.search_lost_once = False
        self.search_started = self.get_clock().now()
        self.search_start_xy = self._xy()
        self.hold_xyz = (*self._xy(), self.target_z)
        self.hold_yaw = self.current_yaw
        self.state = STATE_ACQUIRE

    def _searching_laterally(self):
        return self.search_direction != 'none' and not self.track_found_once

    def _tick_acquire(self):
        det = self._track()

        if not self._searching_laterally():
            x, y, z = self.hold_xyz
            self._hold_position(x, y, z, self.hold_yaw)
            if det is not None:
                self.get_logger().info('Track acquired — aligning')
                self._enter_align()
            return

        # Lateral search — obstacle_traversal's ROLL_SEARCH, either side.
        ok = det is not None
        if ok and (not self.search_require_lose_regain or self.search_lost_once):
            self.get_logger().info(
                'Track found' + (' (after losing it once)' if self.search_lost_once else '')
                + ' — aligning')
            self._enter_align()
            return
        if ok and not self.search_seen_ok:
            self.search_seen_ok = True
            self.get_logger().info(
                f'Track sighted while rolling {self.search_direction} (may be the '
                "take-off pad's own flanking texture) — continuing until it is "
                'lost and regained')
        elif not ok and self.search_seen_ok and not self.search_lost_once:
            self.search_lost_once = True
            self.get_logger().info(
                f'Track lost — continuing roll-{self.search_direction}, watching '
                'for the course track')

        elapsed = (self.get_clock().now() - self.search_started).nanoseconds / 1e9
        if ok and elapsed >= self.search_timeout_s:
            self.get_logger().warn(
                f'Search exceeded {self.search_timeout_s:.0f} s without a clean '
                'lose-then-regain — accepting this lock')
            self._enter_align()
            return

        if self._dist_from(self.search_start_xy) >= self.search_max_distance_m:
            self.get_logger().error(
                f'Rolled {self.search_max_distance_m:.1f} m {self.search_direction} '
                'without finding the track — giving up, holding position')
            self._enter_hold()
            return

        vy = self.search_speed if self.search_direction == 'right' else -self.search_speed
        self._velocity_at_altitude(0.0, vy)

    # ── ALIGN ────────────────────────────────────────────────────────────

    def _enter_align(self):
        self.track_found_once = True
        if not self.align_before_traverse:
            self._enter_traverse()
            return
        self.lost_ticks = 0
        self.stable_since = None
        self.vx_cmd = 0.0
        self.state = STATE_ALIGN

    def _tick_align(self):
        det = self._track()
        if det is None:
            self.lost_ticks += 1
            self._velocity_at_altitude(0.0, 0.0)
            if self.lost_ticks > self.lost_grace_ticks:
                self.get_logger().warn(
                    f'Track lost for more than {self.lost_grace_s:.1f} s while '
                    'aligning — hovering until it re-locks')
                self._enter_acquire()
            return
        self.lost_ticks = 0

        vy, yawspeed = self._track_correction(det)
        self._velocity_at_altitude(0.0, vy, yawspeed)

        if not self._is_aligned(det):
            self.stable_since = None
            return
        now = self.get_clock().now()
        if self.stable_since is None:
            self.stable_since = now
        elif (now - self.stable_since).nanoseconds / 1e9 >= self.align_time_s:
            self.get_logger().info(
                f'Aligned with the track for {self.align_time_s:.1f} s — TRAVERSE '
                f'(max {self.max_forward_speed:.2f} m/s)')
            self._enter_traverse()

    # ── TRAVERSE ─────────────────────────────────────────────────────────

    def _enter_traverse(self):
        self.lost_ticks = 0
        self.vx_cmd = 0.0
        self.forward_enabled = False
        self.blind_forward = False
        self.traverse_started = self.get_clock().now()
        if self.traverse_start_xy is None:
            self.traverse_start_xy = self._xy()
        self.state = STATE_TRAVERSE

    def _tick_traverse(self):
        traversed = self._dist_from(self.traverse_start_xy)
        if 0.0 < self.max_traverse_distance_m <= traversed:
            self.get_logger().info(
                f'Traversed {traversed:.2f} m (max_traverse_distance_m) — end of run')
            self._end_of_track()
            return

        if self._marker(self.end_marker_id) is not None:
            self.get_logger().info(
                f'Finish marker id {self.end_marker_id} detected after {traversed:.2f} m — '
                f'stopping to align at {self.end_marker_yaw_deg:.0f} deg')
            self._enter_end_align()
            return

        det = self._track()
        if not self.align_before_traverse:
            self._tick_traverse_vertical_gate(det, traversed)
            return
        if det is None:
            self.lost_ticks += 1
            self._velocity_at_altitude(self._slew_forward(0.0), 0.0)
            if traversed >= self.min_traverse_distance_m:
                if self.lost_ticks > self.end_lost_ticks:
                    self.get_logger().info(
                        f'Track lost for {self.end_lost_s:.1f} s after '
                        f'{traversed:.2f} m — end of track reached')
                    self._end_of_track()
            elif self.lost_ticks > self.lost_grace_ticks:
                self.get_logger().warn(
                    f'Track lost after only {traversed:.2f} m '
                    f'(< min_traverse_distance_m) — hovering until it re-locks')
                self._enter_acquire()
            return
        self.lost_ticks = 0
        if not self.track_found_once:
            self.track_found_once = True
            self.get_logger().info('Track centreline detected — steering onto it')

        if (traversed >= self.min_traverse_distance_m
                and self._track_end_below(det)):
            self.get_logger().info(
                f'Far end of the track is below the drone after {traversed:.2f} m '
                '— end of track reached')
            self._end_of_track()
            return

        vy, yawspeed = self._track_correction(det)
        speed = self.max_forward_speed
        if self.align_before_traverse:
            speed *= self._alignment_factor(det)
        vx = self._slew_forward(speed)
        self._velocity_at_altitude(vx, vy, yawspeed)

    def _tick_traverse_vertical_gate(self, det, traversed):
        """Default TRAVERSE: forward at max_forward_speed only while the
        track is detected and vertical in the image; yaw/roll onto it
        otherwise. No track = no forward motion (hover, heading held)."""
        if det is None and not self.track_found_once and self.blind_forward_after_s > 0:
            # Track not found yet: after blind_forward_after_s, fly forward on
            # the current heading until it comes into view.
            waited = (self.get_clock().now() - self.traverse_started).nanoseconds / 1e9
            if waited >= self.blind_forward_after_s:
                if not self.blind_forward:
                    self.blind_forward = True
                    self.get_logger().warn(
                        f'No track found in {self.blind_forward_after_s:.1f} s — moving forward '
                        f'at {self.max_forward_speed:.2f} m/s until it comes into view')
                self._velocity_at_altitude(self._slew_forward(self.max_forward_speed), 0.0)
                return
        if det is None:
            self.lost_ticks += 1
            if self.forward_enabled:
                self.forward_enabled = False
                self.get_logger().info('No track in view — stopping forward motion')
            self._velocity_at_altitude(self._slew_forward(0.0), 0.0)
            if (self.track_found_once and traversed >= self.min_traverse_distance_m
                    and self.lost_ticks > self.end_lost_ticks):
                self.get_logger().info(
                    f'Track lost for {self.end_lost_s:.1f} s after '
                    f'{traversed:.2f} m — end of track reached')
                self._end_of_track()
            return
        self.lost_ticks = 0
        if not self.track_found_once:
            self.track_found_once = True
            self.get_logger().info('Track centreline detected — steering onto it')

        if (traversed >= self.min_traverse_distance_m
                and self._track_end_below(det)):
            self.get_logger().info(
                f'Far end of the track is below the drone after {traversed:.2f} m '
                '— end of track reached')
            self._end_of_track()
            return

        go = track_control.forward_gate(
            det, self.forward_enabled, self.forward_vertical_tol_deg,
            self.forward_vertical_hyst_deg)
        if go != self.forward_enabled:
            self.get_logger().info(
                f"Track vertical ({det['angle_deg']:+.1f} deg) — going forward"
                if go else
                f"Track tilted {det['angle_deg']:+.1f} deg — holding forward, yawing to align")
            self.forward_enabled = go

        vy, yawspeed = self._track_correction(det)
        vx = self._slew_forward(self.max_forward_speed if go else 0.0)
        self._velocity_at_altitude(vx, vy, yawspeed)

    # ── Marker alignment (start pad / finish marker) ─────────────────────

    def _enter_start_align(self):
        """Reached altitude: align on the start marker only if it is in
        view; otherwise go straight to track following (after at most
        start_marker_timeout_s to confirm a sighting over a few frames)."""
        self.vx_cmd = 0.0
        self.stable_since = None
        self.marker_status = None
        self.marker_seen_once = False
        self.marker_lost_since = None
        self.phase_started = self.get_clock().now()
        self.state = STATE_START_ALIGN
        self.get_logger().info(
            f'Looking for start marker id {self.start_marker_id} to align at '
            f'{self.start_marker_yaw_deg:.0f} deg (up to {self.start_marker_timeout_s:.1f} s)')

    def _enter_end_align(self):
        self.vx_cmd = 0.0
        self.stable_since = None
        self.marker_status = None
        self.marker_seen_once = True
        self.marker_lost_since = None
        self.state = STATE_END_ALIGN

    def _servo_to_marker(self, marker, target_yaw_deg):
        """One tick of centre + yaw-align; True once within tolerance for
        marker_stabilise_s continuously (any excursion resets the timer)."""
        vx, vy, yawspeed, ex, ey, yerr = track_control.marker_servo(
            marker, self.frame_w, self.frame_h, target_yaw_deg, self.kp_marker_center,
            self.max_marker_vel, self.kp_marker_yaw, self.max_yaw_rate)
        self.marker_status = (ex, ey, yerr)
        self._velocity_at_altitude(vx, vy, yawspeed)
        if not track_control.marker_within_tolerance(
                ex, ey, yerr, self.marker_center_tol_px, self.marker_yaw_tol_deg):
            self.stable_since = None
            return False
        now = self.get_clock().now()
        if self.stable_since is None:
            self.stable_since = now
        return (now - self.stable_since).nanoseconds / 1e9 >= self.marker_stabilise_s

    def _tick_start_align(self):
        marker = self._marker(self.start_marker_id)
        now = self.get_clock().now()
        if marker is None:
            self._velocity_at_altitude(0.0, 0.0)
            self.stable_since = None
            since = self.marker_lost_since if self.marker_seen_once else self.phase_started
            if self.marker_seen_once and self.marker_lost_since is None:
                self.marker_lost_since = since = now
            if (now - since).nanoseconds / 1e9 >= self.start_marker_timeout_s:
                self.get_logger().warn(
                    f'Start marker id {self.start_marker_id} '
                    f'{"lost" if self.marker_seen_once else "not seen"} for '
                    f'{self.start_marker_timeout_s:.1f} s — starting track following anyway')
                self._start_traversal_phase()
            return
        if not self.marker_seen_once:
            self.get_logger().info(f'Start marker id {self.start_marker_id} found — aligning')
        self.marker_seen_once = True
        self.marker_lost_since = None
        if self._servo_to_marker(marker, self.start_marker_yaw_deg):
            ex, ey, yerr = self.marker_status
            self.get_logger().info(
                f'Aligned over start marker (ex {ex:+.0f} ey {ey:+.0f} px, yaw err '
                f'{yerr:+.1f} deg) for {self.marker_stabilise_s:.1f} s — following the track')
            self._start_traversal_phase()

    def _tick_end_align(self):
        marker = self._marker(self.end_marker_id)
        if marker is None:
            # Hover and wait for it to come back — never finish unaligned.
            self._velocity_at_altitude(0.0, 0.0)
            self.stable_since = None
            if self.marker_lost_since is None:
                self.marker_lost_since = self.get_clock().now()
                self.get_logger().warn(
                    f'Finish marker id {self.end_marker_id} out of view — hovering')
            return
        if self.marker_lost_since is not None:
            self.get_logger().info(f'Finish marker id {self.end_marker_id} back in view')
            self.marker_lost_since = None
        if self._servo_to_marker(marker, self.end_marker_yaw_deg):
            ex, ey, yerr = self.marker_status
            self.get_logger().info(
                f'Stabilised over finish marker id {self.end_marker_id} at '
                f'{self.end_marker_yaw_deg:.0f} deg (ex {ex:+.0f} ey {ey:+.0f} px, yaw err '
                f'{yerr:+.1f} deg) — mission complete')
            self._finish()

    def _end_of_track(self):
        """The track ran out before the finish marker was seen: hover and
        keep watching for it rather than finishing unaligned."""
        self.vx_cmd = 0.0
        self.state = STATE_TRACK_END
        self.get_logger().warn(
            f'End of track reached without finish marker id {self.end_marker_id} — '
            'hovering and watching for it')

    def _tick_track_end(self):
        self._velocity_at_altitude(self._slew_forward(0.0), 0.0)
        if self._marker(self.end_marker_id) is not None:
            self.get_logger().info(f'Finish marker id {self.end_marker_id} detected — aligning')
            self._enter_end_align()

    def _finish(self):
        self.vx_cmd = 0.0
        if self.land_at_end:
            self._last_land_request = None
            self.state = STATE_LAND
        else:
            self._enter_hold()

    # ── Terminal states ──────────────────────────────────────────────────

    def _enter_hold(self):
        self.vx_cmd = 0.0
        self.hold_xyz = (*self._xy(), self.target_z)
        self.hold_yaw = self.current_yaw
        self.state = STATE_HOLD
        self.get_logger().info(
            f'POSITION HOLD at x={self.hold_xyz[0]:.2f} y={self.hold_xyz[1]:.2f} '
            f'z={self.hold_xyz[2]:.2f} (NED)')

    def _tick_hold(self):
        x, y, z = self.hold_xyz
        self._hold_position(x, y, z, self.hold_yaw)

    def _tick_land(self):
        """Re-send NAV_LAND every `arm_retry_s` until PX4 reports AUTO_LAND
        (or disarms on touchdown) — a single command can be dropped."""
        status = self.vehicle_status
        if (status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                or not self._is_armed):
            return
        now = self.get_clock().now()
        if (self._last_land_request is None
                or (now - self._last_land_request).nanoseconds / 1e9 >= self.arm_retry_s):
            self._last_land_request = now
            self.px4.land()

    def _tick_aborted(self):
        pos = self.local_position
        self._hold_position(pos.x, pos.y, pos.z, self.current_yaw)

    def destroy_node(self):
        if self.stream is not None:
            self.stream.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackTraversalNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    # OpenCV GUI calls are only reliable from the main thread, so ROS spins
    # on a background thread and the main thread just shows the newest
    # annotated frame — the setpoint stream never waits on the window.
    show_window = node.show_window and bool(os.environ.get('DISPLAY'))
    if node.show_window and not show_window:
        node.get_logger().warn('show_window requested but no DISPLAY — window disabled '
                               '(debug image still on /track_traversal/debug_image)')
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        if show_window:
            cv2.namedWindow('track_traversal', cv2.WINDOW_NORMAL)
            shown = None
            while rclpy.ok() and spin_thread.is_alive():
                frame = node.latest_debug_frame
                if frame is not None and frame is not shown:
                    cv2.imshow('track_traversal', frame)
                    shown = frame
                cv2.waitKey(15)
        else:
            spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        if show_window:
            cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
