#!/usr/bin/env python3
"""
Doll detection and GEOTAGGED counting, as a ROS 2 node.

The model is the TensorRT engine from Downloads/DroneImpl_v7 (db.engine), run
through Ultralytics exactly as jw_px4_arduinocount.py ran it. What is
different here, and it is the whole point of the node, is WHAT A DOLL'S
IDENTITY IS.

    jw_px4_arduinocount.py identified a doll by its position IN THE IMAGE:
    a ByteTrack id, plus a pixel-distance re-identification fallback for when
    the tracker dropped it. That is a perfectly good answer for a fixed
    camera. On a drone flying a box pattern it is not one -- the aircraft
    yaws 90 degrees twice, so the same doll leaves the frame on one side and
    comes back at a completely different pixel, and every time it does the
    old logic mints a new doll.

    This node identifies a doll by WHERE IT IS IN THE ROOM. Every detection is
    projected out of the camera, through the airframe, into the PX4 local NED
    frame, and a detection lands on an existing doll if it is within
    merge_radius metres of it. The track id is still used -- it is what makes
    consecutive frames cheap and what the confirmation counter counts -- but it
    is not the identity. Turn the aircraft round twice and fly back past the
    same doll and it is still doll #3, because it is still in the same corner
    of the room.

    That is what the doll id IS: a room position. It is published as one, too
    -- /doll_report carries "id x y z" for every doll counted, in metres NED
    relative to the arming point, so the count can be checked against the room
    afterwards instead of taken on trust.

HOW A PIXEL BECOMES A ROOM POSITION
-----------------------------------
Three transforms, all of them the ones window_traverse already uses for the
window, and the camera mounting parameters are deliberately the SAME
parameter names so one launch file sets both:

    pixel + depth  -> camera frame   (pinhole intrinsics from CameraInfo)
    camera frame   -> body FRD       (cam_x/y/z, cam_roll/pitch/yaw)
    body FRD       -> local NED      (VehicleAttitude quaternion + position)

DEPTH is the aligned depth image, median-sampled over the middle of the box
exactly the way window_detect samples a window corner, so one dead pixel does
not place a doll on the far wall. A box with no usable depth is DROPPED, not
guessed: a doll at an assumed range lands somewhere arbitrary in NED, which is
the one input that could actually corrupt the count. Detection without depth
is still published as "visible", it just cannot be counted.

WHEN IT RUNS
------------
Gated on /doll_detect_enable, which window_room_traverse publishes: True from
the moment it commits to the inbound traverse, False once it is back outside.
While disabled the node does not run the model at all -- on a Jetson sharing a
USB3 bus and a GPU with the RealSense and the window detector, that matters.
Set require_enable:=false to run it free, which is how you bench it.

TOPICS
------
    subscribes  <image_topic>           colour, the same stream window_detect
                                        thresholds (no second camera)
                <depth_topic>           aligned depth, 16UC1 mm or 32FC1 m
                <camera_info_topic>     intrinsics
                /fmu/out/vehicle_local_position[_vN]
                /fmu/out/vehicle_attitude
                doll_detect_enable      Bool
    publishes   doll_count              Int32, cumulative, never decreases
                dolls_visible           Int32, this frame
                doll_report             String, "n|id:x,y,z|id:x,y,z|..."
                doll_image              Image, annotated (publish_image only)
"""

import math
import threading
import time

import numpy as np

import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

from drone_testing.px4_topics import versioned_names
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32, String

import cv2

from drone_testing.window_detect import (array_to_imgmsg, imgmsg_to_bgr,
                                         imgmsg_to_depth)
from drone_testing.window_traverse import quat_rotate, rpy_to_matrix_frd


class GeotaggedDolls:
    """Every doll ever counted, keyed by where it is in the room.

    A dict of doll_id -> {'p': np(3) NED, 'n': how many detections have landed
    on it, 'first': monotonic, 'last': monotonic}. The position is a running
    mean, so a doll seen from two sides ends up between the two estimates
    rather than wherever it was last seen from.

    The count is monotonic by construction: nothing in here ever removes a
    doll or decrements anything. The only way the total goes up is a detection
    that is confirmed AND lands further than merge_radius from every doll
    already on the list.
    """

    def __init__(self, merge_radius):
        self.merge_radius = float(merge_radius)
        self.dolls = {}
        self._next_id = 1

    def observe(self, p_ned, now):
        """Land one confirmed detection on a doll. Returns (id, is_new)."""
        best_id, best_dist = None, self.merge_radius
        for doll_id, info in self.dolls.items():
            dist = float(np.linalg.norm(p_ned - info['p']))
            if dist < best_dist:
                best_dist, best_id = dist, doll_id

        if best_id is not None:
            info = self.dolls[best_id]
            # Running mean, capped so an old doll is not immovable: after
            # ~20 observations the position stops chasing new ones, which is
            # what stops a mis-ranged frame from walking a doll across the
            # room and out of merge_radius of its own future detections.
            weight = 1.0 / min(info['n'] + 1, 20)
            info['p'] = info['p'] * (1.0 - weight) + p_ned * weight
            info['n'] += 1
            info['last'] = now
            return best_id, False

        doll_id = self._next_id
        self._next_id += 1
        self.dolls[doll_id] = {'p': np.array(p_ned, dtype=float), 'n': 1,
                               'first': now, 'last': now}
        return doll_id, True

    @property
    def total(self):
        return len(self.dolls)

    def report(self):
        return "|".join(
            f"{doll_id}:{info['p'][0]:.2f},{info['p'][1]:.2f},{info['p'][2]:.2f}"
            for doll_id, info in sorted(self.dolls.items()))


class DollDetect(Node):

    # ---- the model --------------------------------------------------------
    MODEL_PATH = '/home/ark-jetson-orin-2/Downloads/DroneImpl_v7/db.engine'
    TRACKER_PATH = '/home/ark-jetson-orin-2/Downloads/DroneImpl_v7/custom_bytetrack.yaml'
    CONFIDENCE = 0.55           # same gate as the standalone script
    MIN_FRAMES_TO_CONFIRM = 5   # a track must survive this many frames before
                                # it is allowed to create or join a doll. Kills
                                # single-frame false positives before they can
                                # ever reach the geotagger.
    MAX_FPS = 6.0               # cap. The model is not the expensive thing on
                                # this Jetson -- the setpoint timer in the
                                # flight node is what must not be starved, and
                                # a doll does not move.

    # ---- the geotag -------------------------------------------------------
    MERGE_RADIUS = 0.60         # m. Two detections closer together than this
                                # in the room are the same doll. Set it from
                                # how far apart the dolls actually are: it must
                                # be comfortably smaller than the smallest gap
                                # between two dolls, and comfortably bigger
                                # than the position error, which at 3 m range
                                # is dominated by depth noise and the flow's
                                # own drift rather than by the pixel.
    DEPTH_MIN = 0.30            # m. Closer than this is the airframe.
    DEPTH_MAX = 8.00            # m. Further is not a doll in this room, it is
                                # the far wall showing through a box.
    DEPTH_PATCH = 0.30          # fraction of the box side the depth median is
                                # taken over, centred. The middle 30% of a doll
                                # is doll; the edges are the floor behind it.
    MIN_DEPTH_PIXELS = 12       # valid depth samples needed in that patch

    def __init__(self):
        super().__init__('doll_detect')

        self.model_path = str(self.declare_parameter(
            'model_path', self.MODEL_PATH).value)
        self.tracker_path = str(self.declare_parameter(
            'tracker_path', self.TRACKER_PATH).value)
        self.confidence = float(self.declare_parameter(
            'confidence', self.CONFIDENCE).value)
        self.min_frames = int(self.declare_parameter(
            'min_frames_to_confirm', self.MIN_FRAMES_TO_CONFIRM).value)
        self.max_fps = float(self.declare_parameter('max_fps', self.MAX_FPS).value)
        self.min_interval = 1.0 / self.max_fps if self.max_fps > 0.0 else 0.0

        self.merge_radius = float(self.declare_parameter(
            'merge_radius', self.MERGE_RADIUS).value)
        self.depth_min = float(self.declare_parameter('depth_min', self.DEPTH_MIN).value)
        self.depth_max = float(self.declare_parameter('depth_max', self.DEPTH_MAX).value)
        self.depth_patch = float(self.declare_parameter(
            'depth_patch', self.DEPTH_PATCH).value)
        self.min_depth_pixels = int(self.declare_parameter(
            'min_depth_pixels', self.MIN_DEPTH_PIXELS).value)

        self.image_topic = str(self.declare_parameter(
            'image_topic', '/camera/camera/color/image_raw').value)
        self.depth_topic = str(self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw').value)
        info_topic = str(self.declare_parameter('camera_info_topic', 'auto').value)
        if info_topic == 'auto':
            info_topic = self.image_topic.rsplit('/', 1)[0] + '/camera_info'
        self.camera_info_topic = info_topic

        # WHERE A DETECTION'S DEPTH COMES FROM.
        #   image       the aligned depth image (the level RealSense), as before.
        #   rangefinder the DOWNWARD camera over a flat floor: every pixel's
        #               depth along the optical axis is the height above the
        #               floor, which the TFmini measures (dist_bottom), minus
        #               target_height (the doll's centre above the floor).
        #               Mount it with cam_pitch -90 deg (image-up = nose).
        self.depth_source = str(self.declare_parameter(
            'depth_source', 'image').value).strip().lower()
        if self.depth_source not in ('image', 'rangefinder'):
            raise SystemExit(f"depth_source must be image|rangefinder, got "
                             f"'{self.depth_source}'")
        self.target_height = float(self.declare_parameter(
            'target_height', 0.10).value)

        self.publish_image = bool(self.declare_parameter('publish_image', False).value)
        self.require_enable = bool(self.declare_parameter('require_enable', True).value)

        # The camera mounting, in the SAME parameter names and the SAME ROS
        # convention (x forward, y left, z up) window_traverse takes, so one
        # launch file configures both and they cannot disagree about where the
        # lens is. Getting these wrong does not stop the node: it puts every
        # doll in the room offset by exactly the error, which merges dolls that
        # are not the same one if the error is large.
        cam_x = float(self.declare_parameter('cam_x', 0.0).value)
        cam_y = float(self.declare_parameter('cam_y', 0.0).value)
        cam_z = float(self.declare_parameter('cam_z', 0.0).value)
        cam_roll = float(self.declare_parameter('cam_roll', 0.0).value)
        cam_pitch = float(self.declare_parameter('cam_pitch', 0.0).value)
        cam_yaw = float(self.declare_parameter('cam_yaw', 0.0).value)
        self.r_cam = rpy_to_matrix_frd(cam_roll, cam_pitch, cam_yaw)
        self.t_cam = np.array([cam_x, -cam_y, -cam_z])   # ROS FLU -> body FRD

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sensor_cbg = MutuallyExclusiveCallbackGroup()
        self.vision_cbg = MutuallyExclusiveCallbackGroup()

        self.local_position = None
        self.attitude = None
        self.depth_image = None
        self.depth_time = 0.0
        self.intrinsics = None
        self._lock = threading.Lock()

        self.local_position_subs = [
            self.create_subscription(
                VehicleLocalPosition, name, self.local_position_callback,
                qos_profile=sensor_qos, callback_group=self.sensor_cbg)
            for name in versioned_names('vehicle_local_position')
        ]
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self.attitude_callback, qos_profile=sensor_qos,
                                 callback_group=self.sensor_cbg)
        self.create_subscription(Bool, 'doll_detect_enable',
                                 self.enable_callback, 10,
                                 callback_group=self.sensor_cbg)
        if self.depth_source == 'image':
            self.create_subscription(Image, self.depth_topic, self.depth_callback,
                                     qos_profile_sensor_data,
                                     callback_group=self.sensor_cbg)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 self.camera_info_callback,
                                 qos_profile_sensor_data,
                                 callback_group=self.sensor_cbg)
        # The image subscription is on its own group: inference takes tens of
        # milliseconds and must not sit in front of the pose it will be paired
        # with.
        self.create_subscription(Image, self.image_topic, self.image_callback,
                                 qos_profile_sensor_data,
                                 callback_group=self.vision_cbg)

        self.count_pub = self.create_publisher(Int32, 'doll_count', 10)
        self.visible_pub = self.create_publisher(Int32, 'dolls_visible', 10)
        self.report_pub = self.create_publisher(String, 'doll_report', 10)
        self.image_pub = (self.create_publisher(Image, 'doll_image', 1)
                          if self.publish_image else None)

        self.enabled = not self.require_enable
        self.dolls = GeotaggedDolls(self.merge_radius)
        self.hits_per_track = {}
        self.track_to_doll = {}
        self.visible = 0
        self.frames = 0
        self.last_processed = 0.0
        self.no_depth_boxes = 0
        self.no_pose_boxes = 0
        self._last_published = (-1, -1)

        # The model is loaded lazily, on the first frame the node is actually
        # enabled for. Loading a TensorRT engine takes several seconds and
        # allocates GPU memory; doing it in __init__ would do it during the
        # aircraft's climb, on a launch file that starts everything at once.
        self.model = None
        self._model_failed = False

        self.create_timer(1.0, self.report_timer, callback_group=self.sensor_cbg)

        self.get_logger().info(
            f"doll_detect: {self.image_topic} + {self.depth_topic}, model "
            f"{self.model_path}, confirming at {self.min_frames} frames, "
            f"merging detections within {self.merge_radius:.2f} m of each "
            f"other in NED. "
            + ("Waiting for doll_detect_enable." if self.require_enable
               else "Running free (require_enable:=false)."))

    # ---------------------------------------------------------------- subs

    def local_position_callback(self, msg):
        self.local_position = msg

    def attitude_callback(self, msg):
        self.attitude = msg

    def enable_callback(self, msg):
        if bool(msg.data) == self.enabled:
            return
        self.enabled = bool(msg.data)
        self.get_logger().warning(
            f"Doll detection {'ENABLED' if self.enabled else 'DISABLED'} by the "
            f"flight node. Total so far: {self.dolls.total}.")
        if not self.enabled:
            # Drop the per-track bookkeeping. The DOLLS survive -- they are the
            # count -- but a track id from before a gap means nothing after it.
            self.hits_per_track.clear()
            self.track_to_doll.clear()
            self.visible = 0
            self._publish_counts()

    def depth_callback(self, msg):
        try:
            depth = imgmsg_to_depth(msg)
        except Exception as exc:
            self.get_logger().warning(f"Cannot convert depth frame: {exc}",
                                      throttle_duration_sec=5.0)
            return
        with self._lock:
            self.depth_image = depth
            self.depth_time = time.monotonic()

    def camera_info_callback(self, msg):
        fx, fy, cx, cy = float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5])
        if fx <= 1.0 or fy <= 1.0:
            return
        if self.intrinsics is None:
            self.get_logger().info(
                f"CameraInfo: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}. "
                "Doll positions are now metric.")
        self.intrinsics = (fx, fy, cx, cy)

    # ------------------------------------------------------------- the model

    def _load_model(self):
        if self.model is not None or self._model_failed:
            return self.model
        try:
            from ultralytics import YOLO
            self.get_logger().warning(
                f"Loading the TensorRT engine {self.model_path}. This takes a "
                "few seconds and allocates GPU memory.")
            self.model = YOLO(self.model_path, task='detect')
            self.get_logger().warning("Doll model ready.")
        except Exception as exc:
            self._model_failed = True
            self.get_logger().error(
                f"Cannot load {self.model_path}: {exc}. Doll detection is off "
                "for the rest of this flight; everything else is unaffected.")
        return self.model

    # ------------------------------------------------------------ the frame

    def image_callback(self, msg):
        if not self.enabled:
            return

        now = time.monotonic()
        if self.min_interval and now - self.last_processed < self.min_interval:
            return
        self.last_processed = now

        model = self._load_model()
        if model is None:
            return

        try:
            frame = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"Cannot convert image frame: {exc}",
                                    throttle_duration_sec=5.0)
            return

        self.frames += 1

        # Pair the detections with the pose AS OF NOW, before inference, not
        # after: the pose that belongs to this image is the one from when the
        # shutter opened, and inference is the longest delay in the chain.
        lp = self.local_position
        att = self.attitude
        with self._lock:
            depth = self.depth_image
            depth_age = now - self.depth_time

        try:
            results = model.track(frame, tracker=self.tracker_path,
                                  persist=True, verbose=False)[0]
        except Exception as exc:
            self.get_logger().error(f"Inference failed: {exc}",
                                    throttle_duration_sec=5.0)
            return

        detections = []
        if results.boxes is not None and results.boxes.id is not None:
            boxes = results.boxes.xyxy.cpu().numpy()
            ids = results.boxes.id.cpu().numpy().astype(int)
            confs = results.boxes.conf.cpu().numpy()
            for box, track_id, conf in zip(boxes, ids, confs):
                if float(conf) < self.confidence:
                    continue
                detections.append((int(track_id), box, float(conf)))

        self.visible = len(detections)
        annotated = self._process(detections, lp, att, depth, depth_age, now,
                                 frame if self.image_pub is not None else None)

        self._publish_counts()
        if self.image_pub is not None and annotated is not None:
            self.image_pub.publish(array_to_imgmsg(annotated, 'bgr8', msg.header))

    def _process(self, detections, lp, att, depth, depth_age, now, frame):
        """Confirm, geotag and count one frame's detections."""
        pose_ok = (lp is not None and lp.xy_valid and lp.z_valid
                   and att is not None)
        if not pose_ok and detections:
            self.no_pose_boxes += len(detections)
            self.get_logger().warning(
                "Dolls in frame but no valid vehicle pose to geotag them with; "
                "not counting them. They will be counted on a later frame if "
                "they are still there.", throttle_duration_sec=5.0)

        if self.depth_source == 'rangefinder':
            depth_ok = (lp is not None and bool(lp.dist_bottom_valid)
                        and lp.dist_bottom - self.target_height > self.depth_min)
        else:
            depth_ok = depth is not None and depth_age < 1.0

        for track_id, box, conf in detections:
            # Case 1: this track is already on a doll. Nothing to decide.
            if track_id in self.track_to_doll:
                doll_id = self.track_to_doll[track_id]
                p = self._geotag(box, depth, lp, att) if (pose_ok and depth_ok) else None
                if p is not None:
                    self.dolls.observe(p, now)
                self._draw(frame, box, f"doll {doll_id}", (0, 255, 0))
                continue

            # Case 2: build confidence over frames before it is allowed to
            # affect the count at all.
            hits = self.hits_per_track.get(track_id, 0) + 1
            self.hits_per_track[track_id] = hits
            if hits < self.min_frames:
                self._draw(frame, box, f"{hits}/{self.min_frames}", (0, 255, 255))
                continue

            if not (pose_ok and depth_ok):
                # Confirmed, but unplaceable. Deliberately NOT counted: a doll
                # with no position cannot be de-duplicated against the ones
                # that have one, and a count that can double is worse than a
                # count that is late. The track keeps its hits, so the moment
                # depth or the pose comes back it is counted on that frame.
                self._draw(frame, box, "no fix", (0, 165, 255))
                continue

            p = self._geotag(box, depth, lp, att)
            if p is None:
                self.no_depth_boxes += 1
                self._draw(frame, box, "no depth", (0, 165, 255))
                continue

            doll_id, is_new = self.dolls.observe(p, now)
            self.track_to_doll[track_id] = doll_id
            if is_new:
                self.get_logger().warning(
                    f"DOLL {doll_id} counted at ({p[0]:+.2f}, {p[1]:+.2f}, "
                    f"{p[2]:+.2f}) NED. Total {self.dolls.total}.")
            else:
                self.get_logger().info(
                    f"Track {track_id} is doll {doll_id} again (within "
                    f"{self.merge_radius:.2f} m of where it was counted); not "
                    "recounted.")
            self._draw(frame, box, f"doll {doll_id}", (0, 255, 0))

        return frame

    # ----------------------------------------------------------- the geotag

    def _geotag(self, box, depth, lp, att):
        """Box + depth + vehicle pose -> the doll's position in NED, or None.

        None means "not placeable", and every caller treats that as "do not
        count", never as "count it anyway at a guess". See the module header.
        """
        if self.intrinsics is None:
            self.get_logger().warning(
                f"No CameraInfo on {self.camera_info_topic} yet; dolls cannot "
                "be placed.", throttle_duration_sec=10.0)
            return None

        if self.depth_source == 'rangefinder':
            # Flat floor under a level, downward camera: z-depth = HAGL.
            distance = float(lp.dist_bottom) - self.target_height
            if not (self.depth_min <= distance <= self.depth_max):
                return None
        else:
            distance = self._box_depth(box, depth)
            if distance is None:
                return None

        fx, fy, cx, cy = self.intrinsics
        x1, y1, x2, y2 = [float(v) for v in box]
        u = 0.5 * (x1 + x2)
        v = 0.5 * (y1 + y2)

        # Pinhole, in the OPTICAL frame: x right, y down, z forward.
        x_opt = (u - cx) / fx * distance
        y_opt = (v - cy) / fy * distance
        # Optical -> the camera-FRD frame r_cam and t_cam are written against:
        # x forward, y right, z down. This is the same convention
        # WindowEstimator._add builds its corners in -- (depth, depth*tan(az),
        # -depth*tan(el)) with az positive right and el positive up -- so the
        # cam_* parameters mean exactly the same thing in both nodes.
        p_cam = np.array([distance, x_opt, y_opt])

        p_body = self.r_cam @ p_cam + self.t_cam
        p_ned = np.array([lp.x, lp.y, lp.z]) + quat_rotate(
            np.asarray(att.q, dtype=float), p_body)
        return p_ned

    def _box_depth(self, box, depth):
        """Median depth over the middle of the box, in metres, or None.

        The median and not the centre pixel: on a doll at 3 m the centre pixel
        drops out often enough that a single-pixel reading would throw away
        most detections, and when it does NOT drop out it is as likely to be
        looking between the doll's arm and the wall as at the doll.
        """
        h, w = depth.shape[:2]
        x1, y1, x2, y2 = [float(v) for v in box]
        bw, bh = x2 - x1, y2 - y1
        if bw <= 1.0 or bh <= 1.0:
            return None

        half_w = max(1.0, bw * self.depth_patch * 0.5)
        half_h = max(1.0, bh * self.depth_patch * 0.5)
        cx_px, cy_px = 0.5 * (x1 + x2), 0.5 * (y1 + y2)

        u0 = int(max(0, math.floor(cx_px - half_w)))
        u1 = int(min(w, math.ceil(cx_px + half_w)))
        v0 = int(max(0, math.floor(cy_px - half_h)))
        v1 = int(min(h, math.ceil(cy_px + half_h)))
        if u1 <= u0 or v1 <= v0:
            return None

        patch = depth[v0:v1, u0:u1].astype(float)
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid >= self.depth_min) & (valid <= self.depth_max)]
        if valid.size < self.min_depth_pixels:
            return None
        return float(np.median(valid))

    # -------------------------------------------------------------- output

    def _draw(self, frame, box, label, colour):
        if frame is None:
            return
        x1, y1, x2, y2 = [int(v) for v in box]
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(frame, label, (x1, max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)

    def _publish_counts(self, force=False):
        """Publish the counts, by default only when they have changed.

        force=True is the once-a-second heartbeat from report_timer, and it is
        there for the same reason that timer publishes /doll_report
        unconditionally: a topic that only publishes on change has nothing in
        it for the minute before the first doll, so anything subscribing late
        -- a display started after the mission, a bag opened at the wrong
        moment -- sits on an empty topic and cannot tell "the count is zero"
        from "nothing is running". The dedup still applies to the per-frame
        path, which is where the message rate would otherwise come from.
        """
        payload = (self.visible, self.dolls.total)
        if payload == self._last_published and not force:
            return
        self._last_published = payload
        visible = Int32()
        visible.data = int(self.visible)
        self.visible_pub.publish(visible)
        total = Int32()
        total.data = int(self.dolls.total)
        self.count_pub.publish(total)

    def report_timer(self):
        """The full geotagged list, once a second, for the log and the ground.

        Published even when nothing has changed: this is the topic anyone
        checking the count against the room afterwards will be reading out of
        a bag, and a topic that only publishes on change is a topic with
        nothing in it for the minute before the first doll.
        """
        msg = String()
        msg.data = f"{self.dolls.total}|{self.dolls.report()}"
        self.report_pub.publish(msg)

        # Same argument as the docstring above, applied to the counts: this is
        # what a late subscriber (doll_count_gui, room_display) needs in order
        # to show 0 rather than "no data" before the first doll.
        self._publish_counts(force=True)

        if self.enabled:
            self.get_logger().info(
                f"dolls: {self.dolls.total} counted, {self.visible} in frame "
                f"({self.frames} frames; {self.no_depth_boxes} boxes dropped "
                f"for no depth, {self.no_pose_boxes} for no pose).",
                throttle_duration_sec=5.0)

    def destroy_node(self):
        self.get_logger().warning(
            f"FINAL DOLL COUNT: {self.dolls.total}. Positions (NED, m): "
            f"{self.dolls.report() or 'none'}.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DollDetect()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.remove_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
