#!/usr/bin/env python3
"""The ring board, seen by the RealSense D435i, in camera-frame numbers.

    ros2 run drone_testing ring_detect
    ros2 topic echo /ring_info

Subscribes to the D435i's COLOUR stream and its camera_info, detects the board
markers, solves whatever subset is visible as ONE RIGID BOARD, reports the pose
at the grab marker (see ring_math.py), and publishes:

    /ring_detected      std_msgs/Bool               debounced: true after
                                                    detect_frames consecutive
                                                    accepted frames, false
                                                    after lost_frames misses
    /ring_geometry      std_msgs/Float32MultiArray  6 rows of 3, below
    /ring_info          std_msgs/String             one human-readable line
    browser             http://<jetson-ip>:8080/    the annotated frame.
                                                    stream_port:=0 turns it off

NO DEPTH IS USED, AND NONE IS NEEDED
------------------------------------
window_detect needs the aligned depth image because an HSV blob has no
intrinsic scale. A marker does: its edge length is known, so the four corners
plus the camera matrix are a complete range measurement, and solvePnP returns
metres. That is why the launch file leaves the depth stream off entirely --
the D435i is being used here as a calibrated monocular camera.

The corollary is that marker_size and the camera intrinsics ARE the scale.
Mistype marker_size by 10 % and every distance, including the 20 cm grab
depth, is out by 10 % in the same direction. Measure the printed marker with a
tape, edge of black square to edge of black square, and do not trust the
number on the PDF you printed it from.

/ring_geometry, row by row -- all in the CAMERA FRD FRAME
    (x FORWARD along the view axis, y RIGHT, z DOWN)

    0   grab-marker centre, metres. Fused, so it is populated even on a frame
        where the grab marker itself is hidden and only its neighbours are
        visible.
    1   board +X (right across the board), unit vector
    2   board +Y (up the board), unit vector
    3   board +Z (out of the board, TOWARDS the camera), unit vector
    4   (n_markers, ambiguous 0/1, distance to the grab marker in metres)
    5   (0/1 per board id, in ascending id order, zero-padded to three)

WHY CAMERA-FRAME AND NOTHING ELSE
---------------------------------
Same reason aruco_pose.py stops where it does: turning this into NED needs the
airframe's attitude, that lives in the flight node, and keeping it there means
this node runs and can be trusted on a bench with no flight controller
attached. ring_grab.py does the rotation, with the full attitude quaternion
rather than the heading, so the answer is tilt-compensated.

The FRD convention is deliberate and is NOT what OpenCV gives back. Rotating
it here means ring_grab can apply exactly the r_cam / t_cam mounting matrix
window_traverse applies, built by the same rpy_to_matrix_frd from the same
cam_roll / cam_pitch / cam_yaw numbers. Two nodes that disagree about the sign
of cam_pitch is a bug nobody finds until it is in the air.

BENCH TEST -- do this before it ever flies

    ros2 launch drone_testing ring_grab.launch.py flight:=false
    ros2 topic echo /ring_info

Stand the board up, point the camera at it and walk a tape measure out. The
reported distance must match the tape to a few centimetres at 1-3 m. Then move
the board to the camera's RIGHT: the info line must say RIGHT. If distance is
wrong, marker_size or the intrinsics are wrong; if a direction is wrong,
image_rotate or the mounting is.
"""

import math
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, String

from drone_testing.aruco_pose import MjpegServer
from drone_testing.ring_math import BoardGeometry, BoardLocator, dict_id, fov_K


GEOMETRY_ROWS = 6
GEOMETRY_COLS = 3


def imgmsg_to_bgr(msg):
    """sensor_msgs/Image -> BGR numpy array, for the encodings the D435i uses."""
    enc = msg.encoding.lower()
    h, w = msg.height, msg.width
    if enc in ('rgb8', 'bgr8'):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return arr if enc == 'bgr8' else cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if enc in ('rgba8', 'bgra8'):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
        return cv2.cvtColor(
            arr, cv2.COLOR_BGRA2BGR if enc == 'bgra8' else cv2.COLOR_RGBA2BGR)
    if enc == 'mono8':
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported image encoding '{msg.encoding}'")


def direction_words(cam_xyz):
    """'2.10 m ahead, RIGHT 0.35 m, DOWN 0.12 m' -- the sign check, in words."""
    fwd, right, down = (float(v) for v in cam_xyz)
    return (f"{fwd:.2f} m ahead, "
            f"{'RIGHT' if right >= 0 else 'LEFT'} {abs(right):.2f} m, "
            f"{'DOWN' if down >= 0 else 'UP'} {abs(down):.2f} m")


class RingDetect(Node):

    IMAGE_TOPIC = '/camera/camera/color/image_raw'
    CAMERA_INFO_TOPIC = 'auto'      # 'auto' = image_topic with the last
                                    # segment swapped for camera_info

    MARKER_IDS = [4, 5, 6]          # bottom, middle, top
    GRAB_MARKER_ID = 4              # the one the ring hangs on
    MARKER_SIZE = 0.10              # m, edge of the black square. MEASURE IT.
    MARKER_GAP = 0.10               # m, blank space between two markers
    ARUCO_DICT = 'DICT_4X4_50'

    MAX_FPS = 15.0                  # cap on frames actually processed
    DEPTH_MIN = 0.25                # m, closer than this is not a board
    DEPTH_MAX = 8.00                # m, further than this is noise
    MAX_REPROJ_PX = 4.0             # px, worst single corner. The residual of
                                    # the accepted solution against the corners
                                    # it was fitted to -- a board that did not
                                    # really fit shows up here. Loose on
                                    # purpose: a board described with the wrong
                                    # pitch reprojects at ~28 px, while a
                                    # correct one stays under 2 px even when it
                                    # fills the frame, so there is a wide gap
                                    # to sit in and a false reject during the
                                    # run-in costs more than a late catch.
    MIN_MARKERS = 1
    DETECT_FRAMES = 3               # accepted frames before /ring_detected
    LOST_FRAMES = 5                 # missed frames before it goes false

    HFOV_DEG = 69.0                 # D435i colour, only used if camera_info
                                    # never arrives
    IMAGE_ROTATE = 0
    STREAM_PORT = 8080
    STREAM_SCALE = 0.6
    JPEG_QUALITY = 70

    def __init__(self):
        super().__init__('ring_detect')

        self.image_topic = str(self.declare_parameter(
            'image_topic', self.IMAGE_TOPIC).value)
        info_topic = str(self.declare_parameter(
            'camera_info_topic', self.CAMERA_INFO_TOPIC).value).strip()
        if info_topic in ('', 'auto'):
            # Derived rather than declared separately, so the two cannot drift
            # apart -- window_detect has the scar tissue on this one. Deriving
            # from image_topic also guarantees we get the COLOUR intrinsics
            # and not the depth module's, which are different.
            info_topic = self.image_topic.rsplit('/', 1)[0] + '/camera_info'
        self.info_topic = info_topic

        ids = list(self.declare_parameter('marker_ids', self.MARKER_IDS).value)
        self.dict_name = str(self.declare_parameter('aruco_dict', self.ARUCO_DICT).value)
        try:
            self.geometry = BoardGeometry.vertical_stack(
                ids=ids,
                marker_size=float(self.declare_parameter(
                    'marker_size', self.MARKER_SIZE).value),
                marker_gap=float(self.declare_parameter(
                    'marker_gap', self.MARKER_GAP).value),
                grab_id=int(self.declare_parameter(
                    'grab_marker_id', self.GRAB_MARKER_ID).value),
                dictionary_id=dict_id(self.dict_name),
            )
        except ValueError as exc:
            # Fatal on purpose. A board geometry that does not describe the
            # board produces a confident pose at the wrong place, and the
            # flight node has no way to tell.
            raise SystemExit(f"Bad board geometry: {exc}")

        self.max_fps = float(self.declare_parameter('max_fps', self.MAX_FPS).value)
        self.depth_min = float(self.declare_parameter('depth_min', self.DEPTH_MIN).value)
        self.depth_max = float(self.declare_parameter('depth_max', self.DEPTH_MAX).value)
        self.max_reproj_px = float(self.declare_parameter(
            'max_reproj_px', self.MAX_REPROJ_PX).value)
        self.min_markers = int(self.declare_parameter('min_markers', self.MIN_MARKERS).value)
        self.detect_frames = int(self.declare_parameter(
            'detect_frames', self.DETECT_FRAMES).value)
        self.lost_frames = int(self.declare_parameter('lost_frames', self.LOST_FRAMES).value)
        self.hfov_deg = float(self.declare_parameter('hfov_deg', self.HFOV_DEG).value)
        self.image_rotate = int(self.declare_parameter(
            'image_rotate', self.IMAGE_ROTATE).value)
        self.stream_port = int(self.declare_parameter('stream_port', self.STREAM_PORT).value)
        self.stream_scale = float(self.declare_parameter(
            'stream_scale', self.STREAM_SCALE).value)
        self.jpeg_quality = int(self.declare_parameter(
            'jpeg_quality', self.JPEG_QUALITY).value)
        self.annotate = bool(self.declare_parameter('annotate', True).value)

        if self.image_rotate not in (0, 90, 180, 270):
            self.get_logger().warning(
                f"image_rotate {self.image_rotate} is not 0/90/180/270; using 0.")
            self.image_rotate = 0

        self.locator = BoardLocator(self.geometry)

        self.K = None
        self.D = np.zeros((5, 1), dtype=np.float64)
        self.info_size = None
        self.info_warned = False

        self.hits = 0
        self.misses = 0
        self.detected = False
        self.frames_in = 0
        self.frames_used = 0
        self.frames_solved = 0
        self.last_processed = 0.0
        self.last_reject = ''
        self._jpeg = None
        self._jpeg_lock = threading.Lock()

        self.create_subscription(Image, self.image_topic, self.image_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic,
                                 self.info_callback, qos_profile_sensor_data)

        self.detected_pub = self.create_publisher(Bool, 'ring_detected', 10)
        self.info_pub = self.create_publisher(String, 'ring_info', 10)
        self.geometry_pub = self.create_publisher(
            Float32MultiArray, 'ring_geometry', 10)

        self.server = (MjpegServer(self, self.stream_port)
                       if self.stream_port > 0 else None)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f"Ring board detection up. image={self.image_topic} "
            f"info={self.info_topic} board=({self.geometry.describe()}) "
            f"dict={self.dict_name}"
            + (f" stream=http://<jetson-ip>:{self.stream_port}/"
               if self.server else ""))

    # ------------------------------------------------------------------ subs

    def info_callback(self, msg):
        if msg.k[0] <= 0.0:
            return
        size = (int(msg.width), int(msg.height))
        if self.info_size is not None and self.info_size != size:
            # Two cameras, or the colour info replaced by the depth module's.
            # Keep the first one and say so once.
            if not self.info_warned:
                self.info_warned = True
                self.get_logger().warning(
                    f"CameraInfo on {self.info_topic} changed size "
                    f"{self.info_size} -> {size}; ignoring the new one. Is "
                    "camera_info_topic pointing at the colour stream?")
            return
        first = self.K is None
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.info_size = size
        if len(msg.d) >= 5:
            self.D = np.array(msg.d[:5], dtype=np.float64).reshape(5, 1)
        if first:
            self.get_logger().info(
                f"CameraInfo: fx={self.K[0, 0]:.1f} fy={self.K[1, 1]:.1f} "
                f"cx={self.K[0, 2]:.1f} cy={self.K[1, 2]:.1f} at "
                f"{size[0]}x{size[1]}. Distances are calibrated.")

    def image_callback(self, msg):
        self.frames_in += 1
        now = time.monotonic()
        if self.max_fps > 0.0 and now - self.last_processed < 1.0 / self.max_fps:
            return
        self.last_processed = now
        self.frames_used += 1

        try:
            bgr = imgmsg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        if self.image_rotate:
            code = {90: cv2.ROTATE_90_CLOCKWISE,
                    180: cv2.ROTATE_180,
                    270: cv2.ROTATE_90_COUNTERCLOCKWISE}[self.image_rotate]
            bgr = cv2.rotate(bgr, code)

        h, w = bgr.shape[:2]
        K = self.intrinsics(w, h)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        board = self.locator.find(gray, K, self.D)
        accepted, reason = self.accept(board)
        if accepted:
            self.frames_solved += 1
            self.publish_geometry(board, msg.header)
        else:
            self.last_reject = reason

        self.update_debounce(accepted)
        self.publish_info(board, accepted, reason)
        self.render(bgr, board, accepted, reason)

    # ------------------------------------------------------------------ gates

    def intrinsics(self, width, height):
        """The calibration if we have it, a FOV guess if we do not."""
        if self.K is not None:
            if self.info_size == (width, height):
                return self.K
            # Same lens, different stream resolution: the intrinsics scale
            # linearly, which is exact for a simple resize and near enough for
            # a mode change on this sensor.
            sx = width / float(self.info_size[0])
            sy = height / float(self.info_size[1])
            K = self.K.copy()
            K[0, :] *= sx
            K[1, :] *= sy
            return K
        if not self.info_warned:
            self.info_warned = True
            self.get_logger().warning(
                f"No CameraInfo on {self.info_topic}; guessing intrinsics from "
                f"hfov_deg={self.hfov_deg}. Bearings will be about right and "
                "DISTANCES WILL CARRY THE FOV ERROR -- fix the topic before "
                "flying a 20 cm grab depth off them.")
        return fov_K(width, height, math.radians(self.hfov_deg))

    def accept(self, board):
        """Everything that can be checked without knowing where the drone is."""
        if board is None:
            seen = self.locator.last_all_ids
            if seen:
                return False, f"markers {seen} in frame, none on the board"
            return False, "no markers"
        if board.n_markers < self.min_markers:
            return False, f"{board.n_markers} marker(s), want {self.min_markers}"
        if not np.all(np.isfinite(board.grab_cam)):
            return False, "pose not finite"
        if board.distance < self.depth_min or board.distance > self.depth_max:
            return False, (f"distance {board.distance:.2f} m outside "
                           f"[{self.depth_min:.2f}, {self.depth_max:.2f}]")
        if board.worst_px > self.max_reproj_px:
            # The solution does not actually fit the corners it was fitted to.
            # Usually a misdescribed board -- wrong marker_gap, wrong id
            # order, a marker that is not on this board at all -- rather than
            # noise, because noise moves every corner a little and this is the
            # WORST one.
            return False, (f"reprojection {board.worst_px:.1f} px "
                           f"(limit {self.max_reproj_px:.1f})")
        # NOT gated: board.ambiguous. A board within ~10 deg of square-on
        # always reports ambiguous, because the two IPPE solutions really are
        # indistinguishable there -- and square-on is the state the whole
        # approach is trying to reach. What is undetermined is the TILT; the
        # position stays good to a centimetre. Rejecting it would reject
        # exactly the frames the run-in depends on.
        return True, ''

    def update_debounce(self, hit):
        """Same shape as aruco_pose's: N consecutive to latch, M to release."""
        if hit:
            self.hits += 1
            self.misses = 0
            if not self.detected and self.hits >= self.detect_frames:
                self.detected = True
                self.get_logger().info("Ring board acquired.")
        else:
            self.misses += 1
            self.hits = 0
            if self.detected and self.misses >= self.lost_frames:
                self.detected = False
                self.get_logger().info("Ring board lost.")
        msg = Bool()
        msg.data = self.detected
        self.detected_pub.publish(msg)

    # ------------------------------------------------------------------ pubs

    def publish_geometry(self, board, header):
        rows = np.zeros((GEOMETRY_ROWS, GEOMETRY_COLS), dtype=np.float32)
        rows[0] = board.grab_cam
        rows[1] = board.right_cam
        rows[2] = board.up_cam
        rows[3] = board.normal_cam
        rows[4] = (float(board.n_markers),
                   1.0 if board.ambiguous else 0.0,
                   float(board.distance))
        for i, marker_id in enumerate(self.geometry.ids[:GEOMETRY_COLS]):
            rows[5][i] = 1.0 if marker_id in board.marker_ids else 0.0

        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='row', size=GEOMETRY_ROWS,
                                stride=GEOMETRY_ROWS * GEOMETRY_COLS),
            MultiArrayDimension(label='col', size=GEOMETRY_COLS,
                                stride=GEOMETRY_COLS),
        ]
        msg.data = rows.reshape(-1).tolist()
        self.geometry_pub.publish(msg)

    def publish_info(self, board, accepted, reason):
        if board is None:
            text = f"no board ({reason})"
        else:
            text = (f"ids={board.marker_ids} "
                    f"{direction_words(board.grab_cam)} "
                    f"dist={board.distance:.2f} "
                    f"rms={board.rms_px:.1f}px"
                    f"{' AMBIGUOUS' if board.ambiguous else ''}"
                    f"{'' if accepted else ' REJECTED: ' + reason}")
        msg = String()
        msg.data = text
        self.info_pub.publish(msg)

    # ---------------------------------------------------------------- render

    def render(self, bgr, board, accepted, reason):
        if self.server is None and not self.annotate:
            return
        if self.annotate and board is not None:
            colour = (0, 255, 0) if accepted else (0, 165, 255)
            for s in board.per_marker:
                quad = s.corners.reshape(-1, 1, 2).astype(np.int32)
                cv2.polylines(bgr, [quad], True, colour, 2)
                centre = s.corners.reshape(4, 2).mean(axis=0).astype(int)
                cv2.putText(bgr, str(s.marker_id), tuple(centre),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            line = (f"ids={board.marker_ids} d={board.distance:.2f}m "
                    f"{'OK' if accepted else reason}")
        else:
            line = reason or 'no board'
        cv2.putText(bgr, line, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if accepted else (0, 0, 255), 2)

        if self.server is None:
            return
        if self.stream_scale != 1.0:
            bgr = cv2.resize(bgr, None, fx=self.stream_scale,
                             fy=self.stream_scale, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', bgr,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if ok:
            with self._jpeg_lock:
                self._jpeg = buf.tobytes()

    def latest_jpeg(self):
        """Called by MjpegServer's handler threads."""
        with self._jpeg_lock:
            return self._jpeg

    def report(self):
        self.get_logger().info(
            f"{self.frames_in / 5.0:.1f} Hz in, {self.frames_used / 5.0:.1f} Hz "
            f"processed, {self.frames_solved} solved, detected="
            f"{self.detected}"
            + (f", last reject: {self.last_reject}" if self.last_reject else ""))
        self.frames_in = 0
        self.frames_used = 0
        self.frames_solved = 0

    def destroy_node(self):
        if self.server is not None:
            self.server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RingDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
