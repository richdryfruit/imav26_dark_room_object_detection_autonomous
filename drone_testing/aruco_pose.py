#!/usr/bin/env python3
"""
Pose of a downward-facing ArUco marker, as a ROS 2 node.

NOT THE REALSENSE. This node opens the DOWN-FACING USB camera directly with
cv2.VideoCapture (see the camera_index / width / height / fourcc parameters
below). It has nothing to do with the D435i that replaced the ZED for the
window mission, and the 2026 camera swap changed nothing in this file. If you
ever do repoint it at the D435i's colour stream, hfov_deg must go 78 -> 70 and
every number in the README's capture-basket table shrinks with it.

Same detection and the same solvePnP maths as the standalone
aruco_down_pose.py -- IPPE_SQUARE on the four corners of one known-size
marker -- wrapped in a node that publishes the result instead of printing
it, and with the camera read moved onto its own thread.

    /aruco/detected     std_msgs/Bool             debounced: true after
                                                  detect_frames consecutive
                                                  hits, false after
                                                  lost_frames misses
    /aruco/point        geometry_msgs/PointStamped marker centre in the
                                                  CAMERA BODY frame, metres,
                                                  published only on a frame
                                                  where the pose solved
    /aruco/info         std_msgs/String           one human-readable line
    browser             http://<jetson-ip>:8080/  MJPEG of the annotated
                                                  frame. stream_port:=0 off.

WHY THIS NODE PUBLISHES CAMERA-FRAME NUMBERS AND NOTHING ELSE
-------------------------------------------------------------
It deliberately does not know about PX4, NED, or the vehicle's attitude.
Rotating the marker vector into NED needs the airframe's roll/pitch/yaw,
which lives in the flight node, and doing it there means this node can be
run and trusted on a bench with no flight controller attached at all.

The frame is the one aruco_down_pose.py defined:

    +x  RIGHT in the image
    +y  UP in the image (towards the top)
    +z  UP, i.e. opposite to where the camera looks

so a marker below the camera has NEGATIVE z, and height is -z.

WHAT THE FLIGHT NODE DOES WITH IT
    With the camera mounted image-up towards the nose and image-right to
    the vehicle's right, the mapping into body FRD is

        forward = y      right = x      down = -z

    See precision_land.py, which is the only consumer.

ON THE SCALE OF THESE NUMBERS
-----------------------------
fx is derived from hfov_deg, not from a calibration, so it carries whatever
error the quoted field of view has. That error does NOT affect x and y:

    apparent marker width in px   p = fx_true * S / Z_true      (measured)
    solver, using fx = k*fx_true  Z = fx*S/p        = k*Z_true
                                  X = u*Z/fx        = u*Z_true/fx_true = X_true

The inflated range and the deflated bearing cancel exactly, so the LATERAL
offsets are right even when the FOV is wrong. The HEIGHT is not -- it is
scaled by k -- which is why the flight node uses the lidar for height and
takes only x and y from here.

What does not cancel is lens distortion: distortion_coeffs is zero by
default, and a wide lens bends the corners worst at the edge of the frame,
which is where the marker sits when the vehicle is most off-centre. Running
cv2.calibrateCamera on a chessboard and passing the real fx/fy/cx/cy and
distortion removes it. Until then, treat the numbers as good near the
centre and slightly optimistic at the edge.

BENCH TEST -- do this before it ever flies

    ros2 run drone_testing aruco_pose
    ros2 topic echo /aruco/info

Put the marker on the floor, hold the airframe over it, and check that the
reported FORWARD / RIGHT words match where the marker actually is relative
to the nose. precision_land.py has a `bench` mode that prints the same
thing in vehicle terms and commands nothing.
"""

import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String


# OpenCV's optical frame is (x right, y DOWN, z FORWARD along the view axis).
# The body frame used here is (x right, y UP, z UP), so the two differ by a
# 180 degree roll about x. Applying this to tvec puts the marker centre in the
# body frame; applying it to the marker rotation lets yaw be read about +z.
R_CF = np.array([[1.0,  0.0,  0.0],
                 [0.0, -1.0,  0.0],
                 [0.0,  0.0, -1.0]])


# ------------------------------------------------------------- mjpeg stream
#
# Same shape as the server in window_detect.py: it hands out JPEG at whatever
# rate the viewer can take and DROPS frames rather than queueing them, so a
# slow laptop on bad WiFi slows only itself and never the detection loop.

class _MjpegHandler(BaseHTTPRequestHandler):
    """Serves the newest annotated frame as multipart JPEG, for a browser."""

    node = None     # set by MjpegServer before the server starts

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._send_page()
        elif self.path.startswith('/stream'):
            self._send_stream()
        elif self.path.startswith('/snapshot'):
            self._send_snapshot()
        else:
            self.send_error(404)

    def _send_page(self):
        body = (b"<html><head><title>aruco down</title>"
                b"<style>body{background:#111;color:#eee;font-family:sans-serif;"
                b"margin:0;text-align:center}img{max-width:100%}</style></head>"
                b"<body><img src='/stream.mjpg'></body></html>")
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_snapshot(self):
        frame = self.node.latest_jpeg()
        if frame is None:
            self.send_error(503, "no frame yet")
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def _send_stream(self):
        self.send_response(200)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        last = None
        try:
            while True:
                frame = self.node.latest_jpeg()
                if frame is None or frame is last:
                    time.sleep(0.02)
                    continue
                last = frame
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass    # the viewer closed the tab; not an error

    def log_message(self, *args):
        pass        # the default handler logs every frame to stderr


class MjpegServer:
    """Threaded HTTP server that never blocks the ROS callbacks."""

    def __init__(self, node, port):
        handler = type('_Handler', (_MjpegHandler,), {'node': node})
        self.server = ThreadingHTTPServer(('0.0.0.0', port), handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       daemon=True)
        self.thread.start()

    def shutdown(self):
        try:
            self.server.shutdown()
            self.server.server_close()
        except Exception:
            pass


def body_words(forward, right):
    """'FORWARD 0.20 m, RIGHT 0.30 m' -- the sign check a human can read."""
    return (f"{'FORWARD' if forward >= 0 else 'BACK'} {abs(forward):.2f} m, "
            f"{'RIGHT' if right >= 0 else 'LEFT'} {abs(right):.2f} m")


class ArucoPose(Node):

    # ---- camera -----------------------------------------------------------
    CAMERA_INDEX = 1
    # The Logitech B910, by serial number. See camera_device in __init__.
    CAMERA_DEVICE = '/dev/v4l/by-id/usb-046d_0823_1469ADD0-video-index0'
    # 4:3 on purpose. The vertical field of view is what decides how low the
    # vehicle can go before the marker stops fitting in the frame, and 4:3 is
    # markedly taller than 16:9 on this sensor: with a 0.80 m marker and a
    # 78 deg horizontal FOV the marker fills the frame vertically at 0.66 m in
    # 4:3 but already at 0.88 m in 16:9. Lower is better -- it is 20 cm less
    # of blind descent. 800x600 also runs at 30 fps where 1280x960 drops to 15,
    # and for a control loop the frame rate is worth more than the pixels.
    WIDTH = 800
    HEIGHT = 600
    FOURCC = 'MJPG'
    CAMERA_FPS = 30.0

    # ---- the marker -------------------------------------------------------
    MARKER_ID = 0
    MARKER_SIZE = 0.80          # m, edge length
    ARUCO_DICT = 'DICT_5X5_50'
    HFOV_DEG = 78.0             # horizontal field of view. See the header:
                                # this scales the reported HEIGHT but cancels
                                # out of x and y.

    # ---- detection --------------------------------------------------------
    DETECT_RATE = 20.0          # Hz the newest frame is processed at
    DETECT_FRAMES = 3           # consecutive hits before /aruco/detected goes true
    LOST_FRAMES = 5             # consecutive misses before it goes false again

    # ---- output -----------------------------------------------------------
    STREAM_PORT = 8080          # 0 disables the browser stream
    STREAM_SCALE = 0.6
    JPEG_QUALITY = 70
    IMAGE_ROTATE = 0            # 0 | 90 | 180 | 270, applied BEFORE detection.
                                # Use this if the camera is bolted on rotated:
                                # the +x-right / +y-up frame follows the
                                # rotated image, so the mounting convention
                                # the flight node assumes stays true.

    def __init__(self):
        super().__init__('aruco_pose')

        self.camera_index = int(self.declare_parameter(
            'camera_index', self.CAMERA_INDEX).value)
        # A STABLE device path, which wins over camera_index when set.
        #
        # /dev/videoN is assigned in plug order and the RealSense takes SIX of
        # them (video0-5), so the down camera's index moves whenever the
        # RealSense is connected or not. An index set with the RealSense
        # unplugged opens a RealSense node once it is plugged in: the marker is
        # never seen, and the RealSense driver can lose its device to it,
        # taking the VIO and the window detection down too. The by-id path
        # names this webcam by its serial number and never moves.
        self.camera_device = str(self.declare_parameter(
            'camera_device', self.CAMERA_DEVICE).value).strip()
        self.width = int(self.declare_parameter('width', self.WIDTH).value)
        self.height = int(self.declare_parameter('height', self.HEIGHT).value)
        self.fourcc = str(self.declare_parameter('fourcc', self.FOURCC).value)
        self.camera_fps = float(self.declare_parameter(
            'camera_fps', self.CAMERA_FPS).value)

        self.marker_id = int(self.declare_parameter(
            'marker_id', self.MARKER_ID).value)
        self.marker_size = float(self.declare_parameter(
            'marker_size', self.MARKER_SIZE).value)
        dict_name = str(self.declare_parameter(
            'aruco_dict', self.ARUCO_DICT).value)
        self.hfov_deg = float(self.declare_parameter(
            'hfov_deg', self.HFOV_DEG).value)

        detect_rate = float(self.declare_parameter(
            'detect_rate', self.DETECT_RATE).value)
        self.detect_frames = int(self.declare_parameter(
            'detect_frames', self.DETECT_FRAMES).value)
        self.lost_frames = int(self.declare_parameter(
            'lost_frames', self.LOST_FRAMES).value)

        self.stream_port = int(self.declare_parameter(
            'stream_port', self.STREAM_PORT).value)
        self.stream_scale = float(self.declare_parameter(
            'stream_scale', self.STREAM_SCALE).value)
        self.jpeg_quality = int(self.declare_parameter(
            'jpeg_quality', self.JPEG_QUALITY).value)
        self.image_rotate = int(self.declare_parameter(
            'image_rotate', self.IMAGE_ROTATE).value)
        self.show_gui = bool(self.declare_parameter('show_gui', False).value)

        # Optional real calibration. Left empty by default, in which case fx
        # comes from hfov_deg and distortion is assumed zero -- see the header
        # for exactly what that costs you.
        fx = float(self.declare_parameter('fx', 0.0).value)
        fy = float(self.declare_parameter('fy', 0.0).value)
        cx = float(self.declare_parameter('cx', 0.0).value)
        cy = float(self.declare_parameter('cy', 0.0).value)
        dist = list(self.declare_parameter(
            'distortion_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0]).value)
        self._cal = (fx, fy, cx, cy)
        self.D = np.array(dist, dtype=np.float64).reshape(-1, 1)

        try:
            dictionary = cv2.aruco.getPredefinedDictionary(
                getattr(cv2.aruco, dict_name))
        except AttributeError:
            raise SystemExit(f"Unknown aruco_dict '{dict_name}'.")

        # OpenCV < 4.7 (e.g. Ubuntu 24.04's apt 4.6) has the old aruco API:
        # DetectorParameters() there builds an object that segfaults on the
        # first attribute write, and there is no ArucoDetector.
        new_api = hasattr(cv2.aruco, 'ArucoDetector')
        params = (cv2.aruco.DetectorParameters() if new_api
                  else cv2.aruco.DetectorParameters_create())
        # Sub-pixel corner refinement. This is the difference between a corner
        # good to a pixel and one good to a tenth, and every centimetre of
        # lateral accuracy comes through those four corners.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if new_api:
            self._detect = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers
        else:
            self._detect = lambda img: cv2.aruco.detectMarkers(  # noqa: E731
                img, dictionary, parameters=params)

        s = self.marker_size / 2.0
        # TL, TR, BR, BL -- cv2.aruco's corner order, and the order
        # SOLVEPNP_IPPE_SQUARE requires. Do not reorder these.
        self.objp = np.array([[-s,  s, 0],
                              [ s,  s, 0],
                              [ s, -s, 0],
                              [-s, -s, 0]], dtype=np.float64)

        self.K = None
        self.detected = False
        self._hits = 0
        self._misses = 0

        self._frame = None
        self._frame_seq = 0
        self._processed_seq = -1
        self._frame_lock = threading.Lock()
        self._jpeg = None
        self._stop = threading.Event()

        self.detected_pub = self.create_publisher(Bool, '/aruco/detected', 10)
        self.point_pub = self.create_publisher(PointStamped, '/aruco/point', 10)
        self.info_pub = self.create_publisher(String, '/aruco/info', 10)

        # EVERY visible marker, one message each, with the id in frame_id as
        # "aruco:<id>". The three topics above describe one marker -- the
        # configured marker_id -- and say nothing about which marker a
        # detection belongs to, which is all precision_land ever needs and is
        # deliberately left exactly as it was. A mission that visits several
        # different markers in sequence needs to tell them apart, and that is
        # what this topic is for: the subscriber filters on the id and can
        # never act on a fix from the wrong pad.
        self.marker_points_pub = self.create_publisher(
            PointStamped, '/aruco/marker_points', 10)

        # SITL: take frames from a ROS image topic instead of a USB camera.
        # Empty (the default) keeps the real camera path below untouched.
        self.image_topic = str(self.declare_parameter('image_topic', '').value).strip()
        if self.image_topic:
            self.camera_device = ''
        source = self.image_topic or self.camera_device or self.camera_index
        if self.image_topic:
            self.cap = None
        elif self.camera_device and not os.path.exists(self.camera_device):
            raise SystemExit(
                f"camera_device {self.camera_device} does not exist. Is the down "
                "camera plugged in? `ls /dev/v4l/by-id/` lists what is.")
        if not self.image_topic:
            self.cap = (cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
                        if self.camera_device else cv2.VideoCapture(self.camera_index))
        if self.cap is not None and not self.cap.isOpened():
            raise SystemExit(f"Could not open camera {source}.")
        self.get_logger().info(f"Down camera: {source}")
        if self.cap is not None and len(self.fourcc) == 4:
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc(*self.fourcc))
        if self.cap is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if self.camera_fps > 0.0:
                self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
            # One-deep buffer: we always want the NEWEST frame. A queued frame
            # is latency, and latency in a landing loop is phase lag.
            try:
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

            # The grab runs on its own thread and always keeps only the latest
            # frame. cap.read() blocks for a frame interval, and doing that
            # inside a ROS timer would stall this node's executor for 33 ms.
            self._grab_thread = threading.Thread(target=self._grab_loop,
                                                 daemon=True)
            self._grab_thread.start()
        else:
            from sensor_msgs.msg import Image
            self.create_subscription(Image, self.image_topic,
                                     self._image_callback, 1)

        self.stream = None
        if self.stream_port:
            try:
                self.stream = MjpegServer(self, self.stream_port)
                self.get_logger().info(
                    f"Browser stream on http://<jetson-ip>:{self.stream_port}/")
            except Exception as exc:
                self.get_logger().error(
                    f"Could not start the stream on port {self.stream_port}: {exc}")

        self.timer = self.create_timer(1.0 / max(detect_rate, 1.0), self.detect_once)

        actual_w = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    if self.cap is not None else self.width)
        actual_h = (int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    if self.cap is not None else self.height)
        self.get_logger().warning(
            f"ArUco down-camera pose: id {self.marker_id}, "
            f"{self.marker_size * 100:.0f} cm marker, {dict_name}, "
            f"camera {actual_w}x{actual_h} @ {self.camera_fps:.0f} fps, "
            f"processing at {detect_rate:.0f} Hz.")
        if fx <= 0.0:
            self.get_logger().warning(
                f"No calibration given: fx derived from hfov_deg={self.hfov_deg:.1f}. "
                "x/y are unaffected by a wrong FOV, HEIGHT is scaled by it, and "
                "lens distortion is NOT corrected. Do not use the height.")

    # ------------------------------------------------------------- capture

    def _grab_loop(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.02)
                continue
            with self._frame_lock:
                self._frame = frame
                self._frame_seq += 1

    def _image_callback(self, msg):
        """sensor_msgs/Image -> BGR, stored like _grab_loop does.

        SITL's camera, or on the aircraft imav_bringup's usb_cam on the C920
        (which owns the device, so this node must not open it too).
        """
        from drone_testing.window_detect import imgmsg_to_bgr
        try:
            frame = imgmsg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(f"Down camera frame: {exc}",
                                    throttle_duration_sec=5.0)
            return
        with self._frame_lock:
            self._frame = np.ascontiguousarray(frame)
            self._frame_seq += 1

    def latest_jpeg(self):
        return self._jpeg

    # ------------------------------------------------------------ intrinsics

    def _intrinsics(self, w, h):
        fx, fy, cx, cy = self._cal
        if fx > 0.0:
            return np.array([[fx, 0, cx if cx > 0 else w / 2.0],
                             [0, fy if fy > 0 else fx, cy if cy > 0 else h / 2.0],
                             [0, 0, 1.0]], dtype=np.float64)
        f = (w / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        return np.array([[f, 0, w / 2.0],
                         [0, f, h / 2.0],
                         [0, 0, 1.0]], dtype=np.float64)

    # -------------------------------------------------------------- detect

    def detect_once(self):
        with self._frame_lock:
            if self._frame is None or self._frame_seq == self._processed_seq:
                return      # nothing new since last time
            frame = self._frame
            self._processed_seq = self._frame_seq

        if self.image_rotate:
            rot = {90: cv2.ROTATE_90_CLOCKWISE,
                   180: cv2.ROTATE_180,
                   270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(self.image_rotate)
            if rot is not None:
                frame = cv2.rotate(frame, rot)

        h, w = frame.shape[:2]
        if self.K is None:
            self.K = self._intrinsics(w, h)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)
        seen = ids.flatten().tolist() if ids is not None else []

        pose = None
        quad = None

        # Solve every marker in the frame, not just the configured one, and
        # publish each on /aruco/marker_points tagged with its id. All markers
        # are assumed to be marker_size across -- self.objp is built from that
        # one number, and a marker of a different physical size solved against
        # it comes out at the wrong RANGE, which would put the vehicle over
        # the wrong point. If mixed sizes ever appear on the course this is
        # the line that has to grow a per-id table.
        now = self.get_clock().now().to_msg()
        for idx, mid in enumerate(seen):
            c_any = corners[idx].reshape(4, 2).astype(np.float64)
            ok, _, t_any = cv2.solvePnP(self.objp, c_any, self.K, self.D,
                                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            px, py, pz = (float(v) for v in R_CF @ t_any.reshape(3))
            m = PointStamped()
            m.header.stamp = now
            m.header.frame_id = f'aruco:{int(mid)}'
            m.point.x, m.point.y, m.point.z = px, py, pz
            self.marker_points_pub.publish(m)
            # `pose is None` keeps the legacy behaviour exactly: the old code
            # used seen.index(), which takes the FIRST occurrence. A duplicate
            # id in one frame is pathological, but it must not quietly change
            # which corner set precision_land is flown on.
            if mid == self.marker_id and pose is None:
                pose = (px, py, pz)
                quad = c_any

        self._update_debounce(pose is not None)

        if pose is not None:
            x, y, z = pose
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_body'
            msg.point.x, msg.point.y, msg.point.z = x, y, z
            self.point_pub.publish(msg)
            # forward = y, right = x, for the mounting this package assumes.
            line = (f"id {self.marker_id} SEEN  cam x={x:+.3f} y={y:+.3f} "
                    f"z={z:+.3f} m  height={-z:.3f} m  |  marker is "
                    f"{body_words(y, x)} of the camera")
        else:
            line = f"id {self.marker_id} not visible (seen: {seen or '-'})"

        self.detected_pub.publish(Bool(data=self.detected))
        self.info_pub.publish(String(data=line))
        self.get_logger().info(line, throttle_duration_sec=1.0)

        self._render(frame, quad, line)

    def _update_debounce(self, hit):
        """Same debounce shape as window_detect: N hits on, M misses off.

        A single frame either way is noise -- a glint, a motion-blurred
        corner -- and neither a lock nor a dropout should turn on one.
        """
        if hit:
            self._misses = 0
            self._hits += 1
            if not self.detected and self._hits >= self.detect_frames:
                self.detected = True
                self.get_logger().warning("Marker ACQUIRED.")
        else:
            self._hits = 0
            self._misses += 1
            if self.detected and self._misses >= self.lost_frames:
                self.detected = False
                self.get_logger().warning("Marker LOST.")

    # -------------------------------------------------------------- render

    def _render(self, frame, quad, line):
        if self.stream is None and not self.show_gui:
            return

        img = frame.copy()
        h, w = img.shape[:2]
        if quad is not None:
            cv2.polylines(img, [quad.reshape(-1, 1, 2).astype(int)],
                          True, (0, 255, 0), 3)
        cv2.drawMarker(img, (w // 2, h // 2), (90, 90, 100),
                       cv2.MARKER_CROSS, 28, 1)
        cv2.putText(img, line[:78], (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255) if quad is not None else (120, 120, 255),
                    1, cv2.LINE_AA)

        if self.show_gui:
            cv2.imshow('aruco down', img)
            cv2.waitKey(1)

        if self.stream is not None:
            out = img
            if 0.0 < self.stream_scale < 1.0:
                out = cv2.resize(img, None, fx=self.stream_scale,
                                 fy=self.stream_scale,
                                 interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(
                '.jpg', out, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                self._jpeg = buf.tobytes()

    # ------------------------------------------------------------- shutdown

    def destroy_node(self):
        self._stop.set()
        if self.stream is not None:
            self.stream.shutdown()
        try:
            self.cap.release()
        except Exception:
            pass
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoPose()
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
