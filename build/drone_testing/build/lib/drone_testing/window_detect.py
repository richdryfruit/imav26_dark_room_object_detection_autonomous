"""
Window detection from the RealSense D435i, as a ROS 2 node.

Same detection logic as the original standalone script -- HSV threshold on
green, largest contour, convex hull approximated to a quadrilateral, corner
depths sampled just inside each corner -- but the frames now come from the
realsense2_camera node over ROS topics instead of from a pyzed stream.

    colour /camera/camera/color/image_raw              (rgb8, 1280x720)
    depth  /camera/camera/aligned_depth_to_color/image_raw
                                                      (16UC1, MILLIMETRES,
                                                       registered to the
                                                       colour frame)

PORTED FROM THE ZED (see the IMAGE_TOPIC block below for the full reasoning).
The three things that changed, and nothing else did:

    topics        /zed/zed_node/... -> /camera/camera/...  The depth one must
                  be the ALIGNED variant; the D435i's depth lens is not the
                  colour lens, so the raw depth map is not registered to the
                  image the contour was found in.
    depth units   32FC1 metres -> 16UC1 millimetres. Already handled by
                  imgmsg_to_depth(); no code change, and 0 still maps to NaN.
    fallback FOV  90 deg -> 69 deg. The D435i's COLOUR sensor is much
                  narrower than the ZED was. This only matters until the
                  first CameraInfo lands, but it matters a lot if it is the
                  only thing that ever lands.

Colour comes out of the D435i as rgb8, where the ZED published bgra8.
imgmsg_to_bgr() already converted both, so that needed no change either.

Both topic names are parameters, so if your camera runs under a different
namespace you do not have to touch the code:

    ros2 run drone_testing window_detect --ros-args \
        -p image_topic:=/camera/camera/color/image_raw \
        -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw

Check what the driver actually publishes with:

    ros2 topic list | grep camera

If the aligned depth topic is missing, realsense2_camera was started without
align_depth.enable:=true. Start it with that, or run with use_depth:=false
and accept a detection with no distances (and so no /window_geometry, and so
no traversal).

Deliberately no cv_bridge: its compiled extension is built against the
distro's NumPy, and a pip-installed NumPy 2 in ~/.local makes it segfault on
the first frame. imgmsg_to_bgr() / imgmsg_to_depth() below do the same job in
pure NumPy, so the node runs whichever NumPy is on the path.

Depth is NOT synchronised with the image through a message filter: the most
recent depth frame is kept and used if it is younger than depth_max_age.
The two come out of the same SDK grab at the same rate, so approximate
pairing is what a synchroniser would give anyway, and a missing or stale
depth frame degrades to "detect the window, report no distances" instead of
dropping the detection entirely.

WHAT IT PUBLISHES

    /window_detected        std_msgs/Bool     debounced: true only after
                                              detect_frames consecutive hits,
                                              false after lost_frames misses
    /window_info            std_msgs/String   pipe-separated detail line,
                                              see publish_info()
    /window_geometry        Float32MultiArray 5x3 of (depth_m, azimuth_deg,
                                              elevation_deg) for the four
                                              corners and the centre, in the
                                              CAMERA frame. This is the input
                                              window_traverse turns into a
                                              window pose in NED -- see
                                              publish_geometry().
    /window_detection/image sensor_msgs/Image the annotated frame

HOW TO SEE IT

    terminal   this node logs one line a second either way, and one WARN the
               moment the detection latches or is lost
               ros2 topic echo /window_detected
               ros2 topic echo /window_info
    picture    ros2 run rqt_image_view rqt_image_view /window_detection/image
    on the LCD lcd_status shows "win YES/no" on row 4 (it subscribes to
               /window_detected itself)

SEEING IT WHILE IT FLIES

    browser   http://<jetson-ip>:8080/  -- an MJPEG stream of the annotated
              frame, no ROS needed on the viewing machine. stream_port:=0
              turns it off.
    ROS       /window_detection/image/compressed is JPEG, small enough for
              WiFi; rqt_image_view picks it by selecting the 'compressed'
              transport. The raw topic stays for anything on the Jetson.

Both are fed by one JPEG encode, downscaled by stream_scale (0.5 = quarter
the pixels) at jpeg_quality. Nothing here ever puts a raw frame on the
network.

`-p show_windows:=true` brings back the two cv2.imshow windows from the
original script. That needs a display, so leave it false on the Jetson
unless you are sitting in front of it with a monitor plugged in.
"""

import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, String


HSV_RANGES = {
    "blue": [(np.array([95, 80, 40]), np.array([130, 255, 255]))],
    "red": [
        (np.array([0, 100, 60]), np.array([10, 255, 255])),
        (np.array([170, 100, 60]), np.array([180, 255, 255])),
    ],
    "green": [(np.array([35, 40, 30]), np.array([90, 255, 255]))],
}


def get_median_depth(depth_img, u, v, box=3):
    h, w = depth_img.shape[:2]
    u = min(max(u, box), w - 1 - box)
    v = min(max(v, box), h - 1 - box)
    depth_values = depth_img[v - box:v + box + 1, u - box:u + box + 1].flatten()
    valid_depths = depth_values[(depth_values > 0) & (~np.isnan(depth_values)) & (~np.isinf(depth_values))]
    if len(valid_depths) > 0:
        return np.median(valid_depths)
    return 0.0


def sample_corner_depth(depth_img, u, v, center, inset=10, box=4):
    """Depth of the window FRAME at one corner, not of what is behind it.

    The sample point is pushed in from the corner towards the middle of the
    quad, because a point exactly on the corner straddles the edge. How far
    in is the whole problem: the frame is only a few pixels wide once the
    window is a few metres away, so one fixed inset that lands on the
    material at 1.5 m lands in the OPENING at 4 m, and what comes back is the
    depth of the far wall seen through the window.

    That is not a small error and it is not random. One corner reading the
    wall behind while the other three read the frame is the single biggest
    source of "corner depths disagree" rejections downstream, and a frame
    reconstructed from three good corners and one bad one is a window in the
    wrong place, the wrong size, and at the wrong angle.

    So sample a LADDER of insets and keep the NEAREST plausible reading.
    Everything visible through the aperture is further away than the frame
    around it -- that is what makes it an aperture -- so of the readings
    taken along a line from the corner inwards, the smallest is the one that
    landed on the material. The others are the room beyond it.
    """
    cu, cv = center
    du, dv = cu - u, cv - v
    norm = np.hypot(du, dv) + 1e-6
    ux, uy = du / norm, dv / norm

    h, w = depth_img.shape[:2]
    best_depth = 0.0
    best_point = (int(min(max(u, 0), w - 1)), int(min(max(v, 0), h - 1)))
    for scale in (0.4, 0.7, 1.0, 1.6):
        step = inset * scale
        su = int(round(u + step * ux))
        sv = int(round(v + step * uy))
        su = min(max(su, 0), w - 1)
        sv = min(max(sv, 0), h - 1)
        d = get_median_depth(depth_img, su, sv, box)
        if d <= 0.0:
            continue
        if best_depth <= 0.0 or d < best_depth:
            best_depth = d
            best_point = (su, sv)
    return best_depth, best_point


def hsv_mask(hsv_image, color):
    mask = None
    for lo, hi in HSV_RANGES[color]:
        m = cv2.inRange(hsv_image, lo, hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)

    # close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)

    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel, iterations=2)

    # eroding_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))
    # mask = cv2.erode(mask,eroding_kernel, iterations=1)

    dialate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
    mask = cv2.dilate(mask,dialate_kernel, iterations=2)

    return mask


def approx_quad(contour):
    peri = cv2.arcLength(contour, True)
    for eps in np.linspace(0.01, 0.06, 6):
        approx = cv2.approxPolyDP(contour, eps * peri, True)
        if len(approx) == 4:
            return approx
    return None


def window_detection(mask, min_area=1500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    hull = cv2.convexHull(largest)
    approx = approx_quad(hull)
    if approx is None or not cv2.isContourConvex(approx):
        return None
    return approx


def border_margin_px(quad, shape):
    """Smallest gap, in pixels, between any corner of the quad and the image edge.

    This is the whole truncation test, and it exists because the detector
    cannot otherwise tell a window from PART of a window. findContours works
    on a mask that is clipped by the image, so a window hanging off the side
    of the frame produces a contour that simply runs along the border -- still
    convex, still four-sided, still passing approx_quad and every downstream
    planarity and side-ratio check, but describing an aperture that is
    narrower than the real one and whose centre is offset from the real one by
    half of whatever was cut off. Flying at that centre is flying at a point
    that is not the middle of the window.

    A margin at or below the caller's threshold means "assume something is
    outside the frame". It cannot distinguish a window that is genuinely
    truncated from one that merely ends near the edge, and it does not try to:
    both are measurements that should not steer an aircraft.
    """
    h, w = shape[:2]
    margin = float('inf')
    for u, v in np.asarray(quad, dtype=float).reshape(-1, 2):
        margin = min(margin, u, v, (w - 1) - u, (h - 1) - v)
    return float(margin)


def filter_depth(new_d, prev_d, alpha=0.3):
    if new_d is not None and new_d > 0:
        if prev_d is None or prev_d == 0:
            return new_d, new_d
        filtered = alpha * new_d + (1 - alpha) * prev_d
        return filtered, filtered
    return prev_d, prev_d


# ---------------------------------------------------------------- conversion
#
# sensor_msgs/Image <-> numpy by hand, instead of through cv_bridge.
#
# cv_bridge's conversion lives in a compiled extension (cv_bridge_boost) that
# is built against whatever NumPy the distro shipped. A pip-installed NumPy 2
# in ~/.local shadows that one, the extension's C API lookup fails, and the
# process dies with SIGSEGV on the first frame -- which is exactly what
# happened on the Jetson (exit code -11). None of this code is compiled, so
# it does not care which NumPy is on the path.

# Encodings a RealSense (and most cameras) publish, keyed in LOWER CASE --
# ROS spells them '32FC1' and '16UC1', so every lookup lowercases first.
# The D435i uses rgb8 for colour and 16uc1 for depth; the ZED this was
# written against used bgra8 and 32fc1. All four are in the table, so the
# camera swap cost nothing here.
_DTYPES = {
    'mono8': (np.uint8, 1), 'mono16': (np.uint16, 1),
    '8uc1': (np.uint8, 1), '8uc3': (np.uint8, 3), '8uc4': (np.uint8, 4),
    'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3),
    'rgba8': (np.uint8, 4), 'bgra8': (np.uint8, 4),
    '16uc1': (np.uint16, 1), '32fc1': (np.float32, 1),
}


def imgmsg_to_array(msg):
    """sensor_msgs/Image -> numpy array, in the message's own encoding."""
    enc = msg.encoding.lower()
    if enc not in _DTYPES:
        raise ValueError(f"unsupported image encoding '{msg.encoding}'")
    dtype, channels = _DTYPES[enc]
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')

    data = np.frombuffer(msg.data, dtype=dtype)
    # step is the row stride in BYTES and may be padded past width*channels.
    stride = msg.step // dtype.itemsize
    array = data[:msg.height * stride].reshape(msg.height, stride)
    array = array[:, :msg.width * channels]
    if channels > 1:
        array = array.reshape(msg.height, msg.width, channels)
    return array


def imgmsg_to_bgr(msg):
    """sensor_msgs/Image -> a 3-channel BGR image, whatever it came in as.

    The D435i publishes colour as rgb8, so the RGB2BGR branch below is the
    live one now; the ZED published bgra8 and took the BGRA2BGR branch. Both
    are kept -- this function is what makes the node camera-agnostic, and
    deleting the unused branch is how the next camera swap becomes a code
    change instead of a parameter.
    """
    enc = msg.encoding.lower()
    array = imgmsg_to_array(msg)

    if enc in ('bgr8', '8uc3'):
        return array.copy()
    if enc == 'rgb8':
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if enc in ('bgra8', '8uc4'):
        return cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(array, cv2.COLOR_RGBA2BGR)
    if enc in ('mono8', '8uc1'):
        return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
    if enc in ('mono16', '16uc1'):
        return cv2.cvtColor((array >> 8).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported colour encoding '{msg.encoding}'")


def imgmsg_to_depth(msg):
    """sensor_msgs/Image -> a float32 depth map in METRES.

    32FC1 is already metres (what the ZED published). 16UC1 is the millimetre
    convention (what the RealSense publishes) and 0 there means "no reading"
    -- it is turned into NaN so the median filter in get_median_depth()
    rejects it rather than averaging a zero in.

    That NaN branch is load-bearing on the D435i in a way it was not on the
    ZED. Stereo depth on a thin window frame drops out far more often than it
    reads wrong, so 0 is the COMMON failure, not the rare one; letting one
    through as "0.00 m" would put a window corner at the camera's own origin.
    """
    array = imgmsg_to_array(msg)
    if msg.encoding.lower() == '16uc1':
        depth = array.astype(np.float32) / 1000.0
        depth[array == 0] = np.nan
        return depth
    return array.astype(np.float32, copy=False)


def array_to_imgmsg(array, encoding, header):
    """numpy array -> sensor_msgs/Image, for the annotated output topic."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(array.strides[0])
    msg.data = np.ascontiguousarray(array).tobytes()
    return msg


# ------------------------------------------------------------- mjpeg stream
#
# Watching the annotated frame from a laptop needs to work over WiFi, and raw
# bgr8 at 1280x720x15 fps is ~40 MB/s, which WiFi will not carry -- so nothing
# below ever ships a raw frame. This server hands out JPEG at whatever rate the
# viewer can take, and drops frames rather than queueing them, so a slow viewer
# slows itself down and not the detection loop.

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
        body = (b"<html><head><title>window detection</title>"
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
                    # Nothing new. Sleeping here rather than spinning is what
                    # keeps this thread off the CPU the detection needs.
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


class WindowDetect(Node):

    # Topic defaults. CHANGED FOR THE REALSENSE D435i (was the ZED).
    #
    # These are the realsense2_camera (ROS 2) names under the default
    # /camera/camera namespace, verified live on this airframe's D435i
    # (serial 040322072759, FW 5.16.0.1):
    #
    #     colour  /camera/camera/color/image_raw                rgb8, 1280x720
    #     depth   /camera/camera/aligned_depth_to_color/image_raw   16UC1, mm
    #
    # Two things about that depth topic and neither is optional:
    #
    #   * It is the ALIGNED one. The D435i's depth sensor is a different,
    #     wider lens than the colour sensor and sits ~15 mm to its left, so
    #     /camera/camera/depth/image_rect_raw is NOT pixel-registered to the
    #     colour frame. Sampling a corner found in the colour image out of
    #     the unaligned depth map reads the wrong part of the scene, and at
    #     a window frame -- a thin object with a wall metres behind it --
    #     "the wrong part of the scene" is exactly the failure the corner
    #     filters in window_traverse exist to catch. The launch file passes
    #     align_depth.enable:=true; if you start realsense2_camera by hand,
    #     you must too, or this topic does not exist at all.
    #
    #   * It is 16UC1 in MILLIMETRES, where the ZED published 32FC1 metres.
    #     imgmsg_to_depth() below already handled both encodings, so this
    #     needed no code change -- but it is the reason it is written that
    #     way, and 0 (RealSense's "no reading") becomes NaN there rather
    #     than a spurious zero-metre corner.
    IMAGE_TOPIC = '/camera/camera/color/image_raw'
    DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
    CAMERA_INFO_TOPIC = '/camera/camera/color/camera_info'

    # Fallback intrinsics, used only until the first CameraInfo arrives (and
    # for good if camera_info_topic is wrong). Getting this wrong scales every
    # angle the geometry topic reports, so check the log line that says which
    # one is in use.
    #
    # CHANGED FOR THE D435i: 90.0 -> 69.0. The gen-1 ZED at HD720 was about
    # 90 deg horizontally; the D435i's COLOUR sensor is 69 deg (nominal) and
    # measured 70.4 deg on this unit (CameraInfo fx=906.84 at width 1280 ->
    # 2*atan(640/906.84)). Note this is the colour sensor specifically: the
    # D435i's depth/infra pair is much wider at 87 deg, and quoting that
    # number here would inflate every reported azimuth by a quarter.
    FALLBACK_HFOV_DEG = 69.0
    MAX_FPS = 10.0              # cap on the detection pipeline; 0 disables it

    # Debounce. A single frame's worth of green is not a window: one flash of
    # colour must not be able to stop a yaw sweep, and one dropped frame must
    # not unstick a vehicle that has already locked onto the real thing.
    DETECT_FRAMES = 3
    LOST_FRAMES = 5

    MIN_AREA = 1500             # px^2, smallest contour taken seriously
    PADDING = 5                 # px each corner is pulled inwards by
    BORDER_MARGIN = 12          # px. A quad with a corner closer than this to
                                # the image edge is reported TRUNCATED -- see
                                # border_margin_px. Comfortably more than the
                                # 4 px the mask dilation adds, so a window that
                                # really is clear of the edge is not flagged by
                                # its own morphology.
    CORNER_ALPHA = 0.4          # corner position smoothing
    DEPTH_ALPHA = 0.3           # corner depth smoothing
    DEPTH_MAX_AGE = 0.5         # s a depth frame stays usable for
    DEPTH_SCALE = 1.0           # multiplier on the raw depth. ROS depth is in
                                # metres; set 100.0 if you want the centimetres
                                # the standalone script printed.
    LOG_PERIOD = 1.0            # s between the routine status lines
    JPEG_QUALITY = 60           # good enough to judge a detection by, about a
                                # tenth the bytes of quality 95
    STREAM_PORT = 8080          # 0 disables the browser stream

    def __init__(self):
        super().__init__('window_detect')

        self.image_topic = str(self.declare_parameter('image_topic', self.IMAGE_TOPIC).value)
        self.depth_topic = str(self.declare_parameter('depth_topic', self.DEPTH_TOPIC).value)
        self.use_depth = bool(self.declare_parameter('use_depth', True).value)
        self.show_windows = bool(self.declare_parameter('show_windows', False).value)
        self.publish_image = bool(self.declare_parameter('publish_image', True).value)
        self.publish_mask = bool(self.declare_parameter('publish_mask', False).value)
        self.publish_compressed = bool(self.declare_parameter(
            'publish_compressed', True).value)
        self.jpeg_quality = int(self.declare_parameter(
            'jpeg_quality', self.JPEG_QUALITY).value)
        self.stream_port = int(self.declare_parameter(
            'stream_port', self.STREAM_PORT).value)
        # Downscale before encoding. Halving each side quarters the bytes and
        # a window is still perfectly judgeable at 640x360.
        self.stream_scale = float(self.declare_parameter('stream_scale', 0.5).value)
        self.color = str(self.declare_parameter('color', 'green').value).strip().lower()
        self.min_area = float(self.declare_parameter('min_area', float(self.MIN_AREA)).value)
        self.border_margin = float(self.declare_parameter(
            'border_margin', float(self.BORDER_MARGIN)).value)
        self.detect_frames = int(self.declare_parameter('detect_frames', self.DETECT_FRAMES).value)
        self.lost_frames = int(self.declare_parameter('lost_frames', self.LOST_FRAMES).value)
        self.depth_scale = float(self.declare_parameter('depth_scale', self.DEPTH_SCALE).value)
        self.depth_units = 'cm' if abs(self.depth_scale - 100.0) < 1e-6 else 'm'
        self.camera_info_topic = str(self.declare_parameter(
            'camera_info_topic', self.CAMERA_INFO_TOPIC).value).strip()
        if self.camera_info_topic in ('', 'auto'):
            # Derive it from image_topic by swapping the last segment. Every
            # image_transport publisher puts CameraInfo next to the image it
            # describes, so this is right by construction -- and it cannot
            # drift out of step with image_topic the way a separately
            # defaulted topic name can. That drift is not hypothetical: the
            # launch file shipped image_topic on zed_wrapper's newer
            # .../rgb/color/rect/image naming while camera_info_topic still
            # said .../rgb/camera_info, so CameraInfo never arrived and every
            # flight ran on the guessed fallback FOV. The same trap exists on
            # the RealSense: /camera/camera/color/image_raw's CameraInfo is
            # /camera/camera/color/camera_info, but the ALIGNED DEPTH topic
            # has a camera_info of its own with the DEPTH intrinsics in it.
            # Deriving from image_topic picks the colour one, which is the
            # frame the contour was actually found in and therefore the only
            # correct answer.
            self.camera_info_topic = (
                self.image_topic.rsplit('/', 1)[0] + '/camera_info')
            self.get_logger().info(
                f"camera_info_topic derived from image_topic: "
                f"{self.camera_info_topic}")
        self.publish_geometry_topic = bool(self.declare_parameter(
            'publish_geometry', True).value)
        self.fallback_hfov = math.radians(float(self.declare_parameter(
            'fallback_hfov_deg', self.FALLBACK_HFOV_DEG).value))

        # Cap on how often the HSV/contour/depth pipeline actually runs. The
        # D435i delivers 30 fps and the pipeline is the single largest CPU
        # consumer on the companion; the aircraft approaches at 0.3-0.45 m/s
        # and the traversal node medians samples over a 2.5 s buffer, so
        # anything above ~10 Hz buys accuracy nobody downstream can use, at
        # the price of CPU the offboard heartbeat needs. 0 = no limit.
        self.max_fps = float(self.declare_parameter('max_fps', self.MAX_FPS).value)
        self.min_frame_interval = (1.0 / self.max_fps) if self.max_fps > 0.0 else 0.0
        self.last_processed = 0.0
        self.frames_skipped = 0

        if self.color not in HSV_RANGES:
            raise SystemExit(
                f"Unknown color '{self.color}'; expected one of {sorted(HSV_RANGES)}")

        # Sensor QoS (best effort, depth 1). A best-effort subscription is
        # compatible with a reliable publisher as well, so this works whichever
        # way the camera driver's QoS is configured -- and on a frame we
        # are processing at camera rate, the newest one is the only one worth
        # having anyway.
        self.create_subscription(Image, self.image_topic,
                                 self.image_callback, qos_profile_sensor_data)
        if self.use_depth:
            self.create_subscription(Image, self.depth_topic,
                                     self.depth_callback, qos_profile_sensor_data)
        # Latched-ish in practice: realsense2_camera republishes CameraInfo with every
        # frame, so one message arrives within a frame time of start-up and the
        # fallback FOV is only ever used for the first frame or two.
        if self.publish_geometry_topic:
            self.create_subscription(CameraInfo, self.camera_info_topic,
                                     self.camera_info_callback,
                                     qos_profile_sensor_data)

        self.detected_pub = self.create_publisher(Bool, 'window_detected', 10)
        self.info_pub = self.create_publisher(String, 'window_info', 10)
        # Geometry for the traversal node. Reliable rather than best-effort:
        # it is a small message at camera rate and the consumer runs a median
        # filter over a window of them, so a dropped one costs an outlier
        # rejection it did not need to make.
        self.geometry_pub = (self.create_publisher(
            Float32MultiArray, 'window_geometry', 10)
            if self.publish_geometry_topic else None)
        self.image_pub = (self.create_publisher(Image, 'window_detection/image', 1)
                          if self.publish_image else None)
        self.mask_pub = (self.create_publisher(Image, 'window_detection/mask', 1)
                         if self.publish_mask else None)
        # The '/compressed' suffix is image_transport's convention, so
        # rqt_image_view finds this by picking the 'compressed' transport on
        # the plain /window_detection/image topic.
        self.compressed_pub = (self.create_publisher(
            CompressedImage, 'window_detection/image/compressed', 1)
            if self.publish_compressed else None)

        self._jpeg = None
        self._jpeg_lock = threading.Lock()
        self.stream = None

        # Detection state, carried between frames exactly as the loop in the
        # standalone script carried it between iterations.
        self.smoothed_corners = None
        self.prev_e1 = self.prev_e2 = self.prev_e3 = self.prev_e4 = 0

        self.depth_image = None
        self.depth_time = 0.0

        # fx, fy, cx, cy from CameraInfo. None until the first one lands, at
        # which point _intrinsics() stops guessing from the fallback FOV.
        self.intrinsics = None
        self._intrinsics_logged = False

        self.hit_streak = 0
        self.miss_streak = 0
        self.detected = False       # the debounced answer
        self.last_info = ''
        self.frames = 0
        self.first_frame_logged = False
        self._last_log = 0.0

        # Heartbeat, so a dead camera is loud rather than silent. The detection
        # itself is published from the image callback, at camera rate.
        self.create_timer(1.0, self.watchdog)
        self.last_image_time = None

        if self.stream_port:
            try:
                self.stream = MjpegServer(self, self.stream_port)
                self.get_logger().info(
                    f"Browser stream on http://<jetson-ip>:{self.stream_port}/ "
                    f"(single frame at /snapshot.jpg).")
            except OSError as exc:
                # Port in use, usually a second copy of this node. Not fatal:
                # the ROS topics are the primary output.
                self.get_logger().warning(
                    f"Could not start the stream on port {self.stream_port}: {exc}")

        self.get_logger().info(
            f"Window detection up. image={self.image_topic} "
            f"depth={self.depth_topic if self.use_depth else 'disabled'} "
            f"color={self.color}. Publishing /window_detected, /window_info"
            + (", /window_geometry" if self.publish_geometry_topic else "")
            + (", /window_detection/image" if self.publish_image else "") + ".")

    # ------------------------------------------------------------------ subs

    def depth_callback(self, msg):
        try:
            self.depth_image = imgmsg_to_depth(msg)
            self.depth_time = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f"Cannot convert depth frame: {exc}",
                                      throttle_duration_sec=5.0)

    def camera_info_callback(self, msg):
        """Keep the pinhole intrinsics the geometry topic is built on.

        msg.k is the 3x3 row-major camera matrix of the RECTIFIED image, which
        is the image this node is thresholding, so k[0]=fx, k[4]=fy, k[2]=cx,
        k[5]=cy are directly the numbers wanted. A zero fx means the wrapper
        has not calibrated yet -- ignore that message rather than latching a
        division by zero.
        """
        fx, fy, cx, cy = float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5])
        if fx <= 1.0 or fy <= 1.0:
            return
        self.intrinsics = (fx, fy, cx, cy)
        if not self._intrinsics_logged:
            self._intrinsics_logged = True
            self.get_logger().info(
                f"CameraInfo from {self.camera_info_topic}: fx={fx:.1f} fy={fy:.1f} "
                f"cx={cx:.1f} cy={cy:.1f} ({msg.width}x{msg.height}). "
                "/window_geometry angles are now metric.")

    def _intrinsics(self, shape):
        """(fx, fy, cx, cy) for the frame, from CameraInfo or from the FOV.

        The fallback assumes a centred principal point and square pixels and
        is only there so a wrong camera_info_topic degrades into a few percent
        of angular scale error instead of no geometry at all. It is warned
        about once a second so it cannot go unnoticed.
        """
        if self.intrinsics is not None:
            return self.intrinsics
        h, w = shape[:2]
        fx = (w / 2.0) / math.tan(self.fallback_hfov / 2.0)
        self.get_logger().warning(
            f"No CameraInfo on {self.camera_info_topic} yet; guessing the "
            f"intrinsics from fallback_hfov_deg="
            f"{math.degrees(self.fallback_hfov):.0f}.",
            throttle_duration_sec=5.0)
        return fx, fx, w / 2.0, h / 2.0

    def current_depth(self):
        """The newest depth frame, or None if it is missing or stale."""
        if self.depth_image is None:
            return None
        if time.monotonic() - self.depth_time > self.DEPTH_MAX_AGE:
            return None
        return self.depth_image

    def image_callback(self, msg):
        # Decimate before the conversion, not after: imgmsg_to_bgr copies the
        # whole frame, so a skipped frame has to be skipped here to be free.
        now = time.monotonic()
        if self.min_frame_interval and now - self.last_processed < self.min_frame_interval:
            self.frames_skipped += 1
            return
        self.last_processed = now

        try:
            cv_image = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"Cannot convert image frame: {exc}",
                                    throttle_duration_sec=5.0)
            return

        self.last_image_time = time.monotonic()
        self.frames += 1
        if not self.first_frame_logged:
            self.first_frame_logged = True
            h, w = cv_image.shape[:2]
            self.get_logger().info(f"First frame from {self.image_topic}: {w}x{h}.")

        self.process(cv_image, msg.header)

    # ------------------------------------------------------------ detection

    def process(self, cv_image, header):
        """One frame. Identical pipeline to the standalone script."""
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        green_mask = hsv_mask(hsv_img, self.color)
        window_contour = window_detection(green_mask, self.min_area)

        depth_image = self.current_depth() if self.use_depth else None
        info = ''

        truncated = False
        margin_px = float('inf')

        if window_contour is not None:
            window_contour = window_contour.reshape(-1, 2)

            # Truncation is measured on the RAW corners, before PADDING pulls
            # them inwards and before CORNER_ALPHA smooths them. Both of those
            # move a corner off the border it is actually sitting on, and a
            # test that runs after them would report a clipped window as clear
            # of the edge by exactly the amount we just moved it.
            margin_px = border_margin_px(window_contour, cv_image.shape)
            truncated = margin_px < self.border_margin

            s = np.sum(window_contour, axis=1)
            u1, v1 = window_contour[np.argmin(s)]
            u3, v3 = window_contour[np.argmax(s)]

            u1, v1 = max(u1 + self.PADDING, 0), max(v1 + self.PADDING, 0)
            u3, v3 = (min(u3 - self.PADDING, cv_image.shape[1] - 1),
                      min(v3 - self.PADDING, cv_image.shape[0] - 1))

            diff = np.diff(window_contour, axis=1)
            u2, v2 = window_contour[np.argmin(diff)]
            u4, v4 = window_contour[np.argmax(diff)]

            u2, v2 = max(u2 - self.PADDING, 0), max(v2 + self.PADDING, 0)
            u4, v4 = (min(u4 + self.PADDING, cv_image.shape[1] - 1),
                      min(v4 - self.PADDING, cv_image.shape[0] - 1))

            corners = np.array([[u1, v1], [u2, v2], [u3, v3], [u4, v4]], dtype=np.float32)

            if self.smoothed_corners is None:
                self.smoothed_corners = corners
            else:
                self.smoothed_corners = (self.CORNER_ALPHA * corners
                                         + (1 - self.CORNER_ALPHA) * self.smoothed_corners)

            u1, v1 = self.smoothed_corners[0].astype(int)
            u2, v2 = self.smoothed_corners[1].astype(int)
            u3, v3 = self.smoothed_corners[2].astype(int)
            u4, v4 = self.smoothed_corners[3].astype(int)

            center = self.smoothed_corners.mean(axis=0)

            d1 = d2 = d3 = d4 = 0.0
            depths_m = None
            centre_depth_m = 0.0
            if depth_image is not None:
                d1, s1 = sample_corner_depth(depth_image, u1, v1, center)
                d2, s2 = sample_corner_depth(depth_image, u2, v2, center)
                d3, s3 = sample_corner_depth(depth_image, u3, v3, center)
                d4, s4 = sample_corner_depth(depth_image, u4, v4, center)

                for sp in (s1, s2, s3, s4):
                    cv2.circle(cv_image, sp, 4, (0, 0, 255), -1)

                d1, self.prev_e1 = filter_depth(d1, self.prev_e1, self.DEPTH_ALPHA)
                d2, self.prev_e2 = filter_depth(d2, self.prev_e2, self.DEPTH_ALPHA)
                d3, self.prev_e3 = filter_depth(d3, self.prev_e3, self.DEPTH_ALPHA)
                d4, self.prev_e4 = filter_depth(d4, self.prev_e4, self.DEPTH_ALPHA)

                # Keep the metric copy BEFORE the display scaling. /window_info
                # and the overlay may be in centimetres (depth_scale=100);
                # /window_geometry is always metres, because the traversal node
                # is doing metric geometry with it and a unit that depends on a
                # display parameter is a crash waiting to happen.
                depths_m = [d1, d2, d3, d4]
                centre_depth_m = float(get_median_depth(
                    depth_image, int(round(center[0])), int(round(center[1])), box=5))
                d1, d2, d3, d4 = (d * self.depth_scale for d in (d1, d2, d3, d4))

            cv2.drawContours(cv_image, [window_contour], -1, (255, 0, 255), 3)
            cv2.putText(cv_image, "Window Detected", (u1 - 10, v1 - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            for pt, label, d in zip([(u1, v1), (u2, v2), (u3, v3), (u4, v4)],
                                    ['d1', 'd2', 'd3', 'd4'], [d1, d2, d3, d4]):
                cv2.circle(cv_image, pt, 6, (255, 0, 0), -1)
                cv2.putText(cv_image, f"{label}={d:.2f}", (pt[0] + 10, pt[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if truncated:
                cv2.putText(cv_image, f"TRUNCATED ({margin_px:.0f} px to edge)",
                            (12, cv_image.shape[0] - 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            info = self.frame_info(cv_image, center, [d1, d2, d3, d4], depth_image,
                                   truncated, margin_px)
            if depths_m is not None:
                self.publish_geometry(cv_image.shape, center, depths_m, centre_depth_m,
                                      truncated, margin_px)
        else:
            # Nothing this frame: drop the smoothing state so a later detection
            # starts from its own corners instead of blending into wherever the
            # last one happened to be.
            self.smoothed_corners = None

        self.update_detection(window_contour is not None, info)
        self.annotate_banner(cv_image)
        self.publish_frames(cv_image, green_mask, header)

        if self.show_windows:
            cv2.imshow("RealSense Image Processing", cv_image)
            cv2.imshow("Green Mask", green_mask)
            cv2.waitKey(1)

    def publish_geometry(self, shape, center, depths_m, centre_depth_m,
                         truncated=False, margin_px=float('inf')):
        """The window as four camera-frame rays with a range on each.

        Layout: 5 rows of 3, row-major, as a Float32MultiArray --

            row 0..3   the corners, in the order the detector produces them:
                       top-left, top-right, bottom-right, bottom-left, walking
                       round the quad, so consecutive rows are adjacent edges
                       and rows 0/2 and 1/3 are the diagonals.
            row 4      the centre of the quad.
            row 5      (truncated, border_margin_px, 0) -- NOT a point. See
                       border_margin_px(): a non-zero first column means at
                       least one corner is at or near the image edge, so the
                       quad above describes the VISIBLE PART of a window
                       rather than the window. Appended as a sixth row rather
                       than sent on a topic of its own so that it cannot
                       arrive separately from the measurement it disqualifies;
                       a consumer written against the old 5x3 layout reads the
                       first 15 values and is unaffected.
            columns    (depth_m, azimuth_deg, elevation_deg)

        depth_m is the camera's depth, which on both the ZED and the
        RealSense is the distance along the OPTICAL AXIS (the Z of the camera
        frame), not the slant range to the point. This is worth re-checking
        on any new depth camera, because the reconstruction below is only
        exact for the optical-axis convention.
        That is what makes the reconstruction below exact rather than
        approximate:

            x_forward = depth
            y_right   = depth * tan(azimuth)
            z_down    = -depth * tan(elevation)

        with azimuth positive to the right of the optical axis and elevation
        positive above it. Deliberately the same convention as the depth_data
        array in drone_imav_obs_course.window_coordinates(), so the arithmetic
        that was flown in simulation carries over unchanged.

        Angles rather than pixels because they are the part that needs the
        intrinsics, and the intrinsics live here where CameraInfo arrives.
        A consumer then needs no calibration of its own, and a change of
        resolution on the wrapper changes nothing downstream.

        A corner whose depth came back as 0 (no valid stereo pixel anywhere in
        the sample box) is published as 0 rather than dropped: the array has a
        fixed shape, and the consumer rejects non-positive depths anyway. This
        is published per FRAME, not per debounced detection -- the consumer
        gates on /window_detected for that, and wants every raw sample it can
        get for its median filter.
        """
        if self.geometry_pub is None:
            return

        fx, fy, cx, cy = self._intrinsics(shape)
        pts = list(self.smoothed_corners) + [np.asarray(center, dtype=np.float32)]
        depths = list(depths_m) + [centre_depth_m]

        data = []
        for (u, v), d in zip(pts, depths):
            az = math.degrees(math.atan2(float(u) - cx, fx))
            # Positive elevation is UP, i.e. towards SMALLER v. The sign here
            # is the one that makes z_down = -d*tan(el) come out right.
            el = math.degrees(math.atan2(cy - float(v), fy))
            data.extend([float(d), az, el])

        finite_margin = margin_px if math.isfinite(margin_px) else 9999.0
        data.extend([1.0 if truncated else 0.0, float(finite_margin), 0.0])

        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='point', size=6, stride=18),
            MultiArrayDimension(label='depth_az_el', size=3, stride=3),
        ]
        msg.data = data
        self.geometry_pub.publish(msg)

    def frame_info(self, cv_image, center, depths, depth_image,
                   truncated=False, margin_px=float('inf')):
        """Pipe-separated detail for /window_info.

        u|v|offset|area|d1|d2|d3|d4|dc|trunc

        offset is the horizontal position of the window centre as a fraction
        of half the image width: -1 hard left, 0 dead centre, +1 hard right.
        That is the number a centring controller wants, and it is independent
        of the resolution the wrapper happens to be publishing at.
        """
        h, w = cv_image.shape[:2]
        cu, cv_ = float(center[0]), float(center[1])
        offset = (cu - w / 2.0) / (w / 2.0)
        area = float(cv2.contourArea(self.smoothed_corners.astype(np.int32)))

        dc = 0.0
        if depth_image is not None:
            dc = float(get_median_depth(depth_image, int(round(cu)), int(round(cv_)), box=5))
            dc *= self.depth_scale

        return "|".join([
            f"{cu:.1f}", f"{cv_:.1f}", f"{offset:+.3f}", f"{area:.0f}",
            f"{depths[0]:.2f}", f"{depths[1]:.2f}",
            f"{depths[2]:.2f}", f"{depths[3]:.2f}", f"{dc:.2f}",
            ('TRUNC' if truncated else 'full'),
        ])

    def update_detection(self, hit, info):
        """Debounce, publish, and say out loud when the answer changes."""
        if hit:
            self.hit_streak += 1
            self.miss_streak = 0
            self.last_info = info
        else:
            self.miss_streak += 1
            self.hit_streak = 0

        if not self.detected and self.hit_streak >= self.detect_frames:
            self.detected = True
            self.get_logger().warning(
                f"WINDOW DETECTED  ({self.describe()})")
        elif self.detected and self.miss_streak >= self.lost_frames:
            self.detected = False
            self.get_logger().warning("Window LOST.")

        msg = Bool()
        msg.data = self.detected
        self.detected_pub.publish(msg)

        if hit:
            info_msg = String()
            info_msg.data = info
            self.info_pub.publish(info_msg)

        now = time.monotonic()
        if now - self._last_log >= self.LOG_PERIOD:
            self._last_log = now
            if self.detected:
                self.get_logger().info(f"window: YES  {self.describe()}")
            else:
                self.get_logger().info(
                    f"window: no   (streak {self.miss_streak} misses, "
                    f"{self.frames} frames processed, "
                    f"{self.frames_skipped} skipped by max_fps={self.max_fps:.0f})")

    def describe(self):
        """Human-readable version of the last good detection."""
        if not self.last_info:
            return ''
        f = self.last_info.split('|')
        try:
            return (f"centre=({float(f[0]):.0f},{float(f[1]):.0f}) "
                    f"offset={float(f[2]):+.2f} area={float(f[3]):.0f}px "
                    f"dist={float(f[8]):.2f}{self.depth_units}")
        except (IndexError, ValueError):
            return self.last_info

    # ------------------------------------------------------------- output

    def annotate_banner(self, cv_image):
        """Big top-left banner, so the state is readable in rqt_image_view."""
        text = "WINDOW LOCKED" if self.detected else "searching..."
        color = (0, 255, 0) if self.detected else (0, 165, 255)
        cv2.putText(cv_image, text, (12, 34), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, color, 2)
        if self.detected and self.last_info:
            cv2.putText(cv_image, self.describe(), (12, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def latest_jpeg(self):
        """Newest encoded frame, for the HTTP handler threads."""
        with self._jpeg_lock:
            return self._jpeg

    def publish_frames(self, cv_image, mask, header):
        if self.image_pub is not None:
            self.image_pub.publish(array_to_imgmsg(cv_image, 'bgr8', header))
        if self.mask_pub is not None:
            self.mask_pub.publish(array_to_imgmsg(mask, 'mono8', header))

        if self.compressed_pub is None and self.stream is None:
            return

        # One encode feeds both the ROS topic and the browser stream.
        frame = cv_image
        if 0.0 < self.stream_scale < 1.0:
            frame = cv2.resize(frame, None, fx=self.stream_scale,
                               fy=self.stream_scale, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return
        data = buf.tobytes()

        if self.compressed_pub is not None:
            msg = CompressedImage()
            msg.header = header
            msg.format = 'jpeg'
            msg.data = data
            self.compressed_pub.publish(msg)

        if self.stream is not None:
            with self._jpeg_lock:
                self._jpeg = data

    def watchdog(self):
        """Complain if the camera stops, and keep /window_detected fresh.

        Without this a dead camera driver looks exactly like "no window in
        sight" to anything downstream, which is the one confusion that could
        leave the vehicle yawing forever with a blind camera.
        """
        if self.last_image_time is None:
            self.get_logger().warning(
                f"No frames on {self.image_topic} yet. Is realsense2_camera "
                "running? Check `ros2 topic list | grep camera`. If the image "
                "topic is there but the depth one is not, the driver was "
                "started without align_depth.enable:=true.",
                throttle_duration_sec=5.0)
            return
        age = time.monotonic() - self.last_image_time
        if age > 2.0:
            self.get_logger().error(
                f"No camera frame for {age:.1f} s -- detection is stale.",
                throttle_duration_sec=5.0)
            if self.detected:
                self.detected = False
                self.hit_streak = 0
                self.get_logger().warning("Window detection dropped: camera went away.")
            msg = Bool()
            msg.data = False
            self.detected_pub.publish(msg)

    def destroy_node(self):
        if self.stream is not None:
            self.stream.shutdown()
        if self.show_windows:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WindowDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
