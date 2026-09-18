"""
Find a horizontal coloured BAR and publish where it is, in angles and depth.

This is bar_cross.py's eyes, and it is the same shape of node as
window_detect: HSV mask -> contour -> shape test -> depth samples ->
/bar_geometry, with a debounced /bar_detected for the flight node to gate on.
Everything about topics, intrinsics, the MJPEG stream and the frame-rate cap
is inherited behaviour from that node and is deliberately identical, so there
is one place to learn how this pipeline is wired.

WHY A BAR IS HARDER THAN A WINDOW, ON THIS ARENA

    The arena floor is RED. Not a red object on the floor -- the floor
    itself, in three strips with textured matting between them. The red bar
    this node is looking for is the same colour as the ground the aircraft is
    flying over, and it is the FIRST obstacle, so the aircraft is low and
    looking slightly down at a large expanse of exactly the colour it wants.

    Colour alone therefore decides nothing here. A red mask on this arena
    returns the floor, every frame, with an area far larger than the bar's.
    Worse, a floor strip running left-to-right across the frame is, in the
    image, a long horizontal red region -- so the obvious shape test does not
    separate them either.

    What actually separates them is HEIGHT, and height is not knowable from
    one image. So the work is split:

      * this node rejects what it can reject cheaply and locally -- things
        that are not bar-shaped, and things below the horizon -- and reports
        the geometry of what survives;
      * bar_cross.py does the test that settles it, by putting the endpoints
        in NED using the vehicle attitude and refusing anything whose height
        above the arming plane is less than min_bar_height. The floor is at
        zero by definition. It cannot pass that test no matter how bar-shaped
        a strip of it looks.

    Layered that way, a false positive on the floor costs a rejected sample
    in the estimator rather than an aircraft flown into the ground.

THE SHAPE TEST
    A minAreaRect on the contour, then two conditions: the long side must be
    min_aspect times the short one, and the long axis must be within
    max_tilt_deg of horizontal IN THE IMAGE. The bar is a long thin
    horizontal thing seen from roughly level; a floor strip seen in
    perspective usually is not (it is wide, wedge-shaped and often tilted),
    and the ones that are get caught by the height test downstream.

THE DEPTH SAMPLES
    Depth is sampled at several points spread ALONG the bar rather than once
    at the middle, for the same reason window_detect samples four corners: one
    sample is one chance to hit a hole in the stereo map. The endpoints and
    the centre are published; the flight node fits the line.

    Points are sampled on the bar's CENTRELINE, which for a round bar is the
    nearest part of it, and the median of a small box around each. A bar is
    thin, so a box that is too big straddles the edge and picks up whatever is
    behind -- box stays small and the ladder of samples does the averaging.

TOPICS
    /bar_detected       Bool, debounced by detect_frames / lost_frames
    /bar_info           String, one line of human-readable state
    /bar_geometry       Float32MultiArray, 4 rows of 3:
                            row 0   left endpoint   (depth_m, az_deg, el_deg)
                            row 1   right endpoint  (depth_m, az_deg, el_deg)
                            row 2   centre          (depth_m, az_deg, el_deg)
                            row 3   (truncated, margin_px, aspect_ratio)
                        Angles are camera-frame: azimuth positive to the
                        RIGHT of the optical axis, elevation positive UP,
                        both from the intrinsics. Depth is always METRES.
    /bar_detection/image[/compressed]   the debug overlay

    q/k mean nothing here; this node does not fly anything.
"""

import math
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, String

from drone_testing.window_detect import (HSV_RANGES, MjpegServer, array_to_imgmsg,
                                         get_median_depth, hsv_mask,
                                         imgmsg_to_bgr, imgmsg_to_depth)


def rect_axes(rect):
    """(centre, long_half_vector, long_len, short_len) for a cv2 minAreaRect.

    cv2 reports width/height/angle in a convention that swaps which side is
    "width" as the rectangle rotates through 90 degrees, which makes reading
    the aspect ratio straight off it a bug waiting to happen. Resolving it to
    an explicit long axis once, here, means nothing downstream has to know the
    convention.
    """
    (cx, cy), (w, h), angle = rect
    if w >= h:
        long_len, short_len = w, h
        theta = math.radians(angle)
    else:
        long_len, short_len = h, w
        theta = math.radians(angle + 90.0)
    half = np.array([math.cos(theta), math.sin(theta)]) * (long_len / 2.0)
    return np.array([cx, cy]), half, float(long_len), float(short_len)


def bar_detection(mask, min_area, min_aspect, max_tilt_deg):
    """The largest bar-shaped contour in the mask, or None.

    "Largest" is by contour area among the ones that PASS the shape test, not
    the largest overall -- on this arena the largest red contour is the floor
    and taking it first and testing it second would find the floor and stop.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        rect = cv2.minAreaRect(contour)
        centre, half, long_len, short_len = rect_axes(rect)
        if short_len < 1.0:
            continue
        aspect = long_len / short_len
        if aspect < min_aspect:
            continue
        # Angle of the long axis off horizontal in the image, folded into
        # [0, 90] so a bar tilted 175 degrees reads as 5, not 175.
        tilt = abs(math.degrees(math.atan2(half[1], half[0])))
        if tilt > 90.0:
            tilt = 180.0 - tilt
        if tilt > max_tilt_deg:
            continue
        if area > best_area:
            best_area = area
            best = (contour, centre, half, long_len, short_len, aspect, tilt)
    return best


class BarDetect(Node):

    IMAGE_TOPIC = '/zed/zed_node/rgb/image_rect_color'
    DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'
    CAMERA_INFO_TOPIC = 'auto'

    FALLBACK_HFOV_DEG = 90.0
    MAX_FPS = 10.0

    DETECT_FRAMES = 3
    LOST_FRAMES = 5

    MIN_AREA = 800              # px^2. Smaller than window_detect's: a bar
                                # 5 m away is a thin sliver, and the shape
                                # test is what keeps the noise out, not area.
    MIN_ASPECT = 4.0            # long side / short side of the minAreaRect
    MAX_TILT_DEG = 25.0         # of the bar's long axis off horizontal, in
                                # the image. Generous, because the aircraft
                                # rolls a few degrees while translating and
                                # the bar tilts with it.
    MIN_ELEVATION_DEG = -20.0   # cheap prefilter: a bar the aircraft is going
                                # to fly OVER is at or above the optical axis
                                # once the aircraft is near its own hover
                                # height. Anything well below is floor. Kept
                                # permissive -- the real height test is in
                                # bar_cross, which knows the attitude.
    SAMPLES_ALONG = 7           # depth samples spread along the centreline
    SAMPLE_BOX = 2              # px half-width of each depth median box
    BORDER_MARGIN = 12          # px. Both ends closer than this to the frame
                                # edge and the bar is TRUNCATED -- its length
                                # is not measurable, though its height still
                                # is, which is why this is reported and not
                                # used to refuse the sample.
    DEPTH_MAX_AGE = 0.5
    DEPTH_MIN = 0.30
    DEPTH_MAX = 10.0
    LOG_PERIOD = 1.0
    JPEG_QUALITY = 60
    STREAM_PORT = 8081          # NOT 8080: window_detect owns that one, and
                                # both may be up at once during a full course.

    def __init__(self):
        super().__init__('bar_detect')

        self.image_topic = str(self.declare_parameter('image_topic', self.IMAGE_TOPIC).value)
        self.depth_topic = str(self.declare_parameter('depth_topic', self.DEPTH_TOPIC).value)
        self.camera_info_topic = str(self.declare_parameter(
            'camera_info_topic', self.CAMERA_INFO_TOPIC).value).strip()
        if self.camera_info_topic in ('', 'auto'):
            self.camera_info_topic = (
                self.image_topic.rsplit('/', 1)[0] + '/camera_info')
            self.get_logger().info(
                f"camera_info_topic derived from image_topic: "
                f"{self.camera_info_topic}")

        self.color = str(self.declare_parameter('color', 'red').value).strip().lower()
        if self.color not in HSV_RANGES:
            raise SystemExit(
                f"Unknown color '{self.color}'; expected one of {sorted(HSV_RANGES)}")

        self.min_area = float(self.declare_parameter('min_area', float(self.MIN_AREA)).value)
        self.min_aspect = float(self.declare_parameter('min_aspect', self.MIN_ASPECT).value)
        self.max_tilt_deg = float(self.declare_parameter('max_tilt_deg', self.MAX_TILT_DEG).value)
        self.min_elevation = math.radians(float(self.declare_parameter(
            'min_elevation_deg', self.MIN_ELEVATION_DEG).value))
        self.samples_along = int(self.declare_parameter('samples_along', self.SAMPLES_ALONG).value)
        self.border_margin = float(self.declare_parameter(
            'border_margin', float(self.BORDER_MARGIN)).value)
        self.detect_frames = int(self.declare_parameter('detect_frames', self.DETECT_FRAMES).value)
        self.lost_frames = int(self.declare_parameter('lost_frames', self.LOST_FRAMES).value)
        self.depth_min = float(self.declare_parameter('depth_min', self.DEPTH_MIN).value)
        self.depth_max = float(self.declare_parameter('depth_max', self.DEPTH_MAX).value)
        self.fallback_hfov = math.radians(float(self.declare_parameter(
            'fallback_hfov_deg', self.FALLBACK_HFOV_DEG).value))
        self.max_fps = float(self.declare_parameter('max_fps', self.MAX_FPS).value)
        self.min_frame_interval = (1.0 / self.max_fps) if self.max_fps > 0.0 else 0.0

        self.publish_image = bool(self.declare_parameter('publish_image', True).value)
        self.publish_compressed = bool(self.declare_parameter('publish_compressed', True).value)
        self.publish_mask = bool(self.declare_parameter('publish_mask', False).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', self.JPEG_QUALITY).value)
        self.stream_port = int(self.declare_parameter('stream_port', self.STREAM_PORT).value)
        self.stream_scale = float(self.declare_parameter('stream_scale', 0.5).value)

        self.create_subscription(Image, self.image_topic,
                                 self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic,
                                 self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 self.camera_info_callback, qos_profile_sensor_data)

        self.detected_pub = self.create_publisher(Bool, 'bar_detected', 10)
        self.info_pub = self.create_publisher(String, 'bar_info', 10)
        self.geometry_pub = self.create_publisher(Float32MultiArray, 'bar_geometry', 10)
        self.image_pub = (self.create_publisher(Image, 'bar_detection/image', 1)
                          if self.publish_image else None)
        self.mask_pub = (self.create_publisher(Image, 'bar_detection/mask', 1)
                         if self.publish_mask else None)
        self.compressed_pub = (self.create_publisher(
            CompressedImage, 'bar_detection/image/compressed', 1)
            if self.publish_compressed else None)

        self._jpeg = None
        self._jpeg_lock = threading.Lock()
        self.stream = MjpegServer(self, self.stream_port) if self.stream_port else None

        self.depth_image = None
        self.depth_time = 0.0
        self.intrinsics = None
        self.last_processed = 0.0
        self.frames = 0
        self.frames_skipped = 0
        self.first_frame_logged = False

        self.hit_streak = 0
        self.miss_streak = 0
        self.detected = False
        self.last_log = 0.0
        self.last_reject = ''

        self.get_logger().warning(
            f"Bar detection up. colour={self.color} image={self.image_topic} "
            f"depth={self.depth_topic} -> /bar_detected, /bar_geometry"
            + (f", MJPEG on :{self.stream_port}" if self.stream else ""))
        self.get_logger().warning(
            f"Shape gate: aspect >= {self.min_aspect:.1f}, long axis within "
            f"{self.max_tilt_deg:.0f} deg of horizontal, area >= "
            f"{self.min_area:.0f} px, elevation >= "
            f"{math.degrees(self.min_elevation):.0f} deg. THE FLOOR IS THE "
            "SAME COLOUR AS THE BAR -- the test that actually separates them "
            "is min_bar_height in bar_cross, which needs the vehicle "
            "attitude. Expect floor contours to be reported here and refused "
            "there.")

    # ------------------------------------------------------------------ subs

    def camera_info_callback(self, msg):
        k = msg.k
        if k[0] > 0.0 and k[4] > 0.0 and self.intrinsics is None:
            self.intrinsics = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))
            self.get_logger().info(
                f"CameraInfo from {self.camera_info_topic}: fx={k[0]:.1f} "
                f"fy={k[4]:.1f} cx={k[2]:.1f} cy={k[5]:.1f}. Bar geometry is "
                "now metric.")

    def depth_callback(self, msg):
        try:
            self.depth_image = imgmsg_to_depth(msg)
            self.depth_time = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"Cannot convert depth frame: {exc}",
                                    throttle_duration_sec=5.0)

    def current_depth(self):
        if self.depth_image is None:
            return None
        if time.monotonic() - self.depth_time > self.DEPTH_MAX_AGE:
            return None
        return self.depth_image

    def _intrinsics(self, shape):
        if self.intrinsics is not None:
            return self.intrinsics
        h, w = shape[:2]
        fx = (w / 2.0) / math.tan(self.fallback_hfov / 2.0)
        self.get_logger().warning(
            f"No CameraInfo on {self.camera_info_topic} yet; guessing the "
            f"intrinsics from fallback_hfov_deg="
            f"{math.degrees(self.fallback_hfov):.0f}. Every angle, and so "
            "every bar height, is scaled wrong until this arrives.",
            throttle_duration_sec=5.0)
        return fx, fx, w / 2.0, h / 2.0

    def image_callback(self, msg):
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
        self.frames += 1
        if not self.first_frame_logged:
            self.first_frame_logged = True
            h, w = cv_image.shape[:2]
            self.get_logger().info(f"First frame from {self.image_topic}: {w}x{h}.")
        self.process(cv_image, msg.header)

    # ------------------------------------------------------------ detection

    def process(self, cv_image, header):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = hsv_mask(hsv, self.color)
        found = bar_detection(mask, self.min_area, self.min_aspect, self.max_tilt_deg)

        info = ''
        if found is None:
            self.last_reject = 'no bar-shaped contour'
            self._miss()
        else:
            contour, centre, half, long_len, short_len, aspect, tilt = found
            info = self._measure(cv_image, centre, half, long_len, short_len,
                                 aspect, tilt, contour)

        self._publish_state(info)
        self._publish_images(cv_image, mask, header)
        self._log()

    def _measure(self, cv_image, centre, half, long_len, short_len, aspect, tilt,
                 contour):
        """Turn a bar-shaped contour into depths and angles, or reject it."""
        depth_img = self.current_depth()
        if depth_img is None:
            self.last_reject = 'no depth frame'
            self._miss()
            return ''

        fx, fy, cx, cy = self._intrinsics(cv_image.shape)
        h, w = cv_image.shape[:2]

        # The endpoints, pulled in slightly so a sample at the very tip does
        # not straddle the end of the bar.
        p_left = centre - half * 0.92
        p_right = centre + half * 0.92
        if p_left[0] > p_right[0]:
            p_left, p_right = p_right, p_left

        # Depth along the centreline. The median of what comes back is the
        # bar's depth; points that return nothing are simply absent, which is
        # why several are taken.
        samples = []
        n = max(3, self.samples_along)
        for i in range(n):
            t = i / (n - 1.0)
            p = p_left + (p_right - p_left) * t
            u = int(round(min(max(p[0], 0), w - 1)))
            v = int(round(min(max(p[1], 0), h - 1)))
            d = get_median_depth(depth_img, u, v, self.SAMPLE_BOX)
            if self.depth_min <= d <= self.depth_max:
                samples.append((t, d, (u, v)))

        if len(samples) < 3:
            self.last_reject = f'only {len(samples)} usable depth samples on the bar'
            self._miss()
            return ''

        depths = np.array([s[1] for s in samples])
        median_depth = float(np.median(depths))

        # A bar is a straight, roughly fronto-parallel object: its depth
        # varies smoothly along its length and not by much. A contour whose
        # samples scatter is not one object -- it is the mask having bridged
        # the bar and something behind it, which on this arena is usually the
        # floor beyond.
        spread = float(np.max(np.abs(depths - median_depth)))
        allowed = max(0.40, 0.25 * median_depth)
        if spread > allowed:
            self.last_reject = (f'depth along the bar spreads {spread:.2f} m '
                                f'(allowed {allowed:.2f})')
            self._miss()
            return ''

        def angles(p):
            az = math.atan2(float(p[0]) - cx, fx)
            el = math.atan2(cy - float(p[1]), fy)
            return az, el

        az_c, el_c = angles(centre)
        if el_c < self.min_elevation:
            self.last_reject = (f'centre elevation {math.degrees(el_c):+.0f} deg '
                                f'is below min_elevation_deg '
                                f'{math.degrees(self.min_elevation):+.0f} -- '
                                'almost certainly the floor')
            self._miss()
            return ''

        # Depth at each published point, interpolated from the samples that
        # actually returned something rather than re-sampled at a pixel that
        # may be one of the holes.
        ts = np.array([s[0] for s in samples])
        d_left = float(np.interp(0.0, ts, depths))
        d_right = float(np.interp(1.0, ts, depths))
        az_l, el_l = angles(p_left)
        az_r, el_r = angles(p_right)

        margin = min(p_left[0], w - 1 - p_right[0],
                     min(p_left[1], p_right[1]),
                     h - 1 - max(p_left[1], p_right[1]))
        truncated = margin < self.border_margin

        self._hit()
        self._publish_geometry(
            [(d_left, az_l, el_l), (d_right, az_r, el_r), (median_depth, az_c, el_c)],
            truncated, margin, aspect)

        self._draw(cv_image, contour, p_left, p_right, samples, median_depth,
                   aspect, tilt, truncated)

        self.last_reject = ''
        return (f"d={median_depth:.2f}m az={math.degrees(az_c):+.0f} "
                f"el={math.degrees(el_c):+.0f} aspect={aspect:.1f} "
                f"tilt={tilt:.0f}deg"
                + (" TRUNCATED" if truncated else ""))

    def _publish_geometry(self, points, truncated, margin, aspect):
        data = []
        for d, az, el in points:
            data.extend([float(d), math.degrees(az), math.degrees(el)])
        data.extend([1.0 if truncated else 0.0, float(margin), float(aspect)])
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='point', size=4, stride=12),
            MultiArrayDimension(label='depth_az_el', size=3, stride=3),
        ]
        msg.data = data
        self.geometry_pub.publish(msg)

    # ------------------------------------------------------------- debounce

    def _hit(self):
        self.hit_streak += 1
        self.miss_streak = 0
        if not self.detected and self.hit_streak >= self.detect_frames:
            self.detected = True
            self.get_logger().warning("BAR DETECTED")

    def _miss(self):
        self.miss_streak += 1
        self.hit_streak = 0
        if self.detected and self.miss_streak >= self.lost_frames:
            self.detected = False
            self.get_logger().warning("Bar LOST.")

    def _publish_state(self, info):
        msg = Bool()
        msg.data = self.detected
        self.detected_pub.publish(msg)
        text = String()
        text.data = info or self.last_reject
        self.info_pub.publish(text)

    def _log(self):
        now = time.monotonic()
        if now - self.last_log < self.LOG_PERIOD:
            return
        self.last_log = now
        state = 'YES' if self.detected else 'no '
        detail = self.last_reject or 'bar in sight'
        self.get_logger().info(
            f"bar: {state} ({detail}; {self.frames} frames processed, "
            f"{self.frames_skipped} skipped by max_fps={self.max_fps:.0f})")

    # --------------------------------------------------------------- output

    def _draw(self, img, contour, p_left, p_right, samples, depth, aspect, tilt,
              truncated):
        cv2.drawContours(img, [contour], -1, (255, 0, 255), 2)
        cv2.line(img, tuple(np.round(p_left).astype(int)),
                 tuple(np.round(p_right).astype(int)), (0, 255, 0), 2)
        for _, _, pt in samples:
            cv2.circle(img, pt, 3, (0, 0, 255), -1)
        label = f"BAR d={depth:.2f}m ar={aspect:.1f} tilt={tilt:.0f}"
        cv2.putText(img, label, (int(p_left[0]), max(20, int(p_left[1]) - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if truncated:
            cv2.putText(img, "TRUNCATED", (12, img.shape[0] - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    def _publish_images(self, cv_image, mask, header):
        if self.image_pub is not None:
            self.image_pub.publish(array_to_imgmsg(cv_image, 'bgr8', header))
        if self.mask_pub is not None:
            self.mask_pub.publish(array_to_imgmsg(mask, 'mono8', header))
        if self.compressed_pub is None and self.stream is None:
            return
        frame = cv_image
        if self.stream_scale and self.stream_scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.stream_scale,
                               fy=self.stream_scale, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return
        payload = buf.tobytes()
        if self.compressed_pub is not None:
            out = CompressedImage()
            out.header = header
            out.format = 'jpeg'
            out.data = payload
            self.compressed_pub.publish(out)
        with self._jpeg_lock:
            self._jpeg = payload

    def latest_jpeg(self):
        with self._jpeg_lock:
            return self._jpeg

    def destroy_node(self):
        if self.stream is not None:
            self.stream.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BarDetect()
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
