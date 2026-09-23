#!/usr/bin/env python3
"""The CARPET TRACK under the aircraft, from the down camera.

The detector is track_traversal_node's own -- drone_testing/commons/
track_detector.py, copied verbatim from the commons package -- so the
mission FSM follows the track with exactly what that node was tuned with.
Everything about FINDING the track lives there; this file is the ROS
wrapper: subscribe, detect, publish, serve the debug view.

    /floor_line   geometry_msgs/PointStamped
                      x = offset_norm   the centreline's lateral offset as a
                          fraction of the half-frame, + = track to the RIGHT
                      y = angle_deg     its tilt from vertical in the image,
                          + = the track leans left going away
                      z = track_width_norm
                  frame_id 'track' when the detection is confirmed, 'none'
                  otherwise (published either way, so a subscriber can tell
                  "no track" from "node dead").

    /floor_line_enable  std_msgs/Bool
                  The image subscription only EXISTS while this is true.
                  The flight node turns it on for the leg between the two
                  markers and off once the second one is reached: an 800x600
                  frame at 20 Hz is real CPU and there is nothing to follow
                  for the rest of the mission.

The browser view (stream_port, 8082) is the detector's own draw(): the
segmentation mask, both fitted boundaries, the centreline, and the
offset/angle readout.
"""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from drone_testing.commons.track_detector import (TRACK_DETECTOR_PARAMS, draw,
                                                  make_track_detector)
from drone_testing.window_detect import imgmsg_to_bgr


class FloorLine(Node):
    def __init__(self):
        super().__init__('floor_line')
        p = self.declare_parameter
        self.image_topic = str(p('image_topic', '/image_raw').value)
        self.max_fps = float(p('max_fps', 10.0).value)
        self.enable_topic = str(p('enable_topic', '/floor_line_enable').value)
        self.require_enable = bool(p('require_enable', True).value)
        self.stream_port = int(p('stream_port', 8082).value)
        self.jpeg_quality = int(p('jpeg_quality', 70).value)

        # Every TRACK_DETECTOR_PARAMS key is a ROS parameter, so the detector
        # is tuned from the launch file without touching its code.
        params = {}
        for key, default in TRACK_DETECTOR_PARAMS.items():
            params[key] = p(key, default).value
        self.detector = make_track_detector(params)
        self.mode = str(params['detector_mode'])

        self.last = 0.0
        self._jpeg = None
        self.stream = None
        self.pub = self.create_publisher(PointStamped, 'floor_line', 10)
        self._qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        self.image_sub = None
        self.enabled = not self.require_enable
        if self.enabled:
            self._set_enabled(True)
        self.create_subscription(Bool, self.enable_topic,
                                 self._enable_callback, 10)
        if self.stream_port:
            from drone_testing.aruco_pose import MjpegServer
            try:
                self.stream = MjpegServer(self, self.stream_port)
                self.get_logger().warning(
                    f"Track view on http://<ip>:{self.stream_port}/")
            except Exception as exc:
                self.get_logger().error(
                    f"no stream on {self.stream_port}: {exc}")
        self.get_logger().warning(
            f"floor_line: {self.image_topic}, {self.mode} detector "
            "(track_traversal_node's own), "
            + ("waiting to be enabled." if not self.enabled else "running."))

    # ------------------------------------------------------------ plumbing

    def _set_enabled(self, on):
        if on and self.image_sub is None:
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self._image, self._qos)
            self.get_logger().warning(f"floor_line ON ({self.image_topic}).")
        elif not on and self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.get_logger().warning("floor_line OFF (marker reached).")
        self.enabled = on

    def _enable_callback(self, msg):
        if bool(msg.data) != self.enabled:
            self._set_enabled(bool(msg.data))

    def _image(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.max_fps > 0.0 and now - self.last < 1.0 / self.max_fps:
            return
        self.last = now
        try:
            frame = imgmsg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(f"frame: {exc}", throttle_duration_sec=5.0)
            return

        det = self.detector(frame)
        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'none'
        if det is not None and det.get('ok'):
            out.point.x = float(det['offset_norm'])
            out.point.y = float(det['angle_deg'])
            out.point.z = float(det.get('track_width_norm', 0.0))
            out.header.frame_id = 'track'
        self.pub.publish(out)

        if self.stream is not None:
            import cv2
            view = draw(frame, det, self.detector)
            ok, buf = cv2.imencode('.jpg', view,
                                   [int(cv2.IMWRITE_JPEG_QUALITY),
                                    self.jpeg_quality])
            if ok:
                self._jpeg = buf.tobytes()

        self.get_logger().info(
            f"no track ({'-' if det is None else det.get('reason', '?')})"
            if out.header.frame_id == 'none' else
            f"track {det['offset_norm']:+.3f} off centre, "
            f"{det['angle_deg']:+.1f} deg, "
            f"{det['track_width_norm'] * 100:.0f}% of frame wide",
            throttle_duration_sec=1.0)

    def latest_jpeg(self):
        return self._jpeg

    def destroy_node(self):
        if self.stream is not None:
            self.stream.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FloorLine()
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
