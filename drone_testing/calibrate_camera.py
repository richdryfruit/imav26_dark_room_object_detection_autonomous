#!/usr/bin/env python3
"""Calibrate the down camera (the C920) from a printed chessboard.

Everything that turns pixels into metres -- the marker offsets aruco_pose
reports, the doll positions doll_detect geotags, the carpet-strip offset
floor_line measures -- currently assumes a pinhole derived from a NOMINAL
field of view (70.4 deg) and no lens distortion. That is a scale error of
several percent near the middle of the frame and much worse at the edges,
where the C920's barrel distortion lives.

    ros2 run drone_testing calibrate_camera --ros-args \\
        -p image_topic:=/image_raw -p squares_x:=9 -p squares_y:=6 \\
        -p square_size:=0.025 -p out:=$HOME/c920.yaml

Hold a printed chessboard (default 9x6 INNER corners, 25 mm squares) in front
of the camera and move it around: near, far, all four corners of the frame,
tilted both ways. It captures a view every capture_gap seconds when the board
is found and enough has changed, prints the running count, and writes the
result once it has `views` of them.

The file it writes is a ROS camera_info YAML. Point usb_cam at it
(camera_info_url: file:///home/.../c920.yaml) and every node downstream gets
real intrinsics through CameraInfo, with no other change: aruco_pose,
doll_detect and floor_line all prefer CameraInfo over their fov parameter.
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image

from drone_testing.window_detect import imgmsg_to_bgr

YAML = """image_width: {w}
image_height: {h}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{k}]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{d}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [{p}]
"""


class Calibrate(Node):
    def __init__(self):
        super().__init__('calibrate_camera')
        p = self.declare_parameter
        self.topic = str(p('image_topic', '/image_raw').value)
        self.nx = int(p('squares_x', 9).value)
        self.ny = int(p('squares_y', 6).value)
        self.size = float(p('square_size', 0.025).value)
        self.views = int(p('views', 20).value)
        self.gap = float(p('capture_gap', 1.0).value)
        self.out = str(p('out', os.path.expanduser('~/down_camera.yaml')).value)
        self.name = str(p('camera_name', 'down_camera').value)

        self.obj = np.zeros((self.nx * self.ny, 3), np.float32)
        self.obj[:, :2] = np.mgrid[0:self.nx, 0:self.ny].T.reshape(-1, 2)
        self.obj *= self.size
        self.objs, self.imgs, self.shape = [], [], None
        self.last = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, self.topic, self._image, qos)
        self.get_logger().warning(
            f"Calibrating from {self.topic}: show the {self.nx}x{self.ny} "
            f"board ({self.size * 1000:.0f} mm squares) from all over the "
            f"frame. Need {self.views} views.")

    def _image(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last < self.gap or len(self.objs) >= self.views:
            return
        gray = cv2.cvtColor(imgmsg_to_bgr(msg), cv2.COLOR_BGR2GRAY)
        self.shape = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(
            gray, (self.nx, self.ny),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            self.get_logger().info("no board in this frame",
                                   throttle_duration_sec=2.0)
            return
        cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        self.objs.append(self.obj)
        self.imgs.append(corners)
        self.last = now
        self.get_logger().warning(f"view {len(self.objs)}/{self.views}")
        if len(self.objs) >= self.views:
            self._solve()

    def _solve(self):
        rms, k, d, _, _ = cv2.calibrateCamera(
            self.objs, self.imgs, self.shape, None, None)
        w, h = self.shape
        proj = [k[0, 0], 0, k[0, 2], 0, 0, k[1, 1], k[1, 2], 0, 0, 0, 1, 0]
        with open(self.out, 'w') as f:
            f.write(YAML.format(
                w=w, h=h, name=self.name,
                k=', '.join(f'{v:.6f}' for v in k.flatten()),
                d=', '.join(f'{v:.6f}' for v in d.flatten()[:5]),
                p=', '.join(f'{v:.6f}' for v in proj)))
        fov = 2.0 * np.degrees(np.arctan(0.5 * w / k[0, 0]))
        self.get_logger().warning(
            f"DONE, reprojection error {rms:.3f} px. fx={k[0, 0]:.1f} "
            f"fy={k[1, 1]:.1f} cx={k[0, 2]:.1f} cy={k[1, 2]:.1f} -> "
            f"horizontal FOV {fov:.1f} deg (the nodes assume 70.4). "
            f"Written to {self.out}: point usb_cam's camera_info_url at it.")
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = Calibrate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
