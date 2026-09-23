"""floor_line around track_traversal_node's detector, and the leg it steers."""
import math

import cv2
import numpy as np
import pytest
import rclpy

from drone_testing.floor_line import FloorLine


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = FloorLine()
    yield n
    n.destroy_node()


def _frame(centre_px, width_px, w=640, h=480, lean=0):
    """Dark speckled floor, bright track, as the real cage floor is."""
    rng = np.random.default_rng(0)
    bgr = np.full((h, w, 3), 45, np.uint8)
    bgr = np.clip(bgr.astype(np.int16)
                  + rng.integers(0, 60, (h, w, 1)).astype(np.int16),
                  0, 255).astype(np.uint8)
    for v in range(h):
        c = centre_px + int(lean * (v - h / 2) / h)
        lo, hi = int(c - width_px / 2), int(c + width_px / 2)
        bgr[v, max(0, lo):max(0, hi)] = (235, 238, 240)
    return bgr


def _detect(node, frame, tries=4):
    """The detector confirms over several frames before it reports ok."""
    det = None
    for _ in range(tries):
        det = node.detector(frame)
    return det


def test_detector_is_the_track_traversal_one(node):
    from drone_testing.commons.track_detector import GreyBackgroundTrackDetector
    assert isinstance(node.detector, GreyBackgroundTrackDetector)
    assert node.mode == 'grey_background'


def test_track_centred_reads_about_zero(node):
    det = _detect(node, _frame(320, 180))
    assert det is not None and det['ok']
    assert abs(det['offset_norm']) < 0.08
    assert abs(det['angle_deg']) < 3.0


def test_offset_sign_is_where_the_track_is(node):
    right = _detect(node, _frame(430, 180))
    assert right is not None and right['offset_norm'] > 0.1
    node.detector.hits = 0
    left = _detect(node, _frame(210, 180))
    assert left is not None and left['offset_norm'] < -0.1


def test_bare_floor_is_not_a_track(node):
    rng = np.random.default_rng(1)
    floor = np.clip(np.full((480, 640, 3), 45, np.int16)
                    + rng.integers(0, 60, (480, 640, 1)).astype(np.int16),
                    0, 255).astype(np.uint8)
    for _ in range(4):
        det = node.detector(floor)
    assert det is None or not det.get('ok')


def test_image_subscription_only_exists_while_enabled(node):
    """800x600 at 20 Hz is real CPU; not paid for the whole mission."""
    from std_msgs.msg import Bool
    node.require_enable = True
    node._set_enabled(False)
    assert node.image_sub is None and not node.enabled
    node._enable_callback(Bool(data=True))
    assert node.image_sub is not None and node.enabled
    node._enable_callback(Bool(data=False))
    assert node.image_sub is None
