#!/usr/bin/env python3
"""The lidar arena's heading seed, from the aircraft sitting on the pad.

The mission is armed with the aircraft pointing exactly at the window,
perpendicular to its wall. wall_localizer's arena frame has +X along that
(south) wall and +Y into the room, so on the pad the aircraft's arena yaw is
+90 deg. wall_localizer computes

    arena_yaw = ENU_yaw - seed_yaw_offset,   ENU_yaw = 90 deg - NED heading

so the seed that makes the pad pose read +90 deg is simply

    seed_yaw_offset = -(EKF2 NED heading on the pad)

    ros2 run drone_testing yaw_seed          # prints the seed, radians

lidar_real.launch.py calls read_pad_heading() itself and passes the result in,
so nobody has to measure or type it. The aircraft must already be sitting
aligned when the lidar stack is launched.
"""

import math
import sys
import time

import rclpy
from px4_msgs.msg import VehicleAttitude
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_testing.px4_topics import versioned_names


def heading_of(q):
    """PX4 VehicleAttitude q (w, x, y, z; FRD->NED) -> NED heading, radians."""
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def seed_from_heading(heading):
    return math.atan2(math.sin(-heading), math.cos(-heading))


def read_pad_heading(timeout=20.0, samples=20):
    """Mean NED heading over a few attitude messages, or None on timeout.

    Runs in its own rclpy context so it is safe to call from a launch file.
    """
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node('yaw_seed', context=ctx)
    got = []
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
    subs = [node.create_subscription(VehicleAttitude, t,
                                     lambda m: got.append(heading_of(m.q)), qos)
            for t in versioned_names('vehicle_attitude', 1)]
    executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    end = time.monotonic() + timeout
    try:
        while len(got) < samples and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
    finally:
        del subs
        node.destroy_node()
        rclpy.shutdown(context=ctx)
    if not got:
        return None
    s = sum(math.sin(h) for h in got)
    c = sum(math.cos(h) for h in got)
    return math.atan2(s, c)


def main():
    h = read_pad_heading()
    if h is None:
        print(f"yaw_seed: no {versioned_names('vehicle_attitude')[0]} in 20 s -- is the agent "
              "up and ROS_DOMAIN_ID right?", file=sys.stderr)
        sys.exit(1)
    print(f"{seed_from_heading(h):.4f}")
    print(f"yaw_seed: pad heading {math.degrees(h):+.1f} deg NED -> "
          f"seed_yaw_offset {seed_from_heading(h):+.4f} rad", file=sys.stderr)


if __name__ == '__main__':
    main()
