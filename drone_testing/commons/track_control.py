#!/usr/bin/env python3
"""Pure track-following control law shared by `track_traversal_node` and
`src/track_detection_standalone.py` (offline video replay).

No ROS or PX4 imports here — only maths on a `TrackDetector` detection dict
— so the standalone script prints exactly the commands the node would send
for the same frame. Change the law here and both follow.

Conventions (same as every track node in this workspace): image top = body
+x (nose), image right = body +y (right). Outputs are body-frame
velocities (m/s) and a yaw rate (rad/s, positive = clockwise seen from
above, i.e. PX4 NED yaw).
"""

import math


def clamp(value, limit):
    return max(-limit, min(limit, value))


def track_correction(det, kp_roll, max_roll_vel, kp_yaw, max_yaw_rate):
    """(vy_body, yawspeed) — identical law to track_centering_node: roll
    nulls the centreline's lateral offset, yaw nulls its tilt."""
    vy_body = clamp(kp_roll * det['offset_norm'], max_roll_vel)
    yawspeed = clamp(-kp_yaw * math.radians(det['angle_deg']), max_yaw_rate)
    return vy_body, yawspeed


def alignment_factor(det, offset_cutoff_norm, angle_cutoff_deg):
    """1.0 when perfectly on the track, falling linearly to 0.0 as the
    offset or the tilt reaches its cutoff — the forward-speed scale."""
    f_off = 1.0 - abs(det['offset_norm']) / offset_cutoff_norm
    f_ang = 1.0 - abs(det['angle_deg']) / angle_cutoff_deg
    return max(0.0, min(1.0, f_off)) * max(0.0, min(1.0, f_ang))


def slew(current, target, max_accel, max_decel, dt):
    """Rate-limit `current` toward `target`: at most max_accel*dt up,
    max_decel*dt down."""
    delta = target - current
    if delta > 0:
        return current + min(delta, max_accel * dt)
    return current + max(delta, -max_decel * dt)


def limit_command(vx, vy, yawspeed, max_forward_speed, max_roll_vel,
                  max_horizontal_vel, max_yaw_rate):
    """Apply every speed limit. Lateral correction has priority: forward
    speed is trimmed so |(vx, vy)| never exceeds max_horizontal_vel."""
    vy = clamp(vy, min(max_roll_vel, max_horizontal_vel))
    vx_room = math.sqrt(max(0.0, max_horizontal_vel ** 2 - vy ** 2))
    vx = clamp(vx, min(max_forward_speed, vx_room))
    return vx, vy, clamp(yawspeed, max_yaw_rate)


def track_end_below(det, end_track_top_frac, frame_h):
    """True once the top of the fitted track centreline (image top = ahead)
    has dropped to `end_track_top_frac` of the frame height, i.e. there is
    no more track ahead of the drone."""
    if end_track_top_frac <= 0.0 or frame_h <= 0:
        return False
    (_, y_a), (_, y_b) = det['center_line']
    return min(y_a, y_b) >= end_track_top_frac * frame_h


def forward_gate(det, moving, vertical_tol_deg, hysteresis_deg):
    """Whether to fly forward this tick: only while a track is detected and
    its centreline is vertical in the image (|angle| within
    `vertical_tol_deg`). Once moving, it keeps going until the tilt exceeds
    `vertical_tol_deg + hysteresis_deg`, so a reading hovering right at the
    threshold doesn't toggle forward motion on and off."""
    if det is None:
        return False
    limit = vertical_tol_deg + (hysteresis_deg if moving else 0.0)
    return abs(det['angle_deg']) <= limit


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def marker_servo(marker, frame_w, frame_h, target_yaw_deg, kp_center, max_center_vel,
                 kp_yaw, max_yaw_rate):
    """Centre + yaw-align over an ArUco marker — the same law as
    landing_guidance_node's CENTER_AND_ALIGN.

    `marker` is a landing_guidance.aruco_utils.MarkerDetection (pixel centre,
    and `angle_deg` = in-image angle of the marker's top edge, CCW positive,
    0 = marker upright in the image). `target_yaw_deg` is the marker angle to
    hold: 0 = drone squared up to the marker, 180 = facing the opposite way.

    Returns (vx, vy, yawspeed, ex_px, ey_px, yaw_err_deg): body-frame
    velocities, yaw rate, the marker's pixel offset from the frame centre,
    and the remaining yaw error.
    """
    cx, cy = frame_w / 2.0, frame_h / 2.0
    ex_px, ey_px = marker.center_x - cx, marker.center_y - cy
    vx = clamp(-kp_center * ey_px / cy, max_center_vel)   # marker above centre -> forward
    vy = clamp(kp_center * ex_px / cx, max_center_vel)    # marker right of centre -> right
    yaw_err_deg = wrap_deg(marker.angle_deg - target_yaw_deg)
    yawspeed = clamp(-kp_yaw * math.radians(yaw_err_deg), max_yaw_rate)
    return vx, vy, yawspeed, ex_px, ey_px, yaw_err_deg


def marker_within_tolerance(ex_px, ey_px, yaw_err_deg, center_tol_px, yaw_tol_deg):
    return (abs(ex_px) <= center_tol_px and abs(ey_px) <= center_tol_px
            and abs(yaw_err_deg) <= yaw_tol_deg)
