"""The pad-heading seed makes wall_localizer read arena yaw +90 on the pad."""
import math

import pytest

from drone_testing.yaw_seed import heading_of, seed_from_heading


def _q_ned(heading):
    return (math.cos(heading / 2), 0.0, 0.0, math.sin(heading / 2))


@pytest.mark.parametrize('h_deg', [0.0, 37.0, -120.0, 179.0])
def test_pad_pose_reads_plus_90_in_the_arena(h_deg):
    h = math.radians(h_deg)
    assert heading_of(_q_ned(h)) == pytest.approx(h)
    seed = seed_from_heading(h)
    enu_yaw = math.pi / 2 - h                     # lidar_loc's yawOf, ENU
    arena = math.atan2(math.sin(enu_yaw - seed), math.cos(enu_yaw - seed))
    assert arena == pytest.approx(math.pi / 2)
