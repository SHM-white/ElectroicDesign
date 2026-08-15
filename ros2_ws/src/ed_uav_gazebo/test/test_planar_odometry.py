import math

import pytest

from ed_uav_gazebo.planar_odometry import (
    bounded_planar_covariance,
    continuous_altitude,
    yaw_only_quaternion,
)


def test_yaw_only_quaternion_removes_roll_and_pitch() -> None:
    # Quaternion generated from roll=.3, pitch=-.2, yaw=1.1.
    roll, pitch, yaw = 0.3, -0.2, 1.1
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )

    result = yaw_only_quaternion(*quaternion)

    assert result[:2] == (0.0, 0.0)
    assert 2.0 * math.atan2(result[2], result[3]) == pytest.approx(yaw)


def test_covariance_assigns_z_owner_without_claiming_roll_pitch() -> None:
    source = [0.0] * 36
    source[0], source[7], source[35] = 0.1, 0.2, 0.3

    result = bounded_planar_covariance(source, altitude_variance=0.0025)

    assert (result[0], result[7], result[35]) == pytest.approx((0.1, 0.2, 0.3))
    assert result[14] == pytest.approx(0.0025)
    assert result[21] == result[28] == 1.0
    assert all(result[2 * 6 + index] == 0.0 for index in range(6) if index != 2)


def test_altitude_is_continuous_but_restart_may_reset_origin() -> None:
    assert continuous_altitude(None, 1.5, 0.0, 3.0) == 1.5
    assert continuous_altitude(1.5, 1.55, 0.1, 3.0) == pytest.approx(1.55)
    assert continuous_altitude(1.5, 4.0, 0.1, 3.0) == pytest.approx(1.8)
