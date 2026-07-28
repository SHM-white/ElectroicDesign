from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass


Matrix = tuple[tuple[float, ...], ...]
Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]
Matrix6 = tuple[
    tuple[float, float, float, float, float, float],
    tuple[float, float, float, float, float, float],
    tuple[float, float, float, float, float, float],
    tuple[float, float, float, float, float, float],
    tuple[float, float, float, float, float, float],
    tuple[float, float, float, float, float, float],
]


@dataclass(frozen=True, slots=True)
class Vector3:
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True, slots=True)
class RigidTransform:
    translation: Vector3
    rotation: Quaternion

    @classmethod
    def from_xyz_rpy(
        cls,
        *,
        xyz_m: tuple[float, float, float],
        rpy_rad: tuple[float, float, float],
    ) -> RigidTransform:
        roll, pitch, yaw = rpy_rad
        half_roll = roll / 2.0
        half_pitch = pitch / 2.0
        half_yaw = yaw / 2.0
        cos_roll, sin_roll = math.cos(half_roll), math.sin(half_roll)
        cos_pitch, sin_pitch = math.cos(half_pitch), math.sin(half_pitch)
        cos_yaw, sin_yaw = math.cos(half_yaw), math.sin(half_yaw)
        return cls(
            translation=Vector3(*xyz_m),
            rotation=Quaternion(
                x=sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
                y=cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
                z=cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
                w=cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
            ),
        )

    def inverse(self) -> RigidTransform:
        inverse_rotation = Quaternion(
            x=-self.rotation.x,
            y=-self.rotation.y,
            z=-self.rotation.z,
            w=self.rotation.w,
        )
        rotated_translation = rotate_vector(inverse_rotation, self.translation)
        return RigidTransform(
            translation=Vector3(
                x=-rotated_translation.x,
                y=-rotated_translation.y,
                z=-rotated_translation.z,
            ),
            rotation=inverse_rotation,
        )

    def compose(self, child_transform: RigidTransform) -> RigidTransform:
        child_translation = rotate_vector(self.rotation, child_transform.translation)
        return RigidTransform(
            translation=Vector3(
                x=self.translation.x + child_translation.x,
                y=self.translation.y + child_translation.y,
                z=self.translation.z + child_translation.z,
            ),
            rotation=multiply_quaternions(self.rotation, child_transform.rotation),
        )


def multiply_quaternions(left: Quaternion, right: Quaternion) -> Quaternion:
    return Quaternion(
        x=left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        y=left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        z=left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
        w=left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    )


def rotate_vector(rotation: Quaternion, vector: Vector3) -> Vector3:
    rotated = multiply_quaternions(
        multiply_quaternions(rotation, Quaternion(vector.x, vector.y, vector.z, 0.0)),
        Quaternion(-rotation.x, -rotation.y, -rotation.z, rotation.w),
    )
    return Vector3(x=rotated.x, y=rotated.y, z=rotated.z)


def covariance_matrix(values: Sequence[float]) -> Matrix6 | None:
    if len(values) != 36 or not all(math.isfinite(value) for value in values):
        return None
    return tuple(
        tuple(values[row * 6 + column] for column in range(6))
        for row in range(6)
    )


def covariance_values(matrix: Matrix6) -> list[float]:
    return [value for row in matrix for value in row]


def transform_twist(
    base_to_lidar: RigidTransform,
    linear_lidar: Vector3,
    angular_lidar: Vector3,
) -> tuple[Vector3, Vector3]:
    lidar_to_base_rotation = transpose(rotation_matrix(base_to_lidar.rotation))
    angular_base = matrix_vector(lidar_to_base_rotation, angular_lidar)
    linear_base_unoffset = matrix_vector(lidar_to_base_rotation, linear_lidar)
    offset_velocity = cross(angular_base, base_to_lidar.translation)
    return (
        Vector3(
            x=linear_base_unoffset.x - offset_velocity.x,
            y=linear_base_unoffset.y - offset_velocity.y,
            z=linear_base_unoffset.z - offset_velocity.z,
        ),
        angular_base,
    )


def transform_twist_covariance(
    covariance: Matrix6, base_to_lidar: RigidTransform
) -> Matrix6:
    lidar_to_base_rotation = transpose(rotation_matrix(base_to_lidar.rotation))
    offset_rotation = matrix_multiply(
        skew(base_to_lidar.translation), lidar_to_base_rotation
    )
    jacobian = block_matrix(lidar_to_base_rotation, offset_rotation)
    return covariance_product(jacobian, covariance)


def transform_pose_covariance(
    covariance: Matrix6,
    odom_to_base: RigidTransform,
    base_to_lidar: RigidTransform,
) -> Matrix6:
    """Propagate a fixed-world left pose perturbation through lidar-to-base.

    For ``p_wb = p_wl - R_wb t_bl``, the first-order error Jacobian is
    ``[[I, skew(R_wb t_bl)], [0, I]]``.
    """
    world_offset = rotate_vector(odom_to_base.rotation, base_to_lidar.translation)
    jacobian = block_matrix(identity_matrix(), skew(world_offset))
    return covariance_product(jacobian, covariance)


def rotation_matrix(rotation: Quaternion) -> Matrix3:
    return (
        (
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            2.0 * (rotation.x * rotation.y - rotation.z * rotation.w),
            2.0 * (rotation.x * rotation.z + rotation.y * rotation.w),
        ),
        (
            2.0 * (rotation.x * rotation.y + rotation.z * rotation.w),
            1.0 - 2.0 * (rotation.x * rotation.x + rotation.z * rotation.z),
            2.0 * (rotation.y * rotation.z - rotation.x * rotation.w),
        ),
        (
            2.0 * (rotation.x * rotation.z - rotation.y * rotation.w),
            2.0 * (rotation.y * rotation.z + rotation.x * rotation.w),
            1.0 - 2.0 * (rotation.x * rotation.x + rotation.y * rotation.y),
        ),
    )


def matrix_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return Vector3(
        x=matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
        y=matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
        z=matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z,
    )


def cross(left: Vector3, right: Vector3) -> Vector3:
    return Vector3(
        x=left.y * right.z - left.z * right.y,
        y=left.z * right.x - left.x * right.z,
        z=left.x * right.y - left.y * right.x,
    )


def skew(vector: Vector3) -> Matrix3:
    return ((0.0, -vector.z, vector.y), (vector.z, 0.0, -vector.x), (-vector.y, vector.x, 0.0))


def identity_matrix() -> Matrix3:
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def transpose(matrix: Matrix3) -> Matrix3:
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


def matrix_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(3)) for column in range(3))
        for row in range(3)
    )


def block_matrix(top_left: Matrix3, top_right: Matrix3) -> Matrix6:
    return tuple(
        tuple(
            top_left[row][column] if column < 3 else top_right[row][column - 3]
            for column in range(6)
        )
        if row < 3
        else tuple(0.0 if column < 3 else top_left[row - 3][column - 3] for column in range(6))
        for row in range(6)
    )


def covariance_product(jacobian: Matrix6, covariance: Matrix6) -> Matrix6:
    intermediate = tuple(
        tuple(sum(jacobian[row][index] * covariance[index][column] for index in range(6)) for column in range(6))
        for row in range(6)
    )
    return tuple(
        tuple(sum(intermediate[row][index] * jacobian[column][index] for index in range(6)) for column in range(6))
        for row in range(6)
    )
