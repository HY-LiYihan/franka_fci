"""Validated pose models and rigid-transform conversion helpers."""

from __future__ import annotations

from typing import Iterable

import numpy as np
from pydantic import BaseModel, ConfigDict, field_validator


def _finite(values: Iterable[float], name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


class Pose(BaseModel):
    model_config = ConfigDict(extra="forbid")

    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    @field_validator("position_m", mode="before")
    @classmethod
    def validate_position(cls, value: Iterable[float]) -> tuple[float, ...]:
        values = _finite(value, "position_m")
        if len(values) != 3:
            raise ValueError("position_m must have exactly 3 values")
        return values

    @field_validator("quaternion_xyzw", mode="before")
    @classmethod
    def validate_quaternion(cls, value: Iterable[float]) -> tuple[float, ...]:
        values = _finite(value, "quaternion_xyzw")
        if len(values) != 4:
            raise ValueError("quaternion_xyzw must have exactly 4 values")
        if np.linalg.norm(values) < 1e-12:
            raise ValueError("quaternion_xyzw must not be zero")
        normalized = np.asarray(values, dtype=float) / np.linalg.norm(values)
        return tuple(float(item) for item in normalized)


def pose_to_matrix(pose: Pose) -> np.ndarray:
    x, y, z, w = pose.quaternion_xyzw
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ]
    )
    matrix[:3, 3] = pose.position_m
    return matrix


def matrix_to_pose(matrix: np.ndarray) -> Pose:
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("transform must be a finite 4x4 matrix")
    rotation = matrix[:3, :3]
    trace = np.trace(rotation)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return Pose(
        position_m=tuple(float(value) for value in matrix[:3, 3]),
        quaternion_xyzw=(x, y, z, w),
    )


def compose(first: Pose, second: Pose) -> Pose:
    return matrix_to_pose(pose_to_matrix(first) @ pose_to_matrix(second))


def inverse(pose: Pose) -> Pose:
    return matrix_to_pose(np.linalg.inv(pose_to_matrix(pose)))


def gripper_to_flange_target(gripper_target: Pose, flange_to_gripper: Pose) -> Pose:
    return compose(gripper_target, inverse(flange_to_gripper))


def flange_to_gripper_pose(flange_pose: Pose, flange_to_gripper: Pose) -> Pose:
    return compose(flange_pose, flange_to_gripper)
