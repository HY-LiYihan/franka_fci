"""Robot and gripper state models."""

from __future__ import annotations

from datetime import datetime

from pydantic import BaseModel, ConfigDict, Field

from .poses import Pose


class RobotState(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    connected: bool = False
    fci_active: bool = False
    robot_mode: str = "unknown"
    errors: list[str] = Field(default_factory=list)
    timestamp: datetime
    joint_position_rad: tuple[float, ...] | None = None
    joint_velocity_rad_s: tuple[float, ...] | None = None
    flange_pose: Pose | None = None
    gripper_pose: Pose | None = None


class GripperState(BaseModel):
    connected: bool = False
    width_m: float | None = None
    max_width_m: float | None = None
    grasped: bool | None = None
    homed: bool = False
    moving: bool = False
    errors: list[str] = Field(default_factory=list)
    timestamp: datetime
