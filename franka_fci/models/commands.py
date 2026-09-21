"""API request and command response models."""

from __future__ import annotations

from datetime import datetime
from enum import Enum
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, field_validator

from .poses import Pose


class CommandStatus(str, Enum):
    accepted = "accepted"
    queued = "queued"
    running = "running"
    succeeded = "succeeded"
    stopped = "stopped"
    failed = "failed"
    rejected = "rejected"


class MotionBase(BaseModel):
    model_config = ConfigDict(extra="forbid")
    duration_s: float = Field(gt=0)
    request_id: str | None = Field(default=None, min_length=1, max_length=128)


class JointPositionRequest(MotionBase):
    target_rad: tuple[float, ...]

    @field_validator("target_rad", mode="before")
    @classmethod
    def validate_target(cls, value: list[float]) -> tuple[float, ...]:
        values = tuple(float(item) for item in value)
        if len(values) != 7:
            raise ValueError("target_rad must contain exactly 7 values for FR3")
        if not all(np.isfinite(values)):
            raise ValueError("target_rad must contain only finite values")
        return values


class CartesianPoseRequest(MotionBase):
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    frame_id: Literal["base"] = "base"
    tool_frame: Literal["gripper"] = "gripper"

    def pose(self) -> Pose:
        return Pose(position_m=self.position_m, quaternion_xyzw=self.quaternion_xyzw)


class GripperMoveRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    width_m: float = Field(ge=0, le=0.08)
    speed_m_s: float = Field(gt=0, le=0.2)
    request_id: str | None = Field(default=None, min_length=1, max_length=128)


class GripperGraspRequest(GripperMoveRequest):
    force_n: float = Field(gt=0, le=100)


class CommandResponse(BaseModel):
    command_id: str
    status: CommandStatus
    accepted_at: datetime


class CommandRecord(CommandResponse):
    request_id: str | None = None
    kind: str
    source: str | None = None
    started_at: datetime | None = None
    finished_at: datetime | None = None
    error_code: str | None = None
    error_message: str | None = None
