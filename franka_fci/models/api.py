"""API response models."""

from pydantic import BaseModel

from .state import GripperState, RobotState


class HealthResponse(BaseModel):
    service: str
    version: str
    status: str
    robot: RobotState
    gripper: GripperState | None
    active_command_id: str | None


class StateResponse(BaseModel):
    robot: RobotState
    gripper: GripperState | None
