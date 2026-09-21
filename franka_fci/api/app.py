"""HTTP API for the FR3 control service."""

from __future__ import annotations

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Request

from franka_fci import __version__
from franka_fci.config import Settings, get_settings
from franka_fci.control.manager import CommandConflict, CommandManager, ServiceUnavailable
from franka_fci.models.api import HealthResponse, StateResponse
from franka_fci.models.commands import (
    CartesianPoseRequest,
    CommandResponse,
    GripperGraspRequest,
    GripperMoveRequest,
    JointPositionRequest,
)
from franka_fci.models.state import GripperState
from franka_fci.robot.adapters import (
    FakeGripperAdapter,
    FakeRobotAdapter,
    GripperAdapter,
    PylibfrankaGripperAdapter,
    PylibfrankaRobotAdapter,
    RobotAdapter,
)
from franka_fci.models.poses import Pose, flange_to_gripper_pose

LOGGER = logging.getLogger(__name__)
def create_app(
    settings: Settings | None = None,
    robot: RobotAdapter | None = None,
    gripper: GripperAdapter | None = None,
    use_fake: bool = False,
) -> FastAPI:
    settings = settings or get_settings()

    def gripper_pose_provider(pose):
        if not settings.gripper_flange_translation_m or not settings.gripper_flange_quaternion_xyzw:
            return None
        return flange_to_gripper_pose(
            pose,
            Pose(
                position_m=settings.gripper_flange_translation_m,
                quaternion_xyzw=settings.gripper_flange_quaternion_xyzw,
            ),
        )

    if robot is None:
        robot = FakeRobotAdapter(gripper_pose_provider) if use_fake else PylibfrankaRobotAdapter(settings.robot_ip, gripper_pose_provider)
    if gripper is None and settings.gripper_enabled:
        gripper = FakeGripperAdapter() if use_fake else PylibfrankaGripperAdapter(settings.robot_ip)
    manager = CommandManager(settings, robot, gripper)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        try:
            manager.start()
        except Exception:
            LOGGER.exception("failed to start control service")
            raise
        try:
            yield
        finally:
            manager.shutdown()

    app = FastAPI(title="franka_fci", version=__version__, lifespan=lifespan)
    app.state.manager = manager
    app.state.settings = settings

    def source(request: Request) -> str | None:
        return request.client.host if request.client else None

    @app.get("/api/v1/health", response_model=HealthResponse)
    def health() -> HealthResponse:
        robot_state = manager.robot_state()
        gripper_state = manager.gripper_state()
        healthy = robot_state.connected and robot_state.fci_active and not robot_state.errors
        return HealthResponse(
            service="franka_fci",
            version=__version__,
            status="ok" if healthy else "degraded",
            robot=robot_state,
            gripper=gripper_state,
            active_command_id=manager.active_command_id,
        )

    @app.get("/api/v1/state", response_model=StateResponse)
    def state() -> StateResponse:
        return StateResponse(robot=manager.robot_state(), gripper=manager.gripper_state())

    def submit_or_error(
        operation, kind: str, request_id: str | None, request: Request, fingerprint: str = ""
    ) -> CommandResponse:
        try:
            record = manager.submit(kind, request_id, source(request), operation, fingerprint)
        except CommandConflict as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except ServiceUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return CommandResponse(
            command_id=record.command_id,
            status=record.status,
            accepted_at=record.accepted_at,
        )

    @app.post("/api/v1/motions/joint-position", response_model=CommandResponse, status_code=202)
    def joint_position(
        body: JointPositionRequest,
        request: Request,
    ) -> CommandResponse:
        if body.duration_s > settings.max_motion_duration_s:
            raise HTTPException(status_code=422, detail="duration_s exceeds configured maximum")
        operation = manager.joint_operation(body.target_rad, body.duration_s)
        return submit_or_error(operation, "joint_position", body.request_id, request, body.model_dump_json())

    @app.post("/api/v1/motions/cartesian-pose", response_model=CommandResponse, status_code=202)
    def cartesian_pose(
        body: CartesianPoseRequest,
        request: Request,
    ) -> CommandResponse:
        if body.duration_s > settings.max_motion_duration_s:
            raise HTTPException(status_code=422, detail="duration_s exceeds configured maximum")
        try:
            operation = manager.cartesian_operation(body.pose(), body.duration_s)
        except ServiceUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return submit_or_error(operation, "cartesian_gripper_pose", body.request_id, request, body.model_dump_json())

    @app.post("/api/v1/gripper/homing", response_model=CommandResponse, status_code=202)
    def gripper_homing(request: Request) -> CommandResponse:
        try:
            operation = manager.gripper_operation(manager.gripper.homing)  # type: ignore[union-attr]
        except ServiceUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return submit_or_error(operation, "gripper_homing", None, request, "homing")

    @app.post("/api/v1/gripper/move", response_model=CommandResponse, status_code=202)
    def gripper_move(
        body: GripperMoveRequest,
        request: Request,
    ) -> CommandResponse:
        try:
            operation = manager.gripper_operation(lambda: manager.gripper.move(body.width_m, body.speed_m_s))  # type: ignore[union-attr]
        except ServiceUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return submit_or_error(operation, "gripper_move", body.request_id, request, body.model_dump_json())

    @app.post("/api/v1/gripper/grasp", response_model=CommandResponse, status_code=202)
    def gripper_grasp(
        body: GripperGraspRequest,
        request: Request,
    ) -> CommandResponse:
        try:
            operation = manager.gripper_operation(lambda: manager.gripper.grasp(body.width_m, body.speed_m_s, body.force_n))  # type: ignore[union-attr]
        except ServiceUnavailable as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return submit_or_error(operation, "gripper_grasp", body.request_id, request, body.model_dump_json())

    @app.get("/api/v1/gripper/state", response_model=GripperState | None)
    def gripper_state() -> GripperState | None:
        return manager.gripper_state()

    @app.get("/api/v1/commands/{command_id}")
    def command(command_id: str):
        record = manager.get(command_id)
        if record is None:
            raise HTTPException(status_code=404, detail="command not found")
        return record

    @app.post("/api/v1/stop")
    def stop():
        manager.stop()
        return {"status": "stop_requested", "command_id": manager.active_command_id}

    return app
