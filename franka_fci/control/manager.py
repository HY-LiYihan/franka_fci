"""Single-worker command manager that isolates API calls from hardware control."""

from __future__ import annotations

import logging
import queue
import secrets
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable

from franka_fci.config import Settings
from franka_fci.models.commands import CommandRecord, CommandStatus
from franka_fci.models.poses import Pose, flange_to_gripper_pose, gripper_to_flange_target
from franka_fci.models.state import GripperState, RobotState
from franka_fci.robot.adapters import GripperAdapter, RobotAdapter

LOGGER = logging.getLogger(__name__)


def utcnow() -> datetime:
    return datetime.now(timezone.utc)


@dataclass
class _Task:
    record: CommandRecord
    operation: Callable[[], None]


class CommandConflict(Exception):
    pass


class ServiceUnavailable(Exception):
    pass


class CommandManager:
    def __init__(
        self,
        settings: Settings,
        robot: RobotAdapter,
        gripper: GripperAdapter | None,
    ) -> None:
        self.settings = settings
        self.robot = robot
        self.gripper = gripper
        self._queue: queue.Queue[_Task | None] = queue.Queue()
        self._records: dict[str, CommandRecord] = {}
        self._request_ids: dict[str, tuple[str, str]] = {}
        self._lock = threading.RLock()
        self._active_id: str | None = None
        self._stop_requested = threading.Event()
        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._run, name="franka-control", daemon=True)
        self._state_worker = threading.Thread(target=self._state_loop, name="franka-state", daemon=True)
        self._robot_state = self.robot.read_state()
        self._gripper_state = self.gripper.read_state() if self.gripper else None

    def start(self) -> None:
        self.robot.connect()
        if self.gripper:
            self.gripper.connect()
        self.refresh_state()
        self._worker.start()
        self._state_worker.start()

    def shutdown(self) -> None:
        self._shutdown.set()
        self.stop()
        self._queue.put(None)
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        if self._state_worker.is_alive():
            self._state_worker.join(timeout=2.0)
        self.robot.disconnect()
        if self.gripper:
            self.gripper.disconnect()

    def refresh_state(self) -> None:
        try:
            robot_state = self.robot.read_state()
        except Exception as exc:  # pragma: no cover - hardware failure path
            LOGGER.exception("robot state update failed")
            robot_state = RobotState(connected=False, errors=[str(exc)], timestamp=utcnow())
        with self._lock:
            self._robot_state = robot_state
            if self.gripper:
                try:
                    self._gripper_state = self.gripper.read_state()
                except Exception as exc:  # pragma: no cover - hardware failure path
                    LOGGER.exception("gripper state update failed")
                    self._gripper_state = GripperState(connected=False, errors=[str(exc)], timestamp=utcnow())

    def _state_loop(self) -> None:
        while not self._shutdown.wait(0.05):
            self.refresh_state()

    def robot_state(self) -> RobotState:
        self.refresh_state()
        with self._lock:
            return self._robot_state.model_copy(deep=True)

    def gripper_state(self) -> GripperState | None:
        self.refresh_state()
        with self._lock:
            return self._gripper_state.model_copy(deep=True) if self._gripper_state else None

    @property
    def active_command_id(self) -> str | None:
        with self._lock:
            return self._active_id

    def submit(
        self,
        kind: str,
        request_id: str | None,
        source: str | None,
        operation: Callable[[], None],
        fingerprint: str = "",
    ) -> CommandRecord:
        with self._lock:
            if request_id and request_id in self._request_ids:
                old_id, old_fingerprint = self._request_ids[request_id]
                if old_fingerprint != f"{kind}:{fingerprint}":
                    raise CommandConflict("request_id is already associated with a different command")
                return self._records[old_id].model_copy(deep=True)
            if self._active_id is not None:
                raise CommandConflict("another command is active")
            if not self._ready_for_motion():
                raise ServiceUnavailable("robot is not ready for motion")
            command_id = f"cmd_{secrets.token_urlsafe(10)}"
            now = utcnow()
            record = CommandRecord(
                command_id=command_id,
                status=CommandStatus.accepted,
                accepted_at=now,
                request_id=request_id,
                kind=kind,
                source=source,
            )
            self._records[command_id] = record
            if request_id:
                self._request_ids[request_id] = (command_id, f"{kind}:{fingerprint}")
            self._active_id = command_id
            self._set_status(record, CommandStatus.queued)
            self._queue.put(_Task(record, operation))
            return record.model_copy(deep=True)


    def get(self, command_id: str) -> CommandRecord | None:
        with self._lock:
            record = self._records.get(command_id)
            return record.model_copy(deep=True) if record else None

    def stop(self) -> None:
        with self._lock:
            if self._active_id is None:
                return
        self._stop_requested.set()
        self.robot.stop()
        if self.gripper:
            self.gripper.stop()

    def _ready_for_motion(self) -> bool:
        state = self._robot_state
        age_ms = (utcnow() - state.timestamp).total_seconds() * 1000
        return bool(state.connected and state.fci_active and not state.errors and age_ms <= self.settings.state_stale_after_ms)

    def _set_status(self, record: CommandRecord, status: CommandStatus, **updates: object) -> None:
        record.status = status
        for key, value in updates.items():
            setattr(record, key, value)

    def _run(self) -> None:
        while not self._shutdown.is_set():
            task = self._queue.get()
            if task is None:
                return
            with self._lock:
                self._set_status(task.record, CommandStatus.running, started_at=utcnow())
            if self._stop_requested.is_set():
                with self._lock:
                    self._set_status(
                        task.record,
                        CommandStatus.stopped,
                        finished_at=utcnow(),
                        error_code="STOP_REQUESTED",
                    )
                    self._active_id = None
                self._queue.task_done()
                continue
            self._stop_requested.clear()
            try:
                task.operation()
                status = CommandStatus.stopped if self._stop_requested.is_set() else CommandStatus.succeeded
                with self._lock:
                    self._set_status(task.record, status, finished_at=utcnow())
            except Exception as exc:
                LOGGER.exception("command %s failed", task.record.command_id)
                with self._lock:
                    self._set_status(
                        task.record,
                        CommandStatus.stopped if self._stop_requested.is_set() else CommandStatus.failed,
                        finished_at=utcnow(),
                        error_code="STOP_REQUESTED" if self._stop_requested.is_set() else "CONTROL_ERROR",
                        error_message=str(exc),
                    )
            finally:
                with self._lock:
                    self._active_id = None
                self._queue.task_done()

    def joint_operation(self, target: tuple[float, ...], duration_s: float) -> Callable[[], None]:
        return lambda: self.robot.move_joint_position(target, duration_s)

    def cartesian_operation(self, gripper_target: Pose, duration_s: float) -> Callable[[], None]:
        if not self.settings.gripper_flange_translation_m or not self.settings.gripper_flange_quaternion_xyzw:
            raise ServiceUnavailable("gripper flange transform is not configured")
        transform = Pose(
            position_m=self.settings.gripper_flange_translation_m,
            quaternion_xyzw=self.settings.gripper_flange_quaternion_xyzw,
        )
        flange_target = gripper_to_flange_target(gripper_target, transform)
        return lambda: self.robot.move_cartesian_flange(flange_target, duration_s)

    def gripper_operation(self, operation: Callable[[], None]) -> Callable[[], None]:
        if not self.gripper or not self.settings.gripper_enabled:
            raise ServiceUnavailable("gripper control is disabled")
        return operation

    def gripper_pose_from_flange(self, flange_pose: Pose) -> Pose | None:
        if not self.settings.gripper_flange_translation_m or not self.settings.gripper_flange_quaternion_xyzw:
            return None
        return flange_to_gripper_pose(
            flange_pose,
            Pose(
                position_m=self.settings.gripper_flange_translation_m,
                quaternion_xyzw=self.settings.gripper_flange_quaternion_xyzw,
            ),
        )
