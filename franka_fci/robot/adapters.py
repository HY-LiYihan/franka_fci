"""Hardware adapters and deterministic fake adapters for tests."""

from __future__ import annotations

import logging
import threading
import time
from abc import ABC, abstractmethod
from datetime import datetime, timezone
from typing import Any, Callable

from franka_fci.models.poses import Pose
from franka_fci.models.state import GripperState, RobotState

LOGGER = logging.getLogger(__name__)


def utcnow() -> datetime:
    return datetime.now(timezone.utc)


class RobotAdapter(ABC):
    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def read_state(self) -> RobotState: ...

    @abstractmethod
    def move_joint_position(self, target_rad: tuple[float, ...], duration_s: float) -> None: ...

    @abstractmethod
    def move_cartesian_flange(self, target: Pose, duration_s: float) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...


class GripperAdapter(ABC):
    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def homing(self) -> None: ...

    @abstractmethod
    def move(self, width_m: float, speed_m_s: float) -> None: ...

    @abstractmethod
    def grasp(self, width_m: float, speed_m_s: float, force_n: float) -> None: ...

    @abstractmethod
    def read_state(self) -> GripperState: ...

    @abstractmethod
    def stop(self) -> None: ...


class PylibfrankaRobotAdapter(RobotAdapter):
    """Thin boundary around pylibfranka; only this class knows its API."""

    def __init__(self, robot_ip: str, gripper_pose_provider: Callable[[Pose], Pose] | None = None):
        self.robot_ip = robot_ip
        self._robot: Any = None
        self._module: Any = None
        self._stop_requested = threading.Event()
        self._gripper_pose_provider = gripper_pose_provider

    def connect(self) -> None:
        try:
            import pylibfranka
        except ImportError as exc:  # pragma: no cover - hardware-only path
            raise RuntimeError("pylibfranka is required on the FR3 industrial PC") from exc
        self._module = pylibfranka
        self._robot = pylibfranka.Robot(self.robot_ip)

    def disconnect(self) -> None:
        self._robot = None

    def _require_robot(self) -> Any:
        if self._robot is None:
            raise RuntimeError("robot is not connected")
        return self._robot

    def read_state(self) -> RobotState:
        robot = self._require_robot()
        raw = robot.read_once()
        joints = tuple(float(value) for value in getattr(raw, "q", ())) or None
        velocities = tuple(float(value) for value in getattr(raw, "dq", ())) or None
        pose = _pose_from_flattened_matrix(getattr(raw, "O_T_EE", None))
        return RobotState(
            connected=True,
            fci_active=True,
            robot_mode=str(getattr(raw, "robot_mode", "unknown")),
            errors=[str(item) for item in getattr(raw, "errors", ())],
            timestamp=utcnow(),
            joint_position_rad=joints,
            joint_velocity_rad_s=velocities,
            flange_pose=pose,
            gripper_pose=self._gripper_pose_provider(pose)
            if pose is not None and self._gripper_pose_provider
            else None,
        )

    def _control(self, make_command: Callable[[float], Any], duration_s: float) -> None:
        robot = self._require_robot()
        start = time.monotonic()
        self._stop_requested.clear()

        def callback(_state: Any) -> Any:
            if self._stop_requested.is_set():
                raise RuntimeError("stop requested")
            progress = min(1.0, (time.monotonic() - start) / duration_s)
            command = make_command(progress)
            if progress >= 1.0:
                finished = getattr(self._module, "motion_finished", None)
                return finished(command) if finished else command
            return command

        robot.control(callback)

    def move_joint_position(self, target_rad: tuple[float, ...], duration_s: float) -> None:
        current = self.read_state().joint_position_rad
        if current is None or len(current) != 7:
            raise RuntimeError("robot did not provide a 7-joint state")
        target = tuple(target_rad)

        def command(progress: float) -> Any:
            values = tuple(a + (b - a) * progress for a, b in zip(current, target))
            return self._module.JointPositions(values)

        self._control(command, duration_s)

    def move_cartesian_flange(self, target: Pose, duration_s: float) -> None:
        def command(_progress: float) -> Any:
            values = _pose_to_flattened_matrix(target)
            return self._module.CartesianPose(values)

        self._control(command, duration_s)

    def stop(self) -> None:
        self._stop_requested.set()


class PylibfrankaGripperAdapter(GripperAdapter):
    def __init__(self, robot_ip: str):
        self.robot_ip = robot_ip
        self._gripper: Any = None

    def connect(self) -> None:
        try:
            import pylibfranka
        except ImportError as exc:  # pragma: no cover - hardware-only path
            raise RuntimeError("pylibfranka is required on the FR3 industrial PC") from exc
        self._gripper = pylibfranka.Gripper(self.robot_ip)

    def disconnect(self) -> None:
        self._gripper = None

    def _require_gripper(self) -> Any:
        if self._gripper is None:
            raise RuntimeError("gripper is not connected")
        return self._gripper

    def homing(self) -> None:
        self._require_gripper().homing()

    def move(self, width_m: float, speed_m_s: float) -> None:
        self._require_gripper().move(width_m, speed_m_s)

    def grasp(self, width_m: float, speed_m_s: float, force_n: float) -> None:
        self._require_gripper().grasp(width_m, speed_m_s, force_n)

    def read_state(self) -> GripperState:
        raw = self._require_gripper().read_once()
        return GripperState(
            connected=True,
            width_m=_optional_float(getattr(raw, "width", None)),
            max_width_m=_optional_float(getattr(raw, "max_width", None)),
            grasped=getattr(raw, "is_grasped", None),
            homed=bool(getattr(raw, "homed", False)),
            moving=bool(getattr(raw, "is_moving", False)),
            errors=[str(item) for item in getattr(raw, "errors", ())],
            timestamp=utcnow(),
        )

    def stop(self) -> None:
        LOGGER.warning("gripper stop requested; hardware-specific stop is not available")


class FakeRobotAdapter(RobotAdapter):
    def __init__(self, gripper_pose_provider: Callable[[Pose], Pose] | None = None):
        self.connected = False
        self.stop_requested = False
        self.last_joint_target: tuple[float, ...] | None = None
        self.last_cartesian_target: Pose | None = None
        self._gripper_pose_provider = gripper_pose_provider

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def read_state(self) -> RobotState:
        pose = Pose(position_m=(0.4, 0.0, 0.4), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))
        return RobotState(
            connected=self.connected,
            fci_active=self.connected,
            robot_mode="automatic" if self.connected else "unknown",
            timestamp=utcnow(),
            joint_position_rad=(0.0,) * 7,
            joint_velocity_rad_s=(0.0,) * 7,
            flange_pose=pose,
            gripper_pose=self._gripper_pose_provider(pose)
            if self._gripper_pose_provider
            else None,
        )

    def move_joint_position(self, target_rad: tuple[float, ...], duration_s: float) -> None:
        self._ensure_connected()
        self.last_joint_target = target_rad
        time.sleep(min(duration_s, 0.01))

    def move_cartesian_flange(self, target: Pose, duration_s: float) -> None:
        self._ensure_connected()
        self.last_cartesian_target = target
        time.sleep(min(duration_s, 0.01))

    def stop(self) -> None:
        self.stop_requested = True

    def _ensure_connected(self) -> None:
        if not self.connected:
            raise RuntimeError("robot is not connected")


class FakeGripperAdapter(GripperAdapter):
    def __init__(self):
        self.connected = False
        self.homed = False
        self.width_m = 0.08
        self.last_grasp: tuple[float, float, float] | None = None

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def homing(self) -> None:
        self._ensure_connected()
        self.homed = True

    def move(self, width_m: float, speed_m_s: float) -> None:
        self._ensure_connected()
        self.width_m = width_m

    def grasp(self, width_m: float, speed_m_s: float, force_n: float) -> None:
        self._ensure_connected()
        self.width_m = width_m
        self.last_grasp = (width_m, speed_m_s, force_n)

    def read_state(self) -> GripperState:
        return GripperState(
            connected=self.connected,
            width_m=self.width_m,
            max_width_m=0.08,
            grasped=bool(self.last_grasp),
            homed=self.homed,
            moving=False,
            timestamp=utcnow(),
        )

    def stop(self) -> None:
        return None

    def _ensure_connected(self) -> None:
        if not self.connected:
            raise RuntimeError("gripper is not connected")


def _optional_float(value: Any) -> float | None:
    return None if value is None else float(value)


def _pose_from_flattened_matrix(value: Any) -> Pose | None:
    if value is None:
        return None
    from franka_fci.models.poses import matrix_to_pose

    values = list(value)
    if len(values) != 16:
        return None
    return matrix_to_pose(__import__("numpy").array(values, dtype=float).reshape((4, 4), order="F"))


def _pose_to_flattened_matrix(pose: Pose) -> tuple[float, ...]:
    from franka_fci.models.poses import pose_to_matrix

    return tuple(float(value) for value in pose_to_matrix(pose).reshape(16, order="F"))
