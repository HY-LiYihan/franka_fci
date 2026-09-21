from fastapi.testclient import TestClient
import pytest

from franka_fci.api.app import create_app
from franka_fci.config import Settings
from franka_fci.models.poses import Pose, flange_to_gripper_pose, gripper_to_flange_target
from franka_fci.robot.adapters import FakeGripperAdapter, FakeRobotAdapter


def test_pose_transform_round_trip() -> None:
    mount = Pose(position_m=(0.0, 0.0, 0.1), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))
    flange = Pose(position_m=(0.4, 0.1, 0.3), quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))
    gripper = flange_to_gripper_pose(flange, mount)
    recovered = gripper_to_flange_target(gripper, mount)
    assert recovered.position_m == pytest.approx(flange.position_m)
    assert recovered.quaternion_xyzw == pytest.approx(flange.quaternion_xyzw)


def test_api_fake_lifecycle_without_auth() -> None:
    settings = Settings(
        gripper_flange_translation_m=(0.0, 0.0, 0.1),
        gripper_flange_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    app = create_app(settings, FakeRobotAdapter(), FakeGripperAdapter())
    with TestClient(app) as client:
        response = client.post(
            "/api/v1/motions/joint-position",
            json={"target_rad": [0.0] * 7, "duration_s": 1.0, "request_id": "same"},
        )
        assert response.status_code == 202
        command_id = response.json()["command_id"]
        repeat = client.post(
            "/api/v1/motions/joint-position",
            json={"target_rad": [0.0] * 7, "duration_s": 1.0, "request_id": "same"},
        )
        assert repeat.status_code == 202
        assert repeat.json()["command_id"] == command_id


def test_lan_settings_do_not_require_token() -> None:
    assert Settings(api_host="0.0.0.0").api_host == "0.0.0.0"
