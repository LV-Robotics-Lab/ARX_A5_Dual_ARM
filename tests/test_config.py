from pathlib import Path

import pytest

from arx_wrapper.config import ArmEndpoint, ArxConfig, CameraEndpoint


def test_defaults_preserve_field_can_and_camera_bindings(tmp_path: Path) -> None:
    config = ArxConfig.from_env({"ARX_VENDOR_ROOT": str(tmp_path)})

    assert config.arm("left").can_interface == "can1"
    assert config.arm("right_arm").can_interface == "can3"
    assert config.camera_bindings == {
        "f1470834": "cam_top",
        "260322270692": "cam_left_wrist",
        "260322273625": "cam_right_wrist",
    }
    assert config.task_data_dir == Path("~/workspace/raw_data").expanduser() / "egg_to_bowl"


def test_environment_overrides_are_typed(tmp_path: Path) -> None:
    config = ArxConfig.from_env(
        {
            "ARX_LEFT_CAN": "can_left",
            "ARX_RIGHT_CAN": "can_right",
            "ARX_URDF_NAME": "a5_head.urdf",
            "ARX_CAM_TOP_SERIAL": "top-123",
            "ARX_CAM_LEFT_WRIST_SERIAL": "left-456",
            "ARX_CAM_RIGHT_WRIST_SERIAL": "right-789",
            "ARX_VENDOR_ROOT": str(tmp_path),
            "ARX_DATA_ROOT": str(tmp_path / "data"),
            "ARX_TASK_NAME": "pick_cube",
            "ARX_ARM_PUBLISH_HZ": "50",
            "ARX_CAMERA_PUBLISH_HZ": "15",
        }
    )

    assert config.arms == (
        ArmEndpoint("left", "can_left", "a5_head.urdf"),
        ArmEndpoint("right", "can_right", "a5_head.urdf"),
    )
    assert config.cameras[0] == CameraEndpoint("cam_top", "top-123")
    assert config.arm_publish_hz == 50.0
    assert config.camera_publish_hz == 15.0
    assert config.task_data_dir == tmp_path / "data" / "pick_cube"


def test_empty_camera_serial_disables_that_binding(tmp_path: Path) -> None:
    config = ArxConfig.from_env(
        {
            "ARX_VENDOR_ROOT": str(tmp_path),
            "ARX_CAM_RIGHT_WRIST_SERIAL": "",
        }
    )

    assert "cam_right_wrist" not in config.camera_bindings.values()


@pytest.mark.parametrize(
    "env, message",
    [
        ({"ARX_LEFT_CAN": "can1", "ARX_RIGHT_CAN": "can1"}, "must be unique"),
        ({"ARX_URDF_NAME": "../a5.urdf"}, "urdf_name"),
        ({"ARX_TASK_NAME": "bad task"}, "task name"),
        ({"ARX_TASK_NAME": ".."}, "task name"),
        ({"ARX_ARM_PUBLISH_HZ": "0"}, "must be positive"),
        ({"ARX_GRIPPER_MIN": "nan"}, "must be finite"),
    ],
)
def test_invalid_configuration_fails_closed(tmp_path: Path, env: dict, message: str) -> None:
    values = {"ARX_VENDOR_ROOT": str(tmp_path), **env}
    with pytest.raises(ValueError, match=message):
        ArxConfig.from_env(values)


def test_as_dict_is_json_safe(tmp_path: Path) -> None:
    config = ArxConfig.from_env({"ARX_VENDOR_ROOT": str(tmp_path)})
    data = config.as_dict()

    assert data["vendor_root"] == str(tmp_path)
    assert isinstance(data["data_root"], str)
    assert data["arms"][0]["side"] == "left"
