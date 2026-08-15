from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from arx_wrapper.safety import MotionGate, SafetyGateError
from arx_wrapper.x5 import (
    X5Config,
    X5DualArm,
    command_kind_for_action_space,
    controller_type_for_action_space,
)


def test_x5_types_are_exported_from_the_top_level_package() -> None:
    import arx_wrapper

    assert arx_wrapper.__version__ == "0.2.0"
    assert arx_wrapper.X5Config is X5Config
    assert arx_wrapper.X5DualArm is X5DualArm


def live_gate() -> MotionGate:
    return MotionGate(
        execute=True,
        clearance_confirmed=True,
        estop_ready=True,
        control_source_exclusive=True,
    )


class FakeRobotConfig:
    def __init__(self) -> None:
        self.joint_dof = 6
        self.gripper_open_readout = -3.4
        self.gripper_width = 0.082
        self.urdf_path = "/fake/x5.urdf"
        self.joint_pos_min = np.full(6, -3.0)
        self.joint_pos_max = np.full(6, 3.0)
        self.base_link_name = "base"
        self.eef_link_name = "eef"
        self.gravity_vector = np.array([0.0, 0.0, -9.81])


class FakeControllerConfig:
    def __init__(self) -> None:
        self.background_send_recv = False
        self.gravity_compensation = False
        self.default_preview_time = 0.0
        self.default_kp = np.arange(1, 7, dtype=np.float64)
        self.default_kd = np.arange(7, 13, dtype=np.float64)
        self.default_gripper_kp = 13.0
        self.default_gripper_kd = 14.0


class FakeJointState:
    def __init__(self, offset: float, gripper: float) -> None:
        self._pos = np.arange(6, dtype=np.float64) + offset
        self._vel = np.arange(6, dtype=np.float64) + offset + 0.1
        self._torque = np.arange(6, dtype=np.float64) + offset + 0.2
        self.gripper_pos = gripper
        self.gripper_vel = offset + 0.3
        self.gripper_torque = offset + 0.4

    def pos(self) -> np.ndarray:
        return self._pos

    def vel(self) -> np.ndarray:
        return self._vel

    def torque(self) -> np.ndarray:
        return self._torque


class FakeJointCommand:
    def __init__(self, dof: int) -> None:
        self._pos = np.zeros(dof, dtype=np.float64)
        self.gripper_pos = 0.0

    def pos(self) -> np.ndarray:
        return self._pos

    def clone(self):
        result = FakeJointCommand(len(self._pos))
        result._pos[:] = self._pos
        result.gripper_pos = self.gripper_pos
        return result


class FakeEEFCommand:
    def __init__(self, pose_6d, gripper_pos: float) -> None:
        self.pose_6d = np.asarray(pose_6d, dtype=np.float64).copy()
        self.gripper_pos = float(gripper_pos)

    def clone(self):
        return FakeEEFCommand(self.pose_6d, self.gripper_pos)


class FakeEEFState:
    def __init__(self, values) -> None:
        self._values = np.asarray(values, dtype=np.float64)

    def pose_6d(self) -> np.ndarray:
        return self._values


class FakeGain:
    def __init__(self, dof: int) -> None:
        self._kp = np.ones(dof, dtype=np.float64)
        self._kd = np.ones(dof, dtype=np.float64)
        self.gripper_kp = 1.0
        self.gripper_kd = 1.0

    def kp(self) -> np.ndarray:
        return self._kp

    def kd(self) -> np.ndarray:
        return self._kd


class FakeController:
    def __init__(self, can_name: str, *, fail_damping: bool = False) -> None:
        self.can_name = can_name
        self.offset = 0.0 if can_name == "can1" else 10.0
        self.state = FakeJointState(
            self.offset,
            0.04 if can_name == "can1" else 0.06,
        )
        self.fail_damping = fail_damping
        self.log_levels = []
        self.joint_commands = []
        self.eef_commands = []
        self.gains = []
        self.damping_calls = 0
        self.home_calls = 0
        self.close_calls = 0

    def set_log_level(self, level) -> None:
        self.log_levels.append(level)

    def get_joint_state(self) -> FakeJointState:
        return self.state

    def get_eef_state(self) -> FakeEEFState:
        return FakeEEFState([0.1 + self.offset, 0.2, 0.3, 0.1, -0.2, 0.3])

    def get_gain(self) -> FakeGain:
        return FakeGain(6)

    def set_gain(self, gain: FakeGain) -> None:
        self.gains.append(gain)

    def set_to_damping(self) -> None:
        self.damping_calls += 1
        if self.fail_damping:
            raise RuntimeError(f"{self.can_name} damping failed")

    def set_joint_cmd(self, command: FakeJointCommand) -> None:
        self.joint_commands.append(command.clone())

    def set_eef_cmd(self, command: FakeEEFCommand) -> None:
        self.eef_commands.append(command.clone())

    def reset_to_home(self) -> None:
        self.home_calls += 1

    def close(self) -> None:
        self.close_calls += 1


class FakeSolver:
    def __init__(self, *args) -> None:
        self.args = args

    def forward_kinematics(self, joint_pos) -> np.ndarray:
        return np.concatenate([np.asarray(joint_pos)[:3], np.zeros(3)])


class FakeSdk:
    __version__ = "fake-1"

    def __init__(
        self,
        *,
        fail_can: str = "",
        fail_damping_can: str = "",
    ) -> None:
        self.fail_can = fail_can
        self.fail_damping_can = fail_damping_can
        self.controllers = []
        robot_factory = SimpleNamespace(get_config=lambda model: FakeRobotConfig())
        controller_factory = SimpleNamespace(
            get_config=lambda controller_type, dof: FakeControllerConfig()
        )
        self.RobotConfigFactory = SimpleNamespace(get_instance=lambda: robot_factory)
        self.ControllerConfigFactory = SimpleNamespace(get_instance=lambda: controller_factory)
        self.LogLevel = SimpleNamespace(INFO="INFO")
        self.Arx5JointController = self._make_controller
        self.Arx5CartesianController = self._make_controller
        self.Arx5Solver = FakeSolver
        self.JointState = FakeJointCommand
        self.EEFState = FakeEEFCommand
        self.Gain = FakeGain

    def _make_controller(self, robot_config, controller_config, can_name):
        del robot_config, controller_config
        if can_name == self.fail_can:
            raise ConnectionError(f"{can_name} failed")
        controller = FakeController(
            can_name,
            fail_damping=can_name == self.fail_damping_can,
        )
        self.controllers.append(controller)
        return controller


def pair(
    *,
    action_space: str = "abs_qpos",
    sdk: FakeSdk | None = None,
    gate: MotionGate | None = None,
) -> tuple[X5DualArm, FakeSdk]:
    fake = FakeSdk() if sdk is None else sdk
    controller = X5DualArm(
        config=X5Config(
            left_can="can1",
            right_can="can3",
            action_space=action_space,
            left_gripper_open_readout=-3.41,
            right_gripper_open_readout=-3.42,
        ),
        gate=gate,
        sdk_loader=lambda: fake,
    )
    return controller, fake


def test_construction_is_lazy_and_status_does_not_load_sdk() -> None:
    loads = []
    controller = X5DualArm(sdk_loader=lambda: loads.append(True))

    assert loads == []
    assert controller.status()["sdk_loaded"] is False
    assert controller.status()["connected"] is False


def test_connect_and_read_state_preserve_14d_order_and_units() -> None:
    controller, sdk = pair()
    controller.connect()

    state = controller.read_state()

    assert state.qpos.dtype == np.float32
    assert state.qpos.shape == (14,)
    assert state.qvel.shape == (14,)
    assert state.effort.shape == (14,)
    assert state.eef_pose_6d.shape == (12,)
    assert state.eef_wxyz.shape == (14,)
    assert np.allclose(state.qpos[:7], [0, 1, 2, 3, 4, 5, 0.04])
    assert np.allclose(state.qpos[7:], [10, 11, 12, 13, 14, 15, 0.06])
    assert np.linalg.norm(state.eef_wxyz[3:7]) == pytest.approx(1.0)
    assert controller.status()["sdk_version"] == "fake-1"
    assert controller.status()["control_mode"] == "damping"
    assert [item.can_name for item in sdk.controllers] == ["can1", "can3"]
    assert [item.damping_calls for item in sdk.controllers] == [1, 1]
    assert all(item.home_calls == 0 for item in sdk.controllers)

    controller.close()


def test_default_motion_gate_blocks_modes_but_safe_stop_and_close_work() -> None:
    controller, sdk = pair()
    controller.connect()

    with pytest.raises(SafetyGateError, match="motion is disabled"):
        controller.set_damping_mode()
    with pytest.raises(SafetyGateError, match="motion is disabled"):
        controller.set_position_mode()
    assert all(item.joint_commands == [] for item in sdk.controllers)

    assert [item.damping_calls for item in sdk.controllers] == [1, 1]
    controller.safe_stop()
    assert [item.damping_calls for item in sdk.controllers] == [2, 2]
    assert all(item.home_calls == 0 for item in sdk.controllers)

    controller.close()
    assert [item.damping_calls for item in sdk.controllers] == [3, 3]
    assert [item.close_calls for item in sdk.controllers] == [1, 1]
    assert all(item.home_calls == 0 for item in sdk.controllers)

    controller.close()
    assert [item.close_calls for item in sdk.controllers] == [1, 1]


def test_qpos_position_mode_latches_state_and_sends_both_sides() -> None:
    controller, sdk = pair(gate=live_gate())
    controller.connect()
    controller.set_position_mode()

    assert controller.control_mode == "position"
    assert np.allclose(sdk.controllers[0].joint_commands[0].pos(), np.arange(6))
    assert np.allclose(sdk.controllers[1].joint_commands[0].pos(), np.arange(6) + 10)

    command = np.arange(14, dtype=np.float32) / 100.0
    controller.send_action("abs_qpos", command)

    left = sdk.controllers[0].joint_commands[-1]
    right = sdk.controllers[1].joint_commands[-1]
    assert np.allclose(left.pos(), command[:6])
    assert left.gripper_pos == pytest.approx(command[6] - 0.005)
    assert np.allclose(right.pos(), command[7:13])
    assert right.gripper_pos == pytest.approx(0.08)

    with pytest.raises(ValueError, match="does not match configured"):
        controller.send_action("abs_eef", command)

    controller.close()


def test_delta_qpos_is_added_to_the_current_normalized_state() -> None:
    controller, sdk = pair(action_space="delta_qpos", gate=live_gate())
    controller.connect()
    controller.set_position_mode()

    delta = np.full(14, 0.01, dtype=np.float32)
    controller.send_action("delta_qpos", delta)

    left = sdk.controllers[0].joint_commands[-1]
    right = sdk.controllers[1].joint_commands[-1]
    assert np.allclose(left.pos(), np.arange(6) + 0.01)
    assert np.allclose(right.pos(), np.arange(6) + 10.01)
    assert left.gripper_pos == pytest.approx(0.045)
    assert right.gripper_pos == pytest.approx(0.065)

    controller.close()


def test_eef_position_mode_and_command_use_xyzrpy_plus_gripper() -> None:
    controller, sdk = pair(action_space="abs_eef", gate=live_gate())
    controller.connect()
    controller.set_position_mode()

    assert len(sdk.controllers[0].eef_commands) == 1
    command = np.array(
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.02] * 2,
        dtype=np.float32,
    )
    controller.send_action("abs_eef", command)

    assert np.allclose(sdk.controllers[0].eef_commands[-1].pose_6d, command[:6])
    assert sdk.controllers[0].eef_commands[-1].gripper_pos == pytest.approx(0.015)
    assert np.allclose(sdk.controllers[1].eef_commands[-1].pose_6d, command[7:13])

    controller.close()


def test_reset_home_is_gated_and_returns_to_damping() -> None:
    controller, sdk = pair(gate=live_gate())
    controller.connect()
    controller.set_position_mode()

    controller.reset_home()

    assert [item.home_calls for item in sdk.controllers] == [1, 1]
    assert controller.control_mode == "damping"
    assert all(item.damping_calls >= 1 for item in sdk.controllers)
    controller.close()


def test_second_side_connect_failure_rolls_back_first_without_homing() -> None:
    sdk = FakeSdk(fail_can="can3")
    controller, _ = pair(sdk=sdk)

    with pytest.raises(ConnectionError, match="can3 failed"):
        controller.connect()

    assert len(sdk.controllers) == 1
    assert sdk.controllers[0].damping_calls == 1
    assert sdk.controllers[0].close_calls == 1
    assert sdk.controllers[0].home_calls == 0
    assert controller.status()["connected"] is False
    assert controller.status()["sdk_loaded"] is False


def test_connect_safe_stop_failure_closes_both_sides_and_never_homes() -> None:
    sdk = FakeSdk(fail_damping_can="can1")
    controller, _ = pair(sdk=sdk)

    with pytest.raises(RuntimeError, match="can1 damping failed"):
        controller.connect()

    assert [item.damping_calls for item in sdk.controllers] == [2, 2]
    assert [item.close_calls for item in sdk.controllers] == [1, 1]
    assert [item.home_calls for item in sdk.controllers] == [0, 0]
    assert controller.status()["connected"] is False
    assert controller.status()["control_mode"] == ""


def test_close_cleans_both_sides_even_if_one_damping_request_fails() -> None:
    sdk = FakeSdk()
    controller, _ = pair(sdk=sdk)
    controller.connect()
    sdk.controllers[0].fail_damping = True

    with pytest.raises(RuntimeError, match="can1 damping failed"):
        controller.close()

    assert [item.damping_calls for item in sdk.controllers] == [2, 2]
    assert [item.close_calls for item in sdk.controllers] == [1, 1]
    assert controller.status()["connected"] is False


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"left_can": "can1", "right_can": "can1"}, "must be different"),
        ({"action_space": "bad"}, "action_space"),
        ({"hz": 0}, "hz"),
        ({"gripper_width_m": float("nan")}, "gripper_width"),
        ({"gripper_command_margin_m": 0.082}, "margin"),
        ({"left_gripper_open_readout": -3.4}, "provided together"),
    ],
)
def test_x5_config_fails_closed(kwargs: dict, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        X5Config(**kwargs)


def test_action_space_helpers_are_explicit() -> None:
    assert controller_type_for_action_space("abs_qpos") == "joint_controller"
    assert controller_type_for_action_space("delta_eef") == "cartesian_controller"
    assert command_kind_for_action_space("delta_qpos") == "qpos"
    with pytest.raises(ValueError, match="unknown X5 action_space"):
        command_kind_for_action_space("bad")
