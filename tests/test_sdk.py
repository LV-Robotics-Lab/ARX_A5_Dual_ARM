from pathlib import Path

import pytest

from arx_wrapper.config import ArxConfig
from arx_wrapper.safety import MotionGate, SafetyGateError
from arx_wrapper.sdk import ArxArm, ArxDualArm, ArxSdkUnavailable


class FakeBackend:
    def __init__(self, config: dict) -> None:
        self.config = config
        self.calls = []
        self.closed = False

    def get_joint_positions(self):
        return [0.1] * 6 + [-1.0]

    def get_joint_velocities(self):
        return [0.2] * 7

    def get_joint_currents(self):
        return [0.3] * 7

    def get_ee_pose(self):
        return [0.4, 0.5, 0.6, 1.0, 0.0, 0.0, 0.0]

    def get_ee_pose_xyzrpy(self):
        return [0.4, 0.5, 0.6, 0.0, 0.0, 0.0]

    def gravity_compensation(self):
        self.calls.append(("gravity_compensation",))
        return True

    def protect_mode(self):
        self.calls.append(("protect_mode",))
        return True

    def set_joint_positions(self, values):
        self.calls.append(("set_joint_positions", tuple(values)))

    def set_ee_pose_xyzrpy(self, *, xyzrpy):
        self.calls.append(("set_ee_pose_xyzrpy", tuple(xyzrpy)))

    def set_gripper_pos(self, value):
        self.calls.append(("set_gripper_pos", value))

    def close(self):
        self.closed = True


def config(tmp_path: Path) -> ArxConfig:
    return ArxConfig.from_env({"ARX_VENDOR_ROOT": str(tmp_path)})


def live_gate() -> MotionGate:
    return MotionGate(True, True, True, True)


def test_construction_is_lazy_and_read_snapshot_works(tmp_path: Path) -> None:
    created = []

    def factory(values):
        backend = FakeBackend(values)
        created.append(backend)
        return backend

    arm = ArxArm("left", config=config(tmp_path), backend_factory=factory)
    assert not arm.is_connected

    arm.connect()
    snapshot = arm.snapshot()

    assert created[0].config == {"can_port": "can1", "urdf_name": "a5.urdf"}
    assert snapshot.side == "left"
    assert snapshot.joint_positions[-1] == -1.0
    assert snapshot.ee_pose_xyz_wxyz[3] == 1.0


def test_motion_methods_are_blocked_by_default(tmp_path: Path) -> None:
    backend = FakeBackend({})
    arm = ArxArm(
        "left",
        config=config(tmp_path),
        backend_factory=lambda _: backend,
    ).connect()

    with pytest.raises(SafetyGateError, match="motion is disabled"):
        arm.gravity_compensation()
    with pytest.raises(SafetyGateError, match="motion is disabled"):
        arm.set_joint_positions([0.0] * 6)
    assert backend.calls == []


def test_live_commands_are_validated_and_forwarded(tmp_path: Path) -> None:
    backend = FakeBackend({})
    arm = ArxArm(
        "right",
        config=config(tmp_path),
        gate=live_gate(),
        backend_factory=lambda _: backend,
    ).connect()

    arm.gravity_compensation()
    arm.set_joint_positions([0, 1, 2, 3, 4, 5])
    arm.set_ee_pose_xyzrpy([0, 0, 0, 0, 0, 0])
    arm.set_gripper_pos(-1.0)

    assert backend.calls == [
        ("gravity_compensation",),
        ("set_joint_positions", (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)),
        ("set_ee_pose_xyzrpy", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        ("set_gripper_pos", -1.0),
    ]
    with pytest.raises(SafetyGateError, match="exactly 6"):
        arm.set_joint_positions([0.0] * 7)
    with pytest.raises(SafetyGateError, match="outside configured range"):
        arm.set_gripper_pos(0.5)


def test_context_manager_closes_backend(tmp_path: Path) -> None:
    backend = FakeBackend({})
    with ArxArm("left", config=config(tmp_path), backend_factory=lambda _: backend) as arm:
        assert arm.is_connected

    assert backend.closed
    assert not arm.is_connected


def test_dual_arm_connect_rolls_back_if_second_arm_fails(tmp_path: Path) -> None:
    first = FakeBackend({})
    calls = 0

    def factory(values):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise ConnectionError("right CAN failed")
        return first

    pair = ArxDualArm(config=config(tmp_path), backend_factory=factory)
    with pytest.raises(ConnectionError, match="right CAN failed"):
        pair.connect()

    assert first.closed
    assert not pair.left.is_connected


def test_dual_arm_teaching_rolls_left_back_if_right_fails(tmp_path: Path) -> None:
    backends = []

    class FailingTeachingBackend(FakeBackend):
        def gravity_compensation(self):
            if self.config["can_port"] == "can3":
                raise RuntimeError("right refused teaching mode")
            return super().gravity_compensation()

    def factory(values):
        backend = FailingTeachingBackend(values)
        backends.append(backend)
        return backend

    pair = ArxDualArm(config=config(tmp_path), gate=live_gate(), backend_factory=factory).connect()
    with pytest.raises(RuntimeError, match="right refused"):
        pair.enter_teaching_mode()

    assert backends[0].calls == [("gravity_compensation",), ("protect_mode",)]


def test_missing_vendor_checkout_has_actionable_error(tmp_path: Path) -> None:
    arm = ArxArm("left", config=config(tmp_path))

    with pytest.raises(ArxSdkUnavailable, match="ARX_VENDOR_ROOT"):
        arm.connect()
