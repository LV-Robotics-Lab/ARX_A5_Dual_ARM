import math

import pytest

from arx_wrapper.safety import (
    MotionGate,
    SafetyGateError,
    finite_vector,
    validate_gripper_position,
)


def test_motion_gate_defaults_to_dry_run() -> None:
    gate = MotionGate()

    assert gate.is_dry_run
    with pytest.raises(SafetyGateError, match="motion is disabled"):
        gate.require_motion()


@pytest.mark.parametrize(
    "kwargs, missing",
    [
        ({"execute": True}, "workspace clearance"),
        (
            {"execute": True, "clearance_confirmed": True},
            "emergency-stop readiness",
        ),
        (
            {"execute": True, "clearance_confirmed": True, "estop_ready": True},
            "exclusive control source",
        ),
    ],
)
def test_motion_gate_reports_missing_confirmation(kwargs: dict, missing: str) -> None:
    with pytest.raises(SafetyGateError, match=missing):
        MotionGate(**kwargs).require_motion()


def test_complete_motion_gate_allows_execution() -> None:
    MotionGate(
        execute=True,
        clearance_confirmed=True,
        estop_ready=True,
        control_source_exclusive=True,
    ).require_motion()


def test_finite_vector_validates_shape_and_numbers() -> None:
    assert finite_vector(range(6), size=6, name="joints") == (0, 1, 2, 3, 4, 5)
    with pytest.raises(SafetyGateError, match="exactly 6"):
        finite_vector(range(5), size=6, name="joints")
    with pytest.raises(SafetyGateError, match="finite"):
        finite_vector([0, 1, 2, 3, 4, math.nan], size=6, name="joints")


def test_gripper_position_is_bounded() -> None:
    assert validate_gripper_position(-1.2, minimum=-3.35, maximum=0.0) == -1.2
    with pytest.raises(SafetyGateError, match="outside configured range"):
        validate_gripper_position(0.1, minimum=-3.35, maximum=0.0)
