"""Fail-closed gates and command validation for real ARX hardware."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Tuple


class SafetyGateError(RuntimeError):
    """Raised before a hardware-changing command when a safety gate is incomplete."""


@dataclass(frozen=True)
class MotionGate:
    """Explicit operator confirmations required before any motion or teaching command."""

    execute: bool = False
    clearance_confirmed: bool = False
    estop_ready: bool = False
    control_source_exclusive: bool = False

    def __post_init__(self) -> None:
        for name in (
            "execute",
            "clearance_confirmed",
            "estop_ready",
            "control_source_exclusive",
        ):
            if not isinstance(getattr(self, name), bool):
                raise TypeError(f"{name} must be a boolean")

    @property
    def is_dry_run(self) -> bool:
        return not self.execute

    def require_motion(self) -> None:
        if not self.execute:
            raise SafetyGateError("motion is disabled; run and review the dry-run first")
        missing = []
        if not self.clearance_confirmed:
            missing.append("workspace clearance")
        if not self.estop_ready:
            missing.append("emergency-stop readiness")
        if not self.control_source_exclusive:
            missing.append("exclusive control source")
        if missing:
            raise SafetyGateError("execution gate incomplete: " + ", ".join(missing))


def finite_vector(values: Iterable[float], *, size: int, name: str) -> Tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size:
        raise SafetyGateError(f"{name} must contain exactly {size} values, got {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise SafetyGateError(f"{name} must contain only finite values")
    return result


def validate_gripper_position(value: float, *, minimum: float, maximum: float) -> float:
    position = float(value)
    if not math.isfinite(position):
        raise SafetyGateError("gripper position must be finite")
    if not minimum <= position <= maximum:
        raise SafetyGateError(
            f"gripper position {position:g} is outside configured range [{minimum:g}, {maximum:g}]"
        )
    return position
