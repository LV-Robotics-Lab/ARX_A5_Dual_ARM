"""Lifecycle and safety wrapper for the optional vendored ARX A5 SDK."""

from __future__ import annotations

import importlib
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Optional, Tuple, Union

from .config import ArmEndpoint, ArxConfig
from .safety import MotionGate, finite_vector, validate_gripper_position


class ArxSdkUnavailable(ImportError):
    """The vendored ARX extension or its Linux shared libraries are unavailable."""


@dataclass(frozen=True)
class ArmSnapshot:
    side: str
    can_interface: str
    joint_positions: Tuple[float, ...]
    joint_velocities: Tuple[float, ...]
    joint_currents: Tuple[float, ...]
    ee_pose_xyz_wxyz: Tuple[float, ...]
    timestamp_s: float

    def as_dict(self) -> dict:
        return asdict(self)


BackendFactory = Callable[[dict], Any]


def _load_vendor_factory(vendor_root: Path) -> BackendFactory:
    root = vendor_root.expanduser().resolve()
    if not (root / "A5" / "bimanual").is_dir():
        raise ArxSdkUnavailable(
            f"ARX vendor checkout not found under {root}. Set ARX_VENDOR_ROOT to the "
            "arx_wrapper checkout containing A5/, then run A5/setup.sh."
        )
    root_string = str(root)
    if root_string not in sys.path:
        sys.path.insert(0, root_string)
    try:
        module = importlib.import_module("A5.bimanual")
    except (FileNotFoundError, ImportError, OSError, TypeError) as exc:
        raise ArxSdkUnavailable(
            "ARX A5 Python bindings are unavailable. Build the vendored SDK on the Linux "
            "robot workstation with `cd A5 && ./setup.sh`; config and doctor remain usable "
            "without the binary extension."
        ) from exc
    return module.SingleArm


class ArxArm:
    """Context-managed A5 arm connection with fail-closed motion methods.

    Constructing the wrapper does not import the vendor binary and does not connect to CAN.
    Read methods are available after ``connect()``. Every hardware-changing method checks the
    supplied :class:`MotionGate` immediately before calling the vendor SDK.
    """

    def __init__(
        self,
        arm: Union[str, ArmEndpoint] = "left",
        *,
        config: Optional[ArxConfig] = None,
        gate: Optional[MotionGate] = None,
        backend_factory: Optional[BackendFactory] = None,
    ) -> None:
        self.config = ArxConfig.from_env() if config is None else config
        self.endpoint = self.config.arm(arm) if isinstance(arm, str) else arm
        self.gate = MotionGate() if gate is None else gate
        self._backend_factory = backend_factory
        self._backend: Any = None

    @property
    def is_connected(self) -> bool:
        return self._backend is not None

    @property
    def raw(self) -> Any:
        if self._backend is None:
            raise RuntimeError(f"ARX {self.endpoint.side} arm is not connected")
        return self._backend

    def connect(self) -> ArxArm:
        if self._backend is not None:
            return self
        factory = self._backend_factory or _load_vendor_factory(self.config.vendor_root)
        self._backend = factory(
            {
                "can_port": self.endpoint.can_interface,
                "urdf_name": self.endpoint.urdf_name,
            }
        )
        return self

    def close(self) -> None:
        backend, self._backend = self._backend, None
        if backend is None:
            return
        candidates = [backend, getattr(backend, "arm", None)]
        for candidate in candidates:
            if candidate is None:
                continue
            for name in ("close", "disconnect", "cleanup"):
                method = getattr(candidate, name, None)
                if callable(method):
                    method()
                    return

    def __enter__(self) -> ArxArm:
        return self.connect()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def snapshot(self) -> ArmSnapshot:
        backend = self.raw
        return ArmSnapshot(
            side=self.endpoint.side,
            can_interface=self.endpoint.can_interface,
            joint_positions=tuple(float(value) for value in backend.get_joint_positions()),
            joint_velocities=tuple(float(value) for value in backend.get_joint_velocities()),
            joint_currents=tuple(float(value) for value in backend.get_joint_currents()),
            ee_pose_xyz_wxyz=tuple(float(value) for value in backend.get_ee_pose()),
            timestamp_s=time.time(),
        )

    def get_joint_positions(self) -> Any:
        return self.raw.get_joint_positions()

    def get_joint_velocities(self) -> Any:
        return self.raw.get_joint_velocities()

    def get_joint_currents(self) -> Any:
        return self.raw.get_joint_currents()

    def get_ee_pose(self) -> Any:
        return self.raw.get_ee_pose()

    def get_ee_pose_xyzrpy(self) -> Any:
        return self.raw.get_ee_pose_xyzrpy()

    def gravity_compensation(self) -> bool:
        self.gate.require_motion()
        return bool(self.raw.gravity_compensation())

    def protect_mode(self) -> bool:
        self.gate.require_motion()
        return bool(self.raw.protect_mode())

    def set_joint_positions(self, positions: Any) -> Any:
        self.gate.require_motion()
        command = finite_vector(positions, size=6, name="joint positions")
        return self.raw.set_joint_positions(command)

    def set_ee_pose_xyzrpy(self, xyzrpy: Any) -> Any:
        self.gate.require_motion()
        command = finite_vector(xyzrpy, size=6, name="end-effector xyzrpy")
        return self.raw.set_ee_pose_xyzrpy(xyzrpy=command)

    def set_gripper_pos(self, position: float) -> Any:
        self.gate.require_motion()
        command = validate_gripper_position(
            position,
            minimum=self.config.gripper_min,
            maximum=self.config.gripper_max,
        )
        return self.raw.set_gripper_pos(command)


class ArxDualArm:
    """Atomic lifecycle wrapper for the configured left/right A5 pair."""

    def __init__(
        self,
        *,
        config: Optional[ArxConfig] = None,
        gate: Optional[MotionGate] = None,
        backend_factory: Optional[BackendFactory] = None,
    ) -> None:
        self.config = ArxConfig.from_env() if config is None else config
        self.gate = MotionGate() if gate is None else gate
        self.left = ArxArm(
            self.config.arm("left"),
            config=self.config,
            gate=self.gate,
            backend_factory=backend_factory,
        )
        self.right = ArxArm(
            self.config.arm("right"),
            config=self.config,
            gate=self.gate,
            backend_factory=backend_factory,
        )

    @property
    def arms(self) -> Tuple[ArxArm, ArxArm]:
        return self.left, self.right

    def connect(self) -> ArxDualArm:
        self.left.connect()
        try:
            self.right.connect()
        except Exception:
            self.left.close()
            raise
        return self

    def enter_teaching_mode(self) -> None:
        self.gate.require_motion()
        self.left.gravity_compensation()
        try:
            self.right.gravity_compensation()
        except Exception:
            # Do not leave a half-configured pair in teaching mode if the second arm fails.
            try:
                self.left.protect_mode()
            finally:
                raise

    def close(self) -> None:
        try:
            self.right.close()
        finally:
            self.left.close()

    def __enter__(self) -> ArxDualArm:
        return self.connect()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()
