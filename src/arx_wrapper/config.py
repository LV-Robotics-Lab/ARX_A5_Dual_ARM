"""Typed, non-secret configuration for an ARX A5 dual-arm rig."""

from __future__ import annotations

import math
import os
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Mapping, Optional, Tuple

_NAME_RE = re.compile(r"^[A-Za-z0-9_.:-]+$")
_CAMERA_NAMES = frozenset({"cam_top", "cam_left_wrist", "cam_right_wrist"})


def _value(env: Mapping[str, str], name: str, default: str) -> str:
    value = env.get(name, default).strip()
    if not value:
        raise ValueError(f"{name} cannot be empty")
    return value


def _positive_float(env: Mapping[str, str], name: str, default: float) -> float:
    raw = env.get(name, str(default)).strip()
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a number, got {raw!r}") from exc
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {value}")
    return value


def _path(env: Mapping[str, str], name: str, default: str) -> Path:
    return Path(_value(env, name, default)).expanduser()


@dataclass(frozen=True)
class ArmEndpoint:
    """One physical A5 arm and its stable SocketCAN identity."""

    side: str
    can_interface: str
    urdf_name: str = "a5.urdf"

    def __post_init__(self) -> None:
        if self.side not in {"left", "right"}:
            raise ValueError(f"arm side must be left or right, got {self.side!r}")
        if not _NAME_RE.fullmatch(self.can_interface):
            raise ValueError(f"invalid CAN interface: {self.can_interface!r}")
        if Path(self.urdf_name).name != self.urdf_name or not self.urdf_name.endswith(".urdf"):
            raise ValueError(f"urdf_name must be a .urdf filename, got {self.urdf_name!r}")


@dataclass(frozen=True)
class CameraEndpoint:
    """Stable RealSense serial-to-mount binding."""

    name: str
    serial: str

    def __post_init__(self) -> None:
        if self.name not in _CAMERA_NAMES:
            allowed = ", ".join(sorted(_CAMERA_NAMES))
            raise ValueError(f"unsupported camera name {self.name!r}; choose {allowed}")
        if not _NAME_RE.fullmatch(self.serial):
            raise ValueError(f"invalid camera serial: {self.serial!r}")


def _camera_bindings(env: Mapping[str, str]) -> Tuple[CameraEndpoint, ...]:
    defaults = (
        ("cam_top", "f1470834"),
        ("cam_left_wrist", "260322270692"),
        ("cam_right_wrist", "260322273625"),
    )
    bindings = []
    for name, default_serial in defaults:
        env_name = "ARX_" + name.upper() + "_SERIAL"
        serial = env.get(env_name, default_serial).strip()
        if serial:
            bindings.append(CameraEndpoint(name=name, serial=serial))
    return tuple(bindings)


@dataclass(frozen=True)
class ArxConfig:
    """Stable configuration for one dual-arm collection workstation."""

    arms: Tuple[ArmEndpoint, ArmEndpoint]
    cameras: Tuple[CameraEndpoint, ...]
    vendor_root: Path
    data_root: Path
    task_name: str
    arm_publish_hz: float = 60.0
    camera_publish_hz: float = 30.0
    gripper_min: float = -3.35
    gripper_max: float = 0.0

    def __post_init__(self) -> None:
        sides = [arm.side for arm in self.arms]
        if sides != ["left", "right"]:
            raise ValueError("arms must be ordered as left, right")
        channels = [arm.can_interface for arm in self.arms]
        if len(set(channels)) != len(channels):
            raise ValueError(f"arm CAN interfaces must be unique, got {channels!r}")
        camera_names = [camera.name for camera in self.cameras]
        camera_serials = [camera.serial for camera in self.cameras]
        if len(set(camera_names)) != len(camera_names):
            raise ValueError(f"camera names must be unique, got {camera_names!r}")
        if len(set(camera_serials)) != len(camera_serials):
            raise ValueError(f"camera serials must be unique, got {camera_serials!r}")
        if self.task_name in {".", ".."} or not _NAME_RE.fullmatch(self.task_name):
            raise ValueError(f"invalid task name: {self.task_name!r}")
        if self.arm_publish_hz <= 0 or self.camera_publish_hz <= 0:
            raise ValueError("publish rates must be positive")
        if not all(math.isfinite(value) for value in (self.gripper_min, self.gripper_max)):
            raise ValueError("gripper limits must be finite")
        if self.gripper_min >= self.gripper_max:
            raise ValueError("gripper_min must be less than gripper_max")

    @classmethod
    def from_env(cls, env: Optional[Mapping[str, str]] = None) -> ArxConfig:
        values = os.environ if env is None else env
        urdf_name = _value(values, "ARX_URDF_NAME", "a5.urdf")
        left = ArmEndpoint(
            side="left",
            can_interface=_value(values, "ARX_LEFT_CAN", "can1"),
            urdf_name=urdf_name,
        )
        right = ArmEndpoint(
            side="right",
            can_interface=_value(values, "ARX_RIGHT_CAN", "can3"),
            urdf_name=urdf_name,
        )
        checkout_root = Path(__file__).resolve().parents[2]
        default_vendor_root = checkout_root if (checkout_root / "A5").is_dir() else Path.cwd()
        return cls(
            arms=(left, right),
            cameras=_camera_bindings(values),
            vendor_root=_path(values, "ARX_VENDOR_ROOT", str(default_vendor_root)),
            data_root=_path(values, "ARX_DATA_ROOT", "~/workspace/raw_data"),
            task_name=_value(values, "ARX_TASK_NAME", "egg_to_bowl"),
            arm_publish_hz=_positive_float(values, "ARX_ARM_PUBLISH_HZ", 60.0),
            camera_publish_hz=_positive_float(values, "ARX_CAMERA_PUBLISH_HZ", 30.0),
            gripper_min=float(values.get("ARX_GRIPPER_MIN", "-3.35")),
            gripper_max=float(values.get("ARX_GRIPPER_MAX", "0.0")),
        )

    def arm(self, side: str) -> ArmEndpoint:
        normalized = side.lower().removesuffix("_arm")
        for arm in self.arms:
            if arm.side == normalized:
                return arm
        raise KeyError(f"unknown arm {side!r}; choose left or right")

    @property
    def camera_bindings(self) -> Mapping[str, str]:
        return {camera.serial: camera.name for camera in self.cameras}

    @property
    def task_data_dir(self) -> Path:
        return self.data_root / self.task_name

    def as_dict(self) -> dict:
        data = asdict(self)
        data["vendor_root"] = str(self.vendor_root)
        data["data_root"] = str(self.data_root)
        data["task_data_dir"] = str(self.task_data_dir)
        return data
