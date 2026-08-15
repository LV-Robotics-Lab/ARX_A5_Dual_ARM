"""Safety-gated dual-arm X5 controller built on the optional ARX5 SDK.

Importing this module does not import the vendor extension or connect to CAN.
The extension is loaded only by :meth:`X5DualArm.connect`, and tests can inject
an SDK-compatible object through ``sdk_loader``.
"""

from __future__ import annotations

import math
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np

from .safety import MotionGate, finite_vector

__all__ = [
    "X5_ACTION_SPACES",
    "X5Config",
    "X5DualArm",
    "X5SdkUnavailable",
    "X5State",
    "command_kind_for_action_space",
    "controller_type_for_action_space",
]

X5_ACTION_SPACES = ("abs_qpos", "delta_qpos", "abs_eef", "delta_eef")
X5_QPOS_ACTION_SPACES = ("abs_qpos", "delta_qpos")
X5_EEF_ACTION_SPACES = ("abs_eef", "delta_eef")


class X5SdkUnavailable(ImportError):
    """The optional ``arx5_interface`` extension cannot be imported."""


@dataclass(frozen=True)
class X5Config:
    """Explicit controller configuration for one ARX X5 dual-arm pair."""

    left_can: str = "can3"
    right_can: str = "can1"
    hz: float = 120.0
    action_space: str = "abs_qpos"
    model: str = "X5"
    gripper_width_m: float = 0.082
    gripper_open_readout: float = -3.4
    left_gripper_open_readout: Optional[float] = None
    right_gripper_open_readout: Optional[float] = None
    gripper_command_margin_m: float = 0.002
    gripper_command_bias_m: float = -0.005
    background_send_recv: bool = True
    gravity_compensation: bool = True
    preview_time_s: float = 0.08
    damping_kd_scale: float = 0.1
    position_kp_scale: float = 1.0
    position_kd_scale: float = 1.0
    gripper_kp_scale: float = 1.0
    gripper_kd_scale: float = 1.0
    log_level: str = "INFO"

    def __post_init__(self) -> None:
        if not self.left_can or not self.right_can:
            raise ValueError("X5 CAN interface names cannot be empty")
        if self.left_can == self.right_can:
            raise ValueError("X5 left and right CAN interfaces must be different")
        if self.action_space not in X5_ACTION_SPACES:
            raise ValueError(f"X5 action_space must be one of {X5_ACTION_SPACES}")
        if not self.model:
            raise ValueError("X5 model cannot be empty")
        if not math.isfinite(self.hz) or self.hz <= 0:
            raise ValueError("X5 hz must be finite and positive")
        if not math.isfinite(self.gripper_width_m) or self.gripper_width_m <= 0:
            raise ValueError("X5 gripper_width_m must be finite and positive")
        if (
            not math.isfinite(self.gripper_command_margin_m)
            or self.gripper_command_margin_m < 0
            or self.gripper_command_margin_m >= self.gripper_width_m
        ):
            raise ValueError("X5 gripper command margin must be within the gripper width")
        if not math.isfinite(self.gripper_command_bias_m):
            raise ValueError("X5 gripper command bias must be finite")
        if not math.isfinite(self.preview_time_s) or self.preview_time_s <= 0:
            raise ValueError("X5 preview_time_s must be finite and positive")
        scales = (
            self.damping_kd_scale,
            self.position_kp_scale,
            self.position_kd_scale,
            self.gripper_kp_scale,
            self.gripper_kd_scale,
        )
        if not all(math.isfinite(value) and value > 0 for value in scales):
            raise ValueError("X5 gain scales must be finite and positive")
        self.gripper_open_readouts()

    def gripper_open_readouts(self) -> Tuple[float, float]:
        left = self.left_gripper_open_readout
        right = self.right_gripper_open_readout
        if left is None and right is None:
            fallback = float(self.gripper_open_readout)
            values = (fallback, fallback)
        elif left is None or right is None:
            raise ValueError(
                "left_gripper_open_readout and right_gripper_open_readout must be provided together"
            )
        else:
            values = (float(left), float(right))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("X5 gripper open readouts must be finite")
        return values

    @property
    def controller_type(self) -> str:
        return controller_type_for_action_space(self.action_space)

    @property
    def command_kind(self) -> str:
        return command_kind_for_action_space(self.action_space)


@dataclass(frozen=True)
class X5State:
    """One synchronized-enough dual-arm snapshot in the public 14D convention."""

    qpos: np.ndarray
    qvel: np.ndarray
    effort: np.ndarray
    eef_pose_6d: np.ndarray
    eef_wxyz: np.ndarray
    timestamp_s: float


SdkLoader = Callable[[], Any]


def _load_x5_sdk() -> Any:
    try:
        import arx5_interface as sdk
    except ModuleNotFoundError as exc:
        raise X5SdkUnavailable(
            "arx5_interface is required for X5 hardware. Initialize "
            "third_party/arx5-sdk and run scripts/install_x5_sdk.sh."
        ) from exc
    except (ImportError, OSError) as exc:
        raise X5SdkUnavailable(
            f"arx5_interface was found but a native dependency could not be loaded: {exc}"
        ) from exc
    return sdk


class X5DualArm:
    """Atomic X5 pair lifecycle with fail-closed motion and mode commands.

    Motion and mode-changing calls require a complete :class:`MotionGate`.
    ``safe_stop()`` and ``close()`` are deliberately exempt: they only request
    controller damping and release resources, never a homing trajectory.
    """

    sides = ("left", "right")
    arm_dof = 6
    action_dim = 14

    def __init__(
        self,
        *,
        config: Optional[X5Config] = None,
        gate: Optional[MotionGate] = None,
        sdk_loader: Optional[SdkLoader] = None,
    ) -> None:
        self.config = X5Config() if config is None else config
        self.gate = MotionGate() if gate is None else gate
        self._sdk_loader = _load_x5_sdk if sdk_loader is None else sdk_loader
        self._sdk: Any = None
        self._controllers: Dict[str, Any] = {}
        self._robot_configs: Dict[str, Any] = {}
        self._controller_configs: Dict[str, Any] = {}
        self._fk_solvers: Dict[str, Any] = {}
        self._side_executor: Optional[ThreadPoolExecutor] = None
        self._closed = True
        self.control_mode = ""

    @property
    def is_connected(self) -> bool:
        return not self._closed and set(self._controllers) == set(self.sides)

    @property
    def sdk_loaded(self) -> bool:
        return self._sdk is not None

    def connect(self) -> X5DualArm:
        if self.is_connected:
            return self
        if self._controllers:
            raise RuntimeError("X5 controller is only partially connected; close it first")

        sdk = self._sdk_loader()
        self._sdk = sdk
        self._side_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="arx_x5")
        self._closed = False
        self.control_mode = "unknown"
        left_readout, right_readout = self.config.gripper_open_readouts()

        try:
            for side in self.sides:
                robot_cfg = sdk.RobotConfigFactory.get_instance().get_config(self.config.model)
                robot_cfg.gripper_open_readout = left_readout if side == "left" else right_readout
                robot_cfg.gripper_width = float(self.config.gripper_width_m)
                controller_cfg = sdk.ControllerConfigFactory.get_instance().get_config(
                    self.config.controller_type,
                    robot_cfg.joint_dof,
                )
                controller_cfg.background_send_recv = bool(self.config.background_send_recv)
                controller_cfg.gravity_compensation = bool(self.config.gravity_compensation)
                controller_cfg.default_preview_time = float(self.config.preview_time_s)
                can_name = self.config.left_can if side == "left" else self.config.right_can

                if self.config.controller_type == "joint_controller":
                    controller = sdk.Arx5JointController(robot_cfg, controller_cfg, can_name)
                elif self.config.controller_type == "cartesian_controller":
                    controller = sdk.Arx5CartesianController(robot_cfg, controller_cfg, can_name)
                else:  # pragma: no cover - guarded by X5Config
                    raise ValueError(
                        f"unsupported X5 controller type {self.config.controller_type!r}"
                    )
                controller.set_log_level(getattr(sdk.LogLevel, self.config.log_level))
                self._controllers[side] = controller
                self._robot_configs[side] = robot_cfg
                self._controller_configs[side] = controller_cfg
                self._fk_solvers[side] = sdk.Arx5Solver(
                    robot_cfg.urdf_path,
                    robot_cfg.joint_dof,
                    robot_cfg.joint_pos_min,
                    robot_cfg.joint_pos_max,
                    robot_cfg.base_link_name,
                    robot_cfg.eef_link_name,
                    robot_cfg.gravity_vector,
                )
            # A successful connection must have a deterministic, non-trajectory
            # control state even when the motion gate remains closed.
            self.safe_stop()
        except Exception:
            try:
                self.close()
            except Exception:
                pass
            raise
        return self

    def read_state(self) -> X5State:
        self._require_connected()

        def read_side(side: str) -> Mapping[str, np.ndarray]:
            controller = self._controllers[side]
            state = controller.get_joint_state()
            arm_pos64 = np.asarray(state.pos(), dtype=np.float64).reshape(self.arm_dof).copy()
            arm_pos = arm_pos64.astype(np.float32)
            arm_vel = np.asarray(state.vel(), dtype=np.float32).reshape(self.arm_dof).copy()
            arm_effort = np.asarray(state.torque(), dtype=np.float32).reshape(self.arm_dof).copy()
            gripper = float(np.clip(float(state.gripper_pos), 0.0, self.config.gripper_width_m))
            eef_pose_6d = self._eef_pose_6d(side, controller, arm_pos64)
            if np.all(np.isfinite(eef_pose_6d)):
                eef_wxyz = np.concatenate(
                    [eef_pose_6d[:3], _rpy_to_quaternion_wxyz(*eef_pose_6d[3:])]
                ).astype(np.float32)
            else:
                eef_wxyz = np.full(7, np.nan, dtype=np.float32)
            return {
                "qpos": np.concatenate([arm_pos, [gripper]]).astype(np.float32),
                "qvel": np.concatenate(
                    [arm_vel, [_optional_float(getattr(state, "gripper_vel", None))]]
                ).astype(np.float32),
                "effort": np.concatenate(
                    [arm_effort, [_optional_float(getattr(state, "gripper_torque", None))]]
                ).astype(np.float32),
                "eef_pose_6d": eef_pose_6d,
                "eef_wxyz": eef_wxyz,
            }

        sides = self._run_sides(read_side)
        return X5State(
            qpos=np.concatenate([sides[side]["qpos"] for side in self.sides]).astype(np.float32),
            qvel=np.concatenate([sides[side]["qvel"] for side in self.sides]).astype(np.float32),
            effort=np.concatenate([sides[side]["effort"] for side in self.sides]).astype(
                np.float32
            ),
            eef_pose_6d=np.concatenate([sides[side]["eef_pose_6d"] for side in self.sides]).astype(
                np.float32
            ),
            eef_wxyz=np.concatenate([sides[side]["eef_wxyz"] for side in self.sides]).astype(
                np.float32
            ),
            timestamp_s=time.time(),
        )

    def send_action(self, action_space: str, action: Iterable[float]) -> None:
        requested = str(action_space)
        if requested != self.config.action_space:
            raise ValueError(
                f"X5 action_space={requested!r} does not match configured "
                f"action_space={self.config.action_space!r}"
            )
        command = _command_array(action, name=f"{requested} command")
        if requested == "abs_qpos":
            self.send_joint_positions(command)
        elif requested == "delta_qpos":
            self.send_joint_positions(self.read_state().qpos + command)
        elif requested == "abs_eef":
            self.send_eef_positions(command)
        elif requested == "delta_eef":
            self.send_eef_positions(_eef_action_from_state(self.read_state()) + command)
        else:  # pragma: no cover - guarded by X5Config
            raise ValueError(f"unknown X5 action_space {requested!r}")

    def set_damping_mode(self) -> None:
        self._require_connected()
        self.gate.require_motion()

        def set_side(side: str) -> None:
            controller = self._controllers[side]
            controller.set_to_damping()
            gain = controller.get_gain()
            gain.kd()[:] *= float(self.config.damping_kd_scale)
            gain.gripper_kd *= float(self.config.damping_kd_scale)
            controller.set_gain(gain)

        try:
            self._run_sides(set_side)
        except Exception:
            self._safe_stop_after_failure()
            raise
        self.control_mode = "damping"

    def set_position_mode(self) -> None:
        self._require_connected()
        self.gate.require_motion()
        sdk = self._sdk

        def set_side(side: str) -> None:
            controller = self._controllers[side]
            state = controller.get_joint_state()
            if self.config.command_kind == "qpos":
                command = sdk.JointState(self._robot_configs[side].joint_dof)
                command.pos()[:] = np.asarray(state.pos(), dtype=np.float64)
                command.gripper_pos = self._clip_gripper_command(float(state.gripper_pos))
                controller.set_joint_cmd(command)
            else:
                pose_6d = self._eef_pose_6d(
                    side,
                    controller,
                    np.asarray(state.pos(), dtype=np.float64),
                )
                _check_finite("current X5 eef pose", pose_6d)
                command = sdk.EEFState(
                    np.asarray(pose_6d, dtype=np.float64),
                    self._clip_gripper_command(float(state.gripper_pos)),
                )
                controller.set_eef_cmd(command)

            controller_cfg = self._controller_configs[side]
            gain = sdk.Gain(self._robot_configs[side].joint_dof)
            gain.kp()[:] = (
                np.asarray(controller_cfg.default_kp, dtype=np.float64)
                * self.config.position_kp_scale
            )
            gain.kd()[:] = (
                np.asarray(controller_cfg.default_kd, dtype=np.float64)
                * self.config.position_kd_scale
            )
            gain.gripper_kp = (
                float(controller_cfg.default_gripper_kp) * self.config.gripper_kp_scale
            )
            gain.gripper_kd = (
                float(controller_cfg.default_gripper_kd) * self.config.gripper_kd_scale
            )
            controller.set_gain(gain)

        try:
            self._run_sides(set_side)
        except Exception:
            self._safe_stop_after_failure()
            raise
        self.control_mode = "position"

    def reset_home(self) -> None:
        self._require_connected()
        self.gate.require_motion()
        try:
            self._run_sides(lambda side: self._controllers[side].reset_to_home())
        except Exception:
            self._safe_stop_after_failure()
            raise
        self.safe_stop()

    def send_joint_positions(self, qpos: Iterable[float]) -> np.ndarray:
        self._require_position_mode("qpos")
        self.gate.require_motion()
        command = _command_array(qpos, name="X5 qpos command")

        def send_side(side: str) -> None:
            target = _side_vector(command, side)
            joint_command = self._sdk.JointState(self.arm_dof)
            joint_command.pos()[:] = target[: self.arm_dof]
            joint_command.gripper_pos = self._gripper_command(float(target[6]))
            self._controllers[side].set_joint_cmd(joint_command)

        try:
            self._run_sides(send_side)
        except Exception:
            self._safe_stop_after_failure()
            raise
        return command.copy()

    def send_eef_positions(self, eef_action: Iterable[float]) -> np.ndarray:
        self._require_position_mode("eef")
        self.gate.require_motion()
        command = _command_array(eef_action, name="X5 eef command")

        def send_side(side: str) -> None:
            target = _side_vector(command, side).copy()
            target[6] = self._gripper_command(float(target[6]))
            eef_command = self._sdk.EEFState(
                np.asarray(target[:6], dtype=np.float64),
                float(target[6]),
            )
            self._controllers[side].set_eef_cmd(eef_command)

        try:
            self._run_sides(send_side)
        except Exception:
            self._safe_stop_after_failure()
            raise
        return command.copy()

    def safe_stop(self) -> None:
        """Request damping on every connected side without initiating a trajectory."""

        if not self._controllers:
            return
        sides = tuple(side for side in self.sides if side in self._controllers)
        self._run_sides(
            lambda side: self._controllers[side].set_to_damping(),
            sides=sides,
        )
        self.control_mode = "damping"

    def close(self) -> None:
        if self._closed and not self._controllers:
            return

        first_error: Optional[BaseException] = None
        try:
            self.safe_stop()
        except BaseException as exc:  # cleanup must continue for both sides
            first_error = exc

        for side in reversed(self.sides):
            controller = self._controllers.get(side)
            if controller is None:
                continue
            try:
                _close_resource(controller)
            except BaseException as exc:
                if first_error is None:
                    first_error = exc

        self._controllers.clear()
        self._robot_configs.clear()
        self._controller_configs.clear()
        self._fk_solvers.clear()
        executor, self._side_executor = self._side_executor, None
        if executor is not None:
            try:
                executor.shutdown(wait=True)
            except BaseException as exc:
                if first_error is None:
                    first_error = exc
        self._sdk = None
        self._closed = True
        self.control_mode = ""

        if first_error is not None:
            raise first_error

    def status(self) -> Mapping[str, Any]:
        sdk_version = ""
        if self._sdk is not None:
            sdk_version = str(getattr(self._sdk, "__version__", ""))
        return {
            "type": type(self).__name__,
            "connected": self.is_connected,
            "sdk_loaded": self.sdk_loaded,
            "sdk_version": sdk_version,
            "action_space": self.config.action_space,
            "command_kind": self.config.command_kind,
            "controller_type": self.config.controller_type,
            "control_mode": self.control_mode,
            "hz": self.config.hz,
            "preview_time_s": self.config.preview_time_s,
            "left_can": self.config.left_can,
            "right_can": self.config.right_can,
            "motion_gate_dry_run": self.gate.is_dry_run,
        }

    def __enter__(self) -> X5DualArm:
        return self.connect()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise RuntimeError("X5 dual-arm controller is not connected")

    def _require_position_mode(self, command_kind: str) -> None:
        self._require_connected()
        if self.control_mode != "position":
            raise RuntimeError("X5 is not in position mode")
        if self.config.command_kind != command_kind:
            raise RuntimeError(
                f"X5 action_space={self.config.action_space!r} uses "
                f"{self.config.command_kind!r} commands, not {command_kind!r}"
            )

    def _eef_pose_6d(
        self,
        side: str,
        controller: Any,
        joint_pos: np.ndarray,
    ) -> np.ndarray:
        try:
            return np.asarray(controller.get_eef_state().pose_6d(), dtype=np.float32).reshape(6)
        except Exception:
            pass
        solver = self._fk_solvers.get(side)
        if solver is not None:
            try:
                return np.asarray(
                    solver.forward_kinematics(
                        np.asarray(joint_pos, dtype=np.float64).reshape(self.arm_dof)
                    ),
                    dtype=np.float32,
                ).reshape(6)
            except Exception:
                pass
        return np.full(6, np.nan, dtype=np.float32)

    def _clip_gripper_command(self, gripper_pos: float) -> float:
        safe_max = self.config.gripper_width_m - self.config.gripper_command_margin_m
        return float(np.clip(gripper_pos, 0.0, safe_max))

    def _gripper_command(self, gripper_pos: float) -> float:
        return self._clip_gripper_command(gripper_pos + self.config.gripper_command_bias_m)

    def _safe_stop_after_failure(self) -> None:
        try:
            self.safe_stop()
        except Exception:
            pass

    def _run_sides(
        self,
        function: Callable[[str], Any],
        *,
        sides: Optional[Sequence[str]] = None,
    ) -> Dict[str, Any]:
        selected = tuple(self.sides if sides is None else sides)
        executor = self._side_executor
        if executor is None:
            return {side: function(side) for side in selected}

        futures = {side: executor.submit(function, side) for side in selected}
        results: Dict[str, Any] = {}
        first_error: Optional[BaseException] = None
        for side in selected:
            try:
                results[side] = futures[side].result()
            except BaseException as exc:
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error
        return results


def controller_type_for_action_space(action_space: str) -> str:
    if action_space in X5_QPOS_ACTION_SPACES:
        return "joint_controller"
    if action_space in X5_EEF_ACTION_SPACES:
        return "cartesian_controller"
    raise ValueError(f"unknown X5 action_space {action_space!r}; expected {X5_ACTION_SPACES}")


def command_kind_for_action_space(action_space: str) -> str:
    if action_space in X5_QPOS_ACTION_SPACES:
        return "qpos"
    if action_space in X5_EEF_ACTION_SPACES:
        return "eef"
    raise ValueError(f"unknown X5 action_space {action_space!r}; expected {X5_ACTION_SPACES}")


def _command_array(values: Iterable[float], *, name: str) -> np.ndarray:
    if isinstance(values, np.ndarray):
        flattened = values.reshape(-1)
    else:
        flattened = np.asarray(tuple(values), dtype=np.float64).reshape(-1)
    return np.asarray(
        finite_vector(flattened, size=14, name=name),
        dtype=np.float32,
    )


def _side_vector(values: np.ndarray, side: str) -> np.ndarray:
    start = 0 if side == "left" else 7
    return values[start : start + 7]


def _eef_action_from_state(state: X5State) -> np.ndarray:
    return np.concatenate(
        [
            state.eef_pose_6d[:6],
            [state.qpos[6]],
            state.eef_pose_6d[6:12],
            [state.qpos[13]],
        ]
    ).astype(np.float32)


def _check_finite(name: str, values: np.ndarray) -> None:
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{name} contains NaN or inf values")


def _optional_float(value: Any) -> float:
    return np.nan if value is None else float(value)


def _rpy_to_quaternion_wxyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(float(roll) * 0.5)
    sr = math.sin(float(roll) * 0.5)
    cp = math.cos(float(pitch) * 0.5)
    sp = math.sin(float(pitch) * 0.5)
    cy = math.cos(float(yaw) * 0.5)
    sy = math.sin(float(yaw) * 0.5)
    quaternion = np.asarray(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float32,
    )
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0 or not math.isfinite(norm):
        raise ValueError("X5 eef quaternion normalization failed")
    return quaternion / norm


def _close_resource(resource: Any) -> None:
    for name in ("close", "disconnect", "cleanup"):
        method = getattr(resource, name, None)
        if callable(method):
            method()
            return
