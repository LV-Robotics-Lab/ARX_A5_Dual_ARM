"""Safety-gated ARX A5/X5 dual-arm wrapper."""

from importlib import import_module

from .config import ArmEndpoint, ArxConfig, CameraEndpoint
from .safety import MotionGate, SafetyGateError
from .sdk import ArmSnapshot, ArxArm, ArxDualArm, ArxSdkUnavailable

_X5_EXPORTS = frozenset({"X5Config", "X5DualArm", "X5SdkUnavailable", "X5State"})

__all__ = [
    "ArmEndpoint",
    "ArmSnapshot",
    "ArxArm",
    "ArxConfig",
    "ArxDualArm",
    "ArxSdkUnavailable",
    "CameraEndpoint",
    "MotionGate",
    "SafetyGateError",
    "X5Config",
    "X5DualArm",
    "X5SdkUnavailable",
    "X5State",
]

__version__ = "0.2.0"


def __getattr__(name: str):
    """Load the NumPy-backed X5 controller layer only when it is requested."""

    if name in _X5_EXPORTS:
        module = import_module(".x5", __name__)
        return getattr(module, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals()).union(_X5_EXPORTS))
