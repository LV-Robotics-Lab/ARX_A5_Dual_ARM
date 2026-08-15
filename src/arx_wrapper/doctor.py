"""Read-only host, vendor checkout, and SocketCAN diagnostics."""

from __future__ import annotations

import platform
import shutil
import subprocess
from dataclasses import asdict, dataclass
from typing import Callable, List, Sequence

from .config import ArxConfig


@dataclass(frozen=True)
class CheckResult:
    name: str
    ok: bool
    detail: str

    def as_dict(self) -> dict:
        return asdict(self)


Runner = Callable[[Sequence[str]], subprocess.CompletedProcess]


def _run(command: Sequence[str]) -> subprocess.CompletedProcess:
    return subprocess.run(command, check=False, capture_output=True, text=True)


def inspect_commands(
    commands: Sequence[str] = ("git", "python3", "cmake", "ip", "roscore"),
) -> List[CheckResult]:
    return [
        CheckResult(
            command, shutil.which(command) is not None, shutil.which(command) or "not found"
        )
        for command in commands
    ]


def inspect_vendor_checkout(config: ArxConfig) -> List[CheckResult]:
    root = config.vendor_root
    checks = [
        ("vendor-root", root / "A5" / "bimanual", "vendored A5 source"),
        ("vendor-urdf", root / "A5" / "bimanual" / "script" / config.arms[0].urdf_name, "URDF"),
    ]
    results = [
        CheckResult(name, path.exists(), f"{description}: {path}")
        for name, path, description in checks
    ]
    library = root / "A5" / "bimanual" / "lib" / "arx_r5_src" / "libarx_r5a_src.so"
    is_linux = platform.system() == "Linux"
    results.append(
        CheckResult(
            "vendor-library",
            library.exists() and is_linux,
            f"Linux vendor shared library: {library}; host={platform.system()}",
        )
    )
    extensions = tuple((root / "A5" / "bimanual" / "api").glob("**/arx_r5_python*.so"))
    results.append(
        CheckResult(
            "vendor-python-extension",
            bool(extensions) and is_linux,
            "built ARX Python extension: "
            + (", ".join(str(path) for path in extensions) if extensions else "not found"),
        )
    )
    return results


def inspect_can_interfaces(
    config: ArxConfig,
    *,
    runner: Runner = _run,
) -> List[CheckResult]:
    if shutil.which("ip") is None and runner is _run:
        return [CheckResult("ip", False, "iproute2 is not installed")]
    results = []
    for endpoint in config.arms:
        completed = runner(["ip", "-details", "link", "show", endpoint.can_interface])
        output = f"{completed.stdout}\n{completed.stderr}".strip()
        if completed.returncode != 0:
            results.append(CheckResult(endpoint.side, False, f"missing {endpoint.can_interface}"))
            continue
        is_up = "state UP" in output or "<UP," in output or ",UP>" in output
        state = "UP" if is_up else "DOWN"
        results.append(
            CheckResult(endpoint.side, is_up, f"{endpoint.can_interface}: state={state}")
        )
    return results


def inspect_camera_config(config: ArxConfig) -> List[CheckResult]:
    expected = {"cam_top", "cam_left_wrist", "cam_right_wrist"}
    configured = {camera.name for camera in config.cameras}
    missing = sorted(expected - configured)
    detail = ", ".join(f"{camera.name}={camera.serial}" for camera in config.cameras)
    if missing:
        detail = (
            f"{detail}; missing: {', '.join(missing)}"
            if detail
            else f"missing: {', '.join(missing)}"
        )
    return [CheckResult("camera-bindings", not missing, detail)]


def run_doctor(config: ArxConfig) -> List[CheckResult]:
    return [
        *inspect_commands(),
        *inspect_vendor_checkout(config),
        *inspect_can_interfaces(config),
        *inspect_camera_config(config),
    ]
