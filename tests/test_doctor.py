import subprocess
from pathlib import Path

from arx_wrapper.config import ArxConfig
from arx_wrapper.doctor import (
    inspect_camera_config,
    inspect_can_interfaces,
    inspect_vendor_checkout,
)


def test_can_diagnostics_are_read_only_and_report_state(tmp_path: Path) -> None:
    commands = []

    def runner(command):
        commands.append(command)
        output = "4: can1: <NOARP,UP,LOWER_UP> state UP" if command[-1] == "can1" else ""
        return subprocess.CompletedProcess(command, 0 if output else 1, output, "")

    config = ArxConfig.from_env({"ARX_VENDOR_ROOT": str(tmp_path)})
    results = inspect_can_interfaces(config, runner=runner)

    assert commands == [
        ["ip", "-details", "link", "show", "can1"],
        ["ip", "-details", "link", "show", "can3"],
    ]
    assert results[0].ok
    assert not results[1].ok


def test_vendor_diagnostics_do_not_import_binary(tmp_path: Path) -> None:
    config = ArxConfig.from_env({"ARX_VENDOR_ROOT": str(tmp_path)})
    results = inspect_vendor_checkout(config)

    assert all(not result.ok for result in results)
    assert any("libarx_r5a_src.so" in result.detail for result in results)


def test_camera_diagnostics_warn_when_binding_is_disabled(tmp_path: Path) -> None:
    config = ArxConfig.from_env(
        {
            "ARX_VENDOR_ROOT": str(tmp_path),
            "ARX_CAM_TOP_SERIAL": "",
        }
    )

    result = inspect_camera_config(config)[0]
    assert not result.ok
    assert "cam_top" in result.detail
