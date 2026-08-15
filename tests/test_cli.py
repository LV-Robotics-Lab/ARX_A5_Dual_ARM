import json
from pathlib import Path

from arx_wrapper import cli
from arx_wrapper.doctor import CheckResult


def test_config_cli_prints_json(monkeypatch, capsys, tmp_path: Path) -> None:
    monkeypatch.setenv("ARX_VENDOR_ROOT", str(tmp_path))

    assert cli.config_main([]) == 0
    output = json.loads(capsys.readouterr().out)
    assert output["arms"][0]["can_interface"] == "can1"


def test_doctor_strict_controls_exit_status(monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        cli,
        "run_doctor",
        lambda _: [CheckResult("one", True, "ok"), CheckResult("two", False, "missing")],
    )

    assert cli.doctor_main([]) == 0
    assert "[WARN] two: missing" in capsys.readouterr().out
    assert cli.doctor_main(["--strict", "--json"]) == 1
    assert json.loads(capsys.readouterr().out)[1]["name"] == "two"
