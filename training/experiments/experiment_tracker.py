#!/usr/bin/env python3
"""Tiny experiment tracker that writes per-run JSON + metrics CSV.

Schema is documented in finetune_comparison_spec.json under
`experiment_tracking`. Stays JSON / CSV so it's grep-able and diff-able
without any external service.

Usage:
    from experiment_tracker import ExperimentRun

    run = ExperimentRun.open(arm_id="C_raw_drag_only", config={...})
    run.log_step(step=10, loss=0.42, lr=3e-4, time_per_step_s=0.05)
    run.finalize(final_metrics={"action_mse": 0.13},
                 exit_status="smoke",
                 stdout_tail="...")
    print(run.run_json_path)
"""

from __future__ import annotations

import csv
import datetime as _dt
import hashlib
import json
import os
import platform
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

RUNS_DIR_DEFAULT = Path.home() / "workspace" / ".experiment_runs"


def _now_iso() -> str:
    return _dt.datetime.utcnow().isoformat(timespec="seconds") + "Z"


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _system_info() -> dict[str, Any]:
    info = {
        "python": sys.version.split()[0],
        "platform": platform.platform(),
    }
    try:
        import torch  # type: ignore
        info["torch"] = torch.__version__
        info["cuda_available"] = bool(torch.cuda.is_available())
        if torch.cuda.is_available():
            info["gpu_name"] = torch.cuda.get_device_name(0)
            info["cuda_version"] = torch.version.cuda
    except Exception as exc:  # pragma: no cover
        info["torch_error"] = str(exc)
    return info


@dataclass
class ExperimentRun:
    run_id: str
    arm_id: str
    runs_dir: Path
    started_at: str
    config: dict[str, Any]
    metrics: list[dict[str, Any]] = field(default_factory=list)
    data_provenance: dict[str, str] = field(default_factory=dict)
    loader_sha256: str | None = None
    final_metrics: dict[str, Any] = field(default_factory=dict)
    finished_at: str | None = None
    exit_status: str = "running"
    system: dict[str, Any] = field(default_factory=dict)
    stdout_tail: str | None = None

    @classmethod
    def open(cls, *, arm_id: str, config: dict[str, Any],
             runs_dir: Path | str | None = None,
             data_provenance: dict[str, str] | None = None,
             loader_path: Path | str | None = None) -> "ExperimentRun":
        runs_dir = Path(runs_dir) if runs_dir else RUNS_DIR_DEFAULT
        runs_dir.mkdir(parents=True, exist_ok=True)
        run_id = (f"{arm_id}_"
                  + _dt.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ"))
        run = cls(
            run_id=run_id,
            arm_id=arm_id,
            runs_dir=runs_dir,
            started_at=_now_iso(),
            config=dict(config),
            data_provenance=dict(data_provenance or {}),
            loader_sha256=_sha256(Path(loader_path)) if loader_path else None,
            system=_system_info(),
        )
        run._csv_path.parent.mkdir(parents=True, exist_ok=True)
        with open(run._csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["step", "loss", "lr", "time_per_step_s"])
        run._write_run_json()
        return run

    @property
    def _csv_path(self) -> Path:
        return self.runs_dir / f"{self.run_id}.metrics.csv"

    @property
    def run_json_path(self) -> Path:
        return self.runs_dir / f"{self.run_id}.run.json"

    def log_step(self, *, step: int, loss: float, lr: float,
                 time_per_step_s: float) -> None:
        row = {"step": int(step), "loss": float(loss), "lr": float(lr),
               "time_per_step_s": float(time_per_step_s)}
        self.metrics.append(row)
        with open(self._csv_path, "a", newline="") as f:
            csv.writer(f).writerow(
                [row["step"], row["loss"], row["lr"], row["time_per_step_s"]])

    def finalize(self, *, final_metrics: dict[str, Any],
                 exit_status: str = "completed",
                 stdout_tail: str | None = None) -> None:
        self.final_metrics = dict(final_metrics)
        self.exit_status = exit_status
        self.finished_at = _now_iso()
        self.stdout_tail = stdout_tail
        self._write_run_json()

    def _write_run_json(self) -> None:
        payload = {
            "schema_version": "drag_demo_experiment_run.v1",
            "run_id": self.run_id,
            "arm_id": self.arm_id,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "exit_status": self.exit_status,
            "config": self.config,
            "data_provenance": self.data_provenance,
            "loader_sha256": self.loader_sha256,
            "metrics_per_step": self.metrics,
            "final_metrics": self.final_metrics,
            "system_info": self.system,
            "stdout_tail": self.stdout_tail,
        }
        self.run_json_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.run_json_path, "w") as f:
            json.dump(payload, f, indent=2, default=str)
