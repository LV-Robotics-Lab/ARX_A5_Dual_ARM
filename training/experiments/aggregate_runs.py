#!/usr/bin/env python3
"""Aggregate per-run JSONs into a comparison table indexed by arm_id.

Implements step_3 of the comparison_protocol in finetune_comparison_spec.json.
For each arm we surface: latest run_id, final_loss, mean_loss_last_10,
total_time_s, exit_status, data_provenance manifest hashes, system_info.

Win logic from step_4 of the spec: a non-C arm "wins" vs C iff
  (a) action_l1_last2s <= 0.9 * C.action_l1_last2s  (>=10% relative improvement)
  AND
  (b) task_success_rate >= C.task_success_rate + 5 percentage points
Both are offline/online metrics that come from a full run, not the smoke run,
so report N/A when those keys are missing.

Usage:
    ~/miniconda3/envs/robo_ctrl/bin/python \\
        ~/workspace/arx_wrapper/training/experiments/aggregate_runs.py \\
        --runs-dir ~/workspace/.experiment_runs \\
        --out ~/workspace/.experiment_runs/finetune_comparison_report.json \\
        --report ~/workspace/.experiment_runs/finetune_comparison_report.md
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


SCHEMA_VERSION = "drag_demo_finetune_comparison_report.v1"
ARM_ORDER = [
    "A_teleop_only",
    "B_drag_plus_editing",
    "C_raw_drag_only",
    "D_mixed_70_30",
]


def load_runs(runs_dir: Path) -> list[dict[str, Any]]:
    runs = []
    for p in sorted(runs_dir.glob("*.run.json")):
        try:
            with open(p) as f:
                run = json.load(f)
        except Exception as exc:
            runs.append({
                "_load_error": str(exc),
                "_path": str(p),
                "arm_id": None,
                "exit_status": "load_failed",
            })
            continue
        run["_path"] = str(p)
        runs.append(run)
    return runs


def latest_per_arm(runs: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    latest: dict[str, dict[str, Any]] = {}
    for r in runs:
        arm = r.get("arm_id")
        if not arm:
            continue
        prev = latest.get(arm)
        if prev is None or (r.get("started_at", "") > prev.get("started_at", "")):
            latest[arm] = r
    return latest


def per_arm_row(arm_id: str, run: dict[str, Any] | None) -> dict[str, Any]:
    if run is None:
        return {
            "arm_id": arm_id,
            "n_runs": 0,
            "latest_run_id": None,
            "exit_status": "no_runs",
            "final_loss": None,
            "mean_loss_last_10": None,
            "first_loss": None,
            "rel_improvement": None,
            "total_time_s": None,
            "steps_completed": None,
            "action_l1_last2s": None,
            "task_success_rate": None,
            "data_provenance": {},
            "loader_sha256": None,
            "gpu": None,
            "started_at": None,
            "finished_at": None,
        }
    fm = run.get("final_metrics") or {}
    sys_info = run.get("system_info") or {}
    return {
        "arm_id": arm_id,
        "n_runs": 1,
        "latest_run_id": run.get("run_id"),
        "exit_status": run.get("exit_status"),
        "final_loss": fm.get("final_loss"),
        "mean_loss_last_10": fm.get("mean_loss_last_10"),
        "first_loss": fm.get("first_loss"),
        "rel_improvement": fm.get("rel_improvement"),
        "total_time_s": fm.get("total_time_s"),
        "steps_completed": fm.get("steps_completed"),
        "action_l1_last2s": fm.get("action_l1_last2s"),
        "task_success_rate": fm.get("task_success_rate"),
        "data_provenance": run.get("data_provenance") or {},
        "loader_sha256": run.get("loader_sha256"),
        "gpu": sys_info.get("gpu_name") or sys_info.get("platform"),
        "started_at": run.get("started_at"),
        "finished_at": run.get("finished_at"),
    }


def win_vs_baseline(arm_row: dict[str, Any], c_row: dict[str, Any]) -> dict[str, Any]:
    """Spec step_4: win iff action_l1_last2s <= 0.9 * C AND task_success_rate >= C + 5pp."""
    if arm_row["arm_id"] == "C_raw_drag_only":
        return {"verdict": "baseline", "reasons": []}
    if arm_row["n_runs"] == 0 or c_row["n_runs"] == 0:
        return {"verdict": "n_a", "reasons": ["arm or baseline has no runs"]}

    reasons = []
    a_l1 = arm_row.get("action_l1_last2s")
    c_l1 = c_row.get("action_l1_last2s")
    if a_l1 is None or c_l1 is None:
        reasons.append("action_l1_last2s missing on arm or baseline")
        l1_ok = None
    else:
        l1_ok = a_l1 <= 0.9 * c_l1
        reasons.append(
            f"action_l1_last2s arm={a_l1:.4f} baseline={c_l1:.4f} "
            f"(arm <= 0.9*baseline? {l1_ok})"
        )

    a_sr = arm_row.get("task_success_rate")
    c_sr = c_row.get("task_success_rate")
    if a_sr is None or c_sr is None:
        reasons.append("task_success_rate missing on arm or baseline")
        sr_ok = None
    else:
        sr_ok = a_sr >= c_sr + 0.05
        reasons.append(
            f"task_success_rate arm={a_sr:.3f} baseline={c_sr:.3f} "
            f"(arm >= baseline+5pp? {sr_ok})"
        )

    if l1_ok is True and sr_ok is True:
        verdict = "win"
    elif l1_ok is False or sr_ok is False:
        verdict = "loss"
    else:
        verdict = "n_a"
    return {"verdict": verdict, "reasons": reasons}


def render_report(payload: dict[str, Any]) -> str:
    lines = [
        "# Drag-demo fine-tuning comparison report",
        "",
        f"Schema: {payload['schema_version']}  runs_dir: `{payload['runs_dir']}`",
        f"Total run.json files seen: {payload['n_runs_total']}",
        "",
        "## Per-arm latest run",
        "",
        "| arm_id | n_runs | latest_run_id | exit | final_loss | mean_l10 | total_s | gpu |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in payload["per_arm"]:
        fl = f"{row['final_loss']:.4f}" if row.get("final_loss") is not None else "-"
        ml = f"{row['mean_loss_last_10']:.4f}" if row.get("mean_loss_last_10") is not None else "-"
        ts = f"{row['total_time_s']:.2f}" if row.get("total_time_s") is not None else "-"
        lines.append(
            f"| {row['arm_id']} | {row['n_runs']} | "
            f"{row.get('latest_run_id') or '-'} | "
            f"{row.get('exit_status') or '-'} | {fl} | {ml} | {ts} | "
            f"{row.get('gpu') or '-'} |"
        )
    lines += [
        "",
        "## Spec step_4 win condition vs C_raw_drag_only",
        "",
        "An arm wins iff action_l1_last2s <= 0.9 * C AND task_success_rate >= C + 5pp.",
        "Both are end-of-full-run metrics; reports N/A when smoke-only.",
        "",
    ]
    for arm in payload["win_table"]:
        lines.append(f"### {arm['arm_id']} -> {arm['win']['verdict']}")
        for r in arm["win"]["reasons"]:
            lines.append(f"- {r}")
        lines.append("")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs-dir", type=Path,
                    default=Path.home() / "workspace" / ".experiment_runs")
    ap.add_argument("--out", type=Path, default=None,
                    help="Path to write the aggregated JSON (default stdout).")
    ap.add_argument("--report", type=Path, default=None,
                    help="Optional Markdown report path.")
    args = ap.parse_args(argv)

    if not args.runs_dir.is_dir():
        raise SystemExit(f"runs dir does not exist: {args.runs_dir}")

    runs = load_runs(args.runs_dir)
    latest = latest_per_arm(runs)
    per_arm = [per_arm_row(arm, latest.get(arm)) for arm in ARM_ORDER]
    c_row = next(r for r in per_arm if r["arm_id"] == "C_raw_drag_only")
    win_table = [{"arm_id": r["arm_id"], "win": win_vs_baseline(r, c_row)}
                 for r in per_arm]

    payload = {
        "schema_version": SCHEMA_VERSION,
        "runs_dir": str(args.runs_dir),
        "n_runs_total": sum(1 for r in runs if r.get("arm_id")),
        "n_runs_load_failed": sum(1 for r in runs if r.get("exit_status") == "load_failed"),
        "per_arm": per_arm,
        "win_table": win_table,
    }

    if args.out is None:
        print(json.dumps(payload, indent=2, default=str))
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, "w") as f:
            json.dump(payload, f, indent=2, default=str)
        print(f"wrote {args.out}  ({args.out.stat().st_size} bytes)")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(render_report(payload))
        print(f"wrote {args.report}  ({args.report.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
