#!/usr/bin/env python3
"""Aggregate stillness failures across all near-miss episodes.

For the 102 episodes that pass every other SOP gate but fail the trailing-2s
stillness at the accepted 0.10 rad threshold, surface a histogram of:
  - which arm fails most often (left vs right)
  - which joint index is the worst offender per arm
  - how far over the gate each episode was

Goal: hand operators a short, actionable list like
  "right arm joint 4 is over the gate in 38/102 near-misses; tighten grip
   stability on that joint during the 2s mark to recover ~37% of near-misses."

Source of truth: the verifier's existing per-episode report. No new scans.

Usage:
    ~/miniconda3/envs/robo_ctrl/bin/python \\
        ~/workspace/arx_wrapper/data_collection/stillness_failure_breakdown.py \\
        --report ~/workspace/.verifier_reports/raw_data_full_report.json \\
        --out ~/workspace/.verifier_reports/stillness_failure_breakdown.json
"""

from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median


SCHEMA_VERSION = "drag_demo_stillness_breakdown.v1"
THRESHOLD_RAD = 0.10


def _is_stillness_only_near_miss(ep: dict, threshold_rad: float) -> bool:
    if ep.get("ok"):
        return False
    if ep.get("label") != "success":
        return False
    checks = ep.get("checks", {}) or {}
    if not checks.get("files", {}).get("all_present"):
        return False
    for cam in (checks.get("videos") or {}).values():
        if not cam.get("ok"):
            return False
    for cam in (checks.get("depths") or {}).values():
        if not cam.get("ok"):
            return False
    if not checks.get("state", {}).get("ok"):
        return False
    if not checks.get("eef_pose", {}).get("ok"):
        return False
    if not checks.get("image_timestamps", {}).get("ok"):
        return False
    stillness = checks.get("stillness", {}) or {}
    if stillness.get("ok"):
        return False
    arms = stillness.get("arms", {}) or {}
    any_over = False
    for arm in arms.values():
        rng = arm.get("max_range_rad")
        if rng is None:
            continue
        if rng > threshold_rad:
            any_over = True
    return any_over


def summarize(report: dict, threshold_rad: float) -> dict:
    near_misses = [
        ep for ep in report.get("episodes", [])
        if _is_stillness_only_near_miss(ep, threshold_rad)
    ]
    arm_offender_count = Counter()
    joint_offender_count: dict[str, Counter] = defaultdict(Counter)
    excess_per_arm: dict[str, list[float]] = defaultdict(list)
    per_episode_rows = []

    for ep in near_misses:
        arms = (ep.get("checks", {}) or {}).get("stillness", {}).get("arms", {}) or {}
        offenders = []
        for arm_name, arm in arms.items():
            rng = arm.get("max_range_rad")
            if rng is None or rng <= threshold_rad:
                continue
            offenders.append((arm_name, arm.get("worst_joint_idx"), rng))
            arm_offender_count[arm_name] += 1
            if arm.get("worst_joint_idx") is not None:
                joint_offender_count[arm_name][arm["worst_joint_idx"]] += 1
            excess_per_arm[arm_name].append(rng - threshold_rad)
        per_episode_rows.append({
            "path": ep["path"],
            "offenders": [
                {"arm": a, "worst_joint_idx": j, "max_range_rad": round(r, 6),
                 "excess_rad": round(r - threshold_rad, 6)}
                for (a, j, r) in offenders
            ],
        })

    arm_totals = []
    for arm in ("left_arm", "right_arm"):
        n = arm_offender_count.get(arm, 0)
        excesses = excess_per_arm.get(arm, []) or [0.0]
        joint_hist = sorted(
            joint_offender_count.get(arm, Counter()).items(),
            key=lambda kv: -kv[1],
        )
        arm_totals.append({
            "arm": arm,
            "n_episodes_over_gate": n,
            "median_excess_rad": round(median(excesses), 6) if excesses else 0.0,
            "max_excess_rad": round(max(excesses), 6) if excesses else 0.0,
            "joint_offender_hist": [
                {"joint_idx": j, "n_episodes": c} for j, c in joint_hist
            ],
        })

    return {
        "schema_version": SCHEMA_VERSION,
        "threshold_rad": threshold_rad,
        "report_source": report.get("report_source") or "<inline>",
        "n_near_miss_stillness_only": len(near_misses),
        "per_arm_summary": arm_totals,
        "per_episode": per_episode_rows,
    }


def render_operator_brief(summary: dict) -> str:
    lines = [
        "# Stillness-failure operator brief",
        "",
        f"Threshold: {summary['threshold_rad']:.3f} rad (~{summary['threshold_rad']*180/3.1416:.1f} deg)",
        f"Near-miss episodes (everything passes except trailing 2 s stillness): "
        f"{summary['n_near_miss_stillness_only']}",
        "",
        "## Per-arm offender counts",
        "",
        "| Arm | Episodes over gate | Median excess (rad) | Max excess (rad) |",
        "| --- | --- | --- | --- |",
    ]
    for arm in summary["per_arm_summary"]:
        lines.append(
            f"| {arm['arm']} | {arm['n_episodes_over_gate']} | "
            f"{arm['median_excess_rad']:.4f} | {arm['max_excess_rad']:.4f} |"
        )
    lines += ["", "## Joint-level offender histogram (worst joint per failing episode)", ""]
    for arm in summary["per_arm_summary"]:
        lines.append(f"### {arm['arm']}")
        if not arm["joint_offender_hist"]:
            lines.append("(no offenders)")
            continue
        for row in arm["joint_offender_hist"]:
            lines.append(f"- joint {row['joint_idx']}: {row['n_episodes']} episodes")
        lines.append("")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--report", type=Path, required=True,
                    help="Path to raw_data_full_report.json from sop_episode_verifier")
    ap.add_argument("--threshold-rad", type=float, default=THRESHOLD_RAD)
    ap.add_argument("--out", type=Path, default=None,
                    help="Write JSON summary here (default: stdout)")
    ap.add_argument("--brief", type=Path, default=None,
                    help="Also write a Markdown operator brief")
    args = ap.parse_args(argv)

    with open(args.report) as f:
        report = json.load(f)
    report["report_source"] = str(args.report)
    summary = summarize(report, args.threshold_rad)

    if args.out is None:
        print(json.dumps(summary, indent=2))
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"wrote {args.out}  ({args.out.stat().st_size} bytes)")
    if args.brief:
        args.brief.parent.mkdir(parents=True, exist_ok=True)
        args.brief.write_text(render_operator_brief(summary))
        print(f"wrote {args.brief}  ({args.brief.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
