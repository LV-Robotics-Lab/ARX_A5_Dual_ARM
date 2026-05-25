#!/usr/bin/env python3
"""Derive the initial-batch manifest from a verifier JSON report.

Reads a `raw_data_full_report.json` (produced by sop_episode_verifier.py) and
writes `initial_batch_manifest.json` that a training loader can consume:

    {
        "schema_version": "drag_demo_initial_batch.v1",
        "generated_at": "2026-05-23T...",
        "verifier_sha256": "0507afe8...",
        "demo_target": 100,
        "n_counted_toward_target": 19,
        "delta_to_target": 81,
        "by_dataset_counted": { "egg_to_bowl": 16, "egg_scoop_v1": 3, ... },
        "stillness_threshold_rad_used": 0.10,
        "eval_tier_strict_threshold_rad": 0.02,
        "sop_gates": [...],
        "counted_episodes": [ { "path", "files": {...}, ... }, ... ],
        "eval_tier_strict_episodes": [...],
        "near_miss_stillness_only": [...],
        "file_or_stream_fail": [...],
        "onsite_collection_brief": "..."
    }

Loader contract: every entry in `counted_episodes` is a folder whose 9 files
listed under `files.*` exist on disk at runtime. See `load_initial_batch.py`
for a smoke test.

Usage:
    python build_initial_batch_manifest.py \\
        --report ~/workspace/.verifier_reports/raw_data_full_report.json \\
        --out    ~/workspace/.verifier_reports/initial_batch_manifest.json
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
from pathlib import Path
from typing import Any

REQUIRED_FILES = (
    "cam_top_rgb.mp4",
    "cam_top_depth.h5",
    "cam_left_wrist_rgb.mp4",
    "cam_left_wrist_depth.h5",
    "cam_right_wrist_rgb.mp4",
    "cam_right_wrist_depth.h5",
    "eef_pose.pkl",
    "image_timestamps.pkl",
    "state.pkl",
)
DEFAULT_VERIFIER = (Path(__file__).resolve().parent
                    / "sop_episode_verifier.py")


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _file_map(ep_path: Path) -> dict[str, str]:
    return {name: str(ep_path / name) for name in REQUIRED_FILES}


def _summarize(ep: dict[str, Any]) -> dict[str, Any]:
    arms = (ep.get("checks", {}).get("stillness", {}) or {}).get("arms", {}) or {}
    files = ep["checks"]["files"]
    return {
        "dataset": ep["dataset"],
        "name": ep["name"],
        "label": ep["label"],
        "path": ep["path"],
        "duration_s": ep.get("duration_s"),
        "left_max_range_rad": arms.get("left_arm", {}).get("max_range_rad"),
        "right_max_range_rad": arms.get("right_arm", {}).get("max_range_rad"),
        "all_files_present": files.get("all_present"),
        "missing_files": files.get("missing", []),
        "files": _file_map(Path(ep["path"])),
        "ok": ep["ok"],
        "reasons": ep.get("reasons", []),
    }


def build_manifest(report_path: Path, verifier_path: Path,
                   eval_tier_threshold_rad: float = 0.02) -> dict[str, Any]:
    report = json.loads(report_path.read_text())
    cfg = report["summary"]["config"]
    eps = report["episodes"]
    threshold_used = cfg["stillness_max_range_rad"]

    counted, near_miss, file_or_stream_fail = [], [], []
    eval_tier = []
    for e in eps:
        if e["label"] != "success":
            continue
        files_ok = e["checks"]["files"]["all_present"]
        streams_ok = (all(v.get("ok") for v in e["checks"]["videos"].values())
                      and all(v.get("ok") for v in e["checks"]["depths"].values()))
        pkls_ok = (e["checks"]["state"].get("ok")
                   and e["checks"]["eef_pose"].get("ok")
                   and e["checks"]["image_timestamps"].get("ok"))
        if e["ok"]:
            counted.append(e)
        elif files_ok and streams_ok and pkls_ok \
                and not e["checks"]["stillness"].get("ok"):
            near_miss.append(e)
        else:
            file_or_stream_fail.append(e)

        # Eval tier — stricter than the threshold used. An episode qualifies
        # for the eval tier if (a) the file/stream/pkl checks pass and (b) the
        # trailing-window joint range stays under eval_tier_threshold_rad on
        # both arms. This stays independent of whichever threshold the verifier
        # ran under.
        arms = (e["checks"]["stillness"].get("arms") or {})
        if files_ok and streams_ok and pkls_ok and arms:
            lmax = arms.get("left_arm", {}).get("max_range_rad")
            rmax = arms.get("right_arm", {}).get("max_range_rad")
            if (lmax is not None and rmax is not None
                    and lmax <= eval_tier_threshold_rad
                    and rmax <= eval_tier_threshold_rad):
                eval_tier.append(e)

    by_dataset_counted: dict[str, int] = {}
    for e in counted:
        by_dataset_counted[e["dataset"]] = by_dataset_counted.get(
            e["dataset"], 0) + 1

    delta = 100 - len(counted)

    onsite_brief = (
        "Onsite collection gap: %d more SOP-compliant episodes needed.\n"
        "SOP per task description: right gripper lifts pan spatula with egg "
        "mold; moves above white plate; rotates spatula so egg mold slides "
        "into plate; holds still for 2 s after placement.\n"
        "Stillness gate: trailing 2 s per-joint max-min <= %.2f rad on both "
        "arms (operator brief: drop hands off controls, count to 2 before "
        "stop).\n"
        "Folder layout: success episodes under <dataset>/0000/, 0001/, ...; "
        "failures retained under <dataset>/_failed/f0000/, f0001/, ... .\n"
        "Required files per episode (9): %s.\n"
        "Diversity: morning, afternoon, cloudy/other; green long gloves with "
        "sleeves; clean table (only pan + white plate); no head occlusion of "
        "cam_top.\n"
        "Operator distribution from task plan: Boris/Gaochen/Jingxiang each "
        "~30 sequences; Xiaobin observe/participate.\n"
        "egg_scoop_v2 is the 2-cam legacy variant (no cam_top); do NOT extend "
        "it — re-collect under the 3-cam stack instead." %
        (delta, threshold_used, ", ".join(REQUIRED_FILES)))

    return {
        "schema_version": "drag_demo_initial_batch.v1",
        "generated_at": _dt.datetime.utcnow().isoformat(timespec="seconds")
                                  + "Z",
        "verifier_path": str(verifier_path),
        "verifier_sha256": _sha256(verifier_path),
        "source_report": str(report_path),
        "source_report_sha256": _sha256(report_path),
        "verifier_config_used": cfg,
        "demo_target": 100,
        "stillness_threshold_rad_used": threshold_used,
        "eval_tier_strict_threshold_rad": eval_tier_threshold_rad,
        "n_counted_toward_target": len(counted),
        "delta_to_target": delta,
        "n_eval_tier_strict": len(eval_tier),
        "n_near_miss_stillness_only": len(near_miss),
        "n_file_or_stream_fail": len(file_or_stream_fail),
        "by_dataset_counted": by_dataset_counted,
        "required_files": list(REQUIRED_FILES),
        "sop_gates": [
            "success folder matches ^\\d{4}$",
            "all 9 required files present and non-empty",
            "each RGB mp4 opens via cv2; 16 sampled frames pass black(mean>=5) and frozen(std>=1) checks",
            "each depth h5 opens with `depth_mm` + `timestamps_ms`; <99% zero-pixel fraction on samples",
            "pickle schemas: state.{left,right}_arm.{joints,timestamps}; eef_pose.{left,right}_arm.{eef_pose,timestamps}; image_timestamps per-cam int64 ms",
            "trailing %.2f s per-joint (max-min) <= %.2f rad on both arms"
            % (cfg["stillness_window_sec"], threshold_used),
        ],
        "counted_episodes": [_summarize(e) for e in counted],
        "eval_tier_strict_episodes": [_summarize(e) for e in eval_tier],
        "near_miss_stillness_only": [_summarize(e) for e in near_miss],
        "file_or_stream_fail": [_summarize(e) for e in file_or_stream_fail],
        "onsite_collection_brief": onsite_brief,
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--report", type=Path, required=True,
                    help="Path to raw_data_full_report.json from the verifier.")
    ap.add_argument("--out", type=Path, required=True,
                    help="Where to write initial_batch_manifest.json.")
    ap.add_argument("--verifier", type=Path, default=DEFAULT_VERIFIER,
                    help="Path to sop_episode_verifier.py (for sha256).")
    ap.add_argument("--eval-tier-threshold-rad", type=float, default=0.02,
                    help="Trailing-window joint-range bound for the eval tier "
                         "(default 0.02 rad, regardless of verifier threshold).")
    args = ap.parse_args(argv)

    manifest = build_manifest(args.report, args.verifier,
                              args.eval_tier_threshold_rad)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    print(f"Wrote {args.out}")
    print(f"  counted_toward_target = {manifest['n_counted_toward_target']}"
          f" / {manifest['demo_target']}"
          f"  (delta {manifest['delta_to_target']})")
    print(f"  eval_tier_strict      = {manifest['n_eval_tier_strict']}")
    print(f"  near_miss             = {manifest['n_near_miss_stillness_only']}")
    print(f"  file_or_stream_fail   = {manifest['n_file_or_stream_fail']}")
    print(f"  by_dataset_counted    = {manifest['by_dataset_counted']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
