#!/usr/bin/env python3
"""Loader smoke test for initial_batch_manifest.json.

Opens every counted episode's 9 files and reports any breakage. Intended both
as a sanity check ("does the loader contract hold right now?") and as a
reference for downstream training code:

    python load_initial_batch.py \\
        --manifest ~/workspace/.verifier_reports/initial_batch_manifest.json

For each counted episode it confirms:
  * every path listed under `files.*` exists and is non-empty
  * each `*_rgb.mp4` opens via cv2 and reports >0 frames
  * each `*_depth.h5` opens via h5py and exposes `depth_mm` + `timestamps_ms`
  * each `.pkl` loads via pickle

Exit code is 0 iff every counted episode passes. Use `--include-eval` to also
smoke-test the strict eval-tier subset.
"""

from __future__ import annotations

import argparse
import json
import pickle
import sys
from pathlib import Path

import cv2
import h5py


def check_episode(ep: dict, *, label: str) -> list[str]:
    errors: list[str] = []
    files = ep["files"]
    for name, p in files.items():
        path = Path(p)
        if not path.is_file() or path.stat().st_size == 0:
            errors.append(f"{label}[{ep['name']}] missing/empty {name} -> {p}")
            continue
        if name.endswith(".mp4"):
            cap = cv2.VideoCapture(p)
            try:
                if not cap.isOpened():
                    errors.append(f"{label}[{ep['name']}] cv2 cannot open {name}")
                    continue
                n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                if n <= 0:
                    errors.append(f"{label}[{ep['name']}] {name} has 0 frames")
            finally:
                cap.release()
        elif name.endswith(".h5"):
            try:
                with h5py.File(p, "r") as f:
                    if "depth_mm" not in f or "timestamps_ms" not in f:
                        errors.append(
                            f"{label}[{ep['name']}] {name} missing depth_mm/timestamps_ms")
            except OSError as exc:
                errors.append(f"{label}[{ep['name']}] h5 open failed for {name}: {exc}")
        elif name.endswith(".pkl"):
            try:
                with open(p, "rb") as f:
                    pickle.load(f)
            except (pickle.UnpicklingError, OSError, EOFError) as exc:
                errors.append(f"{label}[{ep['name']}] pkl load failed for {name}: {exc}")
    return errors


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--include-eval", action="store_true",
                    help="Also smoke-test eval_tier_strict_episodes.")
    args = ap.parse_args(argv)

    manifest = json.loads(args.manifest.read_text())
    counted = manifest.get("counted_episodes", [])
    print(f"manifest: {args.manifest}")
    print(f"schema  : {manifest.get('schema_version')}")
    print(f"counted : {len(counted)} / {manifest.get('demo_target')}"
          f"  threshold={manifest.get('stillness_threshold_rad_used')}")

    all_errors: list[str] = []
    for ep in counted:
        all_errors.extend(check_episode(ep, label="counted"))

    if args.include_eval:
        eval_eps = manifest.get("eval_tier_strict_episodes", [])
        print(f"eval    : {len(eval_eps)} eps "
              f"(strict <= {manifest.get('eval_tier_strict_threshold_rad')} rad)")
        for ep in eval_eps:
            all_errors.extend(check_episode(ep, label="eval"))

    if all_errors:
        print(f"\nFAIL: {len(all_errors)} loader error(s):")
        for e in all_errors[:50]:
            print(f"  - {e}")
        if len(all_errors) > 50:
            print(f"  ... and {len(all_errors)-50} more")
        return 1

    print(f"\nOK: all {len(counted)} counted episodes pass the loader contract"
          + (f" (and {len(manifest.get('eval_tier_strict_episodes', []))}"
             f" eval episodes)" if args.include_eval else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
