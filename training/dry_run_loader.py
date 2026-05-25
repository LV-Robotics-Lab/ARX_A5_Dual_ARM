#!/usr/bin/env python3
"""Dry-run for DragDemoDataset.

Prints, for each mode requested:
  * resolved mixture
  * per-source availability + episode counts (manifest -> loaded -> after-mixture)
  * missing-stream report (e.g. teleop manifest absent)
  * total episodes and samples in the assembled dataset
  * tensor shapes from one real DataLoader batch
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import torch
from torch.utils.data import DataLoader

# Allow running from the directory the file sits in or from elsewhere.
THIS = Path(__file__).resolve().parent
if str(THIS) not in sys.path:
    sys.path.insert(0, str(THIS))

from drag_demo_loader import (DragDemoDataset, SplitConfig, collate,  # noqa: E402
                              ALLOWED_MODES)


def _shape_str(t: torch.Tensor) -> str:
    return f"{tuple(t.shape)} {t.dtype}"


def dry_run(split_config: Path, modes: list[str], *,
            batch_size: int, samples_per_episode: int, image_size: tuple[int, int] | None
            ) -> dict:
    cfg = SplitConfig.from_json(split_config)
    out: dict = {"split_config": str(split_config), "modes": {}}
    for mode in modes:
        try:
            ds = DragDemoDataset(cfg, mode,
                                 samples_per_episode=samples_per_episode,
                                 image_size=image_size)
        except ValueError as exc:
            out["modes"][mode] = {"error": str(exc)}
            print(f"[{mode}] ERROR: {exc}")
            continue
        r = ds.report()
        print(f"\n=== mode: {mode} ===")
        print(f"  mixture        : {r.mixture}")
        print(f"  episodes total : {r.n_total_episodes}")
        print(f"  samples total  : {r.n_total_samples}")
        print(f"  sources_summary:")
        for src, info in r.sources_summary.items():
            print(f"    - {src}: {info}")
        if r.missing_streams:
            print(f"  missing_streams ({len(r.missing_streams)}):")
            for item in r.missing_streams[:5]:
                print(f"    - {item}")
            if len(r.missing_streams) > 5:
                print(f"    ... and {len(r.missing_streams)-5} more")
        else:
            print(f"  missing_streams: none")

        result = {
            "mixture": r.mixture,
            "n_total_episodes": r.n_total_episodes,
            "n_total_samples": r.n_total_samples,
            "sources_summary": r.sources_summary,
            "missing_streams": r.missing_streams,
        }

        if len(ds) > 0:
            loader = DataLoader(ds, batch_size=batch_size, shuffle=False,
                                collate_fn=collate, num_workers=0)
            batch = next(iter(loader))
            print(f"  batch shapes:")
            print(f"    state  : {_shape_str(batch['state'])}")
            print(f"    action : {_shape_str(batch['action'])}")
            for cam, t in batch["rgb"].items():
                print(f"    rgb[{cam}]: {_shape_str(t)}")
            print(f"    meta[0]: {batch['meta'][0]}")
            result["batch_shapes"] = {
                "state": list(batch["state"].shape),
                "action": list(batch["action"].shape),
                "rgb": {cam: list(t.shape) for cam, t in batch["rgb"].items()},
            }
            result["meta_first"] = batch["meta"][0]
        else:
            print(f"  batch shapes: <skipped — empty dataset>")
            result["batch_shapes"] = None
        out["modes"][mode] = result
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--split-config", type=Path,
                    default=THIS / "split_config.json")
    ap.add_argument("--modes", nargs="+", default=list(ALLOWED_MODES),
                    choices=list(ALLOWED_MODES))
    ap.add_argument("--batch-size", type=int, default=4)
    ap.add_argument("--samples-per-episode", type=int, default=8)
    ap.add_argument("--image-size", type=int, nargs=2, default=None,
                    metavar=("W", "H"),
                    help="Optional resize for the loaded frames (W H).")
    ap.add_argument("--json-out", type=Path, default=None,
                    help="Optional path to dump the dry-run summary as JSON.")
    args = ap.parse_args(argv)

    image_size = tuple(args.image_size) if args.image_size else None
    summary = dry_run(args.split_config, args.modes,
                      batch_size=args.batch_size,
                      samples_per_episode=args.samples_per_episode,
                      image_size=image_size)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        # Make sure meta tuples are JSON-serializable.
        args.json_out.write_text(json.dumps(summary, indent=2, default=str))
        print(f"\nWrote dry-run JSON: {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
