#!/usr/bin/env python3
"""SOP episode completeness verifier for the spatula egg-transfer dataset.

Validates each recorded episode against the dataset SOP:

  * Folder layout: success episodes match `^\\d{4}$` at the dataset root; failed
    episodes live under `_failed/` with names matching `^f\\d{4}` (an optional
    suffix like `_broken_no_pkl` is allowed). Anything else is classified
    `other`.
  * The 9 required files: `cam_top_rgb.mp4`, `cam_top_depth.h5`,
    `cam_left_wrist_rgb.mp4`, `cam_left_wrist_depth.h5`,
    `cam_right_wrist_rgb.mp4`, `cam_right_wrist_depth.h5`, `eef_pose.pkl`,
    `image_timestamps.pkl`, `state.pkl`.
  * Per-stream readability for the 3 RGB mp4s and 3 depth h5s.
  * Per-camera black-frame / frozen-frame sampling for the mp4s.
  * Per-camera zero-depth sampling for the h5s.
  * Pickle schema for `state.pkl` (joints + timestamps), `eef_pose.pkl`
    (eef_pose + timestamps), and `image_timestamps.pkl` (per-cam int64 ms).
  * Final 2-second stillness on `state.pkl` joints (both arms).

Usage:
    python sop_episode_verifier.py --root /path/to/dataset_root [--report out.json]

The dataset root may be either a single episode directory or a directory
containing multiple `0000/`, `0001/`, ..., `_failed/f0000/`, ... folders.
Use `--root /path/to/raw_data` to scan every dataset under raw_data.
"""

from __future__ import annotations

import argparse
import json
import os
import pickle
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import h5py
import numpy as np

CAM_KEYS = ("cam_top", "cam_left_wrist", "cam_right_wrist")
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
SUCCESS_RE = re.compile(r"^\d{4}$")
FAILED_RE = re.compile(r"^f\d{4}(_.*)?$")


@dataclass
class VerifierConfig:
    sample_frames: int = 16
    black_lum_threshold: float = 5.0          # mean BGR luminance below = black
    frozen_std_threshold: float = 1.0         # stddev below = frozen/flat frame
    zero_depth_fraction_threshold: float = 0.99
    stillness_window_sec: float = 2.0
    # Per-joint max-min over the trailing window. 0.10 rad (~5.7 deg) is the
    # SOP gate chosen 2026-05-23: tight enough to enforce that the operator
    # actively halted the arm before stop; loose enough to forgive natural
    # gripper-finger relaxation and small wrist drift at the end of the
    # rotation. Tighter values reject too many otherwise-clean episodes (only
    # 13/163 pass at 0.02 rad); looser values let the right wrist still be in
    # motion at the marker, which corrupts the task-end signal for the policy.
    stillness_max_range_rad: float = 0.10
    min_episode_seconds: float = 3.0


def classify_folder(parent: Path, name: str) -> str:
    """Return `success`, `failed`, or `other` based on folder name + parent."""
    if SUCCESS_RE.match(name):
        return "success"
    if FAILED_RE.match(name):
        return "failed"
    return "other"


def _file_check(epdir: Path) -> dict[str, Any]:
    present, missing, sizes = [], [], {}
    for name in REQUIRED_FILES:
        p = epdir / name
        if p.is_file() and p.stat().st_size > 0:
            present.append(name)
            sizes[name] = p.stat().st_size
        else:
            missing.append(name)
    return {"present": present, "missing": missing, "sizes": sizes,
            "all_present": not missing}


def _sample_indices(total: int, count: int) -> list[int]:
    if total <= 0:
        return []
    if total <= count:
        return list(range(total))
    return [int(round(i * (total - 1) / (count - 1))) for i in range(count)]


def _video_check(path: Path, cfg: VerifierConfig) -> dict[str, Any]:
    if not path.is_file():
        return {"ok": False, "error": "missing"}
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        return {"ok": False, "error": "cv2_open_failed"}
    try:
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if n <= 0:
            return {"ok": False, "error": "no_frames",
                    "frames": n, "fps": fps, "w": w, "h": h}

        idxs = _sample_indices(n, cfg.sample_frames)
        black_frames: list[int] = []
        frozen_frames: list[int] = []
        decode_failures: list[int] = []
        sampled = 0
        for idx in idxs:
            cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, frame = cap.read()
            if not ok or frame is None:
                decode_failures.append(idx)
                continue
            sampled += 1
            mean = float(frame.mean())
            std = float(frame.std())
            if mean < cfg.black_lum_threshold:
                black_frames.append(idx)
            elif std < cfg.frozen_std_threshold:
                frozen_frames.append(idx)
        return {
            "ok": (not black_frames and not frozen_frames
                   and not decode_failures and n > 0),
            "frames": n,
            "fps": fps,
            "w": w,
            "h": h,
            "sampled": sampled,
            "black_frames": black_frames,
            "frozen_frames": frozen_frames,
            "decode_failures": decode_failures,
        }
    finally:
        cap.release()


def _depth_check(path: Path, cfg: VerifierConfig) -> dict[str, Any]:
    if not path.is_file():
        return {"ok": False, "error": "missing"}
    try:
        with h5py.File(path, "r") as f:
            if "depth_mm" not in f or "timestamps_ms" not in f:
                return {"ok": False, "error": "missing_datasets",
                        "keys": list(f.keys())}
            dset = f["depth_mm"]
            ts = f["timestamps_ms"]
            n = int(dset.shape[0])
            if n <= 0:
                return {"ok": False, "error": "no_frames", "frames": n}
            ts_n = int(ts.shape[0])
            if ts_n != n:
                return {"ok": False, "error": "ts_mismatch",
                        "frames": n, "timestamps": ts_n}
            idxs = _sample_indices(n, cfg.sample_frames)
            zero_frames: list[int] = []
            for idx in idxs:
                f_arr = dset[idx]
                zero_frac = float(np.mean(f_arr == 0))
                if zero_frac >= cfg.zero_depth_fraction_threshold:
                    zero_frames.append(idx)
            return {
                "ok": not zero_frames,
                "frames": n,
                "h": int(dset.shape[1]),
                "w": int(dset.shape[2]),
                "sampled": len(idxs),
                "zero_frames": zero_frames,
            }
    except OSError as e:
        return {"ok": False, "error": f"h5_open_failed: {e}"}


def _stillness_from_state(state: dict[str, Any], cfg: VerifierConfig
                          ) -> dict[str, Any]:
    """Confirm last `stillness_window_sec` of joints stays within range."""
    arms: dict[str, Any] = {}
    overall_ok = True
    for arm in ("left_arm", "right_arm"):
        if arm not in state:
            arms[arm] = {"ok": False, "error": "missing_arm"}
            overall_ok = False
            continue
        inner = state[arm]
        if not isinstance(inner, dict) or "joints" not in inner \
                or "timestamps" not in inner:
            arms[arm] = {"ok": False, "error": "bad_schema",
                         "keys": list(inner.keys()) if isinstance(inner, dict)
                                 else None}
            overall_ok = False
            continue
        joints = np.asarray(inner["joints"], dtype=np.float64)
        ts = np.asarray(inner["timestamps"], dtype=np.int64)
        if joints.ndim != 2 or joints.shape[0] < 2 or joints.shape[0] != ts.shape[0]:
            arms[arm] = {"ok": False, "error": "bad_shapes",
                         "joints_shape": list(joints.shape),
                         "ts_shape": list(ts.shape)}
            overall_ok = False
            continue
        t_end = int(ts[-1])
        window_start = t_end - int(cfg.stillness_window_sec * 1000)
        mask = ts >= window_start
        window = joints[mask]
        if window.shape[0] < 2:
            arms[arm] = {"ok": False, "error": "window_too_short",
                         "window_samples": int(window.shape[0])}
            overall_ok = False
            continue
        per_joint_range = (window.max(axis=0) - window.min(axis=0))
        max_range = float(per_joint_range.max())
        worst_joint = int(per_joint_range.argmax())
        is_still = max_range <= cfg.stillness_max_range_rad
        arms[arm] = {
            "ok": bool(is_still),
            "window_samples": int(window.shape[0]),
            "max_range_rad": max_range,
            "worst_joint_idx": worst_joint,
        }
        if not is_still:
            overall_ok = False
    return {"ok": overall_ok, "arms": arms}


def _pkl_check(epdir: Path, cfg: VerifierConfig) -> dict[str, Any]:
    out: dict[str, Any] = {}
    # state.pkl + stillness
    p = epdir / "state.pkl"
    if p.is_file():
        try:
            with open(p, "rb") as f:
                state = pickle.load(f)
            out["state"] = {"ok": True, "keys": list(state.keys())}
            out["stillness"] = _stillness_from_state(state, cfg)
        except (OSError, pickle.UnpicklingError, EOFError) as e:
            out["state"] = {"ok": False, "error": f"load_failed: {e!s}"}
            out["stillness"] = {"ok": False, "error": "state_load_failed"}
    else:
        out["state"] = {"ok": False, "error": "missing"}
        out["stillness"] = {"ok": False, "error": "state_missing"}

    # eef_pose.pkl
    p = epdir / "eef_pose.pkl"
    if p.is_file():
        try:
            with open(p, "rb") as f:
                eef = pickle.load(f)
            ok = (isinstance(eef, dict)
                  and all(arm in eef and isinstance(eef[arm], dict)
                          and "eef_pose" in eef[arm]
                          and "timestamps" in eef[arm]
                          for arm in ("left_arm", "right_arm")))
            out["eef_pose"] = {"ok": ok,
                               "keys": list(eef.keys())
                                       if isinstance(eef, dict) else None}
        except (OSError, pickle.UnpicklingError, EOFError) as e:
            out["eef_pose"] = {"ok": False, "error": f"load_failed: {e!s}"}
    else:
        out["eef_pose"] = {"ok": False, "error": "missing"}

    # image_timestamps.pkl
    p = epdir / "image_timestamps.pkl"
    if p.is_file():
        try:
            with open(p, "rb") as f:
                its = pickle.load(f)
            cams_present = [k for k in CAM_KEYS if k in its] \
                           if isinstance(its, dict) else []
            cams_missing = [k for k in CAM_KEYS if k not in cams_present]
            duration_s = None
            if cams_present and isinstance(its, dict):
                arr = np.asarray(its[cams_present[0]], dtype=np.int64)
                if arr.size >= 2:
                    duration_s = float((arr[-1] - arr[0]) / 1000.0)
            out["image_timestamps"] = {
                "ok": not cams_missing,
                "cams_present": cams_present,
                "cams_missing": cams_missing,
                "duration_s_cam_top": duration_s,
            }
        except (OSError, pickle.UnpicklingError, EOFError) as e:
            out["image_timestamps"] = {"ok": False,
                                       "error": f"load_failed: {e!s}"}
    else:
        out["image_timestamps"] = {"ok": False, "error": "missing"}
    return out


def verify_episode(epdir: Path, cfg: VerifierConfig | None = None
                   ) -> dict[str, Any]:
    cfg = cfg or VerifierConfig()
    name = epdir.name
    label = classify_folder(epdir.parent, name)
    result: dict[str, Any] = {
        "path": str(epdir),
        "name": name,
        "label": label,
        "checks": {},
        "ok": False,
        "reasons": [],
    }

    files = _file_check(epdir)
    result["checks"]["files"] = files
    if not files["all_present"]:
        result["reasons"].append("missing_files: " + ",".join(files["missing"]))
        # We still try the streams that are present so the report is useful.

    videos: dict[str, Any] = {}
    for cam in CAM_KEYS:
        videos[cam] = _video_check(epdir / f"{cam}_rgb.mp4", cfg)
        if not videos[cam].get("ok"):
            result["reasons"].append(
                f"video[{cam}]: " + str(videos[cam].get("error")
                                        or {k: videos[cam].get(k) for k in
                                            ("black_frames", "frozen_frames",
                                             "decode_failures") if videos[cam].get(k)}))
    result["checks"]["videos"] = videos

    depths: dict[str, Any] = {}
    for cam in CAM_KEYS:
        depths[cam] = _depth_check(epdir / f"{cam}_depth.h5", cfg)
        if not depths[cam].get("ok"):
            result["reasons"].append(
                f"depth[{cam}]: " + str(depths[cam].get("error")
                                        or {"zero_frames": depths[cam].get("zero_frames")}))
    result["checks"]["depths"] = depths

    pkls = _pkl_check(epdir, cfg)
    result["checks"].update(pkls)
    if not pkls["state"].get("ok"):
        result["reasons"].append(f"state.pkl: {pkls['state'].get('error')}")
    if not pkls["eef_pose"].get("ok"):
        result["reasons"].append(f"eef_pose.pkl: {pkls['eef_pose'].get('error')}")
    if not pkls["image_timestamps"].get("ok"):
        result["reasons"].append(
            f"image_timestamps.pkl: {pkls['image_timestamps'].get('error')}")
    if not pkls["stillness"].get("ok"):
        arms = pkls["stillness"].get("arms", {})
        bad = {a: v for a, v in arms.items() if not v.get("ok")}
        result["reasons"].append(f"stillness: {bad or pkls['stillness'].get('error')}")

    # Duration sanity (warn only — short demos are still valid for the SOP if
    # all 9 files exist and they're still). Long episodes are fine.
    dur = pkls["image_timestamps"].get("duration_s_cam_top")
    if dur is not None and dur < cfg.min_episode_seconds:
        result["reasons"].append(
            f"duration<{cfg.min_episode_seconds}s: {dur:.2f}s")
    result["duration_s"] = dur

    result["ok"] = not result["reasons"]
    return result


def iter_episodes(root: Path) -> list[tuple[str, Path]]:
    """Yield (dataset_name, episode_path) pairs under `root`.

    Recognises three shapes:
      * episode dir directly: root contains REQUIRED_FILES
      * dataset dir: root contains many 0000/, 0001/, ..., and optional _failed/
      * raw_data dir: root contains several dataset dirs as above
    """
    if root.is_dir() and any((root / f).exists() for f in REQUIRED_FILES):
        yield (root.parent.name, root)
        return

    # Otherwise, look for dataset-shaped children.
    children = sorted(p for p in root.iterdir() if p.is_dir())
    if not children:
        return

    # Detect dataset shape: at least one child matches SUCCESS_RE or is _failed.
    def looks_like_dataset(d: Path) -> bool:
        return any(SUCCESS_RE.match(c.name) or c.name == "_failed"
                   for c in d.iterdir() if c.is_dir())

    if any(SUCCESS_RE.match(c.name) or c.name == "_failed" for c in children):
        # `root` is one dataset.
        datasets = [root]
    else:
        datasets = [c for c in children if looks_like_dataset(c)]

    for ds in datasets:
        for entry in sorted(ds.iterdir()):
            if not entry.is_dir():
                continue
            if SUCCESS_RE.match(entry.name):
                yield (ds.name, entry)
        failed = ds / "_failed"
        if failed.is_dir():
            for entry in sorted(failed.iterdir()):
                if entry.is_dir() and FAILED_RE.match(entry.name):
                    yield (ds.name, entry)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True, type=Path,
                    help="Episode dir, dataset dir, or raw_data dir.")
    ap.add_argument("--report", type=Path, default=None,
                    help="Optional path to write the full JSON report.")
    ap.add_argument("--sample-frames", type=int, default=16,
                    help="Frames sampled per video/depth stream (default 16).")
    ap.add_argument("--black-lum-threshold", type=float, default=5.0,
                    help="BGR mean below this is treated as a black frame (default 5).")
    ap.add_argument("--stillness-window-sec", type=float, default=2.0,
                    help="Trailing window for stillness check (default 2.0s).")
    ap.add_argument("--stillness-threshold-rad", type=float, default=0.10,
                    help="Max per-joint range (rad) in the stillness window "
                         "(default 0.10 ≈ 5.7 deg).")
    ap.add_argument("--quiet", action="store_true",
                    help="Suppress per-episode lines; only print summary.")
    args = ap.parse_args(argv)

    cfg = VerifierConfig(
        sample_frames=args.sample_frames,
        black_lum_threshold=args.black_lum_threshold,
        stillness_window_sec=args.stillness_window_sec,
        stillness_max_range_rad=args.stillness_threshold_rad,
    )

    if not args.root.is_dir():
        print(f"error: --root not a directory: {args.root}", file=sys.stderr)
        return 2

    eps = list(iter_episodes(args.root))
    if not eps:
        print(f"error: no episodes found under {args.root}", file=sys.stderr)
        return 2

    results: list[dict[str, Any]] = []
    for dataset, epdir in eps:
        r = verify_episode(epdir, cfg)
        r["dataset"] = dataset
        results.append(r)
        if not args.quiet:
            status = "OK " if r["ok"] else "FAIL"
            tag = r["label"]
            print(f"[{status}] {tag:<7} {dataset}/{r['name']}: "
                  f"{'' if r['ok'] else '; '.join(r['reasons'])}")

    by_label: dict[str, dict[str, int]] = {}
    by_dataset: dict[str, dict[str, int]] = {}
    for r in results:
        bucket = by_label.setdefault(r["label"], {"total": 0, "ok": 0, "fail": 0})
        bucket["total"] += 1
        bucket["ok" if r["ok"] else "fail"] += 1
        ds = by_dataset.setdefault(r["dataset"],
                                   {"success_total": 0, "success_ok": 0,
                                    "failed_total": 0, "failed_ok": 0,
                                    "counted_toward_target": 0})
        if r["label"] == "success":
            ds["success_total"] += 1
            if r["ok"]:
                ds["success_ok"] += 1
                ds["counted_toward_target"] += 1
        elif r["label"] == "failed":
            ds["failed_total"] += 1
            if r["ok"]:
                ds["failed_ok"] += 1

    # SOP gate: only ok success episodes count toward the 100-demo target.
    n_counted = sum(1 for r in results
                    if r["label"] == "success" and r["ok"])

    summary = {
        "root": str(args.root),
        "config": cfg.__dict__,
        "totals": by_label,
        "by_dataset": by_dataset,
        "n_episodes": len(results),
        "n_ok": sum(1 for r in results if r["ok"]),
        "n_fail": sum(1 for r in results if not r["ok"]),
        "n_counted_toward_target": n_counted,
        "demo_target": 100,
    }
    print("\n=== summary ===")
    print(json.dumps(summary, indent=2))

    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        with open(args.report, "w") as f:
            json.dump({"summary": summary, "episodes": results}, f, indent=2,
                      default=_json_default)
        print(f"\nWrote report: {args.report}")

    return 0 if summary["n_fail"] == 0 else 1


def _json_default(o: Any) -> Any:
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"not JSON serializable: {type(o).__name__}")


if __name__ == "__main__":
    sys.exit(main())
