#!/usr/bin/env python3
"""
Visualize a recorded ARX episode in Rerun (https://rerun.io).

Supports two on-disk layouts:

  Streaming (current, written by data_record.py after the streaming rewrite):
    cameraN_rgb.mp4          BGR uint8 frames, lazy lossy mp4
    cameraN_depth.h5         dataset 'depth_mm' (uint16) + 'timestamps_ms'
    image_timestamps.pkl     {cameraN: [ms, ...]}
    state.pkl                joint positions per arm
    eef_pose.pkl             end-effector pose per arm

  Legacy (single-pickle dump, kept for replaying old data):
    image.pkl     BGR uint8 frames per camera        (~30 Hz)
    depth.pkl     float32 depth, millimeters         (~30 Hz)
    state.pkl     joint positions per arm, 7-vector  (~60 Hz)
    eef_pose.pkl  end-effector pose per arm          (~60 Hz)

Each stream is logged at its own header.stamp timestamp, so the Rerun timeline
naturally aligns them — there is no resampling here.

Usage:
  python visualize_episode.py /path/to/raw_data/0000              # spawn viewer
  python visualize_episode.py /path/to/raw_data/0000 --save ep.rrd
  python visualize_episode.py /path/to/raw_data/0000 --no-depth   # smaller
"""

import argparse
import os
import pickle
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import cv2
import h5py
import numpy as np
import rerun as rr
import rerun.blueprint as rrb


def kill_stale_viewer():
    """Close any already-running Rerun viewer.

    Why: rr.init(..., spawn=True) connects to an existing viewer instead of
    spawning a new one, and adds our recording as a sibling "tab" without
    swapping the active view — looks like a black screen on the new episode.

    The spawned viewer binary is just `rerun` (verified against rerun-sdk
    0.19.x); pkill -x matches that exact process name and avoids the false
    positives a -f rerun substring search would hit (e.g. this script's own
    path if it contains "rerun").
    """
    subprocess.run(
        ['pkill', '-x', 'rerun'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    # Give the OS a beat to release the TCP port the viewer was listening on;
    # otherwise the next spawn races and may still connect to the dying one.
    time.sleep(0.5)


def safe_load_pickle(path: Path):
    if not path.exists():
        return None, "missing"
    try:
        with open(path, "rb") as f:
            return pickle.load(f), "ok"
    except Exception as e:
        return None, f"corrupt ({type(e).__name__}: {e})"


# ---------------- Camera discovery ----------------

def discover_cameras(ep_dir: Path):
    """Returns (cam_keys, format), where format ∈ {'streaming', 'legacy', 'none'}."""
    mp4s = sorted(ep_dir.glob("*_rgb.mp4"))
    if mp4s:
        cam_keys = [p.name.replace("_rgb.mp4", "") for p in mp4s]
        return cam_keys, "streaming"

    img_pkl = ep_dir / "image.pkl"
    if img_pkl.exists():
        data, status = safe_load_pickle(img_pkl)
        if data is not None:
            return sorted(data.keys()), "legacy"

    return [], "none"


def has_depth(ep_dir: Path, cam_keys, fmt: str) -> bool:
    if fmt == "streaming":
        return any((ep_dir / f"{k}_depth.h5").exists() for k in cam_keys)
    if fmt == "legacy":
        return (ep_dir / "depth.pkl").exists()
    return False


# ---------------- RGB logging ----------------

def _rel(ts_ms, t0_ms):
    """Convert absolute epoch ms → relative seconds since episode start."""
    return (float(ts_ms) - t0_ms) / 1000.0


def compute_t0_ms(ep_dir: Path, fmt: str, cam_keys):
    """Earliest timestamp (ms) across all data sources for this episode.

    Without this every episode's timeline sits at the Unix epoch and Rerun
    keeps the playhead at the previously-viewed episode's range when you open
    a new one in the same viewer → screen looks blank ("black").
    """
    candidates = []

    # Image timestamps
    if fmt == "streaming":
        ts_map, _ = safe_load_pickle(ep_dir / "image_timestamps.pkl")
        if ts_map:
            for k in (cam_keys or []):
                stamps = ts_map.get(k) or []
                if stamps:
                    candidates.append(stamps[0])

    # Depth timestamps
    if fmt == "streaming":
        for k in (cam_keys or []):
            p = ep_dir / f"{k}_depth.h5"
            if not p.exists():
                continue
            try:
                with h5py.File(p, "r") as f:
                    ts = f.get("timestamps_ms")
                    if ts is not None and ts.shape[0]:
                        candidates.append(int(ts[0]))
            except Exception:
                pass

    # State / eef
    for fname in ("state.pkl", "eef_pose.pkl"):
        data, _ = safe_load_pickle(ep_dir / fname)
        if not data:
            continue
        for side in ("left_arm", "right_arm"):
            stamps = (data.get(side) or {}).get("timestamps") or []
            if stamps:
                candidates.append(stamps[0])

    return min(candidates) if candidates else 0


def log_rgb_streaming(ep_dir: Path, cam_keys, t0_ms: int):
    """Read each camera's mp4 + paired timestamps and log frame-by-frame."""
    ts_map, _ = safe_load_pickle(ep_dir / "image_timestamps.pkl")
    if ts_map is None:
        ts_map = {}
    for cam_key in cam_keys:
        mp4_path = ep_dir / f"{cam_key}_rgb.mp4"
        if not mp4_path.exists():
            continue
        stamps = ts_map.get(cam_key, [])
        cap = cv2.VideoCapture(str(mp4_path))
        path = f"world/{cam_key}/rgb"
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            # Time tag: prefer the original header.stamp if we have it,
            # otherwise fall back to frame index / fps for graceful degradation.
            if i < len(stamps):
                rr.set_time_seconds("time", _rel(stamps[i], t0_ms))
            else:
                fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
                rr.set_time_seconds("time", i / fps)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rr.log(path, rr.Image(rgb))
            i += 1
        cap.release()


def log_rgb_legacy(image_pkl, t0_ms: int):
    if image_pkl is None:
        return
    for cam_key in sorted(image_pkl.keys()):
        frames = image_pkl[cam_key]["image"]
        stamps = image_pkl[cam_key]["timestamps"]
        path = f"world/{cam_key}/rgb"
        for img, ts_ms in zip(frames, stamps):
            rr.set_time_seconds("time", _rel(ts_ms, t0_ms))
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            rr.log(path, rr.Image(rgb))


# ---------------- Depth logging ----------------

def log_depth_streaming(ep_dir: Path, cam_keys, t0_ms: int):
    for cam_key in cam_keys:
        h5_path = ep_dir / f"{cam_key}_depth.h5"
        if not h5_path.exists():
            continue
        path = f"world/{cam_key}/depth"
        try:
            with h5py.File(h5_path, "r") as f:
                depth = f["depth_mm"]
                ts    = f["timestamps_ms"]
                n = min(depth.shape[0], ts.shape[0])
                for i in range(n):
                    rr.set_time_seconds("time", _rel(ts[i], t0_ms))
                    # uint16 millimetres → meter=1000 tells Rerun how to scale
                    rr.log(path, rr.DepthImage(depth[i], meter=1000.0))
        except Exception as e:
            print(f"  skipped {h5_path.name}: {type(e).__name__}: {e}")


def log_depth_legacy(depth_pkl, t0_ms: int):
    if depth_pkl is None:
        return
    for cam_key in sorted(depth_pkl.keys()):
        dframes = depth_pkl[cam_key]["depth"]
        stamps  = depth_pkl[cam_key]["timestamps"]
        path = f"world/{cam_key}/depth"
        for d, ts_ms in zip(dframes, stamps):
            rr.set_time_seconds("time", _rel(ts_ms, t0_ms))
            rr.log(path, rr.DepthImage(d, meter=1000.0))


# ---------------- Joints / eef ----------------

def log_joints(state, t0_ms: int):
    if state is None:
        return
    for side in ("left_arm", "right_arm"):
        if side not in state:
            continue
        joints_list = state[side]["joints"]
        stamps      = state[side]["timestamps"]
        for joints, ts_ms in zip(joints_list, stamps):
            rr.set_time_seconds("time", _rel(ts_ms, t0_ms))
            arr = np.asarray(joints).ravel()
            for j, v in enumerate(arr):
                rr.log(f"plots/{side}/joint{j}", rr.Scalar(float(v)))


def log_eef(eef, t0_ms: int):
    if eef is None:
        return
    color_by_side = {"left_arm": [220, 60, 60], "right_arm": [60, 120, 220]}
    for side in ("left_arm", "right_arm"):
        if side not in eef:
            continue
        poses  = eef[side]["eef_pose"]
        stamps = eef[side]["timestamps"]
        color  = color_by_side[side]
        path   = f"world/{side}/eef"
        for pose, ts_ms in zip(poses, stamps):
            p = np.asarray(pose).ravel()
            xyz = p[:3]
            qw, qx, qy, qz = p[3], p[4], p[5], p[6]
            rr.set_time_seconds("time", _rel(ts_ms, t0_ms))
            rr.log(path, rr.Transform3D(
                translation=xyz,
                rotation=rr.Quaternion(xyzw=[qx, qy, qz, qw]),
                axis_length=0.1,
            ))
            rr.log(f"world/{side}/eef_pt", rr.Points3D([xyz], colors=[color], radii=0.006))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode_dir", type=Path)
    ap.add_argument("--no-depth", action="store_true", help="skip depth (faster, less memory)")
    ap.add_argument("--save", type=Path, default=None,
                    help="write a .rrd file instead of spawning the viewer")
    ap.add_argument("--connect", type=str, default=None,
                    help="connect to an already-running rerun viewer at host:port")
    args = ap.parse_args()

    ep = args.episode_dir
    if not ep.is_dir():
        sys.exit(f"not a directory: {ep}")

    cam_keys, fmt = discover_cameras(ep)
    want_depth = (not args.no_depth) and has_depth(ep, cam_keys, fmt)

    # Load legacy blobs eagerly only if we're in legacy mode (streaming uses
    # lazy readers built directly inside the log_*_streaming functions).
    image_pkl = depth_pkl = None
    image_status = depth_status = "skipped"
    if fmt == "legacy":
        image_pkl, image_status = safe_load_pickle(ep / "image.pkl")
        if want_depth:
            depth_pkl, depth_status = safe_load_pickle(ep / "depth.pkl")

    state, state_status = safe_load_pickle(ep / "state.pkl")
    eef,   eef_status   = safe_load_pickle(ep / "eef_pose.pkl")

    print(f"episode: {ep}")
    print(f"  format       : {fmt}")
    print(f"  cameras      : {cam_keys or '(none)'}")
    print(f"  depth        : {'yes' if want_depth else 'no'}")
    if fmt == "legacy":
        print(f"  image.pkl    : {image_status}")
        print(f"  depth.pkl    : {depth_status}")
    print(f"  state.pkl    : {state_status}")
    print(f"  eef_pose.pkl : {eef_status}")

    has_any = bool(cam_keys) or state is not None or eef is not None
    if not has_any:
        sys.exit("no usable data — nothing to log")

    # One column per camera (RGB on top, Depth below). Cameras laid out
    # horizontally so vertical real-estate is never split more than 2 ways.
    # Old "Vertical(cam_row × N)" stacked 3 rows × 2 panels = 6 rows of
    # vertical pressure and the last camera silently disappeared off the
    # bottom on shorter screens — no scrollbar, no trace in the UI.
    cam_cols = []
    for cam_key in cam_keys:
        col_views = [rrb.Spatial2DView(name=f"{cam_key} RGB", origin=f"/world/{cam_key}/rgb")]
        if want_depth:
            col_views.append(rrb.Spatial2DView(name=f"{cam_key} Depth", origin=f"/world/{cam_key}/depth"))
        cam_cols.append(rrb.Vertical(*col_views))

    side_views = [rrb.Spatial3DView(name="world", origin="/world")]
    if state is not None:
        side_views.append(rrb.TimeSeriesView(name="joints", origin="/plots"))

    if cam_cols:
        # N cam columns share 3 units of width; side column gets 1.
        n = len(cam_cols)
        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                *cam_cols,
                rrb.Vertical(*side_views),
                column_shares=[3 / n] * n + [1],
            ),
            collapse_panels=True,
        )
    else:
        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(origin="/world"),
                rrb.Vertical(*side_views),
                column_shares=[3, 1],
            ),
            collapse_panels=True,
        )

    rec_name = f"arx_episode_{ep.name}"
    # If non-None at end of main(), we'll `rr.disconnect()` to flush and
    # then launch the viewer on this path. Replaces rr.init(spawn=True),
    # whose live TCP stream silently drops RGB chunks on big episodes:
    # the viewer's default memory_limit=75% GCs the oldest data first,
    # and we log RGB before depth/joints, so RGB goes overboard while the
    # later streams survive — three black camera panels, depth and joints
    # fine. Loading from a finalized .rrd has no such streaming pressure.
    pending_viewer_rrd = None

    if args.save is not None:
        rr.init(rec_name)
        rr.send_blueprint(blueprint)
        rr.save(str(args.save))
    elif args.connect is not None:
        rr.init(rec_name)
        rr.connect_tcp(args.connect)
        rr.send_blueprint(blueprint)
    else:
        kill_stale_viewer()
        # Per-episode fixed name: re-runs on the same episode overwrite.
        # PID is not used to keep accumulation bounded (one file per ep).
        pending_viewer_rrd = Path(tempfile.gettempdir()) / f"arx_viz_{ep.name}.rrd"
        rr.init(rec_name)
        rr.send_blueprint(blueprint)
        rr.save(str(pending_viewer_rrd))

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/axes", rr.Arrows3D(
        origins=[[0, 0, 0]] * 3,
        vectors=[[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
    ), static=True)

    # All log_* functions take t0_ms so each episode plays from 0s and the
    # viewer's playhead doesn't get stuck on a previous episode's Unix range.
    t0_ms = compute_t0_ms(ep, fmt, cam_keys)
    print(f"  t0 (epoch ms): {t0_ms}  → timeline anchored at this point")

    if fmt == "streaming":
        log_rgb_streaming(ep, cam_keys, t0_ms)
        if want_depth:
            log_depth_streaming(ep, cam_keys, t0_ms)
    elif fmt == "legacy":
        log_rgb_legacy(image_pkl, t0_ms)
        if want_depth:
            log_depth_legacy(depth_pkl, t0_ms)

    log_joints(state, t0_ms)
    log_eef(eef, t0_ms)

    if args.save is not None:
        print(f"saved -> {args.save}")

    if pending_viewer_rrd is not None:
        # Force the file sink to flush & close so the viewer sees complete
        # data; without this Popen would race the still-buffering writer.
        rr.disconnect()
        print(f"opening viewer on {pending_viewer_rrd}")
        subprocess.Popen(
            ['rerun', str(pending_viewer_rrd)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )


if __name__ == "__main__":
    main()
