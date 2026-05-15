#!/usr/bin/env python3
"""
Visualize a recorded ARX episode in Rerun (https://rerun.io).

Loads any subset of:
  image.pkl     BGR uint8 frames per camera        (~30 Hz)
  depth.pkl     float32 depth, millimeters         (~30 Hz)
  state.pkl     joint positions per arm, 7-vector  (~60 Hz)
  eef_pose.pkl  end-effector pose per arm          (~60 Hz)
                pose layout: [x, y, z, qw, qx, qy, qz]

Each stream is logged at its own header.stamp timestamp, so the Rerun timeline
naturally aligns them — there is no resampling here.

Usage:
  python visualize_episode.py /path/to/raw_data/0000              # spawn viewer
  python visualize_episode.py /path/to/raw_data/0000 --save ep.rrd
  python visualize_episode.py /path/to/raw_data/0000 --no-depth   # smaller
"""

import argparse
import pickle
import sys
from pathlib import Path

import cv2
import numpy as np
import rerun as rr


def safe_load(path: Path):
    if not path.exists():
        return None, "missing"
    try:
        with open(path, "rb") as f:
            return pickle.load(f), "ok"
    except Exception as e:
        return None, f"corrupt ({type(e).__name__}: {e})"


def log_cameras(image, depth, *, log_depth: bool):
    if image is None:
        return
    for cam_key in sorted(image.keys()):
        frames = image[cam_key]["image"]
        stamps = image[cam_key]["timestamps"]
        path = f"world/{cam_key}/rgb"
        for img, ts_ms in zip(frames, stamps):
            rr.set_time_seconds("time", ts_ms / 1000.0)
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            rr.log(path, rr.Image(rgb))

    if log_depth and depth is not None:
        for cam_key in sorted(depth.keys()):
            dframes = depth[cam_key]["depth"]
            stamps = depth[cam_key]["timestamps"]
            path = f"world/{cam_key}/depth"
            for d, ts_ms in zip(dframes, stamps):
                rr.set_time_seconds("time", ts_ms / 1000.0)
                # realsense_pub_node publishes mm cast to float32 → meter=1000
                rr.log(path, rr.DepthImage(d, meter=1000.0))


def log_joints(state):
    if state is None:
        return
    for side in ("left_arm", "right_arm"):
        if side not in state:
            continue
        joints_list = state[side]["joints"]
        stamps      = state[side]["timestamps"]
        for joints, ts_ms in zip(joints_list, stamps):
            rr.set_time_seconds("time", ts_ms / 1000.0)
            arr = np.asarray(joints).ravel()
            for j, v in enumerate(arr):
                rr.log(f"plots/{side}/joint{j}", rr.Scalar(float(v)))


def log_eef(eef):
    if eef is None:
        return
    # arrow colors: left = red, right = blue (matches common L/R convention)
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
            rr.set_time_seconds("time", ts_ms / 1000.0)
            rr.log(path, rr.Transform3D(
                translation=xyz,
                rotation=rr.Quaternion(xyzw=[qx, qy, qz, qw]),
                axis_length=0.1,
            ))
            # also drop a colored dot so the trajectory is visible without axes
            rr.log(f"world/{side}/eef_pt", rr.Points3D([xyz], colors=[color], radii=0.006))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode_dir", type=Path)
    ap.add_argument("--no-depth", action="store_true", help="skip depth (faster, less memory)")
    ap.add_argument("--save", type=Path, default=None,
                    help="write a .rrd file instead of spawning the viewer")
    ap.add_argument("--connect", type=str, default=None,
                    help="connect to an already-running rerun viewer at host:port (e.g. 127.0.0.1:9876)")
    args = ap.parse_args()

    ep = args.episode_dir
    if not ep.is_dir():
        sys.exit(f"not a directory: {ep}")

    image, image_status = safe_load(ep / "image.pkl")
    depth, depth_status = (None, "skipped") if args.no_depth else safe_load(ep / "depth.pkl")
    state, state_status = safe_load(ep / "state.pkl")
    eef,   eef_status   = safe_load(ep / "eef_pose.pkl")

    print(f"episode: {ep}")
    print(f"  image.pkl    : {image_status}")
    print(f"  depth.pkl    : {depth_status}")
    print(f"  state.pkl    : {state_status}")
    print(f"  eef_pose.pkl : {eef_status}")
    if not any(x is not None for x in (image, depth, state, eef)):
        sys.exit("no usable data — nothing to log")

    rec_name = f"arx_episode_{ep.name}"
    if args.save is not None:
        rr.init(rec_name)
        rr.save(str(args.save))
    elif args.connect is not None:
        rr.init(rec_name)
        rr.connect_tcp(args.connect)
    else:
        rr.init(rec_name, spawn=True)

    # World axis as a static anchor (helps orient yourself in 3D view).
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/axes", rr.Arrows3D(
        origins=[[0, 0, 0]] * 3,
        vectors=[[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
    ), static=True)

    log_cameras(image, depth, log_depth=not args.no_depth)
    log_joints(state)
    log_eef(eef)

    if args.save is not None:
        print(f"saved -> {args.save}")


if __name__ == "__main__":
    main()
