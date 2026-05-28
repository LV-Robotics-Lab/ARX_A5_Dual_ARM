#!/usr/bin/env python3
"""
Convert egg_to_bowl_correct episodes → LeRobot v2 dataset format for Pi0 fine-tuning.

Input layout (per episode):
    <src_dir>/NNNN/
        state.pkl            right_arm joints N×7 float64 + timestamps N int64 ms
        image_timestamps.pkl {cam_key: [ms, ...]}   (1:1 with video frames)
        cam_top_rgb.mp4
        cam_left_wrist_rgb.mp4
        cam_right_wrist_rgb.mp4

Output layout:
    <dst_dir>/
        meta/
            info.json        dataset-level metadata + alignment notes
            episodes.jsonl   one line per episode
            stats.json       per-feature mean/std/min/max for normalization
        data/
            chunk-000/
                episode_000000.parquet
                ...
        videos/
            chunk-000/
                observation.images.top/episode_000000.mp4
                observation.images.left_wrist/episode_000000.mp4
                observation.images.right_wrist/episode_000000.mp4

Time alignment design (see 数据管线与训练方案.md §6.2):
    - cam_top timestamps → reference timeline (~30 Hz, std 1.1 ms)
    - Joint states aligned to cam_top via nearest-neighbour (max error ~16 ms at 60 Hz)
    - cam_left_wrist / cam_right_wrist are systematically ~20 ms later than cam_top
      (separate USB devices, hardware-level offset); NOT corrected here — recorded in info.json.
    - No camera-to-joint latency correction applied; Pi0 action chunking tolerates <50 ms offset.

Action space:
    7-dim absolute joint positions [j1..j6, gripper] from corrected state.pkl.
    Gripper hold phase already corrected to 0.0 (see apply_grip_correction.py).

Usage:
    python convert_to_lerobot.py \\
        /media/lv-robotics/"My PSSD"/Feibo/Data/egg_to_bowl_correct \\
        /media/lv-robotics/"My PSSD"/Feibo/Data/egg_to_bowl_lerobot

    # Dry-run single episode:
    python convert_to_lerobot.py <src> <dst> --episodes 2 --dry-run
"""

import argparse
import json
import os
import pickle
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ARM_SIDE = 'right_arm'
CAMERA_KEYS = ['top', 'left_wrist', 'right_wrist']
VIDEO_FPS = 30.0
CHUNK_SIZE = 1000   # episodes per data/videos chunk folder
TASK_DESCRIPTION = 'Pick up the egg and place it into the bowl.'

# Measured camera-to-camera offsets (ms), cam_top as reference = 0.
# Positive = that camera is later than cam_top.
# Source: data inspection on episode 0002 (mean over 769 frames).
CAM_OFFSETS_MS = {
    'top': 0.0,
    'left_wrist': 21.7,
    'right_wrist': 20.1,
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_episode(ep_dir: Path):
    with open(ep_dir / 'state.pkl', 'rb') as f:
        state = pickle.load(f)
    with open(ep_dir / 'image_timestamps.pkl', 'rb') as f:
        img_ts = pickle.load(f)
    return state, img_ts


def align_joints_to_camera(joints: np.ndarray, joint_ts_ms: np.ndarray,
                            cam_ts_ms: np.ndarray) -> np.ndarray:
    """
    Nearest-neighbour match: for each camera timestamp find closest joint row.
    Returns float32 array (n_cam_frames, 7).

    Max alignment error = half a joint-state interval ≈ 8 ms at 60 Hz.
    No latency correction applied — see 数据管线与训练方案.md §6.2.
    """
    jts = joint_ts_ms.astype(np.float64)
    cts = cam_ts_ms.astype(np.float64)
    # For each camera timestamp, find index of nearest joint timestamp
    indices = np.argmin(np.abs(cts[:, None] - jts[None, :]), axis=1)
    return joints[indices].astype(np.float32)


def copy_video(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


# ---------------------------------------------------------------------------
# Per-episode conversion
# ---------------------------------------------------------------------------

def convert_episode(ep_dir: Path, ep_idx: int, dst_dir: Path,
                    dry_run: bool = False):
    """
    Convert one episode directory.
    Returns (records list, n_frames) or raises on error.
    """
    state, img_ts = load_episode(ep_dir)

    joints = np.asarray(state[ARM_SIDE]['joints'], dtype=np.float64)
    joint_ts = np.asarray(state[ARM_SIDE]['timestamps'], dtype=np.float64)

    # Reference timeline: cam_top (earliest, most stable)
    cam_top_ts = np.asarray(img_ts['cam_top'], dtype=np.float64)
    n_frames = len(cam_top_ts)

    # Sanity: video frame count should match timestamp count
    for ck in CAMERA_KEYS:
        pkl_key = f'cam_{ck}'
        n_ts = len(img_ts.get(pkl_key, img_ts.get(ck, [])))
        if abs(n_ts - n_frames) > 2:
            print(f'    WARNING: {ck} has {n_ts} timestamps vs cam_top {n_frames}')

    # Align joints to cam_top timestamps
    aligned = align_joints_to_camera(joints, joint_ts, cam_top_ts)  # (n_frames, 7)

    # Timestamps relative to episode start (seconds)
    ts_rel = (cam_top_ts - cam_top_ts[0]) / 1000.0

    chunk_str = f'chunk-{ep_idx // CHUNK_SIZE:03d}'
    ep_str = f'episode_{ep_idx:06d}'

    if not dry_run:
        # Copy all three camera videos
        for ck in CAMERA_KEYS:
            src_mp4 = ep_dir / f'cam_{ck}_rgb.mp4'
            dst_mp4 = (dst_dir / 'videos' / chunk_str
                       / f'observation.images.{ck}' / f'{ep_str}.mp4')
            if src_mp4.exists():
                copy_video(src_mp4, dst_mp4)
            else:
                print(f'    WARNING: {src_mp4} not found, skipping')

    # Build per-frame records (one row = one camera frame = one training sample)
    records = []
    for i in range(n_frames):
        records.append({
            'episode_index':      ep_idx,
            'frame_index':        i,
            'timestamp':          float(ts_rel[i]),
            'observation.state':  aligned[i].tolist(),   # 7-dim corrected joints
            'action':             aligned[i].tolist(),   # same: position-ctrl, cmd ≈ state
            'next.done':          bool(i == n_frames - 1),
            'task_index':         0,
        })

    if not dry_run:
        parquet_dir = dst_dir / 'data' / chunk_str
        parquet_dir.mkdir(parents=True, exist_ok=True)
        df = pd.DataFrame(records)
        # Store list columns as object arrays (LeRobot convention)
        df.to_parquet(parquet_dir / f'{ep_str}.parquet', index=False)

    return records, n_frames


# ---------------------------------------------------------------------------
# Stats
# ---------------------------------------------------------------------------

def compute_stats(all_arrays: list[np.ndarray]) -> dict:
    arr = np.concatenate(all_arrays, axis=0)
    return {
        'mean': arr.mean(axis=0).tolist(),
        'std':  arr.std(axis=0).tolist(),
        'min':  arr.min(axis=0).tolist(),
        'max':  arr.max(axis=0).tolist(),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src_dir', help='egg_to_bowl_correct base directory')
    ap.add_argument('dst_dir', help='output LeRobot dataset directory')
    ap.add_argument('--episodes', nargs='+', type=int, default=None,
                    help='specific episode indices to convert (default: all)')
    ap.add_argument('--dry-run', action='store_true',
                    help='parse data and print summary without writing output')
    args = ap.parse_args()

    src = Path(args.src_dir)
    dst = Path(args.dst_dir)

    if not args.dry_run:
        dst.mkdir(parents=True, exist_ok=True)
        (dst / 'meta').mkdir(exist_ok=True)

    # Discover episodes
    if args.episodes is not None:
        ep_dirs = [src / f'{i:04d}' for i in sorted(args.episodes)]
    else:
        ep_dirs = sorted(src.glob('[0-9][0-9][0-9][0-9]'))
    ep_dirs = [d for d in ep_dirs if d.is_dir()]

    if not ep_dirs:
        sys.exit(f'No episode directories found in {src}')

    print(f'{"[DRY-RUN] " if args.dry_run else ""}Converting {len(ep_dirs)} episodes')
    print(f'  src: {src}')
    print(f'  dst: {dst}')

    all_states, all_actions = [], []
    episode_meta = []
    total_frames = 0
    failed = []

    for ep_idx, ep_dir in enumerate(ep_dirs):
        ep_str = f'episode_{ep_idx:06d}'
        print(f'  [{ep_idx+1:3d}/{len(ep_dirs)}] {ep_dir.name} → {ep_str}', end='', flush=True)

        try:
            records, n_frames = convert_episode(ep_dir, ep_idx, dst,
                                                dry_run=args.dry_run)
        except Exception as e:
            print(f'  ERROR: {e}')
            failed.append(ep_dir.name)
            continue

        states = np.array([r['observation.state'] for r in records], dtype=np.float32)
        all_states.append(states)
        all_actions.append(states)

        episode_meta.append({
            'episode_index':  ep_idx,
            'tasks':          [TASK_DESCRIPTION],
            'length':         n_frames,
            'source_episode': ep_dir.name,
        })
        total_frames += n_frames
        print(f'  {n_frames} frames')

    if args.dry_run:
        print(f'\n[DRY-RUN] Would write {len(episode_meta)} episodes, {total_frames} frames')
        if failed:
            print(f'Would skip: {failed}')
        return

    # ---- meta/episodes.jsonl ----
    with open(dst / 'meta' / 'episodes.jsonl', 'w') as f:
        for m in episode_meta:
            f.write(json.dumps(m, ensure_ascii=False) + '\n')

    # ---- meta/stats.json ----
    stats = {
        'observation.state': compute_stats(all_states),
        'action':            compute_stats(all_actions),
    }
    with open(dst / 'meta' / 'stats.json', 'w') as f:
        json.dump(stats, f, indent=2)

    # ---- meta/info.json ----
    # Detect image shape from first episode video (best-effort)
    try:
        import cv2
        sample_mp4 = list((dst / 'videos' / 'chunk-000' / 'observation.images.top').glob('*.mp4'))[0]
        cap = cv2.VideoCapture(str(sample_mp4))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        cap.release()
    except Exception:
        h, w = 480, 640

    info = {
        'codebase_version': 'v2.0',
        'robot_type': 'arx_a5_right_arm',
        'total_episodes': len(episode_meta),
        'total_frames': total_frames,
        'fps': VIDEO_FPS,
        'tasks': [{'task_index': 0, 'task': TASK_DESCRIPTION}],
        'features': {
            'observation.state': {
                'dtype': 'float32',
                'shape': [7],
                'names': ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'gripper'],
                'info': 'corrected commanded joint positions (rad); gripper: 0=closed, -3.35=open',
            },
            'action': {
                'dtype': 'float32',
                'shape': [7],
                'names': ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'gripper'],
                'info': 'same as observation.state (position-ctrl: command ≈ state)',
            },
            'observation.images.top': {
                'dtype': 'video',
                'shape': [h, w, 3],
                'video_info': {'video.fps': VIDEO_FPS, 'video.codec': 'mp4v', 'video.pix_fmt': 'bgr24'},
            },
            'observation.images.left_wrist': {
                'dtype': 'video',
                'shape': [h, w, 3],
                'video_info': {'video.fps': VIDEO_FPS, 'video.codec': 'mp4v', 'video.pix_fmt': 'bgr24'},
            },
            'observation.images.right_wrist': {
                'dtype': 'video',
                'shape': [h, w, 3],
                'video_info': {'video.fps': VIDEO_FPS, 'video.codec': 'mp4v', 'video.pix_fmt': 'bgr24'},
            },
        },
        'time_alignment': {
            'reference_camera': 'cam_top',
            'method': 'nearest_neighbour',
            'joint_hz': 60,
            'camera_hz': 30,
            'max_joint_alignment_error_ms': 16.7,
            'camera_offsets_ms': CAM_OFFSETS_MS,
            'camera_offset_note': (
                'left_wrist and right_wrist are systematically ~20 ms later than cam_top '
                '(separate USB devices). Not corrected; consistent across episodes.'
            ),
            'latency_correction_ms': 0,
            'latency_note': (
                'Camera-to-joint system latency not calibrated. '
                'Pi0 action chunking tolerates <50 ms offset. '
                'See 数据管线与训练方案.md §6.2 for calibration plan.'
            ),
        },
        'gripper_correction': {
            'description': 'hold-phase gripper command set to 0.0 to maintain grip torque',
            'hold_thresh': -1.0,
            'hold_target': 0.0,
            'total_frames_corrected': 151834,
            'script': 'data_replay/apply_grip_correction.py',
        },
        'source': str(src),
    }
    with open(dst / 'meta' / 'info.json', 'w') as f:
        json.dump(info, f, indent=2, ensure_ascii=False)

    print(f'\n完成：{len(episode_meta)} episodes，{total_frames} frames')
    if failed:
        print(f'跳过（错误）：{failed}')
    print(f'输出目录：{dst}')


if __name__ == '__main__':
    main()
