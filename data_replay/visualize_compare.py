"""
Compare a teaching episode with a replay-recorded episode in ONE Rerun viewer.

Entity paths are namespaced so both episodes show up side-by-side without
collision:
    teach episode → /teach/...
    replay episode → /r<name>/...     (e.g. /r0002/, /r0002_v1/)

By default the replay's time axis is scaled so its trajectory duration
matches the teaching's duration — this is how comparison is meaningful when
the replay ran at --speed 0.5. Disable with --no-time-align if you want each
to play at its own wall-clock pace.

Usage:
    python data_replay/visualize_compare.py \\
        ~/workspace/raw_data/egg_to_bowl/0002 \\
        ~/workspace/raw_data/egg_to_bowl_replayed/0002

    # save to .rrd instead of spawning viewer:
    python data_replay/visualize_compare.py teach_dir replay_dir --save out.rrd

    # don't normalize replay's time:
    python data_replay/visualize_compare.py teach_dir replay_dir --no-time-align
"""
import argparse
import os
import pickle
import subprocess
import sys
from pathlib import Path

import cv2
import h5py
import numpy as np
import rerun as rr


def _rel(ts_ms, t0_ms, scale=1.0):
    return ((ts_ms - t0_ms) / 1000.0) * scale


def safe_load_pickle(path: Path):
    if not path.exists():
        return None
    try:
        with open(path, 'rb') as f:
            return pickle.load(f)
    except Exception as e:
        print(f'  failed to load {path.name}: {e}')
        return None


def discover_cameras(ep_dir: Path):
    cams = set()
    for p in ep_dir.glob('*_rgb.mp4'):
        cams.add(p.stem[:-len('_rgb')])
    for p in ep_dir.glob('*_depth.h5'):
        cams.add(p.stem[:-len('_depth')])
    return sorted(cams)


def compute_t0_and_last_ms(ep_dir: Path):
    """Return (t0_ms, t_last_ms) by scanning all stored timestamps."""
    firsts, lasts = [], []
    for fname in ('state.pkl', 'eef_pose.pkl', 'image_timestamps.pkl'):
        d = safe_load_pickle(ep_dir / fname)
        if d is None:
            continue
        for key, v in d.items():
            if isinstance(v, dict):
                ts = v.get('timestamps')
                if ts is not None and len(ts) > 0:
                    firsts.append(ts[0]); lasts.append(ts[-1])
            elif isinstance(v, list) and len(v) > 0:
                firsts.append(v[0]); lasts.append(v[-1])
    # Also scan depth h5 timestamps_ms datasets.
    for cam in discover_cameras(ep_dir):
        h5p = ep_dir / f'{cam}_depth.h5'
        if h5p.exists():
            try:
                with h5py.File(h5p, 'r') as f:
                    if 'timestamps_ms' in f and len(f['timestamps_ms']) > 0:
                        firsts.append(int(f['timestamps_ms'][0]))
                        lasts.append(int(f['timestamps_ms'][-1]))
            except Exception:
                pass
    if not firsts:
        return 0, 0
    return min(firsts), max(lasts)


# ---------------- logging functions (prefix-aware) ----------------

def log_rgb(ep_dir, cam_keys, t0_ms, prefix, scale=1.0):
    ts_map = safe_load_pickle(ep_dir / 'image_timestamps.pkl') or {}
    for cam_key in cam_keys:
        mp4 = ep_dir / f'{cam_key}_rgb.mp4'
        if not mp4.exists():
            continue
        stamps = ts_map.get(cam_key, [])
        cap = cv2.VideoCapture(str(mp4))
        path = f'{prefix}/world/{cam_key}/rgb'
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if i < len(stamps):
                rr.set_time_seconds('time', _rel(stamps[i], t0_ms, scale))
            else:
                rr.set_time_seconds('time', i / 30.0)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rr.log(path, rr.Image(rgb))
            i += 1
        cap.release()


def log_depth(ep_dir, cam_keys, t0_ms, prefix, scale=1.0):
    for cam_key in cam_keys:
        h5p = ep_dir / f'{cam_key}_depth.h5'
        if not h5p.exists():
            continue
        path = f'{prefix}/world/{cam_key}/depth'
        try:
            with h5py.File(h5p, 'r') as f:
                depth = f['depth_mm']
                ts = f['timestamps_ms']
                n = min(depth.shape[0], ts.shape[0])
                for i in range(n):
                    rr.set_time_seconds('time', _rel(ts[i], t0_ms, scale))
                    rr.log(path, rr.DepthImage(depth[i], meter=1000.0))
        except Exception as e:
            print(f'  skipped {h5p.name}: {type(e).__name__}: {e}')


def log_joints(state, t0_ms, prefix, scale=1.0):
    if state is None:
        return
    for side in ('left_arm', 'right_arm'):
        if side not in state:
            continue
        joints_list = state[side]['joints']
        stamps = state[side]['timestamps']
        for joints, ts_ms in zip(joints_list, stamps):
            rr.set_time_seconds('time', _rel(ts_ms, t0_ms, scale))
            arr = np.asarray(joints).ravel()
            for j, v in enumerate(arr):
                rr.log(f'{prefix}/plots/{side}/joint{j}', rr.Scalar(float(v)))


def log_eef(eef, t0_ms, prefix, scale=1.0, color_offset=0):
    if eef is None:
        return
    base_colors = {'left_arm': [220, 60, 60], 'right_arm': [60, 120, 220]}
    for side in ('left_arm', 'right_arm'):
        if side not in eef:
            continue
        poses = eef[side]['eef_pose']
        stamps = eef[side]['timestamps']
        col = base_colors[side]
        col = [max(0, min(255, c + color_offset)) for c in col]
        path = f'{prefix}/world/{side}/eef'
        for pose, ts_ms in zip(poses, stamps):
            p = np.asarray(pose).ravel()
            xyz = p[:3]
            qw, qx, qy, qz = p[3], p[4], p[5], p[6]
            rr.set_time_seconds('time', _rel(ts_ms, t0_ms, scale))
            rr.log(path, rr.Transform3D(
                translation=xyz,
                rotation=rr.Quaternion(xyzw=[qx, qy, qz, qw]),
                axis_length=0.1,
            ))
            rr.log(f'{prefix}/world/{side}/eef_pt',
                   rr.Points3D([xyz], colors=[col], radii=0.006))


def log_episode(ep_dir, prefix, t0_ms, scale=1.0, no_depth=False, color_offset=0):
    cams = discover_cameras(ep_dir)
    state = safe_load_pickle(ep_dir / 'state.pkl')
    eef = safe_load_pickle(ep_dir / 'eef_pose.pkl')
    dur_s = ((compute_t0_and_last_ms(ep_dir)[1] - t0_ms) / 1000.0) * scale
    print(f'[{prefix}] {ep_dir.name}  cams={cams}  state={"ok" if state else "MISSING"}  '
          f'eef={"ok" if eef else "MISSING"}  scale={scale:.3f}  shown duration={dur_s:.2f}s')
    log_rgb(ep_dir, cams, t0_ms, prefix, scale)
    if not no_depth:
        log_depth(ep_dir, cams, t0_ms, prefix, scale)
    log_joints(state, t0_ms, prefix, scale)
    log_eef(eef, t0_ms, prefix, scale, color_offset=color_offset)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('teach_dir', type=Path)
    ap.add_argument('replay_dir', type=Path)
    ap.add_argument('--no-depth', action='store_true', help='skip depth (faster, less memory)')
    ap.add_argument('--save', type=Path, default=None,
                    help='save .rrd to this path instead of spawning a viewer')
    ap.add_argument('--no-time-align', action='store_true',
                    help='do NOT scale replay time to match teach duration. '
                         'default: replay timestamps get scaled so its duration matches teach')
    args = ap.parse_args()

    teach = args.teach_dir.expanduser().resolve()
    replay = args.replay_dir.expanduser().resolve()
    if not teach.is_dir():
        sys.exit(f'no such teach dir: {teach}')
    if not replay.is_dir():
        sys.exit(f'no such replay dir: {replay}')

    replay_tag = f'r{replay.name}'  # e.g. r0002, r0002_v1

    teach_t0, teach_last = compute_t0_and_last_ms(teach)
    replay_t0, replay_last = compute_t0_and_last_ms(replay)
    teach_dur = (teach_last - teach_t0) / 1000.0
    replay_dur = (replay_last - replay_t0) / 1000.0
    print(f'teach  duration = {teach_dur:.2f}s')
    print(f'replay duration = {replay_dur:.2f}s')

    if not args.no_time_align and teach_dur > 0 and replay_dur > 0:
        replay_scale = teach_dur / replay_dur
        print(f'time-aligning replay: scale = {replay_scale:.3f}')
    else:
        replay_scale = 1.0

    rr.init('arx_compare', spawn=False)
    rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log('world/axes', rr.Arrows3D(
        vectors=[[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
    ), static=True)

    log_episode(teach, prefix='teach', t0_ms=teach_t0, scale=1.0,
                no_depth=args.no_depth, color_offset=0)
    log_episode(replay, prefix=replay_tag, t0_ms=replay_t0, scale=replay_scale,
                no_depth=args.no_depth, color_offset=-60)  # darker shade for replay

    if args.save:
        rr.save(str(args.save))
        print(f'saved → {args.save}')
        return

    out = Path(f'/tmp/arx_compare_{teach.name}_vs_{replay_tag}.rrd')
    rr.save(str(out))
    print(f'opening viewer on {out}')
    subprocess.Popen(['rerun', str(out)])


if __name__ == '__main__':
    main()
