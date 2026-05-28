"""
Apply gripper hold correction to all valid episodes and write corrected
state.pkl files to a separate output directory.

Original files are NEVER modified. The corrected directory mirrors the
episode numbering of the source, but only contains state.pkl — all other
files (mp4, h5, eef_pose.pkl, image_timestamps.pkl) stay in the source dir
and should be read from there by the training pipeline.

Correction logic (same as replay_episode.py --grip-hold-target):
    If recorded gripper value g > grip_hold_thresh  (i.e. "hold intent" zone,
    close to 0 = closed), replace with grip_hold_target so the motor chases a
    value the object physically blocks → sustained grip torque during training.

Usage:
    python data_replay/apply_grip_correction.py \\
        ~/workspace/raw_data/egg_to_bowl \\
        ~/workspace/raw_data/corrected_data

    # custom threshold / target:
    python data_replay/apply_grip_correction.py \\
        ~/workspace/raw_data/egg_to_bowl \\
        ~/workspace/raw_data/corrected_data \\
        --grip-hold-thresh -1.0 --grip-hold-target 0.0

    # dry-run (print summary, write nothing):
    python data_replay/apply_grip_correction.py ... --dry-run
"""
import argparse
import os
import pickle
import re
import sys

import numpy as np


EPISODE_RE = re.compile(r'^\d{4}$')


def apply_correction(joints: np.ndarray, thresh: float, target: float) -> np.ndarray:
    """Return a copy of joints with gripper column (col 6) corrected."""
    out = joints.copy()
    mask = out[:, 6] > thresh
    out[mask, 6] = target
    return out


def process_episode(src_dir: str, dst_dir: str,
                    thresh: float, target: float, dry_run: bool) -> dict:
    state_src = os.path.join(src_dir, 'state.pkl')
    if not os.path.exists(state_src):
        return {'skipped': 'no state.pkl'}

    with open(state_src, 'rb') as f:
        state = pickle.load(f)

    result = {}
    new_state = {}
    for side in ('left_arm', 'right_arm'):
        if side not in state:
            new_state[side] = state[side]
            continue
        joints = np.asarray(state[side]['joints'], dtype=np.float64)
        corrected = apply_correction(joints, thresh, target)
        n_changed = int((corrected[:, 6] != joints[:, 6]).sum())
        result[side] = {'total': len(joints), 'changed': n_changed}
        new_state[side] = {
            'joints': corrected,
            'timestamps': state[side]['timestamps'],
        }

    if not dry_run:
        os.makedirs(dst_dir, exist_ok=True)
        with open(os.path.join(dst_dir, 'state.pkl'), 'wb') as f:
            pickle.dump(new_state, f)

    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src_root', help='source episode root, e.g. ~/workspace/raw_data/egg_to_bowl')
    ap.add_argument('dst_root', help='output root, e.g. ~/workspace/raw_data/corrected_data')
    ap.add_argument('--grip-hold-thresh', type=float, default=-1.0,
                    help='gripper values ABOVE this are treated as hold-intent (default: -1.0)')
    ap.add_argument('--grip-hold-target', type=float, default=0.0,
                    help='value to write for hold-intent frames (default: 0.0)')
    ap.add_argument('--dry-run', action='store_true',
                    help='print summary only, do not write any files')
    args = ap.parse_args()

    src = os.path.expanduser(args.src_root)
    dst = os.path.expanduser(args.dst_root)

    if not os.path.isdir(src):
        sys.exit(f'source directory not found: {src}')

    episodes = sorted(e for e in os.listdir(src) if EPISODE_RE.match(e))
    if not episodes:
        sys.exit(f'no NNNN episode directories found in {src}')

    print(f'source : {src}')
    print(f'output : {dst}')
    print(f'thresh : g > {args.grip_hold_thresh}  →  {args.grip_hold_target}')
    print(f'episodes: {len(episodes)}  ({episodes[0]} – {episodes[-1]})')
    if args.dry_run:
        print('DRY-RUN: nothing will be written\n')

    total_changed = 0
    skipped = 0
    for ep in episodes:
        src_ep = os.path.join(src, ep)
        dst_ep = os.path.join(dst, ep)
        result = process_episode(src_ep, dst_ep,
                                 args.grip_hold_thresh, args.grip_hold_target,
                                 args.dry_run)
        if 'skipped' in result:
            print(f'  {ep}  SKIPPED ({result["skipped"]})')
            skipped += 1
            continue
        parts = []
        for side, info in result.items():
            parts.append(f'{side}: {info["changed"]}/{info["total"]} frames corrected')
            total_changed += info['changed']
        print(f'  {ep}  {" | ".join(parts)}')

    print(f'\ndone: {len(episodes) - skipped} episodes processed, '
          f'{skipped} skipped, {total_changed} gripper frames corrected total')
    if not args.dry_run:
        print(f'corrected state.pkl files written to: {dst}')


if __name__ == '__main__':
    main()
