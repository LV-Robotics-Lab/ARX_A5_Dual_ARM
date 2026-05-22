#!/usr/bin/env python3

"""
Replay a recorded teaching episode on the real arms to verify data quality.

Reads <episode_dir>/state.pkl, sends joint positions + gripper to both arms
through the SingleArm SDK. The first frame is reached via linear interpolation
from the arms' current poses (warmup) — sending the raw recorded[0] directly
would cause a violent jerk because the arms start somewhere arbitrary.

Reads only state.pkl (joints). Cameras, eef_pose, and timestamps.pkl are ignored
— this is a verification tool, not a re-recording tool. eef pose is derived from
joint angles via FK on the arm side anyway, so commanding joints reproduces eef.

Usage:
    # Quick sanity check (no hardware contact):
    python replay_episode.py ~/workspace/raw_data/egg_to_bowl/0000 --dry-run

    # First real run — half speed, right arm only, long warmup:
    python replay_episode.py ~/workspace/raw_data/egg_to_bowl/0000 \\
        --speed 0.5 --warmup-seconds 8 --no-left

    # Full replay at recorded speed:
    python replay_episode.py ~/workspace/raw_data/egg_to_bowl/0000

CAUTION:
    Before running with hardware, clear the area around both arms, keep a hand
    on the e-stop, and prefer --no-left/--no-right + --speed 0.5 for the first
    test of a new episode. See data_replay/README.md for the full SOP.
"""

import argparse
import os
import pickle
import signal
import sys
import time
from typing import Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load_state(episode_dir: str):
    path = os.path.join(episode_dir, 'state.pkl')
    if not os.path.exists(path):
        raise FileNotFoundError(f'state.pkl not found in {episode_dir}')
    with open(path, 'rb') as f:
        state = pickle.load(f)

    out = {}
    for side in ('left_arm', 'right_arm'):
        joints = np.asarray(state[side]['joints'], dtype=np.float64)
        ts_ms = np.asarray(state[side]['timestamps'], dtype=np.int64)
        if joints.shape[0] != ts_ms.shape[0]:
            raise ValueError(f'{side}: joints rows {joints.shape[0]} != ts rows {ts_ms.shape[0]}')
        if joints.shape[1] != 7:
            raise ValueError(f'{side}: expected 7 columns (6 arm + 1 gripper), got {joints.shape[1]}')
        out[side] = {'joints': joints, 'ts_s': (ts_ms - ts_ms[0]) / 1000.0}
    return out


def clip_window(traj, t_start: float, t_end: Optional[float]):
    """Slice the trajectory to [t_start, t_end] (seconds from episode start)."""
    ts = traj['ts_s']
    i0 = int(np.searchsorted(ts, t_start, side='left'))
    i1 = int(np.searchsorted(ts, t_end, side='right')) if t_end is not None else len(ts)
    if i1 <= i0:
        raise ValueError(f'empty window after clipping: i0={i0}, i1={i1}')
    return {'joints': traj['joints'][i0:i1], 'ts_s': traj['ts_s'][i0:i1] - traj['ts_s'][i0]}


def send_arm_step(arm, row: np.ndarray):
    """row is shape (7,): first 6 = arm joints, last = gripper."""
    arm.set_joint_positions(row[:6])
    arm.set_gripper_pos(float(row[6]))


def warmup_to_first_frame(arms: dict, first_rows: dict, duration_s: float, hz: float = 60.0):
    """
    Linearly interpolate from current arm pose to first recorded row over duration_s.

    Without this the first set_joint_positions call jumps from wherever the arm
    happens to be to the recorded start — which on a dual-arm setup can mean
    50cm+ of unplanned motion at the firmware's maximum joint speed.
    """
    starts = {}
    for side, arm in arms.items():
        if arm is None:
            continue
        cur = np.asarray(arm.get_joint_positions(), dtype=np.float64)
        if cur.shape[0] < 7:
            # Pad with zero gripper — shouldn't happen on A5 but be safe.
            cur = np.concatenate([cur, np.zeros(7 - cur.shape[0])])
        starts[side] = cur[:7]

    n_steps = max(1, int(duration_s * hz))
    dt = 1.0 / hz
    t_next = time.time()
    for k in range(1, n_steps + 1):
        alpha = k / n_steps  # 0 → 1
        for side, arm in arms.items():
            if arm is None:
                continue
            row = (1 - alpha) * starts[side] + alpha * first_rows[side]
            send_arm_step(arm, row)
        t_next += dt
        sleep = t_next - time.time()
        if sleep > 0:
            time.sleep(sleep)


def play_trajectory(arms: dict, trajs: dict, speed: float):
    """
    Walk through every recorded sample, pacing via the recorded timestamps so
    pauses in the demonstration are preserved. fixed-rate sleep would either
    drift if arm.set_joint_positions blocks, or compress the demo if the
    recording wasn't exactly 60 Hz.
    """
    # Pick the side that has data as the timeline driver. If both, use left.
    driver = 'left_arm' if 'left_arm' in trajs else 'right_arm'
    ts = trajs[driver]['ts_s']
    n = len(ts)

    wall_t0 = time.time()
    last_print = wall_t0
    print(f'replaying {n} samples, ~{ts[-1]:.1f}s recorded @ {speed:.2f}x = {ts[-1]/speed:.1f}s wall')

    for i in range(n):
        for side, arm in arms.items():
            if arm is None or side not in trajs:
                continue
            send_arm_step(arm, trajs[side]['joints'][i])

        # Pace from recorded timestamp (sub-millisecond accuracy at the
        # cost of one sleep per sample; arms tolerate this fine at 60Hz).
        target_wall = wall_t0 + ts[i] / speed
        sleep = target_wall - time.time()
        if sleep > 0:
            time.sleep(sleep)

        if time.time() - last_print > 2.0:
            print(f'  t={ts[i]:.1f}s  ({i+1}/{n})')
            last_print = time.time()

    print('replay done')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('episode_dir', type=str, help='path to episode dir, e.g. ~/workspace/raw_data/egg_to_bowl/0000')
    ap.add_argument('--speed', type=float, default=1.0, help='playback speed multiplier (0.5 = half speed)')
    ap.add_argument('--warmup-seconds', type=float, default=5.0,
                    help='time to interpolate from current pose to first recorded frame')
    ap.add_argument('--start', type=float, default=0.0, help='seconds from episode start')
    ap.add_argument('--end', type=float, default=None, help='seconds from episode start (default: to end)')
    ap.add_argument('--no-left', action='store_true', help='skip left arm')
    ap.add_argument('--no-right', action='store_true', help='skip right arm')
    ap.add_argument('--left-can', type=str, default='can1')
    ap.add_argument('--right-can', type=str, default='can3')
    ap.add_argument('--urdf-name', type=str, default='a5.urdf')
    ap.add_argument('--dry-run', action='store_true', help='load data and print summary, no hardware')
    args = ap.parse_args()

    if args.no_left and args.no_right:
        sys.exit('refused: both arms disabled, nothing to do')
    if args.speed <= 0:
        sys.exit('--speed must be > 0')

    ep = os.path.expanduser(args.episode_dir)
    data = load_state(ep)

    trajs = {}
    if not args.no_left:
        trajs['left_arm'] = clip_window(data['left_arm'], args.start, args.end)
    if not args.no_right:
        trajs['right_arm'] = clip_window(data['right_arm'], args.start, args.end)

    print(f'episode: {ep}')
    for side, t in trajs.items():
        joints = t['joints']
        ts = t['ts_s']
        print(f'  {side}: {joints.shape[0]} samples, {ts[-1]:.2f}s, '
              f'gripper range=[{joints[:, 6].min():.3f}, {joints[:, 6].max():.3f}]')

    if args.dry_run:
        print('dry-run: not connecting to arms')
        return

    # Import here so --dry-run works on machines without arx_r5_python.
    from A5.bimanual import SingleArm

    arms = {'left_arm': None, 'right_arm': None}
    if not args.no_left:
        arms['left_arm'] = SingleArm({'can_port': args.left_can, 'urdf_name': args.urdf_name})
    if not args.no_right:
        arms['right_arm'] = SingleArm({'can_port': args.right_can, 'urdf_name': args.urdf_name})

    # SIGINT during replay → stop sending commands. We don't try to "park" the
    # arm because that needs another planned motion; safer to let the firmware
    # hold the last commanded position so the operator can decide via e-stop.
    interrupted = {'flag': False}
    def _on_sigint(signum, frame):
        interrupted['flag'] = True
        print('\nSIGINT received — stopping after current step (use e-stop if you need it NOW)')
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _on_sigint)

    try:
        first_rows = {side: trajs[side]['joints'][0] for side in trajs}
        print(f'warmup: interpolating to first frame over {args.warmup_seconds:.1f}s')
        warmup_to_first_frame(arms, first_rows, args.warmup_seconds)

        play_trajectory(arms, trajs, args.speed)
    except KeyboardInterrupt:
        print('replay aborted')


if __name__ == '__main__':
    main()
