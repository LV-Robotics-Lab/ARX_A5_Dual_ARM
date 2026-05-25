#!/usr/bin/env python3
"""Tests for sop_episode_verifier — synthesises tiny fixtures + real episodes.

Run:  ~/miniconda3/envs/robo_ctrl/bin/python -m unittest \
          ARX_A5_Dual_ARM/data_collection/test_sop_episode_verifier.py -v
"""

from __future__ import annotations

import pickle
import shutil
import tempfile
import unittest
from pathlib import Path

import cv2
import h5py
import numpy as np

from sop_episode_verifier import (
    CAM_KEYS,
    REQUIRED_FILES,
    VerifierConfig,
    classify_folder,
    iter_episodes,
    verify_episode,
)

RAW_DATA = Path.home() / "workspace" / "raw_data"
KNOWN_GOOD = RAW_DATA / "egg_to_bowl" / "0000"
KNOWN_BROKEN = RAW_DATA / "egg_scoop_v1" / "_failed" / "f0010_broken_no_pkl"
KNOWN_V2_TWO_CAM = RAW_DATA / "egg_scoop_v2" / "0000"


def _write_mp4(path: Path, n_frames: int, w: int, h: int, fps: float,
               black: bool = False, frozen: bool = False) -> None:
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    vw = cv2.VideoWriter(str(path), fourcc, fps, (w, h))
    base = np.zeros((h, w, 3), dtype=np.uint8) if black \
        else np.full((h, w, 3), 128, dtype=np.uint8)
    for i in range(n_frames):
        if frozen:
            frame = base
        elif black:
            frame = base
        else:
            frame = base.copy()
            frame[:, :, 0] = (i % 200) + 30
            frame[:, :, 1] = ((i + 30) % 200) + 30
            frame[:, :, 2] = ((i + 60) % 200) + 30
        vw.write(frame)
    vw.release()


def _write_depth_h5(path: Path, n_frames: int, h: int, w: int,
                    base_ts_ms: int, all_zero: bool = False) -> None:
    with h5py.File(path, "w") as f:
        if all_zero:
            arr = np.zeros((n_frames, h, w), dtype=np.uint16)
        else:
            arr = (np.random.RandomState(0).randint(
                500, 4000, size=(n_frames, h, w))).astype(np.uint16)
        f.create_dataset("depth_mm", data=arr, chunks=(1, h, w))
        ts = base_ts_ms + np.arange(n_frames, dtype=np.int64) * 33
        f.create_dataset("timestamps_ms", data=ts)


def _make_synth_episode(epdir: Path, *, still: bool = True,
                        duration_s: float = 4.0,
                        fps: float = 30.0,
                        skip_files: tuple[str, ...] = (),
                        black_cam: str | None = None,
                        zero_depth_cam: str | None = None) -> None:
    epdir.mkdir(parents=True, exist_ok=True)
    n_video = max(2, int(round(duration_s * fps)))
    # Synthetic resolutions are tiny; the verifier doesn't enforce shape.
    sizes = {"cam_top": (64, 48), "cam_left_wrist": (32, 24),
             "cam_right_wrist": (32, 24)}
    base_ts = 1_700_000_000_000
    for cam in CAM_KEYS:
        w, h = sizes[cam]
        rgb = epdir / f"{cam}_rgb.mp4"
        if rgb.name not in skip_files:
            _write_mp4(rgb, n_video, w, h, fps, black=(cam == black_cam))
        depth = epdir / f"{cam}_depth.h5"
        if depth.name not in skip_files:
            _write_depth_h5(depth, n_video, h, w, base_ts,
                            all_zero=(cam == zero_depth_cam))

    n_state = max(2, int(round(duration_s * 60.0)))
    state_ts = base_ts + (np.arange(n_state, dtype=np.int64)
                          * int(round(1000 / 60)))
    if still:
        joints_l = np.tile(np.linspace(0.0, 0.005, 7), (n_state, 1))
        joints_r = np.tile(np.linspace(0.0, 0.005, 7), (n_state, 1))
    else:
        joints_l = np.tile(np.linspace(0.0, 1.0, n_state)[:, None], (1, 7))
        joints_r = np.tile(np.linspace(0.0, 1.0, n_state)[:, None], (1, 7))
    state = {"left_arm": {"joints": joints_l, "timestamps": state_ts},
             "right_arm": {"joints": joints_r, "timestamps": state_ts}}
    eef_pose = {
        "left_arm":  {"eef_pose": np.zeros((n_state, 7)), "timestamps": state_ts},
        "right_arm": {"eef_pose": np.zeros((n_state, 7)), "timestamps": state_ts},
    }
    image_ts = {cam: base_ts + np.arange(n_video, dtype=np.int64) * 33
                for cam in CAM_KEYS}

    for name, payload in [("state.pkl", state),
                          ("eef_pose.pkl", eef_pose),
                          ("image_timestamps.pkl", image_ts)]:
        if name in skip_files:
            continue
        with open(epdir / name, "wb") as f:
            pickle.dump(payload, f)


class TestClassify(unittest.TestCase):
    def test_success_folder(self):
        self.assertEqual(classify_folder(Path("/x"), "0000"), "success")
        self.assertEqual(classify_folder(Path("/x"), "0103"), "success")

    def test_failed_folder(self):
        self.assertEqual(classify_folder(Path("/x"), "f0001"), "failed")
        self.assertEqual(classify_folder(Path("/x"), "f0010_broken_no_pkl"),
                         "failed")

    def test_other_folder(self):
        self.assertEqual(classify_folder(Path("/x"), "_failed"), "other")
        self.assertEqual(classify_folder(Path("/x"), "random_dir"), "other")


class TestSyntheticEpisodes(unittest.TestCase):
    def setUp(self):
        # Use $HOME for scratch — /tmp on this host shares the (full) root fs.
        scratch_root = Path.home() / ".sop_verifier_tmp"
        scratch_root.mkdir(exist_ok=True)
        self.tmp = Path(tempfile.mkdtemp(prefix="sop_verifier_test_",
                                        dir=str(scratch_root)))
        self.cfg = VerifierConfig(sample_frames=4)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_clean_synth_episode_passes(self):
        ep = self.tmp / "0000"
        _make_synth_episode(ep)
        r = verify_episode(ep, self.cfg)
        self.assertTrue(r["ok"], f"expected OK, got reasons={r['reasons']}")
        self.assertEqual(r["label"], "success")

    def test_missing_pkls_fails_with_reason(self):
        ep = self.tmp / "f0010_broken_no_pkl"
        _make_synth_episode(ep, skip_files=("state.pkl", "eef_pose.pkl",
                                            "image_timestamps.pkl"))
        r = verify_episode(ep, self.cfg)
        self.assertFalse(r["ok"])
        self.assertEqual(r["label"], "failed")
        joined = "; ".join(r["reasons"])
        self.assertIn("missing_files", joined)
        self.assertIn("state.pkl", joined)
        self.assertIn("eef_pose.pkl", joined)
        self.assertIn("image_timestamps.pkl", joined)

    def test_black_top_camera_flagged(self):
        ep = self.tmp / "0001"
        _make_synth_episode(ep, black_cam="cam_top")
        r = verify_episode(ep, self.cfg)
        self.assertFalse(r["ok"])
        self.assertGreater(len(r["checks"]["videos"]["cam_top"]["black_frames"]),
                           0)

    def test_zero_depth_flagged(self):
        ep = self.tmp / "0002"
        _make_synth_episode(ep, zero_depth_cam="cam_left_wrist")
        r = verify_episode(ep, self.cfg)
        self.assertFalse(r["ok"])
        self.assertGreater(
            len(r["checks"]["depths"]["cam_left_wrist"]["zero_frames"]), 0)

    def test_motion_at_end_fails_stillness(self):
        ep = self.tmp / "0003"
        _make_synth_episode(ep, still=False)
        r = verify_episode(ep, self.cfg)
        self.assertFalse(r["ok"])
        self.assertFalse(r["checks"]["stillness"]["ok"])

    def test_missing_top_camera_fails(self):
        ep = self.tmp / "0004"
        _make_synth_episode(ep, skip_files=("cam_top_rgb.mp4",
                                            "cam_top_depth.h5"))
        r = verify_episode(ep, self.cfg)
        self.assertFalse(r["ok"])
        joined = "; ".join(r["reasons"])
        self.assertIn("cam_top_rgb.mp4", joined)
        self.assertIn("cam_top_depth.h5", joined)


@unittest.skipUnless(KNOWN_GOOD.is_dir(),
                     f"requires real data at {KNOWN_GOOD}")
class TestRealEpisodes(unittest.TestCase):
    def test_known_good_passes(self):
        r = verify_episode(KNOWN_GOOD)
        self.assertTrue(r["ok"],
                        f"egg_to_bowl/0000 should pass, got: {r['reasons']}")
        self.assertEqual(r["label"], "success")

    @unittest.skipUnless(KNOWN_BROKEN.is_dir(),
                         f"requires {KNOWN_BROKEN}")
    def test_known_broken_fails_with_missing_pkls(self):
        r = verify_episode(KNOWN_BROKEN)
        self.assertFalse(r["ok"])
        joined = "; ".join(r["reasons"])
        self.assertIn("missing_files", joined)
        self.assertIn(".pkl", joined)
        self.assertEqual(r["label"], "failed")

    @unittest.skipUnless(KNOWN_V2_TWO_CAM.is_dir(),
                         f"requires {KNOWN_V2_TWO_CAM}")
    def test_v2_two_cam_fails_missing_top(self):
        r = verify_episode(KNOWN_V2_TWO_CAM)
        self.assertFalse(r["ok"])
        joined = "; ".join(r["reasons"])
        self.assertIn("cam_top_rgb.mp4", joined)
        self.assertIn("cam_top_depth.h5", joined)


@unittest.skipUnless(RAW_DATA.is_dir(), f"requires raw_data at {RAW_DATA}")
class TestIterEpisodes(unittest.TestCase):
    def test_iter_raw_data_finds_known_episodes(self):
        eps = list(iter_episodes(RAW_DATA))
        names = {(d, p.name) for d, p in eps}
        self.assertIn(("egg_to_bowl", "0000"), names)
        self.assertIn(("egg_scoop_v1", "f0010_broken_no_pkl"), names)


if __name__ == "__main__":
    unittest.main(verbosity=2)
