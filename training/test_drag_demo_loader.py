#!/usr/bin/env python3
"""Tests for drag_demo_loader.

Run:
  cd ~/workspace/ARX_A5_Dual_ARM/training && \
    ~/miniconda3/envs/robo_ctrl/bin/python -m unittest \
      test_drag_demo_loader -v
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import torch
from torch.utils.data import DataLoader

THIS = Path(__file__).resolve().parent
if str(THIS) not in sys.path:
    sys.path.insert(0, str(THIS))

from drag_demo_loader import (  # noqa: E402
    ALLOWED_MODES, CAM_KEYS, DragDemoDataset, SplitConfig, collate
)

MANIFEST_PATH = (Path.home() / "workspace" / ".verifier_reports"
                 / "initial_batch_manifest.json")
SPLIT_CONFIG = THIS / "split_config.json"


def _scratch_dir() -> Path:
    root = Path.home() / ".sop_verifier_tmp"
    root.mkdir(exist_ok=True)
    return Path(tempfile.mkdtemp(prefix="loader_test_", dir=str(root)))


class TestSplitConfig(unittest.TestCase):
    def test_parses_default(self):
        cfg = SplitConfig.from_json(SPLIT_CONFIG)
        self.assertIn("drag", cfg.sources)
        self.assertIn("teleop", cfg.sources)
        self.assertTrue(cfg.sources["drag"].available
                        or cfg.sources["drag"].manifest_path is not None)
        for m in ALLOWED_MODES:
            self.assertIn(m, cfg.modes)

    def test_resolve_mode_unknown(self):
        cfg = SplitConfig.from_json(SPLIT_CONFIG)
        with self.assertRaises(ValueError):
            cfg.resolve_mode("does_not_exist")

    def test_mode_shares_sum_to_one(self):
        cfg = SplitConfig.from_json(SPLIT_CONFIG)
        for mode_name, shares in cfg.modes.items():
            self.assertAlmostEqual(sum(shares.values()), 1.0, places=6,
                                   msg=f"mode {mode_name} shares != 1")


@unittest.skipUnless(MANIFEST_PATH.is_file(),
                     f"requires manifest at {MANIFEST_PATH}")
class TestRealDragManifest(unittest.TestCase):
    def setUp(self):
        self.cfg = SplitConfig.from_json(SPLIT_CONFIG)

    def test_drag_only_yields_episodes(self):
        ds = DragDemoDataset(self.cfg, "drag_only", samples_per_episode=2,
                             image_size=(64, 48))
        r = ds.report()
        self.assertGreater(r.n_total_episodes, 0,
                           "drag_only should have episodes from the manifest")
        self.assertEqual(r.missing_streams, [],
                         f"unexpected missing_streams: {r.missing_streams}")
        self.assertGreater(len(ds), 0)

    def test_teleop_only_empty_when_no_teleop_manifest(self):
        ds = DragDemoDataset(self.cfg, "teleop_only", samples_per_episode=2)
        r = ds.report()
        self.assertEqual(r.n_total_episodes, 0)
        self.assertTrue(any(item.get("source") == "teleop"
                            for item in r.missing_streams))

    def test_mixed_refuses_silent_degrade(self):
        ds = DragDemoDataset(self.cfg, "mixed_70_30", samples_per_episode=2)
        r = ds.report()
        self.assertEqual(r.n_total_episodes, 0,
                         "mixed must not silently degrade to drag-only")
        self.assertTrue(any(item.get("source") == "teleop"
                            for item in r.missing_streams))

    def test_batch_shapes_drag_only(self):
        ds = DragDemoDataset(self.cfg, "drag_only", samples_per_episode=2,
                             image_size=(64, 48))
        loader = DataLoader(ds, batch_size=2, shuffle=False,
                            collate_fn=collate, num_workers=0)
        batch = next(iter(loader))
        self.assertEqual(batch["state"].shape, (2, 14))
        self.assertEqual(batch["action"].shape, (2, 14))
        self.assertEqual(batch["state"].dtype, torch.float32)
        for cam in CAM_KEYS:
            self.assertEqual(batch["rgb"][cam].shape, (2, 48, 64, 3))
            self.assertEqual(batch["rgb"][cam].dtype, torch.uint8)
        self.assertEqual(len(batch["meta"]), 2)
        for m in batch["meta"]:
            self.assertEqual(m["source"], "drag")


@unittest.skipUnless(MANIFEST_PATH.is_file(),
                     f"requires manifest at {MANIFEST_PATH}")
class TestMixedWithSyntheticTeleop(unittest.TestCase):
    """Point both sources at the same real manifest, sub-sliced, to verify
    that mixed_70_30 actually mixes and tags each sample with its source."""

    def setUp(self):
        self.tmp = _scratch_dir()
        full = json.loads(MANIFEST_PATH.read_text())
        eps = full["counted_episodes"]
        self.assertGreaterEqual(len(eps), 4,
                                "need >=4 real episodes for the mixture test")
        # Split the 19-episode manifest into two synthetic halves.
        half = max(2, len(eps) // 2)
        drag_eps = eps[:half]
        teleop_eps = eps[half:half * 2] if half * 2 <= len(eps) else eps[half:]
        drag_manifest = dict(full); drag_manifest["counted_episodes"] = drag_eps
        teleop_manifest = dict(full); teleop_manifest["counted_episodes"] = teleop_eps
        self.drag_path = self.tmp / "drag.json"
        self.teleop_path = self.tmp / "teleop.json"
        self.drag_path.write_text(json.dumps(drag_manifest))
        self.teleop_path.write_text(json.dumps(teleop_manifest))
        self.split_path = self.tmp / "split.json"
        self.split_path.write_text(json.dumps({
            "schema_version": "drag_demo_split.v1",
            "sources": {
                "drag":   {"manifest": str(self.drag_path),
                           "episode_filter": "counted_episodes"},
                "teleop": {"manifest": str(self.teleop_path),
                           "episode_filter": "counted_episodes"},
            },
            "modes": {
                "drag_only":   {"drag": 1.0, "teleop": 0.0},
                "teleop_only": {"drag": 0.0, "teleop": 1.0},
                "mixed_70_30": {"drag": 0.7, "teleop": 0.3},
            },
        }))

    def test_mixed_yields_both_sources(self):
        ds = DragDemoDataset(SplitConfig.from_json(self.split_path),
                             "mixed_70_30", samples_per_episode=2,
                             image_size=(64, 48))
        r = ds.report()
        drag_count = r.sources_summary["drag"]["n_episodes_after_mixture"]
        teleop_count = r.sources_summary["teleop"]["n_episodes_after_mixture"]
        self.assertGreater(drag_count, 0)
        self.assertGreater(teleop_count, 0)
        # 70/30 ratio should hold within +/- 1 episode after rounding.
        self.assertAlmostEqual(drag_count / (drag_count + teleop_count),
                               0.7, delta=0.1)
        loader = DataLoader(ds, batch_size=min(8, len(ds)), shuffle=False,
                            collate_fn=collate, num_workers=0)
        sources_seen = set()
        for batch in loader:
            for m in batch["meta"]:
                sources_seen.add(m["source"])
            if {"drag", "teleop"}.issubset(sources_seen):
                break
        self.assertEqual(sources_seen, {"drag", "teleop"})

    def test_teleop_only_with_synthetic_manifest(self):
        ds = DragDemoDataset(SplitConfig.from_json(self.split_path),
                             "teleop_only", samples_per_episode=2,
                             image_size=(64, 48))
        self.assertGreater(len(ds), 0)
        self.assertTrue(all(m.source == "teleop" for m in ds.episodes))


class TestBadInputs(unittest.TestCase):
    def test_unknown_mode_rejected(self):
        cfg = SplitConfig.from_json(SPLIT_CONFIG)
        with self.assertRaises(ValueError):
            DragDemoDataset(cfg, "bogus_mode")


if __name__ == "__main__":
    unittest.main(verbosity=2)
