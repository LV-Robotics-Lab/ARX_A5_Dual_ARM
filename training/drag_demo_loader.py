#!/usr/bin/env python3
"""Baseline fine-tuning loader for drag-demo / teleop / mixed splits.

The loader serves one *timestep* per sample. Each sample bundles:

  * `rgb`     -- dict {cam_top, cam_left_wrist, cam_right_wrist}, each a
                 uint8 tensor of shape (H, W, 3) BGR (cv2 default).
  * `state`   -- float32 tensor of shape (14,) — left[7] || right[7] joint
                 positions at the sampled timestep.
  * `action`  -- float32 tensor of shape (14,) — same joints `state_dt_ms`
                 milliseconds later (next-step joint command target). Last
                 timestep of an episode falls back to the final sample.
  * `meta`    -- dict {source, dataset, episode, episode_idx, t_sample_ms}.

Split config schema (see `split_config.json`):

  {
    "schema_version": "drag_demo_split.v1",
    "sources": {
      "drag":   {"manifest": "/abs/path/to/initial_batch_manifest.json",
                 "episode_filter": "counted_episodes"},
      "teleop": {"manifest": "/abs/path/or/null",
                 "episode_filter": "counted_episodes"}
    },
    "modes": {
      "drag_only":   {"drag": 1.0, "teleop": 0.0},
      "teleop_only": {"drag": 0.0, "teleop": 1.0},
      "mixed_70_30": {"drag": 0.7, "teleop": 0.3}
    }
  }

For a missing source (e.g. teleop not yet collected) the mixture proportion
that depends on it becomes 0 and the loader records it in `missing_streams`
so the caller can surface a clear error or warning. The dry-run script
(`dry_run_loader.py`) prints this report explicitly.
"""

from __future__ import annotations

import bisect
import json
import pickle
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import h5py
import numpy as np
import torch
from torch.utils.data import Dataset


CAM_KEYS = ("cam_top", "cam_left_wrist", "cam_right_wrist")
ALLOWED_MODES = ("drag_only", "teleop_only", "mixed_70_30")


# ---------------------------------------------------------------------------
# Split configuration
# ---------------------------------------------------------------------------


@dataclass
class SourceSpec:
    name: str
    manifest_path: Path | None
    episode_filter: str

    @property
    def available(self) -> bool:
        return self.manifest_path is not None and self.manifest_path.is_file()


@dataclass
class SplitConfig:
    sources: dict[str, SourceSpec]
    modes: dict[str, dict[str, float]]

    @classmethod
    def from_json(cls, path: Path) -> "SplitConfig":
        raw = json.loads(Path(path).read_text())
        sources: dict[str, SourceSpec] = {}
        for name, spec in raw["sources"].items():
            mp = spec.get("manifest")
            sources[name] = SourceSpec(
                name=name,
                manifest_path=Path(mp) if mp else None,
                episode_filter=spec.get("episode_filter", "counted_episodes"),
            )
        modes = {k: dict(v) for k, v in raw["modes"].items()}
        return cls(sources=sources, modes=modes)

    def resolve_mode(self, mode: str) -> dict[str, float]:
        if mode not in self.modes:
            raise ValueError(f"unknown mode {mode!r}; allowed: "
                             f"{sorted(self.modes)}")
        return dict(self.modes[mode])


# ---------------------------------------------------------------------------
# Per-episode IO
# ---------------------------------------------------------------------------


def _load_state(state_pkl: Path) -> dict[str, np.ndarray]:
    """Load state.pkl. Both arms are truncated to the shared sample length so
    indexing left/right in lockstep is always safe — the two arms run on
    independent ROS callbacks during recording, so their array lengths can
    differ by a few samples."""
    with open(state_pkl, "rb") as f:
        d = pickle.load(f)
    arms_joints = {arm: np.asarray(d[arm]["joints"], dtype=np.float32)
                   for arm in ("left_arm", "right_arm")}
    arms_ts = {arm: np.asarray(d[arm]["timestamps"], dtype=np.int64)
               for arm in ("left_arm", "right_arm")}
    n = min(arms_joints[arm].shape[0] for arm in ("left_arm", "right_arm"))
    out: dict[str, np.ndarray] = {}
    for arm in ("left_arm", "right_arm"):
        out[f"{arm}_joints"] = arms_joints[arm][:n]
        out[f"{arm}_ts"] = arms_ts[arm][:n]
    return out


def _load_image_timestamps(its_pkl: Path) -> dict[str, np.ndarray]:
    with open(its_pkl, "rb") as f:
        d = pickle.load(f)
    return {cam: np.asarray(d[cam], dtype=np.int64) for cam in CAM_KEYS
            if cam in d}


def _nearest_index(sorted_ts: np.ndarray, target_ms: int) -> int:
    if sorted_ts.size == 0:
        raise IndexError("empty timestamps array")
    j = bisect.bisect_left(sorted_ts, target_ms)
    if j == 0:
        return 0
    if j == len(sorted_ts):
        return len(sorted_ts) - 1
    before = sorted_ts[j - 1]
    after = sorted_ts[j]
    return j - 1 if (target_ms - before) <= (after - target_ms) else j


def _read_frame(cap: cv2.VideoCapture, frame_idx: int) -> np.ndarray:
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(frame_idx))
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError(f"cv2 failed to read frame {frame_idx}")
    return frame


# ---------------------------------------------------------------------------
# Episode descriptor + sampling
# ---------------------------------------------------------------------------


@dataclass
class EpisodeIndex:
    """Per-episode metadata + sampleable timestep indices.

    Sampling happens in the state.pkl time grid (60 Hz). For each sampled
    state index we look up the nearest RGB frame on every camera via the
    image_timestamps.pkl arrays — handles cameras running at independent
    rates without forcing a brittle global sync.
    """
    source: str
    dataset: str
    name: str
    path: Path
    files: dict[str, Path]
    n_state: int
    sample_indices: np.ndarray            # indices into state arrays
    cam_n_frames: dict[str, int]
    missing_streams: list[str]


def _build_episode_index(source: str, ep: dict[str, Any], *,
                         samples_per_episode: int) -> EpisodeIndex | None:
    files = {k: Path(v) for k, v in ep["files"].items()}
    missing = [name for name, p in files.items() if not p.is_file()]
    if missing:
        return EpisodeIndex(source=source, dataset=ep["dataset"],
                            name=ep["name"], path=Path(ep["path"]),
                            files=files, n_state=0,
                            sample_indices=np.zeros(0, dtype=np.int64),
                            cam_n_frames={}, missing_streams=missing)

    try:
        state = _load_state(files["state.pkl"])
    except (OSError, pickle.UnpicklingError, EOFError, KeyError) as e:
        return EpisodeIndex(source=source, dataset=ep["dataset"],
                            name=ep["name"], path=Path(ep["path"]),
                            files=files, n_state=0,
                            sample_indices=np.zeros(0, dtype=np.int64),
                            cam_n_frames={},
                            missing_streams=[f"state_load_failed: {e}"])
    n_state = min(state["left_arm_joints"].shape[0],
                  state["right_arm_joints"].shape[0])
    if n_state < 2:
        return EpisodeIndex(source=source, dataset=ep["dataset"],
                            name=ep["name"], path=Path(ep["path"]),
                            files=files, n_state=n_state,
                            sample_indices=np.zeros(0, dtype=np.int64),
                            cam_n_frames={},
                            missing_streams=["state_too_short"])
    # Reserve the last sample for action-delta target.
    last_idx = n_state - 2
    n_samples = min(samples_per_episode, last_idx + 1)
    if n_samples <= 0:
        return EpisodeIndex(source=source, dataset=ep["dataset"],
                            name=ep["name"], path=Path(ep["path"]),
                            files=files, n_state=n_state,
                            sample_indices=np.zeros(0, dtype=np.int64),
                            cam_n_frames={}, missing_streams=["no_samples"])
    sample_indices = np.linspace(0, last_idx, n_samples).round().astype(np.int64)

    # Probe video frame counts without holding caps open.
    cam_n_frames: dict[str, int] = {}
    missing_streams: list[str] = []
    for cam in CAM_KEYS:
        rgb_path = files.get(f"{cam}_rgb.mp4")
        if rgb_path is None or not rgb_path.is_file():
            missing_streams.append(f"{cam}_rgb.mp4")
            continue
        cap = cv2.VideoCapture(str(rgb_path))
        try:
            if not cap.isOpened():
                missing_streams.append(f"{cam}_rgb.mp4:open_failed")
                continue
            n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            cam_n_frames[cam] = n
            if n <= 0:
                missing_streams.append(f"{cam}_rgb.mp4:no_frames")
        finally:
            cap.release()
    return EpisodeIndex(source=source, dataset=ep["dataset"],
                        name=ep["name"], path=Path(ep["path"]),
                        files=files, n_state=n_state,
                        sample_indices=sample_indices,
                        cam_n_frames=cam_n_frames,
                        missing_streams=missing_streams)


# ---------------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------------


@dataclass
class DatasetReport:
    """Diagnostic info returned by `DragDemoDataset.report()`."""
    mode: str
    mixture: dict[str, float]
    sources_summary: dict[str, dict[str, Any]]
    n_total_episodes: int
    n_total_samples: int
    missing_streams: list[dict[str, Any]]


class DragDemoDataset(Dataset):
    """Mixture-aware Dataset over drag/teleop/mixed manifests.

    Sampling rule: the dataset enumerates `samples_per_episode` timesteps per
    episode (uniformly spaced over state.pkl), per source. Mixture weighting
    is realised at index-construction time: each source's sample pool is
    truncated to floor(target_share * universe_size) so the resulting
    `len(dataset)` matches the requested mixture as closely as the data
    allows. Sources with `target_share == 0` are dropped entirely.
    """

    def __init__(self, split_config: SplitConfig | str | Path, mode: str,
                 *, samples_per_episode: int = 16,
                 state_dt_ms: int = 100,
                 image_size: tuple[int, int] | None = None,
                 seed: int = 0):
        super().__init__()
        if isinstance(split_config, (str, Path)):
            split_config = SplitConfig.from_json(Path(split_config))
        if mode not in ALLOWED_MODES:
            raise ValueError(f"unknown mode {mode!r}; allowed: {ALLOWED_MODES}")
        self.mode = mode
        self.mixture = split_config.resolve_mode(mode)
        self.samples_per_episode = samples_per_episode
        self.state_dt_ms = state_dt_ms
        self.image_size = image_size
        self.rng = np.random.RandomState(seed)

        per_source_episodes: dict[str, list[EpisodeIndex]] = {}
        sources_summary: dict[str, dict[str, Any]] = {}
        missing_streams_report: list[dict[str, Any]] = []
        for src_name, spec in split_config.sources.items():
            target = self.mixture.get(src_name, 0.0)
            if target <= 0.0:
                sources_summary[src_name] = {
                    "manifest": str(spec.manifest_path) if spec.manifest_path
                                else None,
                    "available": spec.available,
                    "target_share": target,
                    "n_episodes_loaded": 0,
                    "reason": "share=0 in current mode",
                }
                continue
            if not spec.available:
                sources_summary[src_name] = {
                    "manifest": str(spec.manifest_path) if spec.manifest_path
                                else None,
                    "available": False,
                    "target_share": target,
                    "n_episodes_loaded": 0,
                    "reason": "manifest_missing",
                }
                missing_streams_report.append({
                    "source": src_name,
                    "missing": "manifest",
                    "manifest_path": str(spec.manifest_path)
                                     if spec.manifest_path else None,
                })
                continue
            manifest = json.loads(Path(spec.manifest_path).read_text())
            episode_list = manifest.get(spec.episode_filter, [])
            built: list[EpisodeIndex] = []
            for ep in episode_list:
                idx = _build_episode_index(src_name, ep,
                                           samples_per_episode=samples_per_episode)
                if idx is None:
                    continue
                if idx.missing_streams:
                    missing_streams_report.append({
                        "source": src_name,
                        "episode": f"{idx.dataset}/{idx.name}",
                        "missing": idx.missing_streams,
                    })
                if idx.sample_indices.size > 0 and not idx.missing_streams:
                    built.append(idx)
            per_source_episodes[src_name] = built
            sources_summary[src_name] = {
                "manifest": str(spec.manifest_path),
                "available": True,
                "target_share": target,
                "episode_filter": spec.episode_filter,
                "n_episodes_in_manifest": len(episode_list),
                "n_episodes_loaded": len(built),
            }

        # Mixture truncation. Anchor on the binding source: universe is the
        # max size we can support without exceeding any required source's
        # availability. If ANY source with share>0 has 0 episodes, the
        # universe is 0 — we refuse to silently degrade a 70/30 mix into a
        # 100/0 one when teleop is missing.
        chosen: dict[str, list[EpisodeIndex]] = {}
        required_missing: list[str] = []
        ratios: list[float] = []
        for src_name, share in self.mixture.items():
            if share <= 0:
                continue
            eps = per_source_episodes.get(src_name, [])
            if not eps:
                required_missing.append(src_name)
                continue
            ratios.append(len(eps) / share)
        if required_missing:
            universe = 0.0
            for src_name in required_missing:
                sources_summary.setdefault(src_name, {})
                sources_summary[src_name]["mixture_degraded"] = (
                    "required by mode but 0 episodes loaded; refusing to "
                    "silently fall back to a non-mixture distribution")
        else:
            universe = min(ratios) if ratios else 0.0
        for src_name, eps in per_source_episodes.items():
            share = self.mixture.get(src_name, 0.0)
            if share <= 0 or not eps or universe <= 0:
                chosen[src_name] = []
                if share > 0 and src_name in sources_summary:
                    sources_summary[src_name]["n_episodes_after_mixture"] = 0
                continue
            take = min(len(eps), int(round(share * universe)))
            idxs = self.rng.permutation(len(eps))[:take]
            chosen[src_name] = [eps[i] for i in idxs]
            sources_summary[src_name]["n_episodes_after_mixture"] = take

        # Flatten into a list of (episode_idx, sample_offset) pairs.
        self.episodes: list[EpisodeIndex] = []
        self.index: list[tuple[int, int]] = []
        for src_name in chosen:
            for ep in chosen[src_name]:
                ep_id = len(self.episodes)
                self.episodes.append(ep)
                for off in range(ep.sample_indices.size):
                    self.index.append((ep_id, off))

        self._report = DatasetReport(
            mode=mode,
            mixture=self.mixture,
            sources_summary=sources_summary,
            n_total_episodes=len(self.episodes),
            n_total_samples=len(self.index),
            missing_streams=missing_streams_report,
        )

        # Per-episode caches (lazy). VideoCapture instances are not pickleable
        # and not multi-thread safe, so we open per-worker on demand.
        self._video_caps: dict[int, dict[str, cv2.VideoCapture]] = {}
        self._image_ts: dict[int, dict[str, np.ndarray]] = {}
        self._state: dict[int, dict[str, np.ndarray]] = {}

    # -- introspection ------------------------------------------------------

    def report(self) -> DatasetReport:
        return self._report

    def __len__(self) -> int:
        return len(self.index)

    # -- item loading -------------------------------------------------------

    def _cap_for(self, ep_id: int, cam: str) -> cv2.VideoCapture:
        bucket = self._video_caps.setdefault(ep_id, {})
        cap = bucket.get(cam)
        if cap is None:
            cap = cv2.VideoCapture(
                str(self.episodes[ep_id].files[f"{cam}_rgb.mp4"]))
            bucket[cam] = cap
        return cap

    def _img_ts_for(self, ep_id: int) -> dict[str, np.ndarray]:
        cached = self._image_ts.get(ep_id)
        if cached is None:
            cached = _load_image_timestamps(
                self.episodes[ep_id].files["image_timestamps.pkl"])
            self._image_ts[ep_id] = cached
        return cached

    def _state_for(self, ep_id: int) -> dict[str, np.ndarray]:
        cached = self._state.get(ep_id)
        if cached is None:
            cached = _load_state(self.episodes[ep_id].files["state.pkl"])
            self._state[ep_id] = cached
        return cached

    def __getitem__(self, idx: int) -> dict[str, Any]:
        ep_id, off = self.index[idx]
        ep = self.episodes[ep_id]
        state = self._state_for(ep_id)
        state_idx = int(ep.sample_indices[off])
        ts_ms = int(state["left_arm_ts"][state_idx])

        # action target: state at +state_dt_ms (or the last sample if at end).
        target_ms = ts_ms + self.state_dt_ms
        action_idx = _nearest_index(state["left_arm_ts"], target_ms)
        action_idx = min(action_idx, state["left_arm_joints"].shape[0] - 1)

        state_vec = np.concatenate([state["left_arm_joints"][state_idx],
                                    state["right_arm_joints"][state_idx]])
        action_vec = np.concatenate([state["left_arm_joints"][action_idx],
                                     state["right_arm_joints"][action_idx]])

        # RGB lookup per camera at the sampled state timestamp.
        img_ts = self._img_ts_for(ep_id)
        rgb: dict[str, torch.Tensor] = {}
        for cam in CAM_KEYS:
            cam_ts = img_ts.get(cam)
            if cam_ts is None or cam_ts.size == 0:
                raise RuntimeError(f"{ep.name}: missing image timestamps "
                                   f"for {cam}")
            frame_idx = _nearest_index(cam_ts, ts_ms)
            frame_idx = min(frame_idx, ep.cam_n_frames[cam] - 1)
            frame = _read_frame(self._cap_for(ep_id, cam), frame_idx)
            if self.image_size is not None:
                frame = cv2.resize(frame, self.image_size,
                                   interpolation=cv2.INTER_AREA)
            rgb[cam] = torch.from_numpy(frame)

        return {
            "rgb": rgb,
            "state": torch.from_numpy(state_vec.astype(np.float32)),
            "action": torch.from_numpy(action_vec.astype(np.float32)),
            "meta": {
                "source": ep.source,
                "dataset": ep.dataset,
                "episode": ep.name,
                "episode_idx": ep_id,
                "t_sample_ms": ts_ms,
            },
        }

    # Avoid leaking VideoCapture file handles when workers shut down.
    def __del__(self):
        try:
            for caps in self._video_caps.values():
                for cap in caps.values():
                    cap.release()
        except Exception:
            pass


def collate(batch: list[dict[str, Any]]) -> dict[str, Any]:
    """Default collate: stack tensors per camera, stack state/action, keep meta as list."""
    rgb_out: dict[str, torch.Tensor] = {
        cam: torch.stack([b["rgb"][cam] for b in batch], dim=0) for cam in CAM_KEYS
    }
    state = torch.stack([b["state"] for b in batch], dim=0)
    action = torch.stack([b["action"] for b in batch], dim=0)
    meta = [b["meta"] for b in batch]
    return {"rgb": rgb_out, "state": state, "action": action, "meta": meta}
