#!/usr/bin/env python3
"""Smoke training loop for the fine-tuning comparison spec.

Trains a tiny image+state -> action regressor on whichever arm has data
today. With the current data state only arm C (raw drag-only) is READY, so
that's the default. Produces a real loss curve to demonstrate the loader ->
model -> backprop pipeline works end-to-end. Logs go through
experiment_tracker.

Usage (default arm=C_raw_drag_only):
    cd ~/workspace/ARX_A5_Dual_ARM/training/experiments
    ~/miniconda3/envs/robo_ctrl/bin/python train_smoke.py \\
        --steps 100 --batch-size 4 --image-size 64 48
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import sys
import time
import traceback
from contextlib import redirect_stdout
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader

THIS = Path(__file__).resolve().parent
PARENT = THIS.parent
for p in (THIS, PARENT):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from drag_demo_loader import (DragDemoDataset, SplitConfig, collate,  # noqa: E402
                              CAM_KEYS)
from experiment_tracker import ExperimentRun  # noqa: E402

ARM_TO_MODE = {
    "A_teleop_only":     "teleop_only",
    "B_drag_plus_editing": "drag_only",  # uses an alternative manifest
    "C_raw_drag_only":   "drag_only",
    "D_mixed_70_30":     "mixed_70_30",
}


class TinyCamCNN(nn.Module):
    def __init__(self, out_dim: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(3, 32, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(64, 128, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d(1), nn.Flatten(), nn.Linear(128, out_dim),
        )

    def forward(self, x_uint8_bhwc: torch.Tensor) -> torch.Tensor:
        x = x_uint8_bhwc.permute(0, 3, 1, 2).contiguous().float() / 255.0
        return self.net(x)


class ImageStateActionRegressor(nn.Module):
    """3-cam tiny CNN + state MLP -> action MLP."""
    def __init__(self, state_dim: int = 14, action_dim: int = 14,
                 cam_feat_dim: int = 128):
        super().__init__()
        self.cams = nn.ModuleDict({cam: TinyCamCNN(cam_feat_dim)
                                   for cam in CAM_KEYS})
        self.state_enc = nn.Sequential(
            nn.Linear(state_dim, 64), nn.ReLU(inplace=True))
        in_dim = cam_feat_dim * len(CAM_KEYS) + 64
        self.head = nn.Sequential(
            nn.Linear(in_dim, 256), nn.ReLU(inplace=True),
            nn.Linear(256, action_dim))

    def forward(self, batch):
        feats = [self.cams[cam](batch["rgb"][cam]) for cam in CAM_KEYS]
        s = self.state_enc(batch["state"])
        return self.head(torch.cat(feats + [s], dim=-1))


def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _move_to(batch, device):
    return {
        "rgb": {cam: batch["rgb"][cam].to(device, non_blocking=True)
                for cam in CAM_KEYS},
        "state": batch["state"].to(device, non_blocking=True),
        "action": batch["action"].to(device, non_blocking=True),
        "meta": batch["meta"],
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", default="C_raw_drag_only",
                    choices=list(ARM_TO_MODE))
    ap.add_argument("--split-config", type=Path,
                    default=PARENT / "split_config.json")
    ap.add_argument("--steps", type=int, default=100)
    ap.add_argument("--batch-size", type=int, default=4)
    ap.add_argument("--samples-per-episode", type=int, default=8)
    ap.add_argument("--image-size", type=int, nargs=2, default=[64, 48],
                    metavar=("W", "H"))
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--runs-dir", type=Path,
                    default=Path.home() / "workspace" / ".experiment_runs")
    ap.add_argument("--device", default="auto")
    args = ap.parse_args(argv)

    torch.manual_seed(args.seed)
    device = (torch.device(args.device) if args.device != "auto"
              else torch.device("cuda" if torch.cuda.is_available() else "cpu"))

    mode = ARM_TO_MODE[args.arm]
    cfg = SplitConfig.from_json(args.split_config)
    image_size = (args.image_size[0], args.image_size[1])
    ds = DragDemoDataset(cfg, mode,
                         samples_per_episode=args.samples_per_episode,
                         image_size=image_size, seed=args.seed)
    report = ds.report()
    print(f"arm={args.arm}  mode={mode}  episodes={report.n_total_episodes}  "
          f"samples={report.n_total_samples}")
    if len(ds) == 0:
        print("Dataset is empty; refusing to train.")
        print("missing_streams:", report.missing_streams)
        print("sources_summary:", report.sources_summary)
        return 2

    loader = DataLoader(ds, batch_size=args.batch_size, shuffle=True,
                        collate_fn=collate, num_workers=0, drop_last=True)
    model = ImageStateActionRegressor().to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr,
                            betas=(0.9, 0.999), weight_decay=1e-2)
    n_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"model params: {n_params:,}  device: {device}")

    data_provenance: dict[str, str] = {}
    for src_name, spec in cfg.sources.items():
        if spec.available:
            data_provenance[f"{src_name}_manifest"] = str(spec.manifest_path)
            data_provenance[f"{src_name}_manifest_sha256"] = _sha256(
                spec.manifest_path)

    run = ExperimentRun.open(
        arm_id=args.arm,
        config={
            "mode": mode,
            "split_config": str(args.split_config),
            "steps": args.steps,
            "batch_size": args.batch_size,
            "samples_per_episode": args.samples_per_episode,
            "image_size": list(image_size),
            "lr": args.lr,
            "seed": args.seed,
            "model_param_count": n_params,
        },
        runs_dir=args.runs_dir,
        data_provenance=data_provenance,
        loader_path=PARENT / "drag_demo_loader.py",
    )

    buf = io.StringIO()
    step = 0
    losses: list[float] = []
    t0 = time.perf_counter()
    try:
        with redirect_stdout(buf):
            it = iter(loader)
            while step < args.steps:
                try:
                    batch_cpu = next(it)
                except StopIteration:
                    it = iter(loader)
                    batch_cpu = next(it)
                t_step = time.perf_counter()
                batch = _move_to(batch_cpu, device)
                pred = model(batch)
                loss = F.mse_loss(pred, batch["action"])
                opt.zero_grad(set_to_none=True)
                loss.backward()
                opt.step()
                dt = time.perf_counter() - t_step
                losses.append(float(loss.item()))
                run.log_step(step=step, loss=float(loss.item()),
                             lr=args.lr, time_per_step_s=dt)
                if step % 10 == 0:
                    print(f"step {step:4d}  loss {float(loss.item()):.5f}  "
                          f"dt {dt*1000:.1f} ms", flush=True)
                step += 1
        total = time.perf_counter() - t0
        last_n = losses[-10:] if len(losses) >= 10 else losses
        final = {
            "final_loss": float(losses[-1]),
            "mean_loss_last_10": float(sum(last_n) / max(1, len(last_n))),
            "first_loss": float(losses[0]),
            "rel_improvement": float(
                (losses[0] - sum(last_n) / max(1, len(last_n))) / max(1e-9, losses[0])),
            "total_time_s": float(total),
            "steps_completed": step,
        }
        run.finalize(final_metrics=final, exit_status="smoke",
                     stdout_tail=buf.getvalue()[-4096:])
    except Exception as exc:  # pragma: no cover
        run.finalize(final_metrics={"error": str(exc),
                                    "traceback": traceback.format_exc()},
                     exit_status="failed", stdout_tail=buf.getvalue()[-4096:])
        raise

    print(f"\nWrote run JSON: {run.run_json_path}")
    print(f"Wrote metrics  : {run._csv_path}")
    print(json.dumps(final, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
