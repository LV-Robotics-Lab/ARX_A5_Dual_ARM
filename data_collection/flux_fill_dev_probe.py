#!/usr/bin/env python3
"""Probe FLUX.1-Fill-dev readiness on drag-demo frames.

Creates a concrete input gallery and dependency report even when the FLUX model
cannot be run on this host. The mask heuristic targets green glove / hand-like
regions and falls back to a conservative lower-frame ROI so the future inpaint
run has deterministic inputs.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone
from typing import Any

import cv2
import numpy as np
from PIL import Image

TASK_ID = "task_drag_demo_flux_fill_dev_inpaint_probe"
MODEL_ID = "black-forest-labs/FLUX.1-Fill-dev"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def module_status(name: str) -> dict[str, Any]:
    spec = importlib.util.find_spec(name)
    if spec is None:
        return {"installed": False, "version": None}
    try:
        module = __import__(name)
        return {"installed": True, "version": getattr(module, "__version__", "unknown")}
    except Exception as exc:
        return {"installed": True, "version": None, "import_error": repr(exc)}


def run_cmd(args: list[str]) -> dict[str, Any]:
    try:
        proc = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
    except Exception as exc:
        return {"ok": False, "stdout": "", "stderr": repr(exc), "returncode": None, "args": args}
    return {"ok": proc.returncode == 0, "stdout": proc.stdout.strip(), "stderr": proc.stderr.strip(), "returncode": proc.returncode, "args": args}


def env_probe() -> dict[str, Any]:
    token_names = ["HF_TOKEN", "HUGGINGFACE_TOKEN", "HUGGING_FACE_HUB_TOKEN"]
    token_files = [Path.home() / ".cache/huggingface/token", Path.home() / ".cache/huggingface/stored_tokens"]
    cache_hits = []
    for root in [Path.home() / ".cache/huggingface/hub", Path.home() / ".cache/huggingface", Path.home() / "models"]:
        if root.exists():
            for path in root.glob("**/*FLUX*Fill*dev*"):
                cache_hits.append(str(path))
                if len(cache_hits) >= 20:
                    break
    return {
        "python": sys.version,
        "modules": {name: module_status(name) for name in ["torch", "diffusers", "transformers", "accelerate", "safetensors", "PIL", "cv2", "numpy"]},
        "cuda": run_cmd([sys.executable, "-c", "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'no_cuda')"]),
        "hf_token_env_present": [name for name in token_names if os.environ.get(name)],
        "hf_token_files_present": [str(path) for path in token_files if path.exists()],
        "model_cache_hits": cache_hits,
    }


def choose_episode(manifest_path: Path) -> dict[str, Any]:
    manifest = json.loads(manifest_path.read_text())
    episodes = manifest.get("counted_episodes") or manifest.get("eval_tier_strict_episodes") or []
    if not episodes:
        raise RuntimeError(f"no counted/eval episodes in {manifest_path}")
    return episodes[0]


def extract_frame(video_path: Path, frame_frac: float) -> tuple[np.ndarray, dict[str, Any]]:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {video_path}")
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
    idx = max(0, min(n - 1, int(n * frame_frac))) if n else 0
    cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
    ok, frame_bgr = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError(f"cannot read frame {idx} from {video_path}")
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    return frame_rgb, {"video": str(video_path), "frame_count": n, "fps": fps, "frame_index": idx}


def make_hand_mask(frame_rgb: np.ndarray) -> tuple[np.ndarray, dict[str, Any]]:
    h, w = frame_rgb.shape[:2]
    r = frame_rgb[..., 0].astype(np.int16)
    g = frame_rgb[..., 1].astype(np.int16)
    b = frame_rgb[..., 2].astype(np.int16)
    green = (g > 75) & (g > r + 18) & (g > b + 8)
    lower_bias = np.zeros((h, w), dtype=bool)
    lower_bias[int(h * 0.45):, :] = True
    side_bias = np.zeros((h, w), dtype=bool)
    side_bias[:, : int(w * 0.30)] = True
    side_bias[:, int(w * 0.70):] = True
    mask = (green & (lower_bias | side_bias)).astype(np.uint8) * 255
    source = "green_glove_color_heuristic"
    if int(mask.sum() // 255) < max(500, int(h * w * 0.002)):
        mask[:] = 0
        mask[int(h * 0.62): h, 0: int(w * 0.28)] = 255
        mask[int(h * 0.62): h, int(w * 0.72): w] = 255
        source = "fallback_lower_side_roi"
    kernel = np.ones((17, 17), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)
    return mask, {"mask_source": source, "mask_pixels": int((mask > 0).sum()), "mask_fraction": float((mask > 0).mean())}


def overlay_mask(frame_rgb: np.ndarray, mask: np.ndarray) -> np.ndarray:
    out = frame_rgb.copy().astype(np.float32)
    red = np.zeros_like(out)
    red[..., 0] = 255
    a = (mask > 0)[..., None].astype(np.float32) * 0.45
    out = out * (1 - a) + red * a
    return np.clip(out, 0, 255).astype(np.uint8)


def write_runner(out_dir: Path) -> dict[str, str]:
    runner = out_dir / "run_flux_fill_dev.py"
    runner.write_text(f'''#!/usr/bin/env python3
"""Run FLUX.1-Fill-dev on the prepared probe frame.
Requires a machine with diffusers/transformers/accelerate/safetensors, torch CUDA,
and HF access to {MODEL_ID}.
"""
import argparse
import torch
from PIL import Image
from diffusers import FluxFillPipeline

parser = argparse.ArgumentParser()
parser.add_argument("--image", required=True)
parser.add_argument("--mask", required=True)
parser.add_argument("--out", required=True)
parser.add_argument("--prompt", default="Remove the human hands and green gloves only. Preserve robot links, gripper, spatula, egg mold, plate, table, and contact geometry.")
args = parser.parse_args()
pipe = FluxFillPipeline.from_pretrained("{MODEL_ID}", torch_dtype=torch.bfloat16).to("cuda")
image = Image.open(args.image).convert("RGB")
mask = Image.open(args.mask).convert("L")
result = pipe(prompt=args.prompt, image=image, mask_image=mask, height=image.height, width=image.width, guidance_scale=30, num_inference_steps=28).images[0]
result.save(args.out)
''', encoding="utf-8")
    runner.chmod(0o755)
    return {"path": str(runner), "sha256": sha256_file(runner)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=Path.home() / "workspace/.verifier_reports/initial_batch_manifest.json")
    parser.add_argument("--camera", default="cam_right_wrist_rgb.mp4")
    parser.add_argument("--frame-frac", type=float, default=0.55)
    parser.add_argument("--out", type=Path, default=Path.home() / "workspace/.verifier_reports/flux_fill_dev_probe")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    env = env_probe()
    episode = choose_episode(args.manifest)
    video_path = Path(episode["path"]) / args.camera
    frame, frame_meta = extract_frame(video_path, args.frame_frac)
    mask, mask_meta = make_hand_mask(frame)
    overlay = overlay_mask(frame, mask)

    before_path = args.out / "before.png"
    mask_path = args.out / "mask_hand_candidate.png"
    overlay_path = args.out / "mask_overlay.png"
    Image.fromarray(frame, mode="RGB").save(before_path)
    Image.fromarray(mask, mode="L").save(mask_path)
    Image.fromarray(overlay, mode="RGB").save(overlay_path)
    runner = write_runner(args.out)

    blockers = []
    for name in ["diffusers", "transformers", "accelerate", "safetensors"]:
        if not env["modules"].get(name, {}).get("installed"):
            blockers.append({"code": f"missing_{name}", "action": f"Install `{name}` in the runtime env before FLUX.1-Fill-dev inference."})
    if not env["modules"].get("torch", {}).get("installed"):
        blockers.append({"code": "missing_torch", "action": "Install CUDA-enabled torch in the runtime env."})
    if "True" not in env.get("cuda", {}).get("stdout", ""):
        blockers.append({"code": "cuda_unavailable", "action": "Run on a CUDA GPU with enough VRAM for FLUX.1-Fill-dev."})
    if not env["hf_token_env_present"] and not env["hf_token_files_present"]:
        blockers.append({"code": "hf_token_missing", "action": f"Authenticate Hugging Face access for `{MODEL_ID}` before downloading gated weights."})
    if not env["model_cache_hits"]:
        blockers.append({"code": "flux_weights_not_cached", "action": f"Download/cache `{MODEL_ID}` weights or point diffusers to a local snapshot."})

    status = "blocked_dependency" if blockers else "ready_to_run_flux"
    artifacts = {
        "before_png": {"path": str(before_path), "sha256": sha256_file(before_path)},
        "mask_png": {"path": str(mask_path), "sha256": sha256_file(mask_path)},
        "overlay_png": {"path": str(overlay_path), "sha256": sha256_file(overlay_path)},
        "runner": runner,
    }
    report = {
        "task_id": TASK_ID,
        "status": status,
        "generated_at": utc_now(),
        "model_id": MODEL_ID,
        "episode": {"dataset": episode.get("dataset"), "name": episode.get("name"), "path": episode.get("path")},
        "frame": frame_meta,
        "mask": mask_meta,
        "environment": env,
        "artifacts": artifacts,
        "blockers": blockers,
        "qa_policy": "After real FLUX output exists, compute diff only inside mask and verify robot links/spatula/egg mold/plate/contact geometry are unchanged outside mask.",
    }
    report_path = args.out / "report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    md_path = args.out / "report.md"
    md_path.write_text("\n".join([
        "# FLUX.1-Fill-dev Drag-Demo Probe",
        "",
        f"- status: `{status}`",
        f"- model: `{MODEL_ID}`",
        f"- episode: `{episode.get('dataset')}/{episode.get('name')}`",
        f"- frame: `{frame_meta['video']}` index `{frame_meta['frame_index']}`",
        f"- before: `{before_path}` sha256 `{artifacts['before_png']['sha256']}`",
        f"- mask: `{mask_path}` sha256 `{artifacts['mask_png']['sha256']}` ({mask_meta['mask_source']}, {mask_meta['mask_fraction']:.4f} of pixels)",
        f"- overlay: `{overlay_path}` sha256 `{artifacts['overlay_png']['sha256']}`",
        f"- runner: `{runner['path']}` sha256 `{runner['sha256']}`",
        "",
        "## Blockers",
        *(f"- `{b['code']}`: {b['action']}" for b in blockers),
        *( ["- none"] if not blockers else [] ),
        "",
    ]) + "\n", encoding="utf-8")
    print(json.dumps({"status": status, "report": str(report_path), "report_sha256": sha256_file(report_path), "markdown": str(md_path), "markdown_sha256": sha256_file(md_path)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
