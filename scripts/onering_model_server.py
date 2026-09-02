#!/usr/bin/env python
"""JSON HTTP service for the local OneRING checkpoint.

The RING repository has a heavyweight AllenAct/open_clip dependency stack and
keeps recurrent Llama state in the policy object.  This service isolates that
stack in the ``ring`` environment and exposes one task-scoped stateful policy
to Vector OS Nano.
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import os
import sys
import threading
import time
import types
from pathlib import Path
from typing import Any

import numpy as np
from flask import Flask, jsonify, request, send_from_directory
from PIL import Image

DEFAULT_REPO = Path("/media/fishyu/fish-14tb-12/YiFei/OneRING")
DEFAULT_CHECKPOINT = DEFAULT_REPO / "checkpoints/OneRING/ring_model_step_40356421.ckpt"
DEFAULT_SIGLIP = DEFAULT_REPO / "checkpoints/SigLIP"

app = Flask(__name__)
agent: Any = None
agent_lock = threading.RLock()
active_instruction = ""
step_count = 0
args: argparse.Namespace

_DEBUG_HTML = """<!doctype html><meta charset="utf-8"><title>OneRING live input</title>
<style>body{background:#111;color:#eee;font:16px sans-serif;margin:20px}.views{display:flex;gap:16px;flex-wrap:wrap}figure{margin:0;width:min(46vw,720px)}img{width:100%;border:1px solid #555}pre{color:#9ee}</style>
<h2>OneRING live model input</h2><pre id="stats">Waiting for /step...</pre>
<div class="views"><figure><figcaption>Navigation RGB</figcaption><img id="nav"></figure><figure><figcaption>Manipulation RGB</figcaption><img id="manip"></figure></div>
<script>async function refresh(){const t=Date.now();nav.src='/debug/latest_navigation.png?t='+t;manip.src='/debug/latest_manipulation.png?t='+t;try{stats.textContent=JSON.stringify(await (await fetch('/debug/latest.json?t='+t)).json(),null,2)}catch(_){}}setInterval(refresh,250);refresh();</script>"""


def _decode_rgb(value: str) -> np.ndarray:
    raw = base64.b64decode(str(value), validate=True)
    return np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"), dtype=np.uint8)


def _png_bytes(array: np.ndarray) -> bytes:
    buffer = io.BytesIO()
    Image.fromarray(np.asarray(array, dtype=np.uint8), mode="RGB").save(
        buffer, format="PNG"
    )
    return buffer.getvalue()


def _atomic_bytes(value: bytes, path: Path) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_bytes(value)
    os.replace(temporary, path)


def _save_debug_input(navigation_rgb: np.ndarray, manipulation_rgb: np.ndarray, metadata: dict[str, Any]) -> None:
    target = Path(args.debug_dir).expanduser().resolve()
    target.mkdir(parents=True, exist_ok=True)
    index = int(metadata["step"])
    navigation_png = _png_bytes(navigation_rgb)
    manipulation_png = (
        navigation_png
        if manipulation_rgb is navigation_rgb
        else _png_bytes(manipulation_rgb)
    )
    _atomic_bytes(navigation_png, target / f"navigation_{index:06d}.png")
    _atomic_bytes(manipulation_png, target / f"manipulation_{index:06d}.png")
    _atomic_bytes(navigation_png, target / "latest_navigation.png")
    _atomic_bytes(manipulation_png, target / "latest_manipulation.png")


def _save_debug_metadata(metadata: dict[str, Any]) -> None:
    target = Path(args.debug_dir).expanduser().resolve()
    temporary = target / "latest.json.tmp"
    temporary.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temporary, target / "latest.json")


@app.get("/debug")
def debug_viewer():
    return _DEBUG_HTML


@app.get("/debug/<path:filename>")
def debug_file(filename: str):
    return send_from_directory(str(Path(args.debug_dir).expanduser().resolve()), filename, max_age=0)


def _patch_open_clip_for_local_siglip(siglip_dir: Path) -> None:
    """Redirect OneRING's hard-coded ``hf-hub:`` calls to local-dir loading."""
    import open_clip

    local_name = f"local-dir:{siglip_dir}"
    original_create = open_clip.create_model_from_pretrained
    original_tokenizer = open_clip.get_tokenizer

    def create_model(model_name: str, *call_args: Any, **kwargs: Any):
        if str(model_name).startswith("hf-hub:timm/ViT-B-16-SigLIP"):
            model_name = local_name
        return original_create(model_name, *call_args, **kwargs)

    def tokenizer(model_name: str, *call_args: Any, **kwargs: Any):
        if str(model_name).startswith("hf-hub:timm/ViT-B-16-SigLIP"):
            model_name = local_name
        return original_tokenizer(model_name, *call_args, **kwargs)

    open_clip.create_model_from_pretrained = create_model
    open_clip.get_tokenizer = tokenizer


def _load_agent() -> Any:
    repository = Path(args.repository).expanduser().resolve()
    checkpoint = Path(args.checkpoint).expanduser().resolve()
    siglip = Path(args.siglip_dir).expanduser().resolve()
    if not repository.is_dir():
        raise FileNotFoundError(repository)
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    if not (siglip / "open_clip_config.json").is_file():
        raise FileNotFoundError(f"SigLIP config missing under {siglip}")
    os.environ.setdefault("OBJAVERSE_DATA_DIR", str(repository))
    sys.path.insert(0, str(repository))
    _patch_open_clip_for_local_siglip(siglip)
    # Importing OneRING's generic string helpers otherwise eagerly loads the
    # optional Objaverse ``prior`` dataset.  The inference model only needs the
    # conversion helpers, so keep that benchmark-only import out of the server.
    task_instruction = types.ModuleType("utils.task_spec_to_instruction")
    task_instruction.REGISTERED_INSTRUCTION_TYPES = {}
    sys.modules.setdefault("utils.task_spec_to_instruction", task_instruction)
    # ``spoc_model`` imports two checkpoint helpers from the training module;
    # importing that whole module pulls in video datasets and AI2-THOR.  Supply
    # the two inference-safe helpers directly instead.
    import torch

    train_utils = types.ModuleType("training.offline.train_utils")

    def load_allenact_ckpt(model: Any, path: str) -> None:
        state = torch.load(path, map_location="cpu", weights_only=False)["model_state_dict"]
        model.load_state_dict(state)

    def load_pl_ckpt(model: Any, path: str, ckpt_prefix: str = "model.", verbose: bool = False) -> None:
        del verbose
        state = torch.load(path, map_location="cpu", weights_only=False)["state_dict"]
        current = model.state_dict()
        for key in current:
            if ckpt_prefix + key in state:
                current[key] = state[ckpt_prefix + key]
        model.load_state_dict(current)

    train_utils.load_allenact_ckpt = load_allenact_ckpt
    train_utils.load_pl_ckpt = load_pl_ckpt
    sys.modules.setdefault("training.offline.train_utils", train_utils)
    from architecture.models.spoc_models import REGISTERED_MODELS

    package = REGISTERED_MODELS["TuneSpocLlamaModelWTextGoal"]
    package.config.batch_size = 1
    package.config.max_seq_len = int(args.max_seq_len)
    model = package.model_cls.build_agent(
        cfg=package.config,
        ckpt_pth=str(checkpoint),
        device=args.device,
        load_from_pl=False,
    )
    model.eval()
    return model


@app.errorhandler(Exception)
def _error(exc: Exception):
    return jsonify({"error": f"{type(exc).__name__}: {exc}"}), 500


@app.post("/health")
def health():
    return jsonify({"ok": True, "loaded": agent is not None, "model": "onering"})


@app.post("/reset")
def reset():
    global active_instruction, step_count
    if agent is None:
        raise RuntimeError("OneRING model has not loaded")
    payload = request.get_json(force=True) or {}
    active_instruction = str(payload.get("instruction", "")).strip()
    with agent_lock:
        agent.reset()
        step_count = 0
    return jsonify({"ok": True, "instruction": active_instruction})


@app.post("/step")
def step():
    global step_count
    if agent is None:
        raise RuntimeError("OneRING model has not loaded")
    if step_count >= int(args.max_steps):
        raise RuntimeError(
            f"OneRING episode exceeded max_steps={int(args.max_steps)}; call /reset"
        )
    payload = request.get_json(force=True) or {}
    instruction = str(payload.get("instruction") or active_instruction).strip()
    if not instruction:
        raise ValueError("instruction is required")
    request_started = time.perf_counter()
    navigation_rgb = _decode_rgb(payload["navigation_rgb_png"])
    manipulation_value = payload.get("manipulation_rgb_png")
    manipulation_rgb = (
        navigation_rgb
        if manipulation_value is None
        else _decode_rgb(manipulation_value)
    )
    decode_finished = time.perf_counter()
    with agent_lock:
        import torch
        with torch.inference_mode():
            action, probabilities = agent.get_action(
                {
                    "raw_navigation_camera": navigation_rgb,
                    "raw_manipulation_camera": manipulation_rgb,
                    "traj_index": np.array([0], dtype=np.int64),
                },
                instruction,
            )
        inference_finished = time.perf_counter()
        del probabilities
        names = list(agent.get_action_list())
        step_count += 1
        metadata = {
            "step": step_count,
            "action": str(action),
            "action_index": names.index(action),
            "instruction": instruction,
            "navigation_shape": list(navigation_rgb.shape),
            "manipulation_shape": list(manipulation_rgb.shape),
            "timestamp": time.time(),
            "latency_ms": {
                "decode": round((decode_finished - request_started) * 1000.0, 3),
                "inference": round((inference_finished - decode_finished) * 1000.0, 3),
            },
        }
        _save_debug_input(navigation_rgb, manipulation_rgb, metadata)
        response_ready = time.perf_counter()
        metadata["latency_ms"]["debug_save"] = round(
            (response_ready - inference_finished) * 1000.0, 3
        )
        metadata["latency_ms"]["server_total"] = round(
            (response_ready - request_started) * 1000.0, 3
        )
        _save_debug_metadata(metadata)
        print(
            f"[OneRING] step={step_count} action={action} "
            f"input={navigation_rgb.shape}/{manipulation_rgb.shape} "
            f"debug={Path(args.debug_dir).expanduser().resolve()}",
            flush=True,
        )
    return jsonify({
        "action": str(action),
        "action_index": names.index(action),
        "action_names": names,
        "instruction": instruction,
        "latency_ms": metadata["latency_ms"],
    })


def main() -> None:
    global agent, args
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", default=str(DEFAULT_REPO))
    parser.add_argument("--checkpoint", default=str(DEFAULT_CHECKPOINT))
    parser.add_argument("--siglip-dir", default=str(DEFAULT_SIGLIP))
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--max-seq-len", type=int, default=128)
    parser.add_argument("--max-steps", type=int, default=80)
    parser.add_argument("--debug-dir", default="/media/fishyu/fish-14tb-12/YiFei/OneRING_Debug")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7801)
    args = parser.parse_args()
    if args.max_steps > args.max_seq_len:
        raise ValueError("--max-steps must be <= --max-seq-len for OneRING's KV cache")
    agent = _load_agent()
    app.run(host=args.host, port=args.port, threaded=False)


if __name__ == "__main__":
    main()
