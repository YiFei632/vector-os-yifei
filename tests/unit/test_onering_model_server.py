from __future__ import annotations

import base64
import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch
import pytest
from PIL import Image


def _module():
    path = Path(__file__).resolve().parents[2] / "scripts" / "onering_model_server.py"
    spec = importlib.util.spec_from_file_location("onering_model_server_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    try:
        spec.loader.exec_module(module)
    except ImportError as exc:
        pytest.skip(f"OneRING service dependencies unavailable in test environment: {exc}")
    return module


def _png() -> str:
    buffer = io.BytesIO()
    Image.fromarray(np.zeros((8, 10, 3), dtype=np.uint8)).save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def test_step_disables_autograd_saves_debug_and_bounds_episode_state(tmp_path) -> None:
    module = _module()

    class Agent:
        def __init__(self):
            self.grad_enabled = None

        def reset(self):
            pass

        def get_action(self, observations, instruction):
            assert observations["raw_navigation_camera"].shape == (8, 10, 3)
            assert instruction == "go to chair"
            self.grad_enabled = torch.is_grad_enabled()
            return "move_ahead", torch.ones(20) / 20.0

        def get_action_list(self):
            return ["move_ahead"] + [f"action_{index}" for index in range(1, 20)]

    module.agent = Agent()
    module.args = SimpleNamespace(max_steps=1, debug_dir=str(tmp_path))
    module.active_instruction = ""
    module.step_count = 0
    client = module.app.test_client()

    assert client.post("/reset", json={"instruction": "go to chair"}).status_code == 200
    # A missing manipulation frame aliases the navigation frame so RBY-1 does
    # not transmit the same GoPro image twice.
    payload = {"navigation_rgb_png": _png()}
    step_response = client.post("/step", json=payload)
    assert step_response.status_code == 200
    assert step_response.get_json()["latency_ms"]["server_total"] >= 0.0
    assert module.agent.grad_enabled is False
    assert (tmp_path / "latest_navigation.png").is_file()
    assert (tmp_path / "latest_manipulation.png").is_file()
    assert (tmp_path / "latest.json").is_file()
    assert client.get("/debug").status_code == 200
    response = client.post("/step", json=payload)
    assert response.status_code == 500
    assert "exceeded max_steps=1" in response.get_json()["error"]
