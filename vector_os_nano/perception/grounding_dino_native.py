# SPDX-License-Identifier: Apache-2.0
"""Lazy adapter for the original IDEA-Research GroundingDINO checkpoint.

The project already contains a Transformers based detector, but the supplied
``groundingdino_swint_ogc.pth`` is the native repository format.  This adapter
loads that exact checkpoint without downloading a Hugging Face model.
"""
from __future__ import annotations

import logging
import os
import sys
import importlib.util
from pathlib import Path
from typing import Any

import numpy as np

from vector_os_nano.core.types import Detection
from vector_os_nano.perception.grounding_dino import query_to_prompt

logger = logging.getLogger(__name__)

_DEFAULT_REPO = "/media/fishyu/fish-14tb-12/YiFei/GroundingDINO"
_DEFAULT_CHECKPOINT = (
    "/media/fishyu/fish-14tb-12/YiFei/GroundingDINO/checkpoints/"
    "groundingdino_swint_ogc.pth"
)


class NativeGroundingDinoDetector:
    """Open-vocabulary detector backed by a local native GroundingDINO repo."""

    def __init__(
        self,
        *,
        repository: str | None = None,
        checkpoint: str | None = None,
        config: str | None = None,
        device: str | None = None,
        box_threshold: float = 0.25,
        text_threshold: float = 0.20,
    ) -> None:
        self.repository = Path(
            repository or os.environ.get("GROUNDING_DINO_REPO", _DEFAULT_REPO)
        ).expanduser()
        self.checkpoint = Path(
            checkpoint
            or os.environ.get("GROUNDING_DINO_CHECKPOINT", _DEFAULT_CHECKPOINT)
        ).expanduser()
        self.config = Path(
            config
            or os.environ.get(
                "GROUNDING_DINO_CONFIG",
                str(self.repository / "groundingdino/config/GroundingDINO_SwinT_OGC.py"),
            )
        ).expanduser()
        self.requested_device = device or os.environ.get("GROUNDING_DINO_DEVICE")
        self.box_threshold = float(box_threshold)
        self.text_threshold = float(text_threshold)
        self._model: Any = None
        self._transform: Any = None
        self._predict: Any = None
        self._torch: Any = None
        self._device = "cpu"

    def _ensure_loaded(self) -> None:
        if self._model is not None:
            return
        for path, label in (
            (self.repository, "repository"),
            (self.checkpoint, "checkpoint"),
            (self.config, "config"),
        ):
            if not path.exists():
                raise FileNotFoundError(f"GroundingDINO {label} not found: {path}")

        # Prefer the installed package because it carries the compiled ``_C``
        # extension. Fall back to the source tree only in development installs.
        repo_text = str(self.repository)
        if importlib.util.find_spec("groundingdino") is None and repo_text not in sys.path:
            sys.path.insert(0, repo_text)
        try:
            import torch
            import groundingdino.datasets.transforms as transforms
            from groundingdino.util.inference import load_model, predict
        except ImportError as exc:
            raise ImportError(
                "Native GroundingDINO dependencies are missing. Install the local "
                "GroundingDINO package/dependencies in the vector_os_nano environment."
            ) from exc

        self._torch = torch
        self._device = self.requested_device or (
            "cuda" if torch.cuda.is_available() else "cpu"
        )
        logger.info("loading native GroundingDINO on %s", self._device)
        self._model = load_model(
            str(self.config), str(self.checkpoint), device=self._device
        )
        self._model.eval()
        self._predict = predict
        self._transform = transforms.Compose(
            [
                transforms.RandomResize([800], max_size=1333),
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                ),
            ]
        )

    def detect(self, rgb: np.ndarray, query: str) -> list[Detection]:
        if rgb is None or not isinstance(rgb, np.ndarray) or rgb.ndim != 3:
            raise ValueError("detect requires an (H,W,3) RGB array")
        self._ensure_loaded()
        from PIL import Image

        height, width = rgb.shape[:2]
        image_source = Image.fromarray(np.asarray(rgb, dtype=np.uint8))
        image, _ = self._transform(image_source, None)
        boxes, logits, phrases = self._predict(
            model=self._model,
            image=image,
            caption=query_to_prompt(query),
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            device=self._device,
        )

        detections: list[Detection] = []
        for index, box in enumerate(boxes):
            cx, cy, bw, bh = [float(v) for v in box.tolist()]
            x1 = max(0.0, (cx - bw / 2.0) * width)
            y1 = max(0.0, (cy - bh / 2.0) * height)
            x2 = min(float(width), (cx + bw / 2.0) * width)
            y2 = min(float(height), (cy + bh / 2.0) * height)
            score = float(logits[index])
            if score < self.box_threshold:
                continue
            label = str(phrases[index] if index < len(phrases) else query).strip()
            detections.append(
                Detection(
                    label=label or query or "object",
                    bbox=(x1, y1, x2, y2),
                    confidence=score,
                )
            )
        detections.sort(key=lambda item: item.confidence, reverse=True)
        return detections


__all__ = ["NativeGroundingDinoDetector"]
