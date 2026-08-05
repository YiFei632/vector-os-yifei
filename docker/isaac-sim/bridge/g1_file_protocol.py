#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics
"""Versioned atomic-file protocol shared by the G1 ROS and DDS processes."""
from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

from g1_manifest import COMMAND_CHANNEL_SIZES, G1Manifest, validate_joint_vector


SCHEMA_VERSION = 1
COMMAND_FILE = "g1_command.json"
STATE_FILE = "g1_state.json"
MANIFEST_FILE = "robot_manifest.json"


def monotonic_ns() -> int:
    return time.monotonic_ns()


def atomic_write_json(path: str | Path, document: Mapping[str, Any]) -> None:
    """Atomically replace a small JSON document on the shared tmpfs."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    try:
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(document, handle, separators=(",", ":"), allow_nan=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, target)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def read_json(path: str | Path) -> dict[str, Any] | None:
    """Read a protocol document; a concurrent/missing write is not fatal."""
    try:
        with Path(path).open("r", encoding="utf-8") as handle:
            document = json.load(handle)
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return None
    if not isinstance(document, dict):
        return None
    return document


def new_command_document() -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "robot_type": G1Manifest().robot_type,
        "sequence": 0,
        "channels": {},
    }


def update_command_channel(
    document: Mapping[str, Any] | None,
    channel: str,
    values: Sequence[float],
    *,
    ttl_ms: int = 250,
    timestamp_ns: int | None = None,
) -> dict[str, Any]:
    """Return a copy with one independently timestamped command channel."""
    if not 20 <= int(ttl_ms) <= 10_000:
        raise ValueError("G1 command ttl_ms must be between 20 and 10000")
    normalized = validate_joint_vector(channel, values)
    current = dict(document or new_command_document())
    if current.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Unsupported G1 command schema version")
    channels = dict(current.get("channels") or {})
    sequence = int(current.get("sequence", 0)) + 1
    channels[channel] = {
        "sequence": sequence,
        "monotonic_ns": int(timestamp_ns if timestamp_ns is not None else monotonic_ns()),
        "ttl_ms": int(ttl_ms),
        "values": normalized,
    }
    current.update({"sequence": sequence, "channels": channels})
    return current


def fresh_channel_values(
    document: Mapping[str, Any] | None,
    channel: str,
    *,
    timestamp_ns: int | None = None,
) -> list[float] | None:
    """Return a valid non-expired channel, otherwise ``None``."""
    if channel not in COMMAND_CHANNEL_SIZES or not document:
        return None
    if document.get("schema_version") != SCHEMA_VERSION:
        return None
    entry = (document.get("channels") or {}).get(channel)
    if not isinstance(entry, dict):
        return None
    try:
        sent_ns = int(entry["monotonic_ns"])
        ttl_ns = int(entry["ttl_ms"]) * 1_000_000
        now_ns = int(timestamp_ns if timestamp_ns is not None else monotonic_ns())
        if now_ns < sent_ns or now_ns - sent_ns > ttl_ns:
            return None
        return validate_joint_vector(channel, entry["values"])
    except (KeyError, TypeError, ValueError):
        return None


def write_manifest(state_dir: str | Path, extra: Mapping[str, Any] | None = None) -> None:
    document = G1Manifest().as_dict()
    if extra:
        document.update(extra)
    atomic_write_json(Path(state_dir) / MANIFEST_FILE, document)
