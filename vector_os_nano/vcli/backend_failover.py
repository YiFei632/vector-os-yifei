# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Bounded failover for explicit simulation-lifecycle commands.

The model remains the normal tool selector. This module is consulted only after
the model backend has failed with a transient availability error, and it can
resolve only an unambiguous start/stop simulation command. The returned tool is
still dispatched through the normal registry, permission gate, hooks, and
``SimStartTool`` / ``SimStopTool`` implementation.

This deliberately is not a general natural-language planner. Navigation,
manipulation, diagnostics, and ambiguous simulation requests remain model-owned.
"""
from __future__ import annotations

import re
import unicodedata
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class LocalToolFallback:
    """One fully resolved local tool call allowed during a backend outage."""

    tool_name: str
    params: dict[str, Any]


_RATE_LIMIT_MARKERS: tuple[str, ...] = (
    "rate limit",
    "rate_limit",
    "ratelimit",
    "too many requests",
    "would exceed account",
    "would exceed accounts",
    "请求过于频繁",
    "限流",
)

_TEMPORARY_MARKERS: tuple[str, ...] = (
    "temporarily unavailable",
    "service unavailable",
    "server overloaded",
    "overloaded_error",
    "connection error",
    "connection refused",
    "connection reset",
    "timed out",
    "timeout",
)

_START_ZH: tuple[str, ...] = ("打开", "启动", "开启", "运行", "开始")
_STOP_ZH: tuple[str, ...] = ("关闭", "停止", "停掉", "关掉", "退出", "结束")
_START_EN: tuple[str, ...] = ("open", "start", "launch", "run")
_STOP_EN: tuple[str, ...] = ("close", "stop", "shutdown", "exit", "terminate")
_SIM_ZH: tuple[str, ...] = ("仿真", "模拟器")
_SIM_EN: tuple[str, ...] = ("sim", "simulation", "simulator")
_NON_COMMAND_ZH: tuple[str, ...] = (
    "为什么",
    "分析",
    "原因",
    "报错",
    "错误",
    "失败",
    "问题",
    "日志",
    "怎么",
    "如何",
    "检查",
    "是否",
)
_NON_COMMAND_EN: tuple[str, ...] = (
    "why",
    "analyze",
    "analyse",
    "reason",
    "error",
    "failed",
    "failure",
    "problem",
    "issue",
    "log",
    "how",
    "debug",
)
_COMPLEX_ZH: tuple[str, ...] = ("然后", "接着", "随后", "同时", "并且")
_COMPLEX_EN: tuple[str, ...] = ("then", "afterwards", "meanwhile")


def _normalise(text: object) -> str:
    value = unicodedata.normalize("NFKC", str(text)).lower()
    return " ".join(value.split())


def _has_word(text: str, words: tuple[str, ...]) -> bool:
    return any(
        re.search(rf"(?<![a-z0-9]){re.escape(word)}(?![a-z0-9])", text)
        for word in words
    )


def is_rate_limit_error(exc: BaseException) -> bool:
    """Recognise real 429s and providers that disguise rate limits as HTTP 500."""

    status = getattr(exc, "status_code", None)
    if status is None:
        status = getattr(getattr(exc, "response", None), "status_code", None)
    if status == 429:
        return True
    text = _normalise(f"{type(exc).__name__}: {exc}")
    return any(marker in text for marker in _RATE_LIMIT_MARKERS)


def is_transient_backend_error(exc: BaseException) -> bool:
    """Return whether retrying the model later could plausibly succeed."""

    if is_rate_limit_error(exc):
        return True

    status = getattr(exc, "status_code", None)
    if status is None:
        status = getattr(getattr(exc, "response", None), "status_code", None)
    if status in (502, 503, 504):
        return True

    text = _normalise(f"{type(exc).__name__}: {exc}")
    if any(marker in text for marker in _TEMPORARY_MARKERS):
        return True
    return any(
        marker in type(exc).__name__.lower()
        for marker in ("connectionerror", "connecterror", "timeouterror")
    )


def _has_start(text: str) -> bool:
    return any(word in text for word in _START_ZH) or _has_word(text, _START_EN)


def _has_stop(text: str) -> bool:
    return any(word in text for word in _STOP_ZH) or _has_word(text, _STOP_EN)


def _has_sim_noun(text: str) -> bool:
    return any(word in text for word in _SIM_ZH) or _has_word(text, _SIM_EN)


def _negates_lifecycle_action(text: str) -> bool:
    zh_actions = "|".join((*_START_ZH, *_STOP_ZH))
    if re.search(rf"(?:不要|别|不用|不准)\s*(?:{zh_actions})", text):
        return True
    return bool(
        re.search(
            r"\b(?:do not|don't|dont|never)\s+"
            r"(?:open|start|launch|run|close|stop|shutdown|exit|terminate)\b",
            text,
        )
    )


def _start_params(text: str) -> dict[str, Any] | None:
    """Resolve only explicitly selected embodiments; generic Go2 stays ambiguous."""

    dog_arm = (
        "狗臂" in text
        or "背载机械臂" in text
        or ("带机械臂" in text and "不带机械臂" not in text)
        or "piper" in text
        or bool(re.search(r"\bgo2\s*(?:\+|with)?\s*(?:arm|piper)\b", text))
    )
    pure_go2 = (
        "纯四足" in text
        or "四足模式" in text
        or "四足仿真" in text
        or "不带机械臂" in text
        or "无机械臂" in text
        or _has_word(text, ("quadruped",))
        or bool(re.search(r"\bpure\s+go2\b|\bgo2\s+only\b", text))
    )
    arm_only = not dog_arm and (
        "单机械臂" in text
        or "机械臂仿真" in text
        or "机械臂模拟器" in text
        or "so-101" in text
        or "so101" in text
        or bool(
            re.search(
                r"\barm(?:-only)?\s+(?:sim|simulation|simulator)\b",
                text,
            )
        )
    )

    selected = sum((dog_arm, pure_go2, arm_only))
    if selected != 1:
        return None

    if dog_arm:
        params: dict[str, Any] = {"sim_type": "go2", "with_arm": True}
    elif pure_go2:
        params = {"sim_type": "go2", "with_arm": False}
    else:
        params = {"sim_type": "arm"}

    if "无窗口" in text or "不要窗口" in text or _has_word(text, ("headless",)):
        params["gui"] = False
    if _has_word(text, ("gazebo", "isaac", "mujoco")):
        for backend in ("gazebo", "isaac", "mujoco"):
            if _has_word(text, (backend,)):
                params["backend"] = backend
                break
    return params


def resolve_sim_lifecycle_fallback(user_message: str) -> LocalToolFallback | None:
    """Resolve one explicit simulation start/stop command, otherwise ``None``.

    Start commands must name exactly one supported embodiment. In particular,
    generic ``启动仿真`` and bare ``启动 Go2 仿真`` stay unresolved because the
    normal tool contract requires choosing pure quadruped versus dog-arm mode.
    """

    text = _normalise(user_message)
    if (
        not text
        or not _has_sim_noun(text)
        or _negates_lifecycle_action(text)
        or any(marker in text for marker in (*_NON_COMMAND_ZH, *_COMPLEX_ZH))
        or _has_word(text, (*_NON_COMMAND_EN, *_COMPLEX_EN))
    ):
        return None

    has_start = _has_start(text)
    has_stop = _has_stop(text)
    if has_start == has_stop:
        return None

    if has_stop:
        return LocalToolFallback("stop_simulation", {})

    params = _start_params(text)
    if params is None:
        return None
    return LocalToolFallback("start_simulation", params)

