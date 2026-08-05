# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations

import math
from typing import Any, Callable


_AT_POSITION_TOL_M = 0.5
_FACING_TOL_RAD = math.radians(20.0)


def _get_base(agent: Any) -> Any | None:
    if agent is None:
        return None
    base = getattr(agent, "_base", None)
    if base is None:
        return None
    if getattr(base, "_connected", True) is False:
        return None
    return base


def _base_position(base: Any) -> list[float] | None:
    try:
        pos = base.get_position()
        return [float(pos[0]), float(pos[1]), float(pos[2])]
    except Exception:
        return None


def _base_heading(base: Any) -> float | None:
    try:
        return float(base.get_heading())
    except Exception:
        return None


def _angle_delta(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def make_at_position(agent: Any) -> Callable[..., bool]:
    def at_position(x: Any, y: Any, tol: Any = _AT_POSITION_TOL_M) -> bool:
        base = _get_base(agent)
        if base is None:
            return False
        try:
            tx, ty, t = float(x), float(y), float(tol)
        except (TypeError, ValueError):
            return False
        pos = _base_position(base)
        if pos is None:
            return False
        return math.dist((pos[0], pos[1]), (tx, ty)) <= t

    return at_position


def make_facing(agent: Any) -> Callable[..., bool]:
    def facing(heading: Any, tol: Any = _FACING_TOL_RAD) -> bool:
        base = _get_base(agent)
        if base is None:
            return False
        try:
            target, t = float(heading), float(tol)
        except (TypeError, ValueError):
            return False
        yaw = _base_heading(base)
        if yaw is None:
            return False
        return _angle_delta(yaw, target) <= t

    return facing


def make_visited(agent: Any, rooms: dict[str, tuple[float, float, float, float]]) -> Callable[..., bool]:
    room_boxes = {
        str(name): tuple(float(v) for v in box)
        for name, box in (rooms or {}).items()
    }

    def visited(room: Any) -> bool:
        base = _get_base(agent)
        if base is None:
            return False
        box = room_boxes.get(str(room))
        if box is None:
            return False
        pos = _base_position(base)
        if pos is None:
            return False
        x_min, y_min, x_max, y_max = box
        return x_min <= pos[0] <= x_max and y_min <= pos[1] <= y_max

    return visited

def _is_box(box: Any) -> bool:
    """True if *box* coerces to a 4-tuple of floats ``(x_min, y_min, x_max, y_max)``."""
    try:
        x_min, y_min, x_max, y_max = (float(v) for v in box)
    except (TypeError, ValueError):
        return False
    return True

def make_rooms_producer(
    rooms: dict[str, tuple[float, float, float, float]],
) -> Callable[..., dict[str, Any]]:
    room_boxes = {
        str(name): tuple(float(v) for v in box)
        for name, box in (rooms or {}).items()
    }

    def rooms_producer(**_: Any) -> dict[str, Any]:
        out: list[dict[str, Any]] = []
        for name in sorted(room_boxes):
            x_min, y_min, x_max, y_max = room_boxes[name]
            out.append(
                {
                    "name": name,
                    "x": (x_min + x_max) / 2.0,
                    "y": (y_min + y_max) / 2.0,
                }
            )
        return {"rooms": out, "count": len(out)}

    return rooms_producer