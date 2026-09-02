# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Native OpenAI Responses API backend for Vector CLI.

Vector's engine keeps one provider-neutral, Anthropic-like conversation format.
This module converts that format to Responses input items and converts streamed
text/function calls back to :class:`LLMResponse`.
"""
from __future__ import annotations

import json
import logging
import os
import time
from typing import Any, Callable

import openai

from vector_os_nano.vcli.backend_failover import is_rate_limit_error
from vector_os_nano.vcli.backends.types import LLMResponse, LLMToolCall
from vector_os_nano.vcli.session import TokenUsage

logger = logging.getLogger(__name__)


def _field(value: Any, name: str, default: Any = None) -> Any:
    if isinstance(value, dict):
        return value.get(name, default)
    return getattr(value, name, default)


def convert_system(system_blocks: list[dict[str, Any]]) -> str:
    """Join Vector system blocks into Responses ``instructions`` text."""
    return "\n\n".join(
        str(block.get("text", ""))
        for block in system_blocks
        if block.get("text")
    )


def convert_tools(tools: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Convert Anthropic function schemas to Responses function tools."""
    return [
        {
            "type": "function",
            "name": tool["name"],
            "description": tool.get("description", ""),
            "parameters": tool.get(
                "input_schema", {"type": "object", "properties": {}}
            ),
        }
        for tool in tools
    ]


def _stringify_tool_output(content: Any) -> str:
    if isinstance(content, str):
        return content
    return json.dumps(content, ensure_ascii=False)


def convert_messages(messages: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Convert Vector messages into Responses input items.

    Tool history must be represented as explicit ``function_call`` and
    ``function_call_output`` items. Plain user/assistant text remains a message.
    """
    items: list[dict[str, Any]] = []
    for message in messages:
        role = str(message["role"])
        content = message.get("content")
        if isinstance(content, str):
            items.append({"role": role, "content": content})
            continue
        if not isinstance(content, list):
            continue

        text_parts: list[str] = []
        deferred: list[dict[str, Any]] = []
        for block in content:
            if not isinstance(block, dict):
                text_parts.append(str(block))
                continue
            block_type = block.get("type")
            if block_type == "text":
                text_parts.append(str(block.get("text", "")))
            elif block_type == "tool_use":
                deferred.append(
                    {
                        "type": "function_call",
                        "call_id": str(block["id"]),
                        "name": str(block["name"]),
                        "arguments": json.dumps(
                            block.get("input", {}), ensure_ascii=False
                        ),
                    }
                )
            elif block_type == "tool_result":
                deferred.append(
                    {
                        "type": "function_call_output",
                        "call_id": str(block["tool_use_id"]),
                        "output": _stringify_tool_output(block.get("content", "")),
                    }
                )

        text = "".join(text_parts).strip()
        if text:
            items.append({"role": role, "content": text})
        items.extend(deferred)
    return items


def parse_usage(raw_usage: Any) -> TokenUsage:
    """Extract token accounting from a Responses usage object."""
    if raw_usage is None:
        return TokenUsage()
    details = _field(raw_usage, "input_tokens_details")
    return TokenUsage(
        input_tokens=int(_field(raw_usage, "input_tokens", 0) or 0),
        output_tokens=int(_field(raw_usage, "output_tokens", 0) or 0),
        cache_read_tokens=int(_field(details, "cached_tokens", 0) or 0),
    )


def parse_tool_calls(response: Any) -> list[LLMToolCall]:
    """Collect completed function calls from a final Responses object."""
    result: list[LLMToolCall] = []
    for item in _field(response, "output", []) or []:
        if _field(item, "type") != "function_call":
            continue
        raw_arguments = _field(item, "arguments", "") or ""
        try:
            parsed = json.loads(raw_arguments) if raw_arguments else {}
        except json.JSONDecodeError:
            parsed = {"_raw": raw_arguments}
        result.append(
            LLMToolCall(
                id=str(_field(item, "call_id") or _field(item, "id") or ""),
                name=str(_field(item, "name", "")),
                input=parsed,
            )
        )
    return result


def _stop_reason(response: Any, tool_calls: list[LLMToolCall]) -> str:
    if tool_calls:
        return "tool_use"
    if _field(response, "status") == "incomplete":
        details = _field(response, "incomplete_details")
        reason = str(_field(details, "reason", ""))
        if "token" in reason:
            return "max_tokens"
    return "end_turn"


class OpenAIResponsesBackend:
    """Backend for OpenAI-compatible implementations of ``/v1/responses``."""

    def __init__(
        self,
        api_key: str,
        model: str,
        base_url: str = "https://api.openai.com/v1",
        max_retries: int = 3,
        reasoning_effort: str | None = None,
    ) -> None:
        self._client = openai.OpenAI(api_key=api_key, base_url=base_url)
        self._model = model
        self._max_retries = max_retries
        self._reasoning_effort = (
            reasoning_effort
            if reasoning_effort is not None
            else os.environ.get("VECTOR_REASONING_EFFORT", "").strip()
        )

    def call(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
        system: list[dict[str, Any]],
        max_tokens: int,
        on_text: Callable[[str], None] | None = None,
        on_reasoning: Callable[[str], None] | None = None,
    ) -> LLMResponse:
        return self._call_with_retry(
            messages, tools, system, max_tokens, on_text, on_reasoning
        )

    def _call_with_retry(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
        system: list[dict[str, Any]],
        max_tokens: int,
        on_text: Callable[[str], None] | None,
        on_reasoning: Callable[[str], None] | None,
    ) -> LLMResponse:
        last_exc: Exception | None = None
        for attempt in range(self._max_retries):
            try:
                return self._call_streaming(
                    messages, tools, system, max_tokens, on_text, on_reasoning
                )
            except openai.RateLimitError as exc:
                last_exc = exc
            except openai.APIConnectionError as exc:
                last_exc = exc
            except openai.InternalServerError as exc:
                if is_rate_limit_error(exc):
                    raise
                last_exc = exc
            except openai.APIStatusError:
                raise

            delay = 2**attempt
            logger.warning(
                "OpenAI Responses call failed (attempt %d/%d), retrying in %ds",
                attempt + 1,
                self._max_retries,
                delay,
            )
            time.sleep(delay)
        raise last_exc  # type: ignore[misc]

    def _call_streaming(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
        system: list[dict[str, Any]],
        max_tokens: int,
        on_text: Callable[[str], None] | None,
        on_reasoning: Callable[[str], None] | None,
    ) -> LLMResponse:
        kwargs: dict[str, Any] = {
            "model": self._model,
            "input": convert_messages(messages),
            "max_output_tokens": max_tokens,
            "store": False,
        }
        instructions = convert_system(system)
        if instructions:
            kwargs["instructions"] = instructions
        if tools:
            kwargs["tools"] = convert_tools(tools)
        if self._reasoning_effort:
            kwargs["reasoning"] = {"effort": self._reasoning_effort}

        text_parts: list[str] = []
        with self._client.responses.stream(**kwargs) as stream:
            for event in stream:
                event_type = str(_field(event, "type", ""))
                delta = _field(event, "delta")
                if event_type == "response.output_text.delta" and delta:
                    text_parts.append(str(delta))
                    if on_text is not None:
                        on_text(str(delta))
                elif (
                    on_reasoning is not None
                    and "reasoning" in event_type
                    and event_type.endswith(".delta")
                    and delta
                ):
                    on_reasoning(str(delta))
            response = stream.get_final_response()

        final_text = "".join(text_parts)
        if not final_text:
            final_text = str(_field(response, "output_text", "") or "")
            if final_text and on_text is not None:
                on_text(final_text)
        tool_calls = parse_tool_calls(response)
        return LLMResponse(
            text=final_text,
            tool_calls=tool_calls,
            stop_reason=_stop_reason(response, tool_calls),
            usage=parse_usage(_field(response, "usage")),
        )

