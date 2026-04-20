#!/usr/bin/env python3
"""Generate protocol constants from the shared LSH protocol specification.

The generator is intentionally strict:
- it validates the structure of the shared spec before emitting code
- it keeps target-specific identifiers explicit instead of inferring them
- it rewrites files only when content really changed
- it supports a `--check` mode for CI or local drift detection

The wire protocol stays untouched. Only code identifiers and generated files
are derived from the shared specification.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


ROOT = Path(__file__).resolve().parent.parent
SPEC_PATH = ROOT / "shared" / "lsh_protocol.json"
GOLDEN_PAYLOADS_PATH = ROOT / "shared" / "lsh_protocol_golden_payloads.json"
TARGET_CORE = "core"
TARGET_BRIDGE = "bridge"
VALID_TARGETS = {TARGET_CORE, TARGET_BRIDGE}
CLI_TARGET_SHARED_DOC = "shared-doc"
CLI_TARGET_NODE_RED = "node-red"
VALID_CLI_TARGETS = (
    CLI_TARGET_SHARED_DOC,
    TARGET_CORE,
    TARGET_BRIDGE,
    CLI_TARGET_NODE_RED,
)
CPP_IDENTIFIER_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
TS_IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

MSGPACK_FRAME_END = 0xC0
MSGPACK_FRAME_ESCAPE = 0xDB
MSGPACK_FRAME_ESCAPED_END = 0xDC
MSGPACK_FRAME_ESCAPED_ESCAPE = 0xDD


class SpecError(ValueError):
    """Raised when the shared protocol specification is invalid."""


@dataclass(frozen=True)
class CommandSpec:
    """Single protocol command definition."""

    name: str
    value: int
    description: str
    cpp_name: str
    ts_name: str

    @classmethod
    def from_dict(cls, raw: object) -> "CommandSpec":
        if not isinstance(raw, dict):
            raise SpecError("Each command entry must be an object.")

        name = require_identifier(raw, "name", CPP_IDENTIFIER_RE)
        value = require_uint8(raw, "value")
        description = require_non_empty_string(raw, "description")
        cpp_name = optional_identifier(raw, "cppName", CPP_IDENTIFIER_RE) or name
        ts_name = optional_identifier(raw, "tsName", TS_IDENTIFIER_RE) or name
        return cls(
            name=name,
            value=value,
            description=description,
            cpp_name=cpp_name,
            ts_name=ts_name,
        )


@dataclass(frozen=True)
class ClickTypeSpec:
    """Single click-type definition."""

    cpp_name: str
    ts_name: str
    value: int

    @classmethod
    def from_dict(cls, raw: object) -> "ClickTypeSpec":
        if not isinstance(raw, dict):
            raise SpecError("Each clickType entry must be an object.")

        return cls(
            cpp_name=require_identifier(raw, "cppName", CPP_IDENTIFIER_RE),
            ts_name=require_identifier(raw, "tsName", TS_IDENTIFIER_RE),
            value=require_uint8(raw, "value"),
        )


@dataclass(frozen=True)
class StaticPayloadSpec:
    """Pre-serialized payload emitted as compile-time bytes."""

    name: str
    command: str
    cpp_name: str
    symbol_name: str
    targets: tuple[str, ...]

    @classmethod
    def from_dict(cls, raw: object) -> "StaticPayloadSpec":
        if not isinstance(raw, dict):
            raise SpecError("Each staticPayload entry must be an object.")

        name = require_identifier(raw, "name", CPP_IDENTIFIER_RE)
        command = require_identifier(raw, "command", CPP_IDENTIFIER_RE)
        cpp_name = optional_identifier(raw, "cppName", CPP_IDENTIFIER_RE) or name
        symbol_name = optional_identifier(raw, "symbolName", CPP_IDENTIFIER_RE) or name
        targets = require_targets(raw, "targets")
        return cls(
            name=name,
            command=command,
            cpp_name=cpp_name,
            symbol_name=symbol_name,
            targets=targets,
        )


@dataclass(frozen=True)
class DocumentationSpec:
    """Human-readable documentation metadata that does not affect the wire format."""

    trusted_environment: str | None
    handshake: tuple[str, ...]
    compatibility: tuple[str, ...]
    transport: tuple[str, ...]
    constraints: tuple[str, ...]

    @classmethod
    def from_dict(cls, raw: object) -> "DocumentationSpec":
        if raw is None:
            return cls(
                trusted_environment=None,
                handshake=(),
                compatibility=(),
                transport=(),
                constraints=(),
            )
        if not isinstance(raw, dict):
            raise SpecError("'meta.documentation' must be an object when present.")

        return cls(
            trusted_environment=optional_non_empty_string(raw, "trustedEnvironment"),
            handshake=require_optional_string_array(raw, "handshake"),
            compatibility=require_optional_string_array(raw, "compatibility"),
            transport=require_optional_string_array(raw, "transport"),
            constraints=require_optional_string_array(raw, "constraints"),
        )


@dataclass(frozen=True)
class ProtocolSpec:
    """Validated, typed protocol specification."""

    name: str
    spec_revision: int
    wire_protocol_major: int
    notes: str
    documentation: DocumentationSpec
    keys: dict[str, str]
    commands: tuple[CommandSpec, ...]
    click_types: tuple[ClickTypeSpec, ...]
    ts_aliases: dict[str, str]
    static_payloads: tuple[StaticPayloadSpec, ...]

    @classmethod
    def from_dict(cls, raw: object) -> "ProtocolSpec":
        if not isinstance(raw, dict):
            raise SpecError("The top-level protocol spec must be an object.")

        meta = require_object(raw, "meta")
        keys = require_string_map(raw, "keys")

        name = require_non_empty_string(meta, "name")
        spec_revision = require_positive_int(meta, "specRevision")
        wire_protocol_major = require_uint8(meta, "wireProtocolMajor")
        notes = require_non_empty_string(meta, "notes")
        documentation = DocumentationSpec.from_dict(meta.get("documentation"))
        commands = tuple(CommandSpec.from_dict(item) for item in require_array(raw, "commands"))
        click_types = tuple(ClickTypeSpec.from_dict(item) for item in require_array(raw, "clickTypes"))
        ts_aliases = require_optional_string_map(raw, "tsAliases")
        static_payloads = tuple(
            StaticPayloadSpec.from_dict(item) for item in require_array(raw, "staticPayloads")
        )

        spec = cls(
            name=name,
            spec_revision=spec_revision,
            wire_protocol_major=wire_protocol_major,
            notes=notes,
            documentation=documentation,
            keys=keys,
            commands=commands,
            click_types=click_types,
            ts_aliases=ts_aliases,
            static_payloads=static_payloads,
        )
        spec.validate()
        return spec

    def command_by_name(self) -> dict[str, CommandSpec]:
        return {command.name: command for command in self.commands}

    def static_payloads_for(self, target: str) -> tuple[StaticPayloadSpec, ...]:
        return tuple(payload for payload in self.static_payloads if target in payload.targets)

    def validate(self) -> None:
        """Validate cross-field invariants and generator-facing identifiers."""

        ensure_unique((command.name for command in self.commands), "duplicate command name")
        ensure_unique((command.value for command in self.commands), "duplicate command numeric value")
        ensure_unique((command.cpp_name for command in self.commands), "duplicate C++ command identifier")
        ensure_unique((command.ts_name for command in self.commands), "duplicate TypeScript command identifier")

        ensure_unique((click.cpp_name for click in self.click_types), "duplicate C++ click-type identifier")
        ensure_unique((click.ts_name for click in self.click_types), "duplicate TypeScript click-type identifier")
        ensure_unique((click.value for click in self.click_types), "duplicate click-type numeric value")

        commands_by_name = self.command_by_name()
        for alias, target_name in self.ts_aliases.items():
            if target_name not in commands_by_name:
                raise SpecError(f"TypeScript alias '{alias}' targets unknown command '{target_name}'.")
            if alias in {command.ts_name for command in self.commands}:
                raise SpecError(
                    f"TypeScript alias '{alias}' collides with a generated TypeScript command name."
                )
            if not TS_IDENTIFIER_RE.fullmatch(alias):
                raise SpecError(f"TypeScript alias '{alias}' is not a valid identifier.")

        ensure_unique(self.ts_aliases.keys(), "duplicate TypeScript alias")

        for payload in self.static_payloads:
            if payload.command not in commands_by_name:
                raise SpecError(
                    f"Static payload '{payload.name}' targets unknown command '{payload.command}'."
                )

        for target in VALID_TARGETS:
            target_payloads = self.static_payloads_for(target)
            ensure_unique(
                (payload.cpp_name for payload in target_payloads),
                f"duplicate static-payload enum identifier for target '{target}'",
            )
            ensure_unique(
                (payload.symbol_name for payload in target_payloads),
                f"duplicate static-payload symbol identifier for target '{target}'",
            )


@dataclass(frozen=True)
class GoldenPayloads:
    """Golden JSON payload examples used for tests and human documentation."""

    payloads: dict[str, object]

    @classmethod
    def from_dict(cls, raw: object) -> "GoldenPayloads":
        if not isinstance(raw, dict):
            raise SpecError("The golden payload file must be an object.")

        payloads = raw.get("payloads")
        if not isinstance(payloads, dict) or not payloads:
            raise SpecError("'payloads' must be a non-empty object in the golden payload file.")

        validated: dict[str, object] = {}
        for name, payload in payloads.items():
            if not isinstance(name, str) or not name:
                raise SpecError("Golden payload names must be non-empty strings.")
            if not isinstance(payload, dict):
                raise SpecError(f"Golden payload '{name}' must be an object.")
            validated[name] = payload
        return cls(payloads=validated)


def require_object(raw: dict[str, object], key: str) -> dict[str, object]:
    value = raw.get(key)
    if not isinstance(value, dict):
        raise SpecError(f"'{key}' must be an object.")
    return value


def require_array(raw: dict[str, object], key: str) -> list[object]:
    value = raw.get(key)
    if not isinstance(value, list) or not value:
        raise SpecError(f"'{key}' must be a non-empty array.")
    return value


def require_string_map(raw: dict[str, object], key: str) -> dict[str, str]:
    value = raw.get(key)
    if not isinstance(value, dict) or not value:
        raise SpecError(f"'{key}' must be a non-empty object.")

    result: dict[str, str] = {}
    for entry_key, entry_value in value.items():
        if not isinstance(entry_key, str) or not entry_key:
            raise SpecError(f"'{key}' contains an invalid key.")
        if not isinstance(entry_value, str) or not entry_value:
            raise SpecError(f"'{key}.{entry_key}' must be a non-empty string.")
        result[entry_key] = entry_value
    return result


def require_optional_string_map(raw: dict[str, object], key: str) -> dict[str, str]:
    value = raw.get(key)
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise SpecError(f"'{key}' must be an object when present.")

    result: dict[str, str] = {}
    for entry_key, entry_value in value.items():
        if not isinstance(entry_key, str) or not isinstance(entry_value, str):
            raise SpecError(f"'{key}' must map strings to strings.")
        if not entry_key or not entry_value:
            raise SpecError(f"'{key}' cannot contain empty strings.")
        result[entry_key] = entry_value
    return result


def require_positive_int(raw: dict[str, object], key: str) -> int:
    value = raw.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise SpecError(f"'{key}' must be a positive integer.")
    return value


def require_uint8(raw: dict[str, object], key: str) -> int:
    value = raw.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 0xFF:
        raise SpecError(f"'{key}' must be an integer between 0 and 255.")
    return value


def require_non_empty_string(raw: dict[str, object], key: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value.strip():
        raise SpecError(f"'{key}' must be a non-empty string.")
    return value


def optional_non_empty_string(raw: dict[str, object], key: str) -> str | None:
    value = raw.get(key)
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise SpecError(f"'{key}' must be a non-empty string when present.")
    return value


def require_optional_string_array(raw: dict[str, object], key: str) -> tuple[str, ...]:
    value = raw.get(key)
    if value is None:
        return ()
    if not isinstance(value, list):
        raise SpecError(f"'{key}' must be an array when present.")

    items: list[str] = []
    for item in value:
        if not isinstance(item, str) or not item.strip():
            raise SpecError(f"'{key}' can only contain non-empty strings.")
        items.append(item)
    return tuple(items)


def require_identifier(raw: dict[str, object], key: str, pattern: re.Pattern[str]) -> str:
    value = require_non_empty_string(raw, key)
    if not pattern.fullmatch(value):
        raise SpecError(f"'{key}' value '{value}' is not a valid identifier.")
    return value


def optional_identifier(raw: dict[str, object], key: str, pattern: re.Pattern[str]) -> str | None:
    value = raw.get(key)
    if value is None:
        return None
    if not isinstance(value, str) or not pattern.fullmatch(value):
        raise SpecError(f"'{key}' value '{value}' is not a valid identifier.")
    return value


def require_targets(raw: dict[str, object], key: str) -> tuple[str, ...]:
    value = raw.get(key)
    if not isinstance(value, list) or not value:
        raise SpecError(f"'{key}' must be a non-empty array.")

    targets: list[str] = []
    for target in value:
        if not isinstance(target, str) or target not in VALID_TARGETS:
            raise SpecError(
                f"'{key}' contains invalid target '{target}'. Valid targets: {sorted(VALID_TARGETS)}."
            )
        targets.append(target)

    ensure_unique(targets, f"duplicate target in '{key}'")
    return tuple(targets)


def ensure_unique(values: Iterable[object], label: str) -> None:
    seen: set[object] = set()
    for value in values:
        if value in seen:
            raise SpecError(f"{label}: {value!r}")
        seen.add(value)


def load_spec() -> ProtocolSpec:
    """Load and validate the shared protocol specification."""

    try:
        with SPEC_PATH.open("r", encoding="utf-8") as handle:
            raw_spec = json.load(handle)
    except FileNotFoundError as exc:
        raise SpecError(f"Protocol spec not found: {SPEC_PATH}") from exc
    except json.JSONDecodeError as exc:
        raise SpecError(f"Protocol spec contains invalid JSON: {exc}") from exc

    return ProtocolSpec.from_dict(raw_spec)


def load_golden_payloads() -> GoldenPayloads:
    """Load and validate the shared golden payload examples."""

    try:
        with GOLDEN_PAYLOADS_PATH.open("r", encoding="utf-8") as handle:
            raw_payloads = json.load(handle)
    except FileNotFoundError as exc:
        raise SpecError(f"Golden payload spec not found: {GOLDEN_PAYLOADS_PATH}") from exc
    except json.JSONDecodeError as exc:
        raise SpecError(f"Golden payload spec contains invalid JSON: {exc}") from exc

    return GoldenPayloads.from_dict(raw_payloads)


def write_text_if_changed(path: Path, content: str) -> bool:
    """Write a file only if its content changed.

    Returns `True` when the file was created or updated, `False` when the on-disk
    content already matched the generated output.
    """

    normalized_content = content if content.endswith("\n") else f"{content}\n"
    path.parent.mkdir(parents=True, exist_ok=True)

    if path.exists() and path.read_text(encoding="utf-8") == normalized_content:
        return False

    path.write_text(normalized_content, encoding="utf-8")
    return True


def cpp_comment(text: str) -> str:
    """Escape comment terminators to keep generated headers valid."""

    return text.replace("*/", "* /")


def char_literal(char: str) -> str:
    """Render a single character as a safe C++ character literal."""

    if char == "\n":
        return r"'\n'"
    if char == "\r":
        return r"'\r'"
    if char == "\t":
        return r"'\t'"
    if char == "\\":
        return r"'\\'"
    if char == "'":
        return r"'\''"
    return f"'{char}'"


def json_static_payload(command_value: int) -> str:
    """Return the compact logical JSON string for a static `{ "p": value }` payload."""

    return json.dumps({"p": command_value}, separators=(",", ":"))


def json_payload_literal(payload: str) -> str:
    """Render one JSON byte string as safe C++ character literals."""

    return ", ".join(char_literal(char) for char in payload)


def json_raw_static_payload_literal(command_value: int) -> str:
    """Render the raw JSON transport bytes for a static `{ "p": value }` payload."""

    return json_payload_literal(json_static_payload(command_value))


def json_serial_static_payload_literal(command_value: int) -> str:
    """Render the newline-delimited serial JSON bytes for a static payload."""

    return json_payload_literal(json_static_payload(command_value) + "\n")


def msgpack_static_payload_bytes(command_value: int) -> list[int]:
    """Build the raw MsgPack payload bytes for a static `{ "p": value }` payload."""

    bytes_ = [0x81, 0xA1, 0x70]
    if command_value <= 0x7F:
        bytes_.append(command_value)
    else:
        bytes_.extend([0xCC, command_value])
    return bytes_


def msgpack_static_payload_literal(command_value: int) -> str:
    """Render the raw MsgPack bytes for a static `{ "p": value }` payload."""

    return ", ".join(f"0x{byte:02X}" for byte in msgpack_static_payload_bytes(command_value))


def msgpack_static_payload_size(command_value: int) -> int:
    """Return the raw MsgPack size for a static payload."""

    return len(msgpack_static_payload_bytes(command_value))


def msgpack_serial_frame_bytes(payload_bytes: Sequence[int]) -> list[int]:
    """Wrap one raw MsgPack payload in the framed serial transport bytes.

    The serial transport is `END + escaped(payload) + END`.
    Only the transport layer sees these delimiters and escape markers; the
    logical LSH payload itself stays unchanged.
    """

    framed_bytes = [MSGPACK_FRAME_END]
    for byte in payload_bytes:
        if byte == MSGPACK_FRAME_END:
            framed_bytes.extend((MSGPACK_FRAME_ESCAPE, MSGPACK_FRAME_ESCAPED_END))
        elif byte == MSGPACK_FRAME_ESCAPE:
            framed_bytes.extend((MSGPACK_FRAME_ESCAPE, MSGPACK_FRAME_ESCAPED_ESCAPE))
        else:
            framed_bytes.append(byte)
    framed_bytes.append(MSGPACK_FRAME_END)
    return framed_bytes


def msgpack_serial_static_payload_literal(command_value: int) -> str:
    """Render the final framed serial MsgPack bytes for a static payload."""

    return ", ".join(
        f"0x{byte:02X}" for byte in msgpack_serial_frame_bytes(msgpack_static_payload_bytes(command_value))
    )


def msgpack_serial_static_payload_size(command_value: int) -> int:
    """Return the final serial-framed MsgPack size for a static payload."""

    return len(msgpack_serial_frame_bytes(msgpack_static_payload_bytes(command_value)))


def msgpack_payload_literal(command_value: int) -> str:
    """Render the raw, unframed MsgPack payload bytes for documentation."""

    return ", ".join(f"0x{byte:02X}" for byte in msgpack_static_payload_bytes(command_value))


def markdown_escape(value: str) -> str:
    """Escape Markdown table separators to keep generated tables valid."""

    return value.replace("|", r"\|")


def markdown_code_span(value: str) -> str:
    """Render inline code spans compatibly with older Doxygen Markdown parsers."""

    if "'" in value or "`" in value:
        return f"``{value}``"
    return f"`{value}`"


def lower_camel_case(identifier: str) -> str:
    """Convert an upper-snake identifier to lowerCamelCase."""

    parts = identifier.lower().split("_")
    return parts[0] + "".join(part.capitalize() for part in parts[1:])


def protocol_key_description(key_name: str) -> str:
    """Return a human-readable description for a compact wire key."""

    descriptions = {
        "KEY_PAYLOAD": "Command discriminator.",
        "KEY_PROTOCOL_MAJOR": "Handshake-only protocol major used for wire compatibility checks.",
        "KEY_NAME": "Device name.",
        "KEY_ACTUATORS_ARRAY": "Actuator ID array.",
        "KEY_BUTTONS_ARRAY": "Button ID array.",
        "KEY_CORRELATION_ID": "Click correlation ID.",
        "KEY_ID": "Numeric actuator or button ID.",
        "KEY_STATE": "Actuator state or bitpacked state bytes.",
        "KEY_TYPE": "Click type discriminator.",
    }
    return descriptions.get(key_name, "")


def render_cpp_protocol(
    spec: ProtocolSpec,
    header_guard: str,
    cpp_namespace: str,
    *,
    file_name: str,
) -> str:
    """Render the shared command/key header used by C++ targets."""

    key_lines = "\n".join(
        f'        inline constexpr char {name}[] = "{value}";' for name, value in spec.keys.items()
    )
    command_lines = "\n".join(
        f"            {command.cpp_name} = {command.value}, //!< {cpp_comment(command.description)}"
        for command in spec.commands
    )
    click_lines = "\n".join(
        f"            {click_type.cpp_name} = {click_type.value},"
        for click_type in spec.click_types
    )

    return f"""/**
 * @file    {file_name}
 * @author  Jacopo Labardi (labodj)
 * @brief Defines the communication protocol contract (JSON keys and command IDs).
 * @note Do not edit manually. Run tools/generate_lsh_protocol.py instead.
 *
 * Copyright 2026 Jacopo Labardi
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef {header_guard}
#define {header_guard}

#include <stdint.h>

namespace {cpp_namespace}
{{
    namespace protocol
    {{
        inline constexpr uint32_t SPEC_REVISION = {spec.spec_revision}U; //!< Code-only revision, never transmitted on wire.
        inline constexpr uint8_t WIRE_PROTOCOL_MAJOR = {spec.wire_protocol_major}U; //!< Handshake-only protocol major, transmitted only in DEVICE_DETAILS.

        // === JSON KEYS ===
{key_lines}

        /**
         * @brief Valid command types for the 'p' (payload) key.
         */
        enum class Command : uint8_t
        {{
{command_lines}
        }};

        /**
         * @brief Valid click types for the 't' (type) key.
         */
        enum class ProtocolClickType : uint8_t
        {{
{click_lines}
        }};

    }} // namespace protocol
}} // namespace {cpp_namespace}

#endif // {header_guard}
"""


def render_cpp_static_payloads(
    spec: ProtocolSpec,
    *,
    target: str,
    header_guard: str,
    include_directive: str,
    array_type: str,
    file_name: str,
) -> str:
    """Render a target-specific header with pre-serialized static payload bytes."""

    command_values = {command.name: command.value for command in spec.commands}
    target_payloads = spec.static_payloads_for(target)

    payload_lines = []
    for payload in target_payloads:
        command_value = command_values[payload.command]
        json_raw_bytes = json_raw_static_payload_literal(command_value)
        json_serial_bytes = json_serial_static_payload_literal(command_value)
        msgpack_raw_bytes = msgpack_static_payload_literal(command_value)
        msgpack_serial_bytes = msgpack_serial_static_payload_literal(command_value)
        json_raw_size = len(json_static_payload(command_value))
        json_serial_size = len(json_static_payload(command_value) + "\n")
        msgpack_raw_size = msgpack_static_payload_size(command_value)
        msgpack_serial_size = msgpack_serial_static_payload_size(command_value)

        payload_lines.append(
            f"    // --- {payload.name} ---\n"
            f"    inline constexpr {array_type}<uint8_t, {json_raw_size}> JSON_RAW_{payload.symbol_name}_BYTES = "
            f"{{{json_raw_bytes}}};\n"
            f"    inline constexpr {array_type}<uint8_t, {json_serial_size}> JSON_SERIAL_{payload.symbol_name}_BYTES = "
            f"{{{json_serial_bytes}}};\n"
            f"    inline constexpr {array_type}<uint8_t, {msgpack_raw_size}> MSGPACK_RAW_{payload.symbol_name}_BYTES = "
            f"{{{msgpack_raw_bytes}}};\n"
            f"    inline constexpr {array_type}<uint8_t, {msgpack_serial_size}> MSGPACK_SERIAL_{payload.symbol_name}_BYTES = "
            f"{{{msgpack_serial_bytes}}};"
        )

    enum_lines = "\n".join(f"        {payload.cpp_name}," for payload in target_payloads)
    joined_payloads = "\n\n".join(payload_lines)

    return f"""/**
 * @file    {file_name}
 * @author  Jacopo Labardi (labodj)
 * @brief Defines target-specific pre-serialized static payload bytes for raw and serial transports.
 * @note Do not edit manually. Run tools/generate_lsh_protocol.py instead.
 *
 * Copyright 2026 Jacopo Labardi
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef {header_guard}
#define {header_guard}

#include <stdint.h>
{include_directive}

namespace constants::payloads
{{
    enum class StaticType : uint8_t
    {{
{enum_lines}
    }};

{joined_payloads}

}} // namespace constants::payloads

#endif // {header_guard}
"""


def render_ts_protocol(spec: ProtocolSpec) -> str:
    """Render the TypeScript protocol module."""

    key_lines = "\n".join(
        f'  {name.removeprefix("KEY_")}: "{value}",' for name, value in spec.keys.items()
    )

    command_lines: list[str] = []
    for command in spec.commands:
        command_lines.append(f"  {command.ts_name} = {command.value},")
        for alias, target_name in spec.ts_aliases.items():
            if target_name == command.name:
                command_lines.append(f"  {alias} = {command.value},")

    click_lines = "\n".join(
        f"  {click_type.ts_name} = {click_type.value}," for click_type in spec.click_types
    )

    return f"""/**
 * Auto-generated from shared/lsh_protocol.json.
 * Do not edit manually. Run tools/generate_lsh_protocol.py instead.
 */

export const LSH_PROTOCOL_SPEC_REVISION = {spec.spec_revision} as const;
export const LSH_WIRE_PROTOCOL_MAJOR = {spec.wire_protocol_major} as const;

export const LSH_PROTOCOL_KEYS = {{
{key_lines}
}} as const;

export enum ClickType {{
{click_lines}
}}

export enum LshProtocol {{
{chr(10).join(command_lines)}
}}
"""


def render_protocol_markdown(spec: ProtocolSpec, golden_payloads: GoldenPayloads) -> str:
    """Render a human-readable Markdown protocol reference."""

    key_rows = "\n".join(
        "| "
        + " | ".join(
            (
                markdown_code_span(name),
                markdown_code_span(value),
                markdown_escape(protocol_key_description(name)),
            )
        )
        + " |"
        for name, value in spec.keys.items()
    )

    command_rows: list[str] = []
    for command in spec.commands:
        example = golden_payloads.payloads.get(lower_camel_case(command.name))
        json_example = (
            markdown_code_span(json.dumps(example, separators=(",", ":")))
            if example is not None
            else ""
        )
        command_rows.append(
            "| "
            + " | ".join(
                (
                    str(command.value),
                    markdown_code_span(command.cpp_name),
                    markdown_code_span(command.ts_name),
                    json_example,
                    markdown_escape(command.description),
                )
            )
            + " |"
        )

    click_rows = "\n".join(
        "| "
        + " | ".join(
            (
                str(click_type.value),
                markdown_code_span(click_type.cpp_name),
                markdown_code_span(click_type.ts_name),
            )
        )
        + " |"
        for click_type in spec.click_types
    )

    static_payload_rows = "\n".join(
        "| "
        + " | ".join(
            (
                markdown_code_span(payload.name),
                markdown_code_span(payload.command),
                markdown_code_span(payload.cpp_name),
                markdown_code_span(payload.symbol_name),
                ", ".join(markdown_code_span(target) for target in payload.targets),
                markdown_code_span(
                    json_raw_static_payload_literal(spec.command_by_name()[payload.command].value)
                ),
                markdown_code_span(
                    json_serial_static_payload_literal(spec.command_by_name()[payload.command].value)
                ),
                markdown_code_span(
                    msgpack_payload_literal(spec.command_by_name()[payload.command].value)
                ),
                markdown_code_span(
                    msgpack_serial_static_payload_literal(spec.command_by_name()[payload.command].value)
                ),
            )
        )
        + " |"
        for payload in spec.static_payloads
    )

    trust_lines = ""
    if spec.documentation.trusted_environment:
        trust_lines = (
            "## Trusted Environment\n\n"
            f"{spec.documentation.trusted_environment}\n\n"
        )

    handshake_lines = ""
    if spec.documentation.handshake:
        handshake_lines = "## Handshake Contract\n\n" + "\n".join(
            f"- {line}" for line in spec.documentation.handshake
        ) + "\n\n"

    compatibility_lines = ""
    if spec.documentation.compatibility:
        compatibility_lines = "## Compatibility Contract\n\n" + "\n".join(
            f"- {line}" for line in spec.documentation.compatibility
        ) + "\n\n"

    transport_lines = ""
    if spec.documentation.transport:
        transport_lines = "## Transport Encoding\n\n" + "\n".join(
            f"- {line}" for line in spec.documentation.transport
        ) + "\n\n"

    constraints_lines = ""
    if spec.documentation.constraints:
        constraints_lines = "## Wire Constraints\n\n" + "\n".join(
            f"- {line}" for line in spec.documentation.constraints
        ) + "\n\n"

    return f"""# {spec.name}

This document is auto-generated from `shared/lsh_protocol.json` by `tools/generate_lsh_protocol.py`.
Do not edit it manually.

- Spec revision: `{spec.spec_revision}`
- Wire protocol major: `{spec.wire_protocol_major}`
- Revision note: {spec.notes}
- Wire goal: compact payloads with single-character keys and numeric command IDs

{trust_lines}{handshake_lines}{compatibility_lines}{transport_lines}{constraints_lines}## JSON Keys

| Constant | Wire Key | Meaning |
| --- | --- | --- |
{key_rows}

## Commands

| Value | C++ | TypeScript | Golden JSON Example | Description |
| --- | --- | --- | --- | --- |
{chr(10).join(command_rows)}

## Click Types

| Value | C++ | TypeScript |
| --- | --- | --- |
{click_rows}

## Pre-serialized Static Payloads

These payloads are generated as compile-time byte arrays for zero-allocation hot paths.
Each row shows both the logical raw payload bytes and the final serial transport bytes.
The raw forms are used by transports that carry bare payloads, while the serial forms
are already encoded exactly as they should appear on the controller link.

| Name | Command | C++ Enum | C++ Symbol | Targets | JSON Raw Bytes | JSON Serial Bytes | MsgPack Raw Bytes | MsgPack Serial Bytes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
{static_payload_rows}
"""


def describe_path(path: Path) -> str:
    """Return a readable path for logs, preferring relative-to-repo output when possible."""

    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def generated_outputs(
    spec: ProtocolSpec,
    golden_payloads: GoldenPayloads,
    *,
    selected_targets: Sequence[str],
    shared_doc_root: Path | None,
    core_root: Path | None,
    bridge_root: Path | None,
    node_red_root: Path | None,
) -> list[tuple[Path, str]]:
    """Return the generated files requested by the selected targets."""

    outputs: list[tuple[Path, str]] = []

    for target in selected_targets:
        if target == CLI_TARGET_SHARED_DOC:
            root = shared_doc_root or ROOT
            outputs.append(
                (
                    root / "shared" / "lsh_protocol.md",
                    render_protocol_markdown(spec, golden_payloads),
                )
            )
            continue

        if target == TARGET_CORE:
            if core_root is None:
                raise SpecError("--core-root is required when target 'core' is selected.")
            outputs.extend(
                [
                    (
                        core_root / "src" / "communication" / "constants" / "protocol.hpp",
                        render_cpp_protocol(
                            spec,
                            "LSH_CORE_COMMUNICATION_CONSTANTS_PROTOCOL_HPP",
                            "lsh::core",
                            file_name="protocol.hpp",
                        ),
                    ),
                    (
                        core_root / "src" / "communication" / "constants" / "static_payloads.hpp",
                        render_cpp_static_payloads(
                            spec,
                            target=TARGET_CORE,
                            header_guard="LSH_CORE_COMMUNICATION_CONSTANTS_STATIC_PAYLOADS_HPP",
                            include_directive='#include "../../internal/etl_array.hpp"',
                            array_type="etl::array",
                            file_name="static_payloads.hpp",
                        ),
                    ),
                ]
            )
            continue

        if target == TARGET_BRIDGE:
            if bridge_root is None:
                raise SpecError("--bridge-root is required when target 'bridge' is selected.")
            outputs.extend(
                [
                    (
                        bridge_root / "src" / "constants" / "communication_protocol.hpp",
                        render_cpp_protocol(
                            spec,
                            "LSH_BRIDGE_CONSTANTS_COMMUNICATION_PROTOCOL_HPP",
                            "lsh::bridge",
                            file_name="communication_protocol.hpp",
                        ),
                    ),
                    (
                        bridge_root / "src" / "constants" / "payloads.hpp",
                        render_cpp_static_payloads(
                            spec,
                            target=TARGET_BRIDGE,
                            header_guard="LSH_BRIDGE_CONSTANTS_PAYLOADS_HPP",
                            include_directive="#include <array>",
                            array_type="std::array",
                            file_name="payloads.hpp",
                        ),
                    ),
                ]
            )
            continue

        if target == CLI_TARGET_NODE_RED:
            if node_red_root is None:
                raise SpecError("--node-red-root is required when target 'node-red' is selected.")
            outputs.append(
                (
                    node_red_root / "src" / "generated" / "protocol.ts",
                    render_ts_protocol(spec),
                )
            )
            continue

        raise SpecError(f"Unknown target '{target}'. Valid targets: {', '.join(VALID_CLI_TARGETS)}.")

    return outputs


def normalize_cli_targets(selected_targets: Sequence[str]) -> tuple[str, ...]:
    """Remove duplicate CLI targets while preserving the request order."""

    normalized: list[str] = []
    seen: set[str] = set()

    for target in selected_targets:
        if target not in seen:
            normalized.append(target)
            seen.add(target)

    return tuple(normalized)


def run(
    *,
    check_only: bool,
    selected_targets: Sequence[str],
    shared_doc_root: Path | None,
    core_root: Path | None,
    bridge_root: Path | None,
    node_red_root: Path | None,
) -> int:
    """Generate the files or verify that generated files are already up to date."""

    spec = load_spec()
    golden_payloads = load_golden_payloads()
    stale_files: list[Path] = []
    updated_files: list[Path] = []

    for path, content in generated_outputs(
        spec,
        golden_payloads,
        selected_targets=selected_targets,
        shared_doc_root=shared_doc_root,
        core_root=core_root,
        bridge_root=bridge_root,
        node_red_root=node_red_root,
    ):
        if check_only:
            current = path.read_text(encoding="utf-8") if path.exists() else None
            normalized_content = content if content.endswith("\n") else f"{content}\n"
            if current != normalized_content:
                stale_files.append(path)
            continue

        if write_text_if_changed(path, content):
            updated_files.append(path)

    if check_only:
        if stale_files:
            for path in stale_files:
                print(f"stale generated file: {describe_path(path)}", file=sys.stderr)
            return 1
        return 0

    for path in updated_files:
        print(f"updated {describe_path(path)}")
    if not updated_files:
        print("generated files already up to date")
    return 0


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    """Parse command-line arguments."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target",
        action="append",
        choices=VALID_CLI_TARGETS,
        help="generation target to emit",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if any generated file is out of date instead of rewriting it",
    )
    parser.add_argument(
        "--shared-doc-root",
        type=Path,
        help="root directory that contains the target shared/ folder for lsh_protocol.md",
    )
    parser.add_argument(
        "--core-root",
        type=Path,
        help="root directory of the lsh-core repository",
    )
    parser.add_argument(
        "--bridge-root",
        dest="bridge_root",
        type=Path,
        help="root directory of the lsh-bridge repository",
    )
    parser.add_argument(
        "--node-red-root",
        type=Path,
        help="root directory of the node-red-contrib-lsh-logic repository",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Program entry point."""

    args = parse_args(argv or sys.argv[1:])
    selected_targets = normalize_cli_targets(tuple(args.target or [CLI_TARGET_SHARED_DOC]))
    try:
        return run(
            check_only=args.check,
            selected_targets=selected_targets,
            shared_doc_root=args.shared_doc_root.resolve() if args.shared_doc_root else None,
            core_root=args.core_root.resolve() if args.core_root else None,
            bridge_root=args.bridge_root.resolve() if args.bridge_root else None,
            node_red_root=args.node_red_root.resolve() if args.node_red_root else None,
        )
    except SpecError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
