# LSH Protocol

This document is auto-generated from `shared/lsh_protocol.json` by `tools/generate_lsh_protocol.py`.
Do not edit it manually.

- Spec revision: `2026042001`
- Wire protocol major: `3`
- Revision note: Code-only revision. Never transmitted on wire.
- Wire goal: compact payloads with single-character keys and numeric command IDs

## Trusted Environment

The protocol assumes a trusted environment and a cooperative broker. There is no built-in authentication, integrity or confidentiality layer on the LSH payloads.

## Handshake Contract

- `BOOT` is a re-sync trigger only. It never carries compatibility metadata.
- A peer that receives `BOOT` must discard runtime assumptions derived from the sender and re-synchronize according to the active transport/profile.
- After `BOOT`, the authoritative topology snapshot is re-established through `DEVICE_DETAILS`, and the authoritative runtime state snapshot is re-established through `ACTUATORS_STATE`.
- After `BOOT`, wire compatibility is checked when the authoritative peer sends `DEVICE_DETAILS` with `v = wireProtocolMajor`.
- `DEVICE_DETAILS` is the authoritative topology snapshot for the current session.
- Topology is treated as static between two authoritative topology announcements. Runtime hot topology changes are intentionally unsupported.

## Compatibility Contract

- `DEVICE_DETAILS.v` is the only on-wire compatibility field and is transmitted only during the handshake.
- Consumers must reject `DEVICE_DETAILS` when `v` does not match their locally compiled `WIRE_PROTOCOL_MAJOR`.
- If `DEVICE_DETAILS.v` matches, the handshake may continue normally.
- `specRevision` is documentation and generated-code metadata only. It is never transmitted on wire and must not be used for runtime compatibility decisions.

## Transport Encoding

- The logical LSH payload format is transport-agnostic.
- JSON over serial is newline-delimited.
- MsgPack over serial uses `END + escaped(payload) + END`, with `END = 0xC0`, `ESC = 0xDB`, `ESC_END = 0xDC` and `ESC_ESC = 0xDD`.
- MQTT carries raw JSON strings or raw MsgPack payload bytes.
- `PING` is hop-local by default: it probes reachability of the immediate peer on the current transport unless a higher-level profile defines a stronger meaning.
- `BOOT` is role-local by default: it tells the receiving peer to discard runtime assumptions and re-synchronize. Whether it is forwarded across multiple hops is profile-specific, not part of the base wire contract.

## Wire Constraints

- `i`, `c`, actuator IDs and button IDs are positive 8-bit values. `0` is reserved as a sentinel for missing or invalid fields and must not be used on the wire.
- `SET_SINGLE_ACTUATOR.s` accepts only `0` or `1` on the wire.
- Bridge builds may impose a tighter maximum on `n` via `CONFIG_MAX_NAME_LENGTH` (default `4`). Device names must fit the compiled bridge limit.

## JSON Keys

| Constant | Wire Key | Meaning |
| --- | --- | --- |
| `KEY_PAYLOAD` | `p` | Command discriminator. |
| `KEY_PROTOCOL_MAJOR` | `v` | Handshake-only protocol major used for wire compatibility checks. |
| `KEY_NAME` | `n` | Device name. |
| `KEY_ACTUATORS_ARRAY` | `a` | Actuator ID array. |
| `KEY_BUTTONS_ARRAY` | `b` | Button ID array. |
| `KEY_CORRELATION_ID` | `c` | Click correlation ID. |
| `KEY_ID` | `i` | Numeric actuator or button ID. |
| `KEY_STATE` | `s` | Actuator state or bitpacked state bytes. |
| `KEY_TYPE` | `t` | Click type discriminator. |

## Commands

| Value | C++ | TypeScript | Golden JSON Example | Description |
| --- | --- | --- | --- | --- |
| 1 | `DEVICE_DETAILS` | `DEVICE_DETAILS` | `{"p":1,"v":3,"n":"c1","a":[1,5],"b":[7]}` | Device details payload with handshake-only protocol major used for wire compatibility checks. |
| 2 | `ACTUATORS_STATE` | `ACTUATORS_STATE` | `{"p":2,"s":[90,3]}` | Bitpacked actuator state payload. |
| 3 | `NETWORK_CLICK_REQUEST` | `NETWORK_CLICK_REQUEST` | `{"p":3,"c":42,"i":7,"t":1}` | Network click request with correlation ID. |
| 4 | `BOOT` | `BOOT` | `{"p":4}` | Controller boot notification and re-sync trigger. Does not carry version metadata. |
| 5 | `PING_` | `PING` | `{"p":5}` | Ping or heartbeat payload. |
| 10 | `REQUEST_DETAILS` | `REQUEST_DETAILS` | `{"p":10}` | Request device details. |
| 11 | `REQUEST_STATE` | `REQUEST_STATE` | `{"p":11}` | Request current state. |
| 12 | `SET_STATE` | `SET_STATE` | `{"p":12,"s":[90,3]}` | Set all actuators. |
| 13 | `SET_SINGLE_ACTUATOR` | `SET_SINGLE_ACTUATOR` | `{"p":13,"i":5,"s":1}` | Set a single actuator. |
| 14 | `NETWORK_CLICK_ACK` | `NETWORK_CLICK_ACK` | `{"p":14,"c":42,"i":7,"t":1}` | Acknowledge a network click with correlation ID. |
| 15 | `FAILOVER` | `FAILOVER` | `{"p":15}` | General failover signal. |
| 16 | `FAILOVER_CLICK` | `FAILOVER_CLICK` | `{"p":16,"c":42,"i":7,"t":2}` | Failover for a specific click with correlation ID. |
| 17 | `NETWORK_CLICK_CONFIRM` | `NETWORK_CLICK_CONFIRM` | `{"p":17,"c":42,"i":7,"t":1}` | Confirm a network click after ACK using the same correlation ID. |
| 254 | `SYSTEM_REBOOT` | `SYSTEM_REBOOT` | `{"p":254}` | Bridge system reboot command. |
| 255 | `SYSTEM_RESET` | `SYSTEM_RESET` | `{"p":255}` | Bridge system reset command. |

## Click Types

| Value | C++ | TypeScript |
| --- | --- | --- |
| 1 | `LONG` | `Long` |
| 2 | `SUPER_LONG` | `SuperLong` |

## Pre-serialized Static Payloads

These payloads are generated as compile-time byte arrays for zero-allocation hot paths.
Each row shows both the logical raw payload bytes and the final serial transport bytes.
The raw forms are used by transports that carry bare payloads, while the serial forms
are already encoded exactly as they should appear on the controller link.

| Name | Command | C++ Enum | C++ Symbol | Targets | JSON Raw Bytes | JSON Serial Bytes | MsgPack Raw Bytes | MsgPack Serial Bytes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `BOOT` | `BOOT` | `BOOT` | `BOOT` | `core`, `bridge` | ``'{', '"', 'p', '"', ':', '4', '}'`` | ``'{', '"', 'p', '"', ':', '4', '}', '\n'`` | `0x81, 0xA1, 0x70, 0x04` | `0xC0, 0x81, 0xA1, 0x70, 0x04, 0xC0` |
| `PING` | `PING` | `PING_` | `PING` | `core`, `bridge` | ``'{', '"', 'p', '"', ':', '5', '}'`` | ``'{', '"', 'p', '"', ':', '5', '}', '\n'`` | `0x81, 0xA1, 0x70, 0x05` | `0xC0, 0x81, 0xA1, 0x70, 0x05, 0xC0` |
| `ASK_DETAILS` | `REQUEST_DETAILS` | `ASK_DETAILS` | `ASK_DETAILS` | `bridge` | ``'{', '"', 'p', '"', ':', '1', '0', '}'`` | ``'{', '"', 'p', '"', ':', '1', '0', '}', '\n'`` | `0x81, 0xA1, 0x70, 0x0A` | `0xC0, 0x81, 0xA1, 0x70, 0x0A, 0xC0` |
| `ASK_STATE` | `REQUEST_STATE` | `ASK_STATE` | `ASK_STATE` | `bridge` | ``'{', '"', 'p', '"', ':', '1', '1', '}'`` | ``'{', '"', 'p', '"', ':', '1', '1', '}', '\n'`` | `0x81, 0xA1, 0x70, 0x0B` | `0xC0, 0x81, 0xA1, 0x70, 0x0B, 0xC0` |
| `GENERAL_FAILOVER` | `FAILOVER` | `GENERAL_FAILOVER` | `GENERAL_FAILOVER` | `bridge` | ``'{', '"', 'p', '"', ':', '1', '5', '}'`` | ``'{', '"', 'p', '"', ':', '1', '5', '}', '\n'`` | `0x81, 0xA1, 0x70, 0x0F` | `0xC0, 0x81, 0xA1, 0x70, 0x0F, 0xC0` |
