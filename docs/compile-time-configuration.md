# Compile-time Configuration

`lsh-bridge` keeps a deliberately small but important compile-time surface. All supported knobs are ordinary C/C++ preprocessor macros passed through PlatformIO `build_flags`.

The bundled example in [`examples/basic-homie-bridge/platformio.ini`](../examples/basic-homie-bridge/platformio.ini) sets all value-like macros explicitly and leaves the behavior-changing flags commented.

If you are new to the public stack, read these first:

- [Labo Smart Home landing page](https://github.com/labodj/labo-smart-home)
- [LSH reference stack](https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md)
- [LSH getting started guide](https://github.com/labodj/labo-smart-home/blob/main/GETTING_STARTED.md)
- [LSH troubleshooting guide](https://github.com/labodj/labo-smart-home/blob/main/TROUBLESHOOTING.md)

Use this page when you need exact macro ownership, not the runtime story. For
runtime behavior, diagnostics and startup semantics, read
[runtime-behavior.md](./runtime-behavior.md).

## Safe First-Lab Defaults

For a first bring-up, the least painful strategy is:

- start from the bundled example unchanged
- keep the stock topic layout and service topic
- keep the stock codec choices until the end-to-end path works once
- make capacities large enough for the controller you actually flashed
- match serial baud exactly with `lsh-core`

Most "mysterious" first-lab problems are not mysterious at all. They come from
changing more than one of those at the same time.

## Capacity and validation

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_MAX_ACTUATORS` | `12U` | Maximum actuators accepted from the controller. Sizes static ETL containers, packed state buffers and JSON documents. |
| `CONFIG_MAX_BUTTONS` | `12U` | Maximum buttons/clickables accepted from the controller. Sizes validation and JSON buffers. |
| `CONFIG_MAX_NAME_LENGTH` | `4U` | Maximum device name length accepted from the controller. Also contributes to MQTT topic buffer sizing. |

These limits are not cosmetic. If the controller reports more actuators/buttons than compiled, or a device name longer than `CONFIG_MAX_NAME_LENGTH`, the bridge rejects the payload.

## Serial bridge settings

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_ARDCOM_SERIAL_RX_PIN` | `16U` | RX pin used by `ControllerSerialLink::begin()`. |
| `CONFIG_ARDCOM_SERIAL_TX_PIN` | `17U` | TX pin used by `ControllerSerialLink::begin()`. |
| `CONFIG_ARDCOM_SERIAL_BAUD` | `250000U` | UART baud rate between ESP and controller. |
| `CONFIG_ARDCOM_SERIAL_TIMEOUT_MS` | `5U` | Compatibility fallback used as the default value for `CONFIG_ARDCOM_SERIAL_MSGPACK_FRAME_IDLE_TIMEOUT_MS`. |
| `CONFIG_ARDCOM_SERIAL_MSGPACK_FRAME_IDLE_TIMEOUT_MS` | `CONFIG_ARDCOM_SERIAL_TIMEOUT_MS` | Maximum silence while a serial MsgPack frame is still incomplete. Only used when serial uses MessagePack. |
| `CONFIG_ARDCOM_SERIAL_MAX_RX_BYTES_PER_LOOP` | `SERIAL_RX_BUFFER_SIZE` in JSON mode, `SERIAL_RX_MAX_FRAMED_MESSAGE_SIZE` in MsgPack mode | Maximum raw UART bytes consumed by one `ControllerSerialLink::processSerialBuffer()` call before the bridge loop returns to other work. |

Internally the bridge keeps two different serial-related size classes:

- `SERIAL_RX_BUFFER_SIZE`: sized for one complete inbound controller payload
- `SERIAL_MAX_RX_BYTES_PER_LOOP`: fairness budget for one bridge loop iteration, sized to one worst-case escaped serial frame by default in MsgPack mode

These are derived automatically from the controller topology limits above and are not public `CONFIG_*` knobs.

## MQTT topic layout

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_MQTT_TOPIC_BASE` | `"LSH"` | Root prefix for device-scoped topics. |
| `CONFIG_MQTT_TOPIC_INPUT` | `"IN"` | Topic suffix subscribed for device-specific inbound commands. |
| `CONFIG_MQTT_TOPIC_STATE` | `"state"` | Topic suffix used for authoritative state publishes. |
| `CONFIG_MQTT_TOPIC_CONF` | `"conf"` | Topic suffix used for device configuration publishes. |
| `CONFIG_MQTT_TOPIC_EVENTS` | `"events"` | Topic suffix used for controller-backed runtime events such as click traffic and device-level `PING` replies. |
| `CONFIG_MQTT_TOPIC_BRIDGE` | `"bridge"` | Topic suffix used for bridge-local runtime events such as service-level ping replies and diagnostics. |
| `CONFIG_MQTT_TOPIC_SERVICE` | `"LSH/Node-RED/SRV"` | Broadcast/service topic subscribed independently from the device name. |

The bridge derives topic buffer sizes automatically from the compiled strings above plus `CONFIG_MAX_NAME_LENGTH`. There are no public `CONFIG_MQTT_*_LENGTH` knobs in the current library.

## Homie convention and identity

| Macro | Default | What it affects |
| --- | --- | --- |
| `HOMIE_CONVENTION_VERSION` | required `5` | Selects the Homie MQTT convention compiled by the `labodj/homie-v5` PlatformIO package. `lsh-bridge` now requires v5 so discovery is published under `homie/5/<device>/$description`. |
| `CONFIG_HOMIE_FIRMWARE_NAME` | `"lsh-homie"` | Firmware name exposed through Homie. |
| `CONFIG_HOMIE_FIRMWARE_VERSION` | `"1.3.0"` | Firmware version exposed through Homie. |
| `CONFIG_HOMIE_BRAND` | `"LaboSmartHome"` | Homie brand string exposed by the bridge. |

`HOMIE_CONVENTION_VERSION=5` must be passed by the embedding PlatformIO
environment, not hidden inside one bridge source file, because the
`labodj/homie-v5` dependency compiles its own translation units and must see
the same convention selector. The bridge has a compile-time guard that rejects
missing or legacy values instead of silently publishing Homie v3/v4 discovery.

The identity macros must expand to string literals. The Homie dependency builds
its internal flagged string format through macro concatenation, so firmware
identity is intentionally compile-time only in the current bridge API.

## Liveness timers

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_PING_INTERVAL_CONTROLLINO_MS` | `10000U` | Minimum spacing between `PING_` frames sent to the controller. |
| `CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS` | `CONFIG_PING_INTERVAL_CONTROLLINO_MS + 200U` | How long the controller may stay silent before the bridge marks it disconnected. |

## Runtime policy knobs

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS` | `500U` | Delay between repeated `REQUEST_DETAILS` / `REQUEST_STATE` requests during bootstrap and controller resync. |
| `CONFIG_TOPOLOGY_SAVE_RETRY_INTERVAL_MS` | `CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS` | Delay between repeated NVS save attempts while a topology migration is pending. |
| `CONFIG_TOPOLOGY_REBOOT_GRACE_MS` | `CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS` | Hard grace window before rebooting after a successful topology save, even if MQTT stays degraded. |
| `CONFIG_STATE_PUBLISH_SETTLE_INTERVAL_MS` | `40U` | Quiet window before a freshly received authoritative state is mirrored back out to MQTT and Homie. |
| `CONFIG_MQTT_COMMAND_QUEUE_CAPACITY` | `8U` | Number of complete inbound MQTT frames the bridge may buffer while the serial side is busy. Queue RAM cost is `CONFIG_MQTT_COMMAND_QUEUE_CAPACITY * MQTT_COMMAND_MESSAGE_MAX_SIZE`, plus small queue metadata. Valid range: `1..255`. |
| `CONFIG_MQTT_MAX_COMMANDS_PER_LOOP` | `8U` | Maximum number of queued MQTT commands the main loop drains in one iteration while the UART is idle. This is a CPU/fairness knob, not a queue-capacity knob. Valid range: `1..255`. |
| `CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS` | `50U` | Quiet window used to coalesce multiple actuator writes into one outbound `SET_STATE`. |
| `CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS` | `1000U` | Hard limit for how long an unstable pending actuator batch may stay open before the bridge drops it. |
| `CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT` | `32U` | Maximum number of accepted state changes merged into one pending actuator batch before the bridge treats the producer as unstable. |

## Implementation storage

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_LSH_BRIDGE_IMPL_STORAGE_SIZE` | `3072U` | Static byte storage reserved inside the public `LSHBridge` facade for the hidden runtime implementation. This avoids heap allocation while keeping the public header small. Normally leave it unset; if an unusual capacity profile makes the hidden implementation larger than the default, compilation fails with a clear `static_assert` and the embedding project can raise this value. |

Internally the MQTT side also keeps two different size classes:

- `MQTT_COMMAND_MESSAGE_MAX_SIZE`: queue slot size for inbound MQTT commands only
- `MQTT_PUBLISH_MESSAGE_MAX_SIZE`: temporary publish buffer size for outbound MQTT payloads, including retained `conf` topology publishes

Only the outbound publish buffer still scales with worst-case `DEVICE_DETAILS`, because topology snapshots are really published by the bridge. The inbound command queue no longer pays that topology worst-case cost.

## Built-in persistent topology cache

`lsh-bridge` persists the last validated controller `DEVICE_DETAILS`
snapshot in ESP32 NVS. This behavior is intentionally built in and does not
currently expose public `CONFIG_*` knobs.

Important notes:

- only topology is persisted: device name, actuator IDs and button IDs
- the stored record is an explicit byte format with magic/version/checksum, not
  a raw C++ struct dump, so it is not coupled to compiler padding or field
  layout
- older or invalid cache records are ignored and rebuilt from the controller
- runtime actuator `STATE` is never written to flash
- the cache is written only after a real topology change has been confirmed by
  the controller
- the bridge treats the controller as authoritative on every boot

The main compile-time knobs that constrain this cache are the
capacity/validation limits above: `CONFIG_MAX_ACTUATORS`,
`CONFIG_MAX_BUTTONS` and `CONFIG_MAX_NAME_LENGTH`.

## Codec and behavior flags

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_MSG_PACK_ARDUINO` | undefined | Serial side uses MessagePack wrapped in the bridge serial framing transport instead of newline-delimited JSON. |
| `CONFIG_MSG_PACK_MQTT` | undefined | MQTT side uses MessagePack instead of JSON text. |
| `LSH_DEBUG` | undefined | Enables debug logging. With `BridgeOptions::loggingMode = AutoFromBuild`, logs are on only when this flag is defined. |
| `HOMIE_RESET` | undefined | During `bridge.begin()`, erases the stored Homie configuration and reboots into setup/AP mode only when a valid Homie config is currently present. New or already-reset devices enter setup mode automatically when no valid configuration is present. |

If `CONFIG_MSG_PACK_ARDUINO` and `CONFIG_MSG_PACK_MQTT` are both defined or both undefined, the bridge can forward some inbound command payloads without re-encoding. If only one side uses MessagePack, the bridge deserializes and reserializes between the two formats.

Serial MsgPack framing affects only the controller link. It does not change MQTT payloads and it does not change the logical LSH payload format. It only wraps serial MsgPack payloads in a delimiter-and-escape transport frame so the receiver can deframe them without using stream timeouts as implicit framing.

## ETL profile override

`lsh-bridge` ships with a default
[`include/etl_profile.h`](../include/etl_profile.h) so the standard
Arduino/PlatformIO case works without extra setup.

That default profile intentionally keeps ETL compiler/platform detection on the
official auto-detect path and only applies the bridge's default ETL policy
flags.

If an embedding project needs different ETL behavior for another target or
toolchain, the
recommended override path is:

1. Create a small project-local override header, for example `include/lsh_etl_profile_override.h`
2. Pass `-D LSH_ETL_PROFILE_OVERRIDE_HEADER=\"lsh_etl_profile_override.h\"`
3. In that header, `#undef` and redefine only the ETL macros that must change

Example:

```cpp
// include/lsh_etl_profile_override.h
#pragma once

#undef ETL_VERBOSE_ERRORS
#define ETL_THROW_EXCEPTIONS
```

If your build system prefers full ownership of the ETL profile, you may also
provide your own project-level `etl_profile.h` earlier in the include path and
bypass the one shipped by `lsh-bridge`.

The bundled example already demonstrates this hook through
[`examples/basic-homie-bridge/include/lsh_etl_profile_override.h`](../examples/basic-homie-bridge/include/lsh_etl_profile_override.h)
and the matching `LSH_ETL_PROFILE_OVERRIDE_HEADER` build flag in the example
`platformio.ini`.

## Runtime behaviors that are not `CONFIG_*`

Some important bridge behaviors are runtime-managed even after the knobs
above and are not exposed as public compile-time values:

- bridge-local diagnostics published on the MQTT `bridge` topic
- the exact diagnostic payload shapes
- counter aggregation and reset policy for queue-overflow and command-rejection diagnostics
- the NVS-backed `DEVICE_DETAILS` cache and its on-change write policy

Those behaviors are documented in
[`docs/runtime-behavior.md`](./runtime-behavior.md).

## Read Next

- For runtime policy and diagnostics: [runtime-behavior.md](./runtime-behavior.md)
- For the bridge overview and example entry point: [../README.md](../README.md)
- For the public stack profile: <https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md>
- For first-lab symptom diagnosis: <https://github.com/labodj/labo-smart-home/blob/main/TROUBLESHOOTING.md>

## PlatformIO example

```ini
build_flags =
    -I include
    -D LSH_ETL_PROFILE_OVERRIDE_HEADER=\"lsh_etl_profile_override.h\"
    -D CONFIG_MAX_ACTUATORS=12U
    -D CONFIG_MAX_BUTTONS=12U
    -D CONFIG_MAX_NAME_LENGTH=4U
    -D CONFIG_ARDCOM_SERIAL_RX_PIN=16U
    -D CONFIG_ARDCOM_SERIAL_TX_PIN=17U
    -D CONFIG_ARDCOM_SERIAL_BAUD=250000U
    -D CONFIG_ARDCOM_SERIAL_TIMEOUT_MS=5U
    ; -D CONFIG_ARDCOM_SERIAL_MSGPACK_FRAME_IDLE_TIMEOUT_MS=5U
    ; default: CONFIG_ARDCOM_SERIAL_TIMEOUT_MS
    ; -D CONFIG_ARDCOM_SERIAL_MAX_RX_BYTES_PER_LOOP=32U
    ; default: SERIAL_RX_BUFFER_SIZE in JSON mode, SERIAL_RX_MAX_FRAMED_MESSAGE_SIZE in MsgPack mode
    -D CONFIG_MQTT_TOPIC_BASE=\"LSH\"
    -D CONFIG_MQTT_TOPIC_INPUT=\"IN\"
    -D CONFIG_MQTT_TOPIC_STATE=\"state\"
    -D CONFIG_MQTT_TOPIC_CONF=\"conf\"
    -D CONFIG_MQTT_TOPIC_EVENTS=\"events\"
    -D CONFIG_MQTT_TOPIC_BRIDGE=\"bridge\"
    -D CONFIG_MQTT_TOPIC_SERVICE=\"LSH/Node-RED/SRV\"
    -D HOMIE_CONVENTION_VERSION=5
    -D CONFIG_HOMIE_FIRMWARE_NAME=\"lsh-homie\"
    -D CONFIG_HOMIE_FIRMWARE_VERSION=\"1.3.0\"
    -D CONFIG_HOMIE_BRAND=\"LaboSmartHome\"
    -D CONFIG_PING_INTERVAL_CONTROLLINO_MS=10000U
    -D CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS=10200U
    ; -D CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS=500U
    ; -D CONFIG_TOPOLOGY_SAVE_RETRY_INTERVAL_MS=500U
    ; default: CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS
    ; -D CONFIG_TOPOLOGY_REBOOT_GRACE_MS=500U
    ; default: CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS
    ; -D CONFIG_STATE_PUBLISH_SETTLE_INTERVAL_MS=40U
    ; -D CONFIG_MQTT_COMMAND_QUEUE_CAPACITY=8U
    ; -D CONFIG_MQTT_MAX_COMMANDS_PER_LOOP=8U
    ; -D CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS=50U
    ; -D CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS=1000U
    ; -D CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT=32U
    ; -D CONFIG_LSH_BRIDGE_IMPL_STORAGE_SIZE=3072U
    -D CONFIG_MSG_PACK_ARDUINO
    ; default: undefined
    ; -D CONFIG_MSG_PACK_MQTT
    ; default: undefined
    ; -D LSH_DEBUG
    ; default: undefined
    ; -D HOMIE_RESET
    ; default: undefined
```
