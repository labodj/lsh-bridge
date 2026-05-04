# Compile-time Configuration

`lsh-bridge` is built for small ESP32 firmware images, fixed buffers and predictable
runtime behavior. Because of that, capacities, topic names, serial settings, codecs and
Homie identity are compile-time choices passed through PlatformIO `build_flags`.

Start from the bundled
[basic Homie bridge example](https://github.com/labodj/lsh-bridge/tree/main/examples/basic-homie-bridge)
and change one category at a time. For runtime behavior, startup semantics and
diagnostics, read
[docs/runtime-behavior.md](https://github.com/labodj/lsh-bridge/blob/main/docs/runtime-behavior.md).

## First Bring-up Baseline

For a first bring-up:

- build the bundled `release` environment unchanged
- keep the example topic layout and service topic
- keep the example codec choices until the controller-to-MQTT path works once
- make capacities large enough for the controller profile flashed into `lsh-core`
- match serial baud exactly with the controller

Most first bring-up surprises come from changing several of those at the same time. Get
one predictable successful boot first, then tune deliberately.

## Capacity and Validation

| Macro                    | Default | What it affects                                                                                         |
| ------------------------ | ------- | ------------------------------------------------------------------------------------------------------- |
| `CONFIG_MAX_ACTUATORS`   | `12U`   | Maximum actuators accepted from the controller; sizes static ETL containers, state buffers and payloads |
| `CONFIG_MAX_BUTTONS`     | `12U`   | Maximum buttons or clickables accepted from the controller                                              |
| `CONFIG_MAX_NAME_LENGTH` | `4U`    | Maximum controller device-name length; also contributes to MQTT topic buffer sizing                     |

These limits are validation rules, not display-only settings. If the controller reports
more resources than the bridge was compiled for, or a longer device name than
`CONFIG_MAX_NAME_LENGTH`, the bridge rejects the topology.

## Serial Bridge Settings

| Macro                                                | Default                           | What it affects                                                     |
| ---------------------------------------------------- | --------------------------------- | ------------------------------------------------------------------- |
| `CONFIG_ARDCOM_SERIAL_RX_PIN`                        | `16U`                             | ESP32 RX pin used by `ControllerSerialLink::begin()`                |
| `CONFIG_ARDCOM_SERIAL_TX_PIN`                        | `17U`                             | ESP32 TX pin used by `ControllerSerialLink::begin()`                |
| `CONFIG_ARDCOM_SERIAL_BAUD`                          | `250000U`                         | UART baud rate between ESP32 and controller                         |
| `CONFIG_ARDCOM_SERIAL_TIMEOUT_MS`                    | `5U`                              | Compatibility fallback for the MsgPack frame-idle timeout           |
| `CONFIG_ARDCOM_SERIAL_MSGPACK_FRAME_IDLE_TIMEOUT_MS` | `CONFIG_ARDCOM_SERIAL_TIMEOUT_MS` | Silence timeout used to discard one incomplete serial MsgPack frame |
| `CONFIG_ARDCOM_SERIAL_MAX_RX_BYTES_PER_LOOP`         | mode-derived                      | Raw UART byte budget consumed by one serial-processing pass         |

Internally the bridge keeps two serial size classes:

- `SERIAL_RX_BUFFER_SIZE`: enough for one complete inbound controller payload
- `SERIAL_MAX_RX_BYTES_PER_LOOP`: fairness budget for one bridge loop iteration

Those values are derived from capacity limits and the selected codec. They are not
public `CONFIG_*` settings.

## MQTT Topic Layout

| Macro                       | Default              | What it affects                                                         |
| --------------------------- | -------------------- | ----------------------------------------------------------------------- |
| `CONFIG_MQTT_TOPIC_BASE`    | `"LSH"`              | Root prefix for device-scoped topics                                    |
| `CONFIG_MQTT_TOPIC_INPUT`   | `"IN"`               | Device-specific inbound command suffix                                  |
| `CONFIG_MQTT_TOPIC_STATE`   | `"state"`            | Authoritative compact state publish suffix                              |
| `CONFIG_MQTT_TOPIC_CONF`    | `"conf"`             | Controller topology/configuration publish suffix                        |
| `CONFIG_MQTT_TOPIC_EVENTS`  | `"events"`           | Controller-backed runtime events, including device-level `PING` replies |
| `CONFIG_MQTT_TOPIC_BRIDGE`  | `"bridge"`           | Bridge-local diagnostics and service-level replies                      |
| `CONFIG_MQTT_TOPIC_SERVICE` | `"LSH/Node-RED/SRV"` | Broadcast/service topic subscribed independently from device name       |

Topic buffer sizes are derived from these strings plus `CONFIG_MAX_NAME_LENGTH`.
`lsh-bridge` does not expose public `CONFIG_MQTT_*_LENGTH` settings.

## Homie Convention and Identity

| Macro                           | Default           | What it affects                                               |
| ------------------------------- | ----------------- | ------------------------------------------------------------- |
| `HOMIE_CONVENTION_VERSION`      | required `5`      | Homie convention compiled by the `labodj/homie-v5` dependency |
| `CONFIG_HOMIE_FIRMWARE_NAME`    | `"lsh-homie"`     | Firmware name exposed through Homie                           |
| `CONFIG_HOMIE_FIRMWARE_VERSION` | `"1.4.4"`         | Firmware version exposed through Homie                        |
| `CONFIG_HOMIE_BRAND`            | `"LaboSmartHome"` | Homie brand string exposed by the bridge                      |

`HOMIE_CONVENTION_VERSION=5` must be passed by the embedding PlatformIO environment. The
Homie dependency compiles its own translation units, so it must see the same convention
selector as `lsh-bridge`. The bridge rejects missing or legacy values at compile time.
Use the Homie dependency declared by `lsh-bridge`; older manual pins may miss the v5
description and property-retention behavior expected by the bridge.

The identity macros must expand to string literals because the Homie dependency builds
its firmware identity through macro concatenation.

## Liveness Timers

| Macro                                      | Default                                      | What it affects                                                         |
| ------------------------------------------ | -------------------------------------------- | ----------------------------------------------------------------------- |
| `CONFIG_PING_INTERVAL_CONTROLLINO_MS`      | `10000U`                                     | Minimum spacing between `PING_` frames sent to the controller           |
| `CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS` | `CONFIG_PING_INTERVAL_CONTROLLINO_MS + 200U` | Time after the last controller frame before the link is considered lost |

Keep the timeout slightly above the ping interval so one delayed ping does not
immediately look like a disconnected controller.

## Runtime Policy Settings

| Macro                                        | Default                                | What it affects                                                                |
| -------------------------------------------- | -------------------------------------- | ------------------------------------------------------------------------------ |
| `CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS`       | `500U`                                 | Delay between repeated `REQUEST_DETAILS` and `REQUEST_STATE` requests          |
| `CONFIG_TOPOLOGY_SAVE_RETRY_INTERVAL_MS`     | `CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS` | Delay between repeated NVS save attempts during topology migration             |
| `CONFIG_TOPOLOGY_REBOOT_GRACE_MS`            | `CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS` | Grace window before rebooting after a successful topology save                 |
| `CONFIG_STATE_PUBLISH_SETTLE_INTERVAL_MS`    | `40U`                                  | Quiet window before received authoritative state is mirrored to MQTT and Homie |
| `CONFIG_MQTT_COMMAND_QUEUE_CAPACITY`         | `8U`                                   | Complete MQTT frames buffered while the serial side is busy                    |
| `CONFIG_MQTT_MAX_COMMANDS_PER_LOOP`          | `8U`                                   | Queued MQTT commands drained in one loop iteration while the UART is idle      |
| `CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS` | `50U`                                  | Quiet window used to coalesce actuator writes into one `SET_STATE`             |
| `CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS`     | `1000U`                                | Maximum lifetime of one unstable pending actuator batch                        |
| `CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT` | `32U`                                  | Maximum accepted mutations merged into one pending actuator batch              |
| `CONFIG_LSH_BRIDGE_DISABLE_RESET_TRIGGER`    | `0`                                    | Default for `BridgeOptions::disableResetTrigger`                               |

`CONFIG_MQTT_COMMAND_QUEUE_CAPACITY` costs RAM directly because each queued command owns
one fixed-size command buffer. `CONFIG_MQTT_MAX_COMMANDS_PER_LOOP` is a fairness
setting, not a capacity setting.

Leave Homie's physical reset trigger enabled for workbench bring-up. For unattended
bridge boards where GPIO0/BOOT must not act as a factory-reset input, set
`CONFIG_LSH_BRIDGE_DISABLE_RESET_TRIGGER=1` or configure
`BridgeOptions::disableResetTrigger`.

## Implementation Storage

| Macro                                 | Default | What it affects                                                   |
| ------------------------------------- | ------- | ----------------------------------------------------------------- |
| `CONFIG_LSH_BRIDGE_IMPL_STORAGE_SIZE` | `3072U` | Static byte storage reserved inside the public `LSHBridge` facade |

The bridge keeps its implementation opaque without heap allocation. Most projects can
leave this unset. If an unusual capacity profile makes the hidden implementation larger
than the default storage, compilation fails with a clear `static_assert` and the
embedding project can raise this value.

The MQTT side also keeps two size classes:

- `MQTT_COMMAND_MESSAGE_MAX_SIZE`: queue slot size for inbound commands
- `MQTT_PUBLISH_MESSAGE_MAX_SIZE`: temporary buffer for outbound MQTT payloads,
  including retained `conf` topology publishes

The outbound publish buffer is the only buffer that scales with worst-case
`DEVICE_DETAILS`; inbound commands do not pay that topology worst-case cost.

## Built-in Topology Cache

`lsh-bridge` persists the last validated controller `DEVICE_DETAILS` snapshot in ESP32
NVS. This behavior is built in and does not currently expose public `CONFIG_*` settings.

Important details:

- only topology is persisted: device name, actuator IDs and button IDs
- the stored record uses an explicit byte format with magic, version and checksum
- older or invalid cache records are ignored and rebuilt from the controller
- runtime actuator `STATE` is never written to flash
- the cache is written only after the controller confirms a real topology change
- the controller remains authoritative on every boot

The compile-time limits that constrain this cache are `CONFIG_MAX_ACTUATORS`,
`CONFIG_MAX_BUTTONS` and `CONFIG_MAX_NAME_LENGTH`.

## Codec and Behavior Flags

| Macro                     | Default   | What it affects                                                                  |
| ------------------------- | --------- | -------------------------------------------------------------------------------- |
| `CONFIG_MSG_PACK_ARDUINO` | undefined | Controller serial side uses framed MessagePack instead of newline-delimited JSON |
| `CONFIG_MSG_PACK_MQTT`    | undefined | MQTT payloads use MessagePack instead of JSON text                               |
| `LSH_DEBUG`               | undefined | Enables debug logging when `BridgeOptions::loggingMode` is `AutoFromBuild`       |
| `HOMIE_RESET`             | undefined | Erases a valid Homie configuration once, then reboots into setup/AP mode         |

If serial and MQTT use the same codec, the bridge can forward some inbound command
payloads without re-encoding. If only one side uses MessagePack, the bridge deserializes
and serializes again between formats.

Serial MessagePack framing affects only the controller link. MQTT payloads are not
framed, and the logical LSH payload format stays the same.

## ETL Profile Override

`lsh-bridge` ships with a default
[`include/etl_profile.h`](https://github.com/labodj/lsh-bridge/blob/main/include/etl_profile.h)
for the standard Arduino/PlatformIO case. It keeps ETL compiler/platform detection on
the normal auto-detect path and applies only the bridge's default ETL policy flags.

If an embedding project needs different ETL behavior:

1. Create a small project-local override header, for example
   `include/lsh_etl_profile_override.h`.
2. Pass `-D LSH_ETL_PROFILE_OVERRIDE_HEADER=\"lsh_etl_profile_override.h\"`.
3. Redefine only the ETL macros that must change.

Example:

```cpp
// include/lsh_etl_profile_override.h
#pragma once

#undef ETL_VERBOSE_ERRORS
#define ETL_THROW_EXCEPTIONS
```

The bundled example demonstrates this hook through
[examples/basic-homie-bridge/include/lsh_etl_profile_override.h](https://github.com/labodj/lsh-bridge/blob/main/examples/basic-homie-bridge/include/lsh_etl_profile_override.h)
and the matching PlatformIO build flag.

## Runtime Behaviors Outside `CONFIG_*`

Some bridge behavior is runtime-managed rather than compile-time configured:

- bridge-local diagnostics published on the MQTT `bridge` topic
- diagnostic payload shapes and aggregation policy
- the NVS-backed `DEVICE_DETAILS` cache write policy
- queue indexing, critical sections and document pool sizing

Those behaviors are documented in
[docs/runtime-behavior.md](https://github.com/labodj/lsh-bridge/blob/main/docs/runtime-behavior.md).

## PlatformIO Example

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
    ; -D CONFIG_ARDCOM_SERIAL_MAX_RX_BYTES_PER_LOOP=32U
    -D CONFIG_MQTT_TOPIC_BASE=\"LSH\"
    -D CONFIG_MQTT_TOPIC_INPUT=\"IN\"
    -D CONFIG_MQTT_TOPIC_STATE=\"state\"
    -D CONFIG_MQTT_TOPIC_CONF=\"conf\"
    -D CONFIG_MQTT_TOPIC_EVENTS=\"events\"
    -D CONFIG_MQTT_TOPIC_BRIDGE=\"bridge\"
    -D CONFIG_MQTT_TOPIC_SERVICE=\"LSH/Node-RED/SRV\"
    -D HOMIE_CONVENTION_VERSION=5
    -D CONFIG_HOMIE_FIRMWARE_NAME=\"lsh-homie\"
    -D CONFIG_HOMIE_FIRMWARE_VERSION=\"1.4.4\"
    -D CONFIG_HOMIE_BRAND=\"LaboSmartHome\"
    -D CONFIG_PING_INTERVAL_CONTROLLINO_MS=10000U
    -D CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS=10200U
    ; -D CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS=500U
    ; -D CONFIG_STATE_PUBLISH_SETTLE_INTERVAL_MS=40U
    ; -D CONFIG_MQTT_COMMAND_QUEUE_CAPACITY=8U
    ; -D CONFIG_MQTT_MAX_COMMANDS_PER_LOOP=8U
    ; -D CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS=50U
    ; -D CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS=1000U
    ; -D CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT=32U
    ; -D CONFIG_LSH_BRIDGE_DISABLE_RESET_TRIGGER=1
    ; -D CONFIG_LSH_BRIDGE_IMPL_STORAGE_SIZE=3072U
    -D CONFIG_MSG_PACK_ARDUINO
    ; -D CONFIG_MSG_PACK_MQTT
    ; -D LSH_DEBUG
    ; -D HOMIE_RESET
```
