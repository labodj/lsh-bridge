# Compile-time Configuration

`lsh-bridge` keeps a deliberately small but important compile-time surface. All supported knobs are ordinary C/C++ preprocessor macros passed through PlatformIO `build_flags`.

The bundled example in [`examples/basic-homie-bridge/platformio.ini`](../examples/basic-homie-bridge/platformio.ini) sets all value-like macros explicitly and leaves the behavior-changing flags commented.

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
| `CONFIG_ARDCOM_SERIAL_TIMEOUT_MS` | `5U` | Serial read timeout used by `HardwareSerial::setTimeout()`. |

## MQTT topic layout

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_MQTT_TOPIC_BASE` | `"LSH"` | Root prefix for device-scoped topics. |
| `CONFIG_MQTT_TOPIC_INPUT` | `"IN"` | Topic suffix subscribed for device-specific inbound commands. |
| `CONFIG_MQTT_TOPIC_STATE` | `"state"` | Topic suffix used for authoritative state publishes. |
| `CONFIG_MQTT_TOPIC_CONF` | `"conf"` | Topic suffix used for device configuration publishes. |
| `CONFIG_MQTT_TOPIC_MISC` | `"misc"` | Topic suffix used for bridge events and miscellaneous publishes. |
| `CONFIG_MQTT_TOPIC_SERVICE` | `"LSH/Node-RED/SRV"` | Broadcast/service topic subscribed independently from the device name. |

The bridge now derives topic buffer sizes automatically from the compiled strings above plus `CONFIG_MAX_NAME_LENGTH`. There are no public `CONFIG_MQTT_*_LENGTH` knobs in the current library.

## Homie identity

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_HOMIE_FIRMWARE_NAME` | `"lsh-homie"` | Firmware name exposed through Homie. |
| `CONFIG_HOMIE_FIRMWARE_VERSION` | `"1.1.0"` | Firmware version exposed through Homie. |
| `CONFIG_HOMIE_BRAND` | `"LaboSmartHome"` | Homie brand string exposed by the bridge. |

These macros must expand to string literals. `homie-esp8266` builds its
internal flagged string format through macro concatenation, so firmware
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
| `CONFIG_STATE_PUBLISH_SETTLE_INTERVAL_MS` | `40U` | Quiet window before a freshly received authoritative state is mirrored back out to MQTT and Homie. |
| `CONFIG_MQTT_COMMAND_QUEUE_CAPACITY` | `8U` | Number of complete inbound MQTT frames the bridge may buffer while the serial side is busy. Impacts RAM. |
| `CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS` | `50U` | Quiet window used to coalesce multiple actuator writes into one outbound `SET_STATE`. |
| `CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS` | `1000U` | Hard limit for how long an unstable pending actuator batch may stay open before the bridge drops it. |
| `CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT` | `32U` | Maximum number of accepted state changes merged into one pending actuator batch before the bridge treats the producer as unstable. |

## Codec and behavior flags

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_MSG_PACK_ARDUINO` | undefined | Serial side uses MessagePack instead of newline-delimited JSON. |
| `CONFIG_MSG_PACK_MQTT` | undefined | MQTT side uses MessagePack instead of JSON text. |
| `LSH_DEBUG` | undefined | Enables debug logging. With `BridgeOptions::loggingMode = AutoFromBuild`, logs are on only when this flag is defined. |
| `HOMIE_RESET` | undefined | Calls `Homie.reset()` and `Homie.setIdle(true)` during `bridge.begin()`. Enable only if reset-on-boot is intentional. |

If `CONFIG_MSG_PACK_ARDUINO` and `CONFIG_MSG_PACK_MQTT` are both defined or both undefined, the bridge can forward some inbound command payloads without re-encoding. If only one side uses MessagePack, the bridge deserializes and reserializes between the two formats.

## ETL profile override

`lsh-bridge` ships with a default
[`include/etl_profile.h`](../include/etl_profile.h) so the standard
Arduino/PlatformIO case works without extra setup.

That default profile intentionally keeps ETL compiler/platform detection on the
official auto-detect path and only applies the bridge's default ETL policy
flags.

If a consumer needs different ETL behavior for another target or toolchain, the
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

## Runtime behaviors that still are not `CONFIG_*`

Some important bridge behaviors are still runtime-managed even after the knobs
above and are not exposed as public compile-time values:

- bridge-local diagnostics published on the MQTT `misc` topic
- the exact diagnostic payload shapes
- counter aggregation and reset policy for queue-overflow diagnostics

Those behaviors are documented in
[`docs/runtime-behavior.md`](./runtime-behavior.md).

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
    -D CONFIG_MQTT_TOPIC_BASE=\"LSH\"
    -D CONFIG_MQTT_TOPIC_INPUT=\"IN\"
    -D CONFIG_MQTT_TOPIC_STATE=\"state\"
    -D CONFIG_MQTT_TOPIC_CONF=\"conf\"
    -D CONFIG_MQTT_TOPIC_MISC=\"misc\"
    -D CONFIG_MQTT_TOPIC_SERVICE=\"LSH/Node-RED/SRV\"
    -D CONFIG_HOMIE_FIRMWARE_NAME=\"lsh-homie\"
    -D CONFIG_HOMIE_FIRMWARE_VERSION=\"1.1.0\"
    -D CONFIG_HOMIE_BRAND=\"LaboSmartHome\"
    -D CONFIG_PING_INTERVAL_CONTROLLINO_MS=10000U
    -D CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS=10200U
    ; -D CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS=500U
    ; -D CONFIG_STATE_PUBLISH_SETTLE_INTERVAL_MS=40U
    ; -D CONFIG_MQTT_COMMAND_QUEUE_CAPACITY=8U
    ; -D CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS=50U
    ; -D CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS=1000U
    ; -D CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT=32U
    -D CONFIG_MSG_PACK_ARDUINO
    ; default: undefined
    ; -D CONFIG_MSG_PACK_MQTT
    ; default: undefined
    ; -D LSH_DEBUG
    ; default: undefined
    ; -D HOMIE_RESET
    ; default: undefined
```
