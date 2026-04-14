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
| `CONFIG_ARDCOM_SERIAL_RX_PIN` | `16U` | RX pin used by `ArdCom::begin()`. |
| `CONFIG_ARDCOM_SERIAL_TX_PIN` | `17U` | TX pin used by `ArdCom::begin()`. |
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

## Liveness timers

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_PING_INTERVAL_CONTROLLINO_MS` | `10000U` | Minimum spacing between `PING_` frames sent to the controller. |
| `CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS` | `CONFIG_PING_INTERVAL_CONTROLLINO_MS + 200U` | How long the controller may stay silent before the bridge marks it disconnected. |

## Codec and behavior flags

| Macro | Default | What it affects |
| --- | --- | --- |
| `CONFIG_MSG_PACK_ARDUINO` | undefined | Serial side uses MessagePack instead of newline-delimited JSON. |
| `CONFIG_MSG_PACK_MQTT` | undefined | MQTT side uses MessagePack instead of JSON text. |
| `LSH_DEBUG` | undefined | Enables debug logging. With `BridgeOptions::loggingMode = AutoFromBuild`, logs are on only when this flag is defined. |
| `HOMIE_RESET` | undefined | Calls `Homie.reset()` and `Homie.setIdle(true)` during `bridge.begin()`. Enable only if reset-on-boot is intentional. |

If `CONFIG_MSG_PACK_ARDUINO` and `CONFIG_MSG_PACK_MQTT` are both defined or both undefined, the bridge can forward some inbound command payloads without re-encoding. If only one side uses MessagePack, the bridge deserializes and reserializes between the two formats.

## PlatformIO example

```ini
build_flags =
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
    -D CONFIG_PING_INTERVAL_CONTROLLINO_MS=10000U
    -D CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS=10200U
    -D CONFIG_MSG_PACK_ARDUINO
    ; -D CONFIG_MSG_PACK_MQTT
    ; -D LSH_DEBUG
    ; -D HOMIE_RESET
```
