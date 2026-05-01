# lsh-bridge: ESP32 Bridge for Labo Smart Home

[![PlatformIO Registry](https://badges.registry.platformio.org/packages/labodj/library/lsh-bridge.svg)](https://registry.platformio.org/libraries/labodj/lsh-bridge)
[![CI](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fapi.github.com%2Frepos%2Flabodj%2Flsh-bridge%2Factions%2Fworkflows%2Fci.yml%2Fruns%3Fper_page%3D1&query=%24.workflow_runs%5B0%5D.conclusion&label=CI)](https://github.com/labodj/lsh-bridge/actions/workflows/ci.yml)
[![Latest Release](https://img.shields.io/github/release/labodj/lsh-bridge.svg)](https://github.com/labodj/lsh-bridge/releases/latest)
[![License](https://img.shields.io/github/license/labodj/lsh-bridge.svg)](https://github.com/labodj/lsh-bridge/blob/main/LICENSE)

`lsh-bridge` is the ESP32 side of the **Labo Smart Home** controller path. It talks to
an `lsh-core` controller over UART, exposes the controller model over MQTT and Homie,
and keeps the network-facing runtime synchronized with the physical panel.

The best documented path is one bridge per controller: a Controllino-style AVR device
running `lsh-core`, an ESP32 running `lsh-bridge`, an MQTT broker, and the LSH
coordinator or Node-RED logic layer above it.

If you are new to LSH as a whole, start with the
[`labo-smart-home` documentation map](https://github.com/labodj/labo-smart-home/blob/main/DOCS.md)
before changing bridge settings.

The `main` branch can move ahead of tagged releases. For downstream projects, prefer
released tags unless you are intentionally testing coordinated unreleased work across
the LSH repos.

## What lsh-bridge Owns

`lsh-bridge` is intentionally narrow:

- it reads and writes the controller serial protocol
- it rebuilds the MQTT and Homie model from controller `DEVICE_DETAILS`
- it publishes controller state, events and bridge diagnostics
- it coalesces actuator command bursts before sending `SET_STATE` to the controller
- it keeps a validated controller topology cache in ESP32 NVS
- it asks the controller to resync after boot, MQTT recovery or topology changes

The bridge does not own field I/O. Buttons, relays, indicators and local fallback remain
controller responsibilities.

## What You Need

For the documented bridge path:

- PlatformIO
- an ESP32 board supported by `pioarduino/platform-espressif32`
- a controller running `lsh-core`
- a hardware UART between controller and ESP32
- a 5 V / 3.3 V level shifter when the controller UART is 5 V
- an MQTT broker; the bridge publishes Homie discovery from the controller topology

The bridge is optimized for a static controller topology. It can recover from a changed
topology, but that change is still expected to come from reflashing or rebooting the
controller, not from devices appearing dynamically at runtime.

## First Build

The quickest working reference is the bundled PlatformIO example:

```bash
platformio run -d examples/basic-homie-bridge -e release
```

The example also keeps CI-backed variants for codec and optimization coverage:

- `release`: conservative first build
- `release_aggressive`: closer to the optimized deployment style
- `release_json_serial`: JSON on the controller UART
- `release_msgpack_mqtt`: MessagePack on serial and MQTT
- `release_json_serial_msgpack_mqtt`: JSON on serial, MessagePack on MQTT

Keep the `release` profile for the first bring-up. Change codecs, topic names or
capacity limits after the controller, bridge and MQTT path have worked once together.

## Embed the Library

Install from the PlatformIO Registry:

```ini
lib_deps =
    labodj/lsh-bridge@^1.4.1
```

The embedding firmware owns board choice, serial pins, topic names, firmware identity
and deployment policy. A minimal Arduino entry point looks like this:

```cpp
#include <Arduino.h>
#include <lsh_bridge.hpp>

namespace {

lsh::bridge::BridgeOptions makeBridgeOptions() {
  lsh::bridge::BridgeOptions options;
  options.serial = &Serial2;
  options.disableLedFeedback = true;
  return options;
}

lsh::bridge::LSHBridge bridge(makeBridgeOptions());

}  // namespace

void setup() {
  bridge.begin();
}

void loop() {
  bridge.loop();
}
```

Leaving `BridgeOptions::serial` unset resolves to `Serial2`. The other defaults disable
Homie LED feedback, keep ESP32 Wi-Fi modem sleep disabled and use build-driven logging.
Set the options explicitly when you want the firmware entry point to document those
choices.

The full example lives in
[examples/basic-homie-bridge](https://github.com/labodj/lsh-bridge/tree/main/examples/basic-homie-bridge).

## Hardware Integration

In the public panel pattern, the ESP32 bridge is paired one-to-one with the controller:

```text
12/24 VDC supply
        |
        +-------------------------------> Controllino / lsh-core
        |
        +--> 5 V buck converter -------> ESP32 / lsh-bridge

Controller TTL UART
        |
        +--> 5 V / 3.3 V level shifter --> ESP32 UART

Common ground shared by controller, buck converter, level shifter and ESP32.
```

The bridge usually talks to the controller through a hardware UART, not through USB. On
Controllino-style panels, the controller side is 5 V logic and the ESP32 side is 3.3 V
logic, so the UART path needs a proper level shifter.

For panel-level power and wiring context, read the
[Labo Smart Home hardware overview](https://github.com/labodj/labo-smart-home/blob/main/HARDWARE_OVERVIEW.md).

## Runtime Shape

```text
+-------------+    serial    +------------+    MQTT    +--------------+
| lsh-core    | <----------> | lsh-bridge | <--------> | coordinator  |
| controller  |              | ESP32      |            | or Node-RED  |
+-------------+              +------------+            +--------------+
```

Runtime rules:

- the controller remains authoritative for actuator state
- the bridge caches only validated topology, never runtime actuator state
- startup uses cached topology when available, then asks the controller for fresh
  details and state
- device-topic `PING` answers controller reachability only when the runtime is synced
- service-topic `PING` answers bridge reachability on the bridge-local topic
- topology changes are saved to NVS and followed by one controlled reboot
- retained, fragmented, oversize or malformed MQTT commands are rejected and diagnosed

For the detailed runtime story, read
[docs/runtime-behavior.md](https://github.com/labodj/lsh-bridge/blob/main/docs/runtime-behavior.md).

## Configuration

Most bridge choices are compile-time PlatformIO flags because they affect fixed buffers,
topic sizes, protocol codecs or Homie identity. The most common groups are:

- capacities: `CONFIG_MAX_ACTUATORS`, `CONFIG_MAX_BUTTONS`, `CONFIG_MAX_NAME_LENGTH`
- serial: `CONFIG_ARDCOM_SERIAL_RX_PIN`, `CONFIG_ARDCOM_SERIAL_TX_PIN`,
  `CONFIG_ARDCOM_SERIAL_BAUD`
- MQTT topics: `CONFIG_MQTT_TOPIC_BASE`, `CONFIG_MQTT_TOPIC_EVENTS`,
  `CONFIG_MQTT_TOPIC_BRIDGE`, `CONFIG_MQTT_TOPIC_SERVICE`
- Homie: `HOMIE_CONVENTION_VERSION=5`, `CONFIG_HOMIE_FIRMWARE_NAME`,
  `CONFIG_HOMIE_FIRMWARE_VERSION`, `CONFIG_HOMIE_BRAND`
- runtime policy: queue capacity, command coalescing windows, topology reboot timing and
  reset-trigger policy
- codecs: `CONFIG_MSG_PACK_ARDUINO` and `CONFIG_MSG_PACK_MQTT`

Use the bundled example as the starting point. The complete reference lives in
[docs/compile-time-configuration.md](https://github.com/labodj/lsh-bridge/blob/main/docs/compile-time-configuration.md).

## Documentation

- [DOCS.md](https://github.com/labodj/lsh-bridge/blob/main/DOCS.md): repository
  documentation map
- [docs/runtime-behavior.md](https://github.com/labodj/lsh-bridge/blob/main/docs/runtime-behavior.md):
  startup, sync, diagnostics and MQTT behavior
- [docs/compile-time-configuration.md](https://github.com/labodj/lsh-bridge/blob/main/docs/compile-time-configuration.md):
  build flags and capacity policy
- [LSH reference stack](https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md):
  cross-repo runtime model

## Compatibility

Validated target:

- ESP32
- Arduino framework
- `pioarduino/platform-espressif32`
- Homie convention v5

Recommended PlatformIO platform:

```ini
platform = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip
framework = arduino
board = esp32dev
```

`stable` is a moving platform alias. Validate every `lsh-bridge` release against the
current platform before deployment.

OTA updates only the application image (`firmware.bin`). It does not update bootloader,
partition table or other full-flash artifacts. If the ESP32 platform update changes
those artifacts, validate them explicitly and use a full USB/serial flash when required.

ESP8266 compatibility is not a design goal of this repository.

## Maintainer Notes

Maintainer workflow notes live in
[DOCS.md](https://github.com/labodj/lsh-bridge/blob/main/DOCS.md#maintainer-notes).
Package smoke and release publishing use `platformio pkg pack`, so exported Markdown
uses absolute links rather than repository-relative links.
