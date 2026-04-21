# lsh-bridge

`lsh-bridge` is the reusable library form of the LSH ESP bridge runtime.

The goal of this repo is narrow and pragmatic:

- keep the runtime bridge logic that already works
- expose it through a small library facade
- leave deploy-specific PlatformIO tuning, branding and firmware wiring to the consumer project

The library keeps the bridge runtime behind a small public API.

## What the library does

The library runs an ESP32 bridge between:

- an LSH serial controller such as a Controllino or Arduino running `lsh-core`
- an MQTT broker
- the Homie device model exposed on top of MQTT

At runtime it:

- restores the last validated `DEVICE_DETAILS` snapshot from NVS when available
- starts Homie immediately and advances controller sync in the main loop through a non-blocking `details -> state` flow
- creates one Homie node per actuator declared by the controller
- keeps an authoritative compact LSH state topic in sync
- coalesces actuator command bursts before writing `SET_STATE` back to the controller
- drops unstable actuator command storms once one batch exceeds the configured safety limits
- publishes controller-backed runtime traffic on MQTT `events` and bridge-local runtime traffic on MQTT `bridge`
- re-synchronizes the runtime model when MQTT becomes ready again
- treats MQTT device-topic `PING` as controller-backed reachability only when the controller link is alive and synchronized
- treats MQTT service-topic `PING` as a bridge-local probe answered on `bridge` whenever a validated device identity exists, and MQTT service-topic `BOOT` as a bridge-local resync trigger toward the controller
- persists a changed controller topology and performs one controlled reboot so MQTT topics and Homie nodes are rebuilt from a coherent snapshot

## Typical Hardware Topology

In the real LSH installation, `lsh-bridge` is typically paired one-to-one with a **Controllino Maxi** inside an electrical panel.

```text
12/24 VDC power supply
        |
        +-------------------------------> Controllino / lsh-core device
        |
        +--> 5 V buck converter -------> ESP32 / lsh-bridge

Controllino front TTL serial
        |
        +--> 5 V / 3.3 V logic level shifter --> ESP32 UART

Common ground shared by controller, buck converter, level shifter and ESP32.
```

Practical notes from the live installation:

- the bridge talks to the controller over the Controllino front TTL interface, not over USB
- when the controller side is 5 V logic and the ESP32 side is 3.3 V logic, a level shifter is required on the UART path
- the ESP32 is powered from a dedicated 5 V rail derived from the same 12/24 V source used by the controller
- controller-to-bridge wiring in production panels uses solid connectorized links
- some panels expose short USB extension cables outside the enclosure so bridge firmware can be flashed without reopening internal wiring

This repo stays focused on the reusable bridge runtime, but the hardware context matters because it explains why the bridge is intentionally narrow: serial in, MQTT/Homie out, no ownership of the field I/O itself.

For a system-level view of the installation pattern, see the public landing repo hardware notes:
[Labo Smart Home hardware overview](https://github.com/labodj/labo-smart-home/blob/main/HARDWARE_OVERVIEW.md)

## Public API

The public surface is intentionally small:

```cpp
#include <lsh_bridge.hpp>

lsh::bridge::BridgeOptions options;
lsh::bridge::LSHBridge bridge(options);

void setup() {
  bridge.begin();
}

void loop() {
  bridge.loop();
}
```

See [examples/basic-homie-bridge/src/main.cpp](./examples/basic-homie-bridge/src/main.cpp).

## Design notes

- the library currently assumes a single active bridge instance
- Homie remains part of the opinionated runtime surface for now
- runtime command handling and bridge-local diagnostics are documented in
  [docs/runtime-behavior.md](./docs/runtime-behavior.md)
- compile-time `CONFIG_*` knobs are supported and documented in
  [docs/compile-time-configuration.md](./docs/compile-time-configuration.md)
- the protocol source of truth remains vendored as `vendor/lsh-protocol`
- Homie firmware identity is configured at compile time through
  `CONFIG_HOMIE_FIRMWARE_NAME`, `CONFIG_HOMIE_FIRMWARE_VERSION` and
  `CONFIG_HOMIE_BRAND`
- logging defaults to `AutoFromBuild`: logging is disabled in normal builds and stays enabled when `LSH_DEBUG` is defined
- the bridge persists only validated controller `DEVICE_DETAILS` in ESP32 NVS; runtime actuator state always comes fresh from the controller

This is not yet a fully generic bridge framework. It is a clean extraction of the current working runtime into a library-shaped repo.

## Consumer responsibilities

The consumer project should own:

- the PlatformIO environment
- firmware identity and release/versioning policy
- serial pin overrides and other `CONFIG_*` choices
- validation of board settings, partition layout and full-flash artifacts when the ESP32 platform changes
- OTA and deployment tooling
- any project-specific examples and board matrix

## Compile-time Configuration

`lsh-bridge` exposes a deliberate build-time surface for capacities, serial
wiring, MQTT topic naming, liveness timers and codec selection.

The full reference lives in
[docs/compile-time-configuration.md](./docs/compile-time-configuration.md).
Runtime-only behaviors such as actuator command coalescing, unstable command
storm protection, inbound MQTT queue backpressure and bridge-local MQTT
diagnostics such as `mqtt_queue_overflow` and `mqtt_command_rejected`
(including malformed-command rejections) are documented separately in
[docs/runtime-behavior.md](./docs/runtime-behavior.md).
The bundled
[example `platformio.ini`](./examples/basic-homie-bridge/platformio.ini)
explicitly sets the supported value macros and leaves optional behavior flags
commented.

Current supported knobs:

- capacities: `CONFIG_MAX_ACTUATORS`, `CONFIG_MAX_BUTTONS`, `CONFIG_MAX_NAME_LENGTH`
- serial: `CONFIG_ARDCOM_SERIAL_RX_PIN`, `CONFIG_ARDCOM_SERIAL_TX_PIN`, `CONFIG_ARDCOM_SERIAL_BAUD`, `CONFIG_ARDCOM_SERIAL_TIMEOUT_MS`, `CONFIG_ARDCOM_SERIAL_MSGPACK_FRAME_IDLE_TIMEOUT_MS`, `CONFIG_ARDCOM_SERIAL_MAX_RX_BYTES_PER_LOOP`
- MQTT topics: `CONFIG_MQTT_TOPIC_BASE`, `CONFIG_MQTT_TOPIC_INPUT`, `CONFIG_MQTT_TOPIC_STATE`, `CONFIG_MQTT_TOPIC_CONF`, `CONFIG_MQTT_TOPIC_EVENTS`, `CONFIG_MQTT_TOPIC_BRIDGE`, `CONFIG_MQTT_TOPIC_SERVICE`
- Homie identity: `CONFIG_HOMIE_FIRMWARE_NAME`, `CONFIG_HOMIE_FIRMWARE_VERSION`, `CONFIG_HOMIE_BRAND`
- liveness: `CONFIG_PING_INTERVAL_CONTROLLINO_MS`, `CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS`
- runtime policy: `CONFIG_BOOTSTRAP_REQUEST_INTERVAL_MS`, `CONFIG_STATE_PUBLISH_SETTLE_INTERVAL_MS`, `CONFIG_MQTT_COMMAND_QUEUE_CAPACITY`, `CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS`, `CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS`, `CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT`
- codecs and flags: `CONFIG_MSG_PACK_ARDUINO`, `CONFIG_MSG_PACK_MQTT`, `LSH_DEBUG`, `HOMIE_RESET`
- ETL override hook: `LSH_ETL_PROFILE_OVERRIDE_HEADER`

The bundled example also includes a ready-to-use project-local ETL override
header at
[examples/basic-homie-bridge/include/lsh_etl_profile_override.h](./examples/basic-homie-bridge/include/lsh_etl_profile_override.h).

## Development

The bundled example project lives in [examples/basic-homie-bridge](./examples/basic-homie-bridge).

The example exposes two build profiles:

- `release`: conservative and simple, suitable as a library smoke test
- `release_aggressive`: mirrors the original bridge firmware optimization style more closely

To verify the vendored protocol stays aligned:

```bash
python3 tools/update_lsh_protocol.py --check
```

## Updating Vendored Protocol

`lsh-protocol` is vendored in this repo under `vendor/lsh-protocol` via `git subtree`.

Typical update flow after pushing changes to `lsh-protocol`:

```bash
git remote add lsh-protocol git@github.com:labodj/lsh-protocol.git || \
git remote set-url lsh-protocol git@github.com:labodj/lsh-protocol.git
git fetch lsh-protocol
git subtree pull --prefix=vendor/lsh-protocol lsh-protocol main --squash
python3 tools/update_lsh_protocol.py
python3 tools/update_lsh_protocol.py --check
```

The subtree update refreshes the vendored source-of-truth copy. The local wrapper then regenerates
the bridge-specific outputs under `src/constants/`.

## Current boundaries

Inside the library:

- `ControllerSerialLink`
- `VirtualDevice`
- `MqttTopicsBuilder`
- `LSHNode`
- `LSHBridge`

Still intentionally compile-time configured:

- topic strings
- serial pins and baud
- codec selection
- limits for actuators, buttons and device name size

## Compatibility

Validated target:

- ESP32
- `pioarduino/platform-espressif32`

Recommended PlatformIO configuration:

```ini
platform = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip
framework = arduino
board = esp32dev
```

`stable` is a moving release alias. The exact Arduino core / ESP-IDF toolchain
behind it can change over time, so each `lsh-bridge` release should be validated
against the current `stable` platform release before deployment.

`lsh-bridge` itself cannot pin the PlatformIO platform version for consumers.
That choice belongs to the consumer project.

OTA updates only the application image (`firmware.bin`). It does not update the
bootloader, partition table or other full-flash artifacts. If the ESP32 platform
update changes those artifacts, validate them explicitly and use a full USB/serial
flash when required.

ESP8266 compatibility is not a design goal of this repo.
