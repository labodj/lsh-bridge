# lsh-bridge

`lsh-bridge` is the reusable library form of the LSH ESP bridge runtime.

The goal of this repo is narrow and pragmatic:

- keep the runtime bridge logic that already works
- expose it through a small library facade
- leave deploy-specific PlatformIO tuning, branding and firmware wiring to the consumer project

This first extraction intentionally reuses most of the existing implementation. The main structural change is that the old firmware-oriented `main.cpp` has been replaced by a library facade with a minimal public API.

## What the library does

The library runs an ESP32 bridge between:

- an LSH serial controller such as a Controllino or Arduino running `lsh-core`
- an MQTT broker
- the Homie device model exposed on top of MQTT

At runtime it:

- performs the blocking `details -> state` bootstrap handshake
- creates one Homie node per actuator declared by the controller
- keeps an authoritative compact LSH state topic in sync
- coalesces actuator command bursts before writing `SET_STATE` back to the controller
- re-synchronizes the runtime model when MQTT becomes ready again

## Public API

The public surface is intentionally small:

```cpp
#include <lsh_esp_bridge.hpp>

lsh::esp::BridgeOptions options;
options.identity.setFirmwareVersion("1.0.0");
lsh::esp::LSHEspBridge bridge(options);

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
- compile-time `CONFIG_*` knobs are still supported and documented in
  [docs/compile-time-configuration.md](./docs/compile-time-configuration.md)
- the protocol source of truth remains vendored as `vendor/lsh-protocol`
- firmware identity is stored in bounded ETL strings and validated before use
  (`firmwareName <= 32`, `firmwareVersion <= 16`, `homieBrand <= 21`)
- logging now defaults to `AutoFromBuild`, which preserves the original `lsh-esp` behavior:
  logging is disabled in normal builds and stays enabled when `LSH_DEBUG` is defined

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

`lsh-bridge` still exposes a deliberate build-time surface for capacities, serial
wiring, MQTT topic naming, liveness timers and codec selection.

The full reference lives in
[docs/compile-time-configuration.md](./docs/compile-time-configuration.md).
The bundled
[example `platformio.ini`](./examples/basic-homie-bridge/platformio.ini)
explicitly sets the supported value macros and leaves optional behavior flags
commented.

Current supported knobs:

- capacities: `CONFIG_MAX_ACTUATORS`, `CONFIG_MAX_BUTTONS`, `CONFIG_MAX_NAME_LENGTH`
- serial: `CONFIG_ARDCOM_SERIAL_RX_PIN`, `CONFIG_ARDCOM_SERIAL_TX_PIN`, `CONFIG_ARDCOM_SERIAL_BAUD`, `CONFIG_ARDCOM_SERIAL_TIMEOUT_MS`
- MQTT topics: `CONFIG_MQTT_TOPIC_BASE`, `CONFIG_MQTT_TOPIC_INPUT`, `CONFIG_MQTT_TOPIC_STATE`, `CONFIG_MQTT_TOPIC_CONF`, `CONFIG_MQTT_TOPIC_MISC`, `CONFIG_MQTT_TOPIC_SERVICE`
- liveness: `CONFIG_PING_INTERVAL_CONTROLLINO_MS`, `CONFIG_CONNECTION_TIMEOUT_CONTROLLINO_MS`
- codecs and flags: `CONFIG_MSG_PACK_ARDUINO`, `CONFIG_MSG_PACK_MQTT`, `LSH_DEBUG`, `HOMIE_RESET`

## Development

The bundled example project lives in [examples/basic-homie-bridge](./examples/basic-homie-bridge).

The example exposes two build profiles:

- `release`: conservative and simple, suitable as a library smoke test
- `release_aggressive`: mirrors the original `lsh-esp` optimization style more closely

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

- `ArdCom`
- `VirtualDevice`
- `MqttTopicsBuilder`
- `LSHNode`
- `LSHEspBridge`

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
