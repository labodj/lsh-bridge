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
- compile-time `CONFIG_*` knobs are still supported and intentionally preserved
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
- OTA and deployment tooling
- any project-specific examples and board matrix

## Development

The bundled example project lives in [examples/basic-homie-bridge](./examples/basic-homie-bridge).

The example exposes two build profiles:

- `release`: conservative and simple, suitable as a library smoke test
- `release_aggressive`: mirrors the original `lsh-esp` optimization style more closely

To verify the vendored protocol stays aligned:

```bash
python3 tools/update_lsh_protocol.py --check
```

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

ESP8266 compatibility is not a design goal of this repo.
