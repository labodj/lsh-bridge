# Runtime Behavior

This is the operational reference for `lsh-bridge`: startup, topology recovery, MQTT
command handling, diagnostics and Homie integration.

For exact build flags and capacity settings, read
[docs/compile-time-configuration.md](https://github.com/labodj/lsh-bridge/blob/main/docs/compile-time-configuration.md).
For the first-use path and public example, start from the
[README](https://github.com/labodj/lsh-bridge/blob/main/README.md).

## Mental Model

At runtime, the bridge keeps the network view aligned with the controller view without
taking ownership of the physical panel.

That leads to a few rules:

- the controller remains authoritative for runtime actuator state
- MQTT and Homie writes are treated as remote intent until they are valid to send
- cached topology helps the bridge start quickly, but fresh controller details take
  precedence after boot
- bridge-local diagnostics are published separately from controller-backed events
- malformed or stale commands are rejected instead of being replayed later

## Controller Bootstrap and Topology Cache

Startup is non-blocking. If a validated `DEVICE_DETAILS` snapshot is available in ESP32
NVS, the bridge uses it immediately to rebuild MQTT topics and Homie nodes. It still
asks the controller for fresh details and state before treating the runtime as fully
synchronized.

During startup:

- only controller `DEVICE_DETAILS` are cached in NVS
- the cache record uses an explicit byte format with magic, version and checksum
- old or invalid cache records are ignored and rebuilt from the controller
- actuator runtime `STATE` is never persisted
- after startup, the bridge retries `REQUEST_DETAILS` at the configured bootstrap
  interval
- once details are confirmed, it retries `REQUEST_STATE` at the same interval
- the bridge is fully synchronized only after both phases complete
- device-level `SET_*` writes are rejected while the runtime is not synchronized

If no valid cache is present yet:

- Homie still starts, so setup and OTA remain available
- device-scoped LSH topics are not published until the controller identity is known
- service-topic commands are accepted, but service `PING` cannot be published under a
  device-scoped `bridge` topic until the first validated topology exists

If the controller later reports a different topology:

- the bridge stops trusting runtime state immediately
- the new topology is saved to NVS from the main loop
- a `topology_changed` diagnostic is queued when possible
- after the save succeeds, the bridge performs one controlled reboot
- on the next boot, MQTT topics and Homie nodes are rebuilt from the new topology

This keeps the controller authoritative while avoiding long blocking work inside serial
callbacks.

## Actuator Command Coalescing

Inbound MQTT actuator commands are not forwarded to the controller immediately. The
bridge keeps a short desired-state shadow and waits for the input stream to settle
before sending one packed `SET_STATE` frame.

Coalescing behavior:

- the quiet window defaults to `50 ms`
- `CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS` controls that window
- duplicate pending writes do not restart the window
- if desired state returns to the authoritative controller state before the window
  expires, the bridge may send nothing
- the controller remains the authoritative source of truth

This reduces serial chatter and avoids emitting one full `SET_STATE` frame for every
short burst of MQTT writes.

## Unstable Command Storm Protection

The bridge forwards stable intent, not endless toggles. A producer cannot keep one
actuator batch open forever by continuously changing desired states.

Stability limits:

- maximum pending batch duration defaults to `1000 ms`
- `CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS` controls that duration
- maximum accepted mutations in one batch defaults to `32`
- `CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT` controls that mutation limit

When either limit is exceeded:

- the pending batch is dropped
- desired state is realigned with the last authoritative controller state
- no `SET_STATE` frame is sent for that unstable batch
- a bridge-local diagnostic is queued on the MQTT `bridge` topic

## MQTT Inbound Queue Behavior

The AsyncMqttClient callback does not parse and route commands immediately. It validates
complete frames and copies accepted commands into a small bridge-side queue. The main
loop drains that queue with a bounded fairness policy.

Queue behavior:

- queue capacity defaults to `8` complete MQTT frames
- `CONFIG_MQTT_COMMAND_QUEUE_CAPACITY` controls that capacity
- only complete non-fragmented frames are accepted
- retained commands are ignored
- queue overflow drops the new command instead of blocking the callback
- `CONFIG_MQTT_MAX_COMMANDS_PER_LOOP` controls the idle-UART drain budget

Queue overflow is reported as a bridge-local diagnostic. It is treated as a producer or
backpressure problem, not as a reason to block MQTT callbacks.

## Optional Serial MessagePack Framing

When `CONFIG_MSG_PACK_ARDUINO` is enabled, serial MessagePack uses a framed transport.
The LSH payload stays unchanged; the bridge only wraps the serial payload so the UART
receiver can recover frames without relying on stream timeouts.

Framing rules:

- each frame starts with `0xC0`
- the payload escapes `0xC0` and `0xDB`
- each frame ends with `0xC0`

This transport layer is local to the controller UART:

- MQTT payloads are never serial-framed
- ArduinoJson sees only the recovered MessagePack payload
- raw-forwarded serial MessagePack payloads are framed only when they cross the
  controller UART

Serial MessagePack is wire-compatible only with a peer that uses the same framed
transport.

## Homie v5 Model

`lsh-bridge` requires `HOMIE_CONVENTION_VERSION=5` at build time. With the
`labodj/homie-v5` PlatformIO package, the Homie model is published as one retained JSON
document:

```text
homie/5/<device>/$description
```

Each cached controller actuator becomes one Homie node. Its canonical property is
`state`, which is boolean, retained and settable.

Actuator commands are received on:

```text
homie/5/<device>/<actuator-id>/state/set
```

Controller-backed state is mirrored to:

```text
homie/5/<device>/<actuator-id>/state
```

The bridge does not build the `$description` JSON manually. It declares nodes and
properties through the Homie dependency, which owns convention-specific serialization.

## MQTT Topic Split: `events` vs `bridge`

The bridge separates controller-backed traffic from bridge-local traffic:

- `LSH/<device>/events`: controller-backed runtime events such as `NETWORK_CLICK_*`
  payloads and device-level `PING` replies
- `LSH/<device>/bridge`: bridge-local runtime events such as service-level ping replies
  and diagnostics

This prevents an MQTT-side consumer from mistaking a live bridge for a live downstream
controller.

Device-level `PING` is answered on `events` only when both conditions are true:

- the controller link is currently considered connected
- the bridge runtime cache is synchronized with the controller

Service-topic `PING` is answered on `bridge`, never on `events`, whenever the bridge has
a valid device-scoped `bridge` topic. Before the first validated topology is cached, the
service command is accepted but there is no authoritative device identity to publish
under.

The `controller_connected` field becomes `true` only after the bridge has decoded at
least one valid controller frame in the current boot session and that frame is inside
the configured liveness timeout.

## Homie Setup Mode and `HOMIE_RESET`

Homie already enters configuration/setup mode when its stored configuration is invalid
or missing. `lsh-bridge` uses that behavior directly.

Setup behavior:

- brand-new or already-reset devices enter setup/AP mode through `Homie.setup()`
- Homie's physical reset trigger remains enabled by default
- unattended ESP32 bridge boards can disable that trigger through
  `BridgeOptions::disableResetTrigger` or `CONFIG_LSH_BRIDGE_DISABLE_RESET_TRIGGER=1`
- `BridgeOptions::disableWifiSleep` disables ESP32 Wi-Fi modem sleep by default
- `BridgeOptions::disableLedFeedback` disables Homie LED feedback by default
- when `HOMIE_RESET` is defined, the bridge calls `Homie.reset()` only if a valid stored
  Homie configuration exists

This preserves the "force setup on next boot" workflow without creating a reset loop on
devices without stored configuration.

## Bridge-local Diagnostics on `bridge`

Diagnostics are emitted from the main loop, not directly from callbacks. The bridge does
not publish MQTT while already inside the MQTT client's message callback.

Diagnostic kinds currently emitted:

- `actuator_command_storm_dropped`
- `mqtt_queue_overflow`
- `mqtt_command_rejected`
- `homie_command_rejected`
- `topology_changed`
- `last_reset_phase`

### `actuator_command_storm_dropped`

Logical payload shape:

```json
{
  "event": "diagnostic",
  "kind": "actuator_command_storm_dropped",
  "pending_ms": 1000,
  "mutation_count": 32
}
```

Fields:

- `pending_ms`: how long the unstable actuator batch stayed open
- `mutation_count`: accepted command changes merged into the dropped batch

### `mqtt_queue_overflow`

Logical payload shape:

```json
{
  "event": "diagnostic",
  "kind": "mqtt_queue_overflow",
  "dropped_device_commands": 7,
  "dropped_service_commands": 2
}
```

Fields:

- `dropped_device_commands`: device-topic commands dropped because the inbound queue was
  full
- `dropped_service_commands`: service-topic commands dropped because the inbound queue
  was full

### `mqtt_command_rejected`

Logical payload shape:

```json
{
  "event": "diagnostic",
  "kind": "mqtt_command_rejected",
  "rejected_retained_commands": 3,
  "rejected_oversize_commands": 1,
  "rejected_fragmented_commands": 2,
  "rejected_malformed_commands": 4
}
```

Fields:

- `rejected_retained_commands`: retained MQTT commands rejected because replaying them
  after reconnect could apply stale intent
- `rejected_oversize_commands`: MQTT commands rejected before enqueue because they
  exceeded the fixed inbound command buffer
- `rejected_fragmented_commands`: MQTT commands rejected because the bridge only accepts
  complete non-fragmented inbound frames
- `rejected_malformed_commands`: MQTT commands whose delivery shape was acceptable but
  whose payload was not valid JSON/MessagePack for the active codec or did not contain a
  valid `p` command id

Notes:

- when MQTT uses MessagePack, the wire encoding is MessagePack but the logical structure
  is the same
- queue overflow and command-rejection diagnostics are aggregated before publish
- pending bridge-local diagnostics are cleared when MQTT disconnects, so stale warnings
  are not replayed after reconnect

### `homie_command_rejected`

Logical payload shape:

```json
{
  "event": "diagnostic",
  "kind": "homie_command_rejected",
  "rejected_homie_desync_commands": 1,
  "rejected_homie_invalid_payload_commands": 2,
  "rejected_homie_stage_failed_commands": 3
}
```

Fields:

- `rejected_homie_desync_commands`: Homie actuator writes consumed while the bridge was
  not synchronized with the controller
- `rejected_homie_invalid_payload_commands`: Homie actuator writes rejected because the
  value was not a valid boolean command
- `rejected_homie_stage_failed_commands`: valid Homie actuator writes that could not be
  staged into the pending actuator batch

Homie property callbacks intentionally continue to consume these writes. The diagnostic
lets a caller observe the rejection without making Homie redeliver stale writes.

### `service_ping_reply`

Logical payload shape:

```json
{
  "event": "service_ping_reply",
  "controller_connected": true,
  "runtime_synchronized": true,
  "bootstrap_phase": "synced"
}
```

Fields:

- `controller_connected`: whether the bridge currently sees the downstream controller as
  alive
- `runtime_synchronized`: whether the bridge runtime cache is synchronized with the
  controller
- `bootstrap_phase`: high-level bridge sync phase

Current `bootstrap_phase` values:

- `waiting_details`
- `waiting_state`
- `synced`
- `topology_migration_pending_reboot`

### `last_reset_phase`

Logical payload shape:

```json
{
  "event": "diagnostic",
  "kind": "last_reset_phase",
  "phase": "homie_loop",
  "phase_ms": 2593000,
  "boot_count": 6
}
```

Fields:

- `phase`: bridge loop phase last recorded in RTC memory before this boot
- `phase_ms`: `millis()` value captured with that phase
- `boot_count`: bridge-local RTC boot counter captured before the current boot
  increments it

This diagnostic is retained by design. It describes the previous boot, not a
current-session warning, so late subscribers can still inspect the last observed reset
phase after recovery.

## What May Evolve

The diagnostic examples above describe the current logical payloads. Treat them as
operational signals rather than a versioned API. These details may evolve:

- exact diagnostic field set
- when pending diagnostics are aggregated, cleared or reset
- queue indexing and critical-section implementation details
- internal document pool sizing

Consumers can rely on the documented high-level behavior rather than internal scheduling
details.

## Read Next

- [Compile-time configuration](https://github.com/labodj/lsh-bridge/blob/main/docs/compile-time-configuration.md)
- [Bridge README](https://github.com/labodj/lsh-bridge/blob/main/README.md)
- [LSH reference stack](https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md)
- [LSH troubleshooting guide](https://github.com/labodj/labo-smart-home/blob/main/TROUBLESHOOTING.md)
