# Runtime Behavior

`lsh-bridge` has a few important runtime behaviors that matter for anyone
integrating, debugging or operating the bridge in the field.

Some of the key thresholds below are exposed as compile-time `CONFIG_*`
macros. This page documents the behavior itself; the complete build-time knob
reference lives in [`compile-time-configuration.md`](./compile-time-configuration.md).

If you are new to the public stack, read these first:

- [Labo Smart Home landing page](https://github.com/labodj/labo-smart-home)
- [LSH reference stack](https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md)
- [LSH getting started guide](https://github.com/labodj/labo-smart-home/blob/main/GETTING_STARTED.md)
- [LSH troubleshooting guide](https://github.com/labodj/labo-smart-home/blob/main/TROUBLESHOOTING.md)

Use this page when the bridge is already in the picture and you need to
understand why it behaves the way it does under load, reconnects, startup
recovery or malformed input.

## Actuator command coalescing

Inbound MQTT actuator commands are not forwarded immediately to the controller.
Instead, the bridge keeps a short desired-state shadow and waits for the input
stream to become stable before sending a single packed `SET_STATE` frame.

Current behavior:

- the quiet window defaults to `50 ms` and is controlled by `CONFIG_ACTUATOR_COMMAND_SETTLE_INTERVAL_MS`
- duplicate pending writes do not restart the window
- if the requested state returns to the already authoritative controller state
  before the quiet window expires, the bridge may send nothing at all
- the controller remains the authoritative source of truth; the bridge only
  stages remote intent until it is stable enough to send

This design reduces chatter on the controller serial link and avoids emitting a
full `SET_STATE` frame for every short burst of MQTT writes.

## Unstable command storm protection

The bridge intentionally prefers "stable intent only" for actuator writes.

That means a producer cannot keep a batch open forever by continuously
toggling desired states. If the desired state never settles, the bridge treats
the producer as unstable and aborts the batch instead of postponing the send
forever.

Current safety limits:

- maximum pending batch duration defaults to `1000 ms` and is controlled by `CONFIG_ACTUATOR_COMMAND_MAX_PENDING_MS`
- maximum accepted mutations in one batch defaults to `32` and is controlled by `CONFIG_ACTUATOR_COMMAND_MAX_MUTATION_COUNT`

When either limit is exceeded:

- the pending batch is dropped
- the desired-state shadow is realigned with the last authoritative controller
  state
- no `SET_STATE` frame is sent for that unstable batch
- a bridge-local diagnostic is queued for publication on the MQTT `bridge` topic

## MQTT inbound queue behavior

The AsyncMqttClient callback does not parse and route commands immediately.
It only validates complete frames and copies them into a small bridge-side
queue. The main loop later drains that queue with a bounded fairness policy:
one command per loop even while serial RX is pending, or a larger bounded batch
when the UART is idle.

Current behavior:

- queue capacity defaults to `8` complete MQTT frames and is controlled by `CONFIG_MQTT_COMMAND_QUEUE_CAPACITY`
- only complete non-fragmented frames are accepted
- retained commands are ignored
- queue overflow drops the new command instead of blocking the callback

Queue overflow is treated as a producer/backpressure problem and is reported as
a bridge-local diagnostic on the MQTT `bridge` topic.

## Optional serial MsgPack framing

When `CONFIG_MSG_PACK_ARDUINO` is enabled, serial MsgPack always uses a framed
transport. The transport keeps the LSH payload itself unchanged. It only wraps the
serial MsgPack payload in a SLIP-like delimiter-and-escape layer:

- each frame starts with `0xC0`
- the payload escapes `0xC0` and `0xDB`
- each frame ends with `0xC0`

This transport layer is local to the controller serial link:

- MQTT payloads are never framed
- ArduinoJson sees only the deframed pure MsgPack payload
- raw-forwarded serial MsgPack payloads are framed only when they cross the
  controller UART

Serial MsgPack is wire-compatible only with a peer that uses the same framed
transport.

## Controller bootstrap and topology cache

The bridge treats controller bootstrap as a non-blocking background flow.
If a validated `DEVICE_DETAILS` snapshot is available in ESP32 NVS, startup
uses that cached topology immediately to rebuild MQTT topics and Homie nodes.

Current behavior:

- only controller `DEVICE_DETAILS` are cached in NVS
- actuator runtime `STATE` is never persisted; the controller remains the only
  authoritative source of state
- after startup the bridge enters `waiting_details` and slowly retries
  `REQUEST_DETAILS` until the controller confirms its topology
- once confirmed, the bridge enters `waiting_state` and slowly retries
  `REQUEST_STATE` until a fresh authoritative state arrives
- bootstrap retries keep running even while the controller is currently silent,
  so a freshly powered-on or freshly rebooted core can be discovered without
  pretending the serial link is already alive
- the bridge is considered fully synchronized only after both phases complete
- while the bridge is not synchronized, device-level `SET_*` writes are
  rejected instead of being forwarded against a stale model

If no valid cache is present yet:

- Homie starts, so OTA and bridge management remain available
- device-scoped LSH topics are not published yet, because the bridge refuses to
  invent a controller identity before `DEVICE_DETAILS` are known
- service-topic commands are accepted, but the bridge cannot emit a
  bridge-topic `service_ping_reply` until the first validated topology has
  been cached and one controlled reboot has rebuilt the runtime around it

If the controller later reports a different topology:

- the bridge stops trusting runtime state immediately
- the new topology is saved to NVS from the main loop, never from the serial
  callback
- after the save succeeds, the bridge performs one controlled reboot
- on the next boot, MQTT topics and Homie nodes are rebuilt from the new cached
  topology

This keeps the controller authoritative while keeping bootstrap work in the
main loop.

## MQTT topic split: `events` vs `bridge`

The bridge keeps controller-backed traffic and bridge-local runtime traffic
strictly separate:

- `LSH/<device>/events`: controller-backed runtime events such as
  `NETWORK_CLICK_*` payloads and device-level `PING` replies
- `LSH/<device>/bridge`: bridge-local runtime events such as service-level
  ping replies and diagnostics

This keeps Node-RED and other consumers from mistaking a live MQTT bridge for a
live downstream controller.

Device-level `PING` is answered on `events` only when both conditions are true:

- the bridge considers the controller link connected
- the bridge runtime cache is synchronized with the controller

Service-topic `PING` is answered on `bridge`, never on `events`, whenever the
bridge already has a valid device-scoped `bridge` topic. Before the first
validated topology snapshot is cached, the service command is still accepted
but there is no authoritative device identity to publish under.
Its `controller_connected` field becomes `true` only after the bridge has
decoded at least one valid controller frame in the current boot session and
that frame is inside the configured liveness timeout.

## Homie setup mode and `HOMIE_RESET`

Homie already enters configuration/setup mode automatically when its stored
configuration is invalid or missing. `lsh-bridge` uses that behavior directly.

Current behavior:

- on a brand-new or already-reset device, `Homie.setup()` enters setup/AP mode
  by itself when the configuration is missing
- when `HOMIE_RESET` is defined, the bridge calls `Homie.reset()` only if a
  valid stored Homie configuration is actually present
- this keeps the "force setup on next boot" workflow intact without creating a
  reset loop on unconfigured devices

## Bridge-local diagnostics on `bridge`

These diagnostics are emitted from the main loop, not directly from callbacks,
so the bridge never tries to publish MQTT while already inside the MQTT
client's message callback.

Current diagnostic kinds:

- `actuator_command_storm_dropped`
- `mqtt_queue_overflow`
- `mqtt_command_rejected`
- `topology_changed`

Logical payload shape for `actuator_command_storm_dropped`:

```json
{
  "event": "diagnostic",
  "kind": "actuator_command_storm_dropped",
  "pending_ms": 1000,
  "mutation_count": 32
}
```

Meaning:

- `pending_ms`: how long the unstable actuator batch stayed open before the
  bridge aborted it
- `mutation_count`: how many accepted command changes were merged into that
  dropped batch

Logical payload shape for `mqtt_queue_overflow`:

```json
{
  "event": "diagnostic",
  "kind": "mqtt_queue_overflow",
  "dropped_device_commands": 7,
  "dropped_service_commands": 2
}
```

Meaning:

- `dropped_device_commands`: number of device-topic commands dropped because
  the inbound queue was full
- `dropped_service_commands`: number of service-topic commands dropped because
  the inbound queue was full

Logical payload shape for `mqtt_command_rejected`:

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

Meaning:

- `rejected_retained_commands`: number of retained MQTT commands rejected by
  policy because replaying them after reconnect would be unsafe
- `rejected_oversize_commands`: number of MQTT commands rejected before
  enqueue because they exceeded the fixed inbound command buffer
- `rejected_fragmented_commands`: number of MQTT commands rejected because the
  bridge only accepts complete non-fragmented inbound frames
- `rejected_malformed_commands`: number of MQTT commands whose delivery shape
  was acceptable but whose payload was not valid JSON/MsgPack for the active
  codec or did not contain a valid `p` command id

Notes:

- if MQTT is configured for MessagePack, the wire encoding is MessagePack; the
  field names and logical structure above apply unchanged
- queue overflow and command-rejection diagnostics are aggregated before
  publish, so the bridge does not emit one MQTT diagnostic for every single
  rejected or dropped frame
- diagnostics belong to the current MQTT session only; pending bridge-local
  diagnostics are cleared when MQTT disconnects so stale warnings are not
  replayed after reconnect

Logical payload shape for a service-topic `PING` reply:

```json
{
  "event": "service_ping_reply",
  "controller_connected": true,
  "runtime_synchronized": true,
  "bootstrap_phase": "synced"
}
```

Meaning:

- `controller_connected`: whether the bridge currently sees the downstream
  controller link as alive
- `runtime_synchronized`: whether the bridge runtime cache is synchronized with
  the controller and can safely answer controller-backed MQTT probes
- `bootstrap_phase`: high-level bridge sync phase. Current values are
  `waiting_details`, `waiting_state`, `synced` and
  `topology_migration_pending_reboot`

## What Is Not A Stable Runtime Contract

The bridge keeps some behavior intentionally outside the stable public runtime
contract:

- the diagnostic payload field names and shapes
- when pending diagnostics are aggregated, cleared or reset
- low-level implementation details such as queue indexing, critical-section
  usage and document pool sizing

## Read Next

- For exact `CONFIG_*` ownership: [compile-time-configuration.md](./compile-time-configuration.md)
- For the bridge overview and bundled example: [../README.md](../README.md)
- For the public stack semantics around `BOOT`, `PING` and startup repair: <https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md>
- For symptom-based first-lab diagnosis: <https://github.com/labodj/labo-smart-home/blob/main/TROUBLESHOOTING.md>
