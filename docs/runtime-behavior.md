# Runtime Behavior

`lsh-bridge` has a few important runtime behaviors that matter for anyone
integrating, debugging or operating the bridge in the field.

Some of the key thresholds below are now exposed as compile-time `CONFIG_*`
macros. This page documents the behavior itself; the complete build-time knob
reference lives in [`compile-time-configuration.md`](./compile-time-configuration.md).

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
- a bridge-local diagnostic is queued for publication on the MQTT `misc` topic

## MQTT inbound queue behavior

The AsyncMqttClient callback does not parse and route commands immediately.
It only validates complete frames and copies them into a small bridge-side
queue. The main loop later drains that queue when the serial path is idle.

Current behavior:

- queue capacity defaults to `8` complete MQTT frames and is controlled by `CONFIG_MQTT_COMMAND_QUEUE_CAPACITY`
- only complete non-fragmented frames are accepted
- retained commands are ignored
- queue overflow drops the new command instead of blocking the callback

Queue overflow is treated as a producer/backpressure problem and is reported as
a bridge-local diagnostic on the MQTT `misc` topic.

## Bridge-local diagnostics on `misc`

The bridge publishes its own runtime diagnostics on the same MQTT `misc` topic
used for bridge-side events.

These diagnostics are emitted from the main loop, not directly from callbacks,
so the bridge never tries to publish MQTT while already inside the MQTT
client's message callback.

Current diagnostic kinds:

- `actuator_command_storm_dropped`
- `mqtt_queue_overflow`

Logical payload shape for `actuator_command_storm_dropped`:

```json
{
  "bridge_diagnostic": "actuator_command_storm_dropped",
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
  "bridge_diagnostic": "mqtt_queue_overflow",
  "dropped_device_commands": 7,
  "dropped_service_commands": 2
}
```

Meaning:

- `dropped_device_commands`: number of device-topic commands dropped because
  the inbound queue was full
- `dropped_service_commands`: number of service-topic commands dropped because
  the inbound queue was full

Notes:

- if MQTT is configured for MessagePack, the wire encoding is MessagePack; the
  field names and logical structure above still apply
- queue overflow diagnostics are aggregated before publish, so the bridge does
  not emit one MQTT diagnostic for every single dropped frame
- diagnostics belong to the current MQTT session only; pending bridge-local
  diagnostics are cleared when MQTT disconnects so stale warnings are not
  replayed after reconnect

## What remains internal

The bridge still keeps some behavior intentionally internal:

- the diagnostic payload field names and shapes
- when pending diagnostics are aggregated, cleared or reset
- low-level implementation details such as queue indexing, critical-section
  usage and document pool sizing
