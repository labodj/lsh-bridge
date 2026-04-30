# Profiles and Roles

This page is written by hand because the generated protocol reference cannot
answer every design question.

[shared/lsh_protocol.md](../shared/lsh_protocol.md) is the authoritative source
for exact wire keys, command IDs, payload shapes, and golden examples. This
guide explains how to think about those messages when you build a real system:
which peer owns state, what is local to one hop, what can be cached, and what a
bridge or automation runtime is allowed to decide for itself.

If you are implementing LSH outside the original stack, read this page before
mapping the protocol onto your own transport.

## The Short Version

The base LSH protocol is compact and deliberately small. It defines messages,
not a mandatory product topology.

It does not require this shape:

```text
lsh-core -> lsh-bridge -> Node-RED
```

That is one public profile, not the protocol itself.

An LSH implementation may be:

- a networked controller that speaks MQTT directly
- a serial controller behind a bridge
- a bridge that translates serial frames into MQTT payloads
- a standalone coordinator consuming MQTT
- a Node-RED wrapper around a standalone coordinator
- a third-party gateway with its own transport and caching policy

As long as the command IDs, payload shapes, compatibility rules, and documented
semantics are respected, the implementation can still be a valid LSH peer.

## Think In Immediate Peers

The most useful mental model is this:

> Every LSH message is exchanged between two immediate peers on one transport.

If a deployment has multiple hops, each hop has its own local relationship.

Examples:

- a direct MQTT controller and a coordinator are immediate peers
- `lsh-core` and `lsh-bridge` are immediate peers on serial
- `lsh-bridge` and an MQTT automation runtime are immediate peers on MQTT

The base protocol says what a command means on that immediate relationship. It
does not automatically say whether the command must be forwarded, answered from
cache, translated, or collapsed into another operation. Those choices belong to
a profile.

That distinction is what keeps the protocol reusable. A simple one-hop MQTT
device should not have to inherit every rule from a serial bridge deployment.

## Roles

The protocol does not hard-code product names. Instead, real deployments combine
these roles.

### Authoritative State Peer

This peer owns the current device topology and actuator state.

Typical responsibilities:

- emit `DEVICE_DETAILS`
- emit `ACTUATORS_STATE`
- decide what topology and state are authoritative for the current session
- emit `BOOT` when consumers must discard assumptions derived from this peer

In the public serial stack, this role is usually played by `lsh-core`.

### State Consumer

This peer consumes topology and state snapshots and may cache them.

Typical responsibilities:

- validate `DEVICE_DETAILS.v` before trusting a snapshot
- keep a local projection of topology and actuator state
- request fresh snapshots with `REQUEST_DETAILS` or `REQUEST_STATE`
- discard stale state after a relevant `BOOT`

`labo-smart-home-coordinator` is a state consumer when it listens to MQTT device
events and builds its runtime view.

### Adapter / Gateway

This peer connects two transports, two protocol domains, or two runtime
boundaries.

Typical responsibilities:

- forward unchanged payloads when that is safe
- translate JSON/MsgPack or serial/MQTT framing when needed
- maintain a validated cache if the profile allows local answers
- decide which commands are local and which commands must travel to another peer

`lsh-bridge` is the reference adapter between serial LSH and MQTT/Homie, but the
role is broader than that one implementation.

### Orchestrator / Automation Peer

This peer acts on policy. It does not necessarily own hardware state, but it
decides when to request state, send commands, or coordinate higher-level
behavior.

Typical responsibilities:

- request snapshots during startup or after invalidation
- send actuator commands
- participate in the network-click handshake
- expose other-actor commands to an application-specific output
- apply fallback or automation logic above the wire protocol

`labo-smart-home-coordinator` is the standalone public implementation of this
role. `node-red-contrib-lsh-logic` can wrap it for Node-RED users while keeping
the protocol and coordination logic in one reusable library.

## Commands That Need Care

Most LSH commands are intentionally simple. These are the ones where semantics
matter most in multi-hop deployments.

### `DEVICE_DETAILS`

`DEVICE_DETAILS` is the authoritative topology snapshot for the current session.

Consumers should not invent topology details that were never announced. They may
decorate or project the topology for a UI or automation engine, but the
underlying device model comes from the latest trusted `DEVICE_DETAILS`.

`DEVICE_DETAILS.v` is also where runtime wire compatibility is checked. If the
wire protocol major is not supported, reject the snapshot and do not continue as
if the peer were compatible.

### `ACTUATORS_STATE`

`ACTUATORS_STATE` is the authoritative runtime snapshot for actuator state.

A coordinator, bridge, or UI may keep a cache, but that cache is only a
projection of the latest accepted authoritative state. When a profile lets an
adapter answer from cache, the profile should explain when that cache is valid
and when it must be invalidated.

### `REQUEST_DETAILS` and `REQUEST_STATE`

These commands ask the immediate peer for fresh authoritative snapshots.

The peer answering the request does not always have to be the physical hardware
owner. A bridge or coordinator may answer from a validated cache if the profile
explicitly allows it and the cache is still trustworthy.

If you do not have a clearly documented cache policy, forward the request or ask
the authoritative peer.

### `PING`

`PING` is hop-local by default.

It checks reachability of the immediate peer on the current transport. It does
not automatically prove end-to-end health across a bridge, serial link, MQTT
broker, and automation runtime.

A multi-hop stack may reasonably have:

- one serial `PING` between controller and bridge
- one MQTT device-topic `PING` between bridge and coordinator
- one MQTT service-topic `PING` for bridge-local service health

If a profile wants stronger end-to-end semantics, it must document that choice.

### `BOOT`

`BOOT` is role-local by default.

It tells the receiving peer to discard runtime assumptions derived from the
sender and re-synchronize before trusting cached topology or cached state again.

The base protocol does not require `BOOT` to be forwarded across every hop. A
profile may forward it, translate it, or use a local service-topic `BOOT`, but
that behavior must be explicit.

## Profiles

A profile is the layer between the raw protocol and a concrete deployment.

A good profile answers practical questions:

- Which peer is authoritative on this transport?
- Which commands may be answered from cache?
- Which commands must be forwarded?
- Which commands are handled locally?
- Which topics or channels carry device-backed traffic?
- Which topics or channels carry service-local traffic?
- What happens to cached state after restart, reconnect, or `BOOT`?

Without a profile, the wire contract is still valid, but multi-hop behavior is
easy to misunderstand.

## Public LSH MQTT Profile

The current public LSH stack is one concrete profile:

- `lsh-core`: authoritative serial-side controller
- `lsh-bridge`: adapter between serial LSH and MQTT/Homie
- `labo-smart-home-coordinator`: standalone MQTT coordinator and automation peer
- `node-red-contrib-lsh-logic`: Node-RED wrapper around the coordinator role

For the full walkthrough of this public stack, including topic layout and
startup flow, read:

- <https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md>
- <https://github.com/labodj/labo-smart-home/blob/main/FAQ.md>
- <https://github.com/labodj/labo-smart-home/blob/main/TROUBLESHOOTING.md>

In this profile:

- serial `PING` is used by the bridge to assess controller reachability
- MQTT device-topic `PING` is used by automation peers to assess the immediate
  MQTT peer when the bridge has a live and synchronized controller path
- MQTT service-topic `PING` is used for bridge-local runtime status
- a bridge-local service-topic `BOOT` can trigger a controller resync without
  redefining `BOOT` as a mandatory end-to-end traversal command

The MQTT topic split is also explicit:

- `LSH/<device>/events` carries controller-backed runtime traffic
- `LSH/<device>/bridge` carries bridge-local runtime traffic

Those are profile choices. They are useful for the public stack, but they are
not mandatory rules for every future LSH implementation.

## Direct Implementation Without a Bridge

A third-party implementation can speak LSH directly.

For example, a networked controller could:

- publish `DEVICE_DETAILS` and `ACTUATORS_STATE` directly on MQTT
- answer `REQUEST_DETAILS` and `REQUEST_STATE` itself
- treat `PING` as direct device reachability
- use `BOOT` as a local restart or resync signal

That implementation does not need to reproduce the bridge profile. It only needs
to preserve the shared contract: command IDs, payload shapes, compatibility
rules, and hop-local semantics.

## Practical Implementation Advice

For robust integrations:

- Treat `DEVICE_DETAILS` as the topology authority.
- Treat `ACTUATORS_STATE` as the runtime state authority.
- Validate `DEVICE_DETAILS.v` before trusting a peer.
- Keep `PING` hop-local unless your profile clearly says otherwise.
- Keep `BOOT` as a resync signal, not as version negotiation.
- Document every cache and forwarding decision.
- Define roles by behavior, not product names.

## Suggested Reading Order

When implementing LSH from scratch:

1. Read [shared/lsh_protocol.md](../shared/lsh_protocol.md).
2. Read the roles and command notes in this document.
3. Write down your profile: peers, transports, authority, caching, forwarding.
4. Implement transport encoding and decoding.
5. Add compatibility and stale-cache tests before adding high-level behavior.

## Read Next

- Exact generated wire contract: [../shared/lsh_protocol.md](../shared/lsh_protocol.md)
- Protocol repo overview: [../README.md](../README.md)
- Public reference stack walkthrough: <https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md>
- Adoption FAQ: <https://github.com/labodj/labo-smart-home/blob/main/FAQ.md>
