# Profiles and Roles

This document is intentionally written by hand.

The generated file [shared/lsh_protocol.md](../shared/lsh_protocol.md) is the
authoritative wire reference. This guide explains how to _reason_ about the
protocol so it stays easy to implement even outside the original LSH stack.

## Read This First

The base LSH protocol is:

- compact
- transport-agnostic
- peer-to-peer at the message level
- intentionally small in scope

It is **not** tied to a mandatory `core -> bridge -> Node-RED` topology.
That stack is only one valid profile.

An implementation may be:

- a controller speaking directly over MQTT
- a serial controller behind a bridge
- a bridge translating serial to MQTT
- an automation runtime consuming MQTT only
- a custom device or gateway written by a third party

## The Right Mental Model

Think in terms of **immediate peers**, not named products.

Every LSH message is exchanged between one peer and its immediate counterparty
on a specific transport. If a deployment has multiple hops, each hop decides how
to apply the protocol locally.

Examples:

- a direct networked controller and a client are one hop apart
- `lsh-core` and `lsh-bridge` are one hop apart on serial
- `lsh-bridge` and Node-RED are one hop apart on MQTT

The base protocol defines what a command means between immediate peers.
Whether a command is forwarded, translated, cached, collapsed, or answered
locally is a **profile decision**.

## Roles

The protocol does not require hard-coded product names, but most deployments
end up assigning one or more of these roles.

### Authoritative state peer

This peer owns the authoritative device topology and runtime actuator state.

Typical responsibilities:

- emits `DEVICE_DETAILS`
- emits `ACTUATORS_STATE`
- decides whether a topology/state snapshot is authoritative
- may emit `BOOT` when prior runtime assumptions must be discarded

### State consumer

This peer consumes `DEVICE_DETAILS` and `ACTUATORS_STATE` and may cache them.

Typical responsibilities:

- validates compatibility using `DEVICE_DETAILS.v`
- decides whether its local cache is still usable
- may request fresh snapshots with `REQUEST_DETAILS` or `REQUEST_STATE`

### Adapter / gateway

This peer sits between two transports or two protocol domains.

Typical responsibilities:

- forward unchanged payloads when possible
- translate payload encoding when required
- maintain a local projection or cache when useful
- decide which commands are handled locally and which are forwarded

The current `lsh-bridge` is one example of this role, but it is not the only
possible implementation.

### Orchestrator / automation peer

This peer issues requests or commands based on policy rather than direct I/O.

Typical responsibilities:

- request snapshots
- send actuator commands
- participate in the network-click handshake
- apply higher-level automation or fallback logic

## Command Semantics That Matter Most

### `DEVICE_DETAILS`

`DEVICE_DETAILS` is the authoritative topology snapshot for the current session.

Important implications:

- consumers must not invent topology details that were never announced
- consumers must reject incompatible snapshots using `DEVICE_DETAILS.v`
- topology is expected to be stable until a new authoritative announcement

### `ACTUATORS_STATE`

`ACTUATORS_STATE` is the authoritative runtime snapshot for the current session.

Caches or projections may exist, but they are always secondary to the latest
authoritative state actually received.

### `REQUEST_DETAILS` and `REQUEST_STATE`

These ask the peer for fresh authoritative snapshots.

The base contract does **not** require that the peer answering the request is
the ultimate hardware owner. A profile may allow an adapter to answer from a
validated cache if it can do so safely.

### `PING`

`PING` is **hop-local by default**.

That means:

- it probes the reachability of the immediate peer on the current transport
- it does not imply end-to-end health across multiple hops
- a multi-hop deployment may have one `PING` at MQTT level and another at serial level

If a profile wants stronger semantics, it must define them explicitly.

### `BOOT`

`BOOT` is **role-local by default**.

It means:

- the sender tells the receiving peer to discard runtime assumptions derived
  from the sender
- the receiving peer must re-synchronize before treating cached topology or
  cached state as authoritative again

The base protocol does **not** say that `BOOT` must be forwarded across all hops.
That behavior, if desired, belongs to a profile.

## Profiles

A profile is the missing layer between the raw protocol and a concrete product.

A good profile answers questions like:

- which peer is authoritative on this transport?
- which commands may be answered from cache?
- which commands are forwarded?
- which commands are handled locally?
- which commands are allowed on a service topic?
- what does a restart or reconnect do to cached state?

Without a profile, the wire contract alone is still valid, but multi-hop systems
become ambiguous.

## Reference LSH Stack

The current public LSH stack can be described like this:

- `lsh-core`: authoritative serial-side controller
- `lsh-bridge`: adapter between serial LSH and MQTT/Homie
- `node-red-contrib-lsh-logic`: orchestrator and automation peer on MQTT

In this reference stack:

- serial `PING` is used by the bridge to assess controller reachability
- MQTT device-topic `PING` is used by automation peers to assess the immediate
  MQTT peer only when the bridge has a live and synchronized controller path
- MQTT service-topic `PING` is used by automation peers to assess bridge
  reachability and returns bridge-local runtime status
- a bridge-local service-topic `BOOT` can trigger a controller resync without
  redefining `BOOT` as a mandatory end-to-end traversal command

In the current MQTT profile, the topic split is explicit:

- `LSH/<device>/events` carries controller-backed runtime traffic
- `LSH/<device>/bridge` carries bridge-local runtime traffic

That topic split is a profile choice. The base wire contract remains unchanged.

Those are **profile choices**, not mandatory rules for all LSH implementations.

## Direct Implementation Without a Bridge

A third-party implementation can speak LSH directly without reproducing the
reference stack.

For example, a networked controller could:

- expose `DEVICE_DETAILS` and `ACTUATORS_STATE` directly on MQTT
- answer `REQUEST_DETAILS` and `REQUEST_STATE` itself
- handle `PING` directly as the authoritative device
- interpret `BOOT` as a local resync trigger or restart notification

That implementation would still be compliant if it respects:

- command IDs
- payload shapes
- compatibility rules
- hop-local semantics

## Implementation Advice

If you want a robust implementation, keep these rules in mind:

- treat `DEVICE_DETAILS` as the topology authority
- treat `ACTUATORS_STATE` as the runtime state authority
- keep `PING` hop-local unless you have a clearly documented reason not to
- keep `BOOT` as a resync signal, not as a version negotiation mechanism
- document your profile whenever you introduce caching, forwarding, or local handling
- do not rely on product names like `core` or `bridge` when defining the base semantics

## Practical Reading Order

When implementing LSH from scratch:

1. read [shared/lsh_protocol.md](../shared/lsh_protocol.md)
2. understand the roles in this document
3. define your own profile before writing code
4. only then map the protocol onto your chosen transports and cache behavior
