# LSH Protocol

[![Build Status](https://github.com/labodj/lsh-protocol/actions/workflows/ci.yml/badge.svg)](https://github.com/labodj/lsh-protocol/actions/workflows/ci.yml)
[![Latest Release](https://img.shields.io/github/v/release/labodj/lsh-protocol?display_name=tag&sort=semver)](https://github.com/labodj/lsh-protocol/releases/latest)

`lsh-protocol` is the small, shared contract that keeps the LSH ecosystem
speaking the same language.

It does not contain firmware logic, Node-RED flows, automation rules, or device
configuration. It contains the part that must never become a matter of opinion:
wire keys, command IDs, click IDs, golden payloads, compatibility metadata, and
the generator that turns those definitions into code for the public LSH
projects.

The `main` branch can move ahead while coordinated work is in progress. If you
are building outside the public LSH repositories, vendor a tagged release so the
protocol copy in your project stays reproducible.

## Where This Fits

The protocol is shared by these repositories:

- `lsh-core`: the firmware-side controller implementation
- `lsh-bridge`: the serial-to-MQTT bridge implementation
- `labo-smart-home-coordinator`: the standalone automation coordinator
- `node-red-contrib-lsh-logic`: the Node-RED package built around the coordinator

The current public stack is documented from the user-facing side here:

- [LSH reference stack](https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md)
- [LSH glossary](https://github.com/labodj/labo-smart-home/blob/main/GLOSSARY.md)
- [LSH FAQ](https://github.com/labodj/labo-smart-home/blob/main/FAQ.md)

This repository stays lower level. It explains the contract and gives maintainers
one place to update it without hand-editing each consumer.

## Start Here

Choose the document that matches the question in front of you:

- Need exact payload shapes, numeric command IDs, compact JSON keys, or golden
  examples? Read [shared/lsh_protocol.md](./shared/lsh_protocol.md).
- Need to understand what `BOOT`, `PING`, roles, and hop-local behavior actually
  mean? Read [docs/profiles-and-roles.md](./docs/profiles-and-roles.md).
- Need to update generated files in a consumer repository? Jump to
  [Consumer Integration](#consumer-integration).
- Need the practical public MQTT/Homie/Node-RED profile? Start from the
  [reference stack](https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md).

## Repository Contents

- `shared/lsh_protocol.json`
  The compact protocol specification. Edit this when the wire contract changes.
- `shared/lsh_protocol_golden_payloads.json`
  Human-readable golden examples used by tests, generated docs, and consumers.
- `shared/lsh_protocol.md`
  Generated reference documentation. Do not edit it by hand.
- `docs/profiles-and-roles.md`
  Hand-written guidance for implementers. This is where the protocol semantics
  are explained in normal language.
- `tools/generate_lsh_protocol.py`
  The generator used by this repository and by vendored consumer copies.

## Scope

This repo owns:

- wire command IDs
- compact JSON keys
- click type IDs
- protocol compatibility metadata
- golden payload examples
- generated protocol documentation
- role-neutral guidance for implementers

This repo deliberately does not own:

- firmware behavior
- bridge policy
- Node-RED business logic
- Home Assistant or Homie projection
- physical device configuration

That separation matters. A protocol repository should be boring, stable, and
hard to misread. Product behavior belongs in the product repositories.

## Compatibility Model

The compatibility agreement is intentionally simple.

`BOOT` is a re-sync signal. It does not negotiate protocol versions. Runtime
compatibility is checked later, when a peer receives `DEVICE_DETAILS`.

`DEVICE_DETAILS.v` carries the wire protocol major. If it matches the locally
compiled `wireProtocolMajor`, the peers may continue the handshake. If it does
not match, the payload must be rejected.

Keep these two values separate:

- `wireProtocolMajor` decides runtime wire compatibility.
- `specRevision` tracks the source-of-truth revision used to generate docs and
  code.

`specRevision` is useful for humans, CI, and vendoring checks. It is not a wire
negotiation mechanism.

## Transport Model

LSH logical payloads are transport-agnostic. The same command can travel over
serial, MQTT, or another profile-defined transport as long as the payload shape
stays valid.

The current shared transport rules are:

- JSON over serial: newline-delimited JSON
- MsgPack over serial: `END + escaped(payload) + END`, with `END = 0xC0`,
  `ESC = 0xDB`, `ESC_END = 0xDC`, and `ESC_ESC = 0xDD`
- MQTT JSON: raw JSON payload
- MQTT MsgPack: raw MsgPack payload

The base protocol does not decide how many hops your deployment has, whether a
bridge exists, whether commands are forwarded, or how state is projected into
Homie or Home Assistant. Those are profile decisions.

## Trusted Environment

The LSH payload format assumes a trusted environment and a cooperative broker.
It does not provide authentication, encryption, replay protection, or integrity
checks inside the payload itself.

Use the security mechanisms of the transport and deployment: MQTT users and ACLs,
TLS, network isolation, serial trust boundaries, and operating-system controls.

## Consumer Integration

Each consumer repository vendors this repo at `vendor/lsh-protocol` through
`git subtree`.

For stable third-party integrations, vendor a released tag:

```bash
git remote add lsh-protocol git@github.com:labodj/lsh-protocol.git || git remote set-url lsh-protocol git@github.com:labodj/lsh-protocol.git
git fetch lsh-protocol
git subtree add --prefix=vendor/lsh-protocol lsh-protocol <tag> --squash
```

To update an existing vendored copy:

```bash
git remote add lsh-protocol git@github.com:labodj/lsh-protocol.git || git remote set-url lsh-protocol git@github.com:labodj/lsh-protocol.git
git fetch lsh-protocol
git subtree pull --prefix=vendor/lsh-protocol lsh-protocol <tag> --squash
```

Use `main` only when you are intentionally coordinating unreleased protocol work
across multiple LSH repositories:

```bash
git remote add lsh-protocol git@github.com:labodj/lsh-protocol.git || git remote set-url lsh-protocol git@github.com:labodj/lsh-protocol.git
git fetch lsh-protocol
git subtree pull --prefix=vendor/lsh-protocol lsh-protocol main --squash
```

After updating the vendored copy, regenerate or verify the target-specific files
from the consumer repository:

```bash
python3 tools/update_lsh_protocol.py
python3 tools/update_lsh_protocol.py --check
```

Consumer wrappers should default to the vendored subtree. Use `--protocol-root`
or `LSH_PROTOCOL_ROOT` only when you deliberately want to test against a local
protocol checkout.

## Generator Usage

Generate the shared Markdown reference in this repository:

```bash
python3 tools/generate_lsh_protocol.py
```

Check that generated files are up to date:

```bash
python3 tools/generate_lsh_protocol.py --check
```

Generate outputs for the public consumers:

```bash
python3 tools/generate_lsh_protocol.py \
  --target shared-doc \
  --target core \
  --target bridge \
  --target coordinator \
  --target node-red \
  --core-root /path/to/lsh-core \
  --bridge-root /path/to/lsh-bridge \
  --coordinator-root /path/to/labo-smart-home-coordinator \
  --node-red-root /path/to/node-red-contrib-lsh-logic
```

Target meanings:

- `shared-doc` writes the generated Markdown reference.
- `core` writes C++ headers for `lsh-core`.
- `bridge` writes C++ headers for `lsh-bridge`.
- `coordinator` writes TypeScript protocol constants for the standalone
  coordinator package.
- `node-red` writes TypeScript protocol constants for packages that consume the
  protocol directly from a Node-RED repository.

The `node-red` target is retained for direct Node-RED package consumption. The
preferred direction for new LSH automation work is to keep protocol logic inside
`labo-smart-home-coordinator` and let Node-RED wrap that library.

## Maintainer Flow

When the wire contract changes:

1. Edit `shared/lsh_protocol.json`.
2. Update `shared/lsh_protocol_golden_payloads.json` if examples changed.
3. Run `python3 tools/generate_lsh_protocol.py`.
4. Run `python3 tools/generate_lsh_protocol.py --check`.
5. Propagate the vendored protocol copy into consumer repositories.
6. Run each consumer's protocol update/check command.
7. Commit the spec, generated docs, and consumer-generated outputs together in
   the appropriate repositories.

## Versioning

Use these concepts precisely:

- `wireProtocolMajor`: runtime wire compatibility
- `specRevision`: source-of-truth revision for generated code and documentation
- git tags: released protocol milestones

Small documentation or generator-quality changes do not necessarily imply a new
wire protocol major. Any payload shape or command meaning change must be handled
with the same care as a firmware/API compatibility change.
