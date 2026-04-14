# LSH Protocol

Single source of truth for the LSH wire protocol.

This repository contains the compact protocol specification shared by:

- `lsh-core`
- `lsh-bridge`
- `node-red-contrib-lsh-logic`

It is responsible for keeping the protocol contract explicit, versioned, and reproducible across firmware and application repositories.

## Contents

- `shared/lsh_protocol.json`
  Compact protocol spec and metadata.
- `shared/lsh_protocol_golden_payloads.json`
  Golden examples used to validate generated artifacts and codec behavior.
- `shared/lsh_protocol.md`
  Human-readable reference generated from the shared spec.
- `tools/generate_lsh_protocol.py`
  Generator for shared protocol artifacts consumed by the LSH repositories.

## Scope

This repo is the source of truth for:

- wire command IDs
- compact JSON keys
- click type IDs
- protocol compatibility metadata
- golden payload examples
- generated protocol documentation

This repo is not responsible for:

- firmware logic
- MQTT bridging logic
- Node-RED business logic
- hardware configuration

## Design Goals

- compact on-wire representation
- deterministic generated artifacts
- explicit compatibility rules
- simple maintenance across separate git repositories
- no ambiguity about protocol ownership

## Trusted Environment

The LSH protocol assumes a trusted environment and a cooperative broker. It does not embed authentication, integrity, or confidentiality mechanisms in the payload format itself. That constraint is intentional and must stay documented in all consumer repositories.

## Compatibility Model

The compatibility agreement is intentionally minimal:

- `BOOT` is only a re-sync trigger. It does not carry version metadata.
- runtime wire compatibility is checked when the controller later sends `DEVICE_DETAILS`
- `DEVICE_DETAILS.v` must match the locally compiled `wireProtocolMajor`
- `specRevision` is repository/generation metadata only; it is not negotiated on wire

Runtime decision rule:

- if `DEVICE_DETAILS.v` matches, the peers may continue the handshake
- if `DEVICE_DETAILS.v` does not match, the consumer must reject the handshake payload

Keep these terms distinct:

- `wireProtocolMajor` decides whether two peers may speak to each other at runtime
- `specRevision` tracks source-of-truth evolution for generated code and documentation

## Transport Model

The LSH logical payloads are transport-agnostic. Transport behavior is documented here because it affects interoperability between repositories:

- JSON over serial: newline-delimited
- MsgPack over serial: raw MsgPack payload
- MQTT JSON: raw JSON payload
- MQTT MsgPack: raw MsgPack payload

## Workflow

1. edit the shared spec or golden payloads
2. run the generator for the targets you want to update
3. commit the updated generated artifacts
4. propagate changes to consumer repositories

## Consumer Integration

Each consumer repository vendors this repo at `vendor/lsh-protocol` via `git subtree`.

Initial add inside a consumer repository:

```bash
git remote add lsh-protocol git@github.com:labodj/lsh-protocol.git || git remote set-url lsh-protocol git@github.com:labodj/lsh-protocol.git
git fetch lsh-protocol
git subtree add --prefix=vendor/lsh-protocol lsh-protocol main --squash
```

Subsequent updates inside a consumer repository:

```bash
git remote add lsh-protocol git@github.com:labodj/lsh-protocol.git || git remote set-url lsh-protocol git@github.com:labodj/lsh-protocol.git
git fetch lsh-protocol
git subtree pull --prefix=vendor/lsh-protocol lsh-protocol main --squash
```

After updating the vendored copy, regenerate or verify the target-specific outputs from the consumer itself:

```bash
python3 tools/update_lsh_protocol.py
python3 tools/update_lsh_protocol.py --check
```

The consumer wrappers now default to the vendored subtree only. Local sibling repos are no longer auto-discovered. Use `--protocol-root` or `LSH_PROTOCOL_ROOT` only for explicit manual overrides.

## Generator Usage

Generate only the human-readable reference inside this repository:

```bash
python3 tools/generate_lsh_protocol.py
```

Check only:

```bash
python3 tools/generate_lsh_protocol.py --check
```

Generate outputs for consumer repositories explicitly:

```bash
python3 tools/generate_lsh_protocol.py \
  --target shared-doc \
  --target core \
  --target esp \
  --target node-red \
  --core-root /path/to/lsh-core \
  --esp-root /path/to/lsh-bridge \
  --node-red-root /path/to/node-red-contrib-lsh-logic
```

This repository is intentionally standalone:

- it does not assume a monorepo layout
- consumer outputs are emitted only when their target roots are passed explicitly
- `shared-doc` is the default target when no `--target` is provided

Typical maintainer flow from this repository:

```bash
python3 tools/generate_lsh_protocol.py
python3 tools/generate_lsh_protocol.py --check
python3 tools/generate_lsh_protocol.py \
  --target shared-doc \
  --target core \
  --target esp \
  --target node-red \
  --core-root /path/to/lsh-core \
  --esp-root /path/to/lsh-bridge \
  --node-red-root /path/to/node-red-contrib-lsh-logic
```

## Versioning

Keep these concepts distinct:

- `wireProtocolMajor`: runtime wire compatibility only
- `specRevision`: source-of-truth revision for generated code and docs
- git tags: repository release milestones
