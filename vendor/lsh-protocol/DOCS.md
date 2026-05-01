# Documentation

This repository is the shared LSH protocol contract. It is intentionally smaller
than the product repositories: it defines the wire facts that every consumer must
read the same way.

Use this page as the documentation map. The README gives the overview; the
generated reference gives exact values; the profiles guide explains how those
values behave in real deployments.

## Start Here

Read these in order if you are new to the protocol:

1. [README](README.md) for the repository purpose, scope, compatibility model,
   and maintainer workflow.
2. [Profiles and roles](docs/profiles-and-roles.md) for peer responsibilities,
   hop-local behavior, cache boundaries, and public LSH MQTT profile semantics.
3. [Generated protocol reference](shared/lsh_protocol.md) when you need exact
   command IDs, compact JSON keys, payload examples, or generated static bytes.

That path is enough before touching a consumer repository.

## Common Tasks

- Check exact command IDs and wire keys:
  [generated protocol reference](shared/lsh_protocol.md#commands).
- Understand `BOOT`, `PING`, forwarding, and cache responsibility:
  [profiles and roles](docs/profiles-and-roles.md#commands-that-need-care).
- Update the shared contract:
  [maintainer flow](README.md#maintainer-flow).
- Vendor a released protocol copy into another project:
  [consumer integration](README.md#consumer-integration).
- Regenerate shared Markdown or consumer outputs:
  [generator usage](README.md#generator-usage).
- Understand the public LSH stack around this protocol:
  [LSH reference stack](https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md).

## Generated vs Hand-Written

`shared/lsh_protocol.md` is generated from `shared/lsh_protocol.json` and
`shared/lsh_protocol_golden_payloads.json`. Do not edit the generated Markdown
directly. Change the source data or the generator, then run:

```bash
python3 tools/generate_lsh_protocol.py
python3 tools/generate_lsh_protocol.py --check
```

`docs/profiles-and-roles.md` is hand-written. Use it for explanations that do
not belong in the wire table: roles, authority, forwarding, caching, and profile
semantics.

## Project Scope

This repo owns:

- wire command IDs;
- compact JSON keys;
- click type IDs;
- wire compatibility metadata;
- golden payload examples;
- generated protocol documentation;
- role-neutral guidance for implementers.

It does not own firmware policy, bridge policy, Node-RED behavior, Home
Assistant projection, Homie projection, or physical device configuration. Those
decisions live in the repositories that implement the protocol.

## Editing Rule of Thumb

If a change affects the payload shape or command meaning, treat it as a
compatibility change and update consumers in a coordinated way.

If a change only improves generated docs or generated code quality, keep the
wire contract stable and prove that the generator still reproduces the expected
files.
