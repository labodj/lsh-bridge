# lsh-bridge Documentation Map

This page is the navigation hub for `lsh-bridge`.

Use it when you know the kind of answer you need, but not which document to open first.
The README stays focused on the first-use path; the pages below hold the deeper runtime
and configuration details.

## Start Here

- **I am new to `lsh-bridge`**: read the
  [README](https://github.com/labodj/lsh-bridge/blob/main/README.md).
- **I want the first working build**: use
  [examples/basic-homie-bridge](https://github.com/labodj/lsh-bridge/tree/main/examples/basic-homie-bridge).
- **I need exact build flags**: read
  [docs/compile-time-configuration.md](https://github.com/labodj/lsh-bridge/blob/main/docs/compile-time-configuration.md).
- **I am debugging startup, synchronization or MQTT behavior**: read
  [docs/runtime-behavior.md](https://github.com/labodj/lsh-bridge/blob/main/docs/runtime-behavior.md).
- **I want the whole LSH stack first**: start from the
  [`labo-smart-home` documentation map](https://github.com/labodj/labo-smart-home/blob/main/DOCS.md).

## Common Paths

| Goal                          | Where to go                                                                                                                            |
| ----------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| Bring up a first ESP32 bridge | [README: First Build](https://github.com/labodj/lsh-bridge/blob/main/README.md#first-build)                                            |
| Embed the library             | [README: Embed the Library](https://github.com/labodj/lsh-bridge/blob/main/README.md#embed-the-library)                                |
| Tune capacities and topics    | [Compile-time configuration](https://github.com/labodj/lsh-bridge/blob/main/docs/compile-time-configuration.md)                        |
| Understand NVS topology cache | [Runtime behavior](https://github.com/labodj/lsh-bridge/blob/main/docs/runtime-behavior.md#controller-bootstrap-and-topology-cache)    |
| Diagnose MQTT command drops   | [Bridge-local diagnostics](https://github.com/labodj/lsh-bridge/blob/main/docs/runtime-behavior.md#bridge-local-diagnostics-on-bridge) |
| Review stack-level wiring     | [Labo Smart Home hardware overview](https://github.com/labodj/labo-smart-home/blob/main/HARDWARE_OVERVIEW.md)                          |

## Related Stack Docs

- [`labo-smart-home` documentation map](https://github.com/labodj/labo-smart-home/blob/main/DOCS.md)
- [Reference stack](https://github.com/labodj/labo-smart-home/blob/main/REFERENCE_STACK.md)
- [Getting started](https://github.com/labodj/labo-smart-home/blob/main/GETTING_STARTED.md)
- [Troubleshooting](https://github.com/labodj/labo-smart-home/blob/main/TROUBLESHOOTING.md)
- [Protocol roles and profiles](https://github.com/labodj/lsh-protocol/blob/main/docs/profiles-and-roles.md)
- [Canonical wire contract](https://github.com/labodj/lsh-protocol/blob/main/shared/lsh_protocol.md)

## Maintainer Notes

- PlatformIO package metadata lives in
  [library.json](https://github.com/labodj/lsh-bridge/blob/main/library.json).
- The package smoke test exports README, `DOCS.md` and `docs/*`; keep links in exported
  Markdown absolute so the PlatformIO Registry rendering stays valid.
- Verify the vendored protocol snapshot with
  `python3 tools/update_lsh_protocol.py --check`.
- Refresh the vendored protocol after changes in `lsh-protocol`:

```bash
git remote add lsh-protocol git@github.com:labodj/lsh-protocol.git || \
git remote set-url lsh-protocol git@github.com:labodj/lsh-protocol.git
git fetch lsh-protocol
git subtree pull --prefix=vendor/lsh-protocol lsh-protocol main --squash
python3 tools/update_lsh_protocol.py
python3 tools/update_lsh_protocol.py --check
```
