#!/usr/bin/env python3
"""Update lsh-bridge generated protocol files from the standalone lsh-protocol repo."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
TARGET_NAME = "esp"
TARGET_ROOT_FLAG = "--esp-root"
VENDORED_PROTOCOL_ROOT = (REPO_ROOT / "vendor" / "lsh-protocol").resolve()


def validate_protocol_root(root: Path, source: str) -> Path:
    if not (root / "tools" / "generate_lsh_protocol.py").is_file():
        raise SystemExit(f"Invalid {source}: generator not found in {root}")
    return root


def resolve_protocol_root(cli_root: Path | None) -> Path:
    if cli_root is not None:
        return validate_protocol_root(cli_root.expanduser().resolve(), "--protocol-root")

    env_root = os.environ.get("LSH_PROTOCOL_ROOT")
    if env_root:
        return validate_protocol_root(Path(env_root).expanduser().resolve(), "LSH_PROTOCOL_ROOT")

    if (VENDORED_PROTOCOL_ROOT / "tools" / "generate_lsh_protocol.py").is_file():
        return VENDORED_PROTOCOL_ROOT

    raise SystemExit(
        "Vendored lsh-protocol not found at vendor/lsh-protocol. "
        "Run the documented git subtree workflow or override explicitly with "
        "--protocol-root / LSH_PROTOCOL_ROOT.",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if generated files are stale")
    parser.add_argument("--protocol-root", type=Path, help="path to the lsh-protocol repository")
    args = parser.parse_args()

    protocol_root = resolve_protocol_root(args.protocol_root)
    generator = protocol_root / "tools" / "generate_lsh_protocol.py"

    command = [
        sys.executable,
        str(generator),
        "--target",
        TARGET_NAME,
        TARGET_ROOT_FLAG,
        str(REPO_ROOT),
    ]
    if args.check:
        command.append("--check")

    completed = subprocess.run(command, check=False)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
