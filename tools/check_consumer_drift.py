"""Verify each public consumer against one complete upstream protocol manifest."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

PROTOCOL = Path(__file__).resolve().parents[1]
MANIFEST = "shared/lsh_protocol_manifest.json"


def previous_manifest(revision):
    commits = subprocess.check_output(
        ["git", "log", "--first-parent", "--format=%H", "--", MANIFEST], cwd=PROTOCOL, text=True
    ).splitlines()
    for commit in commits:
        manifest = json.loads(subprocess.check_output(["git", "show", f"{commit}:{MANIFEST}"], cwd=PROTOCOL))
        if manifest["specRevision"] != revision:
            return manifest
    return None


def mismatches(workspace, consumer, manifest):
    vendor = workspace / consumer / "vendor/lsh-protocol"
    files = [(vendor / item["path"], item) for item in manifest["inputs"].values()]
    files.extend((workspace / item["path"], item) for item in manifest["consumerArtifacts"]
                 if item["consumer"] == consumer)
    return [str(path) for path, item in files
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=PROTOCOL.parent)
    parser.add_argument("--allow-previous", action="store_true")
    args = parser.parse_args()
    current = json.loads((PROTOCOL / MANIFEST).read_text())
    previous = previous_manifest(current["specRevision"]) if args.allow_previous else None
    consumers = sorted({item["consumer"] for item in current["consumerArtifacts"]})
    for consumer in consumers:
        errors = mismatches(args.workspace, consumer, current)
        if not errors:
            print(f"{consumer}: current revision {current['specRevision']}")
        elif previous and any(a["consumer"] == consumer for a in previous["consumerArtifacts"]) and not mismatches(args.workspace, consumer, previous):
            print(f"{consumer}: awaiting adoption, verified revision {previous['specRevision']}")
        else:
            raise SystemExit(f"{consumer}: protocol drift: {', '.join(errors)}")


if __name__ == "__main__":
    main()
