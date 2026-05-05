"""Check C/C++ formatting with the repository clang-format policy."""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

CPP_SUFFIXES = {".c", ".cc", ".cpp", ".h", ".hpp"}
EXCLUDED_ROOTS = {"vendor", "docs"}


def main() -> int:
    """Run clang-format in check-only mode for tracked C/C++ files."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--write",
        action="store_true",
        help="rewrite files in place instead of checking formatting",
    )
    args = parser.parse_args()

    files = _tracked_cpp_files()
    if not files:
        return 0
    mode_args = ["-i"] if args.write else ["--dry-run", "--Werror"]
    return subprocess.run(  # noqa: S603 - fixed tool, repository-local file list.
        [
            _require_executable("clang-format"),
            *mode_args,
            "--style=file",
            *map(str, files),
        ],
        check=False,
    ).returncode


def _tracked_cpp_files() -> list[Path]:
    completed = subprocess.run(  # noqa: S603 - fixed git command with no user input.
        [_require_executable("git"), "ls-files"],
        check=True,
        capture_output=True,
        text=True,
    )
    files: list[Path] = []
    for line in completed.stdout.splitlines():
        path = Path(line)
        if path.suffix not in CPP_SUFFIXES:
            continue
        if path.parts and path.parts[0] in EXCLUDED_ROOTS:
            continue
        if path.is_file():
            files.append(path)
    return files


def _require_executable(name: str) -> str:
    executable = shutil.which(name)
    if executable is None:
        message = f"{name} was not found on PATH"
        raise SystemExit(message)
    return executable


if __name__ == "__main__":
    raise SystemExit(main())
