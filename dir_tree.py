#!/usr/bin/env python3
"""dir_tree.py — Print a pretty directory tree, excluding *build* directories.

Usage
-----
$ python dir_tree.py [START_PATH]

If START_PATH is omitted, the current working directory (".") is used.
"""

import os
import sys
from pathlib import Path

# Directories to skip (case-insensitive match)
EXCLUDE_DIRS = {"build"}


def walk_dir(directory: Path, prefix: str = ""):
    """Recursively print a directory tree with unicode branches."""
    entries = sorted(directory.iterdir(), key=lambda p: (not p.is_dir(), p.name.lower()))
    # Filter out excluded directories
    entries = [e for e in entries if not (e.is_dir() and e.name.lower() in EXCLUDE_DIRS)]
    last_index = len(entries) - 1

    for index, entry in enumerate(entries):
        connector = "└── " if index == last_index else "├── "
        print(f"{prefix}{connector}{entry.name}")

        if entry.is_dir():
            extension = "    " if index == last_index else "│   "
            walk_dir(entry, prefix + extension)


def main():
    start_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.cwd()
    start_path = start_path.resolve()
    print(start_path)
    walk_dir(start_path)


if __name__ == "__main__":
    main()
