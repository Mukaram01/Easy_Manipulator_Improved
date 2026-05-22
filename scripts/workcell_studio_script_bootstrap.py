#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path


def ensure_repo_root_on_sys_path(current_file: str | Path) -> Path:
    current_path = Path(current_file).resolve()
    repo_root = current_path.parents[1]
    repo_root_str = str(repo_root)
    if repo_root_str not in sys.path:
        sys.path.insert(0, repo_root_str)
    return repo_root
