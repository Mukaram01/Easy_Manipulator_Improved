#!/usr/bin/env python3
"""Ensure the committed Workcell Studio Web3D bundle matches its sources."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def _run(command: list[str], viewer_root: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, cwd=viewer_root, text=True, check=False)


def _require_tool(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(
            f"Required Web3D build tool '{name}' was not found on PATH. "
            "Install Node.js and npm, then rebuild workcell_builder."
        )


def ensure_bundle(viewer_root: Path, stamp: Path | None = None) -> bool:
    """Return True when a stale bundle was rebuilt, otherwise False."""
    _require_tool("node")
    _require_tool("npm")

    lock_id = hashlib.sha256(str(viewer_root.resolve()).encode()).hexdigest()[:16]
    lock_path = Path(tempfile.gettempdir()) / f"workcell-web3d-bundle-{lock_id}.lock"
    with lock_path.open("w", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file, fcntl.LOCK_EX)

        # npm ls catches an absent, incomplete, or lockfile-inconsistent install.
        dependencies = _run(["npm", "ls", "--ignore-scripts", "--depth=0"], viewer_root)
        if dependencies.returncode != 0:
            print("Web3D npm dependencies are unavailable or inconsistent; running npm ci.")
            installed = _run(["npm", "ci"], viewer_root)
            if installed.returncode != 0:
                raise RuntimeError("npm ci failed while preparing the Web3D bundle build.")

        current = _run(["npm", "run", "check:stale-bundle"], viewer_root)
        rebuilt = current.returncode != 0
        if rebuilt:
            print("Web3D production bundle is stale; rebuilding it now.")
            build = _run(["npm", "run", "build:web3d"], viewer_root)
            if build.returncode != 0:
                raise RuntimeError("npm run build:web3d failed; the production bundle was not refreshed.")
            verified = _run(["npm", "run", "check:stale-bundle"], viewer_root)
            if verified.returncode != 0:
                raise RuntimeError("Web3D bundle remained stale after npm run build:web3d.")
        else:
            print("Web3D production bundle is already current; no rebuild needed.")

        if stamp is not None:
            stamp.parent.mkdir(parents=True, exist_ok=True)
            stamp.touch()
        return rebuilt


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--viewer-root",
        type=Path,
        default=repo_root / "workcell_studio_web" / "viewer",
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--stamp", type=Path, help="touch this build stamp after verification")
    args = parser.parse_args()
    try:
        ensure_bundle(args.viewer_root.resolve(), args.stamp)
    except RuntimeError as error:
        print(f"Web3D bundle freshness failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
