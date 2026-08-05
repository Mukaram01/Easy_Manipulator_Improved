#!/usr/bin/env python3
"""Coalesce Workcell Studio Product View preparation by scene revision.

The Qt Product View already coalesces duplicate in-memory requests and retires
superseded preparation/navigation tokens. This module adds the disk-side half of
that contract: one process may prepare a scene/output at a time, unchanged
source revisions reuse the last validated result without rediscovering staged
assets, and a source change that lands during preparation replaces the older
attempt before a result is published.
"""

from __future__ import annotations

import contextlib
import hashlib
import io
import json
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

try:
    import fcntl
except ImportError:  # pragma: no cover - Workcell Studio supports Linux/ROS.
    fcntl = None

CACHE_SCHEMA = "workcell_studio_scene_refresh_cache/v1"
DEFAULT_LOCK_TIMEOUT_S = 120.0


def _option_value(argv: Sequence[str], name: str) -> Optional[str]:
    for index, token in enumerate(argv):
        if token == name:
            return argv[index + 1] if index + 1 < len(argv) else ""
        prefix = name + "="
        if token.startswith(prefix):
            return token[len(prefix):]
    return None


def _has_flag(argv: Sequence[str], name: str) -> bool:
    return name in argv


def _fingerprint(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return "sha256:" + digest.hexdigest()


def source_revision(impl: Any, scene_dir: Path) -> str:
    """Hash the canonical scene and generator inputs in deterministic order."""
    digest = hashlib.sha256()
    paths = sorted({Path(path).resolve() for path in impl.existing_inputs(scene_dir)}, key=str)
    for path in paths:
        digest.update(str(path).encode("utf-8"))
        digest.update(b"\0")
        if not path.is_file():
            digest.update(b"missing\0")
            continue
        digest.update(str(path.stat().st_size).encode("ascii"))
        digest.update(b"\0")
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        digest.update(b"\0")
    return "sha256:" + digest.hexdigest()


def preparation_key(scene_id: str, output_path: Path, revision: str, stage_assets: bool, diagnostic: bool) -> str:
    raw = "\0".join(
        (
            scene_id,
            str(output_path.resolve()),
            revision,
            "stage-assets" if stage_assets else "no-stage-assets",
            "diagnostic" if diagnostic else "strict",
        )
    )
    return "sha256:" + hashlib.sha256(raw.encode("utf-8")).hexdigest()


def _cache_path(output_path: Path, scene_id: str, stage_assets: bool, diagnostic: bool) -> Path:
    variant = f"{scene_id}.{'assets' if stage_assets else 'json'}.{'diagnostic' if diagnostic else 'strict'}"
    safe = hashlib.sha256((variant + "\0" + str(output_path.resolve())).encode("utf-8")).hexdigest()[:20]
    return output_path.parent / ".scene_refresh_cache" / f"{scene_id}.{safe}.json"


def _lock_path(output_path: Path, scene_id: str) -> Path:
    safe = hashlib.sha256((scene_id + "\0" + str(output_path.resolve())).encode("utf-8")).hexdigest()[:20]
    return output_path.parent / ".scene_refresh_cache" / f"{scene_id}.{safe}.lock"


def _read_json(path: Path) -> Optional[dict[str, Any]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


@contextlib.contextmanager
def preparation_lock(lock_path: Path, timeout_s: float = DEFAULT_LOCK_TIMEOUT_S):
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+", encoding="utf-8") as handle:
        if fcntl is None:
            yield 0
            return
        started = time.monotonic()
        while True:
            try:
                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() - started >= timeout_s:
                    raise TimeoutError(f"timed out waiting for Product View preparation lock: {lock_path}")
                time.sleep(0.05)
        waited_ms = int((time.monotonic() - started) * 1000)
        try:
            handle.seek(0)
            handle.truncate()
            handle.write(f"pid={os.getpid()} acquired_unix_ns={time.time_ns()}\n")
            handle.flush()
            yield waited_ms
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def _artifact_snapshot(impl: Any, scene_id: str, scene_dir: Path, output_path: Path, stage_assets: bool) -> dict[str, Any]:
    mesh_index = scene_dir / impl.MESH_INDEX_REL
    asset_dir = Path(impl.ASSET_BUILD_ROOT) / scene_id
    snapshot: dict[str, Any] = {
        "output_path": str(output_path.resolve()),
        "output_fingerprint": _fingerprint(output_path) if output_path.is_file() else "",
        "mesh_index_path": str(mesh_index.resolve()),
        "mesh_index_fingerprint": _fingerprint(mesh_index) if mesh_index.is_file() else "",
        "stage_assets": bool(stage_assets),
    }
    if stage_assets:
        snapshot.update(
            {
                "asset_dir": str(asset_dir.resolve()),
                "asset_dir_exists": asset_dir.is_dir(),
                "asset_dir_mtime_ns": asset_dir.stat().st_mtime_ns if asset_dir.is_dir() else 0,
            }
        )
    return snapshot


def _snapshot_matches(impl: Any, scene_id: str, scene_dir: Path, output_path: Path, stage_assets: bool, cached: dict[str, Any]) -> bool:
    artifacts = cached.get("artifacts")
    if not isinstance(artifacts, dict):
        return False
    current = _artifact_snapshot(impl, scene_id, scene_dir, output_path, stage_assets)
    for key in ("output_path", "output_fingerprint", "mesh_index_path", "mesh_index_fingerprint", "stage_assets"):
        if current.get(key) != artifacts.get(key):
            return False
    if stage_assets:
        for key in ("asset_dir", "asset_dir_exists", "asset_dir_mtime_ns"):
            if current.get(key) != artifacts.get(key):
                return False
    return bool(current.get("output_fingerprint") and current.get("mesh_index_fingerprint"))


def _cached_result(
    impl: Any,
    cache_path: Path,
    scene_id: str,
    scene_dir: Path,
    output_path: Path,
    revision: str,
    stage_assets: bool,
    diagnostic: bool,
) -> Optional[dict[str, Any]]:
    cached = _read_json(cache_path)
    if not cached or cached.get("schema_version") != CACHE_SCHEMA:
        return None
    if cached.get("scene_id") != scene_id or cached.get("source_revision") != revision:
        return None
    if bool(cached.get("diagnostic")) != diagnostic:
        return None
    if not _snapshot_matches(impl, scene_id, scene_dir, output_path, stage_assets, cached):
        return None
    result = cached.get("result")
    if not isinstance(result, dict):
        return None
    reused = dict(result)
    reused.update(
        {
            "status": "current",
            "cache_hit": True,
            "scene_revision": revision,
            "source_revision": revision,
            "preparation_key": preparation_key(scene_id, output_path, revision, stage_assets, diagnostic),
            "asset_ingestion": "cache_reused" if stage_assets else "not_requested",
            "coalesced": True,
        }
    )
    return reused


def _invoke_original(original_main: Callable[[Optional[Sequence[str]]], int], argv: Sequence[str]) -> tuple[int, str]:
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured):
        return_code = original_main(argv)
    return int(return_code), captured.getvalue()


def _parse_result(stdout_text: str) -> Optional[dict[str, Any]]:
    lines = [line for line in stdout_text.splitlines() if line.strip()]
    if not lines:
        return None
    try:
        payload = json.loads(lines[-1])
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


def run_coalesced(impl: Any, original_main: Callable[[Optional[Sequence[str]]], int], argv: Optional[Sequence[str]] = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    scene_arg = _option_value(args, "--scene")
    if scene_arg is not None and not scene_arg.strip():
        print("error: Product View preparation requires a non-empty scene identity", file=sys.stderr)
        return 2
    output_arg = _option_value(args, "--output")
    if scene_arg is None or output_arg is None:
        return original_main(args)

    try:
        scene_id, scene_dir = impl.normalize_scene(scene_arg)
        output_path = impl.normalize_output(output_arg)
    except (FileNotFoundError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    if not scene_id.strip() or scene_id == "No scene":
        print("error: Product View preparation requires a non-empty scene identity", file=sys.stderr)
        return 2

    stage_assets = _has_flag(args, "--stage-assets")
    diagnostic = _has_flag(args, "--allow-incomplete-preview")
    force = _has_flag(args, "--force")
    cache_path = _cache_path(output_path, scene_id, stage_assets, diagnostic)
    lock_path = _lock_path(output_path, scene_id)

    try:
        with preparation_lock(lock_path) as waited_ms:
            revision = source_revision(impl, scene_dir)
            if not force:
                reused = _cached_result(
                    impl, cache_path, scene_id, scene_dir, output_path,
                    revision, stage_assets, diagnostic,
                )
                if reused is not None:
                    reused["preparation_lock_wait_ms"] = waited_ms
                    print(json.dumps(reused, sort_keys=True))
                    return 0

            superseded = 0
            for attempt in (1, 2):
                revision_before = source_revision(impl, scene_dir)
                return_code, stdout_text = _invoke_original(original_main, args)
                if return_code != 0:
                    if stdout_text:
                        print(stdout_text, end="")
                    return return_code
                result = _parse_result(stdout_text)
                if result is None:
                    if stdout_text:
                        print(stdout_text, end="")
                    print("error: Product View freshener did not emit its JSON result", file=sys.stderr)
                    return 3
                revision_after = source_revision(impl, scene_dir)
                if revision_after != revision_before:
                    superseded += 1
                    if attempt == 1:
                        print(
                            "Product View preparation superseded by a newer scene revision; retrying once.",
                            file=sys.stderr,
                        )
                        continue
                    print(
                        "error: scene inputs changed repeatedly while Product View was preparing; no stale result was published",
                        file=sys.stderr,
                    )
                    return 4

                key = preparation_key(scene_id, output_path, revision_after, stage_assets, diagnostic)
                result.update(
                    {
                        "cache_hit": False,
                        "scene_revision": revision_after,
                        "source_revision": revision_after,
                        "preparation_key": key,
                        "preparation_attempt": attempt,
                        "superseded_preparation_count": superseded,
                        "preparation_lock_wait_ms": waited_ms,
                        "asset_ingestion": (
                            "rebuilt" if stage_assets and result.get("status") == "rebuilt"
                            else "validated" if stage_assets
                            else "not_requested"
                        ),
                        "coalesced": waited_ms > 0,
                    }
                )
                cache_payload = {
                    "schema_version": CACHE_SCHEMA,
                    "scene_id": scene_id,
                    "source_revision": revision_after,
                    "diagnostic": diagnostic,
                    "artifacts": _artifact_snapshot(
                        impl, scene_id, scene_dir, output_path, stage_assets
                    ),
                    "result": result,
                }
                _atomic_write_json(cache_path, cache_payload)
                print(json.dumps(result, sort_keys=True))
                return 0
            return 4
    except TimeoutError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 5
