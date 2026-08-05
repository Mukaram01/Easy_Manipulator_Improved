#!/usr/bin/env python3
"""Revision-coalesced entry point for Workcell Studio web-scene preparation."""

from __future__ import annotations

import sys
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import ensure_workcell_studio_web_scene_fresh_impl as _impl
import workcell_studio_scene_refresh_coalescer as _coalescer

# The exporter wrapper, its preserved implementation, and the portability
# normalizer are all generator inputs. A change to any of them must renew the
# scene revision and invalidate the coalesced preparation cache.
_PORTABLE_GENERATOR_INPUTS = (
    "scripts/export_workcell_studio_web_scene_impl.py",
    "scripts/workcell_studio_visual_artifact_portability.py",
)
_impl.GENERATOR_INPUT_RELS = tuple(dict.fromkeys((*_impl.GENERATOR_INPUT_RELS, *_PORTABLE_GENERATOR_INPUTS)))

_EXPORTED_NAMES = {
    name for name in _impl.__dict__ if not name.startswith("__") and name != "main"
}
for _name in _EXPORTED_NAMES:
    globals()[_name] = getattr(_impl, _name)

_ORIGINAL_MAIN = _impl.main


def _sync_impl_globals() -> None:
    # Existing tests and callers monkeypatch constants/helpers on this public
    # module. Mirror those values into the implementation module before every
    # call so the wrapper remains API-compatible with the original script.
    for name in _EXPORTED_NAMES:
        if name in globals():
            setattr(_impl, name, globals()[name])


def main(argv=None) -> int:
    _sync_impl_globals()
    return _coalescer.run_coalesced(_impl, _ORIGINAL_MAIN, argv)


if __name__ == "__main__":
    raise SystemExit(main())
