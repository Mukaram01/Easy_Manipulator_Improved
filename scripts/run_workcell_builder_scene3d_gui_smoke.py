#!/usr/bin/env python3
"""Compatibility wrapper for the oversized Scene3D GUI smoke runner.

The implementation still lives in ``run_workcell_builder_scene3d_gui_smoke_impl.py``
while payload-only helpers are extracted into ``scripts.scene3d_smoke_payload``.
This keeps the public CLI/path stable and lets follow-up PRs shrink the implementation
without touching callers.
"""
from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path
from types import ModuleType

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.workcell_studio_script_bootstrap import ensure_repo_root_on_sys_path

ensure_repo_root_on_sys_path(__file__)

from scripts import scene3d_smoke_payload as _payload_helpers

_IMPL_PATH = Path(__file__).with_name("run_workcell_builder_scene3d_gui_smoke_impl.py")
_IMPL_MODULE_NAME = "scripts.run_workcell_builder_scene3d_gui_smoke_impl"
_NATIVE_PRODUCT_VIEW_BACKEND_ENV = "WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND"
_NATIVE_PRODUCT_VIEW_BACKEND = "native_scene3d"


def _configure_scene3d_smoke_environment() -> None:
    """Force the native viewport whose counters and paint evidence this CLI validates."""
    # Normal Workcell Studio remains free to use its default embedded Web3D Product
    # View.  This diagnostic CLI specifically searches for Scene3DViewportWidget,
    # so an inherited Web3D backend selection must not make the smoke self-invalid.
    os.environ[_NATIVE_PRODUCT_VIEW_BACKEND_ENV] = _NATIVE_PRODUCT_VIEW_BACKEND


def _wire_payload_helpers(module: ModuleType) -> None:
    """Route extracted payload helpers through the implementation module."""
    module._append_unique = _payload_helpers.append_unique
    module._as_int = _payload_helpers.as_int
    module._counter = _payload_helpers.counter
    module._physical_rendered_count = _payload_helpers.physical_rendered_count
    module._enforce_physical_render_evidence = _payload_helpers.enforce_physical_render_evidence


def _load_impl() -> ModuleType:
    spec = importlib.util.spec_from_file_location(_IMPL_MODULE_NAME, _IMPL_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load Scene3D smoke implementation: {_IMPL_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[_IMPL_MODULE_NAME] = module
    spec.loader.exec_module(module)
    _wire_payload_helpers(module)
    return module


_IMPL = _load_impl()


def __getattr__(name: str):
    return getattr(_IMPL, name)


__all__ = [name for name in dir(_IMPL) if not name.startswith("__")]
for _name in __all__:
    globals().setdefault(_name, getattr(_IMPL, _name))


if __name__ == "__main__":
    _configure_scene3d_smoke_environment()
    raise SystemExit(_IMPL.main())
