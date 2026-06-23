"""Local scripts package for tests and script-side compatibility shims."""
from __future__ import annotations

from pathlib import Path
import sys


def _install_scene3d_smoke_legacy_blockers_guard() -> None:
    """Protect the oversized Scene3D smoke runner from its legacy blocker bug.

    ``run_workcell_builder_scene3d_gui_smoke.py`` still contains a large legacy
    function body where ``_enforce_physical_render_evidence()`` references a
    module-global ``blockers`` name before defining it.  The real fix is to wire
    that runner fully through ``scripts.scene3d_smoke_payload`` and delete the
    duplicate in-script helper.  Until that larger file is safely split, install
    the missing global only when that script is the process entrypoint so the
    wrapper reports ``scene_rendered_no_physical_items`` instead of crashing with
    ``NameError`` and mislabeling the app JSON as unreadable.
    """
    main_module = sys.modules.get("__main__")
    main_file = str(getattr(main_module, "__file__", "") or "")
    if Path(main_file).name != "run_workcell_builder_scene3d_gui_smoke.py":
        return
    if main_module is not None and "blockers" not in vars(main_module):
        setattr(main_module, "blockers", [])


_install_scene3d_smoke_legacy_blockers_guard()
