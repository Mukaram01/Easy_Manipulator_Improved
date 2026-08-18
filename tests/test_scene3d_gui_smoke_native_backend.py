from __future__ import annotations

import os

from scripts import run_workcell_builder_scene3d_gui_smoke as smoke


def test_scene3d_gui_smoke_forces_native_product_view_backend(monkeypatch):
    """The native Scene3D smoke must not inherit the normal Web3D default/override."""
    monkeypatch.setenv("WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND", "embedded_web3d")

    smoke._configure_scene3d_smoke_environment()

    assert os.environ["WORKCELL_BUILDER_PRODUCT_VIEW_BACKEND"] == "native_scene3d"
