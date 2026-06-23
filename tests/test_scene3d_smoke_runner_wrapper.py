#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_scene3d_smoke_public_runner_is_thin_wrapper():
    wrapper_text = (REPO_ROOT / "scripts" / "run_workcell_builder_scene3d_gui_smoke.py").read_text(encoding="utf-8")
    impl_text = (REPO_ROOT / "scripts" / "run_workcell_builder_scene3d_gui_smoke_impl.py").read_text(encoding="utf-8")

    assert "run_workcell_builder_scene3d_gui_smoke_impl.py" in wrapper_text
    assert "scene3d_smoke_payload" in wrapper_text
    assert "_wire_payload_helpers" in wrapper_text
    assert "def main()" in impl_text
    assert len(wrapper_text.splitlines()) < 100
