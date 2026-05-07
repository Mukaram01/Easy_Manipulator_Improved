from __future__ import annotations

from pathlib import Path

from tools.workcell_studio_streamlit import backend


def test_render_topdown_targets_svg_empty_targets() -> None:
    svg = backend.render_topdown_targets_svg([])
    assert "<svg" in svg
    assert "No targets discovered yet" in svg
    assert "Traceback" not in svg


def test_render_topdown_targets_svg_with_pick_and_bin() -> None:
    targets = [
        {"id": "pick_zone_main", "type": "pick_zone", "label": "Main pick", "pose": {"frame": "world", "xyz": [0.45, 0.0, 0.08], "rpy": [0, 0, 0]}, "size": [0.3, 0.2, 0.1]},
        {"id": "bin_red", "type": "bin", "label": "Red bin", "pose": {"frame": "world", "xyz": [0.30, 0.35, 0.10], "rpy": [0, 0, 0]}, "size": [0.2, 0.2, 0.1]},
    ]
    svg = backend.render_topdown_targets_svg(targets)
    assert "pick_zone_main" in svg
    assert "bin_red" in svg
    assert "xyz=(0.45,0.00,0.08)" in svg
    assert "<rect" in svg


def test_list_targets_save_load_roundtrip(tmp_path: Path) -> None:
    path = tmp_path / "environment_layout.yaml"
    payload = {
        "schema": "environment_layout/v1",
        "zones": [
            {"id": "pick_zone_main", "type": "pick_zone", "label": "Main", "pose": {"frame": "world", "xyz": [0.45, 0.0, 0.08], "rpy": [0, 0, 0]}, "size": [0.3, 0.2, 0.1]}
        ],
    }
    backend.save_environment_layout(path, payload)
    loaded = backend.load_environment_layout(path)
    assert loaded["zones"][0]["id"] == "pick_zone_main"
    targets = backend.list_environment_targets(path)
    assert targets[0]["type"] == "pick_zone"
    assert targets[0]["pose"]["xyz"][0] == 0.45


def test_create_or_update_environment_target_with_topdown_flow(tmp_path: Path) -> None:
    layout = tmp_path / "environment_layout.yaml"
    res = backend.create_or_update_environment_target(layout, "pick_zone_main", "pick_zone", "Main", "world", [0.45, 0, 0.08], [0, 0, 0], [0.3, 0.2, 0.1], output_path=layout)
    assert res["returncode"] == 0
    targets = backend.list_environment_targets(layout)
    svg = backend.render_topdown_targets_svg(targets)
    assert "pick_zone_main" in svg


def test_backend_does_not_import_streamlit() -> None:
    text = (Path(__file__).resolve().parents[1] / "tools" / "workcell_studio_streamlit" / "backend.py").read_text(encoding="utf-8")
    assert "import streamlit" not in text
