from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]


def test_visual_mesh_index_has_resolution_fields():
    for scene in ["ur5_2f_test", "ur5_2f_sorting_test"]:
        p = ROOT / "scenes" / scene / "generated" / "scene_visual_mesh_index.json"
        data = json.loads(p.read_text(encoding="utf-8"))
        assert "safe_for_preview" in data
        assert any(k in data for k in ["visual_items", "items"])


def test_contract_checker_emits_mesh_resolution_counters():
    src = (ROOT / "scripts" / "check_scene3d_canvas_contract.py").read_text(encoding="utf-8")
    for token in [
        "mesh_uri_count",
        "mesh_resolved_count",
        "stl_count",
        "dae_count",
        "obj_count",
        "supported_mesh_count",
        "unsupported_extension_count",
    ]:
        assert token in src
