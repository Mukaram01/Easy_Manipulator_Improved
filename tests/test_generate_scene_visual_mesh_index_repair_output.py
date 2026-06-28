from __future__ import annotations

import json

from scripts.generate_scene_visual_mesh_index import _repair_existing_index
from scripts.repair_ur5_scene3d_visual_index import REQUIRED_UR5_LINKS


def test_single_scene_repair_reports_missing_end_effector_links(tmp_path, capsys):
    index_path = tmp_path / "scene_visual_mesh_index.json"
    rows = [
        {
            "id": f"mesh_{link}",
            "link": link,
            "mesh_uri": f"package://ur_description/meshes/ur5/visual/{link}.dae",
            "geometry_type": "mesh",
            "resolved": True,
            "render_expected": True,
            "pose": {"xyz": [float(i) * 0.1, 0.0, 0.2], "rpy": [0.0, 0.0, 0.0]},
        }
        for i, link in enumerate(REQUIRED_UR5_LINKS)
    ]
    index_path.write_text(json.dumps({"scene_name": "ur5_2f_test", "visual_items": rows}) + "\n", encoding="utf-8")

    assert _repair_existing_index(index_path) == 0

    out = capsys.readouterr().out
    assert "UR5 repair applied" in out
    assert "end_effector_links=tool0,robotiq_85_base_link" in out


def test_single_scene_repair_reports_already_safe(tmp_path, capsys):
    index_path = tmp_path / "scene_visual_mesh_index.json"
    index_path.write_text(json.dumps({"scene_name": "generic_scene", "visual_items": []}) + "\n", encoding="utf-8")

    assert _repair_existing_index(index_path) == 0

    out = capsys.readouterr().out
    assert "existing index already safe for preview" in out
