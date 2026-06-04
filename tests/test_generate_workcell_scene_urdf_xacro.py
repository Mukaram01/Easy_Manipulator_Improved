from __future__ import annotations

import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


generator = _load("generate_workcell_from_cell_definition", REPO_ROOT / "scripts" / "generate_workcell_from_cell_definition.py")


def test_render_scene_urdf_xacro_contains_world_and_placeholders() -> None:
    warnings: list[str] = []
    text = generator._render_scene_urdf_xacro(
        "generated_demo_cell",
        {
            "robot": {"model": "ur5", "planning_group": "manipulator"},
            "end_effector": {"id": "robotiq_2f", "type": "finger"},
            "environment": {
                "support_surfaces": [
                    {
                        "id": "table-main",
                        "type": "table",
                        "pose_xyz": [0.1, 0.2, 0.3],
                        "pose_rpy": [0.0, 0.0, 1.57],
                        "dimensions": [1.0, 0.5, 0.05],
                    }
                ]
            },
            "objects": [
                {
                    "id": "pick_part",
                    "shape": "box",
                    "pose_xyz": [0.4, 0.0, 0.08],
                    "dimensions": [0.05, 0.05, 0.05],
                }
            ],
        },
        {"tracked": [], "unsupported": []},
        warnings,
    )

    root = ET.fromstring(text)
    links = {link.attrib["name"] for link in root.findall("link")}
    assert root.tag == "robot"
    assert root.attrib["name"] == "generated_demo_cell"
    assert "world" in links
    assert "table_main" in links
    assert "pick_part" in links
    assert "<hardware_interface" not in text
    assert "<ros2_control" not in text
    assert any("placeholder robot comment" in warning for warning in warnings)
    assert any("placeholder tool comment" in warning for warning in warnings)
