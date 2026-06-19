from __future__ import annotations

import importlib.util
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXTRACTOR_PATH = ROOT / "scripts" / "extract_scene_urdf_visual_mesh_index.py"


def _load_extractor():
    spec = importlib.util.spec_from_file_location("extract_scene_urdf_visual_mesh_index", EXTRACTOR_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _assert_pose_close(actual: dict, expected: dict, *, eps: float = 1e-9) -> None:
    for field in ("xyz", "rpy"):
        assert len(actual[field]) == len(expected[field]) == 3
        for a, e in zip(actual[field], expected[field]):
            assert math.isclose(a, e, abs_tol=eps), f"{field}: {actual[field]} != {expected[field]}"


def test_urdf_flattened_visual_pose_is_link_world_times_visual_origin_without_viewport_basis() -> None:
    extractor = _load_extractor()
    world_T_link = extractor.tf_from_xyz_rpy([1.0, 2.0, 3.0], [0.0, 0.0, math.pi / 2.0])
    visual_origin = extractor.tf_from_xyz_rpy([1.0, 0.0, 0.5], [0.25, 0.0, 0.0])

    expected_world_T_visual = extractor.xyz_rpy_from_tf(extractor.matmul4(world_T_link, visual_origin))
    reverse_order_pose = extractor.xyz_rpy_from_tf(extractor.matmul4(visual_origin, world_T_link))

    _assert_pose_close(expected_world_T_visual, {"xyz": [1.0, 3.0, 3.5], "rpy": [0.25, 0.0, math.pi / 2.0]})
    assert reverse_order_pose["xyz"] != expected_world_T_visual["xyz"]

    xml_text = f"""
    <robot name="transform_order_test">
      <link name="world" />
      <joint name="world_to_link" type="fixed">
        <parent link="world" />
        <child link="link" />
        <origin xyz="1 2 3" rpy="0 0 {math.pi / 2.0}" />
      </joint>
      <link name="link">
        <visual name="offset_visual">
          <origin xyz="1 0 0.5" rpy="0.25 0 0" />
          <geometry><box size="0.1 0.2 0.3" /></geometry>
        </visual>
      </link>
    </robot>
    """

    items, _diagnostics = extractor.extract_from_urdf(xml_text, {}, include_diagnostics=True)
    assert len(items) == 1
    item = items[0]

    _assert_pose_close(item["pose"], expected_world_T_visual)
    _assert_pose_close(item["expected_visual_pose"], expected_world_T_visual)
    assert item["baked_world_visual_transform_source"] == "urdf_fk_link_world_times_visual_origin"
    assert item["visual_origin_applied_to_pose"] is True

    assert item["pose"]["xyz"] != reverse_order_pose["xyz"]
    assert item["pose"]["rpy"] != reverse_order_pose["rpy"]

    extractor_source = EXTRACTOR_PATH.read_text(encoding="utf-8")
    assert "'ros_to_viewport_basis_applied':False" in extractor_source
