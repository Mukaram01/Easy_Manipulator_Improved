import importlib.util
import json
import shutil
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
EXPORTER_PATH = ROOT / "scripts" / "export_workcell_studio_web_scene.py"
SPEC = importlib.util.spec_from_file_location("canonical_asset_exporter", EXPORTER_PATH)
EXPORTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXPORTER)


def test_visual_catalog_points_to_real_package_mesh_locations():
    catalog = yaml.safe_load(
        (ROOT / "config" / "workcell_studio_visual_asset_catalog.yaml").read_text(encoding="utf-8")
    )["categories"]

    assert catalog["table"]["preferred_mesh_uris"] == [
        "package://table_description/meshes/visual/table.stl"
    ]
    assert catalog["workbench"]["preferred_mesh_uris"] == [
        "package://workbench_description/meshes/visual/table.stl"
    ]
    assert catalog["camera_realsense"]["preferred_mesh_uris"] == [
        "package://realsense2_description/meshes/d435.dae"
    ]

    assert (ROOT / "assets/environment/table_description/meshes/visual/table.stl").is_file()
    assert (ROOT / "assets/environment/workbench_description/meshes/visual/table.stl").is_file()
    assert (ROOT / "assets/environment/realsense2_description/meshes/d435.dae").is_file()


def test_table_description_uses_separate_visual_and_collision_meshes():
    xacro = (ROOT / "assets/environment/table_description/urdf/table.urdf.xacro").read_text(
        encoding="utf-8"
    )
    table = yaml.safe_load(
        (ROOT / "assets/environment/table_description/table.yaml").read_text(encoding="utf-8")
    )["table"]["links"]["table"]

    assert 'filename="package://table_description/meshes/visual/table.stl"' in xacro
    assert 'filename="package://table_description/meshes/collision/table.stl"' in xacro
    assert table["visual"]["geometry"]["filepath"] == (
        "package://table_description/meshes/visual/table.stl"
    )
    assert table["collision"]["geometry"]["filepath"] == (
        "package://table_description/meshes/collision/table.stl"
    )


def test_builder_catalog_prefers_package_visuals_and_demotes_placeholders():
    source = (
        ROOT / "workcell_builder/workcell_builder/gui/asset_catalog_model.cpp"
    ).read_text(encoding="utf-8")

    for token in [
        'ext == ".dae"',
        'ext == ".obj"',
        "path_is_collision(path)",
        "package_directory_for(path, root)",
        'filename == "d435.dae"',
        'entry.source = placeholder ? "generated_placeholder"',
        '"canonical_asset"',
        "emitted_packages",
    ]:
        assert token in source

    assert source.index('roots.emplace_back(repo_root + "/assets")') < source.index(
        'roots.emplace_back(repo_root + "/workcell_builder/workcell_builder/assets")'
    )


def test_canonical_table_and_camera_stage_as_mesh_backed_web_assets(tmp_path):
    scene_id = "canonical_environment_asset_test"
    scene = tmp_path / scene_id
    scene.mkdir()
    (scene / "environment.yaml").write_text(
        yaml.safe_dump(
            {
                "scene": {"id": scene_id, "name": scene_id},
                "environment": {
                    "support_surfaces": [
                        {
                            "id": "support_surface_table",
                            "type": "support_surface",
                            "role": "support_surface",
                            "category": "work_surface",
                            "frame": "world",
                            "pose_xyz": [0.0, 0.0, 0.0],
                            "pose_rpy": [0.0, 0.0, 0.0],
                            "geometry_type": "box",
                            "primitive_geometry_type": "box",
                            "dimensions": [1.2, 0.8, 0.08],
                        }
                    ],
                    "sensors": [
                        {
                            "id": "configured_camera",
                            "type": "realsense",
                            "role": "camera",
                            "category": "camera",
                            "frame": "world",
                            "pose_xyz": [0.35, 0.0, 0.85],
                            "pose_rpy": [0.0, 1.5708, 0.0],
                            "geometry_type": "box",
                            "primitive_geometry_type": "box",
                            "dimensions": [0.08, 0.08, 0.06],
                        }
                    ],
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    staged_dir = ROOT / "build/workcell_studio_web_scene/assets" / scene_id
    try:
        payload = EXPORTER.build_web_scene(
            scene,
            stage_assets=True,
            output_path=tmp_path / "build" / f"{scene_id}.web_scene.json",
        )

        table = next(item for item in payload["assets"] if item["id"] == "support_surface_table")
        camera = next(item for item in payload["sensors"] if item["id"] == "configured_camera")

        assert table["mesh_staging_status"] == "staged"
        assert camera["mesh_staging_status"] == "staged"
        assert table["original_package_uri"] == (
            "package://table_description/meshes/visual/table.stl"
        )
        assert camera["original_package_uri"] == (
            "package://realsense2_description/meshes/d435.dae"
        )
        assert table["mesh_uri"].endswith("table_description/meshes/visual/table.stl")
        assert camera["mesh_uri"].endswith("realsense2_description/meshes/d435.dae")
        assert table["mesh_scale"] == [0.001, 0.001, 0.001]
        assert table["mesh_local_transform"] == {
            "xyz": [0.2006668243, -0.2022999573, 0.9053377075],
            "rpy": [0.0, 0.0, 0.0],
            "scale": [0.001, 0.001, 0.001],
        }
        raw_min, raw_max, _ = EXPORTER._stl_mesh_local_bounds(table)
        local = table["mesh_local_transform"]
        normalized_min = [raw_min[i] * local["scale"][i] + local["xyz"][i] for i in range(3)]
        normalized_max = [raw_max[i] * local["scale"][i] + local["xyz"][i] for i in range(3)]
        assert normalized_min[2] == pytest.approx(0.0, abs=1e-9)
        assert (normalized_min[0] + normalized_max[0]) / 2.0 == pytest.approx(0.0, abs=1e-9)
        assert (normalized_min[1] + normalized_max[1]) / 2.0 == pytest.approx(0.0, abs=1e-9)
        assert camera["mesh_local_transform"] == {
            "xyz": [0.0149, 0.0, 0.0125],
            "rpy": [1.5707963267948966, 0.0, 1.5707963267948966],
            "scale": [1.0, 1.0, 1.0],
        }
        assert table["editable"] is True and table["locked"] is False
        assert camera["editable"] is True and camera["locked"] is False

        required = payload["viewer_summary"]["required_item_status"]
        assert required["table"]["status"] == "mesh_backed"
        assert required["camera"]["status"] == "mesh_backed"
        assert payload["metadata"]["mesh_contract"]["core_mesh_failures"] == []
    finally:
        shutil.rmtree(staged_dir, ignore_errors=True)


def test_catalog_inference_preserves_explicit_mesh_fallback_and_authored_ownership(tmp_path):
    scene = tmp_path / "semantic_asset_precedence"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir()
    explicit_mesh = scene / "explicit_table.stl"
    explicit_mesh.write_text("solid explicit\nendsolid explicit\n", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text(
        yaml.safe_dump(
            {
                "items": [
                    {"id": "support_surface_table", "type": "support_surface", "role": "support_surface", "geometry_type": "box", "dimensions": [1.2, 0.8, 0.08], "pose": {"xyz": [0.4, 0.0, 0.06], "rpy": [0.0, 0.0, 0.0]}},
                    {"id": "realsense_overhead", "type": "realsense", "role": "camera", "geometry_type": "box", "dimensions": [0.08, 0.08, 0.06], "pose": {"xyz": [0.35, 0.1, 0.85], "rpy": [0.0, 1.5708, 0.0]}},
                    {"id": "explicit_table", "type": "table", "role": "support_surface", "mesh_path": "explicit_table.stl", "geometry_type": "mesh"},
                    {"id": "unsupported_fixture", "type": "custom_fixture", "role": "fixture", "geometry_type": "box", "dimensions": [0.2, 0.2, 0.2]},
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (scene / "generated/scene_visual_mesh_index.json").write_text(
        json.dumps(
            {
                "visual_items": [
                    {"id": "generated_table", "type": "table", "role": "support_surface", "support_surface_ref": "support_surface_table", "mesh_uri": "package://table_description/meshes/visual/table.stl"},
                    {"id": "generated_camera", "type": "realsense", "role": "camera", "canonical_scene_item_id": "realsense_overhead", "camera_id": "realsense_overhead", "mesh_uri": "package://realsense2_description/meshes/d435.dae"},
                    {"id": "generated_robot", "type": "robot", "role": "robot", "link": "base_link", "mesh_uri": "package://ur_description/meshes/ur5/visual/base.dae"},
                ]
            }
        ),
        encoding="utf-8",
    )
    (scene / "cell_definition.yaml").write_text(
        yaml.safe_dump(
            {
                "environment": {
                    "support_surfaces": [
                        {"id": "support_surface_table", "type": "table", "dimensions": [1.2, 0.8, 0.08]}
                    ]
                }
            }
        ),
        encoding="utf-8",
    )

    payload = EXPORTER.build_web_scene(scene, stage_assets=False, output_path=tmp_path / "scene.json")
    items = {item["id"]: item for section in ("robots", "assets", "sensors") for item in payload[section]}

    assert items["support_surface_table"]["mesh_uri"] == "package://table_description/meshes/visual/table.stl"
    assert items["realsense_overhead"]["mesh_uri"] == "package://realsense2_description/meshes/d435.dae"
    assert items["explicit_table"]["mesh_path"] == "explicit_table.stl"
    assert items["explicit_table"].get("visual_asset_catalog_category") is None
    assert items["unsupported_fixture"].get("mesh_uri") is None
    assert items["unsupported_fixture"]["geometry_type"] == "box"
    assert items["support_surface_table"]["render_policy"] == "primary"
    assert items["realsense_overhead"]["render_policy"] == "primary"
    assert items["generated_table"]["render_policy"] == "diagnostic_only"
    assert items["generated_camera"]["render_policy"] == "diagnostic_only"
    assert items["generated_camera"]["canonical_scene_item_id"] == "realsense_overhead"
    assert items["generated_robot"]["render_policy"] == "primary"
    assert sum(item["id"] == "support_surface_table" for item in payload["assets"]) == 1
    assert sum(
        item["render_policy"] == "primary" and item.get("core_mesh_category") == "camera_realsense"
        for item in payload["sensors"]
    ) == 1
    assert items["realsense_overhead"]["editable"] is True
    camera_owner = next(owner for owner in payload["ui_selection_owners"] if owner["id"] == "realsense_overhead")
    assert camera_owner["editable"] is True and camera_owner["locked"] is False
    assert items["support_surface_table"]["pose"] == {"xyz": [0.4, 0.0, 0.06], "rpy": [0.0, 0.0, 0.0]}


def test_imported_pallet_obj_manifest_and_web_staging(tmp_path):
    package = (
        ROOT
        / "workcell_builder/workcell_builder/assets/environment/pallet_description"
    )
    manifest = yaml.safe_load((package / "asset_manifest.yaml").read_text(encoding="utf-8"))
    visual_uri = (
        "package://workcell_builder/assets/environment/pallet_description/"
        "meshes/visual/pallet.obj"
    )

    assert manifest["source"]["creator"] == "Kenney"
    assert manifest["license"]["spdx"] == "CC0-1.0"
    assert manifest["license"]["redistribution_confirmed"] is True
    assert manifest["geometry"]["visual_uri"] == visual_uri
    assert manifest["geometry"]["final_dimensions_m"] == [1.2, 0.8, 0.144]
    assert (package / "meshes/visual/pallet.obj").is_file()
    assert not (package / "meshes/pallet.obj").exists()

    scene_id = "imported_pallet_asset_test"
    scene = tmp_path / scene_id
    scene.mkdir()
    (scene / "environment.yaml").write_text(
        yaml.safe_dump(
            {
                "scene": {"id": scene_id, "name": scene_id},
                "environment": {
                    "support_surfaces": [
                        {
                            "id": "imported_pallet",
                            "type": "pallet",
                            "role": "support_surface",
                            "category": "fixture",
                            "frame": "world",
                            "pose_xyz": [0.0, 0.0, 0.0],
                            "pose_rpy": [1.57079632679, 0.0, 0.0],
                            "mesh_uri": visual_uri,
                            "mesh_scale": [1.2, 0.96, 0.8],
                        }
                    ]
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    staged_dir = ROOT / "build/workcell_studio_web_scene/assets" / scene_id
    try:
        payload = EXPORTER.build_web_scene(
            scene,
            stage_assets=True,
            output_path=tmp_path / "build" / f"{scene_id}.web_scene.json",
        )
        pallet = next(item for item in payload["assets"] if item["id"] == "imported_pallet")

        assert pallet["mesh_staging_status"] == "staged"
        assert pallet["original_package_uri"] == visual_uri
        assert pallet["mesh_uri"].endswith(
            "workcell_builder/assets/environment/pallet_description/meshes/visual/pallet.obj"
        )
        assert pallet["mesh_scale"] == [1.2, 0.96, 0.8]
        staged_mesh = ROOT / "build/workcell_studio_web_scene" / pallet["mesh_uri"]
        assert staged_mesh.is_file()
        assert staged_mesh.read_text(encoding="utf-8").count("\nf ") == 84
    finally:
        shutil.rmtree(staged_dir, ignore_errors=True)
