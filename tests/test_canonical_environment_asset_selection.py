import importlib.util
import shutil
from pathlib import Path

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
                            "id": "workbench_support_surface",
                            "type": "table",
                            "role": "support_surface",
                            "category": "work_surface",
                            "frame": "world",
                            "pose_xyz": [0.0, 0.0, 0.0],
                            "pose_rpy": [0.0, 0.0, 0.0],
                            "mesh_uri": "package://workbench_description/meshes/visual/table.stl",
                            "mesh_scale": [0.001, 0.001, 0.001],
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
                            "mesh_uri": "package://realsense2_description/meshes/d435.dae",
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

        table = next(item for item in payload["assets"] if item["id"] == "workbench_support_surface")
        camera = next(item for item in payload["sensors"] if item["id"] == "configured_camera")

        assert table["mesh_staging_status"] == "staged"
        assert camera["mesh_staging_status"] == "staged"
        assert table["original_package_uri"] == (
            "package://workbench_description/meshes/visual/table.stl"
        )
        assert camera["original_package_uri"] == (
            "package://realsense2_description/meshes/d435.dae"
        )
        assert table["mesh_uri"].endswith("workbench_description/meshes/visual/table.stl")
        assert camera["mesh_uri"].endswith("realsense2_description/meshes/d435.dae")

        required = payload["viewer_summary"]["required_item_status"]
        assert required["table"]["status"] == "mesh_backed"
        assert required["camera"]["status"] == "mesh_backed"
        assert payload["metadata"]["mesh_contract"]["core_mesh_failures"] == []
    finally:
        shutil.rmtree(staged_dir, ignore_errors=True)
