from pathlib import Path
import json

ASSETS = [
    "table_small",
    "table_large",
    "workbench",
    "bin_small",
    "bin_large",
    "tray",
    "tote_box",
    "conveyor_placeholder",
    "fixture_plate",
    "pallet",
    "pedestal",
    "camera_stand",
    "safety_fence_panel",
    "robot_base_plate",
    "calibration_cube",
    "pick_box",
    "cylinder_object",
]
URDF_PRIMITIVES = {
    "fixture_plate": '<box size="0.5 0.5 0.03"/>',
    "calibration_cube": '<box size="0.1 0.1 0.1"/>',
    "pick_box": '<box size="0.08 0.08 0.08"/>',
    "cylinder_object": '<cylinder radius="0.03" length="0.12"/>',
}


def test_curated_assets_use_meshes_only_when_geometry_needs_them():
    root = Path("workcell_builder/workcell_builder/assets/environment")
    assert root.exists()
    for asset_id in ASSETS:
        base = root / f"{asset_id}_description"
        stl = base / "meshes" / f"{asset_id}.stl"
        urdf = base / "urdf" / f"{asset_id}.urdf.xacro"
        assert urdf.exists()
        assert not urdf.is_symlink()

        if asset_id in URDF_PRIMITIVES:
            assert not stl.exists()
            text = urdf.read_text(encoding="utf-8")
            assert "<mesh" not in text
            assert text.count(URDF_PRIMITIVES[asset_id]) == 2
        else:
            assert stl.exists()
            assert not stl.is_symlink()
            assert stl.stat().st_size < 2 * 1024 * 1024


def test_environment_assets_catalog_and_categories_and_paths():
    arr = json.loads(
        Path(
            "workcell_builder/workcell_builder/config/asset_profiles/environment_assets.json"
        ).read_text()
    )
    ids = {item["asset_id"] for item in arr}
    assert set(ASSETS).issubset(ids)

    for item in arr:
        source_path = Path(item["mesh_path"])
        assert not source_path.is_absolute()
        assert source_path.exists()
        assert item["category"] in {
            "Tables / Workbenches",
            "Bins / Trays / Totes",
            "Conveyors",
            "Fixtures",
            "Safety / Fencing",
            "Camera Mounts",
            "Robot Bases",
            "Pick Objects",
        }
        if item.get("geometry_type") == "urdf_primitive":
            assert item["mesh_path"] == item["urdf_path"]
            assert source_path.name.endswith(".urdf.xacro")


def test_no_forbidden_tokens():
    txt = "\n".join(
        Path(path).read_text(encoding="utf-8", errors="ignore").lower()
        for path in [
            "scripts/validate_workcell_asset_catalog.py",
            "scripts/generate_golden_workcell_demo.py",
        ]
    )
    for forbidden in [
        "pyyaml",
        "streamlit",
        "getmotionplan",
        "execute_trajectory",
        "/plan_kinematic_path",
        "real_hardware_enabled: true",
    ]:
        assert forbidden not in txt


def test_portable_bundle_markers_present():
    blob = Path(
        "workcell_builder/workcell_builder/gui/scene_select.cpp"
    ).read_text(encoding="utf-8")
    for marker in [
        "Export Scene Bundle",
        "Import Scene Bundle",
        "Portable Scene Bundle",
        "Bundle Validation Status",
        "Imported Scene Ready",
        "Exported Scene Archive",
    ]:
        assert marker in blob
