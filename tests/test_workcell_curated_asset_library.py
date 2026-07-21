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
    "robot_base_plate": '<box size="0.6 0.6 0.04"/>',
    "calibration_cube": '<box size="0.1 0.1 0.1"/>',
    "pick_box": '<box size="0.08 0.08 0.08"/>',
    "cylinder_object": '<cylinder radius="0.03" length="0.12"/>',
}
IMPORTED_VISUAL_PILOTS = {"pallet": "pallet.obj"}


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
        elif asset_id in IMPORTED_VISUAL_PILOTS:
            visual = base / "meshes" / IMPORTED_VISUAL_PILOTS[asset_id]
            assert visual.exists()
            assert not visual.is_symlink()
            # Keep the previous generated STL only for rollback during the pilot.
            assert stl.exists()
        else:
            assert stl.exists()
            assert not stl.is_symlink()
            assert stl.stat().st_size < 2 * 1024 * 1024


def test_pallet_obj_pilot_is_small_self_contained_and_dimensioned():
    base = Path(
        "workcell_builder/workcell_builder/assets/environment/pallet_description"
    )
    obj = base / "meshes/pallet.obj"
    text = obj.read_text(encoding="utf-8")
    lines = text.splitlines()

    vertices = [
        tuple(float(value) for value in line.split()[1:4])
        for line in lines
        if line.startswith("v ")
    ]
    faces = [line for line in lines if line.startswith("f ")]

    assert len(vertices) == 56
    assert len(faces) == 84
    assert "mtllib " not in text
    assert "usemtl " not in text

    axes = list(zip(*vertices))
    bounds = tuple(max(axis) - min(axis) for axis in axes)
    assert bounds == (1.0, 0.15, 1.0)

    xacro = (base / "urdf/pallet.urdf.xacro").read_text(encoding="utf-8")
    assert "meshes/pallet.obj" in xacro
    assert 'scale="1.2 0.96 0.8"' in xacro
    assert 'rpy="1.57079632679 0 0"' in xacro
    assert '<origin xyz="0 0 0.072" rpy="0 0 0"/>' in xacro
    assert '<box size="1.2 0.8 0.144"/>' in xacro

    assert (base / "LICENSE_KENNEY.txt").exists()
    source = (base / "SOURCE.md").read_text(encoding="utf-8")
    assert "c3006c78bc764fc86a113316cbf2a0e5e48b7231" in source
    assert "CC0 1.0" in source
    assert "rollback" in source.lower()


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

    pallet = next(item for item in arr if item["asset_id"] == "pallet")
    assert pallet["mesh_path"].endswith("pallet.obj")
    assert pallet["default_dimensions_m"] == [1.2, 0.8, 0.144]
    assert pallet["license"] == "CC0-1.0"


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
