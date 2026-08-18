from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts" / "validate_workcell_asset_provenance.py"
SPEC = importlib.util.spec_from_file_location("validate_workcell_asset_provenance", SCRIPT)
assert SPEC and SPEC.loader
module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


def test_north_star_imported_assets_are_portable_and_hash_verified():
    report = module.validate_scene(ROOT / "scenes" / "ur5_2f_test")
    assert report["status"] == "WARN"
    assert report["errors"] == []
    assert report["summary"] == {
        "catalog_asset_count": 2,
        "verified_hash_count": 2,
        "placed_imported_instance_count": 3,
        "portable_reference_count": 3,
    }
    assert len(report["warnings"]) == 3


def test_north_star_manifest_has_complete_v2_provenance_and_unique_ids():
    manifest = yaml.safe_load(
        (ROOT / "scenes/ur5_2f_test/assets/imported/asset_manifest.yaml").read_text(encoding="utf-8")
    )
    assert manifest["schema_version"] == "workcell_studio_asset_manifest/v2"
    ids = [asset["id"] for asset in manifest["assets"]]
    assert len(ids) == len(set(ids)) == 2
    for asset in manifest["assets"]:
        assert all(asset.get(field) not in (None, "") for field in module.REQUIRED_PROVENANCE)
        assert asset["path"] == asset["staged_relative_path"]
        assert asset["import_contract_version"] == "workcell_studio_mesh_import/v2"


def test_layout_serializer_normalizes_scene_local_absolute_mesh_paths():
    cpp = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    assert 'portable_mesh_path.indexOf(QStringLiteral("/scenes/"))' in cpp
    assert 'portable_mesh_path.indexOf(QStringLiteral("/assets/")' in cpp
    assert "state.mesh_path = portable_mesh_path.toStdString()" in cpp
    assert 'asset["staged_relative_path"]' in cpp
    assert 'asset["source_sha256"]' in cpp
    assert 'asset["imported_at_utc"]' in cpp
    assert 'root["scene_path"] = "."' in cpp
    assert 'item["scene_path"] = "."' in cpp
