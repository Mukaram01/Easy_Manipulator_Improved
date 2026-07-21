import json
import subprocess
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "workcell_builder/workcell_builder/config/asset_profiles/environment_assets.json"
REGISTRY = ROOT / "config/workcell_studio_asset_replacements.yaml"
CATALOG = ROOT / "config/workcell_studio_visual_asset_catalog.yaml"
SCRIPT = ROOT / "scripts/manage_workcell_environment_assets.py"


def test_every_generated_placeholder_has_an_explicit_replacement_decision():
    profile = json.loads(PROFILE.read_text(encoding="utf-8"))
    registry = yaml.safe_load(REGISTRY.read_text(encoding="utf-8"))

    assert registry["schema_version"] == "workcell_studio_asset_replacements/v1"
    registered = registry["assets"]
    placeholders = {
        item["asset_id"]
        for item in profile
        if "placeholder" in str(item.get("source_note", "")).lower()
    }

    assert placeholders
    assert not placeholders.difference(registered)
    for asset_id in placeholders:
        decision = registered[asset_id]
        assert decision["current_quality"] in {"placeholder", "acceptable_primitive"}
        assert decision["action"] in {
            "replace_with_canonical",
            "replace_with_downloaded_model",
            "migrate_to_urdf_primitive",
        }
        assert len(decision["expected_dimensions_m"]) == 3


def test_canonical_assets_and_legacy_placeholders_are_not_presented_as_equal():
    catalog = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))["categories"]

    assert catalog["table"]["quality_tier"] == "canonical"
    assert catalog["table"]["replacement_required"] is False
    assert catalog["table"]["preferred_mesh_uris"] == [
        "package://table_description/meshes/visual/table.stl"
    ]
    assert catalog["workbench"]["quality_tier"] == "canonical"
    assert catalog["camera_realsense"]["quality_tier"] == "canonical"

    for key in ("conveyor_placeholder", "bin_box_fixture"):
        assert catalog[key]["quality_tier"] == "placeholder"
        assert catalog[key]["replacement_required"] is True
        assert catalog[key]["support_status"] == "preview_only"
        assert "Placeholder" in catalog[key]["preview_label"]


def test_asset_management_script_compiles_and_requires_redistribution_evidence(tmp_path):
    subprocess.run([sys.executable, "-m", "py_compile", str(SCRIPT)], check=True)

    visual = tmp_path / "candidate.stl"
    visual.write_text("solid candidate\nendsolid candidate\n", encoding="utf-8")
    license_file = tmp_path / "LICENSE.txt"
    license_file.write_text("test licence text", encoding="utf-8")

    base = [
        sys.executable,
        str(SCRIPT),
        "import",
        "--asset-id",
        "pipeline_test_candidate",
        "--display-name",
        "Pipeline Test Candidate",
        "--visual-file",
        str(visual),
        "--dimensions",
        "0.4",
        "0.3",
        "0.2",
        "--source-url",
        "https://example.invalid/model",
        "--author",
        "Test Author",
        "--license-id",
        "CC0-1.0",
        "--license-file",
        str(license_file),
        "--dry-run",
    ]

    rejected = subprocess.run(base, text=True, capture_output=True, check=False)
    assert rejected.returncode == 2
    assert "--redistribution-confirmed is required" in rejected.stderr

    accepted = subprocess.run(
        [*base, "--redistribution-confirmed"],
        text=True,
        capture_output=True,
        check=False,
    )
    assert accepted.returncode == 0, accepted.stdout + accepted.stderr
    payload = json.loads(accepted.stdout)
    assert payload["status"] == "dry_run"
    assert payload["package"].endswith(
        "assets/environment/pipeline_test_candidate_description"
    )


def test_placeholder_deletion_is_reference_and_registry_guarded():
    source = SCRIPT.read_text(encoding="utf-8")

    assert 'status in {"replaced", "delete_when_unreferenced"}' in source
    assert "and not references" in source
    assert "safe_to_delete_now" in source
    assert "Dry run only; pass --apply" in source
    assert "Referenced placeholders remain quarantined" in source
