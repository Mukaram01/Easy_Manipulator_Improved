"""Acceptance coverage for destination-derived Workcell Studio place overlays."""

import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "scenes/ur5_2f_test"
EXPORTER = ROOT / "scripts/export_workcell_studio_web_scene.py"
WORKFLOW = ROOT / "scripts/run_workcell_studio_web_edit_workflow.py"
BUNDLE = ROOT / "workcell_studio_web/viewer/dist/viewer.bundle.js"
BUNDLE_SHA256 = "9269c990ce086df12a796c858788409d374012d5c5e4a38c3d916180ddf365eb"


@pytest.fixture
def scene(tmp_path):
    destination = tmp_path / "ur5_2f_test"
    shutil.copytree(FIXTURE, destination)
    return destination


def _export(scene, output, *, check=True):
    return subprocess.run(
        [sys.executable, str(EXPORTER), "--scene", str(scene), "--output", str(output), "--no-stage-assets"],
        cwd=ROOT,
        check=check,
        capture_output=True,
        text=True,
    )


def _items(payload):
    for section in ("assets", "zones"):
        for item in payload[section]:
            yield item


def _by_id(payload, item_id):
    return next(item for item in _items(payload) if item["id"] == item_id)


def _transform(item):
    pose = item["pose"]
    vector = lambda values: dict(zip(("x", "y", "z"), map(float, values)))
    return {
        "pose": {"xyz": vector(pose["xyz"]), "rpy": vector(pose["rpy"])},
        "scale": vector(item.get("scale", [1.0, 1.0, 1.0])),
    }


def test_unchanged_fixture_overlay_uses_referenced_asset_footprint(scene, tmp_path):
    output = tmp_path / "scene.json"
    _export(scene, output)
    payload = json.loads(output.read_text(encoding="utf-8"))
    environment = yaml.safe_load((scene / "environment.yaml").read_text(encoding="utf-8"))
    selected_zone = environment["task"]["place"]["target_ref"]
    zone_source = next(zone for zone in environment["task_zones"] if zone["id"] == selected_zone)

    zone = _by_id(payload, zone_source["layout_item_ref"])
    asset = _by_id(payload, zone_source["target_ref"])
    assert zone["target_ref"] == asset["id"]
    assert zone["dimensions"][:2] == asset["dimensions"][:2]
    assert 0 < zone["dimensions"][2] <= 0.01


def test_validated_destination_edit_regenerates_overlay_pose_and_footprint(scene, tmp_path):
    output_dir = tmp_path / "web"
    output_dir.mkdir()
    before_path = output_dir / "ur5_2f_test.before.web_scene.json"
    _export(scene, before_path)
    before = json.loads(before_path.read_text(encoding="utf-8"))
    old = _transform(_by_id(before, "target_bin_default"))
    updated = json.loads(json.dumps(old))
    updated["pose"]["xyz"].update(x=0.71, y=-0.19, z=0.24)
    updated["pose"]["rpy"].update(x=0.04, y=-0.08, z=0.37)
    patch = {
        "schema_version": "workcell_studio_web_scene_edit_patch/v1",
        "scene_id": "ur5_2f_test",
        "source_scene_schema_version": before["schema_version"],
        "created_at": "2026-07-29T00:00:00Z",
        "created_by": "static_web_viewer",
        "provenance": {"source_web_scene_file": before_path.name},
        "edits": [{
            "item_id": "target_bin_default",
            "operation": "update_transform",
            "editable_required": True,
            "locked_required": False,
            "old_transform": old,
            "new_transform": updated,
        }],
    }
    patch_path = output_dir / "edit_patch.json"
    patch_path.write_text(json.dumps(patch), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(WORKFLOW), "--scene", str(scene), "--patch", str(patch_path),
         "--output-dir", str(output_dir), "--write"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    assert "persistence verification result: PASS" in result.stdout
    after = json.loads((output_dir / "ur5_2f_test.after.web_scene.json").read_text(encoding="utf-8"))
    destination = _by_id(after, "target_bin_default")
    overlay = _by_id(after, "place_zone_default")
    assert _transform(destination)["pose"] == updated["pose"]
    assert _transform(overlay)["pose"] == updated["pose"]
    assert overlay["dimensions"][:2] == destination["dimensions"][:2]
    assert 0 < overlay["dimensions"][2] <= 0.01


def test_task_target_rebinds_overlay_to_a_second_destination_without_magic_ids(scene, tmp_path):
    environment_path = scene / "environment.yaml"
    layout_path = scene / "layout/workcell_studio_layout.yaml"
    environment = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    second_asset = {
        "id": "shipping_tote_42", "type": "target_bin", "role": "target_bin", "frame": "world",
        "pose_xyz": [0.82, 0.31, 0.27], "pose_rpy": [0.03, -0.06, 0.48],
        "dimensions": [0.46, 0.29, 0.22], "layout_item_ref": "shipping_tote_42",
    }
    # The canonical fixture intentionally mirrors assets in both supported forms.
    environment["assets"].append(dict(second_asset))
    environment["environment"]["assets"].append(dict(second_asset))
    environment["task_zones"].append({
        "id": "shipping_drop_42", "type": "place_zone", "role": "place", "frame": "world",
        "shape": "box", "pose_xyz": [0, 0, 0], "pose_rpy": [0, 0, 0],
        "dimensions": [0.1, 0.1, 0.005], "target_ref": "shipping_tote_42",
        "layout_item_ref": "place_zone_default",
    })
    environment["task"]["place"]["target_ref"] = "shipping_drop_42"
    layout["items"].append({
        "id": "shipping_tote_42", "type": "target_bin", "role": "target_bin", "editable": True,
        "locked": False, "source_layer": "editable_layout",
        "pose": {"xyz": second_asset["pose_xyz"], "rpy": second_asset["pose_rpy"]},
        "dimensions": second_asset["dimensions"], "geometry_type": "box",
    })
    environment_path.write_text(yaml.safe_dump(environment, sort_keys=False), encoding="utf-8")
    layout_path.write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")

    output = tmp_path / "rebound.json"
    _export(scene, output)
    payload = json.loads(output.read_text(encoding="utf-8"))
    destination = _by_id(payload, "shipping_tote_42")
    overlay = _by_id(payload, "place_zone_default")
    assert overlay["target_ref"] == destination["id"]
    assert overlay["pose"] == destination["pose"]
    assert overlay["dimensions"][:2] == destination["dimensions"][:2]


@pytest.mark.parametrize("bad_target", [None, "asset_that_does_not_exist"])
def test_unresolved_selected_zone_target_is_blocking_and_preserves_all_files(scene, tmp_path, bad_target):
    environment_path = scene / "environment.yaml"
    layout_path = scene / "layout/workcell_studio_layout.yaml"
    environment = yaml.safe_load(environment_path.read_text(encoding="utf-8"))
    selected = environment["task"]["place"]["target_ref"]
    zone = next(zone for zone in environment["task_zones"] if zone["id"] == selected)
    if bad_target is None:
        zone.pop("target_ref")
        unresolved = "<target_ref>"
    else:
        zone["target_ref"] = bad_target
        unresolved = bad_target
    environment_path.write_text(yaml.safe_dump(environment, sort_keys=False), encoding="utf-8")
    output = tmp_path / "existing.web_scene.json"
    output.write_bytes(b"pre-existing output must survive\n")
    before = {path: path.read_bytes() for path in (environment_path, layout_path, output)}

    result = _export(scene, output, check=False)

    assert result.returncode != 0
    assert unresolved in result.stderr
    assert "environment.yaml" in result.stderr
    assert {path: path.read_bytes() for path in before} == before


def test_prebuilt_viewer_bundle_was_not_rebuilt_for_python_test_change():
    assert hashlib.sha256(BUNDLE.read_bytes()).hexdigest() == BUNDLE_SHA256
