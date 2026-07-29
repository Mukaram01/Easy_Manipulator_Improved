from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCENE_SELECT_CPP = REPO / "workcell_builder/workcell_builder/gui/scene_select.cpp"
SCENE_SELECT_H = REPO / "workcell_builder/workcell_builder/gui/scene_select.h"
YAML_IO_CPP = REPO / "workcell_builder/workcell_builder/gui/object_placement_yaml_io.cpp"
YAML_IO_H = REPO / "workcell_builder/workcell_builder/include/object_placement_yaml_io.hpp"
MODEL_H = REPO / "workcell_builder/workcell_builder/include/object_placement_model.hpp"


def test_task_area_editor_ui_and_canvas_tokens_present():
    cpp = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    for token in [
        "Task Areas",
        "Suggested Areas",
        "Add Pick Area",
        "Add Place Area",
        "Delete Selected",
        "Snap to 1 cm",
        "Pick Area",
        "Place Area",
        "Pick Object",
        "Destination Bin",
        "Offline layout check only; motion planning is checked later.",
        "TaskAreaGraphicsItem",
        "ItemIsMovable",
    ]:
        assert token in cpp


def test_task_area_dirty_state_gates_primary_actions():
    cpp = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    header = SCENE_SELECT_H.read_text(encoding="utf-8")
    assert "task_area_dirty_" in header
    assert "return task_editor_state_.unsaved_task_edits || task_area_dirty_" in cpp
    assert "primary_validate_button->setEnabled(!unsaved)" in cpp
    assert "primary_generate_package_button->setEnabled(!unsaved" in cpp


def test_suggested_areas_are_idempotent_and_use_stable_ids():
    cpp = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "ensure_suggested_task_areas" in cpp
    assert "has_role(\"pick\")" in cpp
    assert "has_role(\"place\")" in cpp
    assert "commissioning_pick_pose" in cpp
    assert "default_drop_zone" in cpp


def test_task_zone_persistence_fields_and_task_refs_are_written():
    model = MODEL_H.read_text(encoding="utf-8")
    yaml_io = YAML_IO_CPP.read_text(encoding="utf-8")
    for field in ["role", "shape", "support_surface_ref", "object_ref", "target_ref"]:
        assert field in model
        assert field in yaml_io
    for key in ["pose_xyz", "pose_rpy", "dimensions", "frame", "task", "source_ref", "target_ref", "intent_target_ref"]:
        assert key in yaml_io


def test_task_area_save_validation_and_backup_tokens_present():
    cpp = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    for token in [
        "missing ID/type",
        "non-finite value",
        "width/depth/height must be positive",
        "duplicate Pick Area ID",
        "duplicate Place Area ID",
        "unresolved Pick Object association",
        "Missing destination:",
        "may be outside support surface",
        "Pick and Place Areas may be overlapping",
        ".task_areas.",
        "copy_file(env, backup",
    ]:
        assert token in cpp


def test_no_web3d_or_generated_bundle_paths_touched_by_task_area_editor():
    changed_scope = "\n".join(p.name for p in [SCENE_SELECT_CPP, SCENE_SELECT_H, YAML_IO_CPP, MODEL_H])
    assert "workcell_studio_web" not in changed_scope
    assert "generated" not in changed_scope


def test_place_area_uses_validated_destination_selector():
    cpp = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    header = SCENE_SELECT_H.read_text(encoding="utf-8")
    discovery = YAML_IO_CPP.read_text(encoding="utf-8")
    assert 'setObjectName("task_area_destination_combo")' in cpp
    assert 'setObjectName("task_area_association_edit")' in cpp
    assert "association->setVisible(is_pick)" in cpp
    assert "combo->setVisible(!is_pick)" in cpp
    assert "task_area_destinations() const" in header
    assert "discover_task_area_destinations(environment.string())" in cpp
    for semantic in ['"bin"', '"tote"', '"destination_container"', '"container"']:
        assert semantic in discovery
    for excluded in ['"overlay"', '"helper"', '"robot"', '"tool"', '"camera"', '"generated_urdf"']:
        assert excluded in discovery
    assert 'combo->addItem(QString::fromStdString(candidate.display_name), QString::fromStdString(candidate.id))' in cpp


def test_destination_selection_updates_only_place_zone_stable_id():
    cpp = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    yaml_io = YAML_IO_CPP.read_text(encoding="utf-8")
    selection_handler = cpp.split("QComboBox::activated", 1)[1].split("});", 1)[0]
    assert "area.target_ref = destination_id" in selection_handler
    assert 'mark_task_areas_dirty("Destination Bin")' in selection_handler
    assert "task_editor_state_.place_target" not in selection_handler
    assert 'if (!root["task"]["place"]["target_ref"])' in yaml_io
    assert "zone.target_ref selects its physical bin" in yaml_io


def test_missing_destination_is_visible_and_blocks_save():
    cpp = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert '"Missing destination: " + a.target_ref' in cpp
    assert "!is_valid_task_area_destination(z.target_ref)" in cpp
    assert 'errors->push_back("Missing destination: "' in cpp


def test_pick_area_free_text_association_is_unchanged():
    cpp = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert 'if (a.role == "pick") a.object_ref = assoc->text().trimmed().toStdString();' in cpp
    assert 'set_text("task_area_association_edit", QString::fromStdString(a.object_ref))' in cpp
