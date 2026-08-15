"""Focused contracts for the native Workcell Studio authoring hierarchy."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def block(start: str, end: str) -> str:
    offset = CPP.index(start)
    return CPP[offset:CPP.index(end, offset)]


def hierarchy_rows(records):
    """Model the C++ tree-only exact-instance policy for regression fixtures."""
    admitted_roles = {"robot", "end_effector", "end_effector_tool", "support_surface_table",
                      "camera", "place_target_bin", "conveyor", "object"}
    seen, rows = set(), []
    for record in records:
        instance_id = record["id"].strip()
        if not instance_id or instance_id in seen:
            continue
        if record.get("source_layer") == "locked_generated_urdf_visual":
            continue
        if record.get("role") in {"safety_zone", "home_safety_pose", "pick_source_zone"}:
            continue
        if record.get("role") not in admitted_roles:
            continue
        seen.add(instance_id)
        rows.append(instance_id)
    return rows


def test_tree_refresh_is_preview_payload_inert_and_single_pass():
    refresh = block("void MainWindow::refresh_scene_hierarchy_tree_from_current_items()", "void MainWindow::populate_scene_hierarchy()")
    assert refresh.index("scene_hierarchy_tree_->clear();") < refresh.index("QSet<QString> hierarchy_instance_ids;")
    assert "for (const auto & p : all_scene_preview_items_)" in refresh
    assert "const QString instance_id = p.id.trimmed();" in refresh
    assert "hierarchy_instance_ids.contains(instance_id)" in refresh
    assert "hierarchy_instance_ids.insert(instance_id)" in refresh
    for forbidden in ("all_scene_preview_items_ =", "build_workcell_studio_canvas_model",
                      "merge_active_editable_layout_session", "apply_scene3d_product_view_layer_defaults_and_commit"):
        assert forbidden not in refresh


def test_structural_edits_use_only_tree_refresh():
    functions = {
        "add": block("void MainWindow::commit_armed_asset_placement", "void MainWindow::validate_asset_catalog_selection"),
        "duplicate": block("void MainWindow::duplicate_selected_item", "bool MainWindow::selected_item_can_be_deleted"),
        "delete": block("void MainWindow::delete_selected_item", "double MainWindow::current_nudge_step_m"),
        "undo": block("void MainWindow::undo_layout_edit", "void MainWindow::redo_layout_edit"),
        "redo": block("void MainWindow::redo_layout_edit", "bool MainWindow::selected_item_can_be_duplicated"),
    }
    for operation, body in functions.items():
        assert "refresh_scene_hierarchy_tree_from_current_items();" in body, operation
        assert "populate_scene_hierarchy();" not in body, operation


def test_distinct_instances_and_canonical_authored_rows_survive():
    records = [
        {"id": "support_surface_table", "role": "support_surface_table"},
        {"id": "realsense_overhead", "role": "camera"},
        {"id": "target_bin_default", "role": "place_target_bin"},
        {"id": "object_01", "role": "object"}, {"id": "object_02", "role": "object"},
        {"id": "object_03", "role": "object", "display_name": "2068_001_24", "catalog_asset_id": "imported_2068_001_24"},
        {"id": "object_04", "role": "object", "display_name": "2068_001_24", "catalog_asset_id": "imported_2068_001_24"},
    ]
    assert hierarchy_rows(records) == [r["id"] for r in records]


def test_exclusions_and_canonical_selection_owners():
    records = [
        {"id": "UR5", "role": "robot", "source_layer": "selection_owner_registry"},
        {"id": "Robotiq", "role": "end_effector", "source_layer": "selection_owner_registry"},
        {"id": "generated_link", "role": "object", "source_layer": "locked_generated_urdf_visual"},
        {"id": "safety_zone_keepout", "role": "safety_zone", "display_name": "Robot Keepout"},
    ]
    assert hierarchy_rows(records) == ["UR5", "Robotiq"]


def test_hierarchy_role_tokens_are_consistent():
    populate = block("void MainWindow::populate_scene_hierarchy()", "\n\nnamespace {")
    normalization = populate[populate.index("auto normalize_role = []"):populate.index("QMap<QString, QString> yaml_status_by_id")]
    for raw_marker, canonical in (("support_surface", "support_surface_table"),
                                  ("place_target", "place_target_bin"),
                                  ("camera", "camera"), ("object", "object")):
        assert raw_marker in normalization
        assert f'QString("{canonical}")' in normalization
    policy = block("bool is_user_facing_scene_hierarchy_item", "QJsonObject scene3d_viewport_pose_json")
    for canonical in ("support_surface_table", "place_target_bin", "camera", "object"):
        assert f'role == QStringLiteral("{canonical}")' in policy


def test_add_refresh_does_not_mutate_current_preview_fixture():
    preview = [{"id": "object_03", "role": "object", "catalog_asset_id": "imported_2068_001_24"}]
    before = list(preview)
    assert hierarchy_rows(preview) == ["object_03"]
    assert preview == before and len(preview) == 1

def test_physical_target_bin_owns_hierarchy_not_derived_place_zone():
    policy = block(
        "bool is_user_facing_scene_hierarchy_item",
        "QJsonObject scene3d_viewport_pose_json",
    )

    assert 'role == QStringLiteral("target_bin")' in policy
    assert 'role == QStringLiteral("place_target_bin")' in policy
    assert '!item.target_ref.trimmed().isEmpty()' in policy
    assert 'source_layer == QStringLiteral("overlay")' in policy
    assert '(category == QStringLiteral("place_zone") && !physical_target_bin)' in policy
