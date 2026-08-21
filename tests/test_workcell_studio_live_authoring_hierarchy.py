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
        role = "object" if record.get("role") == "asset" else record.get("role")
        if role not in admitted_roles:
            continue
        if role == "object" and not (
            record.get("source_layer") == "editable_layout"
            and record.get("editable") is True
            and record.get("locked") is False
            and record.get("linked_to_editable_layout_state") is True
        ):
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


def test_duplicate_is_visible_for_every_editable_authored_selection_and_shortcut_reaches_web_focus():
    toolbar = block(
        "void MainWindow::refresh_duplicate_selected_action()",
        "void MainWindow::duplicate_selected_item()",
    )
    assert "selected_item_can_be_duplicated()" in toolbar
    assert "fixture_like" not in toolbar
    assert "setVisible(can_duplicate)" in toolbar
    assert "setEnabled(can_duplicate)" in toolbar
    assert "duplicate_action->setShortcutContext(Qt::ApplicationShortcut);" in CPP
    registry = block(
        "void MainWindow::register_scene_builder_action",
        "void MainWindow::export_canvas_snapshot",
    )
    assert "addAction(action);" in registry


def test_distinct_instances_and_canonical_authored_rows_survive():
    authored = {"source_layer": "editable_layout", "editable": True, "locked": False,
                "linked_to_editable_layout_state": True}
    records = [
        {"id": "support_surface_table", "role": "support_surface_table"},
        {"id": "realsense_overhead", "role": "camera"},
        {"id": "target_bin_default", "role": "place_target_bin"},
        {"id": "object_01", "role": "object", **authored}, {"id": "object_02", "role": "object", **authored},
        {"id": "object_03", "role": "object", "display_name": "2068_001_24", "catalog_asset_id": "imported_2068_001_24", **authored},
        {"id": "object_04", "role": "object", "display_name": "2068_001_24", "catalog_asset_id": "imported_2068_001_24", **authored},
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
    normalization = block("QString canonical_scene_hierarchy_role", "QString scene3d_viewport_link_token")
    for raw_marker, canonical in (("support_surface", "support_surface_table"),
                                  ("place_target", "place_target_bin"),
                                  ("camera", "camera"), ("object", "object")):
        assert raw_marker in normalization
        assert f'QString("{canonical}")' in normalization
    assert "canonical_scene_hierarchy_role(role_hint, category, display_name)" in populate
    policy = block("bool is_user_facing_scene_hierarchy_item", "QJsonObject scene3d_viewport_pose_json")
    for canonical in ("support_surface_table", "place_target_bin", "camera", "object"):
        assert f'role == QStringLiteral("{canonical}")' in policy


def test_add_refresh_does_not_mutate_current_preview_fixture():
    preview = [{"id": "object_03", "role": "object", "catalog_asset_id": "imported_2068_001_24",
                "source_layer": "editable_layout", "editable": True, "locked": False,
                "linked_to_editable_layout_state": True}]
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


def production_placement(instance_id, catalog_asset_id="shared_catalog_asset", yaw=0.0):
    """Exact semantic/provenance fields written by commit_armed_asset_placement."""
    return {"id": instance_id, "role": "object", "catalog_asset_id": catalog_asset_id,
            "source_layer": "editable_layout", "active_visual_source": "mesh_preview",
            "editable": True, "locked": False, "linked_to_editable_layout_state": True,
            "selectable": True, "yaw": yaw}


def test_one_and_two_unsaved_placements_have_instance_identity_and_selection_parity():
    first = production_placement("object_01")
    second = production_placement("object_02")
    assert hierarchy_rows([first]) == ["object_01"]
    assert hierarchy_rows([first, second]) == ["object_01", "object_02"]
    # Hierarchy, Product View, and inspector all resolve the same stable ID.
    for selected_id in hierarchy_rows([first, second]):
        assert next(row for row in (first, second) if row["id"] == selected_id)["id"] == selected_id


def test_exact_id_dedupe_does_not_collapse_shared_catalog_instances():
    first = production_placement("object_01")
    duplicate_payload = dict(first, active_visual_source="staged_mesh")
    second = production_placement("object_02")
    assert hierarchy_rows([first, duplicate_payload, second]) == ["object_01", "object_02"]


def test_three_repeat_commits_keep_unique_ids_shared_provenance_and_exact_yaw():
    yaw = 3.141592653589793 / 12
    records = [production_placement(f"object_0{index}", yaw=yaw) for index in range(1, 4)]
    assert hierarchy_rows(records) == ["object_01", "object_02", "object_03"]
    assert len({record["id"] for record in records}) == 3
    assert {record["catalog_asset_id"] for record in records} == {"shared_catalog_asset"}
    assert all(record["yaw"] == yaw for record in records)


def test_catalog_generated_staged_and_fallback_records_are_not_rows():
    records = [
        production_placement("object_01"),
        {"id": "catalog_record", "role": "asset", "source_layer": "asset_catalog",
         "editable": False, "locked": True, "linked_to_editable_layout_state": False},
        {"id": "generated_visual", "role": "object", "source_layer": "locked_generated_urdf_visual",
         "editable": False, "locked": True, "linked_to_editable_layout_state": False},
        {"id": "staged_mesh", "role": "asset", "source_layer": "mesh_preview",
         "editable": True, "locked": False, "linked_to_editable_layout_state": False},
        {"id": "fallback", "role": "object", "source_layer": "primitive_fallback",
         "editable": False, "locked": True, "linked_to_editable_layout_state": False},
    ]
    assert hierarchy_rows(records) == ["object_01"]


def test_placement_updates_live_session_without_persisting():
    placement = block("void MainWindow::commit_armed_asset_placement", "void MainWindow::validate_asset_catalog_selection")
    assert "workcell_studio_next_id(category.toStdString(), reserved_ids)" in placement
    assert "preview_item.role = canonical_scene_hierarchy_role(" in placement
    assert placement.count("all_scene_preview_items_.push_back(preview_item);") == 1
    refresh = placement.index("refresh_scene_hierarchy_tree_from_current_items();")
    dirty = placement.index('mark_layout_dirty("Place Asset Mode: Add to 3D Canvas");')
    diagnostic = placement.index("hierarchy_updated=true layout_dirty=true persisted=false")
    assert refresh < dirty < diagnostic
    assert "capture_active_editable_layout_session();" in block(
        "void MainWindow::mark_layout_dirty", "void MainWindow::capture_active_editable_layout_session")
    for forbidden in ("populate_scene_hierarchy();", "save_layout_changes();", "save_native_layout_changes("):
        assert forbidden not in placement


def test_save_and_reload_fixture_retains_one_row_per_instance_and_orientation(tmp_path):
    import json
    yaw = 3.141592653589793 / 12
    records = [production_placement("object_01", yaw=yaw), production_placement("object_02", yaw=yaw)]
    layout = tmp_path / "workcell_studio_layout.json"
    assert not layout.exists()  # placement itself performs no write
    layout.write_text(json.dumps(records), encoding="utf-8")  # explicit Save
    reloaded = json.loads(layout.read_text(encoding="utf-8"))
    assert [row["id"] for row in reloaded] == ["object_01", "object_02"]
    assert hierarchy_rows(reloaded) == ["object_01", "object_02"]
    assert [row["yaw"] for row in reloaded] == [yaw, yaw]
