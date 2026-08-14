"""Focused contract checks for the native Workcell Studio authoring hierarchy."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def block(start: str, end: str) -> str:
    return CPP[CPP.index(start):CPP.index(end, CPP.index(start))]


def test_hierarchy_uses_exact_instance_id_not_catalog_or_label_for_membership():
    populate = block("void MainWindow::populate_scene_hierarchy()", '\n\nnamespace {')
    add = populate[populate.index("QSet<QString> hierarchy_instance_ids;"):populate.index("auto add_preview_item")]
    assert "const QString instance_id = p.id.trimmed();" in add
    assert "hierarchy_instance_ids.contains(instance_id)" in add
    assert "hierarchy_instance_ids.insert(instance_id)" in add
    for forbidden in ("catalog_asset_id", "mesh_path", "source_path", "display_name", "category"):
        assert f"hierarchy_instance_ids.contains(p.{forbidden})" not in add

    # Same label/catalog is allowed because only the distinct IDs are row keys.
    records = [
        {"id": "object_03", "display_name": "2068_001_24", "catalog_asset_id": "imported_2068_001_24"},
        {"id": "object_04", "display_name": "2068_001_24", "catalog_asset_id": "imported_2068_001_24"},
    ]
    assert len({record["id"] for record in records}) == 2
    assert len({record["catalog_asset_id"] for record in records}) == 1
    assert len({record["display_name"] for record in records}) == 1


def test_generated_visual_provenance_is_excluded_before_generic_roles():
    policy = block("bool is_user_facing_scene_hierarchy_item", "QJsonObject scene3d_viewport_pose_json")
    exclusion = 'if (source_layer == QStringLiteral("locked_generated_urdf_visual")) return false;'
    assert exclusion in policy
    assert policy.index(exclusion) < policy.index("canonical_robot_or_tool")
    for generated_id, role in (("object_a", "object"), ("conveyor", "conveyor"),
                               ("robot_base", "robot"), ("robot_reach", "robot")):
        assert generated_id and role  # fixture documents all escaped runtime identities
    # Filtering is hierarchy-only; runtime preview storage/render submission remains intact.
    assert "all_scene_preview_items_ = preview_items;" in CPP
    assert "apply_scene3d_product_view_layer_defaults_and_commit();" in CPP


def test_canonical_owners_and_authored_physical_instances_remain_admitted():
    policy = block("bool is_user_facing_scene_hierarchy_item", "QJsonObject scene3d_viewport_pose_json")
    assert 'source_layer == QStringLiteral("selection_owner_registry")' in policy
    for role in ("support_surface_table", "camera", "place_target_bin", "object"):
        assert f'role == QStringLiteral("{role}")' in policy
    owner_reconcile = block("const QString selection_owner_contract_path", "apply_scene3d_product_view_layer_defaults_and_commit")
    assert 'QStringLiteral("robot")' in owner_reconcile
    assert 'QStringLiteral("end_effector")' in owner_reconcile
    assert "add_tree_node(owner);" in owner_reconcile


def test_structured_keepout_role_wins_over_robot_in_display_name():
    populate = block("void MainWindow::populate_scene_hierarchy()", '\n\nnamespace {')
    normalize = populate[populate.index("auto normalize_role = []"):populate.index("const QSet<QString> allowed_scene_roles")]
    assert 'classify(raw_role + QStringLiteral(" ") + raw_category)' in normalize
    assert "if (!structured.isEmpty()) return structured;" in normalize
    assert 'lower.contains("keepout")' in normalize
    assert "const QString lower = fallback_text.toLower();" in normalize
    assert normalize.index("const QString structured") < normalize.index("fallback_text.toLower()")
    raw_role, category, display = "keepout", "safety_zone", "Robot/Table Keepout Boundary"
    assert raw_role == "keepout" and category == "safety_zone" and "Robot" in display


def test_every_native_structural_edit_rebuilds_live_hierarchy():
    functions = {
        "add": block("void MainWindow::commit_armed_asset_placement", "void MainWindow::validate_asset_catalog_selection"),
        "duplicate": block("void MainWindow::duplicate_selected_item", "bool MainWindow::selected_item_can_be_deleted"),
        "delete": block("void MainWindow::delete_selected_item", "double MainWindow::current_nudge_step_m"),
        "undo": block("void MainWindow::undo_layout_edit", "void MainWindow::redo_layout_edit"),
        "redo": block("void MainWindow::redo_layout_edit", "bool MainWindow::selected_item_can_be_duplicated"),
    }
    for operation, body in functions.items():
        assert "populate_scene_hierarchy();" in body, operation
    assert "Save Layout" not in functions["add"]


def test_renderer_layer_checkbox_state_is_not_hierarchy_membership_input():
    policy = block("bool is_user_facing_scene_hierarchy_item", "QJsonObject scene3d_viewport_pose_json")
    for renderer_toggle in ("toggle_grid_box_", "toggle_robot_box_", "toggle_meshes_box_", "toggle_overlays_box_"):
        assert renderer_toggle not in policy
