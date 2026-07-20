from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCENE_PREVIEW_CPP = REPO / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
SCENE3D_VIEWPORT_CPP = REPO / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp"
SCENE3D_ASSEMBLY_CPP = REPO / "workcell_builder/workcell_builder/gui/scene3d_candidate_assembly.cpp"
MAINWINDOW_CPP = REPO / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
UR5_SCENE = REPO / "scenes/ur5_2f_test"


def compact_warning_chip_state(mesh_index: dict[str, object], counters: dict[str, int]) -> tuple[bool, str]:
    clean_mesh_index = (
        mesh_index.get("extraction_mode") == "xacro_expanded"
        and mesh_index.get("safe_for_preview") is True
        and int(mesh_index.get("missing", 0)) == 0
        and int(mesh_index.get("unresolved", 0)) == 0
        and int(mesh_index.get("fallback", 0)) == 0
    )
    clean_render_counters = all(
        int(counters.get(key, 0)) == 0
        for key in (
            "missing_geometry_count",
            "mesh_bounds_fallback_rendered_count",
            "primitive_fallback_rendered_count",
            "wireframe_fallback_count",
            "placeholder_count",
        )
    )
    if clean_mesh_index and clean_render_counters:
        return False, "3D Preview Ready"
    return True, "3D Preview Warnings · see Diagnostics"


def test_product_view_defaults_are_wired_to_shared_scene3d_layer_helper() -> None:
    assembly_cpp = SCENE3D_ASSEMBLY_CPP.read_text()
    mainwindow_cpp = MAINWINDOW_CPP.read_text()
    apply_defaults = mainwindow_cpp.split("void MainWindow::apply_scene3d_product_view_layer_defaults_and_commit()", 1)[1]
    apply_defaults = apply_defaults.split("void MainWindow::apply_scene_preview_filter()", 1)[0]

    assert "Scene3DLayerVisibilityDefaults compute_scene3d_default_layer_visibility" in assembly_cpp
    assert "const auto defaults = workcell_builder::compute_scene3d_default_layer_visibility(all_scene_preview_items_);" in apply_defaults
    assert "set_checked_blocked(preview_layer_generated_urdf_visual_box_, defaults.locked_generated_urdf_visual);" in apply_defaults
    assert "set_checked_blocked(preview_layer_primitive_fallback_box_, defaults.primitive_fallback);" in apply_defaults
    assert "set_checked_blocked(preview_layer_overlays_helpers_box_, defaults.overlay);" in apply_defaults
    assert "set_checked_blocked(preview_layer_warnings_missing_assets_box_, defaults.warning);" in apply_defaults


def test_scene_preview_widget_product_defaults_turn_off_viewport_helpers_and_warnings() -> None:
    apply_defaults = SCENE_PREVIEW_CPP.read_text().split("void ScenePreviewWidget::apply_product_view_defaults()", 1)[1]
    apply_defaults = apply_defaults.split("void ScenePreviewWidget::on_reset_view_clicked", 1)[0]

    for expected in (
        "v->debug_overlays_mode = false;",
        "v->show_warnings = false;",
        "v->show_warning_labels = false;",
        "v->show_safety = false;",
        "v->show_pick_place = false;",
        "v->show_reachability_heatmap = false;",
        "v->show_collision_warnings = false;",
        "v->show_work_envelope = false;",
        "v->show_task_route = false;",
        "v->show_approach_retreat = false;",
        "v->show_camera_fov = false;",
        "v->show_pick_coverage = false;",
        "v->show_epd_detections = false;",
        "v->show_detection_labels = false;",
        "v->mesh_preview_mode = ScenePreviewWidget::MeshPreviewMode::Auto;",
        "v->label_mode = ScenePreviewWidget::LabelMode::Selected;",
        "v->fit_include_overlays = false;",
        "v->fit_product_view();",
    ):
        assert expected in apply_defaults


def test_inspector_refresh_for_ur5_2f_test_uses_canonical_metadata_and_launch_path() -> None:
    assert UR5_SCENE.is_dir(), "ur5_2f_test fixture scene must exist"
    assert (UR5_SCENE / "cell_definition.yaml").is_file(), "robot metadata source must exist"
    assert (UR5_SCENE / "launch/demo.launch.py").is_file(), "launch path should be detected"

    cell_definition = (UR5_SCENE / "cell_definition.yaml").read_text()
    assert "ur5" in cell_definition

    mainwindow_cpp = MAINWINDOW_CPP.read_text()
    metadata_helper = mainwindow_cpp.split("static SelectedSceneMetadataSummary selected_scene_metadata_summary", 1)[1]
    metadata_helper = metadata_helper.split("static YAML::Node ensure_map_path", 1)[0]
    metadata_panel_helper = mainwindow_cpp.split("void MainWindow::refresh_selected_scene_metadata_panel()", 1)[1]
    metadata_panel_helper = metadata_panel_helper.split("void MainWindow::refresh_scene_builder_selection_state_ui()", 1)[0]

    assert "out.scene_name = QString::fromStdString(scene.scene_name);" in metadata_helper
    assert "out.scene_path = QString::fromStdString(scene.scene_dir.string());" in metadata_helper
    assert "scene.has_launch_demo" in metadata_helper
    assert "launch/demo.launch.py present" in metadata_helper
    assert '{"robot", "model"}' in metadata_helper
    assert "cell_definition.yaml" in metadata_helper
    assert "metadata.scene_name" in metadata_panel_helper
    assert "metadata.scene_path" in metadata_panel_helper
    assert "metadata.robot" in metadata_panel_helper
    assert "metadata.launch" in metadata_panel_helper


def test_warning_chip_suppressed_for_clean_xacro_expanded_preview_payload() -> None:
    show_warning, chip_text = compact_warning_chip_state(
        {"extraction_mode": "xacro_expanded", "safe_for_preview": True, "missing": 0, "unresolved": 0, "fallback": 0},
        {
            "missing_geometry_count": 0,
            "mesh_bounds_fallback_rendered_count": 0,
            "primitive_fallback_rendered_count": 0,
            "wireframe_fallback_count": 0,
            "placeholder_count": 0,
        },
    )
    assert show_warning is False
    assert chip_text == "3D Preview Ready"

    viewport_cpp = SCENE3D_VIEWPORT_CPP.read_text()
    assert "const bool concise_warning = show_warnings && has_missing_or_fallback_content;" in viewport_cpp
    assert "3D Preview Warnings · see Diagnostics" in viewport_cpp
    assert "3D Preview Ready · %1 items" in viewport_cpp
    assert "show_warning_labels && (debug_overlays_mode || !render_counters_clean)" in viewport_cpp
