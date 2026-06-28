from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCENE_PREVIEW_CPP = REPO / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
SCENE3D_VIEWPORT_CPP = REPO / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp"
SCENE3D_ASSEMBLY_CPP = REPO / "workcell_builder/workcell_builder/gui/scene3d_candidate_assembly.cpp"
MAINWINDOW_CPP = REPO / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
UR5_SCENE = REPO / "scenes/ur5_2f_test"


@dataclass
class PreviewItem:
    source_layer: str = ""
    active_visual_source: str = ""
    status: str = "ok"
    warnings: list[str] = field(default_factory=list)
    mesh_load_warning: str = ""
    source_path_resolution_outcome: str = ""
    id: str = ""
    mesh_available: bool = False
    has_mesh_metadata: bool = False


def _token(value: str) -> str:
    return value.strip().lower()


def product_layer_defaults(items: list[PreviewItem]) -> dict[str, bool]:
    primitive_fallback_count = 0
    missing_mesh_count = 0
    unresolved_package_uri_count = 0
    unsupported_extension_count = 0
    fallback_warning_count = 0
    authoritative_generated_mesh_count = 0

    for item in items:
        source_layer = _token(item.source_layer)
        visual_source = _token(item.active_visual_source)
        combined = "|".join(
            [
                source_layer,
                visual_source,
                _token(item.status),
                "|".join(w.lower() for w in item.warnings),
                _token(item.mesh_load_warning),
                _token(item.source_path_resolution_outcome),
            ]
        )
        if source_layer == "primitive_fallback" or visual_source == "primitive_fallback":
            primitive_fallback_count += 1
        if any(
            marker in combined
            for marker in (
                "missing mesh",
                "missing_source_path",
                "mesh unavailable",
                "mesh unresolved",
                "unresolved",
            )
        ):
            missing_mesh_count += 1
        if "unresolved_package_uri" in combined or "package uri unresolved" in combined:
            unresolved_package_uri_count += 1
        if "unsupported" in combined and ("extension" in combined or "format" in combined):
            unsupported_extension_count += 1
        if "fallback" in combined and any(marker in combined for marker in ("missing", "unavailable", "unresolved")):
            fallback_warning_count += 1
        if (
            source_layer in ("locked_generated_urdf_visual", "generated_urdf_visual")
            and (
                item.mesh_available
                or item.has_mesh_metadata
                or item.id.startswith("generated_urdf_fallback::")
                or item.id.startswith("urdf_visual_")
            )
        ):
            authoritative_generated_mesh_count += 1

    return {
        "editable_layout": True,
        "mesh_preview": True,
        "locked_generated_urdf_visual": True,
        "overlay": False,
        "primitive_fallback": authoritative_generated_mesh_count == 0 and (primitive_fallback_count + missing_mesh_count) > 0,
        "warning": False,
    }


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


def test_loaded_scene_payload_product_defaults_suppress_empty_diagnostics_layers() -> None:
    defaults = product_layer_defaults(
        [
            PreviewItem(source_layer="locked_generated_urdf_visual", mesh_available=True),
            PreviewItem(active_visual_source="mesh_preview"),
            PreviewItem(source_layer="editable_layout"),
        ]
    )

    assert defaults == {
        "locked_generated_urdf_visual": True,
        "mesh_preview": True,
        "editable_layout": True,
        "primitive_fallback": False,
        "overlay": False,
        "warning": False,
    }

    assembly_cpp = SCENE3D_ASSEMBLY_CPP.read_text()
    assert "out.locked_generated_urdf_visual = true;" in assembly_cpp
    assert "out.mesh_preview = true;" in assembly_cpp
    assert "out.editable_layout = true;" in assembly_cpp
    assert "out.primitive_fallback = authoritative_generated_mesh_count == 0" in assembly_cpp
    assert "out.overlay = false;" in assembly_cpp
    assert "out.warning = false;" in assembly_cpp


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
    refresh_helper = mainwindow_cpp.split("void MainWindow::refresh_scene_builder_selected_scene_ui()", 1)[1]
    refresh_helper = refresh_helper.split("void MainWindow::refresh_create_starter_layout_action()", 1)[0]

    assert "out.scene_name = QString::fromStdString(scene.scene_name);" in metadata_helper
    assert "out.scene_path = QString::fromStdString(scene.scene_dir.string());" in metadata_helper
    assert "scene.has_launch_demo" in metadata_helper
    assert "launch/demo.launch.py present" in metadata_helper
    assert '{"robot", "model"}' in metadata_helper
    assert "cell_definition.yaml" in metadata_helper
    assert "metadata.scene_name" in refresh_helper
    assert "metadata.scene_path" in refresh_helper
    assert "metadata.robot" in refresh_helper
    assert "metadata.launch" in refresh_helper


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
