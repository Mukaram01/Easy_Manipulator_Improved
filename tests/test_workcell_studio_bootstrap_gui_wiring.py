from pathlib import Path
import re


SOURCE = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
HEADER = Path("workcell_builder/workcell_builder/include/workcell_studio_canvas_model.hpp").read_text(encoding="utf-8")


def test_create_editable_layout_button_uses_shared_model_layer_bootstrap_helper():
    assert "bootstrap_editable_layout_from_trusted_canonical_yaml" in HEADER
    create_match = re.search(
        r"void MainWindow::create_starter_layout_from_preview\(\)\n\{(?P<body>.*?)\n\}\n\nvoid MainWindow::revert_layout_changes",
        SOURCE,
        re.DOTALL,
    )
    assert create_match is not None
    body = create_match.group("body")
    assert "workcell_builder::bootstrap_editable_layout_from_trusted_canonical_yaml(s.scene_dir, s.scene_name)" in body
    assert "workcell_builder::build_starter_layout_entries_from_preview(model)" not in body
    assert "Create editable layout from preview" in SOURCE
    assert "connect(create_starter_layout_button_, &QPushButton::clicked, this, &MainWindow::create_starter_layout_from_preview)" in SOURCE


def test_invalid_layout_yaml_token_path_stays_warning_not_crash():
    derive_match = re.search(
        r"static LayoutStateModel derive_layout_state_model\([^)]*\)\s*\{(?P<body>.*?)\n\}\n\n\nstatic QMap",
        SOURCE,
        re.DOTALL,
    )
    assert derive_match is not None
    body = derive_match.group("body")
    assert "YAML::LoadFile(layout.string())" in body
    assert "catch (const YAML::Exception &)" in body
    assert "catch (const std::exception &)" in body
    assert "return LayoutStateModel::INVALID_LAYOUT_YAML;" in body
    assert "case LayoutStateModel::INVALID_LAYOUT_YAML:" in SOURCE
    assert "layout_status = SceneWorkflowStepStatus::Warning" in SOURCE
