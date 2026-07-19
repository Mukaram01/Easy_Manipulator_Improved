from pathlib import Path
import re

REPO_ROOT = Path(__file__).resolve().parents[1]
MAINWINDOW_CPP = REPO_ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
MAINWINDOW_H = REPO_ROOT / "workcell_builder/workcell_builder/gui/mainwindow.h"


def _refresh_body() -> str:
    text = MAINWINDOW_CPP.read_text(encoding="utf-8")
    m = re.search(r"void MainWindow::refresh_scene_workflow_rail\(\)\n\{(?P<body>.*?)\n\}\n\nvoid MainWindow::refresh_run_next_menu", text, re.S)
    assert m, "refresh_scene_workflow_rail body not found"
    return m.group("body")


def test_workflow_rail_uses_compact_rows_with_details_tooltip_and_existing_recommendation():
    body = _refresh_body()

    assert "scene_workflow_compact_summary(step)" in body
    assert "scene_workflow_details_tooltip(steps)" in body
    assert "scene_workflow_rail_label_->setToolTip" in body
    assert "format_scene_builder_status_text(step.detail).toHtmlEscaped()" not in body
    assert "recommendation.label" in body
    assert "resolve_recommended_workflow_actions()" in body
    assert "resolve_recommended_workflow_action()" not in body

    assert body.count("Details") <= 1
    assert "<b>%1</b><br/>%2" in body


def test_workflow_status_labels_preserve_warnings_and_avoid_missing_or_ready():
    text = MAINWINDOW_CPP.read_text(encoding="utf-8")
    status_fn = re.search(r"QString MainWindow::scene_workflow_status_text.*?\n\}", text, re.S).group(0)

    assert 'SceneWorkflowStepStatus::Warning: return "Warnings"' in status_fn
    assert 'return "Missing"' not in status_fn
    assert 'return "Ready"' not in status_fn
    for label in ("Done", "Needed", "Warnings", "Blocked"):
        assert f'return "{label}"' in status_fn


def test_workflow_card_has_sensible_min_width_and_long_details_not_inline():
    text = MAINWINDOW_CPP.read_text(encoding="utf-8")
    header = MAINWINDOW_H.read_text(encoding="utf-8")

    assert "workflow_card->setMinimumWidth(260);" in text
    assert "scene_workflow_rail_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);" in text
    assert "QString scene_workflow_compact_summary" in header
    assert "QString scene_workflow_details_tooltip" in header
