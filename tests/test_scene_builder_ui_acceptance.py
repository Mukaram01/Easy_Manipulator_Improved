from __future__ import annotations

from scripts.validate_scene_builder_ui_acceptance import run_checks


def _base_fixture() -> dict[str, str]:
    return {
        "mainwindow_cpp": "\n".join(
            [
                'QSplitter *split = new QSplitter(this);',
                'split->setStretchFactor(0, 3);',
                'QToolButton *more = new QToolButton(this);',
                'more->setText("More Actions");',
                'QMenu *menu = new QMenu(this);',
                'QString previewTitle = "3D Layout Preview";',
                'QString fallback = "2D Layout";',
                'QString focus = "Focus Canvas";',
                'QString hide = "Hide";',
                'QString show = "Show";',
                'auto cmd = "ros2 launch scene demo.launch.py use_fake_hardware:=true";',
                'QString xyz = "XYZ";',
                'QString rpy = "RPY";',
                'QString x_units = "X position in metres";',
                'QString yaw_units = "Yaw in radians";',
            ]
        ),
        "preview_cpp": "Preview panel supports 2D Layout and 3D Layout Preview",
        "layout_editor_cpp": "metres radians",
    }


def test_validator_passes_with_required_tokens_present():
    checks = run_checks(_base_fixture())
    assert all(c.ok for c in checks), [f"{c.name}: {c.details}" for c in checks if not c.ok]


def test_validator_fails_with_clear_diagnostics_when_tokens_or_patterns_missing():
    broken = _base_fixture()
    broken["mainwindow_cpp"] = broken["mainwindow_cpp"].replace("Focus Canvas", "")
    broken["mainwindow_cpp"] += '\nconnect(btn, &QPushButton::click);\naction_button->setFixedWidth(120);\n'

    checks = run_checks(broken)
    failed = {c.name: c.details for c in checks if not c.ok}

    assert "Focus Canvas action wording" in failed
    assert any("Focus Canvas" in d for d in failed["Focus Canvas action wording"])
    assert "fixed-width button anti-pattern checks" in failed


def test_repository_ui_acceptance_validator_runs_on_current_sources():
    checks = run_checks()
    assert checks
    assert any(c.name == "fake hardware launch token" for c in checks)
