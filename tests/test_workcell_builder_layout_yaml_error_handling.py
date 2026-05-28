from pathlib import Path
import re

MAINWINDOW_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()


def _derive_layout_state_model_body() -> str:
    match = re.search(
        r'static LayoutStateModel derive_layout_state_model\([^)]*\)\s*\{(?P<body>.*?)\n\}\n\n\nstatic QMap',
        MAINWINDOW_CPP,
        re.S,
    )
    assert match, 'derive_layout_state_model body should be statically discoverable'
    return match.group('body')


def test_derive_layout_state_model_maps_non_yaml_exceptions_to_invalid_layout_yaml():
    body = _derive_layout_state_model_body()

    assert 'catch (const YAML::Exception &)' in body
    assert 'catch (const std::exception &)' in body
    assert body.count('return LayoutStateModel::INVALID_LAYOUT_YAML;') >= 2


def test_invalid_layout_yaml_is_reported_as_non_crashing_warning_state():
    body = _derive_layout_state_model_body()

    assert 'YAML::LoadFile(layout.string())' in body
    assert 'return LayoutStateModel::INVALID_LAYOUT_YAML;' in body
    assert 'case LayoutStateModel::INVALID_LAYOUT_YAML:' in MAINWINDOW_CPP
    assert 'Save Layout Needed: invalid layout YAML' in MAINWINDOW_CPP
    assert 'layout_status = SceneWorkflowStepStatus::Warning' in MAINWINDOW_CPP
