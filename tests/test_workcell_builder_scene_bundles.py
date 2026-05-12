from pathlib import Path

def test_bundle_api_files_exist():
    assert Path('workcell_builder/workcell_builder/include/workcell_scene_bundle.hpp').exists()
    assert Path('workcell_builder/workcell_builder/src_workcell_scene_bundle.cpp').exists()


def test_scene_select_ui_has_bundle_actions_and_tooltips():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'Export Scene Bundle' in ui
    assert 'Import Scene Bundle' in ui
    assert 'portable bundle' in ui.lower()


def test_scene_select_cpp_has_bundle_slots():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'on_export_scene_bundle_clicked' in cpp
    assert 'on_import_scene_bundle_clicked' in cpp
    assert 'export_scene_bundle(options)' in cpp
    assert 'import_scene_bundle(options)' in cpp


def test_bundle_manifest_contract_markers_present():
    cpp = Path('workcell_builder/workcell_builder/src_workcell_scene_bundle.cpp').read_text(encoding='utf-8')
    assert 'bundle_schema_version' in cpp
    assert 'required_ros_packages' in cpp
    assert 'fake_hardware_default' in cpp
    assert 'not safety certificates' in cpp
