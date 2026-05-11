from pathlib import Path


def test_object_placement_dialog_files_exist_and_wired():
    header = Path('workcell_builder/workcell_builder/include/object_placement_dialog.hpp')
    cpp = Path('workcell_builder/workcell_builder/gui/object_placement_dialog.cpp')
    cmake = Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')
    assert header.exists()
    assert cpp.exists()
    assert 'gui/object_placement_dialog.cpp' in cmake


def test_object_placement_dialog_has_table_and_actions():
    txt = Path('workcell_builder/workcell_builder/gui/object_placement_dialog.cpp').read_text(encoding='utf-8')
    for marker in [
        'QTableWidget', 'Name", "Source", "Mesh path',
        'Add Asset Object', 'Import STL to Asset Library', 'Add Generated Primitive',
        'Edit Pose', 'Duplicate Object', 'Remove Object', 'Refresh Preview', 'Close | QDialogButtonBox::Apply'
    ]:
        assert marker in txt


def test_scene_generation_and_preview_have_placed_objects_markers():
    txt = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for marker in ['placed_objects:', 'source: asset_stl', 'generated_primitive', 'external_stl_warning', 'workcell_studio_summary.json', 'workcell_preview.svg']:
        assert marker in txt
