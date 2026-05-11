from pathlib import Path

def test_schema_docs_and_version_and_sections_exist():
    t = Path('docs/manuals/WORKCELL_SCENE_SCHEMA_V1.md').read_text(encoding='utf-8')
    for m in ['workcell_scene/v1','scene','robot','tool','compatibility','placed_objects','camera','task','safety','metadata']:
        assert m in t

def test_cpp_helper_and_cmake_wiring_exist():
    assert Path('workcell_builder/workcell_builder/include/workcell_scene_schema.hpp').exists()
    assert Path('workcell_builder/workcell_builder/src_workcell_scene_schema.cpp').exists()
    assert 'src_workcell_scene_schema.cpp' in Path('workcell_builder/workcell_builder/CMakeLists.txt').read_text(encoding='utf-8')

def test_validator_stdlib_only_and_markers_and_rules_present():
    t = Path('scripts/validate_workcell_scene.py').read_text(encoding='utf-8').lower()
    assert 'import yaml' not in t and 'pyyaml' not in t
    for m in ['workcell_scene_schema: pass','workcell_scene_schema: warn','workcell_scene_schema: fail','unknown_compatibility','incompatible','fake_hardware_first','runtime_execution_enabled','real_hardware_enabled']:
        assert m in t
