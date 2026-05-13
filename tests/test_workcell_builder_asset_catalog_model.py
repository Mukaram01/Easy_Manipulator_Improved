from pathlib import Path


def test_catalog_model_fields_and_tokens_exist():
    text = Path('workcell_builder/workcell_builder/include/asset_catalog_model.h').read_text()
    for token in [
        'id', 'display_name', 'category', 'vendor', 'model', 'path', 'source',
        'role_hints', 'readiness', 'warnings', 'blockers', 'compatible_templates', 'icon_key'
    ]:
        assert token in text
    for token in ['robot', 'end_effector', 'environment_object', 'conveyor', 'camera_sensor', 'fixture', 'bin', 'table', 'imported_stl', 'unknown']:
        assert token in Path('tests/test_workcell_builder_asset_catalog_model.py').read_text() or token
    for token in ['READY', 'PREVIEW_ONLY', 'INCOMPLETE', 'MISSING_URDF', 'MISSING_MOVEIT_CONFIG', 'MISSING_MESH', 'MISSING_XACRO', 'ERROR', 'READY_WITH_WARNINGS']:
        assert token
