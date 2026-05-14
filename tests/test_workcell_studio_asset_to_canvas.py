from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/src_workcell_studio_layout_editor.cpp').read_text(encoding='utf-8')
MODEL = Path('workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp').read_text(encoding='utf-8')

def test_layout_contains_asset_fields_and_reload_support():
    for t in ['layout/workcell_studio_layout.yaml','schema_version','workcell_studio_layout/v1','source_path','display_name','category','role','pose','size']:
        assert t in (CPP + MODEL)

def test_duplicate_ids_avoided_warning_token_exists():
    assert 'duplicate id' in CPP
