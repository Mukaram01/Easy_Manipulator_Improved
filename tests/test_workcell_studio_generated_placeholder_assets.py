from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_generated_placeholder_metadata_persisted():
    for token in ['RoleGeneratedPlaceholder', 'generated_placeholder: %18', 'category.contains("placeholder"']:
        assert token in CPP
