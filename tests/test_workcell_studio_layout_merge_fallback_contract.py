from pathlib import Path


CPP = Path("workcell_builder/workcell_builder/src_workcell_studio_layout_merge.cpp").read_text(encoding="utf-8")


def test_missing_merge_script_uses_fallback_warning_and_report():
    assert "layout_merge_fallback_used" in CPP
    assert 'root.insert("layout_merge_status", blockers.empty() ? "WARN" : "BLOCKED");' in CPP
    assert 'root.insert("layout_merge_warning", "missing script fallback used");' in CPP
    assert "write_layout_merge_fallback_report" in CPP
    assert "out.status = out.blockers.empty();" in CPP


def test_fallback_blocks_only_on_missing_required_authoring_files_with_exact_names():
    for token in [
        'scene_dir / "environment.yaml"',
        'scene_dir / "environment_layout.yaml"',
        'scene_dir / "layout" / "workcell_studio_layout.yaml"',
        'scene_dir / "cell_definition.yaml"',
        "Next action: click Save Layout.",
        "Next action: click Generate YAML.",
    ]:
        assert token in CPP
