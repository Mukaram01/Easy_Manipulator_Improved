from pathlib import Path


def test_default_scene_root_prefers_workspace_src_for_symlinked_scenes():
    cpp = Path('workcell_builder/workcell_builder/src_scene_select_paths.cpp').read_text(encoding='utf-8')
    assert 'candidates.push_back(fs::path(home) / "workcell_ws" / "src");' in cpp


def test_scene_root_display_supports_tilde_path_label():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'std::string display_path_with_home_tilde' in cpp
    assert 'return "~/" +' in cpp
