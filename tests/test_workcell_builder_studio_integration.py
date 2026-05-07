from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_SELECT_CPP = REPO_ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp"


def test_readme_builder_instructions_present() -> None:
    text = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "./generated/run_builder_validation.sh" in text
    assert "./generated/export_workcell_studio_sources.sh" in text
    assert "./generated/generate_readiness_pack.sh" in text
    assert "python3 -m http.server 8767" in text
    assert "offline/fake-hardware only" in text


def test_generate_readiness_pack_helper_content_is_offline_safe() -> None:
    text = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "WORKCELL_STUDIO_REPO_ROOT" in text
    assert "find_tool_root()" in text
    assert "$TOOL_ROOT/scripts/workcell_studio.py" in text
    assert "$TOOL_ROOT/scripts/validate_builder_generated_scene.py" in text
    assert "$TOOL_ROOT/scripts/export_builder_scene_to_cell_definition.py" in text
    assert "--validate" in text
    assert "--prepare-rviz-preview" in text
    assert "--smoke-dry-run" in text
    assert "--scene-package \\\"$SCENE_DIR\\\"" in text
    assert "--force" in text
    assert "--real-hardware" not in text
    assert "move_group" not in text


def test_generated_shell_scripts_are_marked_executable() -> None:
    text = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "set_script_permissions" in text
    assert "fs::owner_exe" in text
    assert "fs::group_exe" in text
    assert "fs::others_exe" in text


def test_preservation_flow_hints_present() -> None:
    text = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "workcell_builder_task_intent.yaml" in text
    assert "Author task intent via scripts/create_or_update_builder_task_intent.py" in text
    assert "export_builder_scene_to_cell_definition.py" in text


def test_metadata_system_return_code_is_checked() -> None:
    text = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "const int metadata_rc = std::system(cmd.c_str());" in text
    assert "if (metadata_rc != 0)" in text
    assert "append_warning(" in text


def test_readme_fallback_env_hint_present() -> None:
    text = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "generate_readiness_pack.sh /tmp/workcell_readiness_pack test_scene" in text
    assert "If helpers cannot locate tooling, set:" in text
    assert "export WORKCELL_STUDIO_REPO_ROOT=~/workcell_ws/src/easy_manipulation_deployment" in text


def test_scene_select_uses_absolute_metadata_renderer_path_resolution() -> None:
    text = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "python3 scripts/render_workcell_builder_metadata.py" not in text
    assert "WORKCELL_STUDIO_REPO_ROOT" in text
    assert "render_workcell_builder_metadata.py" in text
    assert "resolve_tool_root(" in text


def test_scene_select_missing_metadata_script_warning_mentions_env_var() -> None:
    text = SCENE_SELECT_CPP.read_text(encoding="utf-8")
    assert "Could not locate render_workcell_builder_metadata.py" in text
    assert "Set WORKCELL_STUDIO_REPO_ROOT=/path/to/easy_manipulation_deployment" in text


def test_validate_scene_metadata_optional_is_warn_when_missing_and_pass_when_present() -> None:
    text = (REPO_ROOT / "scripts/validate_builder_generated_scene.py").read_text(encoding="utf-8")
    assert "workcell_builder_metadata.yaml missing (optional)" in text
    assert "workcell_builder_metadata.yaml present" in text
    assert "present (optional)" not in text
