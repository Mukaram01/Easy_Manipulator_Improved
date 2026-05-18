from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/validate_scene_builder_mainline_flow.py"


def test_mainline_validation_script_exists_and_has_expected_contract_tokens():
    assert SCRIPT.exists(), "missing scripts/validate_scene_builder_mainline_flow.py"
    text = SCRIPT.read_text(encoding="utf-8")
    for token in [
        "asset_catalog_malformed_yaml_guard",
        "id_allocation_with_existing_layout",
        "layout_creation_and_non_destructive_save_tokens",
        "task_intent_update_and_backup_tokens",
        "cell_definition_v1_draft_generate_repair_validate",
        "readiness_and_preflight_warnings",
        "action_wiring_tokens",
        "workflow_semantic_contract",
        "recommended_action_handler_wiring_contract",
        "cmake_includes_helper_sources",
    ]:
        assert token in text


def test_mainline_validation_script_tracks_current_scene_builder_wording_contract():
    text = SCRIPT.read_text(encoding="utf-8")
    for token in [
        "Scene Manifest",
        "ROS Package File",
        "Demo Launch",
        "Task Recipe (fallback)",
        "Generate Scene Package",
        "Generate/Update Task Intent",
        "use_fake_hardware:=true",
    ]:
        assert token in text
