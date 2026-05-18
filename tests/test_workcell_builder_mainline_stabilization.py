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


def test_mainwindow_parity_mode_and_generation_ordering_contract():
    text = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    assert "CanvasGeneratedParityMode::PreGeneration" in text
    assert "CanvasGeneratedParityMode::PostGeneration" in text
    assert "--mode %4" in text
    assert "Generated package created but Canvas/Generated parity has blockers." in text
    assert "Generated artifacts are not present yet. Run Generate Scene Package for strict parity." in text
    assert text.index("run_canvas_generated_parity_check(CanvasGeneratedParityMode::PreGeneration") < text.index("generate_workcell_from_cell_definition.py")
    assert text.index("generate_workcell_from_cell_definition.py") < text.index("run_canvas_generated_parity_check(CanvasGeneratedParityMode::PostGeneration")
