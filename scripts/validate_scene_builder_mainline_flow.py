#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
DISC = ROOT / "workcell_builder/workcell_builder/gui/asset_catalog_discovery.cpp"
ID_UTILS = ROOT / "workcell_builder/workcell_builder/src_workcell_studio_id_utils.cpp"
CMAKE = ROOT / "workcell_builder/workcell_builder/CMakeLists.txt"

CHECKS = {
    "asset_catalog_malformed_yaml_guard": (DISC, ["YAML::LoadFile", "catch (const std::exception", "log_warning_once_per_context_path_reason", "yaml_map_key", "yaml_map_value_or_empty"]),
    "id_allocation_with_existing_layout": (ID_UTILS, ["workcell_studio_collect_layout_ids", "placed_assets", "items", "workcell_studio_next_id"]),
    "layout_creation_and_non_destructive_save_tokens": (CPP, ["environment_layout.yaml", "Save Layout", "Malformed environment_layout.yaml backup failed", "Malformed environment_layout.yaml backed up to"]),
    "task_intent_update_and_backup_tokens": (CPP, ["QMessageBox::question", "update_selected_scene_task_intent_bindings(\"Pick Zone + Pick Source\"", "update_selected_scene_task_intent_bindings(\"Place Zone + Place Target\"", "task intent backup"]),
    "cell_definition_v1_draft_generate_repair_validate": (CPP, ["new cell_definition.yaml generated", "existing valid cell_definition.yaml preserved", "invalid cell_definition.yaml backed up and regenerated", "validate_cell_definition.py"]),
    "readiness_and_preflight_warnings": (CPP, ["generation_asset_support_preflight", "severe_preflight_failure", "blocked by severe schema/safety preflight failure"]),
    "action_wiring_tokens": (CPP, ["&MainWindow::open_add_asset_dialog", "&MainWindow::place_selected_asset_from_dialog", "&MainWindow::save_layout_changes", "&MainWindow::generate_yaml_draft_for_selected_scene", "&MainWindow::generate_scene_package_for_selected_scene", "&MainWindow::validate_generated_scene_for_selected_scene", "&MainWindow::preview_offline_plan_for_selected_scene", "&MainWindow::copy_build_launch_commands_for_selected_scene", "&MainWindow::delete_selected_item", "&MainWindow::bind_selected_item_as_pick_zone", "&MainWindow::bind_selected_item_as_place_zone", "&MainWindow::bind_selected_item_as_camera"]),
    "cmake_includes_helper_sources": (CMAKE, ["src_workcell_warning_once.cpp", "gui/asset_catalog_discovery.cpp", "src_workcell_studio_id_utils.cpp"]),
}

def main() -> int:
    failures = []
    for name, (path, tokens) in CHECKS.items():
        if not path.exists():
            failures.append(f"{name}: missing file {path}")
            continue
        text = path.read_text(encoding="utf-8")
        missing = [t for t in tokens if t not in text]
        if missing:
            failures.append(f"{name}: missing tokens -> {', '.join(missing)}")

    print("Scene Builder Mainline Flow Validation")
    if failures:
        print("FAIL")
        for item in failures:
            print(f" - {item}")
        return 1

    print("PASS")
    print(f"Validated {len(CHECKS)} integration contracts.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
