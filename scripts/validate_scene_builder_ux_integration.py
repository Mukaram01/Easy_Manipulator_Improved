#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def check(name: str, ok: bool, detail: str = "") -> bool:
    print(("PASS" if ok else "FAIL") + f": {name}")
    if detail and not ok:
        print(f"  - {detail}")
    return ok


def main() -> int:
    mainwindow_cpp = read("workcell_builder/workcell_builder/gui/mainwindow.cpp")
    mainwindow_h = read("workcell_builder/workcell_builder/gui/mainwindow.h")
    viewport_h = read("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.h")
    viewport_cpp = read("workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp")
    model_h = read("workcell_builder/workcell_builder/include/workcell_studio_canvas_model.hpp")
    model_cpp = read("workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp")

    ok = []
    ok.append(check("primary actions present", all(t in mainwindow_cpp for t in ["Select", "Place Asset", "Move", "Inspect", "Save Layout"])))
    ok.append(check("secondary menus present", all(t in mainwindow_cpp for t in ["Overlays", "Canvas More", "Visual Modes", "Label mode", "Mesh preview mode", "Snap mode"])))
    ok.append(check("toolbar overflow wiring", all(t in mainwindow_cpp + mainwindow_h for t in ["scene_builder_secondary_overflow_button_", "scene_builder_secondary_overflow_menu_", "update_scene_builder_top_controls_overflow"])) )
    ok.append(check("no fixed-width toolbar anti-pattern", "setFixedWidth" not in mainwindow_cpp or "button" not in mainwindow_cpp))
    ok.append(check("scene/item state split", all(t in mainwindow_h + mainwindow_cpp for t in ["SelectedSceneState", "SelectedSceneItemState", "selected_scene_state_", "selected_item_state_", "refresh_scene_builder_selected_scene_ui"])))
    ok.append(check("provenance summary helper", all(t in model_h + model_cpp for t in ["WorkcellStudioProvenanceStatus", "provenance_summary_text", "Editable layout:", "Preview fallback:"])))
    ok.append(check("create editable layout action wiring", all(t in mainwindow_cpp + mainwindow_h for t in ["create_starter_layout_from_preview", "refresh_create_starter_layout_action", "Create editable layout from preview"])))
    ok.append(check("workflow rail editable layout stage", "Editable layout" in mainwindow_cpp))
    ok.append(check("viewport helper passes", all(t in viewport_h + viewport_cpp for t in ["draw_ground_grid_pass", "draw_world_axes_pass", "scene_bounds_from_visible_items", "View: 3D"])) )
    ok.append(check("fake-hardware safety token retained", "use_fake_hardware:=true" in mainwindow_cpp))
    ok.append(check("allowed visible top-level set", all(t in mainwindow_cpp for t in ["Studio Home", "New Cell", "Scenes/Open", "Run Next", "More"])))
    forbidden_scene_action_buttons = re.findall(
        r'new\s+QPushButton\s*\(\s*"(Open in Scene Builder|Validate|Plan / Simulate|Export|Delete Scene)"\s*,',
        mainwindow_cpp,
    )
    ok.append(check("forbidden always-visible scene action QPushButtons absent", len(forbidden_scene_action_buttons) == 0, detail=str(forbidden_scene_action_buttons)))
    ok.append(check("canvas mode and view dropdown controls present", all(t in mainwindow_cpp for t in ["Mode", "View"])))
    ok.append(check("actions side-tab grouped sections present", all(t in mainwindow_cpp for t in ["Actions", "Layout", "Generate", "Validate", "Simulate", "Export", "Diagnostics"])))
    ok.append(check("shared action registry exists", "scene_builder_action_registry_" in mainwindow_h and "register_scene_builder_action" in mainwindow_cpp))
    ok.append(check("top header uses shared actions", "scenes_open_menu->addAction(action_generate_yaml_)" in mainwindow_cpp and "more_menu->addAction(action_diagnostics_run_self_test_)" in mainwindow_cpp))
    ok.append(check("Run Next uses recommendation actions", "refresh_run_next_menu" in mainwindow_cpp and "trigger_recommended_workflow_action" in mainwindow_cpp))
    ok.append(check("no duplicate layout label variants", "Create Starter Layout from Preview" not in mainwindow_cpp and "Create editable layout from preview" in mainwindow_cpp))
    ok.append(check("canonical duplicate visible labels present", all(t in mainwindow_cpp for t in ["Create editable layout from preview", "Canvas/Generated Parity"])))
    # Ensure scene set before item state update in refresh flow.
    flow = re.search(r"selected_scene_state_\s*=\s*\{\};.*selected_scene_state_\.valid\s*=\s*true;.*selected_item_state_\s*=\s*current_selected_scene_item\(\);", mainwindow_cpp, flags=re.S)
    ok.append(check("scene state updated before item state", flow is not None))

    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
