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
    top_header_literals_ok = all(t in mainwindow_cpp for t in ["Studio Home", "New Cell"])
    visible_assignments = {
        "scenes_open_button": "Scenes",
        "run_next_button": "Run Next",
        "more_button": "More",
    }
    visible_details = []
    for button, expected in visible_assignments.items():
        m = re.search(rf'{button}\s*->\s*setText\("([^"]+)"\);', mainwindow_cpp)
        if not m:
            visible_details.append(f"missing {button}")
            continue
        if m.group(1) != expected:
            visible_details.append(f"{button}={m.group(1)!r}")
    ok.append(check("allowed visible top-level set", top_header_literals_ok and len(visible_details) == 0, detail=str(visible_details)))
    top_header_exact = all(t in mainwindow_cpp for t in [
        'scenes_open_button->setText("Scenes");',
        'run_next_button->setText("Run Next");',
        'more_button->setText("More");',
    ])
    ok.append(check("top header exact labels", top_header_exact))
    top_labels = {}
    for button in ["scenes_open_button", "run_next_button", "more_button"]:
        m = re.search(rf'{button}\s*->\s*setText\("([^"]+)"\);', mainwindow_cpp)
        top_labels[button] = m.group(1) if m else ""
    no_label_hacks = all("/" not in text and not text.endswith("_") for text in top_labels.values())
    ok.append(check("top header labels avoid slash/underscore hacks", no_label_hacks))
    trailing_underscore_hacks = [label for label in ["Run Next_", "More_", "Scenes/Open_"] if label in mainwindow_cpp]
    ok.append(check("top header trailing underscore nav-label hacks absent", len(trailing_underscore_hacks) == 0, detail=str(trailing_underscore_hacks)))
    forbidden_topbar = re.findall(r"addWidget\s*\(\s*new\s+(?:QPushButton|QToolButton)\s*\(\s*\"(Validate|Generate|Plan|Simulate|Export|Readiness|Delete Scene|Select|Place Asset|Move|Inspect|Save Layout|Undo|Redo)\"", mainwindow_cpp)
    ok.append(check("forbidden direct top-bar primary actions absent", len(forbidden_topbar) == 0, detail=str(forbidden_topbar)))
    ok.append(check("canvas mode and view dropdown controls present", all(t in mainwindow_cpp for t in ["Mode", "View"])))
    ok.append(check("actions side-tab grouped sections present", all(t in mainwindow_cpp for t in ["Actions", "Layout", "Generate", "Validate", "Simulate", "Export", "Diagnostics"])))
    ok.append(check("shared action registry exists", "scene_builder_action_registry_" in mainwindow_h and "register_scene_builder_action" in mainwindow_cpp))
    ok.append(check("top header uses shared actions", "scenes_open_menu->addAction(action_generate_yaml_)" in mainwindow_cpp and "more_menu->addAction(action_diagnostics_run_self_test_)" in mainwindow_cpp))
    ok.append(check("Run Next uses recommendation actions", "refresh_run_next_menu" in mainwindow_cpp and "trigger_recommended_workflow_action" in mainwindow_cpp))
    ok.append(check("no duplicate layout label variants", "Create Starter Layout from Preview" not in mainwindow_cpp and "Create editable layout from preview" in mainwindow_cpp))
    ok.append(check("canonical duplicate visible labels present", all(t in mainwindow_cpp for t in ["Create editable layout from preview", "Canvas/Generated Parity"])))
    ok.append(check("selected-scene actions moved to Scene Actions menu", all(t in mainwindow_cpp + mainwindow_h for t in ["Scene Actions", "dashboard_scene_actions_button_", "dashboard_scene_actions_menu_", "dashboard_open_scene_action_", "dashboard_delete_action_"])))
    forbidden_selected_scene_buttons = re.findall(r'dashboard_(?:open_scene|validate|plan|export|delete)_button_\s*=\s*new\s+QPushButton\s*\(\s*"(?:Open in Scene Builder|Validate|Plan / Simulate|Export|Delete Scene)"', mainwindow_cpp)
    ok.append(check("no always-visible selected-scene action QPushButtons", len(forbidden_selected_scene_buttons) == 0, detail=str(forbidden_selected_scene_buttons)))
    # Ensure scene set before item state update in refresh flow.
    flow = re.search(r"selected_scene_state_\s*=\s*\{\};.*selected_scene_state_\.valid\s*=\s*true;.*selected_item_state_\s*=\s*current_selected_scene_item\(\);", mainwindow_cpp, flags=re.S)
    ok.append(check("scene state updated before item state", flow is not None))

    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
