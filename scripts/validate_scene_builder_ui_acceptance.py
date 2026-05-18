#!/usr/bin/env python3
from __future__ import annotations
import argparse,re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parents[1]

@dataclass
class CheckResult:
    name:str
    ok:bool
    details:list[str]

def _load_text(paths:Iterable[Path])->str:
    return "\n".join(p.read_text(encoding='utf-8') for p in paths if p.exists())

def _token_check(name,haystack,required):
    missing=[t for t in required if t not in haystack]
    return CheckResult(name,not missing,[f"missing token: {t}" for t in missing])

def _regex_absent_check(name,haystack,forbidden):
    hits=[f"forbidden pattern matched ({label}): {pat}" for label,pat in forbidden.items() if re.search(pat,haystack,re.MULTILINE)]
    return CheckResult(name,not hits,hits)

def run_checks(file_text_map:dict[str,str]|None=None)->list[CheckResult]:
    if file_text_map is None:
        file_text_map={
            'mainwindow_cpp':_load_text([REPO_ROOT/'workcell_builder/workcell_builder/gui/mainwindow.cpp']),
            'preview_cpp':_load_text([REPO_ROOT/'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp']),
            'layout_editor_cpp':_load_text([REPO_ROOT/'workcell_builder/workcell_builder/gui/environment_layout_editor.cpp']),
            'object_placement_cpp':_load_text([REPO_ROOT/'workcell_builder/workcell_builder/gui/object_placement_dialog.cpp']),
            'viewport_h':_load_text([REPO_ROOT/'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.h']),
            'viewport_cpp':_load_text([REPO_ROOT/'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp']),
        }
    main=file_text_map.get('mainwindow_cpp','')
    all_text="\n".join(file_text_map.values())
    checks=[]
    xyz_rpy_ok=bool(re.search(r"XYZ\s*/\s*RPY|Proposed XYZ/RPY|pose XYZ/RPY",all_text) or ("XYZ" in all_text and "RPY" in all_text))
    checks.append(CheckResult("manual XYZ/RPY field identifiers",xyz_rpy_ok,[] if xyz_rpy_ok else ["missing XYZ/RPY field identifier token"]))
    checks.append(_token_check("manual axis/unit labels",all_text,["X position in metres","Yaw in radians"]))
    checks.append(_token_check("unit strings (metres/radians)",all_text,["metres","radians"]))
    checks.append(_token_check("hide/show panel controls",all_text,["Hide","Show"]))
    focus_ok=("Focus Canvas" in all_text) or ("Focus Selected" in all_text)
    checks.append(CheckResult("Focus Canvas action wording",focus_ok,[] if focus_ok else ["missing token: Focus Canvas (or Focus Selected)"]))
    checks.append(_token_check("splitter/resizable layout usage",all_text,["QSplitter","setStretchFactor"]))
    checks.append(_token_check("More Actions grouping",all_text,["More Actions","QToolButton","QMenu"]))
    checks.append(_token_check("3D/2D layout wording",all_text,["3D Layout Preview","2D Layout"]))
    checks.append(_token_check("fake hardware launch token",all_text,["use_fake_hardware:=true"]))
    checks.append(_token_check("scene3d gizmo tokens",all_text,["Gizmo:","Scene3D Gizmo Transform","Snap:","Move","Rotate"]))
    checks.append(_token_check("scene3d pick handle API markers",all_text,["pick_gizmo_axis_at_screen","pick_gizmo_rotation_ring_at_screen","active_gizmo_handle"]))
    checks.append(_token_check("scene3d snap value markers",all_text,["snap_translation_value","snap_rotation_value"]))
    checks.append(_token_check("scene3d asset drag/drop markers",all_text,["application/x-workcell-asset-catalog-item","dragEnterEvent","dropEvent","Drop to place","asset_drop_cb"]))
    checks.append(_token_check("escape cancel restore path markers",all_text,["Qt::Key_Escape","restore"]))
    checks.append(_token_check("mouse release single-commit path markers",all_text,["mouseReleaseEvent","commit","single-commit"]))
    checks.append(_token_check("locked item gating/status markers",all_text,["Locked:","locked"]))
    checks.append(_token_check("inspector sync/layout dirty markers",all_text,["inspector","sync","layout dirty"]))
    checks.append(_regex_absent_check("no hidden QPushButton indirection anti-pattern",main,{"menu_action_new_cell":r'addAction\("Create New Cell"\s*,',"menu_action_plan_simulate":r'addAction\("Open Plan & Simulate"\s*,'}))
    checks.append(_regex_absent_check("fixed-width button anti-pattern checks",main,{"qpushbutton_fixed_width_literal":r"\b[a-zA-Z_][a-zA-Z0-9_]*button[a-zA-Z0-9_]*\s*->\s*setFixedWidth\s*\(\s*\d+\s*\)"}))
    return checks

def main()->int:
    argparse.ArgumentParser(description='Static Scene Builder UI acceptance checks').parse_args()
    checks=run_checks()
    for c in checks:
        print(f"{'PASS' if c.ok else 'FAIL'}: {c.name}")
        for d in c.details: print(f"  - {d}")
    return 0 if all(c.ok for c in checks) else 1

if __name__=='__main__':
    raise SystemExit(main())
