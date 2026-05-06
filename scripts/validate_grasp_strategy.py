#!/usr/bin/env python3
"""Validate offline grasp_strategy/v1 metadata files."""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from capability_registry import DEFAULT_CAPABILITIES_DIR, load_capability_registry, load_structured_data

ALLOWED_STRATEGIES = {"suction_top","suction_side","vacuum_array_top","pinch","enveloping","side_grip","magnetic_pick","custom"}
ALLOWED_APPROACH_AXIS = {"z_down","z_up","x_plus","x_minus","y_plus","y_minus","tool_z","custom"}
ALLOWED_ORIENTATION_MODE = {"vertical","horizontal","tool_aligned","object_aligned","free","custom"}
ALLOWED_TOOL_FAMILIES = {"finger_gripper","three_finger_gripper","suction","vacuum_array","magnetic","tool_changer","custom"}

@dataclass
class Result:
    path: Path
    parser: str
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def status(self) -> str:
        return "FAIL" if self.errors else ("WARN" if self.warnings else "PASS")

def _num_list(v: Any, n: int|None=None) -> bool:
    return isinstance(v,list) and (n is None or len(v)==n) and all(isinstance(i,(int,float)) for i in v)

def validate_doc(doc: dict[str, Any], path: Path, parser: str, strict: bool=False, capabilities_dir: Path|None=None) -> Result:
    r = Result(path=path, parser=parser)
    gs = doc.get("grasp_strategy")
    if doc.get("schema_version") != "grasp_strategy/v1":
        r.errors.append("schema_version must be exactly 'grasp_strategy/v1'.")
    if not isinstance(gs, dict):
        r.errors.append("grasp_strategy mapping is required.")
        return r
    if not isinstance(gs.get("id"), str) or not gs["id"].strip(): r.errors.append("grasp_strategy.id must be a non-empty string.")
    if not isinstance(gs.get("label"), str) or not gs["label"].strip(): r.errors.append("grasp_strategy.label must be a non-empty string.")
    if not isinstance(gs.get("strategy"), str) or gs["strategy"] not in ALLOWED_STRATEGIES: r.errors.append("grasp_strategy.strategy must be a supported strategy enum.")
    fams = gs.get("compatible_tool_families")
    if fams is not None:
        if not isinstance(fams, list) or not all(isinstance(f, str) for f in fams): r.errors.append("compatible_tool_families must be a list of strings when provided.")
        else:
            bad = [f for f in fams if f not in ALLOWED_TOOL_FAMILIES]
            if bad: r.errors.append(f"Unsupported tool families: {bad}")
    if gs.get("approach_axis") not in ALLOWED_APPROACH_AXIS: r.errors.append("approach_axis must be a supported enum.")
    if gs.get("orientation_mode") not in ALLOWED_ORIENTATION_MODE: r.errors.append("orientation_mode must be a supported enum.")
    if not isinstance(gs.get("approach_distance_m"),(int,float)) or gs["approach_distance_m"] <= 0: r.errors.append("approach_distance_m must be a positive number.")
    if not isinstance(gs.get("retreat_distance_m"),(int,float)) or gs["retreat_distance_m"] <= 0: r.errors.append("retreat_distance_m must be a positive number.")
    for key in ("tool_frame_offset_xyz","tool_frame_offset_rpy"):
        if key in gs and not _num_list(gs.get(key),3): r.errors.append(f"{key} must be a numeric list length 3 when provided.")
    for key in ("allowed_roll_angles_deg","allowed_yaw_angles_deg"):
        if key in gs and not _num_list(gs.get(key)): r.errors.append(f"{key} must be a numeric list when provided.")
    if not isinstance(gs.get("contact"), dict): r.errors.append("contact mapping is required.")
    if not isinstance(gs.get("release"), dict): r.errors.append("release mapping is required.")

    registry = load_capability_registry(capabilities_dir)
    r.notes.extend(registry.parser_notes)
    comp_ids = gs.get("compatible_end_effector_capabilities")
    if isinstance(comp_ids, list):
        for cap_id in comp_ids:
            if not isinstance(cap_id, str):
                r.errors.append("compatible_end_effector_capabilities must contain strings only.")
                continue
            rec = registry.get(cap_id)
            if rec is None:
                (r.errors if strict else r.warnings).append(f"Unknown compatible end-effector capability id '{cap_id}'.")
                continue
            if isinstance(fams, list) and rec.family and rec.family not in fams:
                r.errors.append(f"Capability '{cap_id}' family '{rec.family}' is not in compatible_tool_families.")
    return r

def _iter_files(target: Path) -> list[Path]:
    if target.is_file(): return [target]
    return sorted(p for p in target.rglob("*") if p.is_file() and p.suffix.lower() in {".yaml",".yml",".json"})

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", type=Path)
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--capabilities-dir", type=Path, default=DEFAULT_CAPABILITIES_DIR)
    args = ap.parse_args()
    results=[]
    for p in _iter_files(args.path):
        try:
            doc, parser = load_structured_data(p)
            res = validate_doc(doc, p, parser, strict=args.strict, capabilities_dir=args.capabilities_dir)
        except Exception as exc:
            res = Result(path=p, parser="unknown", errors=[str(exc)])
        results.append(res)
    summary={"pass":sum(1 for r in results if r.status=="PASS"),"warn":sum(1 for r in results if r.status=="WARN"),"fail":sum(1 for r in results if r.status=="FAIL"),"total":len(results)}
    if args.json:
        print(json.dumps({"summary":summary,"results":[{"path":str(r.path),"parser":r.parser,"result":r.status,"errors":r.errors,"warnings":r.warnings,"notes":r.notes} for r in results]}, indent=2))
    else:
        for r in results:
            print(f"{r.status}: {r.path}")
            for n in r.notes: print(f"  NOTE: {n}")
            for w in r.warnings: print(f"  WARN: {w}")
            for e in r.errors: print(f"  FAIL: {e}")
        print(f"SUMMARY: PASS={summary['pass']} WARN={summary['warn']} FAIL={summary['fail']} TOTAL={summary['total']}")
    return 1 if summary["fail"] else 0

if __name__ == "__main__":
    raise SystemExit(main())
