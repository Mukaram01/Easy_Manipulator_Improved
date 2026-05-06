#!/usr/bin/env python3
"""Export Workcell Studio catalog metadata for workcell_builder consumption."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from capability_registry import load_structured_data

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAP_DIR = REPO_ROOT / "catalog" / "capabilities"
DEFAULT_GRASP_DIR = REPO_ROOT / "catalog" / "grasp_strategies"
DEFAULT_OUTPUT = REPO_ROOT / "workcell_builder" / "workcell_builder" / "templates" / "workcell_studio_catalog.yaml"


def _load_docs(root: Path) -> list[dict[str, Any]]:
    docs: list[dict[str, Any]] = []
    for p in sorted(root.rglob("*")):
        if not p.is_file() or p.suffix.lower() not in {".yaml", ".yml", ".json"}:
            continue
        doc, _ = load_structured_data(p)
        docs.append(doc)
    return docs


def _cap_entry(doc: dict[str, Any]) -> tuple[str, dict[str, Any]] | None:
    schema = str(doc.get("schema_version", ""))
    if "/" not in schema:
        return None
    key = schema.split("/", 1)[0].replace("_capability", "")
    payload = doc.get(key)
    if not isinstance(payload, dict):
        payload = doc.get("asset") if key == "environment_asset" else None
    if not isinstance(payload, dict):
        return None
    cap_id = payload.get("id")
    if not isinstance(cap_id, str) or not cap_id.strip():
        return None

    family = payload.get("family") if isinstance(payload.get("family"), str) else payload.get("task_family")
    runtime_supported = bool(payload.get("runtime_supported", True))
    preview_only = bool(payload.get("preview_only", not runtime_supported))
    category = key
    if category == "robot":
        category = "robot_capabilities"
    elif category == "end_effector":
        category = "end_effector_capabilities"
    elif category == "sensor":
        category = "sensor_capabilities"
    elif category == "task":
        category = "task_capabilities"

    entry: dict[str, Any] = {
        "id": cap_id,
        "label": payload.get("label", cap_id),
        "family": family,
        "schema": schema,
        "type": key,
        "runtime_supported": runtime_supported,
        "preview_only": preview_only,
    }
    if isinstance(payload.get("compatible_robot_families"), list):
        entry["compatible_robot_families"] = payload["compatible_robot_families"]
    if isinstance(payload.get("compatible_tool_families"), list):
        entry["compatible_tool_families"] = payload["compatible_tool_families"]
    if isinstance(payload.get("default_grasp_strategy"), str):
        entry["default_grasp_strategy"] = payload["default_grasp_strategy"]
    return category, entry


def _grasp_entry(doc: dict[str, Any]) -> dict[str, Any] | None:
    if doc.get("schema_version") != "grasp_strategy/v1":
        return None
    gs = doc.get("grasp_strategy")
    if not isinstance(gs, dict):
        return None
    gid = gs.get("id")
    if not isinstance(gid, str):
        return None
    return {
        "id": gid,
        "label": gs.get("label", gid),
        "schema": "grasp_strategy/v1",
        "strategy": gs.get("strategy"),
        "compatible_tool_families": gs.get("compatible_tool_families", []),
        "compatible_end_effector_capabilities": gs.get("compatible_end_effector_capabilities", []),
        "runtime_supported": bool(gs.get("runtime_supported", False)),
        "preview_only": not bool(gs.get("runtime_supported", False)),
    }


def export_catalog(capabilities_dir: Path, grasp_dir: Path) -> dict[str, Any]:
    catalog: dict[str, Any] = {
        "schema_version": "workcell_builder_catalog/v1",
        "robot_capabilities": [],
        "end_effector_capabilities": [],
        "sensor_capabilities": [],
        "task_capabilities": [],
        "grasp_strategies": [],
        "runtime_status_hints": {
            "runtime_ready": "All selected components declare runtime_supported=true and no blockers.",
            "fake_hardware_ready": "Simulation/fake hardware workflows are supported.",
            "preview_only": "Selection contains preview-only capabilities requiring runtime integration.",
        },
    }
    for doc in _load_docs(capabilities_dir):
        parsed = _cap_entry(doc)
        if not parsed:
            continue
        category, entry = parsed
        if category in catalog:
            catalog[category].append(entry)

    for doc in _load_docs(grasp_dir):
        entry = _grasp_entry(doc)
        if entry:
            catalog["grasp_strategies"].append(entry)

    for key in ("robot_capabilities", "end_effector_capabilities", "sensor_capabilities", "task_capabilities", "grasp_strategies"):
        catalog[key] = sorted(catalog[key], key=lambda i: i["id"])
    return catalog


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--capabilities-dir", type=Path, default=DEFAULT_CAP_DIR)
    ap.add_argument("--grasp-strategies-dir", type=Path, default=DEFAULT_GRASP_DIR)
    ap.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = ap.parse_args()

    catalog = export_catalog(args.capabilities_dir, args.grasp_strategies_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # YAML-compatible JSON formatting keeps parser dependencies optional.
    args.output.write_text(json.dumps(catalog, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote catalog: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
