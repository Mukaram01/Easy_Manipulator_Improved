#!/usr/bin/env python3
"""Generate dependency-light offline task execution plans from dry-run results."""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = REPO_ROOT / "docs" / "manuals" / "generated_execution_plans"
DRY_RUN_PATH = REPO_ROOT / "scripts" / "dry_run_task_recipe.py"
VALIDATOR_PATH = REPO_ROOT / "scripts" / "validate_scene_contract.py"



def _load_module(module_name: str, module_path: Path):
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if not spec or not spec.loader:
        raise RuntimeError(f"Unable to load module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


dry_run = _load_module("dry_run_task_recipe", DRY_RUN_PATH)
validator = _load_module("validate_scene_contract", VALIDATOR_PATH)


@dataclass
class PlanResult:
    scene: str
    status: str
    task_recipe_id: str
    task_type: str
    matched_rule_id: str
    destination_id: str
    markdown_path: Path | None
    json_path: Path | None
    steps_count: int
    notes: list[str]


def _select_end_effector_actions(end_effector: dict[str, Any]) -> tuple[str, str]:
    ee_type = str(end_effector.get("type", "")).strip().lower()
    ee_brand = str(end_effector.get("brand", "")).strip().lower()
    token = f"{ee_type} {ee_brand}"

    if "suction" in token or "airpick" in token:
        return "activate_suction", "deactivate_suction"
    if "finger" in token or "robotiq" in token or "2f" in token or "3f" in token:
        return "close_gripper", "open_gripper"
    return "engage_end_effector", "release_end_effector"


def _resolve_destination(recipe: dict[str, Any], destination_id: str) -> dict[str, Any] | None:
    destinations = recipe.get("destinations")
    if not isinstance(destinations, list):
        return None
    for destination in destinations:
        if isinstance(destination, dict) and str(destination.get("id")) == destination_id:
            return destination
    return None


def _build_steps(task_type: str, close_action: str, open_action: str) -> tuple[list[dict[str, str]], list[str]]:
    task_type_lower = task_type.strip().lower()
    notes: list[str] = []

    if task_type_lower in {"palletising", "palletizing"}:
        notes.append("Pallet pattern execution is not implemented yet; generated generic place-to-destination sequence.")
    elif task_type_lower in {"sort", "sorting", "binning", "inspection", "inspection_routing", "pick_place", "reject"}:
        pass
    else:
        notes.append(
            f"Unknown task type '{task_type}'; generated a conservative generic pick-process-place sequence."
        )

    return (
        [
            {
                "id": "acquire_object",
                "type": "perception_or_self_test",
                "action": "resolve_object",
                "description": "Use self_test object for offline commissioning.",
                "offline_runtime_status": "offline-only",
            },
            {
                "id": "select_grasp_strategy",
                "type": "grasp_selection",
                "action": "select_strategy",
                "description": "Select grasp strategy for end effector.",
                "offline_runtime_status": "offline-only",
            },
            {
                "id": "move_to_pre_grasp",
                "type": "motion_plan",
                "action": "plan_pre_grasp",
                "description": "Plan safe approach to pre-grasp pose.",
                "offline_runtime_status": "runtime-target",
            },
            {
                "id": "move_to_grasp",
                "type": "motion_plan",
                "action": "plan_grasp_approach",
                "description": "Plan final approach to grasp pose.",
                "offline_runtime_status": "runtime-target",
            },
            {
                "id": "close_end_effector",
                "type": "end_effector_action",
                "action": close_action,
                "description": "Engage the selected end effector.",
                "offline_runtime_status": "runtime-target",
            },
            {
                "id": "attach_object",
                "type": "planning_scene_update",
                "action": "attach_collision_object",
                "description": "Attach object to end-effector link.",
                "offline_runtime_status": "runtime-target",
            },
            {
                "id": "move_to_pre_place",
                "type": "motion_plan",
                "action": "plan_pre_place",
                "description": "Move toward destination approach pose.",
                "offline_runtime_status": "runtime-target",
            },
            {
                "id": "move_to_place",
                "type": "motion_plan",
                "action": "plan_place",
                "description": "Move to destination pose.",
                "offline_runtime_status": "runtime-target",
            },
            {
                "id": "release_object",
                "type": "end_effector_action",
                "action": open_action,
                "description": "Release object at destination.",
                "offline_runtime_status": "runtime-target",
            },
            {
                "id": "detach_object",
                "type": "planning_scene_update",
                "action": "detach_collision_object",
                "description": "Detach object from end-effector link.",
                "offline_runtime_status": "runtime-target",
            },
            {
                "id": "retreat",
                "type": "motion_plan",
                "action": "plan_retreat",
                "description": "Retreat from placement pose.",
                "offline_runtime_status": "runtime-target",
            },
            {
                "id": "return_home",
                "type": "motion_plan",
                "action": "plan_return_home",
                "description": "Return to configured safe home state.",
                "offline_runtime_status": "runtime-target",
            },
        ],
        notes,
    )


def build_execution_plan(scene_name: str, manifest: dict[str, Any], dry_run_row: Any) -> dict[str, Any]:
    recipe = manifest.get("task_recipe") if isinstance(manifest.get("task_recipe"), dict) else {}
    robot = manifest.get("robot") if isinstance(manifest.get("robot"), dict) else {}
    end_effector = manifest.get("end_effector") if isinstance(manifest.get("end_effector"), dict) else {}

    destination = _resolve_destination(recipe, dry_run_row.selected_destination_id) or {}
    close_action, open_action = _select_end_effector_actions(end_effector)
    steps, plan_notes = _build_steps(str(dry_run_row.task_type), close_action, open_action)

    return {
        "scene": scene_name,
        "task_recipe_id": str(recipe.get("id", dry_run_row.recipe_id)),
        "task_type": str(recipe.get("type", dry_run_row.task_type)),
        "object_id": str(dry_run_row.object_id),
        "object_attributes": dict(dry_run_row.object_attributes),
        "matched_rule_id": str(dry_run_row.matched_rule_id),
        "destination_id": str(dry_run_row.selected_destination_id),
        "destination_action": str(dry_run_row.selected_action),
        "destination_pose": {
            "frame_id": str(destination.get("frame_id", "(n/a)")),
            "pose_xyz": destination.get("pose_xyz", []),
            "pose_rpy": destination.get("pose_rpy", []),
        },
        "end_effector_brand": str(end_effector.get("brand", "unknown")),
        "end_effector_type": str(end_effector.get("type", "unknown")),
        "moveit_link": str(robot.get("ee_link", "unknown")),
        "grasp_frame": str(end_effector.get("grasp_frame", "unknown")),
        "planning_group": str(robot.get("planning_group", "unknown")),
        "base_frame": str(robot.get("base_frame", "unknown")),
        "steps": steps,
        "notes": list(dry_run_row.notes) + plan_notes,
    }


def _attrs_text(attrs: dict[str, Any]) -> str:
    if not attrs:
        return "(n/a)"
    return ", ".join(f"{key}={attrs[key]}" for key in sorted(attrs.keys()))


def build_plan_markdown(plan: dict[str, Any]) -> str:
    lines = [
        f"# Offline Task Execution Plan: {plan['scene']}",
        "",
        f"- Scene: `{plan['scene']}`",
        f"- Task recipe id: `{plan['task_recipe_id']}`",
        f"- Task type: `{plan['task_type']}`",
        f"- Object id: `{plan['object_id']}`",
        f"- Object attributes: `{_attrs_text(plan['object_attributes'])}`",
        f"- Matched rule id: `{plan['matched_rule_id']}`",
        f"- Destination id: `{plan['destination_id']}`",
        f"- Destination pose: `frame={plan['destination_pose']['frame_id']}, xyz={plan['destination_pose']['pose_xyz']}, rpy={plan['destination_pose']['pose_rpy']}`",
        f"- Destination action: `{plan['destination_action']}`",
        f"- End effector: `type={plan['end_effector_type']}, brand={plan['end_effector_brand']}`",
        f"- Motion context: `planning_group={plan['planning_group']}, base_frame={plan['base_frame']}, moveit_link={plan['moveit_link']}, grasp_frame={plan['grasp_frame']}`",
        "",
        "## Ordered execution steps",
        "",
        "| # | step id | step type | action | description | offline/runtime status |",
        "|---|---|---|---|---|---|",
    ]

    for idx, step in enumerate(plan["steps"], start=1):
        lines.append(
            f"| {idx} | `{step['id']}` | `{step['type']}` | `{step['action']}` | {step['description']} | `{step['offline_runtime_status']}` |"
        )

    lines.extend(
        [
            "",
            "## Notes",
            "",
        ]
    )

    for note in plan["notes"]:
        lines.append(f"- {note}")

    return "\n".join(lines) + "\n"


def evaluate_scene(scene_name: str, manifest_path: Path) -> PlanResult:
    dry_row = dry_run.evaluate_scene(scene_name, manifest_path)

    if dry_row.status != "PASS":
        no_plan_note = f"No execution plan generated because dry-run status is {dry_row.status}."
        notes = list(dry_row.notes) + [no_plan_note]
        return PlanResult(
            scene=scene_name,
            status=dry_row.status,
            task_recipe_id=str(dry_row.recipe_id),
            task_type=str(dry_row.task_type),
            matched_rule_id=str(dry_row.matched_rule_id),
            destination_id=str(dry_row.selected_destination_id),
            markdown_path=None,
            json_path=None,
            steps_count=0,
            notes=notes,
        )

    manifest, _, parser_notes = validator._read_manifest(str(manifest_path))
    plan = build_execution_plan(scene_name, manifest, dry_row)
    plan["notes"] = list(parser_notes) + plan["notes"]

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    json_path = OUTPUT_DIR / f"{scene_name}_execution_plan.json"
    markdown_path = OUTPUT_DIR / f"{scene_name}_execution_plan.md"

    json_path.write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown_path.write_text(build_plan_markdown(plan), encoding="utf-8")

    return PlanResult(
        scene=scene_name,
        status="PASS",
        task_recipe_id=str(plan["task_recipe_id"]),
        task_type=str(plan["task_type"]),
        matched_rule_id=str(plan["matched_rule_id"]),
        destination_id=str(plan["destination_id"]),
        markdown_path=markdown_path,
        json_path=json_path,
        steps_count=len(plan["steps"]),
        notes=list(plan["notes"]),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenes", nargs="*", help="Optional scene names to include.")
    parser.add_argument("--check", action="store_true", help="Print concise PASS/WARN/FAIL/SKIP summary.")
    args = parser.parse_args()

    discovered = dry_run.discover_scene_manifests()
    if args.scenes:
        requested = set(args.scenes)
        discovered = [entry for entry in discovered if entry[0] in requested]

    if not discovered:
        print("No scene manifests discovered for task execution plan generation.")
        return 2

    rows = [evaluate_scene(scene, path) for scene, path in discovered]

    for row in rows:
        if row.status == "PASS":
            print(f"PASS {row.scene:24} steps={row.steps_count} md={row.markdown_path} json={row.json_path}")
        else:
            print(f"{row.status:4} {row.scene:24} no plan generated ({'; '.join(row.notes)})")

    if args.check:
        pass_count = sum(1 for row in rows if row.status == "PASS")
        warn_count = sum(1 for row in rows if row.status == "WARN")
        fail_count = sum(1 for row in rows if row.status == "FAIL")
        skip_count = sum(1 for row in rows if row.status == "SKIP")
        print(f"Summary: PASS={pass_count} WARN={warn_count} FAIL={fail_count} SKIP={skip_count}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
