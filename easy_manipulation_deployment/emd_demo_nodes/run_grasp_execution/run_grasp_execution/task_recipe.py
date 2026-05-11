from __future__ import annotations
import json
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception as exc:  # pragma: no cover
    raise RuntimeError("PyYAML is required for task recipe preview") from exc

SAFETY_MARKERS = ["OFFLINE_ONLY", "NO_MOTION_COMMAND", "NO_MOVEIT_PLAN", "NO_REAL_HARDWARE"]
SUPPORTED_TASK_TYPES = {"pick_place", "sorting", "inspection_only"}


def load_task_recipe(path: str | Path) -> dict[str, Any]:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}


def normalize_task_recipe(recipe: dict[str, Any]) -> dict[str, Any]:
    normalized = dict(recipe)
    normalized.setdefault("task", {})
    normalized.setdefault("grasp", {})
    normalized.setdefault("place", {})
    normalized.setdefault("release", {})
    normalized.setdefault("safety", {})
    normalized.setdefault("warnings", [])
    return normalized


def validate_task_recipe(recipe: dict[str, Any]) -> dict[str, Any]:
    rec = normalize_task_recipe(recipe)
    errors: list[str] = []
    warnings: list[str] = []
    if rec.get("schema_version") != "workcell_task/v1":
        errors.append("schema_version must equal workcell_task/v1")
    task = rec["task"]
    task_type = task.get("type")
    if task_type not in SUPPORTED_TASK_TYPES:
        errors.append("task.type is unknown")
    if task_type in {"pick_place", "sorting"} and not task.get("pick_source"):
        errors.append("task.pick_source is required for pick_place/sorting")
    if task_type in {"pick_place", "sorting"} and not task.get("place_target"):
        errors.append("task.place_target is required for pick_place/sorting")
    if task_type != "inspection_only" and not rec["grasp"].get("strategy"):
        errors.append("grasp.strategy is required unless task.type is inspection_only")
    if float(rec["grasp"].get("approach_distance_m", 0.0)) < 0:
        errors.append("approach_distance_m must be >= 0")
    if float(rec["grasp"].get("retreat_distance_m", 0.0)) < 0:
        errors.append("retreat_distance_m must be >= 0")
    if float(rec["place"].get("clearance_m", 0.0)) < 0:
        errors.append("place.clearance_m must be >= 0")

    safety = rec["safety"]
    if safety.get("fake_hardware_first") is not True:
        errors.append("safety.fake_hardware_first must be true")
    if safety.get("motion_command_sent") is not False:
        errors.append("safety.motion_command_sent must be false")
    if safety.get("runtime_execution_enabled") is not False:
        errors.append("safety.runtime_execution_enabled must be false")

    if task.get("pick_source") == "perception_detection" and not rec.get("epd_adapter"):
        warnings.append("perception_detection selected but no EPD adapter configured")
    warnings.append("conveyor_placeholder is visual/metadata only")
    if task.get("place_target") == "custom_pose_placeholder":
        warnings.append("custom_pose_placeholder selected")
    if not rec.get("tool_compatibility_known", False):
        warnings.append("unknown end-effector/tool compatibility")
    if task_type == "inspection_only":
        warnings.append("inspection_only skips grasp/release execution")

    return {"valid": not errors, "errors": errors, "warnings": warnings, "recipe": rec}


def build_offline_task_plan(recipe: dict[str, Any], environment_context: dict[str, Any] | None = None) -> dict[str, Any]:
    result = validate_task_recipe(recipe)
    rec = result["recipe"]
    status = "READY" if result["valid"] else "BLOCKED"
    task_type = rec["task"].get("type")
    names = ["validate_recipe", "resolve_pick_source", "resolve_place_target", "approach_pick", "apply_grasp_strategy", "retreat_from_pick", "transfer_placeholder", "approach_place", "release_object", "retreat_from_place", "complete"]
    steps = []
    for name in names:
        step_status = status
        desc = name.replace("_", " ")
        if task_type == "inspection_only" and name in {"apply_grasp_strategy", "release_object"}:
            step_status = "NOT_REQUIRED"
            desc += " (inspection-only no manipulation)"
        steps.append({"name": name, "description": desc, "source_fields": ["task", "grasp", "place", "release", "safety"], "safety_classification": SAFETY_MARKERS, "status": step_status})

    return {"result": "PASS" if result["valid"] else "FAIL", "warnings": result["warnings"], "errors": result["errors"], "environment_context": environment_context or {}, "steps": steps}


def write_task_plan_json(plan: dict[str, Any], output_dir: str | Path) -> Path:
    out = Path(output_dir); out.mkdir(parents=True, exist_ok=True)
    path = out / "task_plan_preview.json"
    path.write_text(json.dumps(plan, indent=2), encoding="utf-8")
    return path


def write_task_plan_markdown(plan: dict[str, Any], output_dir: str | Path) -> Path:
    out = Path(output_dir); out.mkdir(parents=True, exist_ok=True)
    path = out / "task_plan_preview.md"
    lines = ["# Task Plan Preview", f"Result: {plan['result']}", "", "## Safety", "- OFFLINE_ONLY", "- NO_MOTION_COMMAND", "- NO_MOVEIT_PLAN", "- NO_REAL_HARDWARE", "", "## Steps"]
    for step in plan["steps"]:
        lines.append(f"- **{step['name']}** ({step['status']}): {step['description']}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def write_task_plan_report(plan: dict[str, Any], output_dir: str | Path) -> dict[str, Path]:
    return {"json": write_task_plan_json(plan, output_dir), "markdown": write_task_plan_markdown(plan, output_dir)}
