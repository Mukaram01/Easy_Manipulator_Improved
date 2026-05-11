#!/usr/bin/env python3
from __future__ import annotations
import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = REPO_ROOT / "easy_manipulation_deployment" / "emd_demo_nodes" / "run_grasp_execution"
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from run_grasp_execution.task_recipe import (  # noqa: E402
    build_offline_task_plan,
    load_task_recipe,
    validate_task_recipe,
    write_task_plan_report,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Offline dry-run task recipe preview")
    parser.add_argument("--recipe", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--print-plan", action="store_true")
    args = parser.parse_args(argv)

    recipe = load_task_recipe(args.recipe)
    validation = validate_task_recipe(recipe)
    if args.strict and validation["warnings"]:
        validation["errors"].append("strict mode: warnings present")
        validation["valid"] = False
    plan = build_offline_task_plan(validation["recipe"])
    reports = write_task_plan_report(plan, args.output_dir)

    if args.print_plan:
        for step in plan["steps"]:
            print(f"- {step['name']}: {step['status']}")

    print(f"recipe={args.recipe}")
    print(f"json={reports['json']}")
    print(f"markdown={reports['markdown']}")
    for warning in validation["warnings"]:
        print(f"WARN: {warning}")
    for error in validation["errors"]:
        print(f"ERROR: {error}")

    if validation["valid"]:
        print("WORKCELL_TASK_RECIPE_PREVIEW: PASS")
        return 0
    print("WORKCELL_TASK_RECIPE_PREVIEW: FAIL")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
