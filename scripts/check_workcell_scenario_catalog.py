#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import re
import sys
import yaml

VALID_STATUSES = {
    "supported_preview",
    "partial_preview",
    "needs_live_epd",
    "planned",
    "blocked",
    "complete_simulation",
    "real_hardware_later",
}

REQUIRED_FIELDS = {
    "id",
    "name",
    "category",
    "current_status",
    "required_capabilities",
    "missing_capabilities",
    "live_epd_feed_required",
    "fake_hardware_preview_supported",
    "task_intent_preview_supported",
    "real_hardware_ready",
    "next_pr",
}

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "catalog/scenarios/industrial_scenarios.yaml"
SCENARIOS_DOC = ROOT / "docs/roadmap/WORKCELL_STUDIO_INDUSTRIAL_SCENARIOS.md"
TODO_DOC = ROOT / "docs/roadmap/WORKCELL_STUDIO_TODO.md"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    report = {"errors": [], "warnings": [], "scenario_count": 0, "scenario_ids": []}

    data = yaml.safe_load(CATALOG.read_text())
    scenarios = data.get("scenarios", []) if isinstance(data, dict) else []
    report["scenario_count"] = len(scenarios)

    doc_text = SCENARIOS_DOC.read_text()
    todo_text = TODO_DOC.read_text()

    for i, scenario in enumerate(scenarios):
        missing = sorted(REQUIRED_FIELDS - set(scenario.keys()))
        if missing:
            report["errors"].append(f"scenario[{i}] missing fields: {', '.join(missing)}")
            continue

        sid = scenario["id"]
        report["scenario_ids"].append(sid)

        if scenario["current_status"] not in VALID_STATUSES:
            report["errors"].append(f"{sid}: invalid status '{scenario['current_status']}'")

        if not scenario["missing_capabilities"]:
            report["errors"].append(f"{sid}: missing_capabilities must not be empty")
        if not scenario["next_pr"]:
            report["errors"].append(f"{sid}: next_pr must not be empty")

        if sid not in doc_text:
            report["errors"].append(f"{sid}: not mentioned in WORKCELL_STUDIO_INDUSTRIAL_SCENARIOS.md")

    for sec in ["## P0", "## P1", "## P2", "## P3", "## P4"]:
        if sec not in todo_text:
            report["errors"].append(f"TODO missing section: {sec}")

    ok = not report["errors"]
    print("Workcell Scenario Catalog Check")
    print(f"- scenarios: {report['scenario_count']}")
    print(f"- errors: {len(report['errors'])}")
    if report["errors"]:
        for err in report["errors"]:
            print(f"  * {err}")

    if args.json_out:
        args.json_out.write_text(json.dumps(report, indent=2))

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
