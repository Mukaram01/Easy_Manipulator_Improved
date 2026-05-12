# Workcell Builder Planning Readiness (Dry-Run Only)

This feature adds **planning readiness checks** and optional **dry-run planning request artifacts**.

- No robot motion execution.
- No MoveIt execute call.
- No gripper command.
- No conveyor runtime command.
- No real hardware requirement.
- Fake hardware remains the default recommendation.

Artifacts written to `<scene>/preview/`:
- `planning_readiness_report.yaml`
- `planning_readiness_report.json`
- `dry_run_planning_request.yaml`

`runtime_mode` is `dry_run_readiness_only` and `can_execute` is always `false`.


## Class-to-Place-Zone Routing
Preview-only class-to-place-zone routing maps EPD class labels from snapshot/live bridge to destination place zones, with unknown/default typically routed to reject bin. No robot motion, no MoveIt execution, no gripper or conveyor hardware commands, EPD GUI remains separate, real hardware later.


## EMD Grasp Request Contract (preview-only)
- EPD provides perception only.
- Existing EMD planner remains downstream planner.
- Workcell Studio now emits grasp_strategy.yaml, emd_grasp_planner_request.yaml/json, and readiness reports.
- No planner execution or robot motion is called in this stage.
