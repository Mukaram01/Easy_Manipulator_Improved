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
