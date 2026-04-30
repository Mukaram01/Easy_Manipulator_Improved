# MVP-1 Generated Cell Acceptance (Offline-First)

## Purpose
MVP-1 adds a conservative acceptance flow that proves a generated workcell can execute a user task recipe end-to-end with current backend components:

`detected_objects/v1 -> validation -> task recipe validation -> runtime_execution_plan/v1 -> EMD grasp bridge payload`

This is intentionally **no-new-GUI** and **no new scene/assets architecture**. It prepares backend hooks for a future GUI button chain.

## Example acceptance scenario
UR5 + Robotiq 2F style sorting logic:
- if `colour == red` -> `red_bin`
- if `colour == blue` -> `blue_bin`
- else fallback -> `reject_bin`

Fixtures:
- `tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml`
- `tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml`

## Run the acceptance flow
```bash
python3 scripts/run_generated_cell_acceptance.py \
  --scene-package ur5_robotiq_generated_cell \
  --task-recipe tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml \
  --detected-objects tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml \
  --output-dir reports/generated_cell_acceptance \
  --json
```

Arguments:
- `--scene-package`: generated scene package name to probe in `scenes/` and `install/share` style locations.
- `--task-recipe`: task recipe input.
- `--detected-objects`: detected objects input.
- `--output-dir`: artifact directory.
- `--strict`: convert WARN to FAIL.
- `--json`: machine-readable summary.

## PASS / WARN / FAIL
- **PASS**: all pipeline stages succeed and no warnings were produced.
- **WARN**: pipeline succeeded, but operator attention is required (example: scene package not found offline, missing destination pose).
- **FAIL**: blockers occurred, or strict mode escalated warnings.

Known runtime boundary is always surfaced:

> destination_resolved is present in the bridge payload; runtime release execution may still use existing release fallback until runtime destination support is implemented.

## Generated-cell readiness checks
The script performs tolerant checks for generated scene package files (for example `package.xml`, `environment.yaml`, plus `launch/`, `config/` directories when available). Missing local ROS workspace/install is WARN by default; strict mode promotes to FAIL.

## Mapping to future GUI buttons
Future GUI flow can call this backend sequence:
1. Validate/import environment and generated scene package.
2. Validate detected objects snapshot and task recipe.
3. Generate runtime plan.
4. Generate EMD bridge payload.
5. Show readiness summary (PASS/WARN/FAIL).

## Real D435i / EPD snapshot path (later)
Later, replace the fixture input with captured snapshot output from `scripts/capture_epd_detected_objects.py`, then rerun this acceptance script for the same task recipe.

## Moving from offline acceptance to ROS execution
After offline acceptance reaches PASS/WARN-understood status:
1. Build/source ROS 2 Humble workspace.
2. Ensure generated scene package is discoverable.
3. Launch existing `run_grasp_execution` stack.
4. Feed generated runtime plan / bridge payload through current runtime integration path.
