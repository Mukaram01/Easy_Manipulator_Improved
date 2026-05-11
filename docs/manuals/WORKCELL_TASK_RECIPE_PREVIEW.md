# WORKCELL Task Recipe Preview

`task_recipe.yaml` is the generated task/grasp intent artifact from Workcell Studio (`schema_version: workcell_task/v1`).

## What the dry-run preview does
- Loads and validates the recipe.
- Builds an **offline-only** ordered task plan preview.
- Writes `task_plan_preview.json` and `task_plan_preview.md`.

## What it does NOT do
- No robot motion commands.
- No MoveIt planning service calls.
- No trajectory execution.
- No real hardware enablement.

## Manual preview command
```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
python3 scripts/preview_task_recipe.py \
  --recipe scenes/<scene_name>/config/task_recipe.yaml \
  --output-dir /tmp/workcell_task_preview \
  --print-plan
```

## Tests
```bash
PYTHONPATH=$PWD:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_task_recipe_loader_preview.py \
  tests/test_workcell_builder_task_grasp_strategy.py \
  tests/test_workcell_builder_healthcheck.py \
  tests/test_workcell_builder_layout_preview_readiness.py \
  tests/test_workcell_builder_launch_smoke_acceptance.py \
  tests/test_workcell_builder_package_consistency.py
```

## Build
```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder run_grasp_execution --event-handlers console_direct+
```

## Healthcheck
```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
python3 scripts/validate_workcell_builder_healthcheck.py \
  --repo-root . \
  --workspace ~/workcell_ws \
  --skip-colcon \
  --skip-launch
```
