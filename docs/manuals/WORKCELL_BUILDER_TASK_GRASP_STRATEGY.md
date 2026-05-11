# Workcell Builder Task & Grasp Strategy

This panel adds configurable **Task & Grasp Strategy** inside the existing Qt `workcell_builder`.

## Scope and safety
- Offline/fake-hardware planning metadata only.
- No robot motion is commanded.
- No MoveIt planning services are called.
- Fake-hardware-first behavior remains the default.

## What the panel configures
- Task type, pick source, place target.
- Grasp strategy, orientation mode, approach axis.
- Approach/retreat/place-clearance distances.
- Allowed roll/yaw alternatives.
- Suction cup count.
- Release strategy.

## Defaults
- Finger tool default: `finger_top`, `open_gripper`, vertical, z_down, approach `0.12`, retreat `0.10`.
- Suction tool default: `suction_top`, `vacuum_off`, vertical, z_down, approach `0.12`, retreat `0.10`.

## Generated artifact
- Task recipe is written to: `<generated_scene>/config/task_recipe.yaml`.
- Schema: `workcell_task/v1`.

## Example finger recipe
```yaml
grasp:
  strategy: finger_top
release:
  strategy: open_gripper
```

## Example suction recipe
```yaml
grasp:
  strategy: suction_top
  suction_cups: 1
release:
  strategy: vacuum_off
```

## Manual validation commands
```bash
cd ~/workcell_ws/src/easy_manipulation_deployment

PYTHONPATH=$PWD:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_workcell_builder_task_grasp_strategy.py \
  tests/test_workcell_builder_healthcheck.py \
  tests/test_workcell_builder_layout_preview_readiness.py \
  tests/test_workcell_builder_asset_picker_dialogs.py \
  tests/test_workcell_builder_functional_asset_selection.py \
  tests/test_workcell_builder_asset_picker.py \
  tests/test_workcell_builder_scene_manager_ux.py \
  tests/test_workcell_builder_placeholder_scene_validation.py \
  tests/test_workcell_builder_package_consistency.py \
  tests/test_workcell_builder_launch_smoke_acceptance.py

cd ~/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder --event-handlers console_direct+

source install/setup.bash
workcell_builder
```
