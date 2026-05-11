# Workcell Builder Healthcheck (Stabilization/Acceptance)

This manual validates the post-Qt Workcell Builder stack in a safe, fake-hardware-first mode.

## 1) Targeted pytest acceptance suite

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment

PYTHONPATH=$PWD:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_workcell_builder_healthcheck.py \
  tests/test_workcell_builder_layout_preview_readiness.py \
  tests/test_workcell_builder_asset_picker_dialogs.py \
  tests/test_workcell_builder_functional_asset_selection.py \
  tests/test_workcell_builder_asset_picker.py \
  tests/test_workcell_builder_scene_manager_ux.py \
  tests/test_workcell_builder_placeholder_scene_validation.py \
  tests/test_workcell_builder_scene_scaffold.py \
  tests/test_workcell_builder_real_generation_buttons.py \
  tests/test_workcell_builder_package_consistency.py \
  tests/test_workcell_builder_launch_smoke_acceptance.py \
  tests/test_workcell_builder_idempotent_regeneration.py
```

## 2) Build validation

```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder --event-handlers console_direct+
```

```bash
source install/setup.bash
cd ~/workcell_ws/src/easy_manipulation_deployment
```

## 3) Healthcheck validation (safe default: skip launch)

```bash
python3 scripts/validate_workcell_builder_healthcheck.py \
  --repo-root . \
  --workspace ~/workcell_ws \
  --run-colcon \
  --skip-launch
```

## 4) Optional smoke launch (headless fake hardware only)

```bash
python3 scripts/validate_workcell_builder_healthcheck.py \
  --repo-root . \
  --workspace ~/workcell_ws \
  --run-colcon \
  --smoke-launch
```

The smoke mode launches:

```bash
ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=false
```

No motion commands or runtime execution are issued by this workflow.
