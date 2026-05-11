# Workcell Golden Demo Acceptance Pack

This artifact is for **developer/test/reference** use. It is **not** a normal operator-first main-page button.

## Generate

```bash
python3 scripts/generate_golden_workcell_demo.py \
  --output-dir /tmp/workcell_golden_demo \
  --scene-name ur5_2f_golden_demo \
  --validate \
  --print-summary
```

Default output is safe:
- `/tmp/workcell_golden_demo/ur5_2f_golden_demo`

To write into repo scenes, opt in explicitly with `--install-into-scenes`.

## Validate

```bash
python3 scripts/validate_golden_workcell_demo.py \
  --scene-dir /tmp/workcell_golden_demo/ur5_2f_golden_demo
```

## Produced files

- `environment.yaml` (`schema_version: workcell_scene/v1`)
- `config/task_recipe.yaml` (`schema_version: workcell_task/v1`)
- `config/perception_metadata.json`
- `config/compatibility_metadata.json`
- `config/readiness_overlay_metadata.json`
- `config/visual_layout_metadata.json`
- `preview/workcell_preview.svg`
- `preview/workcell_preview.html`
- `workcell_studio_summary.json`
- `workcell_studio_summary.md`
- `readiness_report.json`

## Fake-hardware-first launch only

```bash
colcon build --symlink-install --packages-select ur5_2f_golden_demo
source install/setup.bash
ros2 launch ur5_2f_golden_demo demo.launch.py use_fake_hardware:=true launch_rviz:=true
```

Real hardware is intentionally not enabled in this pack.

## Safety proof flags

- `fake_hardware_first: true`
- `motion_command_sent: false`
- `runtime_execution_enabled: false`
- `real_hardware_enabled: false`
- `moveit_plan_service_called: false`

This pack supports deterministic regression testing and reference demos for the full Workcell Studio pipeline while remaining offline-safe.
