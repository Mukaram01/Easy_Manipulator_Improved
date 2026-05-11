# WORKCELL Fake-Hardware Smoke Acceptance

## Purpose
Provide one repeatable acceptance gate for Workcell Studio generated scenes that validates safe fake-hardware launch readiness in headless mode.

## Safe default behavior
`run_workcell_fake_hardware_smoke.py` defaults to static checks only. It does **not** launch ROS unless `--run-launch` is explicitly passed.

## Static-only mode (default)
Checks include schema validation, catalog validation, scene artifacts, safety flags, and fake-hardware launch guidance (`use_fake_hardware:=true`, `launch_rviz:=false`).

## Optional launch mode
Optional local ROS smoke can be enabled with `--run-launch`. Launch command is headless and fake hardware only:

`ros2 launch <scene_package> demo.launch.py use_fake_hardware:=true launch_rviz:=false`

## Golden demo smoke command
```bash
python3 scripts/run_workcell_fake_hardware_smoke.py \
  --generate-golden-demo \
  --skip-launch \
  --print-summary
```

## Generated scene smoke command
```bash
python3 scripts/run_workcell_fake_hardware_smoke.py \
  --scene-dir /tmp/workcell_golden_demo/ur5_2f_golden_demo \
  --skip-launch \
  --print-summary
```

## Optional local ROS smoke only
```bash
python3 scripts/run_workcell_fake_hardware_smoke.py \
  --scene-dir /tmp/workcell_golden_demo/ur5_2f_golden_demo \
  --workspace ~/workcell_ws \
  --run-launch \
  --headless \
  --timeout-seconds 30
```

## Safety note
This acceptance gate is fake-hardware-first and offline-safe. It does **not** prove real robot readiness.
