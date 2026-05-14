# Workcell Studio Golden Flow QA

## Offline Golden Flow
Run:

```bash
python3 scripts/run_workcell_studio_golden_flow.py --scene-dir /tmp/ur5_robotiq_pick_place --json
```

Expected artifacts:
- `layout/workcell_studio_layout.yaml`
- `generated/workcell_studio_layout_merge_report.json`
- `acceptance/generated_scene_acceptance.json`
- `demo/workcell_studio_demo_report.json`
- `preview_launch/preview_launch_session.json`
- `golden_flow/workcell_studio_golden_flow_report.json`
- `golden_flow/workcell_studio_golden_flow_summary.txt`
- `golden_flow/workcell_studio_golden_flow_dashboard.html`

Safety expectations:
- `use_fake_hardware:=true` present in preview commands.
- No denylisted args (`use_fake_hardware:=false`, `real_hardware:=true`, `runtime_execution_enabled:=true`, `execute:=true`, `command_robot:=true`, `send_motion:=true`).
- No robot motion commanded.

If layout is stale (layout timestamp newer than merge/acceptance/demo), rerun Generate Scene / Layout Merge before preview.

If a helper script path fails, GUI must show: `Could not find Workcell Studio helper script` and avoid crash.

## Manual Validation Checklist
```bash
cd ~/workcell_ws
git pull --ff-only
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder
source install/setup.bash
workcell_builder
```

GUI flow:
- New Cell: UR5 + Robotiq 2F Pick and Place
- Add table/bin/object/camera/pick zone/place zone
- Drag/move them
- Save Layout
- Run Layout Merge / Generate Scene
- Run Generated Scene Acceptance
- Run Demo Readiness
- Open Preview Launch
- Copy fake-hardware launch command
- Run Build if dependencies are ready
- Do not run real hardware
- Confirm no robot motion was commanded
