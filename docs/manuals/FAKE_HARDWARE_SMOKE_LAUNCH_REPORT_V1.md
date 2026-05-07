# Fake Hardware Smoke Launch Report (v1)

Schema: `fake_hardware_smoke_launch_report/v1`

This report captures a guarded **fake-hardware-only** launch/readiness smoke check for prepared RViz/MoveIt plan-preview sessions.

- Default mode is **dry-run/report-only**.
- Actual launch requires explicit `--execute`.
- Launch is limited to fake-hardware commands.
- Process is terminated after timeout.
- This does **not** prove planning validity or execution safety.
- This does **not** move the robot.

## Schema

```yaml
schema: fake_hardware_smoke_launch_report/v1
source:
  plan_preview_session: ...
  suggested_command: ...
  scene_package: ...
run:
  mode: fake_hardware_smoke_check
  actually_launched: true/false
  dry_run_only: true/false
  timeout_s: 20
  started_at: ...
  ended_at: ...
  return_code: ...
checks:
  command_safety_checked: true/false
  command_contains_fake_hardware_true: true/false
  forbidden_real_hardware_tokens_absent: true/false
  launch_started: true/false
  launch_exited_cleanly_or_timeout_terminated: true/false
  move_group_seen_in_output: true/false/unknown
  rviz_seen_in_output: true/false/unknown
  robot_state_publisher_seen_in_output: true/false/unknown
safety:
  motion_command_sent: false
  moveit_plan_service_called: false
  runtime_execution_called: false
  real_hardware_enabled: false
  runtime_io_applied: false
result:
  status: PASS/WARN/FAIL/SKIPPED
  warnings: []
  errors: []
  suggested_next_actions: []
```


Readiness integration: `planning_scene_readiness_report/v1` can consume fake-hardware smoke result artifacts.
