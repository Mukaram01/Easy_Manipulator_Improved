# RVIZ_MOVEIT_PLAN_PREVIEW_SESSION_V1

`rviz_moveit_plan_preview_session/v1` is a **session-preparation artifact** for guarded offline plan preview.

- It does **not** start ROS.
- It does **not** call MoveIt services.
- It does **not** move the robot.
- Suggested commands are generated only; the user must run them manually.
- Real mode remains blocked/guarded.

Schema highlights include `source`, `session`, `rviz_moveit`, `plan_preview`, and `safety` with fake-hardware-required defaults.


## Smoke launch report
See `FAKE_HARDWARE_SMOKE_LAUNCH_REPORT_V1.md` for smoke verifier output schema.


Readiness integration: `planning_scene_readiness_report/v1` can consume plan-preview session artifacts for readiness classification.
