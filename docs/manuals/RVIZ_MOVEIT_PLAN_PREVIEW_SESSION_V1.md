# RVIZ_MOVEIT_PLAN_PREVIEW_SESSION_V1

`rviz_moveit_plan_preview_session/v1` is a **session-preparation artifact** for guarded offline plan preview.

- It does **not** start ROS.
- It does **not** call MoveIt services.
- It does **not** move the robot.
- Suggested commands are generated only; the user must run them manually.
- Real mode remains blocked/guarded.

Schema highlights include `source`, `session`, `rviz_moveit`, `plan_preview`, and `safety` with fake-hardware-required defaults.
