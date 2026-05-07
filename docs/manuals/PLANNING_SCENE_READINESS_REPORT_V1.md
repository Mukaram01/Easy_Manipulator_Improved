# PLANNING_SCENE_READINESS_REPORT_V1

Schema: `planning_scene_readiness_report/v1`

This report is a **read-only file/metadata readiness check** for generated scenes and Workcell Studio imports.

- This is **not planning**.
- This is **not a safety certificate**.
- It does **not contact MoveIt**.
- It only checks files, metadata, and prior reports.

## Structure

```yaml
schema: planning_scene_readiness_report/v1
source:
  scene_package: ...
  cell_definition: ...
  task_recipe: ...
  offline_plan_preview_request: ...
  rviz_moveit_plan_preview_session: ...
  smoke_launch_report: ...
checks:
  scene_package:
    package_xml_exists: true/false
    cmake_lists_exists: true/false
    launch_file_exists: true/false
    demo_launch_detected: true/false
  robot:
    robot_model_detected: true/false
    robot_capability_id: ...
    planning_group_known: true/false/unknown
  tool:
    end_effector_detected: true/false
    tool_capability_id: ...
    grasp_strategy_detected: true/false
  planning:
    fake_hardware_required: true
    fake_hardware_default_known: true/false/unknown
    move_group_expected: true/false/unknown
    rviz_expected: true/false/unknown
    controllers_metadata_present: true/false/unknown
  scene_objects:
    support_surface_detected: true/false
    collision_objects_detected: true/false
    pick_source_detected: true/false
    place_target_detected: true/false
  task:
    task_recipe_present: true/false
    offline_plan_preview_request_present: true/false
    waypoint_count: ...
    pick_source_id: ...
    place_target_id: ...
    grasp_strategy: ...
  smoke:
    smoke_report_present: true/false
    smoke_status: PASS/WARN/FAIL/SKIPPED/unknown
  safety:
    motion_command_sent: false
    moveit_plan_service_called: false
    runtime_execution_called: false
    real_hardware_enabled: false
    runtime_io_applied: false
result:
  readiness: PASS/WARN/FAIL
  classification:
    - physical_scene_only
    - task_ready_offline
    - plan_preview_request_ready
    - rviz_preview_prepared
    - fake_hardware_smoke_checked
    - blocked_missing_scene
    - blocked_missing_task
    - blocked_unsafe_flags
  blockers: []
  warnings: []
  suggested_next_actions: []
```
