# OFFLINE_PLAN_PREVIEW_REQUEST_V1

Schema id: `offline_plan_preview_request/v1`.

This artifact is **offline metadata only** and is a safe bridge from `task_recipe/v1` toward future MoveIt/RViz visual plan preview.

- It does **not** execute motion.
- It does **not** launch ROS.
- It does **not** call MoveIt services.
- It is **not** a safety certificate.
- Real runtime execution remains separately guarded.

```yaml
schema: offline_plan_preview_request/v1
source:
  task_recipe: path/or/id
  task_flow_summary: path/or/generated
request:
  id: plan_preview_<task_id>
  mode: offline_preview
  robot:
    id: <robot/capability if known>
    planning_group: <if known or null>
  tool:
    id: <end_effector/capability if known>
    grasp_strategy: <strategy_ref>
  pick:
    source_id: <pick_source>
    object_filter: {...}
    approach_axis: z_down
    approach_distance_m: 0.12
    retreat_axis: z_up
    retreat_distance_m: 0.10
  place:
    target_id: <place_target>
    place_offset_xyz: [0, 0, 0.08]
    release_strategy: tool_release
    retreat_axis: z_up
    retreat_distance_m: 0.12
  waypoints: [home, pre_pick, pick, post_pick, pre_place, place, post_place]
  checks:
    require_pick_source: true
    require_place_target: true
    require_grasp_strategy: true
    require_collision_scene: true
    require_fake_hardware_default: true
safety:
  metadata_only: true
  motion_started: false
  ros_launch_started: false
  moveit_service_called: false
  runtime_io_applied: false
```
