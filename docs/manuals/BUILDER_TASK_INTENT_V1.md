# BUILDER_TASK_INTENT_V1

`workcell_builder` remains the physical scene builder. This sidecar is metadata-only task intent.

- Does **not** command robot motion.
- Does **not** replace `workcell_builder`.
- Captures pick/place/grasp intent from a builder-generated scene.
- Provides the missing layer between scene layout and executable behavior.

```yaml
schema: workcell_builder_task_intent/v1
scene_package: example_scene
task:
  id: sorting_task_001
  type: pick_place
  mode: offline_preview
pick:
  source: {type: zone, id: pick_zone_main, label: Main pick zone}
  object_filter: {class_id: any, color: red}
grasp:
  strategy_ref: suction_top_basic
  approach_axis: z_down
  approach_distance_m: 0.12
  retreat_axis: z_up
  retreat_distance_m: 0.10
place:
  target: {type: destination, id: bin_red, label: Red bin}
  release_strategy: tool_release
routing:
  rules: []
safety:
  metadata_only: true
  runtime_io_applied: false
  motion_started: false
  ros_launch_started: false
```

## Task Flow Summary
Builder task intent validation and preview now expose readiness classifications and missing fields.
