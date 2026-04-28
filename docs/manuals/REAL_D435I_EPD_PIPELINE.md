# REAL_D435I_EPD_PIPELINE

This is the intended production data flow:

```text
Intel RealSense D435i / EPD perception
    ↓
detected_objects/v1 snapshot
    ↓
task_recipe adapter
    ↓
runtime_execution_plan/v1
  (includes destination_resolved pose metadata per routed step)
    ↓
EMD grasp bridge payload
  (preserves destination release pose metadata + fallback warnings)
    ↓
existing grasp_requests / grasp_tasks runtime
```

Mock objects remain only for offline tests and CI.

## Bringup commands

```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py ...

ros2 launch easy_perception_deployment run.launch.py ...
```

## Capture one snapshot

```bash
python3 scripts/capture_epd_detected_objects.py \
  --topic /easy_perception_deployment/epd_localize_output \
  --once \
  --timeout 10 \
  --output reports/detected_objects/latest.yaml
```

## Run full offline conversion pipeline

```bash
python3 scripts/run_perception_task_pipeline.py \
  --task-recipe path/to/task_recipe.yaml \
  --detected-objects reports/detected_objects/latest.yaml \
  --output-dir reports/runtime_pipeline \
  --dry-run
```

## Optional guarded runtime send

```bash
python3 scripts/run_perception_task_pipeline.py \
  --task-recipe path/to/task_recipe.yaml \
  --detected-objects reports/detected_objects/latest.yaml \
  --output-dir reports/runtime_pipeline \
  --send-to-ros \
  --ros-interface service
```

## Runtime boundary note

Destination-aware release pose metadata is now preserved end-to-end through:

`detected_objects/v1 -> task recipe decision rule -> runtime_execution_plan/v1 destination_resolved -> emd_grasp_bridge_payload/v1 destination_pose`.

Current runtime execution (`grasp_requests` / `grasp_tasks`) keeps legacy release behavior (`release_x_offset`, `release_use_grasp_z`) by default, and now adds a safe opt-in adapter for explicit destination release poses via bridge payload path.

Recommended safe runtime knobs:

- `use_explicit_release_pose: false` (default; set `true` only when bridge payload path is provided)
- `explicit_release_pose_bridge_payload_path: /path/to/emd_grasp_bridge_payload.json`
- `explicit_release_pose_frame_policy: require_planning_frame`
- `fallback_to_legacy_release: true`

This is required for physical sorting workflows (e.g., colour/shape/garbage bins) where route-selected destinations must map to real release coordinates while preserving existing scene compatibility.

## Not production-ready yet

- runtime interface extension to consume explicit destination release pose
- safety validation
- IO validation
- physical robot commissioning
