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

Current runtime execution (`grasp_requests` / `grasp_tasks`) still uses legacy release behavior (`release_x_offset`, `release_use_grasp_z`) because the existing EMD messages do not include explicit place/release pose fields yet.

## Not production-ready yet

- runtime interface extension to consume explicit destination release pose
- safety validation
- IO validation
- physical robot commissioning
