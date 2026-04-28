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
    ↓
EMD grasp bridge payload
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

## Not production-ready yet

- final destination-aware placement/release logic
- safety validation
- IO validation
- physical robot commissioning
