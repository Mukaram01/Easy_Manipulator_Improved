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

## MVP live smoke-test commands (generated-cell)

Run tests from repo root (no `PYTHONPATH` required):

```bash
python3 -m pytest -q \
  tests/test_workcell_discovery.py \
  tests/test_run_cell_cycle_panel.py \
  tests/test_run_generated_cell_cycle.py \
  tests/test_capture_epd_detected_objects.py \
  tests/test_generated_cell_acceptance.py \
  tests/test_replay_emd_bridge_payload.py
```

Check camera topics:

```bash
ros2 topic list | grep camera
```

Check EPD topic:

```bash
ros2 topic list | grep easy_perception
```

## Verify TF from camera to world

Before trusting any live replay/execution, verify TF is continuously available from the camera frame to the planning frame (`world` by default):

```bash
ros2 run tf2_ros tf2_echo world camera_depth_optical_frame
ros2 run tf2_ros tf2_echo world camera_color_optical_frame
```

Expected result: transforms print continuously and do not timeout.

Capture one live EPD detection (best-effort QoS, expected PASS/WARN with `objects >= 1`):

```bash
python3 scripts/capture_epd_detected_objects.py \
  --topic /easy_perception_deployment/epd_localize_output \
  --output /tmp/mvp1_live_smoke_test/detected_objects_live.yaml \
  --scene-package ur5_2f_test \
  --timeout 15 \
  --min-objects 1 \
  --once \
  --qos-reliability best_effort \
  --target-frame world \
  --require-transform \
  --json
```

Run generated-cell dry-run from live EPD capture (expected PASS/WARN; `perception_source=live_epd` only when ROS message capture succeeds):

```bash
python3 scripts/run_generated_cell_cycle.py \
  --scene-package ur5_2f_test \
  --task-recipe tests/fixtures/task_recipes/valid_garbage_sorting.yaml \
  --capture-live \
  --epd-topic /easy_perception_deployment/epd_localize_output \
  --epd-qos-reliability best_effort \
  --output-dir /tmp/mvp1_live_smoke_test \
  --min-objects 1 \
  --capture-timeout 15 \
  --once \
  --dry-run \
  --no-replay \
  --target-frame world \
  --require-transform \
  --json
```

If transform is unavailable and `--require-transform` is active, live capture fails conservatively.
Only use `--allow-untransformed` for debugging; this is reported as WARN and runtime replay is unsafe/not recommended.

Offline fake mode is now explicit (`--offline-fake-live`) and intended only for tests/CI.

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

## Exact MVP command sequence (UR5+2F, live dry-run only)

> Safety: this flow is **dry-run/preview only**. Keep `safe_for_robot_motion: false`, use `--dry-run --no-replay`, and do not send controller goals.

1. Launch RealSense:
```bash
ros2 launch realsense2_camera rs_launch.py
```
2. Launch EPD:
```bash
ros2 launch easy_perception_deployment run.launch.py
```
3. Verify topics:
```bash
ros2 topic list | grep -E 'camera|easy_perception_deployment'
```
4. Verify TF world -> camera_depth_optical_frame:
```bash
ros2 run tf2_ros tf2_echo world camera_depth_optical_frame
```
5. Generate workcell bundle:
```bash
python3 scripts/generate_workcell_from_cell_definition.py \
  cell_definitions/demo_ur5_sorting_cell.yaml \
  --output-dir /tmp/generated_workcells \
  --package-name ur5_2f_live_garbage_sorting
```
6. Run generated bundle in live dry-run mode:
```bash
python3 scripts/run_generated_workcell_bundle.py \
  --workcell /tmp/generated_workcells/ur5_2f_live_garbage_sorting \
  --output-dir /tmp/ur5_2f_live_run \
  --capture-live \
  --epd-topic /easy_perception_deployment/epd_localize_output \
  --epd-qos-reliability best_effort \
  --target-frame world \
  --require-transform \
  --gated-dry-run \
  --dry-run \
  --no-replay \
  --preflight-live \
  --preflight-check-tf \
  --preflight-check-ros-topics \
  --preview-task-flow \
  --json
```
7. Preview task-flow markers in RViz:
```bash
python3 scripts/preview_generated_workcell_bundle.py \
  --workcell /tmp/generated_workcells/ur5_2f_live_garbage_sorting \
  --show-task-flow \
  --task-flow-preview /tmp/ur5_2f_live_run/task_flow_preview.json \
  --publish-markers
```
