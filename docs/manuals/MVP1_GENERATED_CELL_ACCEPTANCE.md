# MVP-1 Generated Cell Acceptance (Offline-First)

## Purpose
MVP-1 adds a conservative acceptance flow that proves a generated workcell can execute a user task recipe end-to-end with current backend components:

`detected_objects/v1 -> validation -> task recipe validation -> runtime_execution_plan/v1 -> EMD grasp bridge payload`

This is intentionally **no-new-GUI** and **no new scene/assets architecture**. It prepares backend hooks for a future GUI button chain.

## Example acceptance scenario
UR5 + Robotiq 2F style sorting logic:
- if `colour == red` -> `red_bin`
- if `colour == blue` -> `blue_bin`
- else fallback -> `reject_bin`

Fixtures:
- `tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml`
- `tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml`

## Run the acceptance flow
```bash
python3 scripts/run_generated_cell_acceptance.py \
  --scene-package ur5_robotiq_generated_cell \
  --task-recipe tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml \
  --detected-objects tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml \
  --output-dir reports/generated_cell_acceptance \
  --json
```

Arguments:
- `--scene-package`: generated scene package name to probe in `scenes/` and `install/share` style locations.
- `--task-recipe`: task recipe input.
- `--detected-objects`: detected objects input.
- `--output-dir`: artifact directory.
- `--strict`: convert WARN to FAIL.
- `--json`: machine-readable summary.

## PASS / WARN / FAIL
- **PASS**: all pipeline stages succeed and no warnings were produced.
- **WARN**: pipeline succeeded, but operator attention is required (example: scene package not found offline, missing destination pose).
- **FAIL**: blockers occurred, or strict mode escalated warnings.

## First offline operator test (recommended baseline)
Use this sequence from repository root on a fresh machine:

```bash
colcon build --symlink-install --packages-skip tesseract_rviz tesseract_planning_server tesseract_ros_examples
```

```bash
python3 -m pytest -q tests/test_workcell_discovery.py tests/test_run_cell_cycle_panel.py tests/test_run_generated_cell_cycle.py
```

```bash
python3 scripts/run_generated_cell_cycle.py \
  --scene-package ur5_2f_test \
  --task-recipe tests/fixtures/task_recipes/valid_garbage_sorting.yaml \
  --detected-objects tests/fixtures/detected_objects/valid_epd_garbage_sorting.yaml \
  --output-dir /tmp/mvp1_valid_test \
  --min-objects 1 \
  --once \
  --dry-run \
  --no-replay \
  --json
```

Expected result:
- **PASS** or **WARN** is acceptable for first offline operator test when `blockers=[]`.
- **FAIL** is not acceptable for first offline operator test.

Fixture naming guidance:
- `valid_*`: operator-ready examples.
- `warn_*`: warning-path examples.
- `fail_*`, `missing_*`, `low_confidence_*`, `unknown_*`: developer/test fixtures (kept for regression testing, not default operator selection).

Previous runtime boundary (now addressed in `run_grasp_execution` destination-aware mode):

> destination_resolved is present in the bridge payload; runtime release execution may still use existing release fallback until runtime destination support is implemented.

## Generated-cell readiness checks
The script performs tolerant checks for generated scene package files (for example `package.xml`, `environment.yaml`, plus `launch/`, `config/` directories when available). Missing local ROS workspace/install is WARN by default; strict mode promotes to FAIL.

## Mapping to future GUI buttons
Future GUI flow can call this backend sequence:
1. Validate/import environment and generated scene package.
2. Validate detected objects snapshot and task recipe.
3. Generate runtime plan.
4. Generate EMD bridge payload.
5. Show readiness summary (PASS/WARN/FAIL).

## Real D435i / EPD snapshot path (later)
Later, replace the fixture input with captured snapshot output from `scripts/capture_epd_detected_objects.py`, then rerun this acceptance script for the same task recipe.

## Moving from offline acceptance to ROS execution
After offline acceptance reaches PASS/WARN-understood status:
1. Build/source ROS 2 Humble workspace.
2. Ensure generated scene package is discoverable.
3. Launch existing `run_grasp_execution` stack.
4. Feed generated runtime plan / bridge payload through current runtime integration path.

## Runtime destination-aware placement
Offline acceptance determines routing (`destination_id`) and writes `destination_resolved.destination_pose` into the generated bridge payload. Runtime now consumes that pose for the release/place planning target when available.

- If destination pose exists and is valid, runtime logs:
  - `Using destination-aware release target: destination_id=<id> frame=<frame> ...`
- If destination pose is missing or unresolved, runtime logs:
  - `No destination pose supplied; using legacy release fallback.`
- Legacy compatibility fallback remains available via parameter when destination release planning fails.

### Runtime parameters (grasp_execution.yaml)
- `use_destination_release: true`
- `destination_release_fallback_to_legacy: true`
- `destination_release_require_frame: true`

Frame notes:
- Destination poses should include `frame_id`.
- If `destination_release_require_frame=false` and `frame_id` is missing, runtime warns and safely defaults to `planning_frame`.
- If frame policy rejects a destination frame (for example planning-frame-only policy), the job fails destination-aware planning and uses legacy fallback only when enabled.

### Example end-to-end commands
```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/setup.bash

python3 scripts/run_generated_cell_acceptance.py \
  --scene-package ur5_2f_test \
  --task-recipe tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml \
  --detected-objects tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml \
  --output-dir /tmp/mvp1 \
  --json

ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_test
```

## Replay a generated task into runtime

This replay flow is the backend equivalent of the future GUI **"Simulate Cycle"** button.

1. Acceptance generates `emd_grasp_bridge_payload.json` from detected objects + task recipe.
2. Replay sends that payload through the existing runtime communication boundary (`grasp_requests` service or `grasp_tasks` topic) used by `run_grasp_execution`.
3. `run_grasp_execution` performs destination-aware placement when destination pose data is available, and falls back to legacy release mode when not.

Replay command:

```bash
python3 scripts/replay_emd_bridge_payload.py \
  --payload /tmp/mvp1/emd_grasp_bridge_payload.json \
  --scene-package ur5_2f_test \
  --once
```

Use `--dry-run` for validation-only checks (no ROS runtime send):

```bash
python3 scripts/replay_emd_bridge_payload.py \
  --payload /tmp/mvp1/emd_grasp_bridge_payload.json \
  --scene-package ur5_2f_test \
  --dry-run
```

## Use live RealSense / EPD detections

Terminal 1:
```bash
source /opt/ros/humble/setup.bash
source ~/epd_ros2_ws/install/setup.bash
ros2 launch easy_perception_deployment run.launch.py
```

Terminal 2:
```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/setup.bash
python3 scripts/capture_epd_detected_objects.py \
  --topic /easy_perception_deployment/epd_localize_output \
  --output /tmp/mvp1/live_detected_objects.yaml \
  --scene-package ur5_2f_test \
  --timeout 10 \
  --min-objects 1 \
  --once
```

Then:
```bash
python3 scripts/run_generated_cell_acceptance.py \
  --scene-package ur5_2f_test \
  --task-recipe tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml \
  --detected-objects /tmp/mvp1/live_detected_objects.yaml \
  --output-dir /tmp/mvp1 \
  --json
```

Then:
```bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_test
```

Then:
```bash
python3 scripts/replay_emd_bridge_payload.py \
  --payload /tmp/mvp1/emd_grasp_bridge_payload.json \
  --scene-package ur5_2f_test \
  --once
```


## Run one complete generated-cell cycle

### A) Offline fixture workflow
```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/setup.bash

python3 scripts/run_generated_cell_cycle.py \
  --scene-package ur5_2f_test \
  --task-recipe tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml \
  --detected-objects tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml \
  --output-dir /tmp/mvp1 \
  --dry-run \
  --json
```

### B) Live RealSense / EPD workflow
Terminal 1:
```bash
source /opt/ros/humble/setup.bash
source ~/epd_ros2_ws/install/setup.bash
ros2 launch easy_perception_deployment run.launch.py
```

Terminal 2:
```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/setup.bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_test
```

Terminal 3:
```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/setup.bash

python3 scripts/run_generated_cell_cycle.py \
  --scene-package ur5_2f_test \
  --task-recipe tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml \
  --capture-live \
  --epd-topic /easy_perception_deployment/epd_localize_output \
  --output-dir /tmp/mvp1 \
  --min-objects 1 \
  --capture-timeout 10 \
  --replay \
  --once \
  --json
```


## Run from the operator panel

Launch the optional MVP-1 operator panel:

```bash
python3 scripts/run_cell_cycle_panel.py
```

This panel is a thin GUI layer over `scripts/run_generated_cell_cycle.py`. It does not replace Workcell Builder.

### Offline dry-run example
1. Set `Scene package` to `ur5_2f_test`.
2. Set `Task recipe` to `tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml`.
3. Select `Offline detected_objects file`.
4. Set `Detected objects` to `tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml`.
5. Keep `Dry run` enabled and `Replay to runtime` disabled.
6. Click `Run Cycle`.

### Live EPD example
Before clicking `Run Cycle`, start:
1. EPD / RealSense node (`easy_perception_deployment`).
2. `run_grasp_execution` if replay is selected.

Then select `Live EPD capture`, verify topic `/easy_perception_deployment/epd_localize_output`, and run.

### Replay warning
If `Replay to runtime` is enabled, the panel prompts:

`Replay will send the generated payload to the running grasp execution node. Continue?`

Use replay only when you intentionally want to submit the generated payload to runtime.

### Output files
By default, artifacts are written to `/tmp/mvp1` (or the selected output directory), including:
- `detected_objects_used.yaml`
- `runtime_execution_plan.json`
- `emd_grasp_bridge_payload.json`
- `cycle_report.json`

### Future full GUI mapping
This panel is the first GUI layer over the generated-cell cycle runner. It does not replace Workcell Builder.

## Discover scenes and recipes from the operator panel

Use **Refresh** in `scripts/run_cell_cycle_panel.py` to discover existing scene packages, task recipes, and detected-object fixtures. The panel now provides dropdowns for each, plus **Browse...** fallbacks for manual path selection.

- Scene package dropdown: discovered packages from install/src/generated locations.
- Task recipe dropdown: discovered `task_recipe/v1` YAML/JSON fixtures.
- Detected objects dropdown: discovered `detected_objects/v1` YAML/JSON fixtures.
- Selection details: shows source path, schema/version, object count, and warnings.
- Live EPD mode does not require a detected_objects fixture.
- Replay still requires `run_grasp_execution` to already be running.


## First live D435i / EPD smoke test

A. Build:

```bash
colcon build --symlink-install --packages-skip tesseract_rviz tesseract_planning_server tesseract_ros_examples
```

B. Source:

```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/setup.bash
```

C. Launch RealSense D435i:

```bash
ros2 launch realsense2_camera rs_launch.py \
  device_type:=d435i \
  rgb_camera.color_profile:=424x240x15 \
  depth_module.depth_profile:=424x240x15 \
  pointcloud.enable:=true \
  align_depth.enable:=true
```

D. Launch EPD in another terminal using the existing project command.

E. Confirm topics:

```bash
ros2 topic list | grep camera
ros2 topic list | grep easy_perception
```

F. Run capture-only smoke test:

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
python3 scripts/capture_epd_detected_objects.py \
  --topic /easy_perception_deployment/epd_localize_output \
  --output /tmp/mvp1_live_smoke_test/detected_objects_live.yaml \
  --scene-package ur5_2f_test \
  --timeout 10 \
  --min-objects 1 \
  --once \
  --json
```

G. Run full live dry-run generated-cell cycle:

```bash
python3 scripts/run_generated_cell_cycle.py \
  --scene-package ur5_2f_test \
  --task-recipe tests/fixtures/task_recipes/valid_garbage_sorting.yaml \
  --capture-live \
  --epd-topic /easy_perception_deployment/epd_localize_output \
  --output-dir /tmp/mvp1_live_smoke_test \
  --min-objects 1 \
  --capture-timeout 10 \
  --once \
  --dry-run \
  --no-replay \
  --json
```

Expected outcome:
- PASS or WARN is acceptable.
- FAIL means a real blocker.
- blockers=[] is required before any future replay/motion test.
- detected_objects_used.yaml is saved.
- runtime_execution_plan.json is saved.
- emd_grasp_bridge_payload.json is saved.
- cycle_report.json is saved.
- replay_status remains SKIPPED unless explicitly enabled.
- dry-run remains true.

## One-click gated dry-run
1. Select scene
2. Select task recipe
3. Select detected objects or live EPD
4. Press Run Gated Dry-Run
5. Read PASS/WARN/FAIL
6. Do not proceed to replay/motion unless all blockers are clear

This path is dry-run only and never executes robot motion.
