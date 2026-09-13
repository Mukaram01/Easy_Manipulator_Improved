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

## Golden demo perception-ready (offline)

The golden Workcell Studio readiness demo now emits `generated/perception_profile.yaml` and `generated/perception_readiness_report.json` for offline perception-readiness checks. The profile captures expected RealSense D435i topics, expected EPD output topics, expected frames, and safety-mode defaults (`perception_only`, no motion, no runtime execution, fake hardware default).

Offline replay is validated with `tests/fixtures/perception/detected_objects_snapshot_golden.yaml` so CI can verify perception mapping without launching live RealSense or EPD nodes. This is a dry-run readiness signal only and is **not** live hardware certification.

Live validation later will require explicit runtime bring-up and guarded commissioning checks; this change does not command robot motion.


## Camera placement and frustum preview
1. Add camera.
2. Set XYZ/RPY.
3. Save Cameras to Scene YAML.
4. Open Camera Frustum Preview.
5. Generate YAML.
6. Confirm cell_definition.yaml camera block.
7. Later use metadata for EPD/RealSense integration.

This is visual/configuration only: it does not start RealSense hardware, does not start EPD, and does not enable robot motion.

## Tracking capture and mouse commissioning status (2026-09-13)

The canonical capture adapter supports both message contracts. For tracking:

```bash
python3 scripts/capture_epd_detected_objects.py \
  --message-type tracking --target-class mouse \
  --topic /easy_perception_deployment/epd_tracking_output \
  --scene-package ur5_2f_test --once --timeout 45 \
  --target-frame world --require-transform --json \
  --output /tmp/ur5_2f_live_mouse_acceptance/live_mouse.json
```

This waits for the requested class without deleting other observed obstacles.
There is no replay fallback. Tracking IDs must be unique and complete. Object
acquisition timestamps and source frames are preserved; TF2 uses the observation
stamp and spins the listener while waiting for the transform. Missing source
frames are rejected for required transformation. Invalid schema output fails.

`config/runtime/live_mouse_task.yaml` uses the existing task contract, selects
`mouse` in `pick_zone_main`, and routes to `default_drop_zone`. It explicitly
allows unavailable confidence because EPD's object message has no confidence
field. Confidence remains absent/null, never fabricated; tasks default to
rejecting unavailable confidence. The 30-second acquisition age remains enforced.
This profile is not a completed live execution entry point.

Workstation evidence at `/tmp/ur5_2f_live_mouse_acceptance` is **BLOCKED** for the
full live cycle: real D435i frames and EPD TRACKING_MODE were observed; a live
tracking message transformed successfully to world, but its class was
`fire hydrant` (track `2`), not `mouse`. A separate 45-second mouse-only capture
failed closed. The camera image visibly contains a mouse on a dark textured
surface. No target coordinates, labels, zones, or camera calibration were changed
to make this observation pass. Physical camera calibration remains unverified.

Live 2F candidate consumption by the full-cycle executor, XYZ-only cloud support,
complete fake execution/RViz observation, and planning-scene attachment/placement
acceptance remain unproven in this follow-up. The executor still generates box
grasps internally; do not represent those as consumed EMD ranked cloud grasps.
No physical commissioning is claimed.

## Explicit reachable-object commissioning

`selection_policy: reachable_object` is an explicit fake-hardware commissioning
selection policy. It accepts any semantic class and does not require a source
zone or tabletop-height match. Normal recipes default to `task_semantics` and
retain their class/zone filters. Both policies retain timestamp, geometry,
confidence policy, and full-cycle feasibility checks. All captured objects remain
collision obstacles; only configured target/fingertip contacts are allowed.

From `~/workcell_ws/src/easy_manipulation_deployment`, with Humble, the consistent
MoveIt underlay, EPD install, and Workcell install sourced:

```bash
python3 scripts/run_live_object_acceptance.py \
  --output-dir /tmp/ur5_2f_live_object_acceptance/new-run \
  --domain-id 180 --capture-timeout 180
```

Choose an unused domain and a new evidence directory. This owns and stops the
scene, RViz, real D435i, EPD, capture, and canonical executor. It fixes fake
hardware on, checks controller-manager classes and URDF hardware plugins are
exclusively `mock_components/GenericSystem`, verifies the execution parameter,
and captures live tracking at its timestamp through TF2. Add `--plan-only` to
keep execution disabled. It never uses replay fallback. On CycloneDDS it expands
the owned session's participant-index search range so the complete graph fits.
RViz visual evidence requires X11 `xwininfo`, `xwd`, and Python Pillow.

The runtime tries the existing eight geometry-derived Robotiq top-grasp candidates
in the canonical preferred order until an entire cycle prevalidates. These are
explicitly reported as `canonical_box_geometry_robotiq_2f`; they are not a claim
of consuming EMD's segmented-cloud candidate stream. The executor saves full
planning-scene snapshots before manipulation, while attached, after placement,
and at home. Acceptance requires those state checks and all motion stages, not
only successful action codes.

2026-09-13 evidence: `/tmp/ur5_2f_live_object_acceptance/live` started the full
fake/RViz/camera/EPD stack and stopped it cleanly, but a 180-second class-agnostic
capture found no usable object. Diagnostic sampling received empty tracking and
pose messages, with `geometry_valid_total=0` and `geometry_invalid_total=2`.
The individual geometry-rejection flags were not exposed by the tracking
telemetry inspected here. No label, pose, dimensions, or geometry-quality gate
was altered to manufacture an object. Live motion/attachment/placement/home
remain **BLOCKED**, while the commissioning entry point is implemented. R1.4
remains a clean 9/9 plan-only PASS. Physical commissioning is not claimed.

### Live capture boundary correction

Later live evidence supersedes the geometry blocker above: both EPD modes emit
valid geometry. The capture failure was reproduced with a matching topic/type,
one best-effort publisher and compatible subscriber, but zero capture callbacks.
ROS CLI received a sample while UDP receive-buffer drops were present. Reliable
publisher/subscriber delivery restored direct tracking capture even at the same
large camera profile; direct localization also passed through world TF and
`detected_objects/v1`. This is transport acceptance, not geometry relaxation.

The EPD object publishers retain their SensorDataQoS defaults and now permit ROS
QoS overrides. The commissioning runner explicitly requests reliable delivery
for tracking/localization, passes the selected mode (4/3), and uses the proven
CPU backend and 640x480x15 aligned/synchronized camera profile. It checks topic
type, publisher reliability and valid completed inference before starting capture.
No fixed startup sleep or replay fallback is used.

Direct capture can write `--diagnostics-output /tmp/capture-audit.json`; this
records mode/topic/type, endpoint counts/QoS, domain/RMW, received/lost messages,
raw objects, target filtering, TF, validation and first rejection reasons.
`--target-class` omitted, empty, or `None` means no semantic filter. A tracking
capture retains EPD IDs; both modes retain observation timestamp, source frame,
raw pose, world pose and measured dimensions without inventing confidence.

Evidence: `/tmp/ur5_2f_live_object_acceptance/capture_boundary/`, including the
failed best-effort baseline and successful reliable direct captures in both modes.
