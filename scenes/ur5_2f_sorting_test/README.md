# ur5_2f_sorting_test

Minimal sorting-cell scene for UR5 + Robotiq 2F.

## Launch

```bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_sorting_test launch_rviz:=true
```

The scene includes a workbench table, a destination tray row (`bin_a`, `bin_b`, `reject_bin`), and three pickup items.

## Physical layout

- The `world` frame is the floor (`z = 0.0`), and `table_` is the tabletop top-surface center frame fixed to `z = table_top_z`.
- The workbench uses an explicit body and top slab geometry offset downward from `table_`, so objects on `table_` local `z = 0` are physically on the tabletop.
- The UR5 is table-mounted on a visible mounting plate; the robot base origin is set to `table_top_z + plate_thickness` to avoid any floating appearance.
- `item_red`, `item_blue`, and `item_green` are fixed children of `table_`; each item local `z` is set from half-height/radius so bottoms are exactly on tabletop local `z = 0`.
- `bin_a`, `bin_b`, and `reject_bin` are fixed children of `table_`; each tray frame is at opening/top-center (`local z = tray_height`) and tray geometry extends downward so tray bottoms rest at tabletop local `z = 0`.
- The RealSense camera is mounted above the pickup/sorting area (with a visible mast support) and oriented to look down toward the work area.
- This package remains dry-run only for sorting-plan generation and does not execute robot motion yet.

## Sorting manifest

This scene now includes a scene-local sorting manifest at `sorting_manifest.yaml`.

Object-to-destination routing:

- `item_red` → `bin_a`
- `item_blue` → `bin_b`
- `item_green` → `reject_bin` (default/reject route)

The manifest also includes:

- object metadata (`label`, `class_name`, `frame_id`, `approximate_size_m`, `pick_hint`)
- destination metadata (`name`, `frame_id`, `release_offset_xyz_m`)

## Dry-run sorting task generator

A scene-local helper script generates a dry-run task list from the manifest:

```bash
ros2 run ur5_2f_sorting_test generate_sorting_plan
```

This helper only prints a dry-run plan and metadata for each object route.
It does **not** execute robot motion, perception, grasp planning, or hardware control.

Example output:

```text
Dry-run sorting task plan
Manifest: /.../sorting_manifest.yaml

1. item_red -> bin_a
   source_frame: item_red
   destination_frame: bin_a
   approximate_size_m: [0.050, 0.050, 0.050]
   pick_hint: top_grasp
   release_offset_xyz_m: [0.000, 0.000, 0.050]
2. item_blue -> bin_b
   source_frame: item_blue
   destination_frame: bin_b
   approximate_size_m: [0.050, 0.050, 0.050]
   pick_hint: top_grasp
   release_offset_xyz_m: [0.000, 0.000, 0.050]
3. item_green -> reject_bin
   source_frame: item_green
   destination_frame: reject_bin
   approximate_size_m: [0.050, 0.050, 0.050]
   pick_hint: top_grasp
   release_offset_xyz_m: [0.000, 0.000, 0.060]
```


## Dry-run runtime execution plan preview

Generate a scene-local `runtime_execution_plan/v1` preview from `sorting_manifest.yaml`:

```bash
ros2 run ur5_2f_sorting_test generate_sorting_runtime_plan
```

Print full JSON to stdout:

```bash
ros2 run ur5_2f_sorting_test generate_sorting_runtime_plan --json
```

Write JSON to a file:

```bash
ros2 run ur5_2f_sorting_test generate_sorting_runtime_plan --output /tmp/runtime_plan.json
```

This tool is **dry-run only**. It does not move the robot, does not call `run_grasp_execution`, does not publish ROS grasp tasks, and does not integrate perception.

## Dry-run EMD bridge payload preview


### Offline EPD-style detected_objects fixture path

Generate `runtime_execution_plan/v1` from an offline `detected_objects/v1` fixture:

```bash
ros2 run ur5_2f_sorting_test generate_runtime_plan_from_detections \
  --detections <path-to-detected_objects_v1.json>
```

JSON output:

```bash
ros2 run ur5_2f_sorting_test generate_runtime_plan_from_detections \
  --detections <path-to-detected_objects_v1.json> \
  --json
```

Architecture split:
- EPD detects/classifies objects and provides pose/frame metadata.
- EMD owns label-to-bin routing using scene configuration (`sorting_manifest.yaml`).
- Robo Studio is expected to eventually edit/generate these routing rules and scene metadata.

This path stays dry-run/offline and does not require ROS topic/service publishing or live EPD/camera inputs.

Convert a `runtime_execution_plan/v1` into a scene-local `emd_grasp_bridge_payload/v1` preview:

```bash
ros2 run ur5_2f_sorting_test generate_sorting_emd_bridge_payload
```

Use an existing runtime plan JSON as input:

```bash
ros2 run ur5_2f_sorting_test generate_sorting_emd_bridge_payload --runtime-plan /tmp/runtime_plan.json
```

Print full payload JSON:

```bash
ros2 run ur5_2f_sorting_test generate_sorting_emd_bridge_payload --json
```

Write payload JSON to a file:

```bash
ros2 run ur5_2f_sorting_test generate_sorting_emd_bridge_payload --output /tmp/emd_bridge_payload.json
```

This bridge payload generator is **dry-run only** and remains offline/reviewable. It does **not** execute robot motion, does **not** publish ROS grasp tasks, does **not** call `run_grasp_execution`, and does **not** use live EPD detections yet.

Future integration path (guarded, not enabled yet):

`EPD detected objects -> runtime_execution_plan/v1 -> emd_grasp_bridge_payload/v1 -> guarded EMD execution`

## Offline EPD-style detections to EMD bridge payload

Generate `emd_grasp_bridge_payload/v1` directly from offline `detected_objects/v1` input:

```bash
ros2 run ur5_2f_sorting_test generate_bridge_payload_from_detections --detections <path>
```

Print final payload JSON:

```bash
ros2 run ur5_2f_sorting_test generate_bridge_payload_from_detections --detections <path> --json
```

Optional outputs:

- `--runtime-plan-output <path>` writes intermediate `runtime_execution_plan/v1`
- `--output <path>` writes final `emd_grasp_bridge_payload/v1`

This is the offline/dry-run path. Future live EPD integration will replace only the input boundary (`detected_objects/v1` source), while EMD routing and bridge payload generation remain reviewable and deterministic.

## Validate physical layout

Run the deterministic scene-layout validator before RViz or execution integration:

```bash
ros2 run ur5_2f_sorting_test validate_scene_layout
```

The report verifies table top height, item/bin contact with the tabletop, tray frame placement, positive release offsets, and robot mount consistency.

## Validation

A lightweight test validates that the sorting manifest:

- parses successfully as YAML
- defines required object/destination frames
- references only known objects and destinations in routing
- dry-run plan output includes expected routing lines

Run package-local checks:

```bash
colcon test --packages-select ur5_2f_sorting_test --event-handlers console_direct+
colcon test-result --verbose
```

## Guarded execution handoff preview

Validate and preview what would be handed to execution from `emd_grasp_bridge_payload/v1` without requesting any robot motion.

Generate from static manifest chain:

```bash
ros2 run ur5_2f_sorting_test preview_sorting_execution_handoff --from-static-manifest
```

Generate from offline detections fixture chain:

```bash
ros2 run ur5_2f_sorting_test preview_sorting_execution_handoff \
  --detections <path-to-detected_objects_sample.json>
```

Print handoff preview JSON:

```bash
ros2 run ur5_2f_sorting_test preview_sorting_execution_handoff \
  --detections <path-to-detected_objects_sample.json> \
  --json
```

This tool is still dry-run only and does not move the robot. It validates the bridge payload and previews the execution handoff contract for review. Live EPD integration and robot execution remain separate future steps.

## Robot Studio Lite

The first lightweight Robot Studio workflow wrapper is now available as a scene-local CLI dashboard:

```bash
ros2 run ur5_2f_sorting_test robot_studio_lite
```

JSON dashboard output:

```bash
ros2 run ur5_2f_sorting_test robot_studio_lite --json
```

Use a custom offline `detected_objects/v1` file:

```bash
ros2 run ur5_2f_sorting_test robot_studio_lite --detections <path>
```

Robot Studio Lite provides one dry-run view of:

- scene health (`validate_scene_layout`)
- static manifest routing
- offline detections routing
- bridge payload summary
- guarded execution handoff preview

Safety guardrails are explicit:

- it does **not** move the robot
- it does **not** call live EPD
- it does **not** enable execution

This launcher is intended for developer/operator readiness checks before any future live integration steps.


## Static sorting execution preparation

Prepare static sorting execution artifacts and validation outputs without requesting robot motion:

```bash
ros2 run ur5_2f_sorting_test prepare_static_sorting_execution
```

JSON report output:

```bash
ros2 run ur5_2f_sorting_test prepare_static_sorting_execution --json
```

Manual execution-enabled preparation (still no robot motion) and print launch command:

```bash
ros2 run ur5_2f_sorting_test prepare_static_sorting_execution --manual-enable-execution --print-launch-command
```

This command remains safe by default and in manual-enable mode:

- it does **not** move the robot
- it does **not** call `ros2 launch` automatically
- it prepares and validates handoff artifacts for manual review
- user must manually run and verify RViz using:

```bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_sorting_test launch_rviz:=true
```

A future PR can add an explicit/manual execution trigger only after this preparation layer is stable.

## Manual static sorting executor

First explicit guarded bridge from static sorting preparation to runtime execution handoff:

```bash
ros2 run ur5_2f_sorting_test manual_static_sorting_executor
```

Manual enable preview (still no robot motion):

```bash
ros2 run ur5_2f_sorting_test manual_static_sorting_executor --manual-enable-execution
```

Execution request (must be explicitly enabled first):

```bash
ros2 run ur5_2f_sorting_test manual_static_sorting_executor --manual-enable-execution --execute
```

Safety behavior:

- Default mode is safe dry-run only.
- `--manual-enable-execution` alone still does not move the robot.
- `--execute` is blocked unless `--manual-enable-execution` is also provided.
- The tool will not guess/invent unknown execution APIs.
- If runtime interfaces are missing, it fails safely and instructs the user to launch:
  `ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_sorting_test launch_rviz:=true`

This adapter does not auto-launch `run_grasp_execution`, does not auto-start RViz, does not require live EPD, and does not request robot motion by default.

## Runtime-compatible static sorting bridge payload

Generate runtime-shaped payload (offline only):

```bash
ros2 run ur5_2f_sorting_test generate_static_sorting_runtime_bridge_payload --json
```

Write payload to file:

```bash
ros2 run ur5_2f_sorting_test generate_static_sorting_runtime_bridge_payload \
  --output /tmp/ur5_2f_sorting_runtime_bridge_payload.json
```

Validate payload structure using replay dry-run (no ROS send):

```bash
python3 scripts/replay_emd_bridge_payload.py \
  --payload /tmp/ur5_2f_sorting_runtime_bridge_payload.json \
  --scene-package ur5_2f_sorting_test \
  --dry-run
```

This does **not** move the robot. It only creates the runtime-shaped bridge payload expected by `replay_emd_bridge_payload.py`. A later PR can add an explicit manual send path to `/grasp_requests` after dry-run replay validation is stable.

## Guarded runtime send

Default behavior stays dry-run only and never sends to runtime:

```bash
ros2 run ur5_2f_sorting_test manual_static_sorting_executor
```

Runtime readiness check (still no send):

```bash
ros2 run ur5_2f_sorting_test manual_static_sorting_executor \
  --require-active-runtime \
  --json
```

Blocked preview (missing final confirmation):

```bash
ros2 run ur5_2f_sorting_test manual_static_sorting_executor \
  --require-active-runtime \
  --manual-enable-execution \
  --execute
```

Final guarded send (manual confirmation required):

```bash
ros2 run ur5_2f_sorting_test manual_static_sorting_executor \
  --require-active-runtime \
  --manual-enable-execution \
  --execute \
  --confirm-runtime-send
```

Notes:
- The final command may send to `/grasp_requests` (or publish `/grasp_tasks` if `--ros-interface topic` is used).
- Runtime must already be launched and healthy before the final command.
- Verify RViz and scene safety before using final send.
- Use simulation/mock runtime only at this stage.
- No live EPD integration is involved yet.
