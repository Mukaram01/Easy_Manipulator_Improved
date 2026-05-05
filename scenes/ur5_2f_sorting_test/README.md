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
