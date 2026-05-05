# ur5_2f_sorting_test

Minimal sorting-cell scene for UR5 + Robotiq 2F.

## Launch

```bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_sorting_test launch_rviz:=true
```

The scene includes a table, two destination bins (`bin_a`, `bin_b`), and three placeholder items.

## Physical layout

- The `world` frame is the floor (`z = 0.0`), and `table_` is a self-contained workbench model fixed to the floor with deterministic dimensions and top height.
- The workbench uses an explicit base/body and top slab, so the table is visibly supported and its top surface is exactly `table_top_z`.
- The UR5 is table-mounted on a visible mounting plate; the robot base origin is set to `table_top_z + plate_thickness` to avoid any floating appearance.
- `item_red`, `item_blue`, and `item_green` are initialized in the pickup area with center heights derived from `table_top_z` plus half-height/radius so they sit on the tabletop.
- `bin_a`, `bin_b`, and `reject_bin` form a sorting row of shallow trays; each bin frame is at the tray opening and tray geometry is offset downward so tray bottoms rest on the tabletop.
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
