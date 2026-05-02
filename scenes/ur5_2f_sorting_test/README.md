# ur5_2f_sorting_test

Minimal sorting-cell scene for UR5 + Robotiq 2F.

## Launch

```bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=ur5_2f_sorting_test launch_rviz:=true
```

The scene includes a table, two destination bins (`bin_a`, `bin_b`), and three placeholder items.

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

This helper only prints a plan and metadata for each object route.
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
   destination_frame: bin_a
   approximate_size_m: [0.050, 0.050, 0.050]
   pick_hint: top_grasp
   release_offset_xyz_m: [0.000, 0.000, 0.100]
```

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
