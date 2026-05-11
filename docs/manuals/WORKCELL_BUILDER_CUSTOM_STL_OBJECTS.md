# Workcell Builder Custom STL / Primitive Objects

Use the Qt `workcell_builder` **Add Object** flow, then click **Create Custom STL / Create Primitive Object**.

Supported primitive presets:
- box/block
- table
- bin/tray
- conveyor_placeholder
- fixture_plate

Generated meshes are written into the generated scene package under:
- `meshes/generated_objects/<safe_object_name>.stl`

These entries are reflected in generated outputs (`environment.yaml`, preview, summary, readiness) with markers such as:
- `custom_stl: <name>`
- `generated mesh: meshes/generated_objects/<safe_object_name>.stl`

## Fake-hardware-first safety
This workflow is generation-time only and offline.
It does **not** execute robot motion, does **not** call MoveIt planning APIs, and does **not** enable real hardware.
Launch generated scenes with `use_fake_hardware:=true` during preview/testing.

## Operator flow
1. Open `workcell_builder`
2. Select or create scene
3. Add robot
4. Add end effector
5. Add Object -> Create Custom STL
6. Generate Files
7. Build generated scene
8. Launch with `use_fake_hardware:=true`
