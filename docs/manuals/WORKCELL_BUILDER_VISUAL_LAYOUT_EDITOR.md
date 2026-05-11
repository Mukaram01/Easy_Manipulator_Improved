# Workcell Builder Visual Layout Editor

This editor is a practical 2D/2.5D top-down layout editor, **not** CAD and not a full 3D renderer.

- YAML/environment config remains the source of truth.
- Objects are loaded from placed_objects into ObjectPlacementModel/PlacedObject.
- X/Y is adjusted visually on a top-down grid.
- Z/roll/pitch/yaw, name/source/mesh are edited in fields.
- Save writes back to the same generated environment config/YAML model.
- Complex STL geometry should be built in external CAD tools, then imported.
- Fake-hardware-first: use offline generation and `use_fake_hardware:=true` for launch validation.

## Operator flow
1. Open workcell_builder.
2. Select/create scene.
3. Select robot and end effector.
4. Add/import objects via Object Placement Manager.
5. Open Visual Layout Editor.
6. Drag objects on top-down grid.
7. Edit Z/RPY if needed.
8. Save Layout to Environment YAML.
9. Generate Files.
10. Build generated scene.
11. Launch with `use_fake_hardware:=true`.
