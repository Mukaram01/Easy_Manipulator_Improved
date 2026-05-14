# Workcell Studio Interactive Canvas

- Select movable items: table, conveyor, camera, pick_zone, place_zone, bin, object, fixture.
- Robot base, robot reach envelope, and safety/home marker are locked by default.
- Use **Unlock Robot Base** only when necessary and after acknowledging warning.
- Use **Snap to Grid** and **Fine Move Mode** for drag precision.
- Use **Undo**, **Redo**, **Duplicate Selected**, **Delete Selected** for layout editing.
- Use **Save Layout** to persist and **Revert Layout** to discard unsaved changes.
- **Unsaved Layout Edits** badge indicates pending layout modifications.
- Validation warnings include outside robot reach, camera coverage warning, overlap warning, missing pick zone, missing place zone.
- Safety note: layout editing does not command robot motion.

## Production drag/drop layout editor
- Drag supported for table/conveyor/camera/pick_zone/place_zone/bin/object/fixture.
- Locked by default: robot/robot_base/reach/safety/home. Unlock Robot Base requires explicit toggle.
- Snap to Grid and Fine Move Mode are supported for precise moves.
- Inspector pose editing supports xyz/rpy and keeps selection active.
- Undo/Redo, Duplicate Selected, Delete Selected are integrated with layout dirty state.
- Save Layout writes `layout/workcell_studio_layout.yaml` (`schema_version: workcell_studio_layout/v1`).
- Revert Layout reloads saved layout (fallback: scene manifest/environment defaults).
- Validation warnings include reach, camera coverage, overlap, and missing zones.
- Safety: layout editing is metadata-only and never commands robot motion.


## Live Canvas Interactions
- Drag/drop now uses real `QGraphicsItem` movement for movable assets with snap-to-grid behavior.
- Locked assets (robot/reach/safety) remain guarded, and robot base unlock prompts warning first.
- Inspector pose fields (x/y/z/roll/pitch/yaw) apply live to selected item and mark layout dirty.
- Undo/redo, duplicate/delete, and Save/Revert Layout now operate on live canvas state.
- Safety behavior is unchanged: fake-hardware-first and no robot motion command execution.


## Asset-to-canvas loop
- Open Asset Catalog in Scene Builder and add assets directly to the digital-twin canvas.
- Use task binding shortcuts (Use Selected as Pick Source/Place Target/Pick Zone/Place Zone/Camera).
- Save Layout explicitly, then rerun validation/acceptance when `Layout changed since last acceptance` appears.
- Safety contract remains: fake_hardware_first=true, runtime_execution_enabled=false, motion_command_sent=false.

## Imported and generated assets
- Imported STL/URDF assets persist `source_path`, import flags, and mesh/URDF path fields in layout YAML.
- Generated simple box/cylinder placeholders persist dimensions and `generated_placeholder=true`.
- Missing mesh/URDF warnings and stale readiness indicators are restored on reload/revert.
