# Workcell Studio Digital Twin Canvas

The digital twin canvas now supports interactive layout editing while preserving runtime behavior.

## Controls
- Snap to Grid
- Fine Move Mode
- Undo / Redo
- Duplicate Selected / Delete Selected
- Save Layout / Revert Layout
- Unlock Robot Base

## Safety
Layout editing remains preview-only. No runtime robot motion is commanded from canvas edits.

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


## Interactive Digital-Twin Canvas
- The Scene Builder canvas now supports direct drag interactions, inspector pose editing, and live metadata updates per item.
- Save Layout writes current canvas state to `layout/workcell_studio_layout.yaml`; Revert Layout reloads from saved/default scene state.
- Validation warnings are surfaced during editing without enabling runtime execution or motion commands.
