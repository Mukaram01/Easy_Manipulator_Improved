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

## Scene3D visual quality screenshot smoke matrix

Use `scripts/run_scene3d_visual_quality_screenshots.py` when a change affects Scene3D rendering, mesh/primitive preview, screenshot capture, or the GUI smoke handoff. The wrapper runs the existing Scene3D GUI smoke runner per target and writes a matrix summary containing:

- per-scene smoke JSON path
- per-scene screenshot path
- visual-quality counter summary
- primitive render summary
- mesh failure summary grouped by reason code

Example default target set for visual review:

```bash
python3 scripts/run_scene3d_visual_quality_screenshots.py \
  --scene ur10_2f_test \
  --scene ur5_2f_sorting_test \
  --scene suction_test \
  --synthetic-fixture tests/fixtures/scene3d_visual_quality_minimal_fixture \
  --output-dir build/scene3d_visual_quality_screenshots \
  --xvfb
```

To run the same capture for every enabled entry in the supported scene catalog, use:

```bash
python3 scripts/run_scene3d_visual_quality_screenshots.py \
  --supported-scenes scenes/supported_scenes.yaml \
  --synthetic-fixture tests/fixtures/scene3d_visual_quality_minimal_fixture \
  --output-dir build/scene3d_visual_quality_screenshots \
  --xvfb
```

Scene names are only CLI inputs and report labels. Rendering remains data-driven from each scene package or generated fixture; do not add scene-specific render branches to make one target pass.
