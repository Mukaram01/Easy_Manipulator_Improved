# Legacy Qt Scene3D Debug 3D Preview

> **Status note:** This manual describes the legacy Qt Scene3D Debug 3D Preview. It no longer describes the primary Workcell Studio digital-twin editor direction.

The Qt Scene3D canvas is an experimental preview and diagnostic surface. It can help developers inspect scene metadata, preview generated or fallback visuals, and manually gather evidence while the Workcell Studio 3D path is being improved. It is **not** the production source of truth for cell layout, task intent, scene generation, planning, or simulation acceptance.

Future production 3D editing should move to a browser-based Workcell Studio editor that can provide a cleaner product UI, stronger scene hierarchy/inspector workflows, and a maintainable foundation for true digital-twin-style authoring. Until that production editor exists, authoritative scene state remains in the supported authoring and generated files such as `environment.yaml`, `layout/workcell_studio_layout.yaml`, `cell_definition.yaml`, `scene_manifest.yaml`, and generated ROS 2 package content.

RViz/MoveIt remains the planning and visual truth for fake-hardware simulation. Scene3D preview evidence may be useful for developer diagnosis, but it must not replace RViz/MoveIt validation for planning-scene behavior, robot state, fake-hardware launch readiness, or simulation evidence.

## Legacy preview controls

These controls belong to the legacy Qt preview and should be treated as experimental diagnostics rather than the normal production editing workflow:

- Snap to Grid
- Fine Move Mode
- Undo / Redo
- Duplicate Selected / Delete Selected
- Save Layout / Revert Layout
- Unlock Robot Base

## Safety

Layout edits made through the legacy Qt preview remain preview/metadata-only. They do not command runtime robot motion.

Safe defaults still apply:

- fake hardware remains the default validation path
- no real robot motion is enabled from canvas edits
- generated validation reports are not safety certificates
- real-hardware readiness must remain explicitly guarded outside this preview

## Legacy drag/drop layout preview

The legacy Qt preview has supported the following developer-facing layout experiments:

- Drag support for table/conveyor/camera/pick_zone/place_zone/bin/object/fixture.
- Locked-by-default robot/robot_base/reach/safety/home items, with robot-base unlock requiring an explicit toggle.
- Snap to Grid and Fine Move Mode for precise preview moves.
- Inspector pose editing for xyz/rpy while keeping selection active.
- Undo/Redo, Duplicate Selected, and Delete Selected integrated with layout dirty state.
- Save Layout writing `layout/workcell_studio_layout.yaml` (`schema_version: workcell_studio_layout/v1`).
- Revert Layout reloading saved layout with fallback to scene manifest/environment defaults.
- Validation warnings for reach, camera coverage, overlap, and missing zones.

Treat these behaviors as historical Qt-preview capabilities and diagnostics. They should not be presented as the current main digital-twin editor or as the long-term production editing surface.

## Historical Scene3D visual-quality screenshot diagnostics

The Scene3D visual-quality screenshot smoke matrix is historical/manual diagnostic evidence only. It is not a normal quality gate, production acceptance path, or replacement for RViz/MoveIt fake-hardware validation.

Use `scripts/run_scene3d_visual_quality_screenshots.py` only when a developer intentionally needs manual evidence for legacy Scene3D rendering, mesh/primitive preview behavior, screenshot capture, or GUI smoke handoff diagnosis. The wrapper runs the existing Scene3D GUI smoke runner per target and writes a matrix summary containing:

- per-scene smoke JSON path
- per-scene screenshot path
- visual-quality counter summary
- primitive render summary
- mesh failure summary grouped by reason code

Example diagnostic target set for visual review:

```bash
python3 scripts/run_scene3d_visual_quality_screenshots.py \
  --scene ur10_2f_test \
  --scene ur5_2f_sorting_test \
  --scene suction_test \
  --synthetic-fixture tests/fixtures/scene3d_visual_quality_minimal_fixture \
  --output-dir build/scene3d_visual_quality_screenshots \
  --xvfb
```

If a developer needs a broader historical diagnostic capture for every enabled entry in the supported scene catalog, use this all-scene diagnostic command. Do not treat it as strict production acceptance:

```bash
python3 scripts/run_scene3d_visual_quality_screenshots.py --supported-scenes scenes/supported_scenes.yaml --synthetic-fixture tests/fixtures/scene3d_visual_quality_minimal_fixture --output-dir build/scene3d_visual_quality_screenshots --xvfb
```

The all-scene diagnostic run writes both machine-readable and review-friendly summary artifacts:

- `build/scene3d_visual_quality_screenshots/scene3d_visual_quality_screenshots_summary.json`
- `build/scene3d_visual_quality_screenshots/scene3d_visual_quality_screenshots_summary.md`

Historical diagnostic result meanings:

- `PASS`: the legacy diagnostic captured credible physical mesh/primitive evidence and found no diagnostic blockers.
- `FAIL`: the runner completed but behavior-based visual-quality blockers were reported for manual review.
- `BLOCKED`: screenshot/smoke capture could not provide reviewable evidence, or prerequisite files were missing/unreadable.

Common diagnostic blocker reasons and how to read them:

- Missing screenshot: capture did not produce the expected image artifact, so visual evidence cannot be reviewed.
- Smoke JSON missing/unreadable: the per-scene smoke runner did not write parseable JSON, so counters and evidence paths cannot be trusted.
- Mesh source exists but mesh render count is zero: scene metadata or generated URDF points at mesh-backed geometry, but the renderer did not render any mesh instances.
- URDF primitive source exists but primitive render count is zero: generated primitive geometry exists, but the renderer did not render primitive-backed scene items.
- Placeholder/missing geometry dominates: missing-mesh placeholders or fallback markers outweigh credible physical geometry evidence.
- Overlay/helper visuals dominate: helper graphics such as axes, bounds, labels, grids, guides, or selection affordances dominate the capture instead of physical scene items.
- No physical scene items rendered: the screenshot/smoke evidence contains no credible robot, tool, fixture, table, conveyor, object, bin, zone, or other physical workcell item.

Helper overlays and raw generated fallback bounds do not count as physical render success in this diagnostic evidence. A scene only earns physical render credit from credible mesh-backed visuals or legitimate URDF primitive visuals representing actual workcell geometry.

Scene names are only CLI inputs and report labels. Rendering remains data-driven from each scene package or generated fixture; do not add scene-specific render branches to make one target pass.
