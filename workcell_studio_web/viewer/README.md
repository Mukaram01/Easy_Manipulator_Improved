# Workcell Studio static web scene viewer

This directory contains a **Phase 3 proof-of-life** browser viewer for the Workcell Studio `workcell_studio_web_scene/v1` JSON contract. It is intentionally tiny: static HTML, CSS, and JavaScript only, with no npm package, no bundler, no generated scene outputs committed to the repository, and no replacement of the existing Workcell Builder or RViz/MoveIt validation flow.

The viewer uses an isolated browser import map that loads Three.js and OrbitControls from a CDN at runtime. That keeps this proof-of-life self-contained while avoiding a committed frontend build system.

## Export a scene JSON

From the repository root, export a browser-readable scene file into `build/`:

```bash
python3 scripts/export_workcell_studio_web_scene.py --scene scenes/ur5_2f_test --output build/workcell_studio_web_scene/ur5_2f_test.web_scene.json
```

`build/` outputs are local artifacts and should not be committed.

## Open the viewer

1. Open `workcell_studio_web/viewer/index.html` directly in a browser.
2. Use the **Load web_scene.json** file picker.
3. Select the exported JSON, for example:
   `build/workcell_studio_web_scene/ur5_2f_test.web_scene.json`.

Because Three.js is loaded by CDN import map, the browser needs network access for the viewer to render. If the CDN modules cannot load, the page shows a clear Three.js/CDN load failure rather than silently pretending the scene rendered.

## Current support

- World grid and axes helper.
- Orbit, pan, and zoom camera controls.
- Primitive objects from exported primitive/fallback data.
- Box fallback for unknown assets and mesh-only assets.
- Camera/sensor markers with simple frustum lines.
- Pick, place, spawn, and safety zones as simple transparent geometry when present in the JSON.
- Simple material/color differences between locked/generated preview items and editable/environment items.
- Object list with IDs and labels.
- Object selection from the list and basic canvas picking.
- Inspector fields for ID, label, type, source, pose XYZ/RPY, scale, editable, locked, mesh URI, and primitive details.
- JSON warnings rendered in a dedicated warnings panel.

## Intentional exclusions

This is not the final Web Studio editor. It intentionally excludes:

- Editing transforms or scene data.
- Saving JSON or writing source-of-truth scene files.
- ROS launch, package generation, or backend action execution.
- MoveIt planning or execution.
- EPD live input or perception runtime streaming.
- Mesh-perfect rendering and package URI mesh resolution.
- Authentication, deployment packaging, and frontend framework scaffolding.

RViz/MoveIt remain the backend validation and simulation path for Workcell Studio. This static viewer is only a lightweight browser proof-of-life for reviewing exported scene contract data.
