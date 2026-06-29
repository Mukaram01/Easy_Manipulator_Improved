# Workcell Studio static web scene viewer

This directory contains a **Phase 3 proof-of-life** browser viewer for the Workcell Studio `workcell_studio_web_scene/v1` JSON contract. It is intentionally tiny: static HTML, CSS, and JavaScript only, with no npm package, no bundler, no generated scene outputs committed to the repository, and no replacement of the existing Workcell Builder or RViz/MoveIt validation flow.

The viewer uses an isolated browser import map that loads Three.js and OrbitControls from a CDN at runtime. That keeps this proof-of-life self-contained while avoiding a committed frontend build system.

## Export a scene JSON

From the repository root, export a browser-readable scene file into `build/`:

```bash
python3 scripts/export_workcell_studio_web_scene.py --scene scenes/ur5_2f_test --output build/workcell_studio_web_scene/ur5_2f_test.web_scene.json
```

`build/` outputs are local artifacts and should not be committed.

### Workcell Builder action

Workcell Builder also exposes a selected-scene action named **Export & Open Web 3D Viewer** in the Safety / Review flow. The action resolves the selected scene, runs `scripts/export_workcell_studio_web_scene.py`, writes only to `build/workcell_studio_web_scene/<scene_id>.web_scene.json`, and opens `workcell_studio_web/viewer/index.html` with Qt/QDesktopServices. It does not write under `scenes/`, mutate source YAML, mutate generated scene files, apply edit patches, or change RViz/MoveIt planning truth.

If direct `file://` loading is blocked or unreliable in your browser, run this from the repository root and open the URL manually:

```bash
python3 -m http.server 8765
```

URL: `http://localhost:8765/workcell_studio_web/viewer/index.html`

## Open the viewer

1. Open `workcell_studio_web/viewer/index.html` directly in a browser.
2. Use the **Load web_scene.json** file picker.
3. Select the exported JSON, for example:
   `build/workcell_studio_web_scene/ur5_2f_test.web_scene.json`.

Because Three.js is loaded by CDN import map, the browser needs network access for the viewer to render. If the CDN modules cannot load, the page shows a clear Three.js/CDN load failure rather than silently pretending the scene rendered.

## Phase 3 readability improvements

Phase 3 focuses on making exported scenes easier to inspect in a browser without claiming that the static viewer is a full editor or simulator:

- Scene auto-framing computes bounds from visible scene content and starts the camera at a useful overview angle instead of leaving the user to hunt for the cell manually.
- **Reset View** restores the camera to the computed scene overview after orbiting, panning, or zooming.
- Browser-safe mesh loading attempts are made for relative/local `.stl`, `.dae`, and `.obj` mesh references where the browser and Three.js loaders can support them from the selected/exported scene context.
- Mesh loading remains best-effort. If a mesh URI cannot be fetched, parsed, or rendered safely in the browser, the viewer keeps the scene readable with a primitive or box fallback instead of hiding the object.
- The inspector exposes `render_status` so users can distinguish loaded meshes, primitive fallbacks, and failed mesh attempts for the selected object.
- Runtime mesh warnings are surfaced in the warnings panel and inspector context so missing, blocked, or unsupported assets are visible during review.
- The object list is grouped to make robots, tools, environment items, cameras, zones, and other exported objects easier to scan.
- Simple labels help identify important scene objects without requiring users to click every item.
- Camera objects include lightweight frustum markers so sensor placement and approximate viewing direction are visible in the static scene.

## Current support

- World grid and axes helper.
- Orbit, pan, and zoom camera controls.
- Scene auto-framing with **Reset View**.
- Primitive objects from exported primitive/fallback data.
- Browser-safe relative/local `.stl`, `.dae`, and `.obj` mesh attempts where supported.
- Box fallback for unknown assets, unsupported mesh URIs, blocked mesh access, and mesh-only assets that cannot be rendered by the browser.
- Camera/sensor markers with simple frustum lines.
- Pick, place, spawn, and safety zones as simple transparent geometry when present in the JSON.
- Simple material/color differences between locked/generated preview items and editable/environment items.
- Grouped object list with IDs and labels.
- Simple in-scene labels for readable object identification.
- Object selection from the list and basic canvas picking.
- Inspector fields for ID, label, type, source, pose XYZ/RPY, scale, editable, locked, mesh URI, `render_status`, and primitive details.
- JSON warnings and runtime mesh warnings rendered in a dedicated warnings panel.

## Intentional exclusions and limitations

This is not the final Web Studio editor. It intentionally excludes:

- Editing transforms or scene data.
- Saving JSON or writing source-of-truth scene files back to disk.
- ROS launch, package generation, backend action execution, or runtime control.
- RViz/MoveIt replacement; RViz/MoveIt remain the backend validation and simulation path for Workcell Studio.
- Qt Scene3D fixes; this static browser viewer does not repair or replace the Workcell Builder Qt Scene3D canvas.
- MoveIt planning or execution.
- EPD live input or perception runtime streaming.
- Mesh-perfect rendering.
- `package://` URI resolution in the browser.
- Arbitrary local filesystem mesh access from the browser. Browser security rules only allow safe access to files made available through the selected file context, a local server, or other browser-permitted URLs.
- Authentication, deployment packaging, and frontend framework scaffolding.

This static viewer is only a lightweight browser proof-of-life for reviewing exported scene contract data. It improves readability of exported scenes, but the source-of-truth editing, scene generation, validation, and fake-hardware simulation flows remain in Workcell Studio, generated packages, and RViz/MoveIt.

## Persistence verification loop

The viewer edits are persisted only through the backend applicator; the viewer never writes source YAML, generated files, `cell_definition.yaml`, or `scene_manifest.yaml` directly. For manual verification, use a copied/temp scene rather than a checked-in real scene:

1. Export before `web_scene.json`:

   ```bash
   python3 scripts/export_workcell_studio_web_scene.py \
     --scene scenes/ur5_2f_test \
     --output build/workcell_studio_web_scene/ur5_2f_test.before.web_scene.json
   ```

2. Open the viewer and edit one `editable=true`, `locked=false` layout/environment item.
3. Export `edit_patch.json`.
4. Validate the patch.
5. Run the applicator dry-run and confirm it writes nothing.
6. Run the applicator with `--write` on the temp scene only.
7. Re-export after `web_scene.json`.
8. Run the persistence verifier:

   ```bash
   python3 scripts/verify_workcell_studio_web_scene_edit_persistence.py \
     --scene scenes/ur5_2f_test \
     --web-scene-before build/workcell_studio_web_scene/ur5_2f_test.before.web_scene.json \
     --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json \
     --web-scene-after build/workcell_studio_web_scene/ur5_2f_test.after.web_scene.json
   ```

9. Only then regenerate and validate the scene package through Workcell Studio backend tooling. RViz/MoveIt remains the fake-hardware planning/simulation truth; this static viewer does not replace it and does not enable real-hardware execution.

## Preview-only transform editing and edit patches

The static viewer includes a safe editing surface for Web 3D. When a selected item is both `editable=true` and `locked=false`, it enters **edit mode**: the inspector shows XYZ, RPY, and scale inputs, and the Three.js translation gizmo attaches to the selected object. Dragging the gizmo updates the numeric inspector fields; editing numeric fields updates the object and attached gizmo. These changes update only the in-browser Three.js preview.

Locked or generated preview items, including generated robot/tool visuals, never enter edit mode. Their transform fields stay disabled with the reason: “Locked/generated preview item; edit source layout/environment instead.”

The toolbar includes explicit snap controls. **Snap** enables/disables snapping, **Move snap (m)** defaults to `0.01`, and **Rot snap (deg)** defaults to `5`. Translation snap is applied to gizmo movement and numeric preview edits; rotation snap is available for numeric RPY edits and for future gizmo rotation modes. No implicit ground/table snap is performed.

Preview edit history is browser-local. **Undo** reverts the last preview transform change, **Redo** reapplies an undone preview transform change, **Clear Preview Edits** restores every edited object to its loaded transform, and **Reset Selected** restores only the selected editable item. The dirty banner shows “Unsaved preview edits” plus the changed item count.

Use **Export Edit Patch** to download a `workcell_studio_web_scene_edit_patch/v1` JSON file. The patch contains only the final changed preview state and does not modify `environment.yaml`, layout YAML, `cell_definition.yaml`, `scene_manifest.yaml`, or generated scene files. The browser never writes source YAML directly and never bypasses the validator/applicator loop.

Validate exported patches before application:

```bash
python3 scripts/validate_workcell_studio_web_scene_edit_patch.py \
  --web-scene build/workcell_studio_web_scene/ur5_2f_test.web_scene.json \
  --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json
```

Dry-run backend application. This is the default and writes nothing:

```bash
python3 scripts/apply_workcell_studio_web_scene_edit_patch.py \
  --scene scenes/ur5_2f_test \
  --web-scene build/workcell_studio_web_scene/ur5_2f_test.web_scene.json \
  --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json
```

Apply only when you intentionally want to persist an editable layout/environment transform:

```bash
python3 scripts/apply_workcell_studio_web_scene_edit_patch.py \
  --scene scenes/ur5_2f_test \
  --web-scene build/workcell_studio_web_scene/ur5_2f_test.web_scene.json \
  --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json \
  --write
```

Re-export `web_scene.json` after `--write` to confirm the edited pose persists. The applicator does not launch ROS, does not mutate `generated/`, `cell_definition.yaml`, or `scene_manifest.yaml`, and does not replace RViz/MoveIt as the planning and fake-hardware simulation truth.

## Guided backend edit workflow

The static viewer exports preview-only `edit_patch.json` files. It does **not** write scene YAML directly. After editing an unlocked editable object in the browser, use the repository backend workflow to validate, dry-run, apply, re-export, and verify persistence.

Safe dry-run command:

```bash
python3 scripts/run_workcell_studio_web_edit_workflow.py \
  --scene scenes/ur5_2f_test \
  --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json
```

Apply only after reviewing the dry-run output:

```bash
python3 scripts/run_workcell_studio_web_edit_workflow.py \
  --scene scenes/ur5_2f_test \
  --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json \
  --write
```

The guided workflow exports a before `web_scene`, validates the patch, runs the safe applicator in dry-run mode, and only mutates editable source YAML when `--write` is explicit. In write mode it re-exports an after `web_scene` and verifies persistence. Pass `--run-readiness` only when you also want the optional readiness matrix.

This viewer workflow is still upstream of the normal Workcell Studio generate/validate/simulate path. RViz/MoveIt remains the planning truth, and fake-hardware-first safety defaults are unchanged.
