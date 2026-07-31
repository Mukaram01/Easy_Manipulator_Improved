# Workcell Studio Web 3D Viewer

This directory contains the **Workcell Studio Web 3D Viewer** for the Workcell Studio `workcell_studio_web_scene/v1` JSON contract. Web 3D is now the preferred fast Workcell Studio visual path for browser-based scene review and layout inspection. It remains intentionally lightweight: static HTML, CSS, and bundled JavaScript with exact npm dependencies, a committed generated viewer bundle, and no replacement of the existing Workcell Builder or RViz/MoveIt validation flow.

The viewer uses a pinned npm/esbuild pipeline instead of a browser import map or CDN imports. The authored source stays readable in `viewer.js`, `urdf_robot_renderer.js`, and `src/viewer_entry.js`; `dist/viewer.bundle.js` is generated only by the build script.

## Export a scene JSON

From the repository root, export a browser-readable scene file into `build/`:

```bash
python3 scripts/export_workcell_studio_web_scene.py --scene scenes/ur5_2f_test --output build/workcell_studio_web_scene/ur5_2f_test.web_scene.json --stage-assets
```

Browsers cannot load ROS `package://` URIs directly and cannot read arbitrary package files from your local ROS workspace. Use `--stage-assets` when you want recognizable mesh-backed browser previews: the exporter resolves supported mesh references, copies them under `build/workcell_studio_web_scene/assets/<scene_id>/...`, rewrites exported `mesh_uri` values to browser-safe relative URLs, and preserves the original ROS/source reference in `original_mesh_uri`. `build/` outputs are local artifacts and should not be committed.

### Workcell Builder action

Workcell Builder also exposes a selected-scene action named **Export & Open Web 3D Viewer** in the Safety / Review flow. The action resolves the selected scene, runs `scripts/export_workcell_studio_web_scene.py --stage-assets`, writes only to `build/workcell_studio_web_scene/<scene_id>.web_scene.json`, starts or reuses a repository-root local HTTP server on `127.0.0.1:8765`, and opens `http://localhost:8765/workcell_studio_web/viewer/index.html?scene=build%2Fworkcell_studio_web_scene%2F<scene_id>.web_scene.json` with Qt/QDesktopServices. The completion/failure dialog reports the exported web scene JSON path, the viewer URL, whether the local static asset server started or an existing server was reused, and the exact `cd <repo> && python3 -m http.server 8765 --bind 127.0.0.1` command to run manually if serving cannot be started automatically. Opening through the repository root is important because staged mesh requests resolve as `build/workcell_studio_web_scene/assets/<scene_id>/...`. It does not write under `scenes/`, mutate source YAML, mutate generated scene files, apply edit patches, or change RViz/MoveIt planning truth.

If direct `file://` loading is blocked or unreliable in your browser, or if you staged mesh assets and want relative mesh URLs to resolve consistently, run this from the repository root and open the URL manually:

```bash
python3 -m http.server 8765 --bind 127.0.0.1
```

URL: `http://localhost:8765/workcell_studio_web/viewer/index.html?scene=build%2Fworkcell_studio_web_scene%2Fur5_2f_test.web_scene.json`

## Open the viewer

1. Serve the repository root when using staged assets:

   ```bash
   python3 -m http.server 8765 --bind 127.0.0.1
   ```

2. Open `http://localhost:8765/workcell_studio_web/viewer/index.html?scene=build%2Fworkcell_studio_web_scene%2Fur5_2f_test.web_scene.json` in a browser. Directly opening `workcell_studio_web/viewer/index.html` can still work for JSON-only review, but served HTTP is the expected path for relative staged mesh assets.
3. Use the **Load web_scene.json** file picker.
4. Select the exported JSON, for example:
   `build/workcell_studio_web_scene/ur5_2f_test.web_scene.json`.

Expected result: when source meshes exist and are supported by the exporter/viewer, the robot, table/workbench, camera, and gripper/tool should be recognizable mesh-backed visuals rather than only generic boxes.

The checked-in bundle contains the pinned Three.js and URDF loader modules, so the served viewer does not depend on CDN import maps or Node.js at runtime. A normal Workcell Builder build automatically checks and, when necessary, refreshes this bundle:

```bash
colcon build --symlink-install --packages-select workcell_builder
```

An unchanged subsequent build uses the CMake freshness stamp and does not rerun esbuild or `npm ci`. For direct frontend development, the explicit build remains available:

```bash
cd workcell_studio_web/viewer
npm ci && npm run build:web3d
```

CI still rejects a committed stale bundle. Check the same condition locally with:

```bash
cd workcell_studio_web/viewer
npm ci && npm run check:stale-bundle
```


## Web 3D readability improvements

Web 3D focuses on making exported scenes easier to inspect in a browser without claiming that the static viewer is a full editor or simulator:

- Scene auto-framing computes bounds from visible scene content and starts the camera at a useful overview angle instead of leaving the user to hunt for the cell manually.
- **Reset View** restores the camera to the computed scene overview after orbiting, panning, or zooming.
- Browser-safe mesh loading attempts are made for relative staged `.stl`, `.dae`, and `.obj` mesh references where the browser and Three.js loaders can support them from the selected/exported scene context. ROS `package://` URIs must be resolved by the exporter first; browsers cannot fetch them directly.
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
- Browser-safe relative staged `.stl`, `.dae`, and `.obj` mesh attempts where supported.
- Box fallback for unknown assets, unsupported mesh URIs, blocked mesh access, and mesh-only assets that cannot be rendered by the browser.
- Camera/sensor markers with simple frustum lines.
- Pick, place, spawn, and safety zones as simple transparent geometry when present in the JSON.
- Simple material/color differences between locked/generated preview items and editable/environment items.
- Grouped object list with IDs and labels.
- Simple in-scene labels for readable object identification.
- Object selection from the list and basic canvas picking.
- Inspector fields for ID, label, type, source, pose XYZ/RPY, scale, editable, locked, mesh URI, original mesh URI where exported, `render_status`, and primitive details.
- JSON warnings and runtime mesh warnings rendered in a dedicated warnings panel.

## Manual `ur5_2f_test` visual checks

Use the Web 3D Viewer as the preferred fast Workcell Studio visual path for quick scene review. For the canonical `ur5_2f_test` scene, manually confirm:

- UR5 is assembled and not exploded into disconnected fallback pieces.
- Robotiq 2F gripper is attached to the robot wrist.
- Table/workbench is upright and positioned as a physical work surface.
- Camera/RealSense is visible; if the exact mesh is unavailable, it is clearly marked as a camera/sensor fallback.
- Grid and axes are visible for spatial orientation.
- No required item is fallback-only; any fallback required for review has a clear warning or diagnostic reason.
- Default camera view opens from a useful angle that frames the workcell.
- **Fit/Reset** recenters the scene correctly after orbiting, panning, or zooming.

RViz/MoveIt remains the planning and fake-hardware validation truth. Passing Web 3D visual checks does not prove MoveIt planning, controller wiring, fake-hardware launch readiness, or real-hardware safety.

## Mesh asset staging and primitive fallback troubleshooting

For the canonical manual export, run this from the repository root:

```bash
python3 scripts/export_workcell_studio_web_scene.py --scene scenes/ur5_2f_test --output build/workcell_studio_web_scene/ur5_2f_test.web_scene.json --stage-assets
```

With `--stage-assets`, supported meshes are resolved and copied under `build/workcell_studio_web_scene/assets/ur5_2f_test/...`. The exported `web_scene.json` uses browser-safe relative `mesh_uri` values that point at those staged files, while `original_mesh_uri` preserves the ROS `package://`, source-relative, or other original URI for diagnostics and backend traceability.

If an item still appears as `primitive_fallback`, inspect the selected item in the JSON or viewer inspector and warnings panel:

- `mesh_staging_status`: whether the exporter staged the mesh, skipped it, or failed to stage it.
- `mesh_resolve_warning`: why the source URI could not be resolved or copied, such as an unknown package, missing file, or unsupported URI form.
- `mesh_status`: whether the item has a mesh candidate, staged mesh, unsupported mesh, or fallback-only visual.
- `fallback_reason`: the viewer/exporter reason a primitive was used, such as unsupported mesh type, failed browser load, missing mesh metadata, or no source mesh.

Remaining primitive fallbacks are expected for assets without source meshes or unsupported mesh formats. They should not be treated as proof that scene generation, RViz/MoveIt, or fake-hardware validation is complete.

## Generate and validate after Web 3D edits

After applying a browser-exported edit patch, generation and validation are still a separate Workcell Builder action. Generation/validation is not automatic after patch apply. The browser never writes YAML directly: it only exports `edit_patch.json`, and that patch is validated and dry-run before apply. Apply mode re-exports the after `web_scene.json` and verifies persistence before you move on.

Recommended workflow:

1. Open Web 3D Viewer from Workcell Builder.
2. Edit the scene in the browser.
3. Export `edit_patch.json`.
4. Return to Workcell Builder.
5. Validate, dry-run, and then explicitly apply the patch only if the dry-run output is clean.
6. Confirm that apply re-exports and verifies persistence.
7. Click **Generate & Validate Scene**.
8. Read the PASS/FAIL output.
9. Do not launch robot hardware from this step.
10. Use fake-hardware RViz/MoveIt launch verification as the next phase.

This generation/validation step does not launch ROS, RViz, MoveIt, Gazebo, fake hardware, or real hardware. RViz/MoveIt remains the planning and fake-hardware validation truth after Web 3D authoring and backend scene generation.

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
- `package://` URI resolution in the browser. Browsers cannot load ROS package URIs directly; run `scripts/export_workcell_studio_web_scene.py --stage-assets` so supported meshes are copied to `build/workcell_studio_web_scene/assets/<scene_id>/...` and `mesh_uri` is rewritten to a browser-safe relative URL while `original_mesh_uri` keeps the ROS/source URI for traceability.
- Arbitrary local filesystem mesh access from the browser. Browser security rules only allow safe access to files made available through staged assets, the selected file context, a local server, or other browser-permitted URLs.
- Authentication, deployment packaging, and frontend framework scaffolding.

This static viewer is the preferred fast visual review path for exported scene contract data. It improves readability of exported scenes, but the source-of-truth editing, scene generation, validation, planning, and fake-hardware simulation flows remain in Workcell Studio, generated packages, and RViz/MoveIt.

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

## Applying Web 3D edit patches from Workcell Builder

The recommended UI workflow is:

1. In Workcell Builder, select a scene and run **Export & Open Web 3D Viewer**.
2. Load the exported `build/workcell_studio_web_scene/<scene_id>.web_scene.json` in this static viewer if it was not loaded automatically by your browser workflow.
3. Edit an editable, unlocked item. These edits remain browser preview state only.
4. Export an `edit_patch.json` from the browser. The browser never writes YAML and never mutates generated files.
5. In Workcell Builder, use **Validate Web Edit Patch…** or **Dry Run Web Edit Patch…** and select the exported JSON patch. The file picker defaults to `build/workcell_studio_web_scene`.
6. Use **Apply Web Edit Patch…** only when the dry-run output is clean. Workcell Builder requires explicit confirmation before it runs the backend workflow with `--write`.

Workcell Builder orchestrates `scripts/run_workcell_studio_web_edit_workflow.py`; the Qt UI does not reimplement the patch validator/applicator. In apply mode the backend re-exports the web scene and verifies persistence. Generated-file safety remains in force, browser-side YAML writes are forbidden, RViz/MoveIt remains planning truth, and no real robot motion is started by this patch workflow.
