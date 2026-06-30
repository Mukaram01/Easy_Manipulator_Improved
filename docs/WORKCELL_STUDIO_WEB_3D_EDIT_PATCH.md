# Workcell Studio Web 3D edit patch contract

This phase connects preview editing to a guarded backend persistence step. Web Studio can move editable layout/environment objects in the browser, export a `workcell_studio_web_scene_edit_patch/v1` JSON patch, validate it, and dry-run or apply it to the correct editable source YAML.

## Workflow

Export the current scene for the static viewer:

```bash
python3 scripts/export_workcell_studio_web_scene.py \
  --scene scenes/ur5_2f_test \
  --output build/workcell_studio_web_scene/ur5_2f_test.web_scene.json \
  --stage-assets
```

Use `--stage-assets` for mesh-backed browser review. Browsers cannot load ROS `package://` URIs directly, so the exporter copies supported meshes under `build/workcell_studio_web_scene/assets/<scene_id>/...`, rewrites exported `mesh_uri` values to browser-safe relative URLs, and preserves the ROS/source URI in `original_mesh_uri`. Edit patches operate on item identity and transforms; they do not rewrite `mesh_uri`, `original_mesh_uri`, `mesh_staging_status`, `mesh_resolve_warning`, `mesh_status`, or `fallback_reason`. Those mesh fields are preserved/exported for review and troubleshooting remaining `primitive_fallback` visuals.

Open `workcell_studio_web/viewer/index.html`, load the exported `web_scene.json`, select one `editable=true` and `locked=false` item, and use edit mode to preview the transform. Editable/unlocked selections show enabled XYZ/RPY/scale fields and a Three.js translation gizmo; locked/generated robot/tool previews remain read-only and explain that source layout/environment should be edited instead. Use **Export Edit Patch** to download an edit patch when the preview state is correct.

From Workcell Builder, the **Export & Open Web 3D Viewer** selected-scene action performs only the export-and-open part of this workflow. It writes the export to `build/workcell_studio_web_scene/<scene_id>.web_scene.json` and opens the static viewer. It intentionally does not apply edit patches, write under `scenes/`, mutate source YAML, mutate generated scene files, modify Qt Scene3D visuals, or replace RViz/MoveIt as planning truth. If browser `file://` loading is unreliable, or if staged mesh asset URLs need to resolve consistently, use `python3 -m http.server 8765` from the repository root and browse to `http://localhost:8765/workcell_studio_web/viewer/index.html`.

Validate the patch before applying it:

```bash
python3 scripts/validate_workcell_studio_web_scene_edit_patch.py \
  --web-scene build/workcell_studio_web_scene/ur5_2f_test.web_scene.json \
  --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json
```

Dry-run the backend applicator. This is the default and writes nothing:

```bash
python3 scripts/apply_workcell_studio_web_scene_edit_patch.py \
  --scene scenes/ur5_2f_test \
  --web-scene build/workcell_studio_web_scene/ur5_2f_test.web_scene.json \
  --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json
```

Apply only when you intentionally want to mutate editable source YAML:

```bash
python3 scripts/apply_workcell_studio_web_scene_edit_patch.py \
  --scene scenes/ur5_2f_test \
  --web-scene build/workcell_studio_web_scene/ur5_2f_test.web_scene.json \
  --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json \
  --write
```

Use `--backup` with `--write` to create timestamped `.bak` files next to edited YAML. Backups are optional and are never created for generated outputs.

Re-export the web scene after applying a patch to confirm the persisted pose is reflected:

```bash
python3 scripts/export_workcell_studio_web_scene.py \
  --scene scenes/ur5_2f_test \
  --output build/workcell_studio_web_scene/ur5_2f_test.web_scene.json
```


## Browser edit mode ergonomics

Edit mode is available only for items that are both `editable=true` and `locked=false` and are not generated preview visuals. In edit mode:

- the inspector numeric XYZ/RPY/scale fields remain the authoritative manual transform controls;
- the Three.js translation gizmo attaches to the selected editable object;
- gizmo movement updates the inspector fields;
- inspector numeric changes update the object and attached gizmo;
- snap can be toggled from the toolbar;
- translation snap defaults to `0.01` meters;
- rotation snap defaults to `5` degrees for numeric RPY edits and future rotation-gizmo use;
- **Undo**, **Redo**, **Clear Preview Edits**, and **Reset Selected** affect browser preview state only.

The dirty state displays “Unsaved preview edits” with the changed item count. Exported `edit_patch.json` records only the final preview transform per changed item, preserving the existing patch schema and backend validator/applicator architecture. The browser does not write source YAML directly, does not mutate `generated/`, and does not apply patches by itself.

## Persistence verification loop

Use a copied/temp scene for write testing when you are proving the loop manually; do not mutate checked-in scenes or commit generated `web_scene.json` / `edit_patch.json` outputs. The full loop is:

1. Export a before snapshot:

   ```bash
   python3 scripts/export_workcell_studio_web_scene.py \
     --scene scenes/ur5_2f_test \
     --output build/workcell_studio_web_scene/ur5_2f_test.before.web_scene.json
   ```

2. Edit only an `editable=true`, `locked=false` source layout/environment item in the viewer.
3. Export `edit_patch.json` from the viewer.
4. Validate the patch:

   ```bash
   python3 scripts/validate_workcell_studio_web_scene_edit_patch.py \
     --web-scene build/workcell_studio_web_scene/ur5_2f_test.before.web_scene.json \
     --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json
   ```

5. Dry-run the backend applicator; this must write nothing.
6. Apply with `--write` only on the copied/temp scene. The applicator persists source layout/environment YAML only through safe provenance mapping.
7. Re-export an after snapshot:

   ```bash
   python3 scripts/export_workcell_studio_web_scene.py \
     --scene scenes/ur5_2f_test \
     --output build/workcell_studio_web_scene/ur5_2f_test.after.web_scene.json
   ```

8. Verify persistence:

   ```bash
   python3 scripts/verify_workcell_studio_web_scene_edit_persistence.py \
     --scene scenes/ur5_2f_test \
     --web-scene-before build/workcell_studio_web_scene/ur5_2f_test.before.web_scene.json \
     --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json \
     --web-scene-after build/workcell_studio_web_scene/ur5_2f_test.after.web_scene.json
   ```

9. Only after the verifier passes should you regenerate and validate the ROS scene package. Generated files are not edited directly by the web viewer or applicator. RViz/MoveIt remains the planning truth, and real hardware remains disabled/default-safe.

The verifier checks that `scene_id` matches, each edited item existed before and after, `old_transform` matched the before export, `new_transform` persisted in the after export, locked/generated robot/tool visuals stayed unchanged, and unrelated items stayed unchanged except allowed provenance/timestamp/export metadata.

## Contract

Patch files use `schemas/workcell_studio_web_scene_edit_patch_v1.schema.json` and `schema_version: workcell_studio_web_scene_edit_patch/v1`.

Each patch records:

- `scene_id` and the source web scene schema version;
- `created_by: static_web_viewer`;
- optional provenance such as the source `web_scene.json` file and viewer version;
- an `edits` array of `update_transform` operations with old/new pose XYZ, RPY, and scale XYZ.

Patch exports include only changed items. Exporting with no changed items is allowed and produces an empty `edits` array with a browser warning.

## Applicator safety rules

The applicator reuses the validator and then applies only safe editable updates:

- dry-run is default; `--write` is required for mutation;
- only `update_transform` is supported;
- item provenance must clearly map to `layout/workcell_studio_layout.yaml` or authored `environment.yaml` records;
- ambiguous source mapping fails with a clear error;
- locked items, `editable=false` items, and generated robot/tool visuals are rejected;
- `generated/`, `cell_definition.yaml`, and `scene_manifest.yaml` are never mutated in this phase;
- pose XYZ/RPY are updated; scale is updated only when the source record already has an obvious `scale` or `mesh_scale` field.

Dry-run output lists the scene id, item id, label, source, target file, old transform, new transform, and whether a write would occur. Write mode reports updated file paths, updated item counts, skipped/rejected edits, and the suggested re-export command.

## Generate and validate after Web 3D edits

After a Web 3D edit patch is applied, scene generation and validation remain an explicit operator step. Generation/validation is not automatic after patch apply, and the browser never writes YAML directly. Edit patches are validated and dry-run before apply; apply mode re-exports the web scene and verifies that the persisted editable source YAML is reflected in the after export.

Recommended Workcell Builder flow:

1. Open Web 3D Viewer from Workcell Builder.
2. Edit the scene in the browser.
3. Export `edit_patch.json`.
4. Return to Workcell Builder.
5. Validate, dry-run, and then explicitly apply the patch only if the dry-run output is clean.
6. Confirm that apply re-exports the after web scene and verifies persistence.
7. Click **Generate & Validate Scene** in Workcell Builder.
8. Read the PASS/FAIL output from the generation/validation action.
9. Do not launch robot hardware from this step.
10. Treat fake-hardware RViz/MoveIt launch verification as the next phase.

Safety boundaries:

- This step does not launch ROS, RViz, MoveIt, Gazebo, fake hardware, or real hardware.
- It does not replace RViz/MoveIt; RViz/MoveIt remains the future planning truth and the fake-hardware validation foundation.
- Generated scene validation reports are review evidence only, not safety certificates or permission to run real hardware.

## Relationship to RViz/MoveIt

Web 3D editing is an authoring surface, not a planning or execution backend. No ROS launch is required for patch validation or application. RViz/MoveIt remains the planning and simulation truth, and fake hardware remains the default validation foundation. This workflow does not enable real robot motion, does not replace backend generation/validation/safety gates, and does not modify Qt Scene3D visuals.

## Next phase

Next phase is connecting edit/export/apply/re-export/generate/validate into a guided workflow while preserving preview-only browser edits, backend validation/application, and RViz/MoveIt as planning truth.

## Guided backend edit workflow

Use `scripts/run_workcell_studio_web_edit_workflow.py` to run the supported backend path for a browser-exported Web 3D edit patch without hand-copying every command.

Recommended operator flow:

1. In Workcell Builder, select a scene and use **Export & Open Web 3D Viewer**.
2. Edit an editable, unlocked physical scene item in the browser.
3. Export `edit_patch.json` from the browser. The browser exports a preview patch only; it never writes YAML directly.
4. Run the guided workflow in safe dry-run mode:

   ```bash
   python3 scripts/run_workcell_studio_web_edit_workflow.py \
     --scene scenes/ur5_2f_test \
     --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json
   ```

   The default command exports `build/workcell_studio_web_scene/<scene_id>.before.web_scene.json`, validates the patch, and runs the existing safe applicator in dry-run mode. It does not mutate source files.

5. When the dry-run output is clean, apply explicitly with `--write`:

   ```bash
   python3 scripts/run_workcell_studio_web_edit_workflow.py \
     --scene scenes/ur5_2f_test \
     --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json \
     --write
   ```

   With `--write`, the workflow still validates and dry-runs first, then applies through the safe backend applicator, re-exports `build/workcell_studio_web_scene/<scene_id>.after.web_scene.json`, and runs persistence verification.

Optional modes:

- `--export-only` exports the before web scene and does not require a patch.
- `--validate-only` validates a patch against the exported before web scene and does not apply it.
- `--dry-run-apply` is available for explicit dry-run wording; dry-run is already the default when `--write` is omitted.
- `--verify-persistence` requests persistence verification for write workflows; `--write` already performs it.
- `--run-readiness` optionally runs the scene readiness matrix after the edit workflow.
- `--output-dir` changes where before/after web scene JSON files are written.

Safety notes:

- Source scene files are not mutated unless `--write` is present.
- Locked/generated robot and tool preview edits are rejected by the validator/applicator path.
- Generated files are not edited directly.
- Browser edits remain patch requests; YAML persistence is owned by the backend validator/applicator.
- RViz/MoveIt remains the planning and simulation truth after scene generation and validation.

## Applying Web 3D edit patches from Workcell Builder

Phase 5C adds a guided Workcell Builder UI path around the existing backend workflow script. It does not duplicate validator, applicator, re-export, or persistence verification logic in Qt.

1. Select the intended scene in Workcell Builder and choose **Export & Open Web 3D Viewer**. The builder exports `build/workcell_studio_web_scene/<scene_id>.web_scene.json` and opens the static browser viewer.
2. In the browser, edit only an editable, unlocked physical layout/environment item. Browser edits are preview-only.
3. Use **Export Edit Patch** in the browser to save an `edit_patch.json` file, normally under `build/workcell_studio_web_scene`.
4. Return to Workcell Builder and choose **Validate Web Edit Patch…** or **Dry Run Web Edit Patch…**. The patch picker defaults to `build/workcell_studio_web_scene`, requires an existing JSON file, and runs `scripts/run_workcell_studio_web_edit_workflow.py` for the selected scene.
5. To persist a patch, choose **Apply Web Edit Patch…**. Workcell Builder first runs the same workflow without `--write`. If that dry-run fails, write mode is not offered.
6. If dry-run passes, Workcell Builder shows an explicit confirmation dialog with the scene path, patch path, workflow output/change summary, and a warning that editable source YAML may be updated.
7. Only after confirmation does Workcell Builder rerun `scripts/run_workcell_studio_web_edit_workflow.py` with `--write`. The backend workflow validates, applies through the safe applicator, re-exports an after web scene, and verifies persistence.

Safety boundaries remain unchanged: generated files are not edited directly, the browser never writes YAML, Workcell Builder uses backend validation/dry-run before applying, and RViz/MoveIt remains the planning and simulation truth. This workflow does not start ROS launch files and does not enable real robot motion.
