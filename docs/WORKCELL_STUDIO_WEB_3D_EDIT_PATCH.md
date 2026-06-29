# Workcell Studio Web 3D edit patch contract

This phase connects preview editing to a guarded backend persistence step. Web Studio can move editable layout/environment objects in the browser, export a `workcell_studio_web_scene_edit_patch/v1` JSON patch, validate it, and dry-run or apply it to the correct editable source YAML.

## Workflow

Export the current scene for the static viewer:

```bash
python3 scripts/export_workcell_studio_web_scene.py \
  --scene scenes/ur5_2f_test \
  --output build/workcell_studio_web_scene/ur5_2f_test.web_scene.json
```

Open `workcell_studio_web/viewer/index.html`, load the exported `web_scene.json`, select one `editable=true` and `locked=false` item, and use edit mode to preview the transform. Editable/unlocked selections show enabled XYZ/RPY/scale fields and a Three.js translation gizmo; locked/generated robot/tool previews remain read-only and explain that source layout/environment should be edited instead. Use **Export Edit Patch** to download an edit patch when the preview state is correct.

From Workcell Builder, the **Export & Open Web 3D Viewer** selected-scene action performs only the export-and-open part of this workflow. It writes the export to `build/workcell_studio_web_scene/<scene_id>.web_scene.json` and opens the static viewer. It intentionally does not apply edit patches, write under `scenes/`, mutate source YAML, mutate generated scene files, modify Qt Scene3D visuals, or replace RViz/MoveIt as planning truth. If browser `file://` loading is unreliable, use `python3 -m http.server 8765` from the repository root and browse to `http://localhost:8765/workcell_studio_web/viewer/index.html`.

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

## Relationship to RViz/MoveIt

Web 3D editing is an authoring surface, not a planning or execution backend. No ROS launch is required for patch validation or application. RViz/MoveIt remains the planning and simulation truth, and fake hardware remains the default validation foundation. This workflow does not enable real robot motion, does not replace backend generation/validation/safety gates, and does not modify Qt Scene3D visuals.

## Next phase

Next phase is connecting edit/export/apply/re-export/generate/validate into a guided workflow while preserving preview-only browser edits, backend validation/application, and RViz/MoveIt as planning truth.
