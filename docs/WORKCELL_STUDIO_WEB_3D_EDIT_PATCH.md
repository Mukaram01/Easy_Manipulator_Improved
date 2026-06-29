# Workcell Studio Web 3D edit patch contract

This phase connects preview editing to a guarded backend persistence step. Web Studio can move editable layout/environment objects in the browser, export a `workcell_studio_web_scene_edit_patch/v1` JSON patch, validate it, and dry-run or apply it to the correct editable source YAML.

## Workflow

Export the current scene for the static viewer:

```bash
python3 scripts/export_workcell_studio_web_scene.py \
  --scene scenes/ur5_2f_test \
  --output build/workcell_studio_web_scene/ur5_2f_test.web_scene.json
```

Open `workcell_studio_web/viewer/index.html`, load the exported `web_scene.json`, edit one `editable=true` and `locked=false` item, then use **Export Edit Patch** to download an edit patch.

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

After persistence is proven, the next phase is improving browser transform UX/gizmos while preserving the same dry-run-first backend safety model.
