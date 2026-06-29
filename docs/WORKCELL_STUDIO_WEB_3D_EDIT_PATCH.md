# Workcell Studio Web 3D edit patch contract

This phase adds preview-only transform editing to the static Web 3D viewer. Web Studio can move editable layout/environment objects in the browser and export a JSON edit patch, but it does **not** apply patches to source files yet.

## Contract

Patch files use `schemas/workcell_studio_web_scene_edit_patch_v1.schema.json` and `schema_version: workcell_studio_web_scene_edit_patch/v1`.

Each patch records:

- `scene_id` and the source web scene schema version;
- `created_by: static_web_viewer`;
- optional provenance such as the source `web_scene.json` file and viewer version;
- an `edits` array of `update_transform` operations with old/new pose XYZ, RPY, and scale XYZ.

Patch exports include only changed items. Exporting with no changed items is allowed and produces an empty `edits` array with a browser warning.

## Safety and source of truth

The viewer is not the source of truth in this phase. It does not mutate:

- `environment.yaml`
- `layout/workcell_studio_layout.yaml`
- `cell_definition.yaml`
- `scene_manifest.yaml`
- generated scene files

Locked/generated robot and tool preview items are inspectable only. The viewer disables their transform inputs and tells the user to edit source layout/environment instead.

## Validation

Use the standalone validator before trusting an exported patch:

```bash
python3 scripts/validate_workcell_studio_web_scene_edit_patch.py \
  --web-scene build/workcell_studio_web_scene/ur5_2f_test.web_scene.json \
  --patch build/workcell_studio_web_scene/ur5_2f_test.edit_patch.json
```

The validator checks scene identity, item existence, editability, lock state, generated robot/tool guards, and finite transform numbers. It only reads JSON inputs and never writes source scene files.

## Relationship to RViz/MoveIt

Web 3D editing is an authoring preview. RViz/MoveIt remains the planning and simulation truth, and fake hardware remains the default validation foundation. This change does not enable real robot motion and does not replace backend generation, validation, or safety gates.

## Next phase

The next phase is a guarded backend application step that takes a validated patch and writes the appropriate layout/environment source files deterministically, with clear conflict handling and validation evidence.
