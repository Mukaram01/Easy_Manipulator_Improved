# Workcell Studio environment asset replacement

The generated environment STLs under `workcell_builder/workcell_builder/assets/environment` are legacy layout placeholders. They are not equivalent to proper ROS description-package visuals and must not outrank canonical table, workbench, robot, tool, or camera assets.

## Ownership and safety rules

- Keep UR, Robotiq, suction-tool, and RealSense visuals in their ROS description packages.
- Do not replace articulated robots or tools with a single downloaded STL.
- Use downloaded models only when their licence permits modification and redistribution in this public repository.
- Keep the original source page, creator/manufacturer, licence identifier, and licence text.
- Use metres, ROS Z-up, and a useful floor or mounting origin.
- Use a simplified collision mesh or a primitive collision shape; do not reuse a detailed visual mesh as collision by default.
- Imported assets start as `awaiting_visual_review`. Import does not make an asset canonical.

## Current replacement registry

`config/workcell_studio_asset_replacements.yaml` is the source of truth for legacy placeholder disposition:

- `canonical_available`: a proper repository asset already exists; migrate references instead of downloading another model.
- `awaiting_model`: a licensed external model is needed.
- `code_change_required`: replace the generated STL with an honest URDF primitive.
- `replaced` or `delete_when_unreferenced`: eligible for safe pruning only after all external references are gone.

The first external-model batch is:

1. KLT/industrial bin or tote
2. EUR/EPAL pallet
3. short industrial conveyor section

## Audit the repository

```bash
python3 scripts/manage_workcell_environment_assets.py audit
```

Write machine-readable output:

```bash
python3 scripts/manage_workcell_environment_assets.py audit \
  --format json \
  --output build/environment_asset_audit.json
```

The audit reports:

- mesh and URDF existence;
- SHA-256 and file size;
- basic STL triangle/normal evidence;
- replacement status;
- references outside the asset's own package;
- whether deletion is currently safe.

A poor-looking mesh is not automatically deleted. It must be explicitly marked `replaced` or `delete_when_unreferenced`, and it must have no remaining repository references.

## Import a licensed model

Keep the downloaded model and licence file unchanged, then run:

```bash
python3 scripts/manage_workcell_environment_assets.py import \
  --asset-id klt_tote_600x400 \
  --display-name "KLT Tote 600 x 400" \
  --visual-file ~/Downloads/klt_tote.glb \
  --dimensions 0.6 0.4 0.32 \
  --source-url "https://supplier.example/model" \
  --author "Supplier or creator" \
  --license-id "CC-BY-4.0" \
  --license-file ~/Downloads/LICENSE.txt \
  --redistribution-confirmed
```

Use `--dry-run` first to verify the destination without writing files.

The importer creates:

```text
assets/environment/<asset_id>_description/
├── CMakeLists.txt
├── package.xml
├── asset_manifest.yaml
├── SOURCE.md
├── LICENSE.txt
├── meshes/visual/<original-file>
└── urdf/<asset_id>.urdf.xacro
```

The initial Xacro uses the supplied visual file and a dimensioned box collision. Review the following before catalog approval:

- visible scale against the UR5 and canonical table;
- rotation and Z-up orientation;
- origin and mounting point;
- polygon count and unnecessary internal detail;
- collision geometry;
- source and licence completeness;
- Product View staging and framing.

## Prune obsolete placeholders

Dry-run pruning:

```bash
python3 scripts/manage_workcell_environment_assets.py prune
```

Actual deletion:

```bash
python3 scripts/manage_workcell_environment_assets.py prune --apply
```

The command refuses to delete meshes that remain referenced. Update the asset profile, catalog, scenes, templates, and URDF wrappers first; then set the registry status to `replaced` or `delete_when_unreferenced`.

## What should remain primitive

Detailed internet models are unnecessary for simple geometry. These should move to URDF primitives and then lose their generated STL files:

- calibration cube;
- simple pick box;
- cylinder test object;
- flat fixture plate;
- robot base plate.

This gives exact dimensions, smaller repositories, honest collision geometry, and no meaningless low-quality triangles.
