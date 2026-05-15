# Workcell Studio Scene Bundles

A scene bundle is a portable developer handoff package for one Workcell Studio scene. It is preview-focused metadata packaging, not runtime execution.

## Export
- Open Workcell Studio and select an existing scene.
- Go to **Export** page.
- Click **Export Scene Bundle**.
- The tool creates a bundle folder first, then a zip archive like:
  - `<scene_name>_workcell_studio_bundle/`
  - `<scene_name>_workcell_studio_bundle.zip`

## Import
- Go to **Export** page.
- Click **Import Scene Bundle** and choose a bundle folder or `.zip`.
- Import validates `manifest.json` and requires at least one scene metadata file.
- If a scene name already exists, importer creates a safe suffix name such as `my_scene_imported_2`.

## Safety limitations
- Bundles preserve preview-only metadata defaults:
  - `preview_only: true`
  - `use_fake_hardware_default: true`
  - `no_robot_motion: true`
- Export/import does **not** add runtime execution.
- Export/import does **not** command robot motion.
- Bundles are not safety certificates.
