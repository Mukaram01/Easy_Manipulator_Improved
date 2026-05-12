# Workcell Builder Scene Bundles

Workcell Builder now supports **Export Scene Bundle** and **Import Scene Bundle** in Scene Select.

## Export
- Choose a scene and click **Export Scene Bundle**.
- Select an output directory.
- Builder creates `<scene_name>_workcell_bundle/` with:
  - `bundle_manifest.yaml`
  - `README.md`
  - `scenes/<scene_name>/environment.yaml`
  - optional scene manifest and metadata files when present
  - `assets/environment/<used_object>_description/` for used environment assets
  - `reports/export_summary.json` and `reports/validation_summary.md`
  - `checksums/files.sha256`

## Import
- Click **Import Scene Bundle** and choose a bundle directory.
- Builder validates `bundle_manifest.yaml` and required scene files.
- Builder copies bundled environment assets before importing scene files.
- Existing scenes are not deleted; import uses `<scene_name>_imported` on conflict unless explicit overwrite is enabled.

## Included
- Scene files under `scenes/<scene_name>/`
- Generated/custom environment assets used by the scene
- Manifest, reports, and checksum index

## Not included by default
- Full robot and end-effector package trees

Instead, dependencies are recorded under `required_ros_packages` in `bundle_manifest.yaml`.

## Safety and runtime behavior
- Fake hardware remains default.
- Bundles are offline authoring artifacts only.
- Imported bundles are **not** safety certificates.
- No runtime execution, motion behavior, or EPD separation behavior is changed.


## Camera placement metadata
This flow uses `realsense2_description` cameras attached to support assets via `parent_object` + `parent_link` and `runtime_driver: metadata_only`. No RealSense runtime launch is added; EPD remains separate and detection/pick/place zones are out-of-scope for this PR.
