# WORKCELL External Asset Import Wizard

The **External Asset Import Wizard** is now functional end-to-end inside Qt `workcell_builder`.

## Operator flow
1. Open **External Asset Import Wizard**.
2. Browse and select `.stl`, `.urdf`, or `.xacro`.
3. Enter metadata: asset id, label, category (`Custom / Imported`), type, dimensions, pose, z hint, license, source note, tags.
4. Click **Validate Imported Asset**.
5. Click **Add to Asset Library** (managed folder import).
6. Click **Import and Place** to place into scene tooling.

## Managed paths
- Imported file store: `workcell_builder/workcell_builder/assets/imported/`
- Imported catalog: `workcell_builder/workcell_builder/config/asset_profiles/imported_environment_assets.json`

## Validation behavior
- Rejects unsupported file types.
- Rejects unsafe names and path traversal.
- Rejects symlinks and absolute required asset paths.
- Requires license and source note.

## Picker and placement
Imported assets are discoverable under **Custom / Imported** and can be used in Object Placement Manager, Visual Layout Editor, and schema-v1 scene generation.

## Bundle and round-trip
Imported metadata and files are preserved during scene round-trip and `--include-assets` bundle export/import.

## Troubleshooting
- `unsupported extension`: choose `.stl`, `.urdf`, `.xacro`.
- `path traversal rejected`: provide a safe source path.
- `license/source note required`: fill both fields.
- Collision: importer creates deterministic unique file names (`asset`, `asset_1`, `asset_2`, ...).

## Safety note
Use fake-hardware-first validation before runtime launch. No CAD editing, BOM/reporting, MoveIt planning/execution, or real hardware enablement is added by this workflow.
