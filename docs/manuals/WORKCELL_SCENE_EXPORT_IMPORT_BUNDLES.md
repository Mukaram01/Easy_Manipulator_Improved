# Workcell Scene Export/Import Bundles

Portable bundles let developers exchange full `workcell_scene/v1` scenes as `.workcell.zip` archives.

## Why
- Keep Qt `workcell_builder` as the primary authoring tool.
- Share scene + task + preview + required mesh assets safely.
- Re-import into scene browser for load/edit/validate/regenerate round-trip.

## Export
```bash
python3 scripts/export_workcell_scene_bundle.py --scene-dir scenes/<scene> --output /tmp/<scene>.workcell.zip --validate --include-assets
```

Included files: `manifest.json`, `environment.yaml`, `config/task_recipe.yaml`, optional perception/summary/preview files, and referenced mesh assets.

## Import
```bash
python3 scripts/import_workcell_scene_bundle.py --bundle /tmp/<scene>.workcell.zip --target-scenes-dir scenes --validate --print-summary
```

Import checks bundle format, blocks unsafe paths, restores scene files, and validates scene contract.

## Asset Rules
- Curated assets: preserve `asset_id` metadata in manifest.
- Generated primitive meshes: bundled under `meshes/`.
- Custom/imported meshes: bundled and sanitized on import.
- Absolute external paths: warned and marked unresolved unless explicitly included.

## Safety
Safety flags remain fake-hardware-first and no runtime motion defaults.

## Operator flow
1. Create/edit scene in `workcell_builder`.
2. Run Offline Validation.
3. Generate files.
4. Export Scene Bundle.
5. Share `.workcell.zip`.
6. Import bundle.
7. Open imported scene.
8. Run Offline Validation.
9. Edit/regenerate as needed.
10. Launch later with `use_fake_hardware:=true` only.
