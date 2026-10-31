# Importing CAD into the Asset Library

Workcell Builder imports one `.stl`, `.obj` or `.dae` mesh at a time into this PC's **imported-asset library**. Every cell's Asset Library lists the library, and placing an asset copies it into the cell.

## Operator flow
1. In the Scene Builder's Asset Library, press **+** (Import Asset), or choose **Import Mesh** from the Asset Library menu.
2. Select the mesh file, then choose its source units (millimetres, centimetres or metres).
3. Review the import summary: format, geometry, bounds after unit conversion and origin policy.
4. Enter the licence or owner of the geometry. If you leave it empty, the licence is recorded as `not specified`.
5. The asset is selected in the Asset Library. Press **Add to Scene** to place it, then **Save**.

## Where imported assets live
- **Library:** `workcell_builder/workcell_builder/assets/imported/`. It holds the imported files and their one catalog, `imported_environment_assets.json`.
- **Not in Git:** both are ignored (`.gitignore`), so customer or proprietary CAD is never committed and does not sync to other PCs.
- **In a cell:** Save copies a placed mesh into the cell's `assets/imported/catalog/<content hash>.<ext>`, and the cell refers to that copy by a scene-relative path. A saved cell therefore reopens, generates and validates without the library. STL is stored as binary STL, and OBJ and DAE are copied unchanged.
- **Legacy bootstrap:** the tracked `config/asset_profiles/imported_environment_assets.json` is a read-only legacy bootstrap, an empty list in Git.
  - Until the first import, the library shows the entries in this file. The first import copies them into the local catalog.
  - The file itself is never written. Objects that the old importer appended after the JSON array are recovered.
- **Older scene imports:** imports made by earlier Builder versions into a cell's own `assets/imported/` folder stay in that cell and are listed while it is open.

## What an import records
- **Asset:** a stable ID `imported_<name>`, a label and a category (`Custom / Imported`).
- **File:** the repository-relative mesh path, format, original file name and SHA-256 of the source.
- **Geometry:** source units and `unit_scale_to_m`, bounds, dimensions in metres, the offset from the origin to the bounds centre, and the origin policy (the mesh-local origin is preserved).
- **Provenance:** the licence and a source note. The source's absolute path is never recorded.

## Safety rules
- **Atomic catalog:** the catalog is rewritten atomically as one valid JSON array.
- **No partial imports:** the mesh is copied under a hidden name, verified against the source hash, and only then given its final name. If the catalog cannot be written, the copied mesh is removed.
- **Duplicates:** the same file imported in the same units is reused, not copied again.
- **No overwriting:** a different file with a name already in use gets the next free name (`part_2`, `part_3`, …). An existing file or catalog entry is never overwritten.
- **Rejected input:** symbolic links, unsupported formats, files that do not parse as meshes, implausible unit conversions, and a library folder that is not a real folder inside the repository.
- **Malformed catalog:** blocks every import and is left untouched. Repair it or move it aside.
- **Unsafe entries:** a catalog entry whose path is absolute, escapes the library or is missing is listed as unavailable and cannot be placed.

## Command line
`python3 -m scripts.import_workcell_asset --source part.stl --asset-id part --label Part --asset-type custom [--license ...] [--unit-scale-to-m 0.001]` writes the same local catalog with the same duplicate, no-overwrite and atomic-write rules. `scripts/validate_workcell_asset_catalog.py` validates it.

## Using an imported asset on another PC
The library is local. To share a cell, copy or export its folder: the cell already holds its own copies of the meshes it uses. To reuse a library asset on another PC, import the file there.

## Safety note
Importing and placing CAD changes authored geometry only. It enables no robot motion, controller or real hardware.
