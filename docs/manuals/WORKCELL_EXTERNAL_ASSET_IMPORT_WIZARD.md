# Importing CAD into the Asset Library

Workcell Builder imports one `.stl`, `.obj` or `.dae` mesh at a time into this PC's **imported-asset library**. Every cell's Asset Library lists the library, and placing an asset copies it into the cell.

**Only STL can be placed in a cell.** OBJ and DAE imports appear in the Asset Library, but Save refuses to place them: collision geometry is read from STL only. Convert the mesh to STL and import it again.

## Operator flow
1. In the Scene Builder's Asset Library, press **+** (Import Asset), or choose **Import Mesh** from the Asset Library menu.
2. Select the mesh file, then choose its source units (millimetres, centimetres or metres).
3. Review the import summary: format, geometry, bounds after unit conversion and origin policy.
4. Enter the licence or owner of the geometry. If you leave it empty, the licence is recorded as `not specified`.
5. The asset is selected in the Asset Library. Press **Add to Scene** to place it, then **Save**.

## Where imported assets live
- **Library:** `workcell_builder/workcell_builder/assets/imported/`. It holds the imported files and their one catalog, `imported_environment_assets.json`.
- **Not in Git:** both are ignored (`.gitignore`), so customer or proprietary CAD is never committed and does not sync to other PCs.
- **In a cell:** Save copies a placed imported mesh into the cell's `assets/imported/catalog/<content hash>.stl`, stored as binary STL. The cell refers to that copy by a scene-relative path, so a saved cell reopens, generates and validates without the library.
  - **Not in Git, in any cell:** Git ignores `scenes/*/assets/imported/` in every scene, reference scenes included, so `git add -A` never stages customer CAD.
  - **Files already tracked stay tracked,** such as the two reference parts in `ur5_2f_test`.
  - **Library meshes:** a placed mesh from the tracked repository library (`assets/`) is copied to `assets/library/` instead. That folder stays trackable, so an edited reference scene can be committed with it.
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

## Promoting an imported part into a reference scene
A Git-tracked reference scene that uses an imported part is **not portable**: the part is ignored, so another checkout does not have it. Offline validation reports this as a `Not portable` warning, naming the file. The warning does not change the validation result on this PC.

Promote the part only as a deliberate, reviewed change:
1. **Licence:** confirm that its licence allows redistribution in this repository, and record who approved it.
2. **Add the cell's copy explicitly:** `git add -f scenes/<reference scene>/assets/imported/catalog/<file>.stl`. The `-f` overrides the ignore rule for that one file, and `git add -A` never does this.
3. **Provenance:** record the licence, source, original file name and the SHA-256 of the committed file in the scene's `assets/imported/asset_manifest.yaml`.
4. **Commit, then re-run Offline Validation:** the warning clears once the file is tracked.

If the part must not be redistributed, keep it out of reference scenes and use your own cell.

## Format limitations
- **Placement:** only STL meshes can be placed and generated. OBJ and DAE are imported for preview in the Asset Library.
- **Companion files:** only the selected file is imported. An OBJ's `.mtl` material file and a DAE's texture images are not copied, so materials and textures are not shown.

## Using an imported asset on another PC
The library is local. To share a cell, copy or export its folder: the cell already holds its own copies of the meshes it uses. To reuse a library asset on another PC, import the file there.

## Safety note
Importing and placing CAD changes authored geometry only. It enables no robot motion, controller or real hardware.
