# WORKCELL External Asset Import Wizard

Supports STL/URDF/XACRO (simple). Managed import path: `workcell_builder/workcell_builder/assets/imported/`.

This workflow is not a CAD system. Build geometry externally, then import.

Imported assets are cataloged in `workcell_builder/workcell_builder/config/asset_profiles/imported_environment_assets.json` with license/source_note metadata, and shown in asset picker under `Custom / Imported`.

Bundles include imported assets when using `--include-assets` and preserve relative paths for portability.

Safety note: fake-hardware-first validation only; no robot motion execution.
