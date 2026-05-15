# Workcell Studio Canvas Editor

## Capabilities
- Add to Canvas / Add to 3D Canvas from Asset Catalog
- Asset Catalog double-click placement
- Place Asset mode (editor-only)
- Duplicate / Remove Selected Layout Item
- Snap Grid
- Minimap
- Overlays and warnings
- 3D Preview with 2D Layout fallback

## Placement safety
The placement layer flags warnings for:
- overlap
- outside workspace
- too close to robot base
- below floor/table
- missing dimensions/source metadata
- locked item

> Drag/drop into the 3D preview is deferred to a follow-up PR. Current safe path is Add to Canvas and double-click.

## Persistence
Layout edits are persisted to `environment_layout.yaml` and reused by scene generation workflows.

Save Layout writes a timestamped backup before write and keeps this workflow editor-only (no runtime robot motion, no controller command publishing).
