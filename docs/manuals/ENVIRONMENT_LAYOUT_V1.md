# ENVIRONMENT_LAYOUT_V1

`environment_layout/v1` is an **offline placement schema**.

It defines **where existing assets are placed** in a cell. It is not an asset storage layer.

## Core rules

- Existing `assets/` content remains the source of truth for physical meshes/models/descriptions.
- No asset duplication is required.
- Layout files reference existing assets via `package://...` URIs and/or repo-relative paths.
- Layout files contain placement/zones/safety metadata only.

## Schema

```yaml
schema_version: environment_layout/v1
layout_id: ur5_table_bins_layout
metadata:
  name: UR5 table and bins layout
  description: Uses existing repo assets.
assets:
  - id: main_table
    asset_ref: table_basic
    source:
      package: table_description
      uri: package://table_description/meshes/visual/table.stl
      path: assets/environment/table_description/meshes/visual/table.stl
    type: table
    pose:
      frame: world
      xyz: [0.5, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
    collision: true
    visual: true
zones:
  - id: pick_zone
    type: pick
    frame: world
    bounds_xyz:
      min: [0.2, -0.7, 0.35]
      max: [0.8, -0.2, 0.75]
safety:
  zones:
    - id: robot_reach_warning
      type: warning
      frame: world
      bounds_xyz:
        min: [-1.0, -1.0, 0.0]
        max: [1.0, 1.0, 1.8]
```

## Tooling

- Validate: `python3 scripts/validate_environment_layout.py <layout.yaml>`
- Strict mode: `python3 scripts/validate_environment_layout.py <layout.yaml> --strict`
- JSON output: `python3 scripts/validate_environment_layout.py <layout.yaml> --json`
- Preview: `python3 scripts/generate_environment_preview.py <layout.yaml>`
- Batch checks: `./scripts/check_environment_layouts.sh`

## Cell Definition integration

`cell_definition/v1` can optionally include:

```yaml
environment:
  layout: tests/fixtures/environment_layouts/ur5_table_bins_existing_assets.layout.yaml
```

- Optional and backward-compatible.
- Existing `environment.assets` behavior still works.
- Missing layout path: WARN by default, FAIL in `--strict`.
- Project manifests preserve `environment_layout` metadata for offline import/export workflows.

## Runtime boundary

This schema and tooling are metadata-only.

No runtime ROS launch behavior, MoveIt planning behavior, grasp execution behavior, perception behavior, or controller behavior is changed by this layer.


## Builder scene exports for Workcell Studio

Qt Scene3D in `workcell_builder` is an experimental/legacy Debug 3D Preview, not the source of truth. Generated scenes can now export portable Workcell Studio source files using `scripts/export_builder_scene_to_cell_definition.py`. The export writes `generated/cell_definition.yaml`, `generated/environment_layout.yaml`, and `generated/builder_export_summary.json`. Generated scene files and ROS package outputs remain the backend contract, and RViz/MoveIt remains the planning and visualization truth for simulation validation. These files are for offline commissioning and backend tooling, and are not proof of reachability or runtime safety. Keep fake-hardware-first defaults and runtime send disabled unless separately commissioned.


## Authoring pick/place geometry for builder exports

To make Workcell Studio previews resolve exact coordinates (no fallback), author these entities with stable IDs:

- `zones[]`: pick zones (`id`, `label`, `frame`, `pose.xyz`, `pose.rpy`, `dimensions`)
- `targets[]`: place targets/bins (`id`, `label`, `frame`, `pose.xyz`, `pose.rpy`, `dimensions`)
- `objects[]`: objects (`id`, `class`/`label`/`color`, `frame`, `pose`, `dimensions` or mesh reference)
- optional `camera` pose for preview/readiness reporting

Builder task intent and routing should reference these IDs directly (for example `pick_zone_main` and `bin_red`).

## Workcell Studio Layout Editing (Preview-Only)

In **Scene Builder**, selecting an asset in **Asset Catalog** and pressing **Add to Canvas** creates a persistent layout instance in the selected scene's `environment_layout.yaml`.

- Edit pose in Inspector (`x/y/z/roll/pitch/yaw`).
- Click **Save Layout** to write metadata to `environment_layout/v1` (`placed_assets`).
- Use **Validate Layout** to run offline validation checks (no launch, no motion).
- Use **Remove Selected Layout Item** to delete only the placed instance metadata (never source mesh/URDF files).
- If an existing `environment_layout.yaml` is malformed, Studio creates a timestamped backup before rewriting.

Safety: layout editing is **preview metadata only** and does **not** execute robot motion.
