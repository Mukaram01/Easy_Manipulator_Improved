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
