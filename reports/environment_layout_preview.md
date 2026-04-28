# Environment Layout Preview

- Source: `/workspace/Easy_Manipulator_Improved/tests/fixtures/environment_layouts/ur5_table_bins_existing_assets.layout.yaml`
- Layout id: `ur5_table_bins_existing_assets`
- Schema version: `environment_layout/v1`
- Validation parser: `fallback`

## Assets

| id | type | asset_ref | source | pose xyz | pose rpy |
|---|---|---|---|---|---|
| main_table | table | table_basic | `package://table_description/meshes/visual/table.stl` | `[0.5, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| overhead_sensor | sensor | realsense_d435i_sensor | `package://realsense2_description/meshes/d435.dae` | `[0.7, 0.0, 1.2]` | `[0.0, 0.0, 0.0]` |

## Zones

- `pick_zone` (pick) bounds={'min': [0.2, -0.7, 0.35], 'max': [0.8, -0.2, 0.75]}
- `red_place_zone` (place) bounds={'min': [0.55, 0.3, 0.35], 'max': [0.85, 0.6, 0.75]}

## Safety zones

- `robot_reach_warning` (warning) bounds={'min': [-1.0, -1.0, 0.0], 'max': [1.0, 1.0, 1.8]}

## Validation warnings

- None
