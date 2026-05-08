# Workcell Builder GUI Manual Authoring

This guide covers manual cell authoring workflows in the Workcell Builder GUI metadata path.

## Select assets
Pick robot, end-effector, tables/workbenches, bins/objects, cameras, and custom STL assets from catalog, then assign role and collision mode.

## Place assets
Edit XYZ/RPY/scale and asset notes/visibility/lock metadata in the placement inspector.

## Import/customize STL
Custom STL assets should be marked `custom_stl`, and can remain `visual_only` or use `bounding_box` collision for safe preview-first use.

## Define pick ROI
Use `camera_pointcloud_roi` with:
- camera asset id
- pointcloud topic (`/camera/depth/color/points` or `/camera/camera/depth/color/points`)
- frame (`base_link`, `world`, `camera_link`, `camera_depth_optical_frame`)
- crop box x/y/z min/max

## Pointcloud filters
Configure:
- remove_table_plane / remove_floor
- min/max_height_above_table
- voxel_leaf_size
- min/max_cluster_size

Plain-English intent:
- Do not pick below this height
- Left/right/forward/back crop limits
- Ignore table/floor points
- Downsample pointcloud

## Define place target
Set target id, target type (`pose`, `bin`, `grid`, `reject_bin`, `pass_bin`, `fail_bin`), pose XYZ/RPY, orientation mode, optional class/colour/shape mapping.

## Validate live authoring
Validation reports Ingredients / Cell / Runtime readiness and flags missing robot, tool, support surface, pick ROI, place target, invalid ROI bounds, and preview-only runtime limitations.

## Export canonical files
Export produces:
- `cell_definition.yaml`
- `environment_layout.yaml`
- `task_recipe.yaml`
- `selected_assets.json`
- `compatibility_report.json`
- `builder_export_summary.json`

Includes placements, roles, collision modes, pick ROI + filters, place targets, grasp metadata, and `fake_hardware_default: true`.

## preview_only meaning
`preview_only` means usable for metadata/visual planning but not runtime-ready hardware execution.

## EPD separation
EPD GUI/runtime controls remain separate; workcell_builder stores passive camera/ROI metadata only.
