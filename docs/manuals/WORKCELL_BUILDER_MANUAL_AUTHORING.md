# Workcell Builder Manual Authoring

Workcell Builder now supports **Manual Cell Authoring** mode as the main Workcell Studio workflow.

## Build it myself workflow
1. Open builder tools and start manual authoring mode.
2. Choose ingredients (robot, gripper/tool, camera, table/bin/object, fixtures, custom STL).
3. Place assets by editing xyz/rpy/scale and roles.
4. Define camera pointcloud pick ROI (`camera_pointcloud_roi`) with crop box and filters.
5. Define place targets (single pose, bin target, grid, reject/pass bins).
6. Choose task metadata and grasp strategy.
7. Run authoring validation (ingredients ready, cell ready, runtime ready).
8. Export canonical files.
9. Generate runtime outputs only when supported/fake-hardware-ready.

## ROI controls
Use crop box fields and plain-English constraints such as:
- do not pick below this height,
- left/right/forward/back boundaries,
- maximum object height,
- ignore table/floor points,
- downsample pointcloud.

## Export artifacts
- `cell_definition.yaml`
- `environment_layout.yaml`
- `task_recipe.yaml`
- `selected_assets.json`
- `compatibility_report.json`
- `builder_export_summary.json`

## Safety and boundaries
- Fake hardware remains default.
- Preview-only assets remain WARN/preview-only.
- No EPD GUI controls are embedded into workcell_builder.
- EPD launch/runtime remains separate.
