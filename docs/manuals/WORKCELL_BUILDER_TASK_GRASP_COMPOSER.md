# Workcell Builder Task + Grasp Composer

This adds builder-side task/grasp composition metadata for offline preview/export.

- Task templates: `pick_place`, `sorting_placeholder`, `inspection_placeholder`, `machine_tending_placeholder`.
- Pick source and place target are selected from scene assets and exported.
- Grasp strategies: `auto`, `finger_top`, `finger_side`, `suction_top`, `suction_side`.
- Tool compatibility is validated (finger vs suction) with readiness warnings/blockers.
- Placeholder templates are marked preview-only and runtime-unsupported.
- Fake hardware defaults remain preserved (`use_fake_hardware:=true`).
- Metadata is exported into `cell_definition.yaml`, `task_recipe.yaml`, `grasp_strategy.yaml`, `scene_manifest.yaml`, and `builder_export_summary.json`.
- Visual preview artifacts include task labels and pick→place flow arrows.
