# Workcell Studio Layout-Aware Generation
Saved canvas layout is merged into generated metadata via `scripts/workcell_studio_layout_merge.py`.

Merge priority:
1. `layout/workcell_studio_layout.yaml`
2. `scene_manifest.yaml`
3. `environment.yaml`
4. template defaults.

Outputs are written to `generated/workcell_studio_merged_environment.yaml`, `generated/workcell_studio_merged_scene_manifest.yaml`, `generated/workcell_studio_layout_merge_report.json`, and `generated/workcell_studio_layout_merge_summary.txt`.

`PREVIEW_ONLY` means metadata is preserved but runtime geometry is not claimed launch-ready.
Acceptance/demo become stale when layout save time is newer than merge artifacts. Regenerate scene after layout edits.

Safety: no robot motion commanded.
