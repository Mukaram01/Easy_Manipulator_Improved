# Align & Arrange v1 — acceptance

**Status:** CONFIRMED for the two-object workflow on a disposable copy of `ur5_2f_test`, in the real Workcell Builder with the embedded Web3D Product View, at 1280 × 720 and 1366 × 768.

## Environment

- Ubuntu 22.04, ROS 2 Humble, WSLg (LENLEG).
- Fake hardware only: no planning, execution, Gazebo or robot motion.
- The canonical scenes were not opened. The runner used a disposable workspace whose `scenes/` held `git archive HEAD` copies of `ur5_2f_test` and `ur5_airpick4_test`, with an isolated `XDG_CONFIG_HOME`.

## What the runner does

`workcell_builder --align-arrange-acceptance` (`test/align_arrange_acceptance.inc`) drives the real GUI and fails on any unexpected dialog. Every step passed at both sizes.

| Step | Result |
|---|---|
| Select the Destination Bin; Inspector → **Align & Arrange…** | The panel opens. Operations are disabled with "Choose a reference part first…"; Apply is disabled. |
| **Real 3D-view click** on the workbench top | The workbench becomes the reference; the bin stays selected. The viewer move gizmo is disabled (`gizmoEnabled: false`) and both parts are highlighted. |
| Centre on reference + Place on top | Ghost drawn. Preview: "moves 225.8 mm (world ΔX +60.0 · ΔY −217.7 · ΔZ −0.9 mm) to X 0.5100 · Y 0.0023 · Z 0.0991 m". The top is the workbench's verified `surface` datum. Bounds come from mesh vertices (sorting_bin.stl, 316 triangles; table.stl, 102 734 triangles). Scene files and bin pose are unchanged. |
| Cancel | Files hashed unchanged, layout not dirty, bin pose unchanged, viewer preview cleared. |
| Refusals | Self ("is the part being moved"), task zone ("is a task zone…") and a non-authored item are all refused with reasons. |
| Reference from the list; Centre X + Back + X +50 mm; Apply | The bin is at exactly the previewed target, (0.56, 0.30230645, 0.1); orientation is unchanged. |
| Undo / Redo | Exact restore to (0.45, 0.22, 0.1) / back to the target. |
| Save Layout, close, reopen | Reopened pose equals the applied pose: maximum absolute error **0**. |
| Keep relative (Centre Y), then move the workbench +0.1 m | The bin is mounted on the workbench and follows it by exactly +0.1 m. Two undos restore the saved state. |
| Mate: rim + surface chosen, then a click on empty background | `NO_ASSET`. Mate stays active and Ready with both picks; status: "That click missed every part, so nothing changed. Still picked: …". Cancel moves nothing. |
| Open `ur5_airpick4_test` | Strict preparation is blocked by `destination zone 'place_zone_main' missing or ambiguous`. Builder retries as a read-only diagnostic Product View and shows the blocker. |
| Panel audit (`ui_tour_audit`) at 1280 × 720 and 1366 × 768 | 0 clipped, 0 overflow, 0 unreachable controls, both before and during preview. |

Evidence files:
- `align-and-arrange-v1/evidence_1280x720.json` and `align-and-arrange-v1/evidence_1366x768.json`;
- screenshots `align-and-arrange-v1/*.png`.

## Save Layout inspection

After Apply → Save, `layout/workcell_studio_layout.yaml` differs from the HEAD copy only in:
- the bin's pose lines, now `0.56` and `0.30230645`, because planned positions are rounded to 1 nm and no float noise is written;
- three `role:` values.

`environment.yaml` additionally changes the `camera:` pose block.

The role and camera rewrites come from Save itself, not from this workflow:

| Rewrite | Change |
|---|---|
| Role | `support_surface` → `support_surface_table` |
| Role | `target_bin` → `place_target_bin` |
| Role | `pick_zone` → `pick_source_zone` |
| `camera:` pose | `[0.4, -0.25, …]` → `[0.40000000000000013, -0.36004294887185107, …]` |

Align & Arrange never sends a role or camera edit. The same rewrites, with identical numbers, are present in an uncommitted canonical `ur5_2f_test` edited before this work. No wholesale YAML reformatting was produced by this Save path. They are recorded as a separate follow-up.

## Commands

```bash
python3 -m pytest -q tests/test_asset_alignment.py
build/workcell_builder/workcell_align_arrange_controller_test
build/workcell_builder/workcell_mate_tool_controller_test
build/workcell_builder/workcell_scene_preview_widget_ui_test --gtest_filter='ScenePreviewPreparationFailure.*'
workcell_builder --align-arrange-acceptance --workspace "$WS" --scene ur5_2f_test --airpick-scene ur5_airpick4_test \
  --smoke-output "$EVIDENCE" --window-size 1280x720
```

## Not covered

- Native Scene3D fallback: shows numbers but no ghost. Not run.
- Rotated references in the GUI: no canonical scene has one. Covered by `tests/test_asset_alignment.py`, which checks a 30° and a 90° reference.
- 4K and scaled displays.
