# Engineering UI truth audit v1 (stacked on #3221)

Scope: Workcell Studio Scene Builder and its panels.
- Desktop-first, maximized windows.
- Every visible control checked against its command and authoritative state.
- Defects fixed, not just listed.

No STEP import, no simulation, no hardware.

## Displays actually tested

The LENLEG host now exposes three real monitors through WSLg:

| Monitor | Size | Device pixel ratio |
|---|---|---|
| rdp-1 | 3840 × 2160 | 1.0 |
| rdp-2 | 3840 × 2160 | 1.0 |
| rdp-0 (primary) | 2560 × 1600 | 1.0 |

Maximized runs used `--screen NAME`. The window is placed on that monitor before it is first shown and is then maximized by the window manager; no size is forced.

- **4K:** the Builder maximizes to 3828 × 2055 on rdp-1. This is a genuine 4K workspace, not a scaled or forced window.
- **2560 × 1600:** maximizes to 2548 × 1495 on rdp-0.
- **1920 × 1080:** windowed only. No 1920 × 1080 monitor is attached.
- **1366 × 768 and 1280 × 720:** windowed regression checks.
- **HiDPI:** WSLg reports DPR 1.0 on all three monitors (the Windows display scale is not passed through), so DPR > 1 could not be tested on this host. Text size 112/125 % (Appearance & Layout) is the in-app scale.

Finding: getting onto a second monitor exposed a product defect.
- `keepWindowInsideScreen()` clamped the window into `QWindow::screen()`.
- That screen lags behind a move, and is the primary monitor until the window is mapped.
- So a Builder window placed on a second monitor was dragged back to the primary.
- **Fixed:** it now clamps into the monitor the frame overlaps most.

## Control truth inventory

`ui_tour_audit` now records every visible control:
- class, name, text, tooltip, enabled;
- the number of handlers connected to the signals a user triggers (`clicked/pressed/toggled`, action `triggered`, `currentIndexChanged/activated`, `valueChanged/editingFinished`, `textChanged/editingFinished/returnPressed`);
- whether it has a menu.

An enabled, visible control with no connection and no menu is a dead-control candidate. The tour visits every studio page, every Scene Builder tab and the New Cell wizard.

The before-tour on 4K inventoried **161 distinct visible controls** and found **3 unconnected**. All three are read on demand, so they are real:

| Control | Read by |
|---|---|
| "Show RViz during the cycle" | read when a planning cycle starts |
| `livePerceptionCamera` | filled from the discovered RGB-D streams and read by the live-capture actions |
| `newCellDisplayName` | read when the wizard finishes |

No two visible controls in one view share a label. The duplicates found were semantic: different labels for the same command (below).

## Duplicate and competing controls

| Control | Status | Reason |
|---|---|---|
| Inspector **Mate by Picking Faces…** | **Removed** | Same command (`begin_visual_mate`) as the toolbar **Mate** button. The context-menu entry and the arrangement toolbar's Flush/Coincident/Concentric remain, as contextual entry points to the same command. |
| Inspector **Align & Arrange…** panel and Ctrl+Shift+A | **Removed** | A second alignment UI next to the arrangement toolbar, on the same engine. Its two unique features (exact ΔX/ΔY/ΔZ offsets and keep relative) moved into the toolbar's Align preview bar, so no capability is lost. Its controller, panel, gtest and acceptance runner were deleted. |
| Overflow menu **Layout → Save Layout** | Kept, repaired | Same command as the Save Layout button, but it silently did nothing when the button was disabled. It now states why ("read-only diagnostic preview" / "nothing to save"). |
| Inspector **Mount To / Relative Pose…** | Kept | Unique: explicit parent datum, child datum and offset. The toolbar's Fixed mount always uses the reference origin. |
| Web viewer "Quick placement" (`contextual_placement_actions.js`) | Not visible in the Builder | Lives in the standalone viewer's own inspector, which the embedded Builder hides. Left for the standalone review page; a candidate for removal in a later scoped change. |

## Defects fixed in this PR

The status of every panel is in the [panel matrix](ui-truth-audit-v1/matrix.md).

| # | Defect | Fix |
|---|---|---|
| 1 | Builder on a second or third monitor was dragged back to the primary | Clamp to the monitor the frame overlaps most |
| 2 | Save Layout enabled in a read-only diagnostic preview (any edit, task change, save-controller early branch) | Disabled on every path; Ctrl+S and the menu refuse with a reason |
| 3 | Save Layout *painted* as active while disabled (the #3221 "enabled in the screenshot" report) | Stale `layoutDirty` highlight cleared and repolished on a cell switch; `:disabled` stylesheet rule |
| 4 | Context header kept the previous cell's name | Header names the opened cell |
| 5 | Delete / Duplicate with several parts selected acted on the reference only | Disabled with a reason; the commands refuse too |
| 6 | Arrow-key / viewer nudge and direct drag moved one member of a multi-selection | Blocked with two or more parts selected |
| 7 | Undo history, selection, dirty and stale-validation state crossed into another cell | Cell switch asks (Stay / Discard edits and open) and starts a clean session |
| 8 | Checks: Generate YAML "Done" whenever the file existed; Validate always "Needed"; preview row read as a task step; rows floated mid-panel | Stale / Not run / Failed from evidence; "3D preview (visual only)"; rows top-aligned |
| 9 | Optional outputs listed as red "missing" | Grey "not generated (optional output)" |
| 10 | Status strip "Product View ready" while the viewer said Loading; pill text cut off | "Product View: N physical items drawn"; wider pill |
| 11 | Two duplicate arrangement entry points | Removed (see above); offsets and keep relative moved into the toolbar |

## The intermittent Cancel failure from #3221

| | |
|---|---|
| Claim tested | "Cancel changed the scene" (one run in 7 at 1366 × 768, before the runner recorded which pose changed). |
| Evidence now | Opening a cell writes no authored file (`open_wrote_authored_files: false` in every run). The baseline is taken when the tool starts. Cancel records every moved part. |
| Runs | 0 failures in 16 full runs since: 4K (7), 2560 (2), 1920 Web3D (2), 1920 native, 1366, 1280 (3). |
| Verdict | **UNVERIFIED, not reproduced.** The most likely cause remains a late Product View pose refresh after load that stale-dated the old baseline; that is no longer possible with the tool-start baseline. If it recurs, the evidence names the part. |

## Tests

- **GUI acceptance:** `--multi-select-acceptance`, 30 steps, PASS at 4K maximized, 2560 maximized, 1920 and 1280 on the final code. Every configuration is listed in the matrix.
- **gtests:** run from the package source directory, as CTest does.
  - `scene_selection_set`, `arrange_controller`, `mate_tool_controller`, `semantic_role_edit`, `scene3d_mesh_offset_convention` and `layout_serialization_contract` (22/22) pass.
  - `scene3d_mesh_preview_regression` (9), `scene_preview_widget_ui` (9) and `preview_item_suppression` (1) have failures identical on the #3221 base.
- **Python:** 263 test files, compared with the #3221 base.
  - No new failures from this change.
  - One assertion was updated on purpose (`test_workflow_rail_compact_rendering`: the rail label's size policy, which is the Checks layout fix).
  - Two files need a newer `jsonschema` than this host has, on both trees.
  - `test_r11_canonical_readiness` needs a ROS library path; it errors on the base and fails here for the same environmental reason.
- **Other checks:** the Web viewer bundle is current (`check_stale_bundle.mjs`), and `git diff --check` is clean.

## Limitations

- HiDPI (DPR > 1) is not testable on this host: WSLg reports 1.0.
- No 1920 × 1080 or 2560 × 1440 monitor is attached. 1920 is windowed; 2560 is a real 2560 × 1600 monitor.
- At 4K under WSLg, the screen can lag the viewer's state by a few seconds. Screenshots are taken after the viewer reports its terminal state, plus 5 s.
- Checks shows no timestamps or revisions yet, and there is no jump from a failed check to the part.
- Xacro tools have generic thumbnails.
- The standalone viewer's "Quick placement" remains (not shown in the Builder).
- No validation, generation, simulation or hardware was run.

## Screenshots (`ui-truth-audit-v1/screens/`, native resolution, 128-colour PNG)

| Before | After |
|---|---|
| `before_4k_scene_builder_checks.png`: Checks rows floating mid-panel, "3D Scene Preview: Done" | `after_2560_scene_builder_checks.png`: Stale / Needed rows top-aligned, "3D preview (visual only)" |
| `before_4k_inspector.png` | `after_2560_inspector.png`, `after_2560_assets.png` |
| — | `after_4k_multi_selection.png`, `after_4k_align_front_preview.png`, `after_4k_gap_preview.png`: 4K maximized, all 24 tools inline |
| — | `after_4k_airpick_read_only_settled.png`: AirPick header, Save muted and disabled, read-only pill |
| — | `after_1280_multi_selection.png`: regression size, More (⋯) overflow |

The tour's own 4K "after" captures were taken before the viewer drew under WSLg, so the 4K after-shots come from the acceptance runner, which waits for the viewer's terminal state.
