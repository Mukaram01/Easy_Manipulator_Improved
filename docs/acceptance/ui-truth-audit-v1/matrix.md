# Panel matrix — PASS / FIXED / BLOCKED / NOT TESTED

**Legend**
- **PASS:** checked through real GUI interaction and correct as found.
- **FIXED:** a defect was found and fixed here, then checked again in the GUI.
- **BLOCKED:** cannot be checked in this environment.
- **NOT TESTED:** not exercised in this milestone.

The runner is `--multi-select-acceptance` (30 steps). Unless stated otherwise, evidence is in `evidence_*.json`; screenshots are in `screens/`. Files marked `_prev` come from the 29-step set, before the AirPick settle step was added.

## Scene Hierarchy

| Area | Status | Evidence / note |
|---|---|---|
| Parent/child tree and stable IDs | PASS | Mounted parts are shown under their parent, with a relation suffix. IDs are unchanged through save and reopen (reopen error 0). |
| Ctrl+click multi-selection | PASS | Real mouse events with Ctrl on the tree viewport; 4 rows selected. |
| Shift range | PASS (unit) | `SceneSelectionSet.extend`; the GUI drive was not repeated here. |
| Reference indication | PASS | Chip `4 selected · ref: …`, Inspector summary, orange outline in 3D. |
| Delete with several selected | FIXED | Was silently applied to the reference only. It is now disabled, explained, and refused by the command itself. |
| Selection after undo, redo, reopen and cell switch | FIXED | Kept through undo and redo. Cleared, with no stale entries, on a cell switch. |
| Visibility / locking / grouping actions | PASS / NOT TESTED | Locked parts are refused as moving parts (unit and GUI refusal text). Grouping is disabled with an explanation; no group contract exists. Visibility toggles were not exercised. |

## Assets / Asset Library

| Area | Status | Note |
|---|---|---|
| Catalog entries and categories | PASS (visual) | 105 entries with category prefixes ([OBJECT], [CAM], [TOOL]). |
| Thumbnails | PASS / limitation | Mesh thumbnails render. Xacro-based tools show a generic file icon; the log states the reason ("no suitable reader" for `.urdf.xacro`). That is honest, but not a preview. |
| Placeholder assets | PASS | Placeholder cameras are named "… Placeholder" in the catalog. |
| Import / placement / duplicate identity | NOT TESTED | Unchanged by this PR; placement is covered by the existing Mate acceptance (camera stand). |
| Materials / source metadata | NOT TESTED | The catalog has no material model yet; this is the STEP milestone. |

## Inspector

| Area | Status | Note |
|---|---|---|
| Single selection, editable vs locked | PASS | Unchanged. |
| Multiple selection | FIXED | Summary card. The transform section is titled "Transform — *reference* only" and points to Move / rotate… for all parts. |
| Duplicate arrangement actions | FIXED | Inspector *Mate by Picking Faces…* and *Align & Arrange…* removed (see the duplicate report). |
| Units and precision | PASS | Metres and degrees in the Inspector; millimetres in the arrangement fields; nm-rounded persistence. |
| Apply / Undo / Redo / persistence | PASS | Arrangement: one undo step per Apply, exact undo/redo, save and reopen error 0. |

## Checks / Validation

| Area | Status | Note |
|---|---|---|
| Layout | FIXED | Rows floated mid-panel: the tab stack took the 4,200-px Task page height. Tabs now size to the current page, and the rail is top-aligned. |
| Generate YAML | FIXED | Showed **Done** whenever the file existed. It is now **Stale** when the authored cell changed after generation (measured after an edit). |
| Validate | FIXED | Showed **Needed** in every case. It is now **Not run**, **Stale** or **Failed**, from `acceptance/generated_scene_acceptance.json` and in-session edits. Stale state no longer leaks into another cell. |
| 3D preview row | FIXED | Was labelled "3D Scene Preview: Done" beside task steps. It is now "3D preview (visual only)", or for a read-only preview a warning stating that the task is blocked. |
| Running state, timestamps, revision | NOT TESTED / limitation | No validation was run in this milestone (no ROS, Generate or Validate execution). The report status is used, but its timestamp is not displayed yet. |
| Navigation to the problem | PARTIAL | The *Run Next* button runs the recommended action. Jumping from a failed check to the offending part is not implemented. |

## Workflow / Guidance

| Area | Status | Note |
|---|---|---|
| Scene Files list | FIXED | Optional outputs (fallback recipes, readiness/preview reports) showed red "missing". They are now grey "not generated (optional output)"; required files are still red when missing. |
| Next action | PASS | Driven by `resolve_recommended_workflow_actions()`; disabled actions carry a blocker tooltip. |

## 3D Viewport

| Area | Status | Note |
|---|---|---|
| Selection and reference outlines | PASS | 4 outlines, reference orange (viewer state). |
| Ghost previews | PASS | Ghosts for every moving part; cleared on Cancel and Apply. |
| Drawn vs authored transforms | PASS (Web3D) / FIXED (native) | Native mesh offset order fixed in #3221 and checked by a unit test. |
| Read-only diagnostic state | FIXED | Save Layout could be re-enabled by any edit or task change and by the save controller's early branch. It is now disabled everywhere; Ctrl+S and the menu refuse with a reason; no file is written (hash-checked in AirPick). |
| Save Layout painted state | FIXED | The #3221 report ("disabled when checked, enabled in the screenshot") was real: the button *was* disabled but still painted as the blue accent button. The `layoutDirty` highlight carried over from the previous cell, and the stylesheet had no disabled rule. The highlight is now cleared on a cell switch and a disabled Save is always painted muted. The runner checks the painted pixels (mean RGB 233/237/241), not just `isEnabled()`. |
| Viewer edits across cells | PASS | After AirPick finishes loading, the viewer reports scene `ur5_airpick4_test`, `dirtyCount` 0 and terminal `scene_ready`. Nothing from the previous cell leaks. |
| Multi-selection safety | FIXED | Viewer keyboard nudge and direct drag are blocked with two or more parts selected (gizmo already held off). |
| Status pill | FIXED | Truncated at every width ("safe to e…"); the limit was raised. |
| Status strip wording | FIXED | "Product View ready" while the viewer still said "Loading"; it now reports "Product View: N physical items drawn". |

## Header

| Area | Status | Note |
|---|---|---|
| Cell title in the context header | FIXED | It followed the Home table's selected row, so a cell opened another way kept the previous cell's name ("UR5 2F Test" over AirPick). The header now names the opened cell, published by the window. |

## Window and layout

| Area | Status | Note |
|---|---|---|
| Maximized on a second or third monitor | FIXED | The window was clamped back onto the primary monitor (`keepWindowInsideScreen`). |
| 4K maximized (3828 × 2055) | PASS | Real monitor rdp-1. 30/30; all 24 tools inline. `evidence_max4k_rdp1.json` |
| 2560 × 1600 maximized (2548 × 1495) | PASS | Real monitor rdp-0. 30/30; all 24 tools inline. `evidence_max2560_rdp0.json` |
| 1920 × 1080 windowed | PASS | 30/30 with Web3D on rdp-2. The native Scene3D run passed 29/29 on the previous step set, with 5 viewer-only steps skipped and listed. `evidence_w1920.json`, `evidence_native1920_prev.json` |
| 1366 × 768 / 1280 × 720 | PASS | Regression checks. 1366: 29/29 on the previous step set, 17 tools in More. 1280: 30/30, 20 tools in More, every tool reachable. |
| Residuals | PASS | Align front, align left + ΔX 30 mm and 60 mm gap: residual 0 m in every run; save → reopen maximum error 0. |
| HiDPI (DPR > 1) | BLOCKED | WSLg reports DPR 1.0 on all monitors. |
| Panel widths on large workspaces | FIXED | Hierarchy cap 360 → 480 px and Inspector 560 → 720 px, so names are not cut at 4K. The 3D view keeps a 10:1 share. |
