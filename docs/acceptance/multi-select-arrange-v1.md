# Multi-selection and arrangement toolbar v1 — acceptance

**Status:** CONFIRMED in the real Workcell Builder with the embedded Web3D Product View, at 1280 × 720 and 1366 × 768. The cell is a disposable fixture: `ur5_2f_test` plus two more sorting bins (`multi-select-arrange-v1/build_fixture.py`), built in a git worktree and restored after every run.

The native Scene3D fallback was not run in the GUI. In that view, Ctrl+click is handled by the same `preview_item_toggled` path, but no ghost is drawn.

## Environment

- LENLEG, WSL2 (WSLg), Ubuntu 22.04, ROS 2 Humble.
- Only `workcell_builder` was built, with two jobs, in a disposable workspace whose source is the worktree.
- Isolated `XDG_CONFIG_HOME`.
- Fake hardware only: no planning, execution, Gazebo, EPD or robot motion.

## Runner

```bash
python3 docs/acceptance/multi-select-arrange-v1/build_fixture.py scenes/ur5_2f_test   # on a scratch copy
workcell_builder --multi-select-acceptance --workspace "$WS" --scene ur5_2f_test --smoke-output "$OUT" --window-size 1280x720
```

The runner (`test/multi_select_acceptance.inc`) uses real input events and fails on any unexpected dialog.

| Step | Result (both sizes) |
|---|---|
| Plain 3D click on the workbench | Selection = [workbench]; it is the reference |
| **Ctrl+click in 3D** on the destination bin | Added. The viewer reports the toggle; the selection is unchanged otherwise |
| **Ctrl+click in the Scene Hierarchy** on two bins | Selection = 4 parts, reference = workbench |
| Views agree | 4 hierarchy rows selected; 4 outlines in 3D (reference orange); move gizmo off; Inspector "4 parts selected — reference Industrial Workbench"; chip `4 selected · ref: Industrial W…` (full text in its accessible name) |
| Tool availability | Align, Distribute, Move and Mount enabled. Flush disabled: "works on exactly two parts". Group disabled: "not part of the scene contract yet" |
| Align front → preview | 3 ghosts; scene files hashed unchanged; no pose changed |
| Cancel | Poses and files unchanged; layout not dirty; selection kept |
| Align front → Apply | Exactly **one** new undo step. An independent re-plan of the same request moves nothing (residual **0 m**) |
| Undo / Redo | All three parts restored / re-applied exactly (1e-12); selection kept (4 parts, same reference) |
| Distribute with gap, 60 mm, X | Preview: "Order along X: Destination Bin → Industrial Workbench → Reject Bin → Spare Bin · gap 60.0 mm". After Apply, the re-plan residual is **0 m** |
| Save Layout → close → reopen | Maximum pose difference **0** |
| Fixed mount (2 bins on the workbench) → move the workbench +0.1 m | Both bins are mounted by one Apply and follow by exactly +0.1 m. Two undos restore the saved arrangement |
| Narrow view | 1280: 7 tools shown, 20 entries in **More (⋯)**. 1366: 10 shown, 17 in More. Every tool is reachable |
| Layout audit | 0 clipped text, 0 overflow, 0 unreachable. Vertical squeeze entries are identical to the pre-existing Project Aura baseline (another studio page sets the stack minimum) |

- Runs: 6 PASS out of 7 at the final code.
- The one failure, at 1366 × 768, was "Cancel changed the scene" before the runner recorded which pose changed. It did not reproduce in the next 6 runs.
- The runner now records the moved parts and takes its baseline when the tool starts.
- The cause is UNVERIFIED. The most likely candidate is a late Product View pose refresh after load, which would have stale-dated the baseline.

## Save Layout fidelity (this branch)

The saved files differ from the fixture only in the moved parts' pose lines (`layout_after_save` in the evidence). The destination zone follows its bin.

- No role rewrites.
- No camera rewrites.
- No reformatting.
- Nanometre-rounded values.

The #3220 Align & Arrange acceptance re-run on this branch also passes (25/25). Its save touches 10 pose lines; on #3220 the same flow also rewrote roles and the camera pose.

## AirPick

`ur5_airpick4_test` still opens as a **read-only diagnostic** Product View: the robot, tool and table are drawn. Save Layout reported disabled when checked, but the later screenshot shows it enabled; that is pre-existing behaviour and was not investigated here.

- The viewer pill reads "3D preview only · Read-only · task blocked: …".
- The status strip reads "3D preview only (read-only) · task blocked — open Checks".
- The missing `place_zone_main` destination is still refused; no destination is invented.

Evidence: `multi-select-arrange-v1/evidence_*.json` and screenshots `multi-select-arrange-v1/*.png`.
