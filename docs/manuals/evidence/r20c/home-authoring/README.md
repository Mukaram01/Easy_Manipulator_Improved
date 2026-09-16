# R2.0c HOME PC authoring evidence — 2026-09-16

**PARTIALLY CONFIRMED.** This slice reuses the existing unmerged R2.0c editor
(commits `178ba39a`, `6438bd21`) on top of main `7b3207ba`. It adds draft
preservation before scene navigation and disk-conflict detection before
validation. Clean reopening reloads external edits; dirty drafts are retained.
No second Builder, schema, resolver, or planner is introduced.

## Confirmed scope

- Ubuntu 22.04.5 / ROS 2 Humble; application build succeeded.
- 76 focused Python tests, 15 Qt editor tests, 9 model tests, 15 wizard tests passed.
- Both draft-loss and stale-validation regressions failed before the fixes.
- The canonical scene's actual environment and v1 task were copied to a temporary
  directory. The real TaskIntentEditor loaded/migrated them, edited object class,
  grasp approach and EXACT target-local pose, clicked Save, was destroyed, then
  reopened. Normalized bytes/hash, bindings and numerical values matched.
- A 1 cm local X edit resolved to world `[0.46, 0.22, 0.11]`. Environment bytes
  and original canonical authored files were unchanged. See `canonical-editor.png`.
- The canonical editor test also passed with `QT_QPA_PLATFORM=xcb` on DISPLAY=:0.
  This is a displayed component test, not a human-operated full Builder workflow.
- Save/Discard/Cancel dialogs, invalid EXACT persistence, external-edit conflict,
  separate/custom/aliased scene roots, and failed cell publication are covered.

## Commands

Run from `/home/user/workcell_ws/src/easy_manipulation_deployment`:

```bash
source /opt/ros/humble/setup.bash
source /home/user/workcell_ws/install/setup.bash
cmake --build /home/user/workcell_ws/build/workcell_builder --target workcell_builder workcell_task_intent_model_test workcell_task_intent_editor_test workcell_new_cell_wizard_test -j2
python3 -m pytest -q tests/test_task_intent_authoring.py tests/test_task_intent_v2.py tests/test_builder_task_intent.py tests/test_physical_destination.py tests/test_task_intent_resolver.py
/home/user/workcell_ws/build/workcell_builder/workcell_task_intent_editor_test
/home/user/workcell_ws/build/workcell_builder/workcell_task_intent_model_test
/home/user/workcell_ws/build/workcell_builder/workcell_new_cell_wizard_test
QT_QPA_PLATFORM=xcb WORKCELL_TASK_EVIDENCE_DIR=/tmp/r20c-home-evidence /home/user/workcell_ws/build/workcell_builder/workcell_task_intent_editor_test --gtest_filter=TaskIntentEditor.CanonicalSceneSaveCloseReopenPreservesPhysicalTruth
```

The initial multi-target build regenerated its Makefile but did not recognize
its newly added editor-test target in that invocation. A subsequent targeted
build succeeded. Qt emitted platform/session warnings; the wizard test emitted
QThreadStorage shutdown warnings. These logs are not claimed to be pristine.

## Full Studio smoke: FAILED, not milestone closure

```bash
QT_QPA_PLATFORM=xcb timeout 90 /home/user/workcell_ws/build/workcell_builder/workcell_builder --scene3d-smoke --workspace /home/user/workcell_ws --ros-distro humble --scene ur5_2f_test --scene-path /home/user/workcell_ws/src/easy_manipulation_deployment/scenes/ur5_2f_test --smoke-output /tmp/r20c-home-evidence/studio-smoke.json --smoke-screenshot /tmp/r20c-home-evidence/studio-smoke.png --exit-after-smoke
```

Exit 1. The recorded report lists `scene3d_viewport_widget_not_found`,
`active_viewport_counter_handoff_failed`, and paint/screenshot blockers. It
resolved the requested canonical scene and assembled 18 preview items, but did
not prove rendered viewport acceptance. The owned Product View server logged
shutdown. No robot/MoveIt execution was launched. No claim is made that this
failure is new or baseline; a baseline comparison was not run.

## Remaining acceptance

- Drive Home/New Cell navigation and the complete edit/save/close/reopen flow
  through the full Builder shell on this workstation. The navigation guards
  are reviewed, and their editor behavior is tested; the shell path is not
  covered by the component test.
- Resolve/replace the failing full Studio viewport acceptance path with evidence
  from the actual product view; do not substitute the component screenshot.
- R2.0d/e shared GUI/generation/runtime parity and migration acceptance remain
  unfinished. The R2.0b evidence only proves the documented legacy top plan-only
  case, not all strategies or complete v2 runtime parity.
- Physics feasibility and Stage-A piled-object simulation remain gated.

EPD stayed unchanged at `432a15317ff899fa8636533bf24dc44369b360b0`.
No camera, physics backend, or real hardware was used. Fake-hardware and EXACT
no-fallback semantics are retained. This evidence does not claim motion readiness.
