# R2.0d/e authoring and runtime parity

Scope: saved TaskIntent v2 → shared resolution → generated task, Validate,
Product View and MoveIt plan-only consumption. R2.0c is accepted as closed for
this work; its manual viewport acceptance is not reopened. No physics, EPD,
camera or real-hardware work is included.

## Source-of-truth investigation

Baseline: main `4d90665be5424d08c1cd03879a871f5ba439eeb0` (PR #3170).
Branch: `codex/r20de-authoring-runtime-parity`.

| Boundary | Authority and transformation | Defect found / correction |
|---|---|---|
| New Cell → Builder | Existing wizard creates the physical cell; `TaskIntentEditor` binds `TaskIntentModel` v2 | Reuse existing controls and atomic Save; no second task UI |
| Save → reopen | `config/workcell_builder_task_intent.yaml`; existing Python authoring adapter migrates v1 and validates v2 | C++/Python semantic hash excludes scene path and migration provenance |
| Physical Save | Authored layout merge writes environment assets and target-local placement | Preserve R1.9 `physical_destination.py`; no new destination representation |
| Generate button | Existing MainWindow action invokes the saved-task exporter | Removed draft repair/default reconstruction and stop package generation on export failure |
| Export | Existing exporter reads persisted normalized model and current physical inputs | Remove missing-task reconstruction and independent legacy task defaults |
| Resolve | Existing `task_intent_resolver.py` selects eligible observations, grasp and placement | Nullable confidence fixed; source/routing constraints explicit; deterministic result identity |
| Feasibility | Existing `full_cycle_preplanner.py` evaluates the complete cycle in a private scene | Consume authored distances, clearance, aperture and constraints; unsupported constraints block |
| Resolution artifact | Existing `generated/task_intent_resolution.{json,yaml}` is derived evidence | Check normalized intent, physical context and semantic result hashes; stale/corrupt evidence cannot authorize planning |
| Generate | Existing scene generator projects the checked resolution into `task_recipe/v1` and manifest | Prevent runtime-source config copying from restoring an old task recipe |
| Offline compatibility preview | Existing preview request is a projection of the resolved recipe and cell | Authored motion distances/axes and resolved pose replace hardcoded offsets; missing v2 context blocks |
| Validate | Existing validator reads the same result and checks handoff hashes | Its task-flow summary uses the resolution, not a second legacy layout interpretation |
| Product View | Existing exporter reads the same result; physical region still comes from R1.9 | Active binding comes from TaskIntent, not `environment.task`; display readiness, source, target, fallback and hashes |
| Plan / Simulate | Existing runtime adapter calls the shared resolver and full-cycle preplanner | No v2 commissioning override; consumer replans only the chosen candidate/destination, never starts another AUTO search |
| Fresh generated launch | Existing UR5/2F physical profile and launch template | Add existing equipment SRDF/controllers/scene loader bindings; mock hardware only, no new planner |
| UI completion | Existing selected-package build and preview process supervision | Selected scene is passed explicitly; plan-only command; successful resolution refreshes Generate/Validate/Product View |

All copied task/recipe/manifest/view fields are projections. Missing evidence
permits a **blocked diagnostic physical handoff**, not an executable task. The
existing Plan Saved Task action consumes a current READY/WARNING result; when
the saved task is unresolved/blocked it resolves against MoveIt, then refreshes
generated consumers. A command-line caller does the same with export/generate →
`--resolve-task` → export/generate → validate/view → normal plan consumption.
MoveIt initially produced invalid attached-object transfer paths, causing AUTO
resolution on restore to select a different candidate. The existing shared UR5
OMPL configuration now uses path-preserving TOTG (`path_tolerance: 0.0`) and
finer collision interpolation (`longest_valid_segment_fraction: 0.001`).
Zero TOTG tolerance alone retained the same result in four probes but three
needed a rejected-path retry, so the final check also tightens collision interpolation. Collision validation remains enabled. This addresses the actual feasibility
drift instead of excluding chosen candidates from the hash. See the official
[TOTG explanation](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html)
and [OMPL interpolation documentation](https://moveit.picknik.ai/humble/doc/examples/ompl_interface/ompl_interface_tutorial.html).
Invalid sampled OMPL paths (`INVALID_MOTION_PLAN`, -2) can retry the identical
plan-only goal twice; retries are recorded and cannot change task selection.
The normal consumer rechecks the exact resolved candidate with MoveIt. A
planning failure after bounded identical-request retries blocks it; it cannot replace that candidate under AUTO.

## Acceptance setup and commands

Humble on `/home/user/workcell_ws`, with a disposable scene workspace at
`/tmp/r20de-acceptance-workspace`. Canonical authored files in the repository were
not changed. The existing Qt New Cell wizard created `r20de_fresh_cell`, followed
by real TaskIntentEditor Save/reopen for both cells. Both explicitly select the
replayed `cup`, AUTO grasp/place, 300 s observation age, and 1 mm placement
clearance. The canonical saved `bottle` is 180 mm tall while its destination
region is 120 mm tall; the historical commissioning runner selected `cup` via an
override. This acceptance authors `cup` explicitly rather than retaining that
silent runtime substitution.

```bash
source /opt/ros/humble/setup.bash
source /home/user/workcell_ws/install/setup.bash
# Workcell Builder including Qt/model tests:
colcon build --packages-select workcell_builder ur5_moveit_config --parallel-workers 2
# Run TaskIntentEditor tests from the repository root: wizard catalog discovery
# in the inherited test harness depends on that working directory.
QT_QPA_PLATFORM=offscreen /home/user/workcell_ws/build/workcell_builder/workcell_task_intent_editor_test
# Retained New Cell acceptance: start with a NEW disposable directory.
mkdir -p /tmp/r20de-acceptance-workspace/src/easy_manipulation_deployment/scenes
cp -a scenes/ur5_2f_test /tmp/r20de-acceptance-workspace/src/easy_manipulation_deployment/scenes/
# Existing wizard/editor test:
R20DE_ACCEPTANCE_WORKSPACE=/tmp/r20de-acceptance-workspace QT_QPA_PLATFORM=offscreen \
  /home/user/workcell_ws/build/workcell_builder/workcell_task_intent_editor_test \
  --gtest_filter=TaskIntentEditor.RuntimeParityCreatesFreshCellAndSavesReplayTask
colcon --log-base /tmp/r20de-acceptance-workspace/log build \
  --base-paths /tmp/r20de-acceptance-workspace/src/easy_manipulation_deployment/scenes/ur5_2f_test \
               /tmp/r20de-acceptance-workspace/src/easy_manipulation_deployment/scenes/r20de_fresh_cell \
  --build-base /tmp/r20de-acceptance-workspace/build \
  --install-base /tmp/r20de-acceptance-workspace/install --symlink-install
source /tmp/r20de-acceptance-workspace/install/setup.bash
python3 scripts/run_task_runtime_parity_acceptance.py \
  --scene /tmp/r20de-acceptance-workspace/src/easy_manipulation_deployment/scenes/ur5_2f_test \
  --scene /tmp/r20de-acceptance-workspace/src/easy_manipulation_deployment/scenes/r20de_fresh_cell \
  --output /tmp/r20de-chain-acceptance-6 --domain-id 150
python3 scripts/run_physical_destination_acceptance.py \
  --scene scenes/ur5_2f_test --output /tmp/r20de-r19-complete.json
python3 -m pytest -q \
  tests/test_task_intent_v2.py tests/test_task_intent_resolver.py \
  tests/test_task_runtime_parity.py tests/test_physical_destination.py \
  tests/test_full_cycle_preplanner.py tests/test_runtime_pick_inputs.py \
  tests/test_task_intent_authoring.py tests/test_new_cell_physical_profile.py \
  tests/test_validate_builder_generated_scene.py tests/test_export_workcell_studio_web_scene.py \
  tests/test_offline_plan_preview_request.py tests/test_workcell_builder_regression_repairs.py \
  tests/test_builder_task_intent.py tests/test_builder_task_intent_to_task_recipe.py
QT_QPA_PLATFORM=offscreen ctest --test-dir /home/user/workcell_ws/build/workcell_builder \
  -R 'workcell_(task_intent_(model|readiness)|new_cell_wizard|rviz_preview_metadata_command|mainwindow_rviz_preview_compile)_test' \
  --output-on-failure
git diff --check
```

The chain runner gives each launch a fresh DDS domain and uses real Generate, recipe validation, scene validation, Product
View export, and two independent MoveIt processes (resolve and consume) per
phase. It compares entire physical destination records, not just IDs. It then
edits the authored place region +10 mm through the existing layout Save merge,
restores the original authored state, exercises PREFERRED fallback and blocked
EXACT, and restores the original task. It never reads a precomputed PASS file.

## Verified results — 2026-09-17

| Cell | Intent SHA256 prefix | Baseline / restored resolution prefix | Edited resolution prefix | Parity |
|---|---|---|---|---|
| `ur5_2f_test` | `04172e87623f` | `3619bbe52494` | `0f1e5f5a6e3f` | 8/8; restore identical |
| `r20de_fresh_cell` | `bc0a32260129` | `1f590e4574e9` | `4f5709922717` | 8/8; restore identical |

Both resolve `default_drop_zone` on `target_bin_default` at `[0.45, 0.22, 0.11]` m,
RPY `[0, 0, 0]`, region `[0.20, 0.10, 0.12]` m. The +10 mm edit moves X to
0.46 m. Both restoration passes return the full original identity. PREFERRED
fallback is reported; invalid EXACT has one attempt, no fallback, and BLOCKED
readiness. All 22 real resolve/plan launches report no execution goals and clean
shutdown; no invalid-path retries were needed in this final run.

- [Contract acceptance](contract-acceptance.json): full hashes, physical destination,
  each consumer identity, policy cases and runtime safety audits.
- [Studio command consumption](studio-command-consumption.json): both selected-cell
  `--resolve-if-needed` commands consume the current resolution unchanged.
- [Save/reopen](authoring-save-reopen.json): C++ saved/reopened and Python normalized
  hashes agree for both cells; environment bytes are unchanged.
- `python-tests.log`: 212 focused tests passed. `cpp-tests.log`: five targets passed.
  `task-editor-tests.log`: 15 passed, opt-in retained test skipped; that retained
  test passes separately in `new-cell-save-reopen.log`.
- `humble-build.log`, `scene-build.log`: Builder/UR5 config and both scene packages.
  `r19-destination-regression.json`: R1.9 geometry/rejection/edit-restore regression.
  `product-view-build.log`: bundle build and freshness check.
- [Source manifest](source-manifest.json): SHA256 of implementation/test inputs.
  [Rejected pre-fix drift](rejected-restore-drift.json) preserves the failure that
  led to the shared MoveIt configuration correction.

## Limits

- Evidence is MoveIt planning with fake hardware and replay. No trajectory
  execution, physics, live perception or physical readiness is claimed.
- Direct pick/place only. Conditional routing, nonzero TCP adjustments, custom
  orientation tolerances, unsupported axes/orientation modes, and unavailable
  contact-quality measurements are explicit blockers. Zero tolerance fields use
  the existing nominal planner checks (3 mm position / 0.01 rad placement).
- Equipment bindings are for the existing UR5 + Robotiq 2F profile. Other robot or
  tool support is not broadened. Existing hand-edited launches are preserved.
- Generated resolution evidence is discarded semantically when authored task,
  physical context or observations change. Resolve again; no automatic policy
  or destination substitution is allowed for a consumed result.
- Prior R1.5 commissioning CLI remains available for its explicit legacy inputs;
  the Studio task action is plan-only. R1.9 geometry regressions are retained.
- Seven additional failures reproduce on baseline `4d90665`: two in
  `test_existing_new_cell_contract_writer.py` (missing authored task), five in
  `test_workcell_studio_web_scene_edit_persistence.py` (old transform/scale fixtures).
  They are retained unchanged; see `baseline-known-failures.txt`.
- Unrelated old Streamlit authoring tests reference removed `bin_red`; no repair
  of that separate UI/fixture is included. CTest's inherited editor-test working
  directory cannot discover wizard scenarios; the editor suite is run from the
  repository root instead.

Rollback: revert this PR and regenerate derived scene outputs. No authored
canonical task, scene or `.worktrees/` contents are changed by the implementation.
