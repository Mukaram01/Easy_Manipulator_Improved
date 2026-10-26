# R3 main reconciliation — 26 October 2026

Base: main `02bf9eeb3228552b9f9dca30371e5365462a4a5a`. Original R3 head:
`27c8bb32f747732ced501bd394dd7795d6d4173a`. Both ancestries are retained in the local integration merge.
The working checkout is `/home/ros2/workcell_ws/r3_reconcile_ws/src/easy_manipulation_deployment`.

## Result and scope

**PARTIALLY CONFIRMED.** Reconciled the R3 catalog interface, selection, force variants and optional transport
with the newer Stage-A/Stage-B code. Keep the PR draft. These results do not commission real hardware,
three-finger or suction engagement, or establish repeated picking reliability.

Preserved main's model-derived command joint, SRDF tool group, URDF limits and aperture, physical destination
seating, slab collision geometry, Stage-B replay detection/calibration/recording-clock/class binding,
qualification admission, contact/telemetry/recovery checks and noninteractive ROS launch.

Catalog interfaces are checked against the launched binding before planning; an incompatible joint, group,
range or aperture raises `TOOL_BINDING_MISMATCH`. An interface-free cell keeps its model-derived tool values.
The baseline state helper also keeps an authored home/tool goal when no separate release argument is supplied.
Saved cells own their motion policy; conflicting CLI policy input is rejected before creating a session.

## Fresh verification

- Focused baseline on main: 334 passed, 1 failed in 10.41 s. The failure was the pre-existing test indexing the
  launch target before accounting for `--noninteractive`. Its command-boundary assertion is now corrected.
- Final canonical runner `FAST_TEST_FILES`, with ROS Humble and the workstation workspace sourced:
  **1,246 passed in 131.05 s**. Log: `/home/ros2/workcell_ws/r3_reconcile_ws/canonical-fast-tests.log`.
- Explicit selection/persistence tests plus saved-policy rejection: **13 passed in 96.89 s**.
  Both force variants passed the real Studio Save projection, reopen, Generate, Validate, stale-handoff check,
  actuator-effort derivation and matching hold receipt. This is command-path evidence, not interactive GUI proof.
- Binding mismatch/no-interface regressions were run red before correction (6 failures), then included in the
  passing canonical suite. The authored baseline-helper and saved-policy regressions also ran red before green.
- Isolated `workcell_builder` build: PASS; canonical `ur5_2f_test` symlink build: PASS. No C++ source changed.
  Logs: `/home/ros2/workcell_ws/r3_reconcile_ws/build-final.log`, `/home/ros2/workcell_ws/r3_reconcile_ws/build-scene.log`.
- Independent code review found no additional confirmed blocking defect. Its unsourced test invocation reported
  79 passed / 3 environment failures (xacro metadata and moveit_msgs unavailable); the sourced canonical and
  persistence results above are the integration validation evidence, not that unsourced invocation.
- Stage-B acceptance cell copied and regenerated with the reconciled exporter/generator: validation rc 0;
  retained source hashes unchanged. Record: `/home/ros2/workcell_ws/r3_reconcile_ws/acceptance/authoring.json`.
- Bounded Stage-B no-motion Resolve: **BLOCKED**. Report: `/home/ros2/workcell_ws/r3_reconcile_ws/acceptance/replay-resolve-retry/stage-a1-targeted-report.json`.
  Executor failure: `SEARCH_BUDGET_EXHAUSTED: candidate search budget exhausted`.

The replay detector and recording-clock checks passed; tool revalidation was VALID and the fresh simulator
hold receipt matched. The executor reported `execution_attempted=false`, `shutdown_clean=true` and `recovery_required=false`. The owned launch exited with code 0, no signal escalation and no surviving owned processes. Candidate search exhausted the
explicit 180 s diagnostic budget before a full plan was certified; this is a runtime blocker, not a passing
acceptance. It does not establish whether the same budget would pass on main: no matched baseline runtime
comparison was run. No planning limit or acceptance threshold was relaxed. The run used the uncommitted
integration tree. Only final evidence documentation changed after this run; the subsequently committed product code is the tested code.

Native preflight: 10 action tests passed; 102 support/contact tests passed and one optional known-clear
fixture was skipped. An initial preflight stopped before launch because a normal isolated colcon install
owned a copied telemetry library. That generated artifact was preserved as `.so.colcon-original`; the
commissioning script then created and verified its required build link on retry. No ownership check was
bypassed and no qualified native source was edited. Original attempt retained under `acceptance/replay-resolve`.

## Remaining acceptance and rollback

No new interactive Builder acceptance or full executed physics cycle is claimed here. The 24 October R3
plan-only results and NO-GO history remain historical. Main's later Stage-A/Stage-B evidence remains valid
only for the revisions and scenarios it measured; it is not relabelled as evidence for this combined revision.
Optional keep-level execution and repeated-cycle qualification remain separate acceptance work.

Rollback the reconciliation through a reviewed revert relative to its main first parent; do not overwrite
main's newer files with the old R3 branch. Keep the unsupported-tool refusal and real-hardware lock.
