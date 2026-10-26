# R3 main reconciliation — 26 October 2026

Base: main `02bf9eeb3228552b9f9dca30371e5365462a4a5a`. Original R3 head:
`27c8bb32f747732ced501bd394dd7795d6d4173a` (draft #3212, left unchanged). The reconciled content is ported as a
single commit on `integration/r3-tool-interfaces-main`; the obsolete #3211 stack under #3212 is not in its history.
The 2F closure search and reserve use main's arithmetic bit for bit (`open + step*i`, `position - step`).

## Result and scope

**Simulation acceptance PASS** for the 2F-85 on this revision (below). Reconciled the R3 catalog interface,
selection, force variants and optional transport with the newer Stage-A/Stage-B code. This does not commission
real hardware, three-finger or suction engagement, keep-level transport execution, or repeated-picking reliability.

Preserved main's model-derived command joint, SRDF tool group, URDF limits and aperture, physical destination
seating, slab collision geometry, Stage-B replay detection/calibration/recording-clock/class binding,
qualification admission, contact/telemetry/recovery checks and noninteractive ROS launch.

Catalog interfaces are checked against the launched binding before planning; an incompatible joint, group,
range or aperture raises `TOOL_BINDING_MISMATCH`. An interface-free cell keeps its model-derived tool values.
The baseline state helper also keeps an authored home/tool goal when no separate release argument is supplied.
Saved cells own their motion policy; conflicting CLI policy input is rejected before creating a session.

## Verification on LENLEG (26 October 2026, source `66103b58`, clean tree; EPD `9e9b61c`)

- Fast suite (runner `FAST_TEST_FILES`): branch **1,246 passed**; main **1,170 passed, 1 failed** (the
  noninteractive-launch index test, corrected here). Affected source-contract tests: 88 passed / 15 failed on
  both branch and main, identical failure sets (pre-existing). Strict catalog check: every end-effector record
  carrying an interface passes; two pre-existing placeholder records fail as on main.
- `workcell_builder` symlink build: PASS (only the script install list changed; no C++ source changed).
- Stage-A FULL, all qualified overlays: **resolve, cancel, telemetry, stationary, contact-release, full-cycle
  PASS**. Tool revalidation VALID (11 checks), interface `robotiq_2f_85`, hold variant `stage_a_20n` bound.
  Evidence: `~/workcell_ws/r3_integration_evidence/stage_a_full_20261026_174319`.
- Stage-B replay → class-filtered pick/place → home, two consecutive unattended Builder cycles: **both COMPLETED**.
  Cup 0.72 bound and picked, bottle 0.78 excluded (`outside_authored_selection`); both fingertips in contact
  100 % of lift/transfer/place, offset drift ≤ 0.1 mm over 0.726 m; placed within 0.07 mm of the destination
  centre; zero displacement after release; attachments empty, ACM restored, final scene collision-valid,
  clean shutdown, no orphans. Evidence: `~/workcell_ws/r3_integration_evidence/branch_stage_b_20261026_180039`.
- Matched main baseline (same cell recipe, same Builder path): PASS once the host was unloaded; see below.

## Host contention finding

Three earlier main-baseline attempts did not complete. Two were stopped by the unchanged 250 ms telemetry
freshness guard (transfer, home), and one was interrupted externally. The cause was host contention: a VMware
VM started at 13:11 made every per-sample cost about 2.2× slower, including Gazebo's C++ serialization. With
the VM stopped, main passed and the branch passed every gate. No threshold was changed. Physics acceptance on
LENLEG needs other hypervisor guests stopped.

## Rollback

Revert the single port commit. Do not overwrite main's newer files with the old R3 branch. Keep the
unsupported-tool refusal and the real-hardware lock.
