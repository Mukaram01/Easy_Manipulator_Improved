# Detached Contact Separation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Qualify exact initial detached mesh/BOX contacts with continuous support geometry and independent irreversible expiry.

**Architecture:** Enumerate contacts afresh from the immutable scene and bind records to an explicit epoch and the trajectory start. Choose a fixed BOX face plane against the full robot-link mesh. Compose interval kinematics and monotonic support proof with the existing chronological JTC auditor; only accepted intervals commit pair transitions. All unproven pairs remain strict.

**Tech Stack:** C++17, MoveIt/FCL, installed Humble JTC, Boost outward intervals, GoogleTest; existing Python recovery tests.

**Spec:** User attachment `/home/ubuntu/.codex/attachments/48b07110-2b22-44b8-aee2-d88479acfb0e/Pasted text.txt`.

## Global Constraints

- Initial-contact bound remains 100 µm. Baseline ACM unchanged.
- Real hardware locked; no physical acceptance, observation, retry, or resolution.
- All initial pairs must expire and final state must be strict-clear before qualification.
- No authority R until complete proof and exact provenance; A stays revoked.
- Work on `codex/stage-a1-grasp-retention`, PR #3174 stays draft, no merge.
- Stop on an uncertifiable historical opening; do not relax numerical limits.

## Review Focus

- Truncated contact enumeration must fail even when returned pairs look eligible.
- Mimic correlations and nonzero fixed transforms must enter full-vertex derivative bounds.
- Floating JTC/FK error and stored-waypoint switches must not produce false monotonicity.
- Failed parent intervals must not prematurely expire pairs before child certification.
- Stale start states and unrelated collisions must never inherit exact-pair permissions.

### Task 1: Deterministic separation certificate and composition

Files: `workcell_builder/workcell_builder/controller_interval_certificate.hpp`, new `detached_contact_certificate.hpp`, `test/support_contact_policy_test.cpp` in the same directory.

Interface: `certifyDetached(trajectory, scene, epoch, options)` returns a continuous audit plus initial pair records and transition evidence. The ordinary `certify` default remains strict.

- [x] Add a real mesh/prismatic/BOX fixture and a red test expecting a separating path to certify through the new interface; first demonstrate existing strict rejection.
- [x] Implement complete capped FCL contact enumeration, reject self/attached/unknown types, and record exact geometry, poses, overlap and fixed plane.
- [x] Implement interval full-chain support velocity for every mesh vertex. Use root derivative controls, preserve mimic dependence, and account for controller/FK error and endpoint switches. Unsupported or numerically undecidable motion is UNCERTIFIED.
- [x] Compose exact active-pair interval proof with the ordinary pair coverage loop; transactionally commit independent ACTIVE → SEPARATED → EXPIRED events only on certified leaves.
- [x] Add tests for exact/different/new pairs, geometric approach, raw-depth independence, positive expiry, recontact, two independent pairs, self collision, no plane, incomplete enumeration, midpoint collision, stale epoch/start, derivative cancellation, and waypoint boundaries.
- [x] Run full C++ suite (including real Stage A fixture) and the nine scoped Python recovery/planning suites; require existing support/pile and recovery behavior unchanged.

### Task 2: Historical requalification and decision gate

Files: tracked evidence helper/test and `docs/manuals/STAGE_A1_DETACHED_SEPARATION.md`; generated evidence under `/home/ubuntu/workcell_ws/stage-a1-detached-separation-20260924`.

- [x] Only after Task 1 tests pass, run saved iteration 104485 opening diagnostic with all eight moving links and complete scene.
- [x] Record pairs, planes, interval counts, expiry interval/clearance, new-contact witnesses, endpoint state and runtime.
- [x] If proof fails, document the exact first blocker and stop without R or transport execution.
- [ ] **Not applicable: historical candidate rejected.** If proof passes, implement a separately tested recovery candidate interface and authority R bound to epoch, robot/scene/contact/candidate/certificate/trajectory hashes. Reuse owned executor with exact R trajectory admission and permanent A revocation.
- [ ] **Not applicable: no qualified withdrawal / no R.** If integration applies, test complete fake-transport sequence through stationary strict-clear endpoint and CLEAR_FOR_OBSERVATION_PLANNING, then stop before observation.
- [x] Request independent proof/composition review, resolve findings, rerun affected gates, commit/push fully validated certificate and any separately qualified integration. Verify draft PR and remote HEAD.

## Execution ledger

- Native execution in the user-requested existing branch; no checkout or authority change.
- Initial real mesh/BOX regression RED (strict FCL collision) → GREEN with opt-in certificate.
- Mimic-start discontinuity regression RED (incorrect acceptance) → GREEN (explicit rejection).
- Fresh independent proof/code review found a uniform-error lever-arm issue. Added failing mixed revolute/prismatic regression, then computed error using full interval coordinate ranges and reach. Final full suite passes.
- Metric plane clearance regression RED → GREEN with outward norm division.
- Failed-parent expiry rollback and backwards stored-waypoint regressions pass.
- Final gates: 64 C++ / 342 Python pass; original real Stage-A audit counters unchanged. Historical diagnostic was run only after gates passed, then reproduced against final corrected sources.
- Historical stop: exact measured mimic start differs from controller-derived start by up to 5.3930193644191604e-11 rad; one complete eligible initial pair, stable plane available, no expiry, zero certified intervals. R not issued; simulator/fake transport and physical acceptance not run.
- Ruling: preserve the measured state and reject the discontinuity; no numerical normalization or tolerance relaxation. This conservatively blocks recovery until its continuous start provenance is proved.
- The separately qualified recovery integration branch of the plan is intentionally not entered, per user stop conditions. Generic certificate is validated for the declared supported domain; physical withdrawal remains blocked.
