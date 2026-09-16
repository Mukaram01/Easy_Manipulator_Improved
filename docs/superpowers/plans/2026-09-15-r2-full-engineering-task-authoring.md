# R2.0 Full Engineering Task Authoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a capability-based Workcell Builder Task Authoring workspace that persists explicit user intent, resolves 2F grasp/place candidates through one authoritative readiness path, and keeps preview, generated recipe, and fake-hardware runtime contracts in sync.

**Architecture:** One C++ `TaskIntentModel` owns v2 authored state and dirty/save lifecycle. `pick.selection` is the sole object eligibility authority. Adapters normalize v1 without behavior changes, validate intent, call a deterministic resolver, and emit separate grasp/place resolutions. Installed tool identity comes from the physical scene; intent declares only required capabilities. Existing `task_recipe/v1` remains a derived compatibility output. Product View and Plan/Simulate consume the resolution artifact; R1.9 target-local destination resolution remains the only physical destination path.

**Tech Stack:** ROS 2 Humble, Qt/C++, YAML, existing Python validators/adapters, MoveIt/PlanningScene fake hardware, Product View web export.

**Spec:** [R2.0 Full Engineering Task Authoring Design](../specs/2026-09-15-r2-full-engineering-task-authoring-design.md)

## Global constraints

- Work only in `~/workcell_ws/src/easy_manipulation_deployment`.
- Preserve `ur5_2f_test`, R1.6/R1.9 behavior, Humble, fake-hardware-first defaults, and real-hardware locks.
- Do not implement suction, EPD/RealSense, Gazebo, Isaac, or an operator HMI in R2.0.
- Do not add raw YAML as the primary UX.
- Do not weaken collision, destination, or safety checks.
- Each milestone must leave a reviewable test/evidence artifact; do not claim runtime support without workstation evidence.

## Milestone R2.0a — Contract, normalization, and capability registry

- [ ] Add v2 schema types and canonical serialization/hash in `task_intent_model`, with sole `pick.selection` authority, explicit metres/radians, and target-local pose frames.
- [ ] Normalize v1 without behavior changes: preserve explicit strategy constraints as migrated EXACT, materialize missing catalog fields with provenance, and preserve place behavior through R1.9 references.
- [ ] Define the 2F capability profile, real IDs (`top_2f`, `side_grip_basic`, `finger_pinch_basic`), and installed-tool-to-capability mapping; keep suction unsupported.
- [ ] Extend `validate_builder_task_intent.py` with stable codes for policy completeness, capability mismatch, exact constraints, safety, routing, and destination-chain errors.
- [ ] Add fixtures for AUTO, PREFERRED, EXACT, invalid exact, and legacy v1 migration.
- [ ] Verify schema round-trip preserves numeric units, null confidence, target-local references, and safety flags.

**Exit evidence:** validator JSON shows deterministic v2 normalization and diagnostics for all policy modes; existing v1 fixtures still pass their prior contract.

## Milestone R2.0b — Shared readiness and resolver

- [ ] Implement the resolver interface: normalized intent + installed scene tool profile + scene + observation → separate grasp/place resolutions, checks, selected result, status.
- [ ] Integrate `physical_destination.py` for target-local world pose and usable-region checks; remove any resolver-local pose fallback.
- [ ] Enforce AUTO/PREFERRED/EXACT for both grasp and placement; persist PREFERRED fallbacks, consume EXACT local pose unchanged, and provide no EXACT fallback branch.
- [ ] Add grasp checks for reachability, orientation, aperture, contact, and collision; add placement checks for target reachability, usable region, orientation collision, and retreat feasibility.
- [ ] Emit `generated/task_intent_resolution.yaml` and JSON with intent hash and provenance.
- [ ] Make `task_recipe/v1` conversion consume the selected resolution while retaining legacy mirrors.

**Exit evidence:** headless resolver fixtures demonstrate valid AUTO, reported PREFERRED fallback, valid EXACT, and blocked EXACT; output destination matches the R1.9 resolver.

## Milestone R2.0c — Task Authoring workspace and persistence

- [ ] Build the dedicated Task Authoring panel in `mainwindow.cpp`/UI with What/Pick eligibility, How to grasp, Where, How to place, and Validation sections.
- [ ] Bind every widget to `TaskIntentModel`; remove duplicate direct writes from `SceneSelect` and legacy environment editor paths.
- [ ] Add policy help, capability-filtered real catalog IDs, grasp approach/lift and place approach/retreat controls, TCP/contact/aperture controls, canvas selection, and target-local resolved-destination readout.
- [ ] Implement atomic Save, close/reopen normalization, dirty-state labels, and action gating for Save/Validate/Generate/Plan.
- [ ] Keep Product View edits routed through existing authored layout paths; task edits remain in the task-intent source.
- [ ] Add Qt/model tests for widget-to-model mapping, policy changes, exact-field requirements, and persistence.

**Exit evidence:** a same-session GUI smoke record edits `ur5_2f_test`, saves, closes/reopens, and shows byte-equivalent normalized intent and identical bindings.

**2026-09-16 scoped implementation evidence — PARTIALLY CONFIRMED:** Existing Scene Builder task controls now use `TaskIntentEditor` and `TaskIntentModel` v2. Pick/source and physical target/region bindings, AUTO/PREFERRED/EXACT, motion fields and advanced constraints edit one draft. Save uses `QSaveFile`, detects external changes, reopens the result, and preserves unedited fields and environment/equipment bindings. Existing v1 migration and R1.9 requested-local-pose validation are reused. Invalid EXACT stays authored and blocks generation/planning; stopping an active preview remains available. Legacy SceneSelect generation preserves an existing task file. The existing New Cell wizard is reused; its scene-name edit now refreshes Next/Create availability.

- **Headless, confirmed:** 76 tests passed with `python3 -m pytest -q tests/test_task_intent_authoring.py tests/test_task_intent_v2.py tests/test_builder_task_intent.py tests/test_physical_destination.py tests/test_task_intent_resolver.py`.
- **Qt, headless, confirmed:** `workcell_task_intent_model_test` (9), `workcell_task_intent_editor_test` (5), and `workcell_new_cell_wizard_test` (15) passed. The editor test clicks the existing wizard's Create and Open action for a fresh UR5+2F cell, edits controls, saves, destroys/reopens the editor, and compares normalized intent/hash and unchanged environment bytes. It also checks invalid EXACT, policy changes, advanced units, malformed input, and external-edit protection. This is not workstation GUI acceptance.
- **Targeted build:** `cmake --build /home/ubuntu/workcell_ws/build/workcell_builder --target workcell_builder workcell_task_intent_model_test workcell_task_intent_editor_test workcell_new_cell_wizard_test -j2` succeeded; the final application-only rebuild also succeeded.
- **Existing test limitations:** an earlier affected-label run had 70 passes and four failures in `test_workcell_studio_task_intent_panel.py`, `test_workcell_studio_task_binding_persistence.py`, and `test_new_cell_wizard_links_zones.py`. Each failed token was independently confirmed absent in baseline HEAD `7b3207ba`; those unrelated assertions were not changed.
- **Actual GUI session, blocked:** launched the built Studio on the available display, opened its existing New Cell wizard, and entered a fresh name. The session reported missing `/home/ubuntu/workcell_ws/scenes` and `/home/ubuntu/workcell_ws/src/scenes`; subsequent accessibility actions returned stale object errors and window/screen captures were black. The session was stopped without YAML or terminal repair. Fresh-cell GUI Save/close/reopen acceptance is **UNVERIFIED**, not replaced by the headless test. Repeat once on a working display/workspace after resolving that environment blocker.
- A new scaffold still needs authored R1.9 physical region geometry before planning; authoring persistence does not imply motion readiness. No R2.0d/e preview/runtime integration, EPD, camera, physics, merge, or real motion was performed.


**2026-09-16 HOME PC follow-up — PARTIALLY CONFIRMED:** Reused the unmerged
R2.0c implementation and fixed Home/scene-switch draft loss plus stale disk
validation/reopen behavior. 76 Python and 39 Qt tests pass; a displayed canonical
editor Save/destroy/reopen preserves normalized hash and the 1 cm R1.9 edit.
Full Studio viewport smoke failed, so complete GUI acceptance remains open.
See [commands, logs, screenshot and limitations](../../manuals/evidence/r20c/home-authoring/README.md).

## Milestone R2.0d — Preview, generation, and runtime parity

- [ ] Update Product View export to load the resolution artifact and show selected/rejected candidates and blockers.
- [ ] Block or diagnostic-render stale/missing resolution; never synthesize a preview pose.
- [ ] Update Plan/Simulate and Checks to consume the shared readiness result and expose one actionable blocker.
- [ ] Pass resolution hash and policy to grasp planning/execution; enforce the same exact/fallback rules immediately before planning.
- [ ] Preserve fake hardware and `no_robot_motion` defaults; record runtime gating decisions.
- [ ] Add parity tests comparing authored intent hash, resolution, task recipe, Product View payload, and runtime request.

**Exit evidence:** one generated canonical scene shows the same selected grasp and target-local destination in Builder, Product View, recipe, and dry-run runtime payload.

## Milestone R2.0e — Acceptance and migration closure

- [ ] Run acceptance scenarios A–J from the spec, including motion-intent edits, target/region/local-pose consistency, hash parity, v1 behavior preservation, and R1.9 regression.
- [ ] Run the R1.9 1 cm target-local edit and canonical restore path to prove no destination regression.
- [ ] Exercise v1 open → explicit Save → v2 reopen and legacy recipe consumption.
- [ ] Capture readiness, resolver, preview, and fake-hardware evidence under `docs/manuals/evidence/r20/`.
- [ ] Update the roadmap only after evidence passes; label any skipped live camera/real hardware work explicitly.
- [ ] Perform a code review focused on source-of-truth ownership, policy enforcement, and preview/runtime parity.

**Exit evidence:** acceptance matrix is filled with command/UI evidence, limitations, and clean shutdown; no production claim is made for suction or real hardware.

## Exact first implementation slice after approval

Implement **R2.0a contract normalization only**: add the v2 typed model/normalizer, sole pick-selection authority, target/region/local-pose contract, installed-tool capability mapping, semantic release intent, behavior-preserving v1 migration, and validator fixtures. Do not change Qt widgets, generation, Product View, or runtime in that slice. This creates a testable contract before touching UI or motion behavior.

## Review checkpoints

1. Approve this design and schema.
2. Review R2.0a contract fixtures before UI work.
3. Review resolver output and destination parity before enabling Generate.
4. Review GUI persistence smoke before Product View/runtime wiring.
5. Review the complete acceptance pack before any roadmap “R2.0 complete” statement.
