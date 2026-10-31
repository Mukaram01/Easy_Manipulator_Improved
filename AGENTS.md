# AGENTS.md — Workcell Studio / Easy Manipulation Deployment

## Purpose and authority

This file is the operating contract for AI/Codex work in this repository.

Use it to keep development aligned with the evidence-driven Workcell Studio roadmap. When repository evidence, runtime evidence, and old planning notes disagree, prefer the newest verified repository/runtime evidence and update the documentation rather than preserving an optimistic claim.

### Instruction-document authority

- `AGENTS.md` is the authoritative contributor and AI/Codex operating contract.
- `docs/manuals/WORKCELL_STUDIO_ROADMAP.md` remains the detailed historical/current product roadmap, but its dated status paragraphs may lag verified releases. The focused release charter and sequencing below supersede stale M0/M1 or "Gazebo later" instructions until that roadmap is reconciled in a separate scoped change. Preserve historical evidence; do not silently relabel it.
- `Workcell_Studio_AI_Operating_Guardrail.md` is not a current or historical tracked path in the available repository refs. It was not renamed to the roadmap. Treat requests to read it as stale references to this `AGENTS.md`, and do not recreate a second operating contract under that name.

Do not measure progress by PR count, test count, or the amount of scaffolding added. Measure progress by user-visible acceptance evidence for the canonical Workcell Studio workflow.

## Workspace layout and generated output

The normal workspace root contains exactly `src/`, `build/`, `install/` and `log/`, including hidden entries. Never create new milestone, experiment, trial, evidence, backup or nested-workspace directories at that root.

- Put persistent evidence and logs under the existing `log/` hierarchy; disposable scratch belongs under `/tmp`.
- Reuse canonical source/build/install trees. Dependency source snapshots belong under `src/.runtime-sources/` with `COLCON_IGNORE`; do not recreate `trajopt_DISABLED_BACKUP` at the root.
- User-created cells live in the repository's `scenes/` and are Git-ignored by default. User-imported CAD under `workcell_builder/workcell_builder/assets/imported/` is ignored too. Only the reference scenes allowlisted in `.gitignore` are tracked, with tracked, licensed assets. Never commit a user cell or customer CAD as a side effect; never untrack a reference scene without a decision. See `docs/manuals/SCENE_CREATION_WORKFLOW.md`, "Where cells are saved".
- On LENLEG, source is consolidated on canonical main. The qualified G6 deployment is stored beneath `install/qualified-runtime` and `log/runtime/qualified-g6`. Its original absolute paths are provided only inside its private, unprivileged runtime view; they are not host-root aliases.
- Use the installed Builder/HMI entrypoints or `workcell-builder`, `workcell-operator-hmi` and `workcell-qualified check` after sourcing `install/setup.bash`. See the workstation's `log/WORKSPACE_LAYOUT.md` for qualified CLI commands and entrypoint refresh after a package reinstall.
- Preserve qualification records, raw evidence, native/source identities, user edits and the migration map. Never rewrite a record or loosen a gate to make relocated data appear qualified.
- The recorded LENLEG G4/G6 bounded Gazebo qualification is not a request to repeat physics campaigns for housekeeping. Real hardware and continuous production remain locked. A different cell or changed qualification-bound source/library is not automatically qualified.

## Project identity

This repository is the primary Workcell Studio / Easy Manipulation Deployment workspace.

Workcell Studio is an internal configurable robotic-cell platform for authoring, generating, validating, simulating, demonstrating, and eventually commissioning industrial robotic cells. Its first go-to-market hypothesis is an **internal integration/engineering accelerator**, with optional integrator licensing only after customer demand and product support costs are validated. The purpose of the investor showcase is to earn credible pilot interest and potential fundraising discussions, **not** to claim that investment, valuations, performance or market fit have already been secured.

**Current investor-demo release scope is exactly three flagship scenarios:** (1) controlled pick-and-place, (2) configurable inspection/sorting including indexed conveyors, and (3) random bin picking. Multi-robot composition, fixed/external cameras, interchangeable tools and reusable capability definitions are cross-cutting requirements, not fourth scenarios. Machine tending, palletising, welding, assembly and other uses remain future extensions, not current release commitments.

The primary product UI is `workcell_builder`. Improve `workcell_builder` directly.

Do not replace the product with Streamlit, notebooks, dashboards, or separate demo UIs. Those may exist only as optional reports, exports, validation summaries, or demo artifacts.

Keep Easy Manipulation Deployment / Workcell Builder separate from EPD. Do not merge the EPD GUI or perception implementation into Workcell Builder.

## Product boundary

`Easy_Manipulator_Improved` owns:

- Workcell Studio product shell
- `workcell_builder`
- scene and cell authoring
- environment and task authoring contracts
- generated scene/workcell packages
- validation and readiness reporting
- RViz/MoveIt orchestration
- grasp-planner and grasp-execution wiring
- fake-hardware simulation
- runtime readiness checks
- consumption of normalized EPD results
- review/demo/commissioning bundles

`epd_Improved` owns:

- RealSense input
- detection
- localization
- tracking
- classification
- perception algorithms
- EPD GUI and perception-specific configuration
- the authoritative perception-adapter implementation/configuration where appropriate

Boundary:

```text
EPD produces normalized perception results.
Workcell Studio consumes those results as runtime task inputs.
Workcell Studio owns cell authoring, generation, validation, planning, simulation, and task orchestration.
```

Do not make EPD own cell definition, scene generation, task definition, task planning, or Workcell Studio UI state.

## North-star workflow

The product workflow is:

```text
open or create cell
→ edit physical scene and task intent
→ save
→ close and reopen with identical state
→ generate/refresh the scene package
→ validate the generated handoff
→ build in Ubuntu 22.04 + ROS 2 Humble
→ launch RViz/MoveIt with fake hardware
→ plan and simulate the task/grasp
→ consume replayed or live perception when required
→ export review evidence
→ guarded real hardware later
```

The north-star scene is:

```text
scenes/ur5_2f_test
```

The second canonical scene is:

```text
scenes/suction_test
```

Do not broaden fixes to the full scene catalog until the relevant contract has been proven on `ur5_2f_test`. Apply the same proven contract to `suction_test` before claiming modality breadth.

## Evidence standard

A file existing, a static test passing, or a launch command being documented does not by itself prove product readiness.

Use these evidence labels:

- **CONFIRMED** — directly inspected or run with attached evidence.
- **PARTIALLY CONFIRMED** — some contract or command is verified, but the full user workflow is not.
- **INFERRED** — a reasonable conclusion that still needs direct proof.
- **BLOCKED** — required evidence cannot currently be collected or a real blocker prevents completion.
- **UNVERIFIED** — no reliable evidence has been inspected.

Never call a scene `supported` solely because it is present in the repository or passes static validation. A supported label must be backed by workstation acceptance evidence appropriate to the milestone.

When a required environment is unavailable:

- do not invent success;
- do not replace runtime evidence with a synthetic fixture and call the milestone complete;
- report exactly what was run;
- report exactly what was skipped and why;
- identify the smallest next action on a real ROS Humble workstation.

## Active release charter — three scenarios, investor evidence

**Decision date: 2026-10-21.** This charter is the priority for new work. Historical R1/R2 milestone material remains useful evidence but must not be reopened automatically.

### Verified baseline and outstanding proof (snapshot; re-check live repository before each task)

- The existing Workcell Builder/New Cell Wizard, TaskIntent v2, AUTO/PREFERRED/EXACT resolution, Save/Generate/Validate/Product View, and MoveIt fake-hardware flow have meaningful implementation and recorded acceptance. The repository roadmap records R1.9 complete same-session fake-hardware cycles and R2.0d/e authoring/runtime parity. Do not restart M0/M1 or redesign these components without a reproducible regression.
- Physics-backed Stage-A has demonstrated measured two-finger contact, attachment, 100 mm lift, transfer and placement approach in different bounded sparse-world trials. **One accepted complete physics-backed cycle, reliable repeated pile picking, and the full simulated RGB-D→EPD loop remain UNPROVEN.** Passing five of six gates is not full-cycle acceptance.
- As reported in Trial 22 (2026-10-21), an approximately 190 ms executor stall correlated with stale telemetry during contact-release; the underlying native call was not identified and the stop was not formally verified. The Draft #3198 stack contains relevant changes and evidence. Do not merge it or declare the runtime commissioned solely on these reports. Verify PR/head/current runtime status rather than assuming this snapshot remains current.
- Known stall classes include bridge delivery, simulator scheduling, Python GC (mitigated in a bounded trial), and an as-yet-unidentified executor-thread hold. Avoid another hour-long full suite simply to locate a millisecond-scale defect.
- Real hardware is not validated or authorised. Keep real_hardware_enabled false and fake hardware as the default.

### Release outcomes (priority order, not a demand to redo completed work)

1. **Fast and safe developer feedback:** profile the existing six-gate runner; isolate a failing gate or no-motion component; find the unexplained executor stall with evidence. Maintain a separate full acceptance mode. Do not introduce another broad validator framework.
2. **Core Stage A reliability:** achieve one complete physics-backed grasp→lift→transfer→place→release→retreat→home cycle using the qualified sparse cell; then show repeated picking from a changing pile using simulator RGB-D processed through the existing EPD boundary. Distinguish EMPTY from NO VALID PICK. Real contact and resettling must be measured, not faked with attach/detach alone.
3. **Stage B perception:** replace simulated images with a calibrated real D435i at a configurable external/fixed location while keeping robot execution virtual. Preserve the same normalized object/task interfaces; do not feed camera-frame coordinates directly into a world-frame planner.
4. **Stage B2 scenario expansion:** deliver controlled pick-and-place and indexed-conveyor intelligent sorting, including configurable camera/sensor placement, multi-attribute rules, destinations and a first two-robot independent-station cell. Reuse current Builder, resolver, MoveIt and EPD contracts. Moving-belt interception comes only after stopped/indexed tracking passes.
5. **Stage B3 investor-demo qualification:** demonstrate all three scenarios, variant configurations (not just renamed scenes), at least two robot instances, and both 2F and suction where capability evidence allows; package repeatable measurements, limitations, video and business evidence.
6. **Stage C physical commissioning:** a later, separate hardware-specific programme with independent machine risk assessment, guarding, E-stops, functional safety engineering, interlocks and commissioning. Simulation evidence alone never authorises physical motion.

### Product acceptance gates

**Gate A — Controlled pick-and-place**

- A beginner uses the existing GUI to configure a supported robot/tool, source, known part, placement target, orientation and task policy; saves, reopens, generates and validates without manual YAML/source repair.
- MoveIt collision checks, task resolution, planned approach/grasp/retreat/transfer/place, execution, release, home, failures and re-run are demonstrated with attached evidence.
- Preserve earlier proven fake-hardware acceptances; do not confuse them with uncompleted Gazebo contact-physics acceptance.

**Gate B — Intelligent sorting with an indexed conveyor**

- Configure an indexed/stopped conveyor with speed, direction, encoder/position or indexed-state interface, stations, stop/start interlocks, sensing/pick zones and reject/output destinations using capabilities rather than bespoke scene edits.
- Support fixed upstream/overhead cameras with authored XYZ/RPY, calibrated extrinsics/intrinsics, timestamps and camera-to-world transforms. The camera may be separated from the robot; no eye-in-hand assumption.
- Evaluate ordered, versioned rules using *combined* attributes: colour, supported geometry/dimensions, object class/shape, quality/rejection, barcode or other validated inputs; use sensor/model evidence for material classification and expose UNKNOWN/confidence thresholds. RGB alone must not be assumed to identify material composition reliably.
- Route to designated bins or reject/quarantine without silently substituting unsupported predicates. Track an object from observation through conveyor displacement to its actual pick opportunity; invalidate stale observations.
- Demonstrate repeatable correct routing for a small test set with a documented confusion matrix/rule outcomes and failure handling. Moving-belt pick/intercept is explicitly deferred until accurate timing/encoder-tracking, synchronisation and collision validation are proven.

**Gate C — Random bin picking**

- Real, settled pile of same known part in a physically hollow bin; simulated RGB-D enters EPD or its existing normalized adapter, *not* perfect simulator poses as the planner input.
- Resolve reachable, collision-valid grasp and placement using current AUTO/PREFERRED/EXACT semantics; physically retain the item through motion; release at bound support; allow pile to settle and re-observe.
- Repeat to EMPTY when absence is supported by perception coverage, or NO VALID PICK with a specific evidence-backed blocker. Report successful picks, failed attempts, reasons and recovery.
- Prove at least one change of part profile or bin/destination layout without modifying generic resolver/planner code.

**Gate D — Reuse across equipment**

- At least two robot instances in one configured scene (first independently scheduled, then coordinated shared-zone/handoff only once safe), correct namespaces, TF, controllers, planning groups, collision permissions and per-robot status.
- Demonstrate a supported robot/tool swap or different validated tool modality without rewriting scenario-specific motion logic. Do not label unsupported robot brands/grippers as production-ready.
- Generate identical behaviour from the same authored inputs and trace changes via provenance.

**Gate E — Investor showcase**

- 5–10-minute guided demonstration of the existing Workcell Studio GUI creating or reopening a cell, changing a meaningful parameter, saving, generating, validating, showing RViz planning, and executing the relevant simulator task with operator-visible status.
- Three concise scenario segments, each with a known-good configuration, captured workstation video, logs, a failure path, repeatability evidence and exact backend/perception mode. Prefer a reproducible live demo plus a clearly labelled prerecorded fallback, never pass prerecorded video off as live.
- Include measured setup time, execution time, per-task throughput, success/failure rate with trials and denominator, position/reachability/collision correctness, RTF, CPU/RAM, startup times, recovery outcomes, and assumptions. Show before/after engineering-hours savings **only when measured against a realistic baseline**.
- Investor data room: product overview, architecture and IP/provenance, supported/unsupported capability matrix, demo replay instructions, test evidence, sample-generated customer deliverables, deployment/security/safety limitations, licences/dependencies, customer discovery and pilot/LOI status, cost model, go-to-market hypotheses, budget/milestones and explicit risks.
- Do not promise a specific valuation, multimillion-pound raise, investment return or time-to-market. Treat fundraising as a potential consequence of credible product proof and commercial traction, never as an engineering acceptance criterion.

### Milestone statuses and reporting

Report each item as CONFIRMED, PARTIALLY CONFIRMED, INFERRED, BLOCKED or UNVERIFIED and distinguish: implemented source, static tests, real ROS workstation demo, repeated simulation acceptance and live physical deployment. A milestone cannot be marked complete because unit tests pass or a video looks good.

Legacy labels M0–M12 and R1/R2 remain historical references in the roadmap; this charter is the active release sequence. If asked to reconcile detailed roadmap status, make a narrowly scoped documentation PR rather than deleting history.

### Testing speed and bounded qualification

- **FAST:** focused unit/contract tests without Gazebo when possible (aspirational 1–3 minutes).
- **TARGETED:** select a single gate or no-motion reproducer in the existing supervised runner with required preflight, fresh state, controller ownership, stop verification and cleanup (aspirational 5–15 minutes where feasible).
- **FULL:** existing six-gate complete qualification with unchanged safety checks and traceable acceptance evidence. Reserve for milestone candidates or cross-cutting changes, not every local edit.
- Record measured wall time broken into build/startup/planning/wait/motion/cleanup and simulator RTF. Targets are not verified timings or justification to relax guards.
- For Trial 22-like stalls, prefer bounded stack/callback/GIL and writer-backlog diagnostics first; a targeted PASS is never a FULL PASS.
- Never weaken telemetry freshness, retention, collision or physical-contact checks; no unapproved retry, timeout widening or motion. Honor qualified overlay/file hashes and restore clean owned processes.

## Source-of-truth ownership

Do not allow multiple files to silently compete as the authority for the same state.

### `environment.yaml`

Role:

- authored physical scene/environment definition.

Authority:

- canonical for physical assets, stable IDs, placements, and editable environment state unless the repository has an explicitly newer canonical schema.

### `layout/workcell_studio_layout.yaml`

Role:

- authored editor-state layer.

Authority:

- authoritative only for editor-specific layout/transform metadata assigned to it;
- must not silently override runtime task or planning semantics.

### `task_intent.yaml` or the repository's canonical task-intent file

Role:

- authored statement of what the workcell should achieve.

Authority:

- canonical task goal and destination semantics;
- not the low-level execution trace.

### `task_recipe.yaml` or scene-local task recipe

Role:

- executable or semi-authored recipe.

Authority:

- derived from task intent and capability mappings unless explicitly designed as an authored layer;
- never the sole source of mission truth.

### `cell_definition.yaml`

Role:

- generated runtime handoff.

Authority:

- canonical generated exchange model consumed by downstream generation/validation/runtime stages;
- do not hand-edit around generator defects unless the repository explicitly supports that workflow.

### `scene_manifest.yaml`

Role:

- generated scene inventory, package index, and provenance summary.

Authority:

- derived contract report, not a competing hand-authored source.

### Web3D/Product View exported payloads

Role:

- cached UI/runtime view payloads.

Authority:

- never authoritative;
- regenerate from authored state.

### UI edit patches

Role:

- transient deltas.

Authority:

- apply into the correct authored source, verify persistence, then treat the patch as non-authoritative cache.

### Generated URDF/SRDF, launch files, and runtime plans

Role:

- generated runtime artifacts.

Authority:

- derived from the canonical handoff and asset packages;
- fix the generator or source contract instead of creating one-off generated-file patches.

### Perception adapter config

Role:

- integration configuration.

Authority:

- EPD-owned where it controls perception behavior;
- Workcell Studio may generate or reference scene-local bindings without duplicating perception ownership.

Before editing scene state, identify:

1. the authored source;
2. the generated handoff;
3. the cached view payload;
4. the runtime artifact;
5. any compatibility mirror.

Never write business-critical state to a mirror or cache merely because it is the easiest file to modify.

## Canonical paths and mirrors

The repository has historically contained asset/scene compatibility mirrors and workspace-exposure helpers.

Rules:

- confirm the authoritative repository path from current code and README before editing;
- treat helper-created workspace links/copies as build-discovery mechanisms, not new authorities;
- do not make users reason about mirror paths in the normal Workcell Builder flow;
- do not save successfully to one tree while runtime resolves a different tree;
- when path ambiguity is detected, fail clearly with the candidate paths and selected authority.

## Flagship scenario and shared industrial capability contracts

Exactly **three flagship release templates** are in scope: controlled pick-and-place, indexed-conveyor sorting/inspection, and random bin picking. Industrial capabilities are shared composable modules across them. Do not write three parallel planning/execution stacks or introduce generic workflow engines without proving a missing capability.

### Configurable physical equipment and sensors

- Robots: one or multiple configurable arms; robot assets, bases, joints, namespaces, planning groups, hardware interfaces, frame trees, reach envelopes and controller ownership. Start with verified UR configurations; other brands are adapters requiring evidence.
- End effectors: 2F and suction first, then others only if validated. Document TCP, gripper mode, grasp/contact geometry, force/aperture/vacuum/attachment model, capability limits and object compatibility.
- Conveyors: belt/roller geometry, origin/axis, working length, width, height, speed and direction, stationary/indexed mode, encoder/drive/PLC signals, pick and reject zones, object tracking and safe stop/clear/fault semantics. Continuous-motion interception is a separate, later capability with demonstrated timing.
- Cameras: any number of fixed upstream/overhead/side cameras and robot-mounted cameras, all with own physical pose, calibration, camera ID, frame, optical intrinsics, depth availability, timestamp and confidence/quality. Multiple calibrated observations must not become double picks of the same tracked item.
- Sensors: photoeyes, encoders, scales, barcode readers, proximity and material sensors as supported; define data type, units, uncertainty, update frequency, time basis, signal/fault interpretation and simulation adapter. Never pretend an RGB classifier proves physical material.
- Environment: bins, tray grids, fixtures, pick/drop zones, obstacles, robot bases, camera mounts, collision models, physical support/destination identities, authorised contact zones and conveyor sections.
- Destinations and job policy: route priorities, pass/fail/unknown paths, capacity/occupancy, reject/quarantine, timing window, task sequence, operator controls, stop reason and bounded recovery.

### Multi-robot architecture and acceptance

- A cell may contain more than one robot, each with unique identity/namespace, TF frames, controller interfaces, tool config, task ownership, E-stop/interlock state and time-stamped event trace.
- First prove two robots working independently in one generated cell with no namespace, topic, planning-scene or task cross-talk.
- Next coordinate shared regions using explicit zone reservations and arbitration, deadlock/failure handling, collision constraints and safe state transitions. Robot-to-robot handoffs must be separately accepted for grasp transfer/contact.
- Shared planning across robots is not automatically supplied by a multi-robot RViz picture. Use MoveIt multi-group planning only when current adapter supports it and measured acceptance proves synchronized execution; otherwise coordinate independently planned robots through explicit handshakes.
- One robot's fault, missing telemetry or unverified stop must not permit the other to enter its reserved hazard zone. Never claim functional-safety-rated coordination from ROS messaging alone.
- Where multi-robot support requires a canonical schema extension, evolve the current scene/cell contracts with migrations and tests; do not replace TaskIntent, the resolver, MoveIt or Workcell Builder.

### Task logic / recipe usability

- Keep task creation understandable in the existing Builder: what object, what source, what predicate(s), which actuator, what destination, what to do on UNKNOWN/fault, and who may enter a shared zone.
- Use compatible capability profiles to generate scene-local task, robot, sensor and I/O bindings. Preserve stable authored identities on save/reopen/regeneration.
- Task rule ordering, units (mm vs m, RGB vs HSV etc.), thresholds, priority, conflict detection, simulation playback and audit trail must be explicit.
- Workcell Studio orchestrates; RViz visualises; MoveIt plans; Gazebo supplies optional task-required contact/physics; EPD detects/localises. **Do not duplicate RViz as a robot planner or Gazebo as a physics engine inside the Builder.** Existing 3D canvas is for physical layout editing and intelligible previews only.

## Investor readiness and commercial discipline

Workcell Studio's commercial north star is **shorter engineering time from cell definition to a credible simulated commissioning package**, proven across the three industrial scenarios. An attractive GUI alone is insufficient.

- Prioritise a polished, stable, reproducible demonstration over more features or another large validator layer. Build assets for trustworthy screenshots/video, investor room and technical due diligence.
- Product narrative: problem (costly specialist setup/integration), solution (one configurable authoring and task-execution workflow), proof (three working scenarios, multi-robot configurability, external sensing, measured time savings), customers (integrators/machine builders/automation teams), moat hypothesis (reuse/integration/workflow data), commercial model (internal deployments initially; optional software licensing later).
- Track pilot targets, discovery interviews, letters of intent only if real, actual engineering time before/after, delivered cell/project economics, support burden, integrations and repeat deployment time. Do not invent users, revenue, customer references, market shares, valuation or fundraising commitments.
- Keep robot/third-party model and code licences, contribution history, proprietary asset rights, data/privacy handling, external service dependencies and supply-chain/security risks reviewable. Identify anything that would prevent commercial use before promising it.
- Investor showcase ≠ safety certification or factory deployment. Show real-hardware locked; keep separate factory acceptance, electrical/PLC/safety engineering, cyber-security, maintenance and liability plans for a future physical deployment.
- After first credible investor demo, seek feedback from real manufacturers/integrators and prioritize their repeatable pain points over speculative feature breadth. A million-pound raise is an aspiration, not a success claim or requirement.


## Workcell Builder rules

Improve the existing Workcell Builder rather than creating a parallel product.

The main UI should keep these concepts visible:

- scene/library selector;
- active scene and evidence-backed readiness state;
- 3D canvas;
- scene hierarchy;
- inspector;
- Save;
- Generate/Refresh;
- Validate;
- Plan/Simulate;
- logs/evidence;
- fake-hardware/real-execution lock state.

Contextual controls may include:

- tool/TCP settings;
- destination bindings;
- task-area metadata;
- perception source;
- diagnostic overlays.

Hide under advanced/developer surfaces:

- compatibility mirror repair;
- source-overlay debugging;
- optional simulator backends;
- synthetic test utilities;
- migration tooling.

UI rules:

- no silent no-op buttons;
- no placeholder actions in the primary flow;
- every disabled action explains the blocker;
- errors name the failed action, responsible file/command, and next corrective step;
- secondary actions belong in menus, dropdowns, or advanced panels;
- do not add more always-visible buttons unless essential to the north-star workflow.

## 3D editing and Product View rules

The 3D surface is an **authoring and layout editor**, not a replacement for RViz, MoveIt or Gazebo. Preserve useful existing UI work; do not rebuild path planning, physics, interactive robot controls or advanced RViz features inside Workcell Builder. The 3D surface must be a trustworthy editor, not only a preview.

Required behavior:

- mesh-backed visuals where assets exist;
- primitive fallback only where a mesh is unavailable or intentionally disabled;
- reliable camera controls;
- stable selection/picking of the intended item;
- inspector-based XYZ/RPY editing;
- deterministic transform round-trip;
- correct undo/redo;
- scene hierarchy ↔ 3D selection ↔ inspector synchronization;
- clear distinction between editable authored items and locked generated previews.

Near-term model:

```text
Editable physical layout item = select, move, rotate, inspect, save.
Generated robot/tool/URDF preview = locked, provenance-backed visual.
Task overlay = semantic object, not physical geometry.
Inspector = authoritative manual XYZ/RPY editing surface.
Cached Product View payload = regenerated, never authoritative.
```

Product View should show physical/product content first:

- generated robot visuals;
- tool/gripper meshes;
- editable environment items;
- table/workbench;
- camera body;
- bins, fixtures, conveyors, and other authored physical assets.

Hide by default unless explicitly enabled:

- helper bounds;
- diagnostic labels;
- warning anchors/badges;
- safety, pick, and place zones;
- reachability overlays;
- collision overlays;
- work envelope;
- task route;
- approach/retreat arrows;
- camera FOV;
- pick coverage;
- EPD detections and labels;
- primitive robot fallback boxes when generated robot meshes exist.

When classifying preview content, inspect all relevant identity/provenance fields, not only `role` or `category`:

- `source_layer`;
- `active_visual_source`;
- `role`;
- `category`;
- `id`;
- `display_name`;
- `status`;
- warnings and mesh-load warnings;
- source path and mesh metadata.

No canvas action may silently fail. Selection, transform, save, load, mesh preview, or regeneration failures must identify the responsible state/file.

## Robot, tool, environment, and capability rules

Support must be capability-based, not hardcoded per scene.

Initial practical combinations include:

- UR5 + Robotiq 2F;
- UR5 + suction;
- UR3 + suction;
- UR10 + 2F;
- UR5 + AirPick-style suction where supported;
- simple delta/cartesian placeholder + suction later.

Future support may include Fanuc, ABB, SCARA, gantry/cartesian, delta robots, custom arms, tool changers, and custom end effectors.

Do not add one-off logic that prevents future robot/tool swaps.

Tool behavior, TCP, mount link, grasp frames, orientation defaults, offsets, suction-cup layout, and capabilities should be described in inspectable metadata/config where practical.

Fix generator/capability defaults instead of hand-patching one generated xacro when the defect is systemic.

## Scene support rules

Important scenes may include:

- `ur5_2f_test` — canonical first scene;
- `suction_test` — canonical second scene;
- `ur5_3f_test`;
- `ur3_suction_test`;
- `ur10_2f_test`;
- `ur5_airpick4_test`;
- builder-generated pick/place demos;
- sorting/conveyor scenarios.

Presence does not equal support.

Every scene status should answer:

- What is the intended support level?
- Which authored files exist?
- Which generated files exist?
- Can it save/reopen?
- Can it regenerate deterministically?
- Can it validate?
- Can it build as a ROS 2 package?
- Can it launch in fake hardware?
- Can it simulate the intended task?
- Which acceptance artifacts prove this?
- What exact blocker remains?

Do not hide a broken scene by silently removing it from the catalog. Change its status only with an explicit reason and evidence.

## Generated package and provenance rules

Generated packages must be reproducible and traceable.

A mature scene package should consistently include or reference the repository's canonical equivalents of:

- package metadata (`package.xml`, `CMakeLists.txt` where applicable);
- authored environment/task inputs;
- `cell_definition.yaml`;
- `scene_manifest.yaml`;
- `layout/workcell_studio_layout.yaml` where applicable;
- `launch/demo.launch.py`;
- `urdf/scene.urdf.xacro` and related planning artifacts;
- generated visual/mesh index where used;
- validation/readiness summary.

Generated artifacts should declare:

- schema version;
- generator name/version;
- generator commit SHA;
- source file paths and hashes;
- generation timestamp;
- scene ID;
- relevant asset/capability versions where practical.

Do not stamp misleading provenance. If source hashes or commits cannot be resolved, mark them unavailable rather than fabricating values.

## RViz/MoveIt, grasp planning, and execution rules

RViz/MoveIt with fake hardware is the primary short-term runtime foundation.

A generated scene is not manipulation-ready unless it can prove:

- package discovery succeeds;
- robot model and state are valid;
- robot base frame is correct;
- planning group is valid;
- end-effector/tool frames are correct;
- environment objects are in the planning scene;
- destination/task semantics are available;
- object/task input is available or replayable;
- grasp planner can produce a useful candidate where required;
- approach/grasp/retreat/transfer/place can be planned in simulation;
- simulated execution can complete safely;
- real execution remains locked.

A green build or a loaded RViz window is not proof of a complete task flow.

Failures should expose actionable runtime state, not only stack traces or generic `failed` flags.

## EPD/RealSense bridge rules

Keep replay and live perception as explicit modes using one normalized contract.

Recommended modes:

```text
perception: off
perception: replayed_snapshot
perception: live_epd
```

The normalized result should support, as applicable:

- scene ID;
- camera ID;
- timestamp;
- frame ID;
- object ID or track ID;
- class/label;
- pose or localized centroid;
- dimensions/orientation where available;
- confidence/quality;
- task binding or object-role metadata.

A missing perception runtime should block perception-backed execution clearly. It must not crash Workcell Builder, rewrite authored scene state, or make EPD the task owner.

Simulation should be able to use replayed perception so the north-star task is not dependent on a live camera for every validation run.

## Safety rules

Never weaken safety gates.

Defaults must remain safe:

- fake hardware by default;
- no real robot motion by default;
- no automatic runtime send by default;
- no uncontrolled service/topic publishing;
- real-hardware mode requires explicit guarded flags;
- dry-run/preview-first path remains available;
- validation reports are not safety certificates.

Generated packages may support `use_fake_hardware:=true/false`, but real hardware must never become the default path.

If a change touches launch files, controllers, execution nodes, hardware parameters, services, or runtime publishing:

- verify fake hardware remains the default;
- verify real motion requires explicit opt-in;
- document new preflight blockers;
- do not claim physical readiness from simulation-only evidence.

## ROS and platform rules

Current supported baseline:

- Ubuntu 22.04;
- ROS 2 Humble;
- MoveIt 2 / RViz as the primary planning and visualization foundation.

Do not migrate ROS distributions unless explicitly requested.

Do not make Gazebo, Ignition/Gazebo Sim, or Isaac Sim mandatory for core scene authoring, validation or MoveIt fake-hardware workflows. **Gazebo Fortress (or a separately qualified compatible backend) is nevertheless required for the Stage-A contact-physics/pile acceptance case.** Keep Isaac optional and deferred.

Optional simulation backends must consume the same canonical scene/task contracts rather than creating a second source of truth.

## Validation and acceptance evidence

Always choose the smallest command set that proves the requested change, but distinguish static, GUI, build, launch, and task evidence.

### Static/unit evidence

Use targeted tests for the changed contract. Do not run the entire suite by habit when a focused test is sufficient, and do not add a test-only PR unless it protects a real product fix or catches an escaped defect.

### Workcell Builder evidence

For authoring changes, collect on a real display-enabled workstation where possible:

- scene opened;
- intended physical item selected;
- transform/task edit performed;
- undo/redo checked where relevant;
- save result identified;
- close/reopen persistence checked;
- generated output/diff inspected.

### Build evidence

Confirm the actual workspace path before running ROS commands. Typical layouts are `/home/user/workcell_ws` or `/home/ubuntu/workcell_ws`; Codex containers may only support static inspection.

Canonical pattern:

```bash
cd /home/user/workcell_ws
source /opt/ros/humble/setup.bash
./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --parallel-workers 2
source install/setup.bash
```

Use the repository's current documented script/paths if they differ. Do not blindly copy commands without checking that they exist.

### Fake-hardware launch evidence

Canonical intent:

```bash
ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true
```

If the current launch signature differs, inspect the launch file and use the real supported arguments. Never omit fake-hardware intent when validating the safe path.

### North-star evidence bundle

Success ultimately requires:

1. Workcell Builder opens `ur5_2f_test`.
2. The intended physical bin/environment item is selected.
3. Transform edit and save are recorded.
4. Reopen proves persistence.
5. Generation/refresh artifact contains honest provenance.
6. Humble build log is clean for the required packages.
7. Fake-hardware launch is recorded.
8. Planning scene shows correct robot, tool, environment, frames, and destination semantics.
9. Replay or live EPD input is traced where required.
10. Simulated pick/place completes and returns home.
11. Review/demo bundle is exported.
12. Real execution is visibly locked.

## PR rules

Use small incremental PRs that close a real product gap.

Branch naming:

- use the milestone/intent-oriented branch requested by the current plan when one is explicitly defined;
- otherwise use `codex/<short-task-name>` or another clear scoped prefix;
- do not create a broad umbrella branch for unrelated fixes.

Each PR should:

- target one repository unless a cross-repo interface change is genuinely required;
- state the milestone and user-visible blocker;
- avoid broad refactors;
- preserve ROS 2 Humble compatibility;
- preserve working scenes and safety gates;
- identify authored/generated/runtime files affected;
- include focused tests or validation commands;
- include manual workstation checks when the milestone requires them;
- state commands not run and why;
- include risks and rollback notes;
- avoid generated/minified/binary churn unless essential.

For every validator/test-only PR, prefer at least two product-fix PRs before adding another validator layer, unless a real escaped defect proves the validator is necessary.

PR summary template:

```text
Milestone / blocker:
- roadmap milestone
- user-visible blocker being closed

Summary:
- what changed
- why it matters for Workcell Studio
- affected authored/generated/runtime layers

Validation:
- static/unit commands and results
- GUI/workstation checks and results
- build/launch/task checks and results
- commands not run and exact reason

Evidence:
- screenshots/logs/recordings/artifacts produced
- support status justified by this evidence

Safety:
- fake hardware remains default
- no real-hardware path was enabled accidentally

Risks / rollback:
- known limitations
- safe revert path
```

## Codex/AI task behavior

Before editing:

1. Read `AGENTS.md`, `docs/manuals/WORKCELL_STUDIO_ROADMAP.md`, and only the additional manuals relevant to the scoped change. Do not request the obsolete `Workcell_Studio_AI_Operating_Guardrail.md` path.
2. Inspect the actual repository files that own the state.
3. Inspect recent merged PRs or commits relevant to the blocker when available.
4. Identify authored, generated, cached, mirror, and runtime layers.
5. State the current milestone and acceptance gate.
6. Keep the task scoped to one blocker.

While editing:

1. Fix product code/contracts before changing tests that expose a real mismatch.
2. Prefer deterministic failure with actionable context over silent fallback.
3. Preserve canonical working behavior.
4. Avoid scene-specific hacks when the root cause is generator/capability/path logic.
5. Do not hand-edit derived artifacts as the primary fix.
6. Do not claim success without the evidence required by the milestone.
7. Do not broaden scope merely because nearby cleanup is attractive.

When blocked:

- name the exact missing dependency, file, environment, display, hardware, or runtime condition;
- show the command/result that established the blocker;
- propose the smallest next action;
- do not label the milestone complete.

## User communication rules

Do not automatically provide a Codex prompt.

When the user asks “what next?”, “is this correct?”, “what is going on?”, or “are we going in the right direction?”, answer with:

1. current milestone;
2. what the latest PR actually changed;
3. evidence collected;
4. what remains broken or unproven;
5. the next product action;
6. whether a Codex prompt is needed.

Only write a Codex prompt when the user explicitly asks for one.

A Codex prompt should contain:

- one repository;
- one branch;
- one milestone/blocker;
- exact source-of-truth files to inspect;
- prohibited broad changes;
- acceptance criteria;
- focused tests;
- real-workstation manual validation where required;
- safety statement;
- rollback expectation.

## Next-change selection questions

Before recommending or implementing a change, ask:

1. Does it close a real evidence-backed blocker for one of the **three** flagship scenarios, or improve a shared capability they need?
2. Does it reuse proven Workcell Builder, TaskIntent, resolver, MoveIt, RViz, EPD and simulator contracts instead of duplicating them?
3. Does it measurably reduce the time to get accurate engineering evidence without weakening safety?
4. Does it improve fixed-camera calibration, industrial sorting/conveyor capabilities or multi-robot correctness?
5. Does it bring us closer to a repeatable 5–10 minute **honest investor showcase** or measured customer engineering-time savings?
6. Is the capability actually missing, rather than already delivered by merged R1/R2 code? Is there a simpler targeted test?
7. Does it preserve safe defaults, provenance, single-source-of-truth and exact blockers?

If the answer is no to all seven, defer it. Do not authorize another long full-cycle attempt merely because an isolated test failed. Avoid broad refactors and new simulators/planners unless the existing stack has a demonstrated capability gap.

## Anti-goals

Do not:

- rewrite the whole architecture;
- merge EPD into Workcell Builder;
- combine the EPD GUI with the Workcell Builder UI;
- replace `workcell_builder` with Streamlit or another parallel product;
- broaden beyond the three flagship scenarios before their shared contracts are proven;
- call a scene supported without workstation evidence;
- treat static green checks as end-to-end proof;
- add validators as milestones by themselves;
- create synthetic-only evidence and present it as runtime proof;
- make Isaac or Gazebo mandatory for core Builder/RViz fake-hardware authoring; Gazebo remains task-required only for physics-backed Stage-A acceptance;
- start real robot motion as an early milestone;
- enable real robot motion by default;
- weaken fake-hardware-first safety;
- add primary-UI clutter or placeholder buttons;
- leak compatibility-mirror/path-repair complexity into the normal user flow;
- create scene-specific generated-file hacks;
- hide missing files or broken metadata behind vague success messages;
- store business-critical scene state in cached Product View payloads or transient patches;
- make perception own task planning or scene generation;
- claim simulation/execution works without a trace or acceptance artifact;
- optimize large bundles/minified files while the canonical workflow is blocked;
- use one passing scene as proof that the full platform is stable.

## Definition of done

A change is done only when:

- it closes a specific current blocker in the appropriate authored/generated/runtime layer;
- focused automated tests and relevant workstation/targeted acceptance pass, or the exact blocker is recorded;
- fake-hardware-first, telemetry, contact/retention, FCL/collision, cancellation, recovery and real-hardware locks remain intact;
- provenance, manual commands, timing, limitations, rollback and user-facing behaviour are reviewable;
- no unsupported readiness, investor, safety or market claim was introduced.

**Investor-demo release** is done only when a non-author user can configure and run **three distinct supported scenarios** from the existing Builder, including a fixed off-robot camera, indexed-conveyor rules, a physically simulated random pile, and a demonstrably correct two-robot cell. Each scenario must have reproducible evidence, a meaningful variant that does not require hardcoded generic planner changes, measured reliability, an understandable failure path and an investor-friendly bounded demo. Known unsupported features must be visibly labelled.

**Physical industrial commissioning** is explicitly **not** part of investor-demo acceptance. It remains separately gated by equipment-specific safety design, testing and qualified acceptance.

The release path is:

1. Preserve accepted R1/R2 Builder/TaskIntent/MoveIt capabilities.
2. Make Stage-A physics/repeated-perception reliable and cheap to diagnose.
3. Validate Stage-B real external-camera perception with virtual robot.
4. Add indexed-conveyor sorting and evidence-backed multi-robot composition.
5. Validate three scenarios and produce measured investor showcase/data room.
6. Pursue real pilot feedback and commercial traction; decide on funding/licensing based on evidence.
7. Undertake guarded Stage-C hardware commissioning only when separately authorised.

