# AGENTS.md — Workcell Studio / Easy Manipulation Deployment

## Purpose and authority

This file is the operating contract for AI/Codex work in this repository.

Use it to keep development aligned with the evidence-driven Workcell Studio roadmap. When repository evidence, runtime evidence, and old planning notes disagree, prefer the newest verified repository/runtime evidence and update the documentation rather than preserving an optimistic claim.

### Instruction-document authority

- `AGENTS.md` is the authoritative contributor and AI/Codex operating contract.
- `docs/manuals/WORKCELL_STUDIO_ROADMAP.md` is the current product roadmap; use other manuals only when they are relevant to the scoped change.
- `Workcell_Studio_AI_Operating_Guardrail.md` is not a current or historical tracked path in the available repository refs. It was not renamed to the roadmap. Treat requests to read it as stale references to this `AGENTS.md`, and do not recreate a second operating contract under that name.

Do not measure progress by PR count, test count, or the amount of scaffolding added. Measure progress by user-visible acceptance evidence for the canonical Workcell Studio workflow.

## Project identity

This repository is the primary Workcell Studio / Easy Manipulation Deployment workspace.

Workcell Studio is an internal configurable robotic-cell platform for authoring, generating, validating, simulating, demonstrating, and eventually commissioning industrial robotic cells.

This is not only a sorting demo. Sorting, inspection, machine tending, conveyor picking, binning, palletising, and similar workflows are templates inside Workcell Studio.

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

## Current roadmap position

The immediate sequence is M0 followed directly by M1.

### M0 — Honest executable baseline

Goal:

- define what is authored, generated, cached, and runtime;
- reset optimistic scene-support claims;
- name canonical scenes;
- define acceptance evidence and provenance requirements.

M0 does not mean adding more validators. It means making project claims honest and unambiguous.

### M1 — Complete `ur5_2f_test` authoring round-trip

Goal:

- open the canonical scene;
- select the intended editable physical item;
- move/rotate it;
- update task/destination bindings where applicable;
- undo/redo correctly;
- save;
- close and reopen;
- recover identical transforms and task bindings;
- regenerate from the saved authored state.

M1 is not complete merely because the robot, bin, or mesh is visible. Visual correctness is necessary, but the milestone is the complete edit/save/reopen/regenerate contract.

### Current priority order

When choosing the next change, prefer this order:

1. Repair a regression introduced by the latest merged change when it blocks the canonical workflow.
2. Remove ambiguity between authored, generated, cached, mirror, and runtime state.
3. Make `ur5_2f_test` selection, transforms, task bindings, save, reopen, and regeneration deterministic.
4. Make the generated `ur5_2f_test` package build and launch in fake hardware.
5. Prove a complete simulated 2F pick/place flow with safe return home.
6. Apply the same contract to `suction_test`.
7. Propagate proven contracts to the rest of the scene catalog.
8. Simplify the Workcell Builder user flow.
9. Strengthen deterministic generation and provenance.
10. Harden task/grasp authoring and runtime introspection.
11. Formalize replay/live EPD integration.
12. Produce the demo/review bundle.
13. Add optional Gazebo/Isaac backends only after the core path works.
14. Add guarded physical commissioning last.

Do not let visual polish, broad cleanup, optional simulators, or another validator loop displace the current canonical workflow blocker.

## Milestone ladder

### M0 — Establish an honest executable baseline

Exit gate:

- authored/generated/cached/runtime ownership is documented;
- canonical scenes are named;
- unproven scene support labels are blocked or experimental;
- acceptance evidence requirements are explicit.

### M1 — Make `ur5_2f_test` pass the complete authoring workflow

Exit gate:

- one recorded edit/save/reopen/regenerate round-trip;
- identical transforms and task bindings after reopen;
- no silent save or selection failure.

### M2 — Make `ur5_2f_test` build and launch correctly in fake hardware

Exit gate:

- canonical workspace build succeeds;
- generated package is discoverable;
- RViz/MoveIt fake-hardware launch succeeds;
- robot, tool, environment, destination semantics, and frames are correct;
- launch evidence is captured.

### M3 — Demonstrate one complete simulated 2F pick/place flow

Exit gate:

- approach, grasp, retreat, transfer, place, and safe home complete in simulation;
- operator-visible trace or recording exists;
- no real-hardware mode is enabled.

### M4 — Complete the second canonical suction scene

Exit gate:

- `suction_test` passes the same relevant authoring, generation, build, launch, and simulation checklist.

### M5 — Spread proven contracts across supported scenes

Exit gate:

- every scene labelled supported has attached acceptance evidence;
- unproven scenes remain blocked or experimental with a specific blocker.

### M6 — Simplify and productize Workcell Builder UX

Exit gate:

```text
open scene → edit → save → generate → validate → launch
```

is an obvious novice flow without developer-only clutter.

### M7 — Strengthen deterministic scene/package generation

Exit gate:

- generated artifacts declare schema version, generator name/version, generator commit, source hashes, and generation timestamp;
- authored and derived layers are unambiguous.

### M8 — Harden task/grasp authoring and simulation

Exit gate:

- task intent and executable recipe responsibilities are explicit;
- runtime objective/behavior state is inspectable;
- failures have actionable reasons.

### M9 — Formalize replay/live EPD integration

Exit gate:

- stable adapter schema exists;
- canonical replay evidence exists;
- live mode uses the same normalized runtime contract.

### M10 — Produce customer/investor demo and commissioning bundle

Exit gate:

- reproducible bundle contains screenshots, scene metadata, validation summary, runtime evidence, and visible real-execution lock state.

### M11 — Add optional Gazebo/Isaac backends

Exit gate:

- optional adapters only;
- neither simulator is required by the core authoring/fake-hardware workflow.

### M12 — Add guarded physical commissioning

Exit gate:

- real execution is explicit, opt-in, preflight-guarded, and separately documented;
- normal authoring and simulation cannot accidentally trigger robot motion.

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

The 3D surface must become a trustworthy editor, not only a preview.

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

Do not make Gazebo, Ignition/Gazebo Sim, or Isaac Sim mandatory for the core workflow yet.

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

1. Does it close M0 ambiguity or M1 authoring round-trip risk?
2. Does it make `ur5_2f_test` save/reopen/regenerate correctly?
3. Does it improve the generated runtime handoff or fake-hardware launch?
4. Does it produce missing acceptance evidence?
5. Does it preserve the Easy/EPD boundary?
6. Does it avoid another test/validator-only loop?

If the answer is no to all six, it is probably not the next task.

## Anti-goals

Do not:

- rewrite the whole architecture;
- merge EPD into Workcell Builder;
- combine the EPD GUI with the Workcell Builder UI;
- replace `workcell_builder` with Streamlit or another parallel product;
- broaden the scene catalog before canonical contracts are proven;
- call a scene supported without workstation evidence;
- treat static green checks as end-to-end proof;
- add validators as milestones by themselves;
- create synthetic-only evidence and present it as runtime proof;
- make Isaac or Gazebo mandatory before the core flow is stable;
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

- the requested blocker is closed at the correct source-of-truth layer;
- relevant authored/generated/runtime files are updated consistently;
- focused automated validation passes;
- required manual/workstation evidence is attached or explicitly marked blocked;
- fake-hardware and real-execution safety defaults are preserved;
- user-facing behavior is documented when needed;
- the PR explains what changed, why, evidence, risks, and rollback;
- no unsupported readiness claim was introduced.

For the canonical Workcell Studio path, “done” ultimately means:

```text
honest scene contract
+ deterministic edit/save/reopen
+ repeatable generation with provenance
+ clean ROS 2 Humble build
+ RViz/MoveIt fake-hardware launch
+ complete simulated task/grasp
+ replay/live perception contract
+ reviewable demo evidence
+ guarded real-hardware future
```
