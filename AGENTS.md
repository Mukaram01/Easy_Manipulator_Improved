# AGENTS.md - Workcell Studio / Easy Manipulation Deployment

## 0. Authority and mission

This file is the repository-level operating contract for Codex/AI contributors.

Workcell Studio is an internal configurable industrial robotic-cell platform. The product is not a one-off sorting demo and it is not a collection of validators. The product must let an engineer author a cell, save it, regenerate it, validate it, launch it safely, plan a task, simulate a complete manipulation cycle, consume perception, and produce evidence that the same authored truth survived every boundary.

The primary product UI is `workcell_builder`. Improve it directly. Do not replace it with Streamlit, notebooks, a second engineering GUI, or a demo-only application.

The current product priority is singular:

> **Make one complete industrial cycle work correctly, repeatably, observably, and safely on `scenes/ur5_2f_test` before broadening the platform.**

A change that does not materially advance that full-cycle closure is normally not the next change.

Authoritative planning documents:

- `AGENTS.md` - AI/Codex operating contract.
- `docs/manuals/WORKCELL_STUDIO_ROADMAP.md` - human-readable product roadmap and current execution order.
- `docs/manuals/workcell_studio_master_roadmap.yaml` - machine-readable milestone/gate contract for agents and automation.

When old docs, generated artifacts, source code, and verified runtime evidence disagree, prefer the newest verified repository/runtime evidence and update the docs. Never preserve an optimistic claim merely because it is already written down.

## 1. North-star industrial cycle

### Canonical scene

`scenes/ur5_2f_test`

### Canonical first-cycle hardware/model contract

- Robot: UR5.
- Tool: Robotiq 2F / finger gripper.
- Planning stack: ROS 2 Humble + MoveIt 2 + RViz.
- Default execution mode: fake hardware.
- Camera/perception source: RealSense D435i through normalized EPD input.
- First target class: `bottle` unless the canonical task config is intentionally changed.
- Destination: the scene-authored default place/drop destination.
- Perceived workpiece is runtime data. Do not add authored fake bottles/cups merely to make the cycle look complete.

### The full cycle

```text
open Workcell Builder
-> select ur5_2f_test
-> inspect/edit authored physical scene
-> edit task intent if required
-> Save Layout
-> close/reopen and recover identical authored state
-> Generate Scene Package
-> validate generated package and authored/generated parity
-> build/discover ROS package
-> launch RViz/MoveIt with fake hardware
-> start replay or live perception input
-> accept only the configured target class
-> reject stale/unreachable/collision-invalid/grasp-invalid candidates
-> create/update PlanningScene object
-> generate grasp candidates
-> plan approach
-> move to pre-grasp
-> grasp/close
-> attach object
-> retreat
-> transfer
-> plan/place at authored destination
-> release/detach
-> retreat
-> return home/safe state
-> report success/failure reason and evidence
-> stop runtime cleanly
-> return to the same Workcell Studio session
-> edit/save/run again without restarting the application
```

The scene is not "done" until that cycle passes the acceptance gates below.

## 2. Full-cycle acceptance gates - R1

Every gate must have an explicit PASS/BLOCKED result and evidence. Do not skip a gate because a later layer appears to work.

### G0 - Reproducible workspace baseline

Pass when:

- supported baseline is Ubuntu 22.04 + ROS 2 Humble;
- repository/workspace path is unambiguous;
- required packages are discoverable after the documented build/bootstrap path;
- `xacro`, `ament_index_python`, `workcell_builder`, MoveIt packages, grasp packages, and required scene/asset packages resolve;
- a clean checkout does not rely on undocumented manual file copying.

### G1 - Authored source-of-truth correctness

Pass when:

- physical editable state has one clear authority;
- task intent/destination semantics have one clear authority;
- cache/mirror/generated artifacts are not treated as authored truth;
- stable IDs survive load/save/regeneration;
- the canonical scene contains valid robot/tool/camera/environment/task metadata.

### G2 - Workcell Builder authoring round-trip

Pass when, in the GUI:

- the intended editable item can be selected from Product View and hierarchy;
- inspector, hierarchy, 3D selection, and authored item identity stay synchronized;
- XYZ/RPY editing is deterministic;
- undo/redo is correct;
- Save succeeds visibly;
- close/reopen restores identical authored transforms and task bindings;
- no Product View cache or transient patch is required to recover state.

### G3 - Deterministic generation

Pass when:

- Generate Scene Package consumes canonical authored inputs;
- regenerated outputs are deterministic apart from explicitly allowed provenance fields;
- generated package contains the required ROS/package/launch/URDF/task/runtime contract;
- generation never silently preserves stale derived state;
- generated artifacts record honest provenance/source hashes/version where supported.

### G4 - Authored -> generated -> Product View parity

Pass when:

- physical asset IDs, poses, mesh identities/scales, robot base, tool mount, camera pose, and destination semantics match across authored state and generated runtime state;
- Product View is derived from current canonical state;
- a stale Web3D payload cannot be mistaken for current truth;
- post-generation parity reports zero blockers for the canonical scene.

### G5 - ROS build and package discovery

Pass when:

- the generated/refreshed canonical package builds in the documented Humble workspace;
- package discovery resolves the same scene path the builder authored;
- there is no duplicate/mirror path ambiguity;
- no manual repair of generated files is required after generation.

### G6 - Fake-hardware launch and frame truth

Pass when:

- `ur5_2f_test` launches with fake hardware explicitly enabled;
- robot state is valid;
- base/world/tool/grasp frames are correct;
- planning group and end-effector are correct;
- table/bin/camera/environment appear at the authored transforms;
- planning-scene collision objects match physical scene intent;
- no real robot motion path is enabled by default.

### G7 - Perception ingress parity

Pass in replay mode first, then live mode:

- normalized detected-object input carries timestamp, frame, identity/track, class, pose/localization, and quality fields available from EPD;
- scene/camera/frame binding is explicit;
- transforms into the planning frame are valid and timestamp-aware;
- replay and live use the same downstream contract;
- missing perception blocks perception-backed execution clearly instead of corrupting scene state.

### G8 - Candidate eligibility and selection

Pass when every candidate is evaluated by explicit policy:

- class matches configured target;
- observation is fresh enough;
- pose is transformable to planning frame;
- object is in configured pick region when required;
- object is reachable;
- grasp strategy is compatible with tool/object;
- candidate grasp is collision valid;
- destination is valid;
- non-target classes are never picked merely because they are visible.

The decision and rejection reason for each candidate must be inspectable.

### G9 - Grasp planning correctness

Pass when:

- grasp candidates are generated from tool capability + task intent, not scene-name hacks;
- pre-grasp/grasp/retreat poses are valid;
- TCP/grasp frame offsets are correct;
- collision checking is enabled;
- planner failure exposes actionable reasons;
- at least one valid target can produce a complete planned manipulation sequence.

### G10 - Complete simulated manipulation cycle

Pass when fake-hardware execution completes:

```text
approach -> grasp -> attach -> retreat -> transfer -> place -> release/detach -> retreat -> home
```

Required invariants:

- no unexplained teleport/reset;
- attached object follows the tool correctly;
- place/release uses authored destination semantics;
- collision objects are updated consistently;
- timeout/failure stops safely;
- success is reported only after return-home/safe terminal state.

### G11 - Recovery, retry, and idempotence

Pass when:

- a failed candidate can be rejected and the next candidate evaluated without restarting the stack;
- retries are bounded and reason-coded;
- repeated detections do not create duplicate PlanningScene objects;
- completed/attached objects are not re-selected incorrectly;
- stop/cancel leaves the scene in a known safe state;
- the full cycle can run twice consecutively in one session.

### G12 - Workcell Studio lifecycle closure

Pass when:

- closing/stopping RViz or task execution does not kill Workcell Builder;
- Workcell Builder reconnects to the same scene state;
- generation/readiness status refreshes correctly;
- another edit/save/generate/simulate cycle works without application restart;
- background Product View processes/ports are owned and cleaned up correctly.

### G13 - Evidence bundle and support claim

Pass when the canonical run emits or records:

- authored input hashes/identity;
- generation/validation report;
- parity report;
- build command/result;
- fake-hardware launch transcript;
- planning/task trace;
- candidate accept/reject reasons;
- final task outcome;
- screenshots or recording of Workcell Builder + RViz/MoveIt;
- explicit statement that real execution remained locked.

Only after G0-G13 pass may `ur5_2f_test` be treated as a proven canonical full-cycle scene.

## 3. Perfection bar - non-functional requirements

"Works once" is not enough. R1 must also satisfy:

### Determinism

- same authored inputs produce the same semantic generated outputs;
- stable IDs do not drift;
- generation order does not change behavior;
- no mtime-only truth for readiness when content fingerprints are available.

### Observability

Every major state transition exposes:

- state;
- reason;
- relevant scene/object ID;
- source file/config where applicable;
- next corrective action on failure.

No generic `failed` message where a precise reason can be produced.

### Idempotence

- Save can be repeated safely;
- Generate can be repeated safely;
- perception updates do not duplicate world objects;
- start/stop/start does not accumulate stale processes or ports.

### Fail-safe behavior

- fake hardware first;
- no uncontrolled motion/publishing;
- invalid transforms, frames, task bindings, collisions, stale perception, or missing controllers block execution;
- no fallback that silently weakens collision or safety semantics.

### Product clarity

The novice happy path should remain obvious:

```text
Open/Create -> Edit -> Save -> Generate -> Validate -> Plan/Simulate -> Review evidence
```

Developer diagnostics are secondary. Disabled actions explain why they are disabled.

### Architecture quality

Prefer reusable capability/contract fixes over scene-specific branches. A canonical-scene exception is acceptable only when the underlying requirement is truly scene-specific and documented.

## 4. Source-of-truth ownership

Before editing, identify all five layers:

1. authored source;
2. generated handoff;
3. cached Product View/UI payload;
4. runtime artifact/state;
5. compatibility mirror/workspace exposure path.

Rules:

### `environment.yaml`

Canonical authored physical/runtime scene semantics where currently used: physical assets, stable IDs, robot/tool/camera/environment pose/state, task/environment metadata.

### `layout/workcell_studio_layout.yaml`

Canonical editor-state/layout layer for editable visual layout metadata. It must round-trip to the authoritative physical scene through the supported save path. It must never silently become a competing runtime truth.

### task intent file

Canonical authored statement of what the cell should do: target class, pick source, destination, grasp intent, constraints.

### task recipe

Executable/derived runtime recipe. Prefer deriving it from task intent + robot/tool capabilities rather than hand-maintaining a second mission truth.

### `cell_definition.yaml`

Generated exchange/handoff model. Fix its generator/source contract rather than hand-editing it around defects.

### `scene_manifest.yaml`

Generated inventory/provenance/readiness contract. It reports state; it is not a competing authored source.

### Product View/Web3D payloads and transient UI patches

Caches/transient deltas only. Never store business-critical state solely here.

### generated URDF/SRDF/launch/runtime files

Derived runtime artifacts. Fix source/generator logic rather than patching them manually as the primary solution.

If path or authority is ambiguous, fail loudly and name the candidate paths. Do not save to one tree while runtime resolves another.

## 5. Architecture boundary

`Easy_Manipulator_Improved` owns:

- Workcell Studio product shell and `workcell_builder`;
- cell/scene/environment/task authoring;
- capability-aware package generation;
- validation/readiness/parity;
- MoveIt/RViz fake-hardware orchestration;
- grasp-planner/execution integration;
- normalized perception consumption;
- operator/review/commissioning artifacts.

`epd_Improved` owns:

- RealSense acquisition;
- detection/localization/tracking/classification;
- perception algorithms and perception-specific GUI/configuration.

Boundary:

```text
EPD produces normalized perception results.
Workcell Studio consumes them.
EPD does not own cell definition, scene generation, task intent, planning, or Workcell Studio UI state.
```

Do not merge EPD into Workcell Builder.

## 6. Workcell Builder and Product View rules

Primary UI concepts:

- active scene;
- Product View/3D canvas;
- scene hierarchy;
- inspector;
- task intent;
- Checks/readiness;
- Save;
- Generate;
- Validate;
- Plan/Simulate;
- logs/evidence;
- visible fake-hardware/real-execution lock state.

Required behavior:

- no silent no-op buttons;
- every disabled action has a visible blocker;
- stable scene/item selection across refresh;
- inspector <-> hierarchy <-> Product View identity parity;
- editable authored items are editable;
- generated robot/tool/URDF visuals are locked unless an explicit authored control owns them;
- mesh-backed visuals where available;
- primitive fallback only when honest and necessary;
- selection, transform, save, generation, preview, and reload failures name the responsible state/file/action;
- Product View cache is refreshed from canonical state, never treated as authority.

Hide developer-only complexity from the default user path: mirror repair, raw provenance internals, synthetic fixtures, parser traces, optional simulator backends.

## 7. Robot/tool/grasp rules

Support is capability-based, not scene-name based.

The first proven contract is UR5 + Robotiq 2F. The second is UR5 + suction. Broader robot/tool support comes after the canonical contracts are closed.

Tool/capability metadata should own, where applicable:

- mount link;
- TCP/grasp frame;
- mount transform;
- TCP offset;
- grasp methods;
- allowed touch links;
- opening/width/cup capabilities;
- approach/orientation defaults;
- actuator/runtime IO requirements.

Grasp logic should consume capability + task + perception/object state. Do not hardcode a valid grasp merely to satisfy the demo.

## 8. Perception contract rules

Supported modes should converge on:

```text
perception: off
perception: replayed_snapshot
perception: live_epd
```

Replay and live must feed the same normalized downstream object contract.

At minimum support, where available:

- scene ID;
- camera ID;
- timestamp;
- source frame;
- object/track ID;
- class/label;
- localized pose/centroid;
- dimensions/orientation;
- confidence/quality;
- task/object role metadata.

Perception data is runtime state, not authored geometry.

## 9. Safety law

Never weaken safety gates to make a test pass.

Defaults:

- fake hardware by default;
- no real robot motion by default;
- no automatic runtime send;
- no uncontrolled service/topic publishing;
- explicit dry-run/preview path;
- explicit real-hardware opt-in only after separate commissioning milestones;
- validation reports are not safety certificates.

If a change touches launch, controllers, execution, hardware parameters, services, or motion publishing, explicitly verify the fake-hardware default and the real-motion lock.

## 10. Codex / Astra Ultra execution protocol

The agent is expected to behave like a senior robotics/platform engineer, not a patch generator.

### Before editing

1. Read this file and `docs/manuals/WORKCELL_STUDIO_ROADMAP.md`.
2. Read `docs/manuals/workcell_studio_master_roadmap.yaml` when present.
3. Inspect the exact source-of-truth files and the relevant recent commits/PRs.
4. Reproduce the blocker with the smallest meaningful command or GUI flow.
5. Name the current gate (G0-G13) and the concrete acceptance criterion being closed.
6. Identify whether evidence can be collected in the current environment or requires the real ROS/GUI workstation.

### While editing

1. Fix the root contract, not the symptom.
2. Prefer the smallest robust change that removes the class of defect.
3. Preserve working canonical behavior.
4. Do not hand-edit derived artifacts as the primary fix.
5. Do not add a new fallback that hides invalid state.
6. Do not broaden into unrelated cleanup.
7. Add focused regression coverage for the real defect.
8. Run the relevant tests/commands immediately after the fix.
9. If another failure appears in the same gate and is caused by the same root contract, continue until the gate closes or an external blocker is proven.
10. Stop only at a real external dependency/hardware/display blocker, not at the first inconvenient error.

### When blocked

Report:

- exact command/action;
- exact failure;
- responsible layer/file;
- evidence gathered;
- why the current environment cannot complete the gate;
- smallest real-workstation action required next.

Never mark the gate complete.

### Completion output

Every substantial task should end with:

- gate/milestone closed;
- files changed;
- root cause;
- automated validation run and result;
- manual validation still required;
- safety statement;
- risks/rollback;
- next highest-value gate.

## 11. Anti-loop and anti-bloat rules

Do not:

- add validators as milestones by themselves;
- write test-only PRs unless protecting a real fix or catching an escaped defect;
- polish Scene3D metrics while the canonical cycle is broken;
- create synthetic-only success evidence;
- call a scene supported without workstation evidence;
- add scene-specific hacks for a systemic generator/path/capability defect;
- perform broad architecture rewrites before R1 closure;
- make Gazebo/Isaac mandatory;
- merge EPD and Workcell Builder;
- add a third engineering source of truth;
- hide broken metadata behind permissive fallbacks;
- weaken collision checking/ACM/safety merely to make motion succeed;
- introduce real-hardware motion into the normal authoring/simulation path;
- optimize PR count, test count, or code volume instead of user-visible acceptance.

A useful heuristic: for every test/validator-only change, prefer at least two concrete product-fix changes unless the test catches a newly escaped defect.

## 12. PR and branch rules

Prefer small milestone-oriented PRs.

Branch naming:

- `fix/<gate>-<short-root-cause>` for defects;
- `feat/<gate>-<short-capability>` for scoped capabilities;
- `docs/<topic>` for roadmap/documentation-only work;
- `codex/<short-task-name>` when no stronger convention exists.

Each PR must state:

- R1 gate/milestone;
- user-visible blocker;
- source-of-truth layer affected;
- root cause;
- implementation summary;
- focused tests/commands and results;
- manual workstation checks and status;
- evidence artifacts;
- fake-hardware/real-motion safety status;
- risk and rollback.

Do not combine unrelated gates in one PR unless the same root cause spans them and splitting would make the fix less correct.

## 13. Definition of done

A code change is done when the requested gate is closed at the correct layer, relevant tests pass, required manual evidence is either attached or explicitly blocked, and safety defaults remain intact.

R1 is done only when the north-star cycle is proven end to end and repeatably:

```text
author -> save -> reopen -> generate -> parity -> build -> fake launch
-> perception -> select -> grasp plan -> approach -> grasp/attach
-> transfer -> place/release -> retreat -> home -> report -> stop
-> return to builder -> edit/save/run again
```

Required confidence bar:

- cold-start pass;
- second consecutive pass without restart;
- close/reopen persistence pass;
- at least one intentional failure/rejection path reports the correct reason and recovers safely;
- evidence bundle attached;
- real hardware remains locked.

Only after this bar is met should roadmap priority move from "make one full cycle perfect" to "generalize the proven contract across scenes, tools, and use cases."
