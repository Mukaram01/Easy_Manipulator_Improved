# Workcell Studio Master Roadmap

Last reset: 2026-09-13  
Roadmap baseline: `integration/workcell-studio-r15` at `59004aaa` (`R1.5: consolidate runtime selection retry destination and EPD bridge`)

## Executive objective

Workcell Studio now has enough architecture, UI, generation, Product View, scene contracts, validation, fake-hardware plumbing, perception adapters, grasp planning/execution code, and runtime retry work that the next step must not be another broad feature wave.

The next step is to **close one complete industrial cycle at production-quality engineering standards**.

The canonical target is:

```text
Workcell Builder
-> ur5_2f_test
-> edit/save/reopen
-> generate/validate/parity
-> ROS 2 Humble build
-> RViz/MoveIt fake hardware
-> replay perception first, then live D435i/EPD
-> class-aware target selection
-> reachability/collision/grasp validation
-> approach/grasp/attach
-> retreat/transfer
-> place/release/detach
-> retreat/home
-> deterministic result + evidence
-> stop cleanly
-> return to Workcell Builder
-> edit/save/run again without restarting
```

This is the **R1 Full-Cycle Closure** milestone. Until it passes, new scene breadth, major UI redesign, Gazebo, Isaac, real hardware, and broad cleanup are secondary.

## Why this is the right point to focus

The repository already contains the major product layers:

- `workcell_builder` as the engineering/product shell;
- embedded Product View / Web3D and native compatibility paths;
- canonical scene/layout/task metadata;
- generated ROS 2 scene packages;
- fake-hardware-first launch paths;
- MoveIt/RViz planning integration;
- perception capture/normalization utilities;
- perceived-object grasp planning/execution utilities;
- destination-aware placement logic;
- candidate filtering/retry work;
- strict EPD runtime conversion work;
- acceptance and parity tooling.

The current integration head specifically advances runtime selection, retry, destination intent, and the EPD bridge. The remaining risk is not lack of components; it is **whether all components share one source of truth and survive the complete lifecycle without manual repair**.

## Product laws

These laws override local convenience.

1. **One source of truth per state.** Authored, generated, cached, runtime, and compatibility-mirror state must never silently compete.
2. **Fake hardware first.** Real motion is not part of R1.
3. **No silent fallback.** Missing frames, transforms, assets, task bindings, collision state, or perception quality must fail with an actionable reason.
4. **No scene-name hacks for systemic behavior.** Robot/tool/task behavior comes from capabilities and scene/task contracts.
5. **Runtime workpieces come from perception.** Do not author fake target objects into the scene to make the cycle look successful.
6. **A visual preview is not runtime proof.** Product View, generated URDF, MoveIt planning state, and executed fake-hardware behavior must agree.
7. **A green unit test is not milestone closure.** Runtime/GUI evidence is required where the gate is inherently runtime/GUI.
8. **No validator treadmill.** Add checks only when they protect a real product contract or catch an escaped defect.
9. **No manual patching after Generate.** If a generated artifact must be hand-edited, the generator/source contract is still broken.
10. **Repeatability is part of correctness.** The full cycle must work again without restarting the application or repairing state by hand.

# R1 - One complete industrial cycle

## R1 acceptance scene

Primary scene: `scenes/ur5_2f_test`

Primary stack:

- UR5;
- Robotiq 2F;
- ROS 2 Humble;
- MoveIt 2 / RViz;
- fake hardware;
- RealSense D435i + EPD for live acceptance;
- replayed `detected_objects/v1` for deterministic regression acceptance.

### Target-class contract

The repository currently has more than one task-class signal: the scene metadata has historically used `bottle`, while the current R1 runtime acceptance config (`config/runtime/r1_4_task.yaml`) selects `cup` and the retry replay contains two cups plus a bottle distractor.

R1 must deliberately converge this to **one acceptance target class**. The recommended acceptance path is `cup` because the current retry fixture already proves positive targets plus a non-target distractor. If maintainers choose `bottle` instead, all authored task intent, runtime config, replay data, and evidence must be changed together. A hidden `cup`/`bottle` split is a blocker, not a compatibility feature.

## R1 gate map

| Gate | Name | What must become true | Evidence required |
| --- | --- | --- | --- |
| G0 | Workspace determinism | Correct repo/workspace/package/asset discovery from documented Humble bootstrap | bootstrap/build transcript |
| G1 | Authored truth | scene, layout, task, destination, robot/tool/camera semantics have unambiguous owners | source-of-truth audit + file diff |
| G2 | Builder round-trip | edit -> undo/redo -> save -> close/reopen restores identical authored state | GUI recording + YAML diff |
| G3 | Generation determinism | Generate consumes current authored state and emits a complete package with no stale carry-over | generator transcript + artifact hashes |
| G4 | Parity | authored -> generated -> Product View physical state agrees | parity report with zero blockers |
| G5 | ROS build/discovery | generated/refreshed scene is discoverable and builds without manual repair | colcon log |
| G6 | Fake-hardware launch | MoveIt/RViz scene is correct and safe | launch transcript + screenshot/recording |
| G7 | Perception ingress | replay/live normalized objects arrive in world/planning frame using one contract | object snapshot + transform diagnostics |
| G8 | Candidate policy | only eligible configured targets survive class/freshness/zone/reach/collision/grasp gates | per-candidate decision trace |
| G9 | Grasp plan | valid pregrasp/grasp/retreat/place plan is produced from capability + task + object state | planner trace |
| G10 | Full execution | approach -> grasp -> attach -> transfer -> place -> release -> home completes | execution trace + RViz recording |
| G11 | Recovery/retry | blocked first target is rejected; next valid target can succeed; no duplicate objects/state leaks | retry trace |
| G12 | Lifecycle closure | stop runtime/RViz, return to same builder, edit/save/run again without restart | second-cycle recording |
| G13 | Evidence bundle | all relevant inputs/results/provenance captured; real motion visibly locked | exported acceptance bundle |

R1 is complete only when **G0-G13 PASS**.

# Detailed gate specification

## G0 - Workspace and dependency determinism

### Goal

A clean supported workstation can reproduce the environment without hidden local state.

### Required checks

- confirm actual workspace root before commands;
- source `/opt/ros/humble/setup.bash`;
- run the repository's current workspace-layout exposure helper only where required;
- `rosdep` resolves required dependencies;
- `colcon` can build the required packages;
- `xacro` is discoverable;
- `ament_index_python` imports;
- `workcell_builder` resolves from the intended overlay;
- scene and asset packages resolve from the intended repository/workspace tree;
- no duplicate scene tree wins silently because of sourcing order.

### Exit gate

One documented bootstrap/build path works on the real workstation from a clean state.

## G1 - Source-of-truth convergence

### Goal

Every important value has one authoritative owner.

### Must resolve explicitly

- physical object pose and dimensions;
- robot base pose;
- tool mount/TCP/grasp frame;
- camera pose/frame;
- pick zone;
- destination/place zone;
- target class;
- grasp strategy;
- perception mode/binding;
- generated handoff provenance.

### Required ownership model

- `environment.yaml`: canonical physical/runtime scene semantics where currently defined;
- `layout/workcell_studio_layout.yaml`: editor-state/layout authoring layer;
- task-intent file: authored objective;
- task recipe/runtime task config: derived/executable behavior, not a second secret mission truth;
- `cell_definition.yaml`: generated exchange/handoff;
- `scene_manifest.yaml`: generated inventory/provenance/readiness report;
- Product View JSON: cache/view payload only;
- generated URDF/SRDF/launch: derived runtime output.

### Exit gate

No R1 value depends on contradictory copies. The target-class split is removed deliberately.

## G2 - Builder authoring round-trip

### Goal

`workcell_builder` behaves like a trustworthy engineering authoring tool.

### Required user flow

1. Open `ur5_2f_test`.
2. Select the intended editable environment item in Product View.
3. Confirm hierarchy and inspector select the same stable ID.
4. Move and rotate it.
5. Modify a relevant task/destination binding if required.
6. Undo.
7. Redo.
8. Save Layout.
9. Close the scene or application.
10. Reopen.
11. Confirm identical pose/task state.
12. Confirm generated/locked visuals were not accidentally authored.

### Product-quality rules

- no silent save;
- no transient browser patch as sole persistence;
- no duplicate robot/camera/environment visual owner;
- no selection-ID drift after refresh;
- mesh-derived dimensions are not accidentally rewritten as arbitrary authored primitive dimensions;
- disabled controls explain their blocker.

### Exit gate

Cold reopen proves byte/semantic-equivalent authored state for the tested edit.

## G3 - Deterministic package generation

### Goal

The saved scene becomes a complete runtime handoff with one action.

### Required behavior

Generate Scene Package must:

- validate the current authored inputs;
- run any required canonical layout merge deliberately;
- regenerate the scene package;
- update provenance/fingerprints;
- reject ambiguous or malformed authored state;
- never depend on an ignored cache file for truth;
- never silently keep a stale output because it already exists.

### Required package contract

At minimum, the canonical equivalents of:

- `package.xml`;
- `CMakeLists.txt` where required;
- `environment.yaml`;
- `cell_definition.yaml`;
- `scene_manifest.yaml`;
- `layout/workcell_studio_layout.yaml`;
- task intent/recipe/runtime config;
- `launch/demo.launch.py`;
- `urdf/scene.urdf.xacro`;
- generated visual/index/readiness artifacts used by the product.

### Determinism check

Run generation twice from unchanged authored inputs. Semantic outputs must remain equivalent apart from explicitly allowed generation metadata such as timestamps.

### Exit gate

No manual generated-file edit is needed before G4/G5.

## G4 - Physical/parity truth

### Goal

Workcell Builder, generated artifacts, and Product View represent the same physical cell.

### Compare

- stable asset IDs;
- object poses;
- robot base transform;
- tool mount/TCP;
- camera pose;
- mesh URI/identity;
- mesh local transform/scale;
- place-zone/destination semantics;
- generated robot visual transform chain.

### Critical lifecycle rule

Post-generation parity must validate against the Product View payload that corresponds to the newly generated canonical state. A previously published Web3D payload must never make a current generation fail or pass incorrectly.

### Exit gate

Post-generation parity: zero blockers and zero unexplained transform/mesh mismatches.

## G5 - ROS build and package discovery

### Goal

The generated/refreshed scene is a real ROS package, not merely a valid YAML bundle.

### Acceptance

- package discovery resolves the intended `ur5_2f_test` path;
- build succeeds in the supported workspace;
- no missing asset packages;
- no source/install overlay mismatch;
- no manual copy/link repair beyond the documented bootstrap step;
- launch files import and resolve dependencies.

### Exit gate

Repeatable `colcon` build with captured transcript.

## G6 - Fake-hardware RViz/MoveIt truth

### Goal

The canonical generated runtime loads correctly and safely.

### Verify visually and programmatically

- robot state valid;
- correct UR5 model;
- correct 2F tool;
- correct planning group;
- correct world/base/tool/grasp frames;
- authored table/bin/camera positions;
- planning-scene collision objects;
- destination zone alignment;
- no duplicate physical geometry;
- fake hardware explicitly on;
- real execution off.

### Exit gate

RViz/MoveIt recording plus launch transcript with no critical frame/controller/planning errors.

## G7 - Perception ingress: replay first, live second

### Replay acceptance

Use a deterministic `detected_objects/v1` payload that contains:

- at least two configured-target candidates;
- one intentionally invalid/blocked target candidate;
- one non-target distractor;
- world-frame poses or a valid transform path;
- stable object IDs;
- realistic dimensions.

The current R1 retry replay already expresses this pattern.

### Live EPD acceptance

Live mode must preserve:

- source ROS observation timestamp;
- source frame;
- EPD identity/track identity policy;
- real dimensions/pose;
- explicit policy when confidence is not supplied by the EPD message;
- transform to planning frame using observation time where required.

### Exit gate

Replay and live feed the same downstream normalized object contract.

## G8 - Candidate policy engine

### Goal

Selection is explainable and safe.

For each perceived object, record decisions for:

1. target-class match;
2. confidence policy;
3. freshness;
4. transform validity;
5. source-zone membership if configured;
6. physical gripper aperture/shape compatibility;
7. reachability;
8. collision feasibility;
9. grasp-candidate validity;
10. destination validity;
11. retry/exclusion history.

### Required R1 behavior

The highest-ranked candidate may intentionally fail because it is obstructed. The runtime must reject it for the correct reason and try the next eligible candidate without touching the non-target distractor.

### Exit gate

Per-candidate machine-readable decision trace exists and the correct candidate wins.

## G9 - Grasp-plan integrity

### Goal

Turn a selected runtime object into a collision-aware manipulation plan.

### Required checks

- object dimensions fit 2F capability;
- grasp frame/TCP offsets are sourced from tool/cell metadata;
- top/side/orientation policy follows task/grasp config;
- approach distance and retreat are honored;
- candidate rotations are bounded and deterministic;
- collision check includes scene objects and target handling policy;
- place target comes from authored/generated destination contract;
- planner errors identify the failed stage and candidate.

### Exit gate

At least one valid candidate produces a complete pregrasp -> grasp -> retreat -> transfer -> place -> retreat trajectory plan.

## G10 - Full fake-hardware execution

### Goal

Complete a believable industrial pick/place cycle.

### Required sequence

```text
HOME/READY
-> selected object published/registered
-> PlanningScene object present
-> pre-grasp plan/execute
-> grasp plan/execute
-> gripper close
-> attach object
-> retreat plan/execute
-> transfer/place approach plan/execute
-> release/open
-> detach/remove/update object state
-> retreat
-> home/safe return
-> terminal SUCCESS
```

### Invariants

- object does not teleport independently of the attached tool;
- attached object is not simultaneously treated as a free-world collision object incorrectly;
- release occurs only after valid destination arrival;
- destination uses authored semantics, not hardcoded coordinates hidden in script logic;
- any timeout/planning failure reaches a known safe terminal state;
- SUCCESS is only emitted after final safe/home condition.

### Exit gate

One recorded fake-hardware full cycle with complete task trace.

## G11 - Retry, failure handling, and state hygiene

### Required acceptance cases

- top-ranked target blocked by collision -> reject and try next;
- non-target distractor -> never selected;
- stale object -> rejected;
- invalid frame/transform -> blocked before motion;
- no valid grasp -> task fails cleanly;
- invalid destination -> task blocked before grasp or before unsafe transfer;
- duplicate observation -> does not create duplicate PlanningScene identities;
- stop/cancel -> runtime reaches a known safe state.

### Exit gate

At least one success-after-retry scenario and one clean terminal failure scenario are recorded.

## G12 - Product lifecycle closure

### Goal

The workflow feels like one product, not scripts that happen to work once.

### Acceptance

- stop/close RViz without killing Workcell Builder;
- no orphan Product View server/process;
- no stale dynamic port ownership;
- builder status changes back to an accurate idle/ready state;
- same scene remains selected;
- another edit/save/generate/launch/task cycle works;
- second cycle does not need application restart or manual cleanup.

### Exit gate

Two consecutive full product cycles in one Workcell Builder session.

## G13 - Acceptance/evidence bundle

### Bundle contents

- repository commit SHA;
- canonical scene ID/path;
- authored source hashes;
- generated provenance/fingerprint;
- task target class and destination ID;
- perception mode;
- normalized object snapshot;
- candidate-decision report;
- grasp-plan summary;
- parity report;
- build transcript;
- launch transcript;
- task execution trace;
- final result;
- screenshots/recording;
- explicit fake-hardware/real-motion lock status;
- known warnings/limitations.

### Exit gate

A reviewer can understand and reproduce the successful run without relying on tribal knowledge.

# R1 quality bar: "SOTA for this scope"

R1 does not need every future robot or simulator. It does need unusually high quality inside its chosen vertical slice.

## Determinism

- stable IDs;
- stable task/destination bindings;
- repeatable generation;
- no mtime-only readiness truth when semantic fingerprints can be used;
- deterministic ranking when candidate scores tie.

## Observability

Every state transition should expose:

- state name;
- object/task/scene ID;
- reason;
- input source;
- next action or terminal classification.

Recommended task states:

```text
IDLE
INPUT_WAIT
CANDIDATE_EVALUATION
PLANNING_PREGRASP
MOVING_PREGRASP
PLANNING_GRASP
MOVING_GRASP
GRIPPING
ATTACHING
RETREATING
TRANSFERRING
PLACING
RELEASING
DETACHING
RETURNING_HOME
SUCCESS
FAILED
CANCELLED
```

The exact implementation may differ, but the runtime must be inspectable at comparable resolution.

## Idempotence

- repeated Save is safe;
- repeated Generate is safe;
- repeated perception frames update rather than duplicate stable runtime objects;
- start/stop/start is safe;
- retry does not leave stale attached objects, planner state, or hidden scene mutations.

## Failure taxonomy

Prefer stable reason codes in addition to human text, for example:

```text
TARGET_CLASS_MISMATCH
CONFIDENCE_BELOW_THRESHOLD
OBSERVATION_STALE
TF_UNAVAILABLE
OUTSIDE_PICK_ZONE
GRIPPER_INCOMPATIBLE
UNREACHABLE
COLLISION_BLOCKED
NO_VALID_GRASP
DESTINATION_INVALID
PLAN_FAILED
EXECUTION_TIMEOUT
ATTACH_FAILED
PLACE_FAILED
HOME_RETURN_FAILED
USER_CANCELLED
```

Do not force these exact names if existing contracts already have better stable codes. The requirement is stable machine-readable failure semantics.

## Performance

Do not prematurely optimize microseconds, but record timings for:

- perception-to-normalized-object latency;
- candidate evaluation;
- grasp generation;
- motion planning stages;
- total task time.

Use timings to find real bottlenecks after correctness is closed.

# R1 implementation sequence - recommended next PRs

The sequence below is designed for Codex/Astra Ultra. A PR may combine adjacent items only when one root cause spans both and the result remains reviewable.

## PR 1 - R1 truth convergence

Suggested branch: `fix/r1-source-of-truth-convergence`

Close:

- target-class/task-intent mismatch;
- destination owner ambiguity;
- authored vs generated vs runtime ownership comments/docs where code still disagrees;
- ensure runtime configs are derived from or explicitly checked against canonical task intent.

Acceptance: one target class + one destination ID propagate from authoring to runtime.

## PR 2 - R1 generation/parity lifecycle

Suggested branch: `fix/r1-generation-product-view-parity`

Close:

- Generate -> publish current Product View payload -> strict post-generation parity ordering;
- stale Product View payload cannot cause false blocker/pass;
- generation state and parity state refresh atomically in Workcell Builder.

Acceptance: Generate Scene Package ends with current canonical parity PASS on `ur5_2f_test`.

## PR 3 - R1 clean build/fake launch proof

Suggested branch: `fix/r1-humble-fake-launch-proof`

Close:

- any remaining workspace/package discovery issues;
- frame/tool/planning-group launch defects;
- capture canonical fake-hardware evidence script or documented command set.

Acceptance: clean build + launch evidence.

## PR 4 - R1 candidate decision engine

Suggested branch: `feat/r1-candidate-decision-trace`

Close:

- one evaluation pipeline for class/freshness/confidence/zone/gripper/reach/collision/grasp/destination;
- stable rejection reason codes;
- deterministic ranking;
- current retry replay proves blocked-first-target -> next-target selection.

Acceptance: machine-readable candidate report is correct for replay fixture.

## PR 5 - R1 manipulation state machine

Suggested branch: `feat/r1-manipulation-state-machine`

Close:

- explicit task state transitions;
- attach/detach lifecycle;
- stage-specific timeouts/errors;
- safe terminal state;
- return-home requirement before success.

Acceptance: replay target completes fake-hardware approach/grasp/place/home.

## PR 6 - R1 retry/state hygiene

Suggested branch: `fix/r1-retry-state-hygiene`

Close:

- duplicate PlanningScene objects;
- stale candidate reuse;
- failed-attempt cleanup;
- bounded retry;
- cancel/stop cleanup.

Acceptance: intentional first-candidate failure followed by second-candidate success in one run.

## PR 7 - R1 Workcell Builder runtime orchestration

Suggested branch: `feat/r1-builder-runtime-orchestration`

Close:

- Plan/Simulate starts the approved fake-hardware task path;
- builder shows task state, selected object, rejection/failure reason, and terminal outcome;
- stop works;
- no terminal-only hidden recovery step.

Acceptance: full task can be initiated/observed/stopped from Workcell Studio engineering flow.

## PR 8 - R1 second-cycle lifecycle acceptance

Suggested branch: `fix/r1-repeat-cycle-lifecycle`

Close:

- RViz/process teardown;
- Product View server ownership;
- stale ports;
- readiness refresh;
- second launch/task in same session.

Acceptance: two complete cycles without restarting Workcell Builder.

## PR 9 - R1 live D435i/EPD acceptance

Suggested branch: `feat/r1-live-epd-acceptance`

Close:

- live timestamp/frame identity;
- world transform at observation time;
- explicit confidence policy;
- scene/camera binding;
- calibrated live target accepted by the same candidate/task pipeline.

Acceptance: one live camera full-cycle fake-hardware run, or a precisely documented calibration/hardware blocker if external conditions prevent it.

## PR 10 - R1 evidence pack and freeze

Suggested branch: `feat/r1-acceptance-bundle`

Close:

- export canonical evidence bundle;
- one command/checklist to verify the recorded acceptance contract;
- freeze `ur5_2f_test` support claim at the evidence-backed level.

Acceptance: R1 G0-G13 matrix is all PASS.

# R1 test pyramid

## Layer A - Pure/static unit tests

Use for:

- parsers;
- candidate scoring/filtering;
- source-of-truth validation;
- destination resolution;
- capability compatibility;
- state machine transitions;
- retry bookkeeping;
- deterministic generation helpers.

These tests are fast but cannot close GUI/runtime gates alone.

## Layer B - Offline scene/package integration

Use for:

- generate twice and compare semantics;
- manifest/package contract;
- parity report;
- xacro expansion where environment supports it;
- replay object normalization;
- candidate decision report;
- dry-run plan request generation.

## Layer C - ROS headless integration

Use for:

- package discovery;
- robot_description/xacro;
- MoveIt/planning-scene setup;
- grasp plan requests;
- fake-hardware runtime nodes where GUI is not required.

## Layer D - GUI acceptance

Use on display-enabled workstation for:

- selection/inspector parity;
- 3D edit/save/reopen;
- Generate/Validate/Plan state consistency;
- Product View lifecycle;
- process teardown/second cycle.

## Layer E - Full fake-hardware end-to-end

Use for:

- replayed perception full cycle;
- retry/failure path;
- live EPD full cycle;
- return-home and safe stop.

R1 cannot be declared complete without Layer D + Layer E evidence.

# After R1 passes

Only then generalize the proven contract.

## R2 - Second modality: `suction_test`

Apply the same G0-G13 checklist where relevant.

Do not merely make suction visuals load. Prove suction capability metadata, grasp policy, attach/release behavior, destination semantics, fake-hardware cycle, and evidence.

## R3 - Scene catalog truth

- apply the proven contracts to `ur3_suction_test`, `ur10_2f_test`, `ur5_3f_test`, `ur5_airpick4_test`, and other enabled scenes;
- every `supported` scene has acceptance evidence;
- experimental/blocked scenes carry an exact blocker.

## R4 - Workcell Builder productization

After functionality is proven:

- simplify Home/Scene Builder action hierarchy;
- make Inspector/Task/Checks roles obvious;
- hide developer detail by default;
- make Run Next always actionable;
- improve evidence/report navigation;
- add operator-friendly language without hiding engineering truth.

## R5 - Capability-driven generation breadth

Prove swapability across:

- robot profile;
- tool profile;
- camera/sensor profile;
- environment asset set;
- task template;
- grasp strategy.

Add new robots/tools only through capability contracts that preserve the proven R1 architecture.

## R6 - Replay/live perception product contract

Formalize versioned adapter schema, replay fixtures, live diagnostics, calibration ownership, and generated scene-local bindings while keeping EPD separate.

## R7 - Operator HMI

Introduce a separate runtime/end-user surface only after engineering closure.

The Operator HMI consumes approved scene/task/runtime contracts and exposes:

- approved recipe/target selection;
- Start/Stop;
- camera/runtime status;
- object/cycle counters;
- alarm/failure reasons;
- accepted/rejected object decisions;
- production status.

It is not a third engineering authoring system.

## R8 - Demo/customer/investor bundle

Produce polished review artifacts from proven runtime evidence, not from hand-curated screenshots disconnected from the real workflow.

## R9 - Optional simulation backends

Gazebo Sim first where physics/conveyors/sensors add real value. Isaac Sim later for high-fidelity digital-twin/visual workloads.

Neither becomes mandatory for the core Workcell Studio flow.

## R10 - Guarded physical commissioning

Only after the fake-hardware contract is mature:

- explicit hardware mode;
- controller/hardware identity checks;
- calibration checks;
- safety I/O/checklist;
- speed/zone policy;
- tool state verification;
- commissioning evidence;
- explicit operator confirmation.

Normal Workcell Builder simulation must never accidentally enter this mode.

# Roadmap governance for Codex/Astra Ultra

Before starting work, the agent must state:

1. current R1 gate;
2. exact blocker;
3. source-of-truth layer;
4. evidence already present;
5. acceptance criterion for this change.

During work:

- reproduce first;
- fix root cause;
- add focused regression coverage;
- run relevant validation;
- continue through same-root-cause failures until the gate closes or a real external blocker is proven;
- do not widen scope for aesthetic cleanup.

At completion, report:

- gate status;
- root cause;
- files changed;
- tests/commands run;
- runtime/GUI checks run;
- evidence produced;
- safety status;
- residual blocker;
- next gate.

# Immediate next move

The integration branch already contains the R1 runtime-selection/retry/destination/EPD consolidation. The highest-value next work is therefore:

1. **resolve the canonical task/source-of-truth mismatch (especially the target class);**
2. **make Generate -> current Product View -> strict post-generation parity one coherent lifecycle;**
3. **prove clean Humble build + fake launch;**
4. **drive the existing replay/retry runtime into a complete grasp/place/home state machine;**
5. **surface that runtime coherently through Workcell Builder;**
6. **then perform live D435i/EPD acceptance.**

That sequence turns the project from "many advanced pieces" into one genuinely complete industrial system slice. After that, scaling becomes engineering rather than guesswork.
