- Workcell Builder now includes a Scene Library + Asset Library workflow and safe STL-to-environment-asset import path while keeping fake-hardware-first generation defaults.
# Workcell Studio Roadmap

## Purpose

Workcell Studio is the internal configurable robotic-cell platform for defining, generating, validating, simulating, and ultimately operating deployment-ready workcells. It is the product platform direction for this repository.

Sorting is only one scenario template in Workcell Studio, alongside other templates such as inspection routing, reject handling, and palletizing.

## Current milestone — R1 canonical UR5 + Robotiq 2F industrial cell

**Canonical scene:** `ur5_2f_test`

The immediate objective is not more breadth. It is to make one industrial scene complete, understandable, reproducible, and trustworthy from first open through fake-hardware planning/simulation and back into Workcell Studio.

### Already established

- Canonical UR5 + Robotiq 2F scene with authored workbench, destination bin, RealSense camera, task zones, home state, grasp/task metadata, and collision-aware MoveIt integration.
- Perception-backed task intent for a target class such as `bottle`; perceived workpieces are runtime data rather than authored fake geometry.
- Product View is embedded in Workcell Builder and now uses an owned loopback server with dynamic ports and restart-safe lifecycle handling.
- Product View edit/save/reopen transform persistence has been repaired, including repeated saves and authored transform rebasing.
- Scene generation, generated package structure, validation, fake-hardware defaults, RViz/MoveIt launch path, and planning-scene collision objects have all been exercised successfully during recent acceptance work.
- Real hardware remains locked/off by default.

### R1.1 — Fix the current Plan / Simulate readiness contradiction first

Investigation on 2026-09-10 found a split-brain readiness path in Workcell Builder:

- The visible workflow rail (`scene_workflow_steps`) can mark **Generate Scene Package**, **Validate**, and **RViz/MoveIt Fake-Hardware Launch** as `Done` from package/validation file state.
- The **Run Next: Plan / Simulate** recommendation uses the separate `selected_scene_preview_ready()` gate.
- `selected_scene_preview_ready()` currently requires `generated/workcell_studio_layout_merge_report.json` to exist and compares its filesystem modification time with `layout/workcell_studio_layout.yaml`.
- `scenes/*/generated/` is intentionally gitignored. A clean checkout therefore does not carry that merge report, and a normal `git pull` can also make the authored layout newer by mtime even when its content is already represented by the committed/generated scene package.
- In that blocked branch, `resolve_recommended_workflow_actions()` places disabled **Plan / Simulate** first and enabled **Generate scene package** second. The side panel blindly promotes the first entry and disables the whole primary `Run Next` button, making the corrective action effectively hidden.

This is the highest-priority Workcell Builder bug because the UI currently says the scene is complete while simultaneously preventing the user from taking the next step.

#### Required fix

Create **one canonical readiness result** for the Scene Builder workflow rail, Run Next recommendation, Plan / Simulate page, and Home/scene status. Do not maintain separate partially overlapping truth calculations.

Freshness must be content/contract based, not dependent on volatile filesystem mtimes or an ignored cache artifact. Reuse the repository's durable authored-input fingerprint / current acceptance semantics, or another single explicit generation fingerprint, so a clean checkout or `git pull` cannot falsely invalidate an unchanged scene.

Acceptance requirements:

- The workflow rail must never show all prerequisite steps `Done` while the recommended next action is disabled.
- If Plan / Simulate is blocked, the visible step that is blocking it must also be `Blocked`/`Needed` and must show the exact reason.
- The primary Run Next action must always be actionable. If Plan / Simulate is blocked, promote the enabled corrective action (for example **Generate Scene Package** or **Validate**) instead of promoting a disabled action.
- Disabled secondary actions must have a visible blocker explanation, not tooltip-only state.
- A clean checkout of a valid `ur5_2f_test` must not require an ignored `generated/` cache file merely to determine whether the committed scene is current.

### R1.2 — Make the Scene Builder side panels product-quality

The three right-side tabs must have distinct jobs and use operator/engineer language instead of exposing implementation noise by default.

#### Inspector

Show only the selected physical/semantic scene item's useful editable state:

- display name and semantic role;
- position/orientation;
- lock/editability/source state;
- collision/mesh summary where useful;
- concise warnings that apply to that selected item.

Mesh/STL dimensions imported from authoritative geometry should be read-only unless the asset type explicitly supports authored primitive dimensions. File paths, provenance internals, raw IDs, and parser details belong under a collapsed **Advanced** section.

#### Task

Show the task in human-readable form first:

- target class, e.g. `bottle`;
- pick source/zone;
- grasp strategy;
- place target;
- release/retreat behavior;
- camera/perception source;
- confidence/age rule with clear wording such as `Unset — current adapter does not provide confidence` when applicable.

Do not put the whole Scene Builder workflow/progress rail at the top of the Task tab. Task should answer: **what will this cell try to do?**

#### Checks

Move the workflow/progress/readiness summary here. Checks should answer: **what is ready, what is blocked, and what do I do next?**

Show a compact ordered sequence:

`Scene -> Save Layout -> Generate YAML -> Generate Scene Package -> Validate -> 3D Preview -> RViz/MoveIt Fake Hardware -> Export`

Then show one clear current status, blocker/warning count, exact blocker text, and one enabled recommended action. Detailed reports/paths can remain available under an expandable diagnostics area.

### R1.3 — Close the full Workcell Builder round trip

The canonical acceptance flow must work from one Workcell Studio session without terminal-only recovery steps:

`Open ur5_2f_test -> inspect/edit -> Save Layout -> Generate -> Validate -> Plan / Simulate -> launch RViz/MoveIt fake hardware -> stop/close RViz -> return to the same Workcell Studio scene -> edit/save again -> reopen and verify persistence`

Required acceptance:

- no stale Product View server/port ownership;
- no duplicate robot/camera/environment visuals;
- no transform drift between Product View, authored YAML, generated scene, and RViz truth;
- closing/stopping RViz leaves Workcell Studio alive and usable;
- the same scene can be edited and simulated again without restarting the application;
- generation/validation state updates immediately and consistently across Home, Scene Builder, Checks, and Plan / Simulate.

### R1.4 — Complete the first fake-hardware industrial pick/place cycle

After R1.1-R1.3 are stable, finish the complete `ur5_2f_test` fake-hardware cycle using the existing perception -> PlanningScene -> grasp-planning path.

Target behavior:

`camera/EPD detection -> normalize stable object -> target-class filter -> transform to world -> PlanningScene collision object -> reachability/collision-aware grasp candidate -> approach -> grasp -> attach -> transfer -> place -> release -> retreat`

Selection policy for the first vertical slice:

- if the configured target class is `cup`, pick reachable/collision-free cups;
- if it is `bottle`, pick reachable/collision-free bottles;
- objects of other classes are visible to perception/planning as appropriate but are not task targets;
- do not pick an object simply because it is detected; target-class, freshness, reachability, grasp validity, collision, and destination checks must all pass.

The full fake-hardware cycle remains pending final calibrated/live perception acceptance. Historical/replayed targets are useful regression evidence but are not a substitute for a calibrated live camera acceptance run.

### R1.5 — Add the separate end-user runtime HMI after engineering closure

Workcell Studio should ultimately have three clearly separated user experiences rather than forcing runtime operation into the engineering GUIs:

1. **Workcell Builder** — engineering/configuration GUI for physical cell layout, task intent, generation, validation, and simulation handoff.
2. **EPD GUI** — engineering/configuration GUI for perception model training/deployment and camera/perception diagnostics.
3. **Workcell Operator HMI** — runtime/end-user GUI for selecting an approved recipe/target, starting/stopping the cell, viewing camera/status/output, counters/alarms, and understanding why an object was accepted/rejected.

The Operator HMI must consume approved Workcell Studio task/scene contracts; it must not become a third authoring source of truth. Engineering changes stay in Workcell Builder/EPD. Operator controls should be intentionally narrow and safe.

## Near-term visual target

The near-term engineering experience target is now:

- **Embedded Product View in Workcell Builder** for primary scene authoring and inspection.
- **RViz + MoveIt** as the authoritative generated-scene/planning/fake-hardware truth.
- **Plan / Simulate** as the Workcell Studio launch/control surface that bridges the two cleanly.

A separate web/operator HMI is a later runtime surface, not a replacement for Workcell Builder.

## Recommended simulation sequence

1. **RViz + MoveIt now**
   - Baseline for ROS 2 Humble commissioning and motion-planning workflows.
   - Keep fake hardware and headless validation paths as defaults.
2. **Gazebo Sim later**
   - Add conveyors and basic physics simulation when needed.
   - Optional extension, not a baseline requirement.
3. **Isaac Sim later**
   - Use for investor-grade visuals and advanced simulation demos.
   - Optional extension, not mandatory for functional deployment.

## Repository boundaries

High-level ownership boundaries:

- **EPD (Easy Perception Deployment)** owns perception pipelines and detected-object outputs.
- **Easy_Manipulator_Improved** owns Workcell Studio orchestration: generation, validation, planning integration, simulation handoff, and runtime task integration.
- **Generated scene packages** own launch/config output artifacts for specific workcell instances.
- **Future Operator HMI** consumes approved scene/task/runtime contracts and does not redefine engineering configuration.

## Scope guardrails

- Preserve ROS 2 Humble as the supported baseline.
- Do not require Gazebo Sim or Isaac Sim for core operation.
- Preserve fake-hardware-first defaults and dry-run/offline validation language.
- Keep runtime launch behavior backward compatible.
- Do not weaken collision checking or Allowed Collision Matrix policy merely to make a demo pass.
- Do not treat a visual preview as planning/runtime readiness proof.

## Architecture guardrail

- Workcell Studio does not replace `workcell_builder`; it augments it.
- `workcell_builder` remains the primary visual scene builder/editor.
- Capability/grasp catalogs and validators are backend services used by builder generation and QA tooling.
- CLI/wizard scripts are for automation/testing and are not a competing UI path.
- Product View is an authoring surface; RViz/MoveIt remains generated planning truth.
- The future Operator HMI is an operation surface, not another engineering authoring path.

## Builder scene exports for Workcell Studio

`workcell_builder` remains the primary visual workflow. Generated scenes can now export portable Workcell Studio source files using `scripts/export_builder_scene_to_cell_definition.py`. The export writes `generated/cell_definition.yaml`, `generated/environment_layout.yaml`, `generated/workcell_builder_task_intent.yaml` (when enough task authoring metadata exists), and `generated/builder_export_summary.json`. These files are for offline commissioning and backend tooling, and are not proof of reachability or runtime safety. Keep fake-hardware-first defaults and runtime send disabled unless separately commissioned.

## Curated Demo Catalog

`catalog/workcell_studio_demos.yaml` defines investor/customer-friendly offline demo templates. Generated demo bundles are strictly offline/demo/validation artifacts and do not alter runtime robot behavior or launch behavior. Preview-only demos support sales/concept visualization and must not be treated as real runtime support.

## Create-cell wizard

`workcell_builder` remains the visual scene editor. `workcell_studio.py create-cell` is a fast catalog-driven YAML creation path that generates cell definition/layout, validates, produces static previews, and can optionally emit offline bundle artifacts.

Next safe bridge: guarded RViz/MoveIt plan-preview session preparation from offline plan-preview requests, without real-hardware launch or uncontrolled motion execution.

- builder scene -> task intent -> task recipe -> offline plan request -> RViz preview session -> smoke launch -> planning scene readiness

Readiness pipeline: builder scene -> task intent -> task recipe -> task flow -> static preview -> offline plan request -> RViz preview session -> smoke launch -> planning scene readiness -> readiness pack.

## Readiness classification shorthand

- `physical_scene_only`: cell/layout metadata exists but task flow authoring is missing or incomplete.
- `task_intent_present` / `task_preview_ready`: builder task intent exists and validates for offline preview-oriented task flow checks.
- `runtime_ready`: only when downstream runtime safety gates, launch checks, and existing commissioning requirements pass; task intent alone does not imply runtime execution readiness.

## Canonical golden builder-to-readiness demo

Use `scripts/run_golden_builder_readiness_demo.py` as the canonical acceptance path for builder-generated scenes.

- Offline/fake-hardware only (no real hardware motion).
- Proves: builder scene validation -> Workcell Studio export -> task intent validation -> readiness pack -> static preview/dashboard.
- Uses existing Workcell Studio scripts and safety defaults; it does not replace `workcell_builder`.

Manual run:

```bash
python3 scripts/run_golden_builder_readiness_demo.py \
  --scene-package scenes/ur5_2f_test \
  --output-dir /tmp/golden_builder_demo \
  --force --json
```

This demo is for repeatable regression/acceptance checks only and is not a real-hardware readiness certificate.

### Safe RViz/MoveIt fake-hardware visual preview extension

The golden demo also emits an RViz/MoveIt preview-readiness section in `golden_builder_demo_summary.json`.

- It reports preview readiness for robot description, end effector metadata, support/table context, pick/place zones, task-flow markers, and fake-hardware launch metadata.
- If a safe command can be generated, it is included as a manual command (for example with `use_fake_hardware:=true`).
- If preview metadata is incomplete, the summary is explicitly classified `rviz_preview_partial` and lists concrete blockers/warnings.
- This remains fake-hardware/no-real-motion by default and does **not** enable real robot execution.
