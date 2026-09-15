# R2.0 Full Engineering Task Authoring Design

**Status:** Design for review; no production implementation in this pass  
**Date:** 2026-09-15  
**Scope:** Workcell Builder engineering authoring for the canonical `ur5_2f_test` scene, with a capability-based contract that can later describe suction tools.

## 1. Purpose and non-goals

R2.0 lets an engineering user author four separate decisions in Workcell Builder:

1. **What** is the task and which perceived object(s) are eligible.
2. **How** the object is grasped.
3. **Where** it is picked from and placed.
4. **How to place** it, including release, orientation, clearance, and retreat.

The authoring surface expresses user intent. A separate resolver/planner evaluates whether that intent is physically valid for the selected robot, tool, object observation, and destination. The UI must never silently alter an authored exact request to make it pass.

R2.0 does not implement suction planning, live camera/EPD, real hardware, Gazebo, Isaac, or a new operator HMI. It does not replace Product View, RViz, MoveIt, `task_intent.yaml`, `task_recipe.yaml`, or the R1.9 target-local physical destination resolver. All defaults remain fake-hardware/offline-preview safe.

## 2. Design principles and ownership

- `environment.yaml` and the authored layout remain the physical scene source of truth.
- `config/workcell_builder_task_intent.yaml` is the single authored engineering task-intent source of truth after R2.0 migration.
- `task_recipe.yaml` is derived and runtime-facing; it must contain the resolved policy and provenance of the intent that produced it.
- Product View reads the same authoritative intent/resolution artifact used by generation. It may display a diagnostic state, but it cannot invent a pose or destination.
- RViz/MoveIt and the grasp planner are the physical validity authorities. A preview is not proof of runtime reachability.
- Capability catalogs describe tool capabilities and strategy requirements. They are data, not UI-specific conditionals.
- EPD provides normalized observations only; Workcell Studio owns filtering, task resolution, planning, and execution gates.
- R1.9 target-local physical destination semantics remain authoritative: `place.target.id` resolves through the physical target's local frame and `usable_placement` metadata to one checked world destination.

## 3. Terminology

| Term | Meaning |
|---|---|
| Intent | What the engineer requested, including policy (`AUTO`, `PREFERRED`, `EXACT`) and constraints. |
| Capability | A tool/strategy property such as two-finger parallel grasp, aperture range, approach directions, or release mechanism. |
| Candidate | A concrete grasp or placement pose generated from an observation and scene. |
| Resolution | The deterministic result of matching intent to capabilities, candidates, and physical checks. |
| Fallback | A resolver choice different from the requested preferred strategy. Allowed only in `PREFERRED`, and always reported. |
| Exact | A hard requirement. If no candidate satisfies it, resolution is `BLOCKED` before execution. |
| READY/WARNING/BLOCKED | User-facing readiness classes. `READY` means the authored request has a valid checked resolution; `WARNING` means it is usable with an explicit caveat; `BLOCKED` means generation/plan/execute is gated. |

## 4. Authoritative R2.0 schema

The schema version advances to `workcell_builder_task_intent/v2`. Readers must accept v1 and normalize it into this model before validation. Writers emit v2 only after migration is complete.

### 4.1 Schema shape

```yaml
schema: workcell_builder_task_intent/v2
scene_package: scenes/ur5_2f_test
task:
  id: bottle_to_default_bin
  type: pick_place
  template: pick_place
  target_policy:
    class_id: bottle
    source_ref: detected_objects/v1
    max_age_seconds: 2.0
    min_confidence: null
pick:
  source:
    type: pick_zone
    id: pick_zone_main
  object_filter:
    class_id: bottle
    color: null
  grasp:
    policy: AUTO                 # AUTO | PREFERRED | EXACT
    capability: two_finger_parallel
    strategy_ref: top_2f
    approach:
      axis: z_down
      distance_m: 0.12
    orientation:
      mode: vertical
      allowed_roll_deg: [0.0]
      allowed_yaw_deg: [0.0, 90.0, 180.0, 270.0]
    tool_offset_xyz_m: [0.0, 0.0, 0.0]
    tool_offset_rpy_rad: [0.0, 0.0, 0.0]
place:
  target:
    type: destination
    id: default_drop_zone
    region_ref: default_drop_zone
  placement:
    policy: AUTO                 # AUTO | PREFERRED | EXACT
    orientation:
      mode: target_default
      rpy_rad: [0.0, 0.0, 0.0]
      tolerance_rad: [0.0, 0.0, 0.0]
    clearance_m: 0.05
    usable_region_required: true
    retreat:
      axis: z_up
      distance_m: 0.10
  release:
    strategy: open_gripper
routing:
  mode: direct
  rules:
    - id: default_place
      when: {always: true}
      destination: default_drop_zone
tool:
  capability_profile_ref: robotiq_2f
safety:
  preview_only: true
  use_fake_hardware: true
  runtime_io_applied: false
  motion_started: false
  ros_launch_started: false
provenance:
  authored_by: workcell_builder
  source_schema: workcell_builder_task_intent/v2
```

`strategy_ref` is optional for `AUTO`, required for `PREFERRED` and `EXACT`. `capability` is required for all policies; it is the stable cross-tool vocabulary. A future suction profile can use `vacuum_pick` without changing the task shape.

### 4.2 Policy examples

**AUTO** — choose any catalog strategy satisfying the capability and physical checks:

```yaml
grasp: {policy: AUTO, capability: two_finger_parallel, approach: {axis: z_down, distance_m: 0.12}}
place: {placement: {policy: AUTO, orientation: {mode: target_default}, clearance_m: 0.05}}
```

**PREFERRED** — try `top_2f`; a fallback is permitted and must be surfaced and persisted in the resolution report:

```yaml
grasp:
  policy: PREFERRED
  capability: two_finger_parallel
  strategy_ref: top_2f
```

**EXACT** — use only this strategy and constraints; any mismatch blocks:

```yaml
grasp:
  policy: EXACT
  capability: two_finger_parallel
  strategy_ref: top_2f
  orientation: {mode: vertical, allowed_roll_deg: [0.0], allowed_yaw_deg: [90.0]}
```

### 4.3 Resolution artifact

Generation writes `generated/task_intent_resolution.yaml` (and JSON for UI/test consumers). It records the normalized intent hash, capability profile, selected candidate, rejected candidates, fallback reason, physical checks, and status. This is derived data and may never overwrite the authored request.

```yaml
schema: workcell_task_intent_resolution/v1
intent_sha256: <hash>
status: READY
policy: PREFERRED
requested_strategy_ref: top_2f
selected_strategy_ref: finger_side
fallback:
  used: true
  reason: top_2f candidate collided with target rim
pick_candidate: {id: grasp_07, pose_frame: object, score: 0.82}
place_resolution:
  target_id: default_drop_zone
  target_frame: bin_target_local
  world_pose_xyz: [0.45, 0.22, 0.13]
checks:
  - {code: grasp_collision_free, status: PASS}
  - {code: target_reachable, status: PASS}
  - {code: usable_region, status: PASS}
```

## 5. UI design: Builder Task Authoring workspace

Add a dedicated **Task Authoring** workspace/panel in Workcell Builder. It is a single engineering surface; do not expose raw YAML in the primary flow.

### 5.1 Layout

```text
┌ Task Authoring ────────────────────────────────┐
│ Status: READY | Save   Validate   Generate     │
├ What ──────────────────────────────────────────┤
│ Template [Pick and place]  Target class [bottle]│
│ Pick source [pick_zone_main]  Object rule ...   │
├ How to grasp ──────────────────────────────────┤
│ Policy [AUTO ▾]  Capability [Two-finger ... ▾]  │
│ Strategy [Auto ▾]  Approach [Z down ▾] [0.12 m] │
│ Orientation [Vertical ▾]                       │
│ [Advanced grasp constraints ▸]                  │
├ Where ─────────────────────────────────────────┤
│ Place target [default_drop_zone] [Select in view]│
│ Destination: target-local region (read-only)   │
│ [Show resolved destination]                     │
├ How to place ──────────────────────────────────┤
│ Policy [AUTO ▾]  Orientation [Target default ▾] │
│ Clearance [0.05 m]  Release [Open gripper ▾]    │
│ Retreat [Z up ▾] [0.10 m]                       │
├ Validation ────────────────────────────────────┤
│ READY  0 blockers  0 warnings                  │
│ [details: checks and candidate reasoning ▸]     │
└────────────────────────────────────────────────┘
```

The 3D view supplies `Select in view` and highlights pick zones, targets, the resolved placement region, and any rejected candidate reason. Product View remains read-only with respect to task intent; edits commit through Builder's authoring model.

### 5.2 Interaction rules

- Policy labels include one-line help: AUTO chooses, PREFERRED prefers with reported fallback, EXACT never falls back.
- Changing a capability filters strategy choices; an incompatible manually selected strategy is an inline `BLOCKED` error, never auto-replaced.
- `EXACT` exposes required orientation, approach, aperture, contact, and retreat constraints. Missing exact values are blocked.
- Advanced controls are expandable and include aperture/contact tolerances, candidate score weights, tool offsets, and routing conditions.
- Save marks the authored intent clean only after atomic write and re-read normalization. Generate and Plan/Simulate are disabled while blockers or unsaved task edits remain.
- Validation details show user language first, then diagnostic code, evidence source, and “fix in Task Authoring” action.
- Fallback banners are persistent until the policy or inputs change; they are included in the resolution artifact and task recipe provenance.

## 6. Runtime and preview data flow

```text
Builder controls
  → TaskIntentModel (v2, dirty/clean)
  → atomic save config/workcell_builder_task_intent.yaml
  → normalize + static validation
  → capability registry + tool profile
  → grasp/placement resolver
      inputs: canonical scene, R1.9 destination resolver, normalized EPD/replay observation
      outputs: candidates, selected resolution, checks, READY/WARNING/BLOCKED
  → generated/task_intent_resolution.yaml
  → task_recipe/v1 adapter (compatibility mirror + provenance)
  → Product View preview and Plan/Simulate
  → runtime executor gate (fake hardware only by default)
```

The preview consumes the same resolution artifact. If resolution is absent or stale, Product View renders a diagnostic overlay and the unresolved task, never a fabricated pose. Runtime execution must re-check the intent hash and physical checks immediately before planning.

## 7. Validation and readiness rules

Validation has two layers:

1. **Intent validation** (deterministic, no robot motion): schema, IDs, policy completeness, capability/strategy compatibility, numeric ranges, routing, safety flags, and R1.9 destination-chain consistency.
2. **Physical resolution** (offline planner/MoveIt scene): candidate reachability, collision, orientation, aperture/contact fit, target reachability, usable-region containment, placement orientation collision, and retreat feasibility.

Map outcomes as follows:

| Condition | AUTO | PREFERRED | EXACT |
|---|---|---|---|
| Requested strategy unavailable | choose compatible catalog strategy + WARNING | choose fallback + WARNING | BLOCKED |
| Candidate unreachable/colliding | choose another valid candidate | choose another valid candidate; record fallback | BLOCKED |
| Exact orientation unavailable | choose valid orientation | choose valid orientation + WARNING | BLOCKED |
| Aperture/contact invalid | choose another candidate | fallback if capability still satisfied | BLOCKED |
| Destination unresolved or outside usable region | BLOCKED | BLOCKED | BLOCKED |
| Place orientation collides | choose valid orientation | fallback + WARNING | BLOCKED |
| Retreat infeasible | choose valid retreat | fallback + WARNING | BLOCKED |

No policy may bypass target-local destination resolution, collision checking, or safety locks. Any `BLOCKED` result gates Generate, Plan/Simulate, and execution.

## 8. Acceptance matrix

| ID | Scenario | Expected evidence |
|---|---|---|
| A | AUTO with reachable bottle and default bin | Save/reopen identical v2 intent; resolver selects a valid 2F candidate; status READY; preview and recipe use same resolution hash. |
| B | PREFERRED `top_2f` made invalid by candidate collision | Status WARNING; selected fallback and reason visible; resolution and recipe persist fallback; no silent substitution. |
| C | EXACT valid orientation/aperture/contact/retreat | Status READY; selected strategy and exact constraints appear in report; generated recipe preserves them. |
| D | EXACT impossible orientation or unreachable target | Status BLOCKED before Generate/Plan/Execute; exact diagnostic and corrective control shown; no runtime call. |
| E | Edit, Save, close, reopen, Generate, Validate, then change runtime input | Authored values survive round trip; generated recipe and resolution change only after explicit save/generate; Product View and runtime consume the edited values and matching hash. |
| F | R1.9 1 cm target-local bin edit | Destination world pose changes through physical resolver; preview, recipe, validation, and runtime report the same resolved pose. |
| G | Legacy v1 scene | Open migrates in memory with a warning, preserves behavior, writes v2 only on explicit Save, and keeps v1 recipe consumers working. |

## 9. Files and components expected to change during implementation

These are planned touch points, not changes made by this design pass:

- `workcell_builder/workcell_builder/gui/mainwindow.cpp`, `mainwindow.ui`, `environment_task_editor.hpp`: Task Authoring workspace, status/actions, dirty/save lifecycle.
- `workcell_builder/workcell_builder/gui/scene_select.cpp` and its UI: remove duplicate task/grasp authority; route existing controls and canvas assignment into the shared model.
- `workcell_builder/workcell_builder/include/task_intent_model.hpp`, new model/normalizer source: v2 types, migration, stable serialization/hash.
- `workcell_builder/workcell_builder/include/task_intent_readiness.hpp` and source: one readiness result shared by rail, Task Authoring, Checks, and Plan/Simulate.
- `workcell_builder/workcell_builder/include/grasp_strategy_model.hpp` and `src_grasp_strategy_model.cpp`: capability catalog, policy filtering, compatibility diagnostics.
- `scripts/validate_builder_task_intent.py`: v2 structural and policy validation plus stable diagnostic codes.
- `scripts/convert_builder_task_intent_to_task_recipe.py`: v2-to-v1 recipe adapter and provenance.
- New `scripts/resolve_task_intent.py` (or equivalent library): deterministic grasp/place resolution and resolution artifact.
- `scripts/physical_destination.py` and its callers: consume the existing R1.9 target-local contract without duplicating pose logic.
- `scripts/perceived_object_grasp_plan.py`, `scripts/perceived_object_grasp_execute.py`: accept resolved policy/candidate contract; enforce EXACT and fallback reporting.
- `scripts/export_workcell_studio_web_scene_impl.py` and Product View viewer modules: read resolution artifact, render diagnostics, never synthesize task poses.
- `workcell_builder/workcell_builder/templates/workcell_builder_task_intent_template.yaml`: v2 template with explicit policy fields.
- Tests under `tests/` and `workcell_builder/workcell_builder/test/`: schema, migration, UI-model, resolver, preview parity, and acceptance fixtures.

## 10. Migration and backward compatibility

1. Detect `workcell_builder_task_intent/v1` and normalize to an in-memory v2 model.
2. Map `grasp.strategy_ref` to `PREFERRED` when present, otherwise `AUTO`; map existing approach/orientation/retreat fields directly.
3. Map `place.target`, `region_ref`, and offsets to the R1.9 target-local target reference. Ignore historical world-pose fallbacks when an authored physical target exists; emit a migration warning if they disagree.
4. Preserve `task_recipe/v1` output shape and legacy `rules` mirror. Add `builder_task_intent.schema_version`, `intent_sha256`, and resolution provenance as additive fields.
5. Do not rewrite a v1 file merely by opening it. Explicit Save writes v2 atomically and records a migration notice in the readiness report.
6. Existing catalogs with `finger_top`, `finger_side`, and `top_2f` remain aliases to the capability registry. Suction names may be recognized as catalog metadata but remain unsupported for R2.0 execution.
7. Older consumers that ignore unknown fields continue to read `task_recipe/v1`; consumers requiring v2 must fail clearly with an upgrade diagnostic.

## 11. Risks and mitigations

- **Split-brain state:** eliminate direct widget-to-YAML writes; one model owns dirty state and serialization.
- **Preview/runtime drift:** require resolution hash parity and reuse the physical destination resolver.
- **Over-permissive fallback:** enforce policy in resolver and executor; EXACT has no fallback branch.
- **Legacy scene breakage:** normalize v1 in memory and retain additive recipe fields.
- **Capability vocabulary explosion:** version a small registry; add suction only as a future profile, not UI scaffolding in the R2.0 slice.
- **False readiness:** keep READY tied to actual resolver checks, never to file existence alone.

## 12. Definition of done for the design

Human approval is required before production implementation. Approval should explicitly authorize the v2 schema, policy semantics, single-model ownership, and the first implementation slice in the plan below.
