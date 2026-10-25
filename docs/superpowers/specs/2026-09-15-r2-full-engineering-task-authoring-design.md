# R2.0 Full Engineering Task Authoring Design

**Status:** Revised design for approval; production implementation is explicitly out of scope
**Date:** 2026-09-15
**Canonical scene:** `scenes/ur5_2f_test`

## 1. Approved architecture and boundaries

One `TaskIntentModel` is the only authored task source. It is saved as
`config/workcell_builder_task_intent.yaml`; generated `task_recipe.yaml`, the
resolution artifact, Product View payloads, and runtime requests are derived
from it. R1.9 `physical_destination.py` remains the only physical destination
resolver. Product View, Generate, Validate, planner, and runtime consume the
same resolution result and hash.

The model describes engineering intent, not live state. Dynamic execution
facts (`runtime_io_applied`, `motion_started`, `ros_launch_started`) belong in
resolution/runtime/evidence artifacts. The installed tool comes from the
physical scene/environment; task intent can require a capability but cannot
declare the installed tool.

R2.0 supports the existing Robotiq 2F path. Suction, EPD/RealSense, real
hardware, Gazebo, Isaac, and an operator HMI remain out of scope. Defaults stay
fake-hardware/offline-preview safe.

## 2. Authority model

- **Environment:** physical target asset, its frame, `usable_placement` volume,
  and named place-region geometry; installed robot/tool identity.
- **Task intent:** task/template, one pick eligibility section, grasp and place
  requests, required capabilities, and safety policy.
- **Resolver:** candidate selection, fallback reporting, physical checks, and
  resolved target-local/world poses.
- **Runtime/evidence:** dynamic state, launch/execution facts, and evidence.

Object class/source/confidence/age rules appear exactly once in
`pick.selection`. Physical target geometry and usable-region geometry never
appear in task intent.

## 3. Corrected v2 schema

```yaml
schema: workcell_builder_task_intent/v2
scene_package: scenes/ur5_2f_test
task:
  id: bottle_to_default_bin
  type: pick_place
  template: pick_place
pick:
  selection:
    source_ref: detected_objects/v1
    source_type: perception
    zone_ref: pick_zone_main
    object_filter:
      class_id: bottle
      color: null
      min_confidence: null
      max_age_seconds: 2.0
  grasp:
    policy: AUTO                 # AUTO | PREFERRED | EXACT
    required_capability: two_finger_parallel
    strategy_ref: null           # required for PREFERRED/EXACT
    approach:
      axis: z_down
      distance_m: 0.12
    orientation:
      mode: vertical
      allowed_roll_deg: [0.0]
      allowed_yaw_deg: [0.0, 90.0, 180.0, 270.0]
      tolerance_rad: [0.0, 0.0, 0.0]
    tcp_offset_xyz_m: [0.0, 0.0, 0.0]
    tcp_offset_rpy_rad: [0.0, 0.0, 0.0]
    contact: {required: true, min_quality: 0.0}
    aperture: {min_m: 0.0, max_m: 0.085}
    lift: {axis: z_up, distance_m: 0.15}
place:
  target:
    asset_ref: target_bin_default
    region_ref: default_drop_zone
  placement:
    policy: AUTO                 # AUTO | PREFERRED | EXACT
    requested_local_pose: null   # {xyz_m: [...], rpy_rad: [...]} or absent
    orientation:
      mode: target_default
      rpy_rad: [0.0, 0.0, 0.0]
      tolerance_rad: [0.0, 0.0, 0.0]
    approach: {axis: z_down, distance_m: 0.10}
    clearance_m: 0.05
    retreat: {axis: z_up, distance_m: 0.10}
  release: {strategy: tool_release}
routing:
  mode: direct
  rules: [{id: default_place, when: {always: true}, destination: default_drop_zone}]
safety:
  execution_mode: simulation_preview
  require_fake_hardware: true
  real_hardware_enabled: false
  preview_policy: diagnostic_if_unresolved
provenance:
  authored_by: workcell_builder
  source_schema: workcell_builder_task_intent/v2
```

### Units, frames, and policy

All distances are metres; `_rad` angles are radians and `_deg` angles are
degrees. Grasp poses and TCP offsets use the frames documented by the strategy
catalog. `requested_local_pose` is always in the physical target asset's local
frame. The resolver transforms it through the R1.9 target-local chain to world.
Task intent contains no target or region dimensions.

- **AUTO:** strategy and local pose may be absent; choose any valid candidate.
- **PREFERRED:** try the requested strategy/local pose first using its authored
  constraints. If the requested grasp strategy fails, an alternate strategy is
  evaluated with that alternate strategy's reviewed catalog geometry
  (approach axis/distance, orientation, TCP offset and lift distance); the
  substitution and first preferred-strategy blocker are persisted and produce
  `WARNING`.
- **EXACT:** requested strategy/constraints/local pose are consumed unchanged;
  any invalidity is `BLOCKED`, with no fallback.

`release.strategy: tool_release` is semantic. The installed profile maps it to
`fingers_open` for 2F; a future suction profile may map it to vacuum/seal
release. Suction is not implemented here.

### Catalog vocabulary

| Existing catalog ID | Stable capability | Status |
|---|---|---|
| `top_2f` | `two_finger_parallel` | supported |
| `side_grip_basic` | `two_finger_parallel` | supported |
| `finger_pinch_basic` | `two_finger_parallel` | supported |
| `robotiq_2f_85` | installed-tool catalog identity | scene/profile mapping |
| `finger_gripper` | installed-tool catalog identity | scene/profile mapping |

No `finger_side` or `finger_top` alias may be emitted without a canonical,
tested alias table.

## 4. Split resolution artifact

Generation writes `generated/task_intent_resolution.yaml` and a JSON mirror;
it never overwrites authored intent.

```yaml
schema: workcell_task_intent_resolution/v1
readiness_status: READY       # READY | WARNING | BLOCKED
normalized_intent_sha256: <hash>
physical_scene_provenance: {scene_package: scenes/ur5_2f_test, environment_sha256: <hash>, generation_id: <id>}
capability_profile:
  installed_tool_id: robotiq_2f_85
  capabilities: [two_finger_parallel]
  release_mapping: {tool_release: fingers_open}
grasp_resolution:
  requested_policy: PREFERRED
  required_capability: two_finger_parallel
  requested_strategy_ref: top_2f
  selected_strategy_ref: side_grip_basic
  fallback: {used: true, reason: top_2f candidate collided with the target object}
  selected_candidate: {id: grasp_07, pose_frame: object, score: 0.82}
  checks: [{code: reachable, status: PASS}, {code: orientation, status: PASS}, {code: aperture, status: PASS}, {code: contact, status: PASS}, {code: collision_free, status: PASS}]
place_resolution:
  requested_policy: EXACT
  target_asset_ref: target_bin_default
  region_ref: default_drop_zone
  requested_local_pose: {xyz_m: [0.0, 0.0, 0.05], rpy_rad: [0.0, 0.0, 0.0]}
  selected_local_pose: {xyz_m: [0.0, 0.0, 0.05], rpy_rad: [0.0, 0.0, 0.0]}
  world_pose: {frame: world, xyz_m: [0.45, 0.22, 0.13], rpy_rad: [0.0, 0.0, 0.0]}
  fallback: {used: false, reason: null}
  checks: [{code: target_reachable, status: PASS}, {code: inside_usable_region, status: PASS}, {code: orientation_collision_free, status: PASS}, {code: retreat_feasible, status: PASS}]
runtime_policy: {execution_mode: simulation_preview, require_fake_hardware: true, real_hardware_enabled: false}
```

The artifact records requested/selected local poses, resolved world pose,
actual installed profile, provenance, hash, readiness, checks, and fallbacks.

## 5. Task Authoring UI and data flow

One **Task Authoring** workspace presents: **What** (template and the sole Pick
eligibility section), **How to grasp** (policy, required capability, real
strategy ID, approach/orientation, TCP, contact/aperture, lift), **Where**
(physical target asset and named region), **How to place** (policy, local XYZ/RPY,
orientation, approach, semantic release, clearance, retreat), and **Validation**
(READY/WARNING/BLOCKED and actionable diagnostics). Advanced constraints are
expandable; raw YAML is not the primary UX. Save is atomic and re-reads the
normalized model. Generate, Plan/Simulate, and execution use the shared result.

```text
Task Authoring [READY] [Save] [Validate] [Generate]
What / Pick eligibility → How to grasp → Where → How to place → Validation
```

```text
TaskIntentModel v2 → save → normalize/validate → installed-tool lookup + observation
 → grasp resolver + physical_destination.py + place resolver
 → split resolution artifact → recipe/Product View/Plan/runtime
```

Checks cover schema/IDs, policy completeness, capability compatibility,
approach/lift/retreat feasibility, reachability, exact orientation, aperture,
contact, collision, target reachability, usable-region containment, placement
orientation collision, and safety policy. Destination failure is always
`BLOCKED`; unresolved preview is diagnostic only.

## 6. Acceptance matrix

| ID | Acceptance | Required evidence |
|---|---|---|
| A | AUTO selects valid 2F candidate | candidate, local/world pose, READY, matching hash |
| B | PREFERRED uses preference or reports fallback | preference retained; fallback reason persisted; WARNING when used |
| C | EXACT valid grasp + exact target-local placement | all exact inputs consumed unchanged |
| D | EXACT invalid grasp/place | BLOCKED before generation/planning/execution; zero execution calls |
| E | Change grasp approach/lift and place approach/retreat | runtime receives every changed value after save/generate |
| F | Target + region + requested local point | one consistent R1.9 world resolution; no duplicated geometry |
| G | Save/reopen | normalized intent and bindings identical |
| H | Product View/recipe/runtime parity | same normalized intent/resolution hash |
| I | v1 migration | old behavior preserved before explicit Save; no rewrite on open |
| J | R1.9 regression | 1 cm target-local edit and canonical restore remain consistent |

## 7. Migration and back-compatibility

Readers accept v1 and normalize only in memory; opening v1 never writes it.

- A v1 explicit `grasp.strategy_ref: top_2f` becomes an explicit strategy
  constraint, migrated as `EXACT`, never as a preference that can silently
  fall back. Missing approach/orientation/aperture/contact/lift values are
  materialized from the referenced catalog entry and recorded as
  `migration.materialized_from_catalog`; unavailable catalog data is BLOCKED.
- Existing place target/region/offset behavior maps to `asset_ref`, `region_ref`,
  and `requested_local_pose` through R1.9. World-pose disagreement is reported,
  never silently preferred over authored physical geometry.
- Duplicate v1 object filters consolidate under `pick.selection` with a warning.
- `open_gripper`/similar v1 release values normalize to semantic `tool_release`;
  actual actuation is resolved from the installed profile.
- Legacy dynamic safety fields are read for compatibility and moved to runtime
  evidence; v2 writes only execution/preview policy.
- Explicit Save writes v2 atomically. `task_recipe/v1` and its `rules` mirror
  remain available with additive intent/resolution provenance.

## 8. Planned touch points and self-review

Implementation will touch the existing MainWindow/SceneSelect editors, new
`TaskIntentModel` files, readiness/grasp strategy models, validator/recipe
adapters, a resolver, Product View export/viewer, planner/executor adapters,
templates, and tests. This revision changes documentation only.

Self-review: duplicate object authority is removed; target/region/geometry are
separate; units and frames are explicit; AUTO/PREFERRED/EXACT semantics are
hard-gated; dynamic runtime state is out of authored intent; and v1 migration
preserves behavior. Human approval is required before R2.0a.
