# R2.0b Shared Readiness / Planner Extraction Design

**Status:** Design approved in chat; written review pending before implementation
**Date:** 2026-09-15
**Branch:** `codex/r20b-shared-readiness-resolver`
**Base:** merged R2.0a (`f4261f7c`)

## 1. Problem

R2.0b needs one authoritative resolver that can enforce `AUTO`, `PREFERRED`, and `EXACT` grasp/place intent and return truthful `READY`, `WARNING`, or `BLOCKED` results.

The existing fake-hardware runtime already performs the required collision-aware full-cycle preplanning, but that logic is embedded inside `scripts/perceived_object_grasp_execute.py`. Its current grasp candidate generation is effectively top-down box grasp generation and does not implement the catalog strategy IDs (`top_2f`, `side_grip_basic`, `finger_pinch_basic`) as distinct physical planning strategies.

Therefore R2.0b must not create a parallel planner or claim readiness from schema/IK alone. The existing full-cycle preplanner must become a reusable authority first.

## 2. Decision

Extract the existing collision-aware full-cycle candidate preplanning logic behind a reusable Python API, preserving its safety behavior and stage semantics. Then feed that shared preplanner with strategy-specific candidate generators.

The architecture is:

```text
TaskIntentModel v2
        |
        v
R2.0b intent resolver
        |
        +--> installed tool/capability profile
        +--> observation/object selection
        +--> R1.9 physical destination resolver
        |
        v
strategy candidate generation
  top_2f / side_grip_basic / finger_pinch_basic
        |
        v
shared full-cycle preplanner
  approach -> grasp -> close -> attach -> lift
  -> transfer -> place -> open -> retreat -> home
        |
        v
TaskIntentResolution/v1
  READY / WARNING / BLOCKED
        |
        +--> derived task_recipe/v1
        +--> runtime later
        +--> Product View later
```

No second planner is introduced. `perceived_object_grasp_execute.py` becomes an adapter/consumer of the same shared preplanning authority.

## 3. Authority boundaries

- `TaskIntentModel v2` remains the only authored task source.
- `scripts/physical_destination.py` remains the sole target-local to world destination authority.
- The strategy catalog remains the vocabulary/parameter source for real strategy IDs.
- The extracted preplanner owns physical feasibility of a complete candidate cycle.
- The resolver owns policy (`AUTO`, `PREFERRED`, `EXACT`), deterministic selection, fallback reporting, provenance, and readiness aggregation.
- Runtime execution owns actual trajectory execution only after a fully prevalidated cycle.

Derived selected/resolved values are never written back into authored intent.

## 4. Planner extraction

The extraction must preserve the existing proven behavior rather than redesign it.

Reusable responsibilities include:

- candidate full-cycle preplanning against a private PlanningScene;
- collision-aware MoveGroup plan-only segments;
- IK only as a seed/constraint source, never as readiness proof;
- straight/contact corridor validation where currently required;
- target-only ACM transition handling;
- close-gripper contact validation;
- private attachment/detachment state transitions;
- lift, transfer, place, open, retreat, and home prevalidation;
- object containment at the R1.9 destination;
- unchanged-live-scene verification semantics;
- stable stage failure information.

The initial extraction must reproduce current `top_2f` behavior before adding other strategies.

If exact parity cannot be demonstrated, the extraction is not accepted.

## 5. Strategy-specific candidate generation

### `top_2f`

First supported strategy. It must reproduce the current top-down deterministic candidate set and current canonical fake-hardware behavior.

### `side_grip_basic`

Implement a real side-approach candidate generator using catalog semantics (`approach_axis: x_plus`, horizontal orientation) and live object geometry. It must generate object-derived poses rather than scene-specific constants.

### `finger_pinch_basic`

Implement a real tool-aligned parallel pinch candidate generator consistent with the catalog. It may share lower-level geometry/orientation helpers with `top_2f`, but must remain a distinct canonical strategy ID and output.

All strategy candidate ordering must be deterministic for identical inputs.

Unknown or unsupported strategies fail closed.

## 6. Policy semantics

### AUTO

Evaluate all supported strategy/candidate combinations allowed by intent and installed capability. Choose the first fully feasible result using a documented deterministic ordering.

### PREFERRED

Evaluate the requested strategy first. If no complete feasible candidate exists, evaluate allowed alternatives. Any fallback produces `WARNING` and records requested strategy, selected strategy, and stable rejection reason.

### EXACT

Evaluate only the requested strategy and exact authored constraints. No substitution, clamping, snapping, or fallback. Any failed required check produces `BLOCKED`.

The same no-fallback rule applies to EXACT placement.

## 7. Place resolution

R1.9 remains authoritative.

The resolver supplies `asset_ref`, `region_ref`, and selected/requested target-local pose. `physical_destination.py` performs the physical target/region validation and world transform.

For AUTO place, the resolver may deterministically choose a valid local point within the same target/region.

For PREFERRED place, the requested local pose is tried first; fallback, if required, stays inside the same target/region and is explicit.

For EXACT place, the authored local pose is consumed unchanged or the result is `BLOCKED`.

No separate world-space fallback is permitted.

## 8. Resolution artifact

The versioned artifact remains `workcell_task_intent_resolution/v1` and records at minimum:

- normalized intent hash;
- physical scene/environment provenance;
- installed tool and capabilities;
- observation identity/provenance;
- requested/selected grasp strategy;
- selected candidate ID/object-local pose;
- effective grasp approach/orientation/TCP/contact/aperture/lift;
- requested/selected target-local placement pose;
- resolved world placement pose;
- effective place approach/orientation/clearance/release/retreat;
- checks with stable codes and truthful PASS/FAIL/NOT_RUN semantics;
- fallback decision/reason;
- grasp/place/overall readiness.

YAML and JSON mirrors must be semantically identical. Volatile timestamps are excluded from semantic hashes.

## 9. Safety rules

- Fake hardware only for R2.0b acceptance.
- No real hardware enablement.
- No live EPD or RealSense dependency.
- No `IK == READY` shortcut.
- No fabricated PASS for MoveIt-dependent checks.
- No broad ACM relaxation.
- No scene-specific IDs in generic resolver/preplanner code.
- No silent strategy fallback.
- No authored-intent mutation from resolution.

## 10. TDD sequence

Implementation proceeds in this order:

1. Keep the existing R2.0b resolver contract tests RED.
2. Add extraction parity tests around the current top-down/full-cycle preplanner API.
3. Extract the shared preplanner with no intentional behavior change.
4. Prove `top_2f` parity with existing R1.5/R1.9 fake-hardware behavior.
5. Add strategy candidate-generation tests for `side_grip_basic` and `finger_pinch_basic`.
6. Implement those strategy generators.
7. Implement resolver policy logic using the shared preplanner.
8. Implement resolution YAML/JSON artifact and derived recipe conversion.
9. Run focused regression and workstation fake-hardware acceptance.

## 11. Acceptance

R2.0b is complete only when evidence proves:

- `top_2f` extraction preserves current full-cycle preplanning behavior;
- AUTO selection is deterministic;
- PREFERRED uses the requested strategy when valid and reports explicit fallback when not;
- EXACT never substitutes a strategy or place pose;
- invalid EXACT grasp/place is `BLOCKED` before execution;
- unsupported capability is `BLOCKED`;
- all world placement comes through R1.9;
- full collision-aware preplanning, not IK alone, gates `READY`;
- YAML/JSON resolution mirrors are semantically equal;
- derived recipe selected values match the resolution artifact;
- R2.0a and R1.9 regressions remain green;
- fake-hardware workstation evidence proves the canonical `ur5_2f_test` path;
- no real trajectory execution is performed during plan-only acceptance.

## 12. Out of scope

R2.0b does not add Task Authoring Qt UI, Product View rendering, live EPD, RealSense, suction implementation, operator HMI, arbitrary behavior graphs, or real hardware. Those remain later milestones.
