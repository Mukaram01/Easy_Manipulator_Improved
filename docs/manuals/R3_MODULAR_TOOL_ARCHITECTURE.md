# R3 — Modular robot, end-effector and grasp execution architecture

Status: **PARTIALLY CONFIRMED.** Reconciled with main `02bf9eeb` on 26 October 2026. The 24 October runtime results below are historical evidence, not qualification of this combined revision. Three-finger and suction engagement remain refused before motion.

## Reconciliation with the qualified runtime (26 October 2026)

Main's launched-model tool binding remains authoritative: the saved command joint, unique SRDF end-effector group,
URDF limits and catalog aperture are retained. A catalog interface may use a commissioned subrange, but a
different joint/group or an exceeded limit/aperture stops with `TOOL_BINDING_MISMATCH`. A cell without an
interface uses its launched-model binding; only old contracts without either binding use legacy 2F values.

The Stage-B replay detection, calibration and recording-clock checks, target-class binding, physical destination
seating, simulator slab collision geometry, qualification gates and `ros2 launch --noninteractive` remain in place.
Tool-description validation runs after binding to the actual launched model and before planning.

The reference runner accepts `--motion-policy` for its copied canonical cell. With `--scene-package`, the saved
cell owns the policy: author it in that cell and Generate again. A CLI override is rejected before any session
is created, rather than silently ignored.

Selection persistence is checked through the existing Studio Save projection, reopen, Generate and Validate
entry points with both experimental force variants, including stale-handoff rejection and matching derived
actuator effort/receipt. This is command-path acceptance, not a fresh interactive GUI acceptance claim.

Fresh integration test, build and runtime results are recorded in `R3_MAIN_RECONCILIATION.md`. Keep the PR draft
until its remaining acceptance limitations are reviewed; a merge does not qualify physical hardware.

## What changed

The task pipeline (resolver → full-cycle preplanner → executor) no longer names a gripper. The only tool-specific source is the
end-effector capability record in `catalog/capabilities/end_effectors/*.yaml`, bound by the cell:

```text
environment.yaml (authored)
  end_effector.capability: robotiq_2f_85        # which catalog record
  end_effector.hold_variant: stage_a_90n        # optional named force setting
  motion_policy.transport: {...}                # optional transport_policy/v1
        │  export_builder_scene_to_cell_definition (passes through)
        ▼
cell_definition.yaml (generated handoff) ──► load_grasp_contract
        │                                         tool_interface   (end_effector_interface/v1)
        │                                         hold variant     (end_effector_hold/v1)
        │                                         transport_policy (transport_policy/v1)
        ▼
preplanner + executor ── ask the interface, never a vendor or joint name
```

| Block in the capability record | Module | Lifecycle step it owns |
| --- | --- | --- |
| `end_effector.interface` | `scripts/tool_interface.py` | Engagement (joint closure search, or IO signals), release goal, feedback (measured contacts or a vacuum signal), declared recovery, TCP link, touch links, aperture, planning group |
| `end_effector.hold` | `scripts/end_effector_hold.py` | Hold method (opposing contact, multi-contact, vacuum seal), hold reserve, actuator force and named force variants |
| `grasp_strategy.compatible_end_effector_capabilities` | `task_intent_v2.strategy_supports_tool` | Which grasp generators the tool may use |
| `interface.grasp_capabilities` | `task_intent_v2.installed_tool_capabilities` | What a task may require of the tool |

A cell with no catalog interface keeps its launched-model tool binding. Only an older contract with neither binding uses `tool_interface.LEGACY_TWO_FINGER`.

## Tool families

| Tool | Interface kind | Hold method | Planned engagement today |
| --- | --- | --- | --- |
| Robotiq 2F-85 | `joint_position` (gripper_finger1_joint, 0..0.804 rad, 80-step closure) | opposing contact | Yes |
| Robotiq 3F | `joint_position` (three proximal joints) | multi-contact (any 2 fingers) | No: `TOOL_CLOSURE_UNCOMMISSIONED` until its closure range is commissioned |
| OnRobot AirPick, vacuum array | `io_signal` (engage / release / feedback signals, release mode) | vacuum seal | No: `TOOL_ENGAGEMENT_UNSUPPORTED` until the planner has an IO engagement step |

Refusals happen at `GENERATE_GRASPS`, before any motion is planned.

## Selection and replacement

`scripts/end_effector_selection.py --scene DIR [--capability ID] [--hold-variant NAME] [--urdf F --srdf F] [--apply]` reports three verdicts separately:

1. **Catalog compatibility:** robot family, the task's required grasp capability and strategy, a valid interface and hold profile, and that the tool model matches the model the scene's description is generated from.
2. **Execution readiness:** whether the planner can express this tool's engagement.
3. **Description revalidation:** command joints actuated and within limits, the TCP link, collision geometry on every touch link, and the planning group.

`--apply` writes only the end-effector binding into the authored `environment.yaml`: capability, variant, and the TCP link and touch links from the interface. A different tool *model* is refused with `TOOL_MODEL_CHANGE_REQUIRES_SCENE_GENERATION`, because it changes the generated description, not just the binding. The executor repeats the description revalidation against the live MoveIt `robot_description` and `robot_description_semantic` on every run (`TOOL_REVALIDATION_FAILED`).

## Transport

`transport_policy/v1`, authored per cell:

```yaml
motion_policy:
  transport:
    schema: transport_policy/v1
    orientation: keep_level            # or free (the previous behaviour)
    tilt_tolerances_rad: [0.05, 0.15]  # bounded ladder, at most 3 attempts in total
    allow_unconstrained_fallback: false
    destination_approach_height_m: 0.2 # optional waypoint above the placement pose
```

`keep_level` plans PREPLAN_TRANSFER under an orientation path constraint on the tool link: tilt is bounded and yaw is free. It is allowed only when the tool's approach axis is vertical (`TRANSPORT_KEEP_LEVEL_UNSUPPORTED` otherwise). The existing controller-interval audit re-checks the constraint and collisions on the interpolated controller spline. Tolerances widen only along the authored ladder. An unconstrained attempt is added only when authored, and a budget or lifecycle stop is never retried. No joint path is named, so the policy applies to any robot. Without a policy, transport is unchanged.

## Experimental force settings

The 2F-85 hold profile keeps the Stage-A force settings as named variants: `stage_a_20n` (the default, frozen) and `stage_a_90n`. Both have status `experimental`, and neither is qualified as reliable or universally suitable (see the LENLEG 2026-10-24 Stage-A GO/NO-GO report). A cell selects one with `end_effector.hold_variant`. The variant is bound into the simulator receipt and verified by the executor.

## Functional workflow versus physical-grasp qualification

| Validation | Proves | Command |
| --- | --- | --- |
| Plan-only Resolve (this slice) | Certified nine-stage plan, tool revalidated against the live description | `run_stage_a1_finish.py --mode targeted --gate resolve` |
| Functional validation | One executed physical cycle with recorded (unchanged) retention limits | `run_stage_a1_finish.py --mode functional --prior-evidence RUN` |
| Strict Stage-A acceptance | All six gates under strict limits | `run_stage_a1_finish.py --mode full` |

The 24 October R3 experiments reported strict Stage-A **NO-GO**. Main subsequently acquired passing Stage-A evidence as described in `STAGE_B_REPLAY_PERCEPTION.md`; neither that evidence nor the historical R3 runs qualifies the reconciled revision without its own runtime checks.

## Vertical-slice evidence (LENLEG, 2026-10-24, commit 1018cc27)

Both runs are plan-only targeted Resolve sessions (`--world-profile sparse --search-timeout 900`) with the qualified
MoveIt and bridge overlays. The commissioning library hash is unchanged (`206554d0…`) and nothing moved. The runner marked the tree dirty
because this note was being written; no code differed from the commit.

| Session | Result | Tool / live revalidation | Transfer | Search |
| --- | --- | --- | --- | --- |
| `r3_slice_resolve_default_20261024_164632` | PASS: 9 certified stages in Resolve and in revalidation | `robotiq_2f_85`, group `gripper`, VALID (11 checks), hold variant `stage_a_20n` bound | Free (no policy) | 1 candidate (`finger_pinch_basic::000`) |
| `r3_slice_resolve_keep_level_20261024_164810` | PASS: 9 certified stages in Resolve and in revalidation | Same | `keep_level`: Resolve 0.05 rad failed, then 0.15 rad certified; revalidation certified at 0.05 rad | 5 candidates: keep-level transfer infeasible for ::000 and ::002 in discovery, PLACE wall rejections for ::001 and ::003 as before, ::000 resolved on retry |

Label: **CONFIRMED** for the two-finger plan-only path; **INFERRED** for three-finger and suction (catalog validation and unit tests only).
The transport constraint costs search time on this cell, so keep it opt-in per cell.

## Historical gaps from 24 October (not current acceptance claims)

- The fake-hardware plan-only Resolve of the reference task is **BLOCKED on the base branch**: candidate slices run out during certification, and the closure step fails. Evidence: `r3_baseline_fake_plan_20261024_162540`. Plan-only evidence therefore uses the simulator session.
- The intent vocabulary accepts only `two_finger_parallel`, as does the C++ task-intent model (GUI-owned). Three-finger and suction tasks need a coordinated vocabulary change.
- IO engagement and release steps (suction), commissioning of the 3F closure range, and scene generation from a capability record's tool model are not implemented.
- A HOME via-point fallback: the straight joint-space HOME certificate can be inconclusive at a robot self pair (forearm and wrist_2 at 2.0 mm). The bounded transport ladder does not cover HOME.
