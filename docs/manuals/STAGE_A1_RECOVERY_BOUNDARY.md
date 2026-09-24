# Stage A1 recovery boundary after grasp retention loss

## Scope and evidence

This is the architecture map for Stage A1 post-grasp-failure recovery at
`b29be27c655a88cac8061ec8232ef75cb8e9dad3`. It describes the existing
simulator executor and the deliberately limited recovery qualification boundary.
The latest recorded lift failure was `REAL_SEPARATION`: required opposing
finger contact was lost during lift. That physical event invalidates the
resolved grasp and motion assumptions even if contact later returns.

This document does not claim a successful retreat, fresh retry, full cycle,
simulation acceptance, or physical acceptance. It authorizes no motion.
The real-execution lock, commissioning gates, contact guard, collision
thresholds, and safety limits remain in force.

## Before: one initial-state cycle

This section and the reuse table below describe the pre-change execution path.

1. `perceived_object_grasp_execute.main` reads one `--detections` snapshot,
   normalizes it with `runtime_pick_inputs.normalize`, and selects eligible
   observations. Simulator mode binds the snapshot to the current receipt.
2. `wait_for_robot_baseline` requires canonical home and open gripper. The
   executor inserts observations, reads the planning scene as `initial`,
   checks home joints and object geometry, and copies its baseline ACM.
3. `plan_authored_cycle` consumes `scene_resolution` during execution.
   A generated resolution binds the selected object, candidate, approach IK,
   transfer IK seed, extraction intent, and physical destination.
4. `preplan_full_cycle` copies `initial` into a private predicted scene.
   It derives the grasp, object-in-tool relation, attachment, lift,
   transfer, placement, retreat, and home from that observation and scene.
   It returns nine planned motion stages plus attach/detach records.
5. `plan_segment` binds each MoveIt request to the predicted stage's
   `view.robot_state`; the returned cycle stores executable trajectories.
   Some lift checks also refer to the original `initial` collision objects.
6. The executor checks the live scene against `initial`, then walks the
   stored cycle. It does not resolve a second cycle inside this invocation.

The candidate search's internal retry is a planning retry for the **same**
candidate and scene. `plan_authored_cycle` may reuse a proved IK branch or
cached evaluation inside that search. It is not a post-disturbance retry.

## Before: contact-loss and cleanup path

`ContactGuard.check` and `HeldObject.check` reject loss of required opposing
contact or excessive object-in-tool slip. The execution monitor propagates
that error from the active trajectory action. The executor requests
cancellation of its owned action goal, checks terminal status, verifies a
measured stationary window, restores the baseline ACM, and calls
`measured_reconcile`. `simulator_execution.cancel_owned` requires confirmed
cancellation and stop before reconciliation and revokes the ACM allowance
even when cancellation verification fails.

`measured_reconcile` reads a fresh measured pose for the selected object and
reconciles its attachment/world placement. It checks selected-object geometry
and the ACM. The outer failure/finally path records the failure, repeats
safe cleanup inspection when ROS remains alive, and exits. It does not
command retreat, capture new perception, or re-enter resolution.

Reconciliation is narrower than a new scene observation: it does not prove
that every disturbed pile object has an updated planning-scene pose.
Cancellation, stationary verification, and ACM restoration are necessary
preconditions, not permission to move.

## Reusable mechanisms and their limits

| Mechanism | Reusable contract | Initial-state assumption or missing gate |
| --- | --- | --- |
| `cancel_owned`, `wait_stopped` | Stop and verify the owned action and measured stationary window. | A failed confirmation forbids recovery motion. |
| `measured_reconcile` | Revoke contact allowance and reconcile the selected object's measured pose. | It uses the original object geometry and does not refresh the whole pile. |
| `scene_now`, `assert_scene_match` | Read and compare robot, world, attachment, and ACM state. | A comparison against the old predicted scene is invalid after disturbance. |
| `plan_segment` | Collision-aware MoveIt planning from a supplied scene's robot state. | Current closure references `initial` for lift checks; a future attempt needs a new context. |
| `preplan_full_cycle` | Build and validate nine stages from a supplied start scene and observation. | Its steps and trajectories are bound to that supplied scene and candidate. |
| `resolve_task_intent(..., resolved=None)` | Enumerate new candidates from supplied observations. | The supplied observations and scene must first be fresh and mutually reconciled. |
| `plan_authored_cycle(..., resolved=None)` | Create a new local candidate search and cycle. | Its same-scene search cache/IK seeds must not cross recovery attempts. |
| `scene_resolution`, `consume_resolution` | Validate generated task handoff and saved resolution integrity. | `resolved` revalidates the old selected candidate and bound IK/extraction data. |
| `simulator_observations.capture` | Capture settled Fortress model poses against the live receipt. | No executor call exists; `refresh_from` rejects changed geometry and only renews timestamps. |
| `capture_epd_detected_objects.main` | Obtain and TF-normalize an EPD message in a separate capture command. | No executor call exists; receipt and post-stop freshness need a new contract. |
| `runtime_pick_inputs.scene_diff` | Build planning-scene ADD diffs for normalized objects. | `observations_to_insert` rejects changed duplicate simulator IDs; full dynamic-world reconciliation is absent. |

`wait_for_robot_baseline` is an initial preflight check, not a recovery motion
primitive. `PREPLAN_RETREAT` and `PREPLAN_HOME` belong to the pre-failure
cycle; neither trajectory can be executed after disturbance.

## Implemented boundary

`GraspRetentionLoss` is the typed opposing-contact/slip failure, with a copy
of its rejected measurement. `begin_recovery` creates `RetentionRecovery`,
clears the resolved cycle, marks prevalidation false, and rejects every later
`ExecuteTrajectory` call through the `action` gate. It records historical
candidate, resolution, execution-attempt, and raw observation-file SHA-256
identities as evidence only. No old plan becomes an input to recovery.

During an active goal, `cancel_owned` requires accepted owned cancellation,
terminal status, and a new measured stop before reconciliation; ACM revocation
remains unconditional. `begin_recovery` clears any earlier stop/cancel proof.
`RetentionRecovery.confirm_stop` also requires the stop sample to postdate
the rejected loss sample in the same simulator run and process. A loss during
a hold obtains its own post-failure stop window.

`retention_reconciliation_scene` uses one post-stop simulator sample for every
bound pile object's pose and every robot joint. It rejects missing object
coverage, unknown attachments, missing live model joints, moving/nonfinite
joints, and invalid object geometry. `reconcile_retention_loss` applies the
baseline ACM and measured world, verifies the readback, removes the selected
attachment, and records sample, robot-state, scene, and receipt SHA-256
evidence. It queries current-state collision validity and physical contacts.
The terminal `RetentionRecovery.qualify_retreat` records the reason and always
enters `RECOVERY_BLOCKED`; repeated cleanup does not advance it. There is no
retreat command, fresh capture, fresh resolution, or retry continuation.

## Decision: stop at retreat qualification

The current implementation scope ends after cancellation, stop verification,
attachment/ACM/world reconciliation, and an explicit attempt to qualify
retreat. The qualification result is `BLOCKED`; it must cause a clean,
stationary exit with a specific reason. There is no authorization to send a
fallback Cartesian jog, reuse `PREPLAN_RETREAT`, or treat home as inherently
safe.

The immediate blocker is the post-loss contact state. Once the object is
detached in the planning scene and the baseline ACM is restored, the measured
fingers/tool may still touch or overlap the selected object or disturbed
neighbors. A valid robot joint reading and a fresh object pose do not prove a
collision-valid planning **start** state. The existing planner has no
qualified generic egress primitive that can begin at this contact state,
preserve collision checks, and establish a safe separation corridor. A
retreat cannot be justified until that start-state/contact policy and full
dynamic scene geometry are explicitly validated.

The recovery boundary must therefore fail closed when the measured start is
collision-invalid, the scene is incomplete, an attachment remains, the ACM
differs from baseline, stationary status is unverified, or no collision-aware
retreat can be qualified. Its outcome is a recovery failure, not a candidate
search failure. No retry count is consumed by merely evaluating a blocked
retreat, and no second attempt begins.

## Future contract beyond this boundary

The following stages are **unimplemented** and must remain unreachable while
retreat qualification is blocked:

```text
EXECUTING
  -> CANCEL_AND_STOP
  -> FAILURE_RECONCILE
  -> QUALIFY_RETREAT
  -> [BLOCKED: CLEAN_EXIT]
  -> RETREAT -> RETREAT_VERIFY
  -> OBSERVATION_POSITION -> OBSERVATION_VERIFY
  -> FRESH_OBSERVATION -> FRESH_SCENE
  -> FRESH_RESOLUTION -> RETRY_GATE -> EXECUTING
```

When an egress primitive is proven, a future attempt must use an explicit
attempt context. The context begins with verified current measured robot
state, no attachment, restored baseline ACM, and a complete, current
planning scene. Retreat and travel to a known observation pose must each
be newly planned against that state/scene and checked after execution.

Fresh capture must occur after observation-position verification. For the
simulator, call `simulator_observations.capture` without `refresh_from` to
accept genuinely changed measured poses, likely through a separate process
because it owns its own `rclpy` lifecycle. For EPD, use the capture adapter's
source timestamp, frame, TF result, and object identities. Never renew an old
snapshot's timestamps and call it a new observation. Reject absent, stale,
pre-stop, frame-invalid, or receipt-mismatched captures.

Reconcile **all** dynamic objects: add/update current detections, remove
objects no longer present only under an explicit policy, and read the live
scene back to verify world geometry, attachments, ACM, and robot state.
`observations_to_insert` cannot perform this changed-geometry refresh.

Run `resolve_task_intent` and `plan_authored_cycle` with `resolved=None`,
newly normalized observations, a newly reconciled scene, and a fresh search
deadline. The prior `expected_resolution`, selected candidate, grasp/tool
transform, IK seeds, extraction intent, local search cache, cycle steps, and
trajectories must be inaccessible to the new attempt. A candidate with the
same semantic ID may reappear only by new enumeration and normal gates.
The existing generated-resolution handoff must be updated or an equivalent
same-attempt gate added before execution; the current executor requires a
saved resolution and forbids `--resolve-task` with `--start` commissioning.

Record one provenance entry per attempt: attempt number, previous candidate
and abandonment reason, simulator receipt or EPD source, capture identity
and source timestamp/sample sequence, raw observation digest, measured
robot-state digest, scene digest/revision, selected candidate identity, and
resolution identity. Require the new capture to postdate the verified stop
and observation pose. `task_intent_resolver` excludes timestamps from its
observation hash, so that hash alone cannot prove fresh capture.

Stage A1 may permit **one** recovery retry only after all preceding gates
pass. A second retention loss must cancel, stop, reconcile, and exit cleanly
without recursion. Planning retries within one initial scene do not count
as recovery retries and cannot authorize reuse across the boundary.

## Test applicability at the qualification boundary

The original A–N recovery test list remains the acceptance target. The
current limited scope can establish only the first safety boundary:

| Test | Boundary expectation |
| --- | --- |
| A. Contact loss -> cancellation | Applicable: assert owned cancel, terminal result, measured stop, and qualification transition. |
| B. No old objects reused | Applicable: old candidate/cycle/trajectory must be inaccessible after entering recovery. |
| C. Current-state start | Applicable to qualification input only; no retreat execution claim. |
| D. Reconcile attachment/ACM | Applicable: verify no attachment and baseline ACM or block. |
| E. Failed retreat qualification | Applicable: explicit stationary recovery failure and no motion. |
| F. Failed observation motion | Blocked: observation motion is unreachable. |
| G. Failed fresh perception | Blocked: fresh capture is unreachable. |
| H. Failed fresh resolution | Blocked: fresh resolution is unreachable. |
| I. New observation provenance | Blocked: no post-failure capture yet. |
| J. Bounded retry | Applicable only as zero retries while qualification is blocked. |
| K. Successful recovery chain | Blocked: no qualified retreat. |
| L. Second failure after retry | Blocked: no first retry. |
| M. Normal no-failure execution | Applicable regression; preserve the existing path. |
| N. Cancel/telemetry/stationary/contact-release regressions | Applicable regression; do not weaken existing guards. |
