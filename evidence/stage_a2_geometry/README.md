# Stage-A2 measured cube geometry, offline replay

Baseline: published Stage-A1 `8603cbda`. Raw capture and ROS logs are local at
`/tmp/workcell_stage_a2_acceptance`; the compact acceptance record is adjacent.
The original Stage-A1 checkout was not used for builds or edited.

PASS: real Ogre RGB-D, external EPD inference (5 detections ≥0.80), 4 valid
surface observations, 3 reconstructed collision boxes. One merged mask failed
localisation; another surface failed complete-face extent checks. Rejections are
not exported to planning. Independent timestamped simulator poses are used
only by the existing acceptance validator, never by reconstruction.

The operator explicitly associates the existing `environment_asset/v1` profile
with this batch: 25 mm cube, declared dimension tolerance ±0.25 mm. This is not
inferred from the class label. Segmented depth points, calibration, verified
static camera transform, surface centroid, fitted face normal/centre, declared
size, estimated pose and conditional uncertainty remain separate. The estimated
volume centre lies half the declared height along the inward measured normal.
Only a dominant, substantially complete top face supports this estimate. Face
yaw is bounded modulo cube symmetry; partial/merged/nonplanar faces fail closed.

Measured centre errors: 0.845–0.931 mm; orientation errors modulo cube symmetry:
0.292–0.464°. Conditional centre bounds: 2.668–2.879 mm; orientation bounds:
11.000–13.000°. Conservative box edges: 38.888–40.616 mm. All eight independent
simulator corners fit each envelope; declared dimensions match truth. These are
bounds for this verified ideal simulator camera and planar-face model, not a
calibration/noise qualification for physical cameras. Declared tolerance and
pixel support assumptions need separate verification for other workpieces.

PASS: existing normalized snapshot and PlanningScene box contracts. Real MoveIt
accepted all 3 objects through the existing runtime adapter. Two conservative
boxes were outside the authored pick selection; the remaining object generated
and evaluated `top_2f` candidates in the existing full-cycle preplanner.

BLOCKED: no full-cycle feasible grasp was established. The existing acceptance
runner reported FAIL after approach action/IK timeouts and its 120 s orchestration
deadline. Its raw reasons are retained in `acceptance.json`; no retreat, transport
or placement feasibility is claimed. No successful reachability/clearance claim
can be drawn from a timed-out attempt. Execution action goals were empty,
trajectory execution disabled, execution_attempted false and shutdown clean.

BLOCKED: scene completeness. Independent validation sees 10 cubes but only 3
have accepted geometry. This replay is suitable only for feasibility inspection;
it does not authorize physical execution. Generated inputs carry source.plan_only
and both replay and direct execution entry paths reject an execution request.
Frozen simulation observations remain frozen; only explicit replay creates a
wall-clock replay observation for the existing offline planner.

BLOCKED: live ROS–Gazebo bridge commissioning retains its existing qualification
prerequisite (baseline SIGABRT not reproduced). No bridge investigation or gate
change occurred. Robot/gripper execution and Stage-A1 retention are NOT RUN.

PASS: 158 focused Python tests, optional RGB-D C++ build and geometry CTest,
and generated disposable scene build. Builder C++/CMake is unchanged; the
previous full Humble Builder build is reused, with changed Python scripts staged
in a private prefix and checked by the existing runner's byte audit. AUTO,
PREFERRED and EXACT semantics remain under the existing resolver tests.

## Reproduce capture and geometry

From `/home/user/workcell_ws_stage_a2`, use the existing capture/build commands in
`docs/manuals/STAGE_A_RGBD_PERCEPTION.md`, with this checkout as the working
directory, `RGBD_RUN=/tmp/workcell_stage_a2_acceptance_new`, and a 30 s settling
period before bracketing poses. At its snapshot export step use:

```bash
python3 scripts/stage_a_rgbd_snapshot.py "$RGBD_RUN/capture/snapshot.json" \
  --output "$RGBD_RUN/capture/world_snapshot.json" \
  --camera-pose 0.4 -0.217 0.614 0 1.5707963267948966 0 \
  --workpiece-profile catalog/capabilities/environment_assets/asset_stage_a_cube_25mm.yaml \
  --replay-output "$RGBD_RUN/plan_only_replay.json"
PYTHONPATH="$RGBD_RUN/protos" python3 scripts/stage_a_rgbd/validate_capture.py \
  "$RGBD_RUN/capture" --poses-before "$RGBD_RUN/poses_before.json" \
  --poses-after "$RGBD_RUN/poses_after.json" > "$RGBD_RUN/independent_validation.json"
```

## Reproduce plan-only acceptance on this workstation

The existing authored scene selects bottles. Make a disposable cube/AUTO scene
using the existing authoring migration and generator; do not change the product
scene or the protected Stage-A1 checkout. No execution flag is used.

```bash
cd /home/user/workcell_ws_stage_a2
source /opt/ros/humble/setup.bash
export A2_RUN=$(mktemp -d /tmp/workcell-a2-plan.XXXXXX)
export A2_SCENE="$A2_RUN/scene/ur5_2f_test"
PYTHONPATH=scripts python3 - <<'PY'
import os, shutil, yaml
from pathlib import Path
from task_intent_authoring import load_authoring
from export_builder_scene_to_cell_definition import export_scene
from generate_workcell_from_cell_definition import generate_package
p = Path(os.environ['A2_SCENE'])
shutil.copytree('scenes/ur5_2f_test', p)
intent = load_authoring(p)['task_intent']
intent['pick']['selection']['object_filter']['class_id'] = 'cube'
intent['pick']['selection']['object_filter']['min_confidence'] = .8
intent['pick']['grasp']['policy'] = 'AUTO'
intent['pick']['grasp']['strategy_ref'] = None
(p/'config/workcell_builder_task_intent.yaml').write_text(yaml.safe_dump(intent, sort_keys=False))
export_scene(p, p, validate=True)
assert generate_package(p/'cell_definition.yaml', p.parent, p.name, False, False,
                        existing_package_dir=p) == 0
PY
colcon --log-base "$A2_RUN/log" build --base-paths "$A2_SCENE" \
  --build-base "$A2_RUN/build" --install-base "$A2_RUN/install" \
  --packages-select ur5_2f_test
source /tmp/pr3175_verify/install/setup.bash
source "$A2_RUN/install/setup.bash"
cp -a --reflink=auto /tmp/pr3175_verify/install/workcell_builder "$A2_RUN/workcell_builder"
cp scripts/{runtime_pick_inputs,perceived_object_grasp_execute,full_cycle_preplanner,stage_a_rgbd_snapshot}.py \
  "$A2_RUN/workcell_builder/lib/workcell_builder/"
export AMENT_PREFIX_PATH="$A2_RUN/workcell_builder:$AMENT_PREFIX_PATH"
python3 scripts/run_r14_plan_only_acceptance.py \
  --scene-dir "$A2_SCENE" --output-dir "$A2_RUN/planning" \
  --detections /tmp/workcell_stage_a2_acceptance/plan_only_replay.json \
  --resolve-task --candidate-wall-budget 300 --timeout 330 --domain-id 185
```

`/tmp/pr3175_verify/install` is the previously verified published-baseline Humble
Builder installation, not the protected checkout. On another workstation use a
Humble build of this branch instead. The runner fails closed on script or scene
byte mismatches. An expected timeout is not a PASS: inspect `acceptance.json`.
Next product action: diagnose the bounded MoveIt approach/IK failure from these
logs, then rerun this same acceptance. Do not shrink conservative envelopes or
change bridge gates to force a successful result.

## Timeout diagnosis — 2026-10-10

The original 12 s action response wait and 0.75 s discovery / 20 s retry cycle
slices excluded the cost of controller-spline certification. With the same
EPD replay, startup completed before planning; idle scene/FK/IK/validity services
responded in 1–4 ms and collision-aware home-pose IK returned SUCCESS. During an
active plan, advertised FK and validity services did not respond within 12 s.
The existing controller certificate consumed 62.801 s and 38.030 s before
returning UNCERTIFIED/PRECISION_OR_DEPTH_LIMIT. A later run certified an actual
approach in 23.130 s and MoveIt returned SUCCESS after the caller had timed out.
This establishes a response/candidate budget mismatch, not unreachable IK.

The client also continued candidate search with cancellation unconfirmed.
Transport failures now retain their own reason codes, owned-goal/cancellation
evidence and BLOCKED status. Discovery, queued retries and extraction alternatives
stop on a transport failure. Diagnostic validity failures cannot be swallowed
into a geometric rejection. Normal returned MoveIt failures retain their existing
AUTO/PREFERRED/EXACT handling; a returned TIMED_OUT code is distinct from a
missing action response.

The existing runner accepts optional `--candidate-wall-budget` for plan-only
acceptance, bounded to 300 s. It allocates the full candidate cycle, including
certification, and clamps server/acceptance/result waits to the same absolute
candidate/global deadline. Separate bounded cancellation/cleanup remains.
Execution requests reject this option. Default timing and all collision gates
remain unchanged. The workstation command above uses 300 s based on the measured
23–63 s certificate cost; OMPL segment computation budgets are unchanged.

The bounded corrected-budget run and final stage outcomes are recorded in
`timeout_diagnosis.json`. Raw local logs and the existing adapter's opt-in CDR
trace are at `/tmp/workcell_stage_a2_certified_budget`. To retain that trace on
the next run, set `WORKCELL_PLANNING_TRACE_DIR="$A2_RUN/planning/adapter_trace"`
before invoking the same acceptance runner. No additional validator was added.

Latest result: the response-timeout cascade is corrected for explicit plan-only
acceptance. The bounded run finished in 291.528 s with actual MoveIt responses,
13 distinct candidates / 19 attempts. Seven attempts passed certified approach
and Cartesian descent. All seven failed closing with INVALID_MOTION_PLAN and
controller PRECISION_OR_DEPTH_LIMIT, including [0,1] ns intervals. Five attempts
returned planner TIMED_OUT (-6), six other approaches returned INVALID_MOTION_PLAN
(-2), and the side-grip attempt returned IK NO_SOLUTION (-31). None of these is
reported as a missing response or a proven geometric collision.

Full acceptance: FAIL. Grasp descent: PASS; grasp closing: FAIL; retreat and
placement: BLOCKED/not reached. Zero execution action goals, execution_attempted
false, trajectory execution disabled, clean shutdown, no owned processes left.
The same two objects remain outside authored selection. No rejected observation
was substituted, no geometry was shrunk, and incomplete coverage still prohibits
execution. Geometry/profile/bridge/native sources were unchanged in this fix.

Verification: 287 focused tests plus 6 additional CLI guard cases PASS;
Python compilation PASS. No native build was rerun because no native/CMake or
dependency changes were made. The final absolute-deadline clamp was completed
after the long runtime started and tested with simulated setup delay; the runtime
finished inside its global budget. Its exact executed script hashes are retained;
this is not a complete current-head feasibility PASS.

Next action: inspect the saved closing request and private scene in the existing
adapter trace, identify the pair that remains uncertified at the reported interval,
and correct only a demonstrated certification/clearance defect. A depth/precision
limit is not proof of collision. Then run the command above again. Do not loosen
certification, geometry, contact rules, task selection or bridge qualification.
