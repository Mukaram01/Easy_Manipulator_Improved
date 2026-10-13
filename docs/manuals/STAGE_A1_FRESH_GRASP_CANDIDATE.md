# Stage-A1 — Fresh grasp candidate exclusion after measured physical retention loss

**Status: implementation only — physical acceptance NOT reached.**

## Why this exists

The preserved real Stage-A1 attempt selected `runtime::part_07` with grasp index 7,
passed a measured 1.101 s stationary hold, then lost one opposing fingertip contact
at simulator iteration 104156 during `EXECUTE_LIFT`. Cancellation and measured
scene reconciliation succeeded. Reusing a motion plan or sending a retreat from
this disturbed physical state is **not authorized**. The separate recovery
certificate is blocked by physical mimic follower / controller-start binding;
no recovery authority R was issued.

Evidence:
`evidence/stage_a1/retirement_20260925/release_failure/report.json`.
The report records object/grasp index but **not the strategy ref**. An engineer
must verify and explicitly supply the historical strategy when using it;
the code does not guess from the grasp number. The report's external SHA-256
declarations are retained history, not independent verification here.

## Bounded implementation

`scripts/stage_a_retention_candidate_veto.py` accepts only an operator-chosen
report of a measured physical opposing-finger PASS-to-FAIL transition during
`EXECUTE_LIFT`, with owned cancellation and measured reconciliation. It
extracts exact object ID, grasp index, an explicitly supplied strategy, and
the SHA-256 of the actual loaded report bytes.

The **existing** Stage-A `plan_authored_cycle` discovery may exclude exactly
that tuple (object, strategy, candidate index) from a **new simulator,
plan-only, fresh-resolution** candidate search:

- A matching candidate is recorded in the derived resolution as
  `MEASURED_RETENTION_LOSS`, marked nonretryable, and never sent to IK/MoveIt.
- Different objects, strategies and grasp indices are still eligible for the
  existing complete-cycle motion/collision tests. Their eventual physics
  success is **not presumed**.
- The veto cannot be passed to `--start`, to real-hardware or fake-backend
  execution, to replay, or to a resolved-grasp revalidation. The existing
  executable, support/contact thresholds, cancellation and recovery policies
  are unchanged.
- The Stage-A six-gate runner forwards the veto **only to the fresh resolve
  subprocess** of each new isolated session. Each subsequent prevalidated
  execution consumes the generated resolution normally, without a bypass.
- This is a conservative plan-space *exclusion*, **not** a physical stability
  classifier. A successful MoveIt plan or alternative candidate is still
  not a successful lift.

## Optional headless procedure (Ubuntu 22.04 / ROS 2 Humble)

First inspect the failure and the underlying historical resolution to
confirm the grasp strategy. Only then run:

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
source /opt/ros/humble/setup.bash
python3 -m pytest -q tests/test_stage_a_retention_candidate_veto.py \
  tests/test_stage_a1_finish_runner.py \
  tests/test_perceived_object_grasp_execute.py

python3 scripts/run_stage_a1_finish.py \
  --output /tmp/stage-a1-fresh-candidate-001 \
  --through resolve \
  --retention-failure-report evidence/stage_a1/retirement_20260925/release_failure/report.json \
  --retention-failure-strategy finger_pinch_basic
```

The strategy in the example is **conditional on checking the historical
resolution**. Change it if the record shows a different strategy. The output
directory must be new. The runner maintains its existing explicit simulator
identity and fake/real-hardware lock gates. Re-run the remainder of the
commissioning ladder only with independently verified prerequisites and a
fresh evidence directory; do not skip blocked gates to get a green result.

## Required evidence before claiming Stage A completion

1. TaskIntent, scene hash, original failure report hash, fresh candidate
   attempt ledger and selected alternative are present and unambiguous.
2. Fresh complete-cycle MoveIt plan and measured stationary retention with
   **both** required fingertip contacts.
3. Current-grasp contact remains valid during actual physical lift, transfer,
   landing and measured release (not merely a software attachment).
4. Owned goal cancellation remains effective for failure cases, and the
   current blocked recovery cannot send a command.
5. Full single-session physics acceptance, post-release world-state
   reconciliation, collision/ACM restoration, home, clean shutdown, and no
   real-hardware actuation all pass and are recorded.

**No ROS/Gazebo/Qt runtime, live physical grasp or full-cycle acceptance was
run by the author of this change.** This PR must remain draft until the
workstation provides genuine passing evidence. It does not advance the
separate uncertain detached-contact/mimic certificate.
