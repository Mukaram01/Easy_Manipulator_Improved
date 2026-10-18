# Stage-A1 — Mimic follower authority for the simulated Robotiq 2F-85

**Status: implementation and offline tests only. Physical acceptance NOT reached.
No ROS/Gazebo runtime was available to the author of this change.**

## First physical blocker

Preserved evidence (`evidence/stage_a1/retirement_20260925/release_failure/report.json`):
`runtime::part_07`, grasp 7, passed a 1.101 s stationary hold. At `EXECUTE_LIFT`,
iteration 104156, `gripper_finger1_finger_tip_link` lost contact while finger 2
stayed in contact. Relative tool/object motion was 3.956 µm / 8.54e-7 rad. Tool z
(0.190006 m) and object z (0.0375 m) had not changed yet, so the contact was lost
at the very start of the lift. It was not lost during carry.

## Root cause

In the URDF tree the Robotiq four-bar loop is broken. Each contacting fingertip
hangs off `gripper_finger*_inner_knuckle_joint` → `gripper_finger*_finger_tip_joint`.
Both are passive mimic followers of `gripper_finger1_joint`. The actuated leader
(effort 1000 N·m) drives only the outer knuckle branch, which does not hold the
pads.

The source xacro was written for the Gazebo-classic kinematic mimic plugin. It
gives those followers `effort="0.1"`. The Stage-A simulator derivation removes
that plugin and uses `ign_ros2_control` mimics instead. These track the measured
leader through a `JointVelocityCmd` servo. DART limits that servo's force to the
follower's SDF effort limit. The actual derived Stage-A URDF
(`continuous_jtc/withdrawal_fixture/robot.urdf`) confirms that all four
pad-bearing followers keep 0.1 N·m.

The 0.0101 rad closure reserve keeps those servos saturated. As a result:

- each pad presses with at most about 0.1 / 0.0571 m ≈ **1.75 N**. The
  Robotiq 2F-85 specification is 20–235 N.
- the finger links' own weight, the lift acceleration and the part's
  0.196 N weight use a large share of a servo that has no reserve left.

## Correction

`simulator_backend.mimic_follower_authority` models the rigid linkage. Before
`ign_ros2_control` derivation, every mimic follower's effort limit is raised to
the leader's transmitted torque, τ_f = τ_l / |multiplier|. The new limits are:

| joint | source effort | derived effort |
|---|---|---|
| gripper_finger2_joint | 1000 | 1000 (unchanged) |
| gripper_finger{1,2}_inner_knuckle_joint | 0.1 | 1000 |
| gripper_finger{1,2}_finger_tip_joint | 0.1 | 1000 |

The change has these limits:

- Effort limits are only raised, never lowered.
- Positions, velocities, couplings, MoveIt, the measured 0.02 rad mimic-coupling
  check and every contact, slip, freshness, ACM and pile guard are unchanged.
- A follower or leader without a finite effort limit fails closed.
- The records are written to the simulator receipt as `mimic_follower_authority`,
  and the URDF hash binds them.
- Grip force is now governed by the single declared leader actuator, as on the
  real device.

Retention evidence gains a diagnostic, `max_mimic_coupling_error_rad`. It is the
largest measured follower error while the part is carried. It does not change
any threshold.

## Workstation qualification (Ubuntu 22.04, Humble, Fortress)

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
git fetch origin claude/ur5-physical-execution-pv367y
git checkout claude/ur5-physical-execution-pv367y
source /opt/ros/humble/setup.bash && source ~/workcell_ws/install/setup.bash
python3 -m pytest -q tests/test_simulator_backend.py tests/test_simulator_execution.py \
  tests/test_stage_a1_finish_runner.py tests/test_stage_a_retention_candidate_veto.py
# Python-only change: no colcon rebuild is required for it.
OUT=~/workcell_ws/stage-a1-follower-authority-$(date +%Y%m%d-%H%M%S)
python3 scripts/run_stage_a1_finish.py --output "$OUT" --through full-cycle
jq .mimic_follower_authority "$OUT"/*/runtime/receipt.json | head -40
```

Run it once without the candidate veto, so that the same part_07 / grasp 7
candidate can be reselected. That is the controlled A/B against the historical
failure. Then classify the run:

- **Hypothesis confirmed:** contact-release passes LIFT with both fingertip
  contacts and the unchanged slip limits. Then continue to full-cycle and repeat
  it from clean sessions.
- **Hypothesis rejected:** the same single-fingertip loss happens at LIFT with
  derived effort 1000. Preserve the first-violation sample, including the
  follower joint positions in `summary.measurements.jsonl`. Then run the veto
  procedure in `STAGE_A1_FRESH_GRASP_CANDIDATE.md`.
- **New failure:** for example, high squeeze disturbs the pile during CLOSE, or
  DART is unstable at the higher force. Record it as the new first failure.
  The physically meaningful knob is then the leader's declared actuator effort
  (the 2F-85 spec is 20–235 N, about 1.1–13.4 N·m at this lever). Do not relax
  any guard.

No PASS may be claimed from this document or from the offline tests.
