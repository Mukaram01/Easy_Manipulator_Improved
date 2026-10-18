# Stage-A1 — Mimic follower authority for the simulated Robotiq 2F-85

**Status: UNVALIDATED SIMULATION HYPOTHESIS. It is not a confirmed root-cause fix.**
Only offline tests were run. No ROS/Gazebo runtime was available to the author.
Physical acceptance has NOT been reached. The PR must stay Draft until this
procedure produces fresh passing evidence.

## Observed first failure (preserved evidence)

`evidence/stage_a1/retirement_20260925/release_failure/report.json`:

- `runtime::part_07`, grasp 7, passed a 1.101 s stationary hold.
- At `EXECUTE_LIFT`, iteration 104156, `gripper_finger1_finger_tip_link` lost
  contact. Finger 2 stayed in contact.
- Relative tool/object motion was 3.956 µm / 8.54e-7 rad, inside the slip limits.
- Tool z (0.190006 m) and object z (0.0375 m) had not changed yet.

## Coupling model as actually simulated

These facts were verified from source and from the derived Stage-A URDF
`continuous_jtc/withdrawal_fixture/robot.urdf`.

- The URDF tree breaks the Robotiq four-bar loop. Each pad is carried by
  `gripper_finger*_inner_knuckle_joint` → `gripper_finger*_finger_tip_joint`.
- `gripper_finger1_joint` is the only commanded joint (effort 1000). It drives
  the outer-knuckle branch, which carries no pad.
- `simulator_description` removes the classic kinematic mimic plugin. It declares
  the five followers as `ign_ros2_control` mimics.
- `ign_ros2_control` (Humble) enforces each mimic **independently**. It sends a
  `JointVelocityCmd` of `-(q_follower - m*q_leader_measured) * update_rate`, and
  `update_rate` is 100 Hz in `scenes/ur5_2f_test/config/ros2_controllers.yaml`.
  DART runs that command as a servo whose force is capped by the follower's SDF
  `<effort>`.
- The pads therefore hold the part through four independent servos: two inner
  knuckles and two tips. No closed-chain force transmission exists, unlike the
  real gripper.
- The closure reserve (0.01005 rad past first opposing contact) keeps the pad
  servos commanded beyond contact. They saturate at their effort cap, so **the
  follower effort cap is what sets grip force** in this model.

## Grip-force bound (static, lever 0.0571 m knuckle axis → tip joint)

| follower effort cap | pad normal-force bound | compared with the 2F-85 spec of 20–235 N |
|---|---|---|
| 0.1 N·m (source; historical run) | ≤ ~1.75 N | ~11× below the minimum |
| 1000 N·m (this PR) | ≤ ~17.5 kN | ~75× above the maximum |

These are upper bounds from static torque balance. The pad sits beyond the tip
joint, so the real lever is longer and the bound slightly lower. They are not
measured forces. The measurement plugin publishes contact **positions only**, so
the joint_state_broadcaster here exports only position and velocity. This
pipeline has no measured contact force or joint effort.

The change raises follower caps from 0.1 to τ_l/|m|. Treat it as a test of one
thing: whether the 0.1 N·m cap explains the LIFT loss. It does **not** model
realistic Robotiq grip force. Expected risks of the 1000 N·m value:

- the first pad to touch pushes the part hard before the opposing pad arrives,
  which can disturb the pile during CLOSE;
- very large opposing constraint forces on a 20 g part may make DART unstable;
- tip and knuckle servos with equal caps keep the same static geometry, so
  edge-contact behaviour may not change.

If this hypothesis is confirmed, a later change must replace 1000 N·m with a
reviewed effort in the 20–235 N range (about 1.1–13.4 N·m at this lever) and
requalify. That is out of scope here.

Unchanged: all contact, slip, freshness, ACM, pile, support and collision gates;
the 0.02 rad measured coupling check; MoveIt; the recovery lock (R not issued,
A revoked); `real_hardware: false`. Retention evidence gains only the diagnostic
field `max_mimic_coupling_error_rad`.

## Workstation acceptance procedure (Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress 6)

Run the steps in order. **Stop at the first step that fails, and record that
failure.** Each runner call uses a new output directory and starts fresh
isolated sessions. Never edit a guard, threshold or evidence file to continue.
Do not pass `--retention-failure-report`. The run must be able to reselect
part_07 / grasp 7 for the controlled A/B.

### 0. Build, tests and environment

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment
git fetch origin claude/ur5-physical-execution-pv367y
git checkout claude/ur5-physical-execution-pv367y && git rev-parse HEAD
source /opt/ros/humble/setup.bash && source ~/workcell_ws/install/setup.bash
ign gazebo --versions          # expect 6.x
python3 -m pytest -q tests/test_simulator_backend.py tests/test_simulator_execution.py \
  tests/test_stage_a1_finish_runner.py tests/test_stage_a_retention_candidate_veto.py \
  tests/test_grasp_recovery_boundary.py
export RUNS=~/workcell_ws/stage-a1-follower-authority-$(date +%Y%m%d-%H%M%S); mkdir -p "$RUNS"
```

PASS means every test passes on the workstation. Python-only change: no colcon
rebuild is needed for it.

### 1. Controller and coupling verification (`--through telemetry`)

```bash
python3 scripts/run_stage_a1_finish.py --output "$RUNS/1-telemetry" --through telemetry
R=$(ls "$RUNS"/1-telemetry/*-telemetry/runtime/receipt.json)
jq '.hardware_class, .mimic_follower_authority' "$R"
python3 - "$(dirname "$R")/robot.urdf" <<'PY'
import sys,xml.etree.ElementTree as ET
r=ET.parse(sys.argv[1]).getroot()
for j in r.findall('joint'):
    m=j.find('mimic')
    if m is not None or j.get('name')=='gripper_finger1_joint':
        print(j.get('name'),j.find('limit').get('effort'),m.attrib if m is not None else 'LEADER')
for c in r.findall('ros2_control'):
    for j in c.findall('joint'):
        if 'gripper' in j.get('name'):
            print(' ',j.get('name'),[i.get('name') for i in j.findall('command_interface')],[p.text for p in j.findall('param')])
PY
```

PASS requires all of the following:

- `hardware_class` is `ign_ros2_control/IgnitionSystem`.
- The five followers show derived effort 1000 and keep their multipliers
  (`finger2`/`inner_knuckle` 1, `finger_tip` −1).
- Only `gripper_finger1_joint` has a command interface.
- The gate reports `MOTION_TELEMETRY_PASS`, `real_hardware: false`, and clean
  shutdown with all children exiting 0.

### 2. Measured close, contact and stationary retention (`--through stationary`)

```bash
python3 scripts/run_stage_a1_finish.py --output "$RUNS/2-stationary" --through stationary
S=$(ls -d "$RUNS"/2-stationary/*-stationary)
jq '.result, .retention' "$S"/stationary/summary.json
```

Run this coupling and contact analysis on the stationary trace. It uses only
existing recorded samples:

```bash
python3 - "$S"/stationary/summary.measurements.jsonl <<'PY'
import sys,json
L='gripper_finger1_joint';F={'gripper_finger2_joint':1,'gripper_finger1_inner_knuckle_joint':1,
 'gripper_finger2_inner_knuckle_joint':1,'gripper_finger1_finger_tip_joint':-1,'gripper_finger2_finger_tip_joint':-1}
worst={};phases={}
for line in open(sys.argv[1]):
    rec=json.loads(line);s=rec['measurement'];ph=rec['phase']
    j={k.split('::')[-1]:v for k,v in s['joints'].items()}
    for f,m in F.items():worst[(ph,f)]=max(worst.get((ph,f),0),abs(j[f][0]-m*j[L][0]))
    tips=sorted({n.split('::')[2] for c in s['contacts'] for n in (c['a'],c['b']) if 'finger_tip' in n})
    phases.setdefault(ph,[]).append((s['iteration'],j[L][0],tips))
for (ph,f),e in sorted(worst.items()):print(f'{ph:12s} {f:40s} max|err| {e:.5f} rad')
for ph,v in phases.items():print(ph,'samples',len(v),'first',v[0],'last',v[-1])
PY
```

PASS requires all of the following:

- `STATIONARY_RETENTION_PASS`, with retention of at least 1 s of simulated time.
- Both `gripper_finger1_finger_tip_link` and `gripper_finger2_finger_tip_link`
  in contact with the target.
- Relative slip within the unchanged 2 mm / 0.01 rad limits.
- The post-close pile admission certificate accepted, with no unpermitted
  contact.
- Every follower error ≤ 0.02 rad. Record `max_mimic_coupling_error_rad`.

Also compare the target pose at first opposing contact with its pose at the
retention start. A pile disturbance during CLOSE large enough to break the
admission certificate counts as a **failure of the 1000 N·m hypothesis**. It is
not a guard to relax.

### 3. Lift with retained contact (`--through contact-release`)

```bash
python3 scripts/run_stage_a1_finish.py --output "$RUNS/3-contact-release" --through contact-release
C=$(ls -d "$RUNS"/3-contact-release/*-contact-release)
jq '.result, .selected_object_id, .selected_grasp_index, .failure, .failed_stage' "$C"/contact-release/summary.json
```

Run the step 2 analysis script on `"$C"/contact-release/summary.measurements.jsonl`.

PASS requires all of the following:

- `CONTACT_RELEASE_PASS`.
- Both fingertip contacts held on every sample through `EXECUTE_LIFT` and
  transfer, with slip inside the limits.
- The owned OPEN goal reaches SUCCEEDED.
- A fresh post-terminal sample shows the gripper open and both required
  fingertip contacts absent.
- Measured PlanningScene detachment, then free settling.

Note the selected object and grasp. The A/B is valid only if this is part_07 /
grasp 7, or another candidate that failed historically.

### 4. Full physical cycle (`--through full-cycle`)

Run this only if steps 1–3 passed.

```bash
python3 scripts/run_stage_a1_finish.py --output "$RUNS/4-full-cycle" --through full-cycle
jq . "$RUNS"/4-full-cycle/stage-a1-final-report.json
```

PASS requires all of the following:

- All six gates pass in this one fresh run.
- The final report shows: approach, close, opposing contact, lift, transfer,
  place, measured release, destination containment and settling, retreat,
  home, PlanningScene reconciled with no attachment, clean controller and
  simulator shutdown, no surviving owned processes, and `real_hardware: false`.
- Then repeat step 4 from a new directory at least twice more. Report the exact
  count, for example "3/3" or "2/3 with the failure stage". Do not report a
  percentage built from one run.

### Reporting the first failure

For the first failing step, report:

- the gate;
- `failed_stage`;
- iteration and sim_ns;
- missing contacts, with translation and rotation;
- follower errors at that sample, from the analysis script;
- the selected object and grasp;
- the evidence paths with SHA-256.

Interpret it as follows:

- **Same single-fingertip loss at LIFT with followers at 1000 N·m:** the
  hypothesis is rejected. Restore nothing. The next candidate path is the #3183
  veto procedure in `STAGE_A1_FRESH_GRASP_CANDIDATE.md`.
- **Pile disturbance or instability at CLOSE:** the hypothesis worked in the
  wrong direction (too much force). The next change is a reviewed realistic
  follower effort, as described above.
- **Step 1 fails:** controller or coupling setup is wrong. Fix that before any
  grasp claim.
