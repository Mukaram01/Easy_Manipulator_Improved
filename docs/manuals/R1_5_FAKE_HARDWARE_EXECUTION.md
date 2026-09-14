# R1.5 deterministic fake-hardware execution

**CONFIRMED on 2026-09-14:** clean GitHub main
`341a92dc88e3cfb2c26cf2a48901d52b5a200569` completed the canonical cup
pick/place sequence unattended, including retreat and home. No motion-planning,
geometry, scene, controller, or ACM-policy fix was needed. This document uses
R1.5 for the requested execution milestone, separate from the roadmap's future
operator-HMI heading.

The focused follow-up adds measured final home/placement/collision evidence and
an explicit execution mode to the existing acceptance runner. It also keeps the
ROS context alive during interrupt cleanup and uses idempotent shutdown.

## Reproduce

Use the existing MoveIt underlay at commit
`53979331bcaa472ddd3a3fc895907172df01374d`; no underlay files were modified.
Choose unused ROS domains and new output directories on subsequent runs.

```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/local_setup.bash
colcon build --packages-select workcell_builder ur5_2f_test --parallel-workers 2
source ~/workcell_ws/install/local_setup.bash
cd ~/workcell_ws/src/easy_manipulation_deployment
python3 -m pytest -q tests/test_runtime_pick_inputs.py \
  tests/test_transactional_pick_cycle.py tests/test_perceived_object_grasp_execute.py \
  tests/test_perceived_object_grasp_plan.py tests/test_r14_plan_only_acceptance.py
python3 scripts/run_r14_plan_only_acceptance.py \
  --timeout 120 --domain-id 196 --output-dir /tmp/r15-plan-only
python3 scripts/run_r14_plan_only_acceptance.py --execute \
  --timeout 900 --domain-id 197 --output-dir /tmp/r15-final
```

The execution runner launches exactly:

```bash
ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true \
  allow_trajectory_execution:=true launch_rviz:=false
```

It runs the installed canonical executor with `--start --replay`,
`config/runtime/r1_4b_task.yaml`, and `config/runtime/r1_4_replay.yaml`.
Installed executor/scene inputs must match this checkout. The runner sets
`ROS_LOCALHOST_ONLY=1` and its isolated `ROS_DOMAIN_ID`, waits for scene readiness,
allows up to 300 seconds for candidate search within the 900-second run budget,
and cleans up only its owned process groups. No camera or EPD process is used.

## Required evidence

`acceptance.json` requires all nine successful plans, unchanged live scene during
prevalidation, all nine successful ExecuteTrajectory results, and terminal
`SUCCEEDED` observations for nine MoveIt goals, seven arm-controller goals and
two gripper-controller goals. Accepted or executing goals cannot pass.

The selected object is `runtime::sample-cup`, grasp index 3, from
`canonical_box_geometry_robotiq_2f`. This is not ranked-cloud grasp consumption.
The bottle stays an unchanged collision obstacle. The executor validates each
measured transition against the prevalidated scene, including real collision
geometry, attachment link/touch links and ACM. Only configured fingertips may
contact the selected target; the baseline ACM is restored.

After grasp, PlanningScene evidence proves the cup absent from world and attached
once to `ee_palm`. After release it is present once in world with no attachment.
The destination is loaded from generated `cell_definition.yaml` and the replay
task's `default_drop_zone`: `[0.45, 0.22, 0.13]`. Placement uses measured FK and the
rigid attachment transform. The existing 3 mm placement threshold is retained.
Final joints are measured against canonical generated-cell safe home and gripper-open
state at 0.001 rad (approximately 0.057 degrees), tighter than the existing
0.005 rad intermediate-state check. A final MoveIt state-validity service query
must succeed under the restored baseline ACM.

The committed [machine-readable record](evidence/r15_fake_hardware.json) includes
baseline, final execution, plan-only, interrupt, build/test and file-hash evidence.
Full logs and four PlanningScene snapshots per completed execution are preserved
in `/home/ubuntu/workcell_ws/r15-evidence`; the interrupt driver is included there.
Two selected packages built and 93 focused tests passed. All recorded owned
process groups exited; no launch crashes remained.

## Interrupt behavior and remaining blocker

Only after normal success, SIGINT was deliberately sent two seconds into
`EXECUTE_RETREAT`. The executor reported FAIL/recovery-required, kept the cup
placed, inspected the scene with a valid ROS context, restored the baseline ACM,
and exited without a traceback or double shutdown. It did not command home.
Cancellation was attempted with bounded waits but **not confirmed**: the active
MoveIt action instead finished with terminal status 4 (SUCCEEDED).

The first interruption driver's assertion incorrectly required cancellation to
succeed. Its assertion failed; the executor itself exited cleanly with truthful
failure evidence. That raw diagnostic is retained, not relabelled as a cancelled
trajectory. The evidence separately marks the cleanup contract and cancellation
outcome.

The pinned MoveIt `execute_trajectory_action_capability.cpp` executes the blocking
trajectory callback in a mutually exclusive callback group. Its cancel callback
cannot run during that execution, and does not call its preempt helper. Prompt,
confirmed interruption of an active trajectory is therefore the **next real
product blocker**; the uninterrupted deterministic execution milestone is complete.
No MoveIt source was changed and no direct-controller bypass was introduced.
An already-invalid ROS context skips cancellation/recovery service calls and
uses `try_shutdown`; that external-context branch has not been separately fault
injected on this workstation.

Reverting this focused change removes the extra acceptance evidence and interrupt
cleanup handling. It does not change authored scene, grasp geometry, motion plans,
hardware defaults, or the known-good underlay. Fake-hardware execution remains an
explicit opt-in and is not physical-hardware commissioning evidence.
