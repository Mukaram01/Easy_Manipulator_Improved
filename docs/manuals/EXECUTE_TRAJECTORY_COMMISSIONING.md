# Simulator commissioning ExecuteTrajectory capability (MoveIt 2.5.9)

This opt-in correction keeps the existing MoveIt trajectory execution manager
(TEM), controller manager, controller handles and `/execute_trajectory` action.
It does not authorize ordinary simulator or real-hardware execution. Physical
pick/place remains uncommissioned.

## Provenance and scope

Reviewed upstream tag **2.5.9**:

- [ExecuteTrajectory capability](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/move_group/src/default_capabilities/execute_trajectory_action_capability.cpp)
- [Trajectory execution manager](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp)
- [Existing action controller handle](https://github.com/moveit/moveit2/blob/2.5.9/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/action_based_controller_handle.h)

The upstream capability blocks its single mutually exclusive callback executor
in accepted-goal work. Its cancel callback accepts without calling TEM stop,
and its completion path has no canceled terminal transition. The reviewed
installed package is `ros-humble-moveit-ros-move-group` version
`2.5.9-1jammy.20260804.221133`; the installed default capability SHA256 is
`7b6f41c43a079b13ef137033f6a83db97e11629dac661903caf28bde1179542d`.
The exact upstream ExecuteTrajectory source SHA256 is
`bbaf7c13a18c3a40b7389d0676033457616b5885e91b7748ab4902e1b19e0943`.

The correction reserves one UUID, rejects overlap, and uses joined execution,
stop and callback workers. Start decision and cancellation share a mutex;
blocking preparation/start/wait/stop never holds the callback mutex. A cancel
during start is acknowledged promptly and forwarded when TEM start returns.
Cancellation before start prevents start. Stop and completion synchronize
before the single terminal transition. After TEM's result wait and join, a read-only observer queries the exact newly
active controller goal's immutable GetResult response. Missing or ambiguous
goal identity fails closed. The existing 2.5.9 handle's cached PREEMPTED status
is insufficient: its late cancel acknowledgement can overwrite natural success.
TEM's cancel acknowledgement alone is never stop evidence. Natural success remains success. Shutdown rejects new
work, requests stop and joins workers; no detached threads are used.

Humble's ExecuteTrajectory goal has no controller-name field. The backend uses
existing active controller handles covering the requested joints, rejects
ambiguous ownership, and passes those same controllers to TEM. No follower
command ownership changes. An independent read-only controller status/result
audit in the commissioning client requires the exact newly active controller
goal to be executing at cancellation and terminate CANCELED.

## Scoped build and load

Use the existing frozen workspace only:

```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/local_setup.bash
cd ~/workcell_ws/src/easy_manipulation_deployment
scripts/build_commissioning_capability.sh
ROS_DOMAIN_ID=191 ~/workcell_ws/build/workcell_builder/workcell_execute_action_test
```

`WORKCELL_BUILD_COMMISSIONING_CAPABILITY` defaults OFF. When enabled, CMake
requires MoveIt **2.5.9 EXACT**. The script builds only the capability, its action
test and the existing measurement target, then registers the local plugin via
scoped overlay symlinks. It never installs to `/opt`. It writes a manifest
binding source hashes to the resulting library hash in the existing install
prefix. The client verifies that manifest against the current source, library,
actual `/proc` mapping inode, simulator domain/partition, positive receipt and
sole MoveGroup action-server publisher before motion.

Launch requires `execution_backend:=simulator simulator_commissioning:=true`
and the usual explicit simulator world/output/domain/partition. This selects
`workcell/CommissionExecuteTrajectory` and disables
`move_group/MoveGroupExecuteTrajectoryAction`; exactly one action server remains.
`simulator_commissioning` defaults false. The executor still requires the
explicit existing `--simulator-commission cancel` path and current authoritative
Resolve/Generate binding and full-cycle revalidation. No trajectory is replayed.

## Fresh telemetry

Measurements keep their original acquisition timestamps, simulator receipt,
iteration and simulation time. Publication, callback receive, consume and write
timing are recorded separately. A dedicated acquisition executor and bounded
evidence writer queue prevent action waits and serialization from starving
acquisition. Overflow, a gap, a stale sample or contradictory identity latches
failure; no samples are re-stamped and no thresholds are relaxed. Queue limits
are explicit and overflow is never silently discarded. Stop acceptance still
requires complete consecutive joint measurements below the existing velocity
threshold for at least 300 ms of advancing simulation time, with a fresh endpoint.

The stationary qualification script sends **no motion commands** and exercises
ROS waits, queued evidence writing and the same stop-window implementation:

```bash
python3 scripts/qualify_stationary_telemetry.py --receipt /path/to/new/receipt.json \
  --output /path/to/new/evidence --duration 10
```

Action tests use the production action server with actual ROS action clients
and a controllable backend, isolated from robot controllers. These tests and
stationary telemetry are prerequisites; only a measured moving-cancellation
trial can establish physical cancellation acceptance. Retain failed attempts.

## Qualification boundary, 2026-09-19

An intermediate moving-cancellation trial passed on capability SHA256
`78c1d52610f7349bd7575f20d2ffb450abc77c1d099476b515158afe7143a023`,
after which source review added the immutable controller-result audit described
above.

The **final loaded commissioning capability** was subsequently qualified with
SHA256
`9f750e46a438d4b415afb07d3d3b77ee66f636fd3beedb3ec8b3e5d90d8d0489`.

Measured final-build cancellation evidence:

- the owned `ExecuteTrajectory` goal returned genuine **CANCELED**;
- the owned controller goal independently returned genuine **CANCELED**;
- controller-result callback observation: **3.062 ms**;
- independent GetResult observation: **11.935 ms**;
- measured stop began **411.113 ms** after cancellation;
- **305** consecutive fresh samples over **304 ms** confirmed stationary motion;
- maximum additional joint travel was **0.001385311 rad**;
- baseline ACM was restored, no attachments remained, the collision state was
  valid and all ten parts remained within the existing acceptance tolerances;
- owned processes stopped and the frozen source/binary hashes remained unchanged.

This qualifies the bounded moving-cancellation primitive for the frozen
simulator commissioning build. It does **not** qualify physical grasp retention,
release, transfer, full pick/place, ordinary simulator execution or real
hardware. Those remain separately gated.


## Stage-A completion runner

The remaining simulator gates are orchestrated by
`scripts/run_stage_a1_finish.py`. It does not replay an old trajectory and it
does not reuse a mutated simulator session. Every gate creates a fresh isolated
ROS domain / Ignition partition, captures fresh settled physics observations,
runs Resolve, regenerates the handoff, and revalidates all nine stages before
that gate is allowed to move.

The frozen inputs are pinned in the runner:

- pristine Stage-A world SHA256:
  `39c2aafb62a01af49663f21b734534843d0d4e4e034a164da2eadb03a761f60e`;
- qualified commissioning capability SHA256:
  `9f750e46a438d4b415afb07d3d3b77ee66f636fd3beedb3ec8b3e5d90d8d0489`.

Default workstation paths correspond to the retained September 18 evidence
workspace. A new evidence root is always required:

```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/install/local_setup.bash
cd ~/workcell_ws/src/easy_manipulation_deployment

python3 scripts/run_stage_a1_finish.py \
  --output ~/workcell_ws/stage-a1-finish-$(date +%Y%m%d-%H%M%S) \
  --through full-cycle
```

Gate sequence:

```text
fresh Resolve / nine-stage revalidation
→ approach-only motion telemetry (<250 ms unchanged guard)
→ physical close + >=1 s opposing-contact retention
→ physical lift + release + resettling
→ full physical transfer/place/release/retreat/home
```

The runner stops on the first failed gate, preserves its logs and summaries,
and terminates only the process group it owns. Full-cycle admission additionally
requires the already qualified cancellation summary plus successful telemetry,
retention and contact-release summaries from the immediately preceding fresh
sessions. Real hardware remains locked throughout.
