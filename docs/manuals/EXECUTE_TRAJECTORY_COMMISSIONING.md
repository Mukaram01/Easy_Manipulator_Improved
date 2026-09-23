# Simulator commissioning ExecuteTrajectory capability (MoveIt 2.5.9 / 2.5.10)

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
goal identity fails closed. The reviewed 2.5.9/2.5.10 handle's cached PREEMPTED status
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
accepts only reviewed MoveIt **2.5.9 or 2.5.10**. The script builds only the capability, its action
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



### MoveIt 2.5.10 compatibility review

The second workstation carries `moveit_ros_move_group 2.5.10` rather than 2.5.9.
Before widening the commissioning build gate, the following upstream files were
compared between MoveIt tags **2.5.9** and **2.5.10** and found byte-identical:

- `moveit_ros/move_group/src/default_capabilities/execute_trajectory_action_capability.cpp`;
- `moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp`;
- `moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/action_based_controller_handle.h`;
- `moveit_ros/move_group/include/moveit/move_group/move_group_capability.h`;
- `moveit_ros/move_group/include/moveit/move_group/move_group_context.h`.

Therefore the local correction is explicitly bounded to **2.5.9 and 2.5.10 only**.
Any later MoveIt version remains blocked until the same source/API review is
repeated. The build records the actual installed MoveIt version in
`commission_execute_build.json`, and the live identity gate checks that recorded
version before accepting the loaded binary.

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

The portable Stage-A physics world is tracked at
`scenes/ur5_2f_test/worlds/stage_a0.sdf` and pinned by Git blob identity:
`b687b5bf3c48e4a731d87c86f99e6c3696b21599`.
The runner also records its local SHA256 in the evidence report, but admission
uses the Git blob identity so the check is reproducible across workstations.
It contains a static support plane, a physical destination-bin floor/walls and
ten identical 25 mm dynamic workpieces inside the authored pick zone. Runtime
telemetry and ros2_control plugins are still injected only into the generated
per-session world; the tracked source world contains neither.

For portability across the two development workstations, the runner no longer
trusts a historical machine-specific capability binary. It rebuilds the pinned
reviewed MoveIt 2.5.9/2.5.10 commissioning capability from the current tracked sources, runs its
action tests, then performs a **fresh moving-cancellation qualification on that
exact binary** before telemetry, retention, contact-release or full-cycle
evidence may authorize the next gate. All prerequisite summaries and the final
cycle must carry the same capability SHA256.

The Stage-A task fixture is also prepared from the tracked canonical
`scenes/ur5_2f_test` source for every fresh session. The existing task-intent
migration/validation and real grasp-strategy catalog are reused; no retained
`/home/user` evidence path is required. A new evidence root is always required:

```bash
source /opt/ros/humble/setup.bash
source ~/workcell_ws/moveit_teardown_overlay/install/local_setup.bash
source ~/workcell_ws/install/local_setup.bash
cd ~/workcell_ws/src/easy_manipulation_deployment

python3 scripts/run_stage_a1_finish.py \
  --output ~/workcell_ws/stage-a1-finish-$(date +%Y%m%d-%H%M%S) \
  --through full-cycle
```

Gate sequence:

```text
fresh Resolve / nine-stage revalidation
→ fresh moving-cancellation qualification of current binary
→ approach-only motion telemetry (<250 ms unchanged guard)
→ physical close + >=1 s opposing-contact retention
→ physical lift + release + resettling
→ full physical transfer/place/release/retreat/home
```

The runner stops on the first failed gate, preserves its logs and summaries,
and terminates only the process group it owns. Full-cycle admission requires
the cancellation, telemetry, retention and contact-release summaries produced
by the immediately preceding fresh sessions, all bound to the same current
capability binary and both qualified MoveIt overlay binaries. Build the local
source overlay using [the dependency instructions](MOVEIT_HUMBLE_TEARDOWN_OVERLAY.md)
first. Real hardware remains locked throughout.

Shutdown sends its initial SIGINT only to the owned ROS launch supervisor,
which forwards it to children and collects their exit status. Sending SIGINT
to the entire group first caused duplicate delivery and a race between
`Popen.poll()` and the asyncio child watcher, losing a bridge exit status as
`Unknown child process ... returncode 255`. Group-wide TERM/KILL escalation
remains available for stragglers with the same deadlines; missing or abnormal
exit status still fails the gate.


### Fresh post-close pile admission

After successful physical close, the guard opens a provisional certificate
using fresh measured contacts, then checks every sample through the existing
closure-stop window. This preserves a valid, exact pair when contact telemetry
is intermittent but current measured FCL geometry and neighbor motion remain
within the unchanged 0.1 mm bound. Each newly admitted pair needs actual fresh
physical contact points; stale points or predicted neighbors cannot admit it.

The guard expires cleared pairs irreversibly, then freezes the remaining exact
set before retention or further arm motion. Later new pairs and recontacts
fail closed. The one-second opposing-fingertip retention proof, slip limits,
250 ms freshness limit and bounded lift corridor remain independent and
unchanged. Evidence includes admission samples, continuous check counts,
freeze state, expiry events and the first rejected sample/pair when present.

The escaped stationary evidence is preserved at
`~/workcell_ws/stage-a1-finish-20260923-110012`: a single late sample omitted
`part_00`, despite earlier valid post-close contact and continuously bounded
measured geometry. That run did not prove retention or any later physical gate.

### Simulator closure position reserve

Simulator planning requires both exact allowed fingertip contacts, then checks
one additional existing closure-search step (0.804 / 80 = 0.01005 rad). Every
examined contact must still involve only the target and allowed fingertips;
the final step must retain both contacts and remain within the existing joint
range. If the reserve does not fit or introduces a forbidden contact, the
candidate fails. Fake-hardware closure keeps its existing policy.

This is a bounded position command, not force control or proof of retention.
The commanded and first-opposing-contact positions are recorded in close-stage
metadata. Live opposing contacts, slip, freshness, pile geometry, lift and
release still have to pass unchanged. The motivating run
`~/workcell_ws/stage-a1-finish-20260923-115152` stopped safely at lift when one
near-tangent fingertip contact disappeared; its physical gate remains failed.

### Fortress ros2_control plugin identity

The simulator backend no longer hard-codes a single renamed ros2_control library.
At runtime it inspects the installed ROS prefix and selects one reviewed,
matching contract by real library presence. For Fortress/Humble it prefers the
legacy compatibility pair
`libign_ros2_control-system.so` +
`ign_ros2_control/IgnitionSystem` +
`ign_ros2_control::IgnitionROS2ControlPlugin`; if only the renamed gz library
is present it uses the matching gz hardware/plugin identities. The selected
absolute library path and SHA256 are stored in the simulator spec/receipt and
must be mapped into the owned Fortress process during live identity validation.

Simulator startup exceptions are also written to
`runtime/startup-failure.json`. The Stage-A runner watches that file while
waiting for `receipt.json`, so a failed physics/model spawn is reported
immediately rather than appearing only as controller-manager spawner timeouts.


### Fortress entity-creation service

The portable Stage-A world explicitly loads Fortress Physics, UserCommands and
SceneBroadcaster systems. Dynamic robot insertion uses
`/world/<world>/create`; relying on host-specific default server plugins let
the second workstation start physics without exposing that service. The
simulator backend now verifies the create service is advertised before issuing
the insertion request and fails immediately with a UserCommands diagnostic if
it is missing.


### Fortress create-service discovery readback

A later second-workstation run proved that Fortress itself logged
`Create service on [/world/a0/create]` while a separate `ign service -l`
probe still failed to discover that service. Service-list discovery is therefore
not used as an admission gate.

Robot insertion now calls the create service directly with a bounded maximum of
three attempts. A timeout is retried **only after** the authoritative
`/world/<world>/scene/info` readback proves that `workcell_robot` is absent.
If a create response is lost but the scene readback proves the model exists, the
runner accepts the insertion without issuing a duplicate request. The receipt
records attempt count and whether scene readback recovered a lost response.


### Scene readback synchronization after create

A successful Fortress create call can instantiate the model and initialize
gz_ros2_control before SceneBroadcaster's `scene/info` service reflects the new
entity. The second workstation demonstrated this ordering: the controller
manager initialized successfully, then an immediate scene readback still missed
`workcell_robot`.

The create path now waits up to ten seconds for authoritative scene readback
after a positive `data: true` response. A positive create is never retried.
Lost/timeout responses get a shorter bounded readback window before a retry is
permitted. This removes the race without weakening identity: the receipt is
still written only after SceneBroadcaster proves the robot model exists.
