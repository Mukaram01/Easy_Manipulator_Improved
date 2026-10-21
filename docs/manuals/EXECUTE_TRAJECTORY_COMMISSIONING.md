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
accepts only reviewed MoveIt **2.5.9 or 2.5.10**. The script builds the capability,
support adapter, their tests and the existing measurement target with `-O2 -g`
and assertions enabled. The support targets retain directed rounding and disabled
floating-point contraction. It then registers the local plugin via
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

Interrupted simulator PLAN_ONLY goals get a five-second cancellation response
wait and up to 120 seconds to return a terminal result, matching their result
watchdog allowance. The native adapter chain shares one 120-second steady-clock
deadline starting before planning; certification and controller sampling check
that deadline and the owning ROS context. Interruption clears the trajectory
and returns failure. MoveGroup's synchronous planning callback can delay handling
cancellation until certification finishes. Search cannot continue without the
owned goal's terminal result; elapsed time or cancellation rejection is not proof
of termination. Motion action deadlines are unchanged.

## Fresh telemetry

Measurements keep their original acquisition timestamps, simulator receipt,
iteration and simulation time. Publication, callback receive, consume and write
timing are recorded separately. A dedicated acquisition executor and bounded
evidence writer queue prevent action waits and serialization from starving
acquisition. Once armed, overflow, a gap, a stale sample or contradictory identity
latches failure permanently. Before the first arm, an otherwise valid sample
older than 250 ms is rejected and recorded; a later fresh sample may restore
passive acquisition. Rejected samples never become the current measurement or
enter the motion queue. Identity, structure and time-order errors still latch
before arm, including errors carried by an aged sample. `fresh()` and `arm()`
retain the 250 ms limit, and arm still checks the receipt and publisher identity.
No samples are re-stamped. Queue limits are explicit and overflow is never
silently discarded. Stop acceptance still
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
`b533d6de2b454939f0ebedbcfb3aed5ccb16051b`.
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
source ~/workcell_ws/ros_gz_bridge_shutdown_overlay/setup.bash
source ~/workcell_ws/install/local_setup.bash
cd ~/workcell_ws/src/easy_manipulation_deployment

python3 scripts/run_stage_a1_finish.py \
  --output ~/workcell_ws/stage-a1-finish-$(date +%Y%m%d-%H%M%S) \
  --through full-cycle
```

`--search-timeout` sets the finite wall-clock candidate-search budget, including
continuous collision certification (default 300 seconds). Slow workstations can
use a larger explicit bound, for example `--search-timeout 900`. The report records
that budget; candidate ordering, per-segment planning limits, motion deadlines
and physical acceptance guards are unchanged. Exhausting the budget still blocks
acceptance and is reported separately from a proved infeasible grasp.
An in-flight simulator PLAN_ONLY request has a 120-second result watchdog because
continuous certification follows the planner's own time limit. A measured
revalidation transfer certified successfully in 76.4 seconds under simulator load.
A timed-out owned planning goal must reach a terminal action state before another candidate can
start. If it does not, the session fails and shuts down. Physical execution keeps
its existing action deadlines.
Simulator discovery has a 60-second candidate slice within the global search
budget, checked between calls; an in-flight request retains the action watchdog
and settlement allowance above. It still uses one planner attempt with a 0.75-second planner
limit; the larger wall allowance includes mandatory continuous certification.
Observed successful approaches took 3.7–44.3 seconds, so the old 0.75-second
whole-candidate allowance always stopped those candidates before grasp checks.
Simulator retries use a 180-second candidate slice, checked between calls within
the global search budget.
Observed retries needed 120.9 seconds through retreat and 124.6–131.5 seconds
through transfer, exhausting the previous 120-second allowance before the
remaining stages could start. Once discovery finds a retryable candidate, it
reserves the three-candidate retry beam's allowance before starting another
discovery slice. Candidate order and the ranked retry beam are unchanged.
Fake-hardware discovery keeps its 0.75-second wall
bound and retries keep their 20-second bound.

The qualified bridge executable is built and proven using
[the bridge shutdown overlay instructions](ROS_GZ_BRIDGE_SHUTDOWN_OVERLAY.md).
The runner checks its source, patch, proof and binary hashes before launch and
its actual executable/library mappings before Resolve in every session.
A missing or changed bridge overlay blocks commissioning.

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

### Execution modes: full, targeted and fast

The same runner has three modes. Only `--mode full` (the default) through
`full-cycle` can report Stage-A acceptance; its report is
`stage-a1-final-report.json` and carries `"stage_a_acceptance": true` only when
all six gates pass. A passing `--through` prefix records `false`.

```bash
# FAST: no ROS launch. Commissioning build + its C++ unit tests + the 22 focused
# Stage-A Python suites (runtime-only selections such as
# WORKCELL_SIMULATOR_PHYSICS_ROOT are removed from the unit-test environment).
python3 scripts/run_stage_a1_finish.py --output <new dir> --mode fast

# TARGETED: one gate in its own fresh owned session.
python3 scripts/run_stage_a1_finish.py --output <new dir> --world-profile sparse \
  --search-timeout 900 --mode targeted --gate contact-release
```

A targeted gate runs the same preflight (world and bridge identity, commissioning
build and C++ tests), keeps the session index, ROS domain and partition it has in
the full sequence, and inside that session performs the same fresh Resolve,
observation refresh, nine-stage revalidation, gate execution, gate assertions and
owned shutdown checks as a full run. Nothing is skipped inside a gate; only the
*earlier gates* are not run. `resolve`, `cancel`, `telemetry`, `stationary` and
`contact-release` can be targeted. `full-cycle` cannot: the executor admits it
only with passing cancellation, telemetry, retention and contact-release evidence
bound to the same binaries, and the runner builds that evidence only from the
sessions of the same full run, so it is available only in full mode. Targeted
session reports carry `mode` and `scope` too; never hand-assemble targeted
summaries into `--commission-evidence`. Targeted and fast reports
are written as `stage-a1-targeted-report.json` / `stage-a1-fast-report.json`
with `"scope": "DIAGNOSTIC_NOT_STAGE_A_ACCEPTANCE"` and
`"stage_a_acceptance": false`; they never replace the full acceptance sequence,
and a targeted motion gate does not re-qualify cancellation on the current
binary (only the cancel gate does; the report records
`"cancellation_qualified_this_run"`).

`--gil-watchdog-ms N` (targeted `resolve` only, 50–1000 ms, off by default) arms an
executor diagnostic: when its heartbeat thread cannot run for about N − 25 ms
(a GIL-holding call, a garbage collection, or CPU starvation), CPython's
`faulthandler` watchdog writes every thread's stack to
`<summary>.measurements.gil-holds.txt` and the overrun is recorded in the timing
evidence (`gil_hold`, `gil_watch_summary`). The dump does not mark which thread
held the lock; read it together with the `gc` and `receive` events of the same
timing file. It changes no measurement, guard or threshold. Its own re-arm held
the GIL for 400 ms in Trial 26 and failed a lift, so the runner refuses it for
every gate that moves the robot and for full mode.

`--gil-probe-ms N` (targeted/full, 20–1000 ms, off by default) builds
`scripts/gil_probe.c` into the evidence directory (source and library hashes,
compiler and Python version are recorded) and attributes every GIL hold of at
least N ms in each executor session:

- its native thread reads only static `_PyRuntime` fields (GIL locked flag,
  switch number, holder thread-state pointer) and never dereferences an
  interpreter object, thread state or frame;
- Python threads register themselves under the GIL (thread-state pointer, CPython's
  never-reused thread-state id, kernel tid, GIL switch number); a sampler thread asks
  for the GIL every 20 ms, and after a wait of at least N/4 ms takes one native
  snapshot (GC disabled, GIL kept, switch number read at both ends) of every thread's
  Python frames and of the live thread-state -> (ident, id) map under CPython's head
  lock. Each hold gets the holder's identity — `verified` only when the live id at
  its address is the id of the registration in force at the hold's switch;
  otherwise `snapshot only`, `registered, unconfirmed`, `address reused, ambiguous`,
  `ambiguous (thread starting)` or `unattributed` — and its frames at the release
  (`exact` only when no thread ran during the snapshot and at most one between the
  release and it);
- during a hold it reads the holder's `/proc/self/task/<tid>` CPU, run state and
  syscall (`running` or the blocking call and its PC);
- `--gil-probe-native-pc` also samples the holder's native PC during long holds
  only: one signal at a time, each sent right after the holder's syscall file read
  `running`, and none for the rest of a hold once it is seen inside a syscall
  (`running` is the kernel's TASK_RUNNING, which includes the not-yet-sleeping part of
  a syscall, so the first signals of a hold can still meet one); the handler copies
  registers and 4 KiB of its stack (no unwinding); stack-scan candidates cover only
  the libraries mapped when the probe started; `scripts/gil_probe_report.py <session>`
  symbolizes them from the session's maps snapshot. Libraries without symbols
  (the stripped `python3.10`, `_json`) are named by library, and their
  "nearest exported symbol" is marked unreliable; libc resolves through
  `libc6-dbg`. Stack-scan callers are heuristic;
- the executor and acquisition loops record calls that took ≥ 20 ms
  (`loop_slow`: `spin_once`, `execution_monitor`, `wait_set_*`, `take_message`).

Holds of at least N/2 ms are also recorded (`gil_probe_brief_hold`: identity and
frames only), and a hold still in progress when the probe stops is recorded as
`gil_probe_open_hold`, so back-to-back shorter tenures are visible too.

Limits: a hold that begins before the sampler's first loop (probe start) gets
no snapshot; about 2 % of offline release snapshots are `holder may have run
again`; the native thread costs about 1.5 % of one core and the sampler about
0.5 %, with no measured change to callback latency or throughput under the
release-phase replay load (one of 15 replays with native PC had a single 103 ms
sample age with no hold recorded; not reproduced, cause unknown). A hold is
evidence of *where* the GIL was kept, not proof of why.

Measured on the sparse world (2026-10-21, one workstation; full-mode figures
from Trials 20–22):

| Mode | Measured wall time | Contents |
|---|---|---|
| fast | 145 s | preflight 96 s (build check, C++ action + support/Cartesian tests), 974 Python tests in 22 suites 49 s |
| targeted `resolve` | 695 s | preflight 97 s + one plan-only session 598 s |
| targeted `telemetry` | 776 s | preflight 97 s + one session 681 s (approach motion) |
| full through `full-cycle` | 2885–4240 s | preflight + six sessions of 391–1367 s each |

A session's time is dominated by its own fresh Resolve candidate search
(89–936 s observed) and revalidation (56–372 s); the gate itself took
65–263 s. Targeted `stationary` and `contact-release` were not measured.

On a measured collision rejection, the executor preserves the first rejected
measurement, exact queried robot state and full MoveIt validity response before
cancellation or reconciliation. The owned goal record also retains its exact
commanded trajectory, serialized before submission, so planned and measured
motion can be compared without reconstructing a stochastic plan. These records
do not change collision acceptance or owned cancellation.

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

### Transfer IK seed continuity

Resolve retains the arm IK seed from a successfully planned transfer in the
existing hashed task handoff. Revalidation seeds only the transfer IK query
from that configuration and rejects missing, nonfinite or changed arm solutions
outside the existing approach tolerance of 0.0001 rad. Missing saved transfer
bindings block revalidation. The current Cartesian goal, measured planning start,
gripper state and private collision scene are still used. The seed is bound
to model, planning group, tool, frame and stage. It is neither a trajectory
nor collision authority, and it does not change planning time or tolerances.

This addresses the failure preserved at
`~/workcell_ws/stage-a1-finish-20260923-121649`: Resolve proved the complete
cycle, but revalidation selected a different transfer IK branch intersecting
the bin. The strict existing approach-branch binding remains unchanged.

### Simulator closure position reserve

Simulator planning requires both exact allowed fingertip contacts. It refines
the first observed contact bracket with at most ten bisection queries, then
checks one full closure-search step (0.804 / 80 = 0.01005 rad) beyond the
verified contact endpoint. This removes coarse search overshoot while retaining
the entire position reserve. Every
examined contact must still involve only the target and allowed fingertips;
the final step must retain both contacts and remain within the existing joint
range. If the reserve does not fit or introduces a forbidden contact, the
candidate fails. Fake-hardware closure keeps its existing policy.

This is a bounded position command, not force control or proof of retention.
The commanded position, observed contact bracket and refinement count are
recorded in close-stage metadata. Live opposing contacts, slip, freshness, pile geometry, lift and
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

### Simulator lift dynamics

The existing Cartesian lift uses velocity and acceleration scaling 0.02 in the
verified simulator path; other stages and fake-hardware planning retain 0.2.
The physical trial at `~/workcell_ws/stage-a1-finish-20260923-125904` showed that
tracking error and 7.27 micrometres of held-object slip closed an uncertified
neighbor gap of 6.82 micrometres near peak initial acceleration. The measured
certificate correctly rejected the new contact. Dense replay of the commanded
trajectory with the measured attachment remained collision-free.

This conservative lift profile reduces commanded dynamics; it does not change
collision/contact/slip/freshness bounds, the frozen certificate, planning budget
or execution deadline. The subsequent `20260923-131514` trial still failed at
10 mm rise because a same-height side neighbor had not cleared 0.1 mm. Speed
reduction therefore does not establish extraction feasibility. The
planning metadata records the applied scaling, and the owned goal records the
resulting positions, velocities, accelerations and times.


### Generic single-cycle extraction feasibility

The existing resolver considers all fresh targets allowed by TaskIntent in its
confidence/height order, with ID only breaking ties. EXACT still evaluates its
one object/grasp; AUTO and PREFERRED retain their permitted fallback semantics.
A grasp is selectable only after the entire cycle has passed.

At the private post-close attachment checkpoint, the existing preplanner tries
at most four extraction intents: the authored vertical lift, then an aggregate
and up to two individual away directions derived from obstructing nearby BOX
geometry. Each has the same authored rise and at most 2 mm total lateral offset,
inside the unchanged 2.5 mm initial-separation corridor. Downward support normals
that already clear vertically do not distort the away direction. No object ID,
world axis or captured pile coordinate determines a direction.

The existing FCL geometry interface rejects a predicted extraction unless exact
initial contacts and initially nearby pairs clear 0.1 mm before 10 mm rise.
Positive-gap pairs never gain contact permission. Initial penetration remains
bounded by 0.1 mm; new contacts and regained expired allowances fail. Geometry
is checked before planning and along densified object poses from the actual
planned Cartesian trajectory. Full-arm MoveIt/private-scene checks remain
required, including non-BOX obstacles. Geometry prediction never certifies a
physical grasp or changes the live ACM/contact policy.

Every variant must also pass transfer, placement, containment, release,
retreat and home before selection. Failed variants restore the same private
checkpoint, retain structured rejection evidence and share the existing
candidate/global deadlines. Exhausted extraction variants yield
`NO_VALID_EXTRACTION`; later-stage and budget failures remain distinct.
The resolution hash binds the successful relative extraction intent and proven
IK solutions. Consumption checks only that bound candidate and plans fresh
trajectories against the current scene. Failed variants cannot export transfer
seeds to another variant.

Evidence records fresh targets, evaluated objects/grasps/variants, rejection
stages and selected intent. During execution all existing fresh physical
measurements, opposing fingertips, retention, slip, exact pile certification,
irreversible expiry and cancellation checks remain authoritative. This is one
pick/place cycle; reobserve/repeat orchestration and hardware access remain out
of scope. Use the same six-gate runner above with a fresh output directory.

### Controller interpolation feasibility

The `20260923-134919` contact-release trial stopped during approach, before
physical close or extraction. Its 129 MoveIt waypoints were collision-free,
but replay through the installed joint trajectory controller found eight
self-colliding samples between waypoints 49 and 50 (4.954–4.961 s). The measured
forearm/wrist contact reproduced exactly in the unchanged private scene.
Cancellation, reconciliation and shutdown were clean; the first four gates
passed and the full-cycle gate did not run.

The existing Cartesian adapter now audits ordinary planner responses after time
parameterization, using the installed controller's interpolation of emitted
positions, velocities and accelerations. The same audit supplements its
Cartesian and initial-support paths. Sampling is chronological, at most 1 ms
apart and additionally bounded by polynomial joint travel; malformed data,
invalid states or an exhausted sample bound reject the trajectory before it
can make a candidate READY. The maximum duration is the existing 120 s
execution deadline, with at most 120001 checked states. No timing, waypoint,
collision allowance or trajectory is repaired by this audit.

Every sample retains the private scene, carried geometry and other joint states;
leader updates propagate mimics. Full-robot collision/path checks and existing
initial-contact predicates remain in force. Contact expiry never resets between
spline segments. Rejections return `INVALID_MOTION_PLAN` to the existing bounded
retry policy. The adapter returns failure locally because throwing out of an
adapter can cause MoveIt to skip it. The build now explicitly depends on
`joint_trajectory_controller`, whose actual interpolation implementation is used.

### Simulator support geometry parity

The `20260923-141151` trial passed the first four gates, then stopped during
approach on a fingertip/support contact. Its 122 waypoints and 12020 controller
samples are clear against the frozen private scene. The simulator, however,
had an additional plane at z=0 while MoveIt had the canonical finite table box.
The first preserved contacting state lies more than 557 mm outside that box.
Replacing only the diagnostic planning support with the physical plane makes
2477 commanded samples collide; tracking error is not required to explain it.
The exact first rejected sample was not preserved; the cancellation sample at
iteration 101401 is the first preserved contacting state.

Simulator preparation must consume the same canonical collision manifest as
MoveIt for its support fixture, including dimensions and world pose. A plane
placeholder is not an alternate source of physical geometry. Missing,
ambiguous or unsupported support geometry must fail before simulation starts.
Keep fixture identity and friction, bind the manifest and generated geometry
in the runtime receipt, and retain every existing physical contact guard.
This corrects the simulator environment to the authored table; it does not
qualify the earlier failed run or establish release/full-cycle acceptance.

The preserved run is `/home/ubuntu/workcell_ws/stage-a1-finish-20260923-141151`.
Independent geometry and exact controller replay evidence are in
`/home/ubuntu/workcell_ws/support-parity-analysis-20260923-141151` and
`/home/ubuntu/workcell_ws/approach-support-analysis-20260923-141151`.

### Direct physics poses and current-grasp retention

The `20260923-143517` trial passed Resolve, cancellation, telemetry and stationary
retention, then stopped safely during lift on a 617.184 nm measured downward
step. Exact replay of its 42 waypoints and 4533 controller samples is monotonically
upward, with matching initial arm joints. The historical stream cannot recover
poses that it never recorded, so it does not establish whether that entire
reversal was physical.

Installed Fortress 6.18.0 / DART physics 5.4.0 suppresses `ChangedWorldPoses`
updates below 1 micrometre. Both composed ECS poses and link `WorldPose` follow
that cache. The existing telemetry plugin now requests world poses on identity
observation children of the original links. Fortress resolves these read-only
requests directly from physics each tick. They add no bodies, collisions,
constraints, forces or commands. Model poses use the direct canonical-link pose
and its validated fixed local transform; unsupported frame relationships fail.

Each tick invalidates old query data before physics updates it. Missing entities,
changed bindings, unavailable/nonfinite results or stale ticks produce an explicit
error with no fallback poses. The existing acquisition error latch blocks motion.
The receipt declares the required source, and every sample records method,
world frame, simulator version, query iteration/time and original model/link/query
entity identities. Live acquisition rejects legacy receipts and mismatched source
metadata; ordinary fake-hardware behavior is unchanged.

An independent gravity-only probe observes 100 distinct monotonic poses spanning
504.9 nm while both cached link channels remain unchanged. A paired control
without queries for its first 99 ticks has the same cached trajectory and exactly
the same final direct physics state. Production telemetry emits 1000 valid
source-bound samples; disabling physics produces 1000 explicit errors and no
poses. Six focused C++ regressions cover frame identity, nonzero canonical offsets,
missing entities, stale/unavailable writes and observation-only components.
The sources, commands, actual loaded-library identities and trace hashes are in
`/home/ubuntu/workcell_ws/simulator-pose-precision-20260923/provenance.json`.

Every current grasp now runs the existing 1.1 s monitored hold and requires
at least 1.0 simulated second of retention before attachment or lift. Stationary,
contact-release and full-cycle share that sequence. The held reference and frozen
pile certificate remain unchanged throughout; every queued measurement is checked.
Retention evidence binds the current run, target, close goal and resolution to
its measured start/end ticks. Another session's stationary result cannot authorize
this grasp. The runner requires this binding for all three physical grasp gates.
Collision, penetration, slip, freshness, extraction, rotation and duration limits
remain unchanged. Retention qualification and pose precision are independent
requirements; neither substitutes for the other.

### Pre-attachment geometric qualification (2026-09-27)

The simulator's authoritative full-cycle preplanner now calls `grasp_capture`
**before** creating its private rigid attachment, both during Resolve and
resolved-candidate revalidation. The closure reserve remains a position command;
its contact-link membership is no longer sufficient for geometric eligibility.
Ordinary fake-hardware planning keeps its existing behavior. AUTO, PREFERRED
and EXACT continue through the existing resolver, including EXACT no-fallback.

Inputs are the live world-rooted URDF, SRDF identity, actual tool collision
BOXes/binary-STL triangles with scales/origins/hashes, the target BOX in world
metres, fresh independently measured joint/link poses, and separately labelled
PLANNED open/closed states. Raw measured mimic discrepancies are retained and
propagated through mimic dependencies into predicted geometry. Contradictory
measured link poses, unsupported geometry/frames, changed mesh contents or
missing evidence reject the candidate. Live model parameters and a fresh passive
measurement are checked again when calculation finishes. Measured joints and tool
frames, which supply FK and mimic evidence, must remain numerically equal within
1e-9 SI units. Original acquisition timestamps are retained; the 250 ms freshness
limit applies to each measurement when checked.

Target facets are evaluated at the immutable bound observation pose. Both the
initial and completion target measurements must agree with that same pose within
the existing 0.25 mm translation and 0.00025 sign-invariant quaternion-distance
limits. The reference never moves with successive measurements. This replaces
the earlier 1e-9 equality requirement between passive target measurements: two
live candidates passed all 21 geometry samples but failed that completion check,
even though the passive target pose is not an input to the facet calculation.
Both measurements and completed observation-agreement checks are retained, including
rejections. This admission rule does not certify exact latest-target geometry or
continuous robustness within the admitted pose range; physical retention still
requires the later contact gates.

Two distinct fingertip regions must face opposite faces of the same target.
The closing direction comes from the URDF's relative fingertip motion between
the planned open and closed states, recomputed for each joint perturbation.
It does not assume that a mounted tool closes along grasp-frame Y. Each sample
records the physical direction and its angle to the selected target face.
Actual facets are clipped to the allowed contact-depth slab. A disk must fit
inside both facet footprints and the target face interior. Whole-mesh projection
overlap cannot certify capture. A separate conservative half-space check rejects
any deeper fingertip protrusion hidden by a qualifying facet. Open aperture,
opposition, depth, patch radius and edge margins are recorded. This single-facet
rule can reject physically useful curved or finely tessellated contact regions;
unsupported configurations fail closed.

Policy defaults are configurable through the grasp contract's
`geometric_qualification` mapping, loaded from
`cell_definition.yaml:end_effector.geometric_qualification`. Values and units
are declared in `grasp_capture.DEFAULTS` and written into each fixture. The
1 mm interior radius is four times the 0.25 mm translation sample; the 1 mm
penetration cap is a geometric limit, not a compliance model. Target rotation
samples are ±0.5 degrees, face opposition is limited to 15 degrees, and open
clearance is 1 mm. Tool-joint samples use 0.001 rad or 0.05 mm; measured mimic
limits use 0.02 rad or 0.1 mm. These are commissioning policy values, not
holding-force guarantees or values fitted to a particular object.

Robustness is **SAMPLED_NOT_CONTINUOUS_PROOF**: nominal plus independent positive
and negative translations, rotations and moving tool-joint perturbations.
Passing those samples does not prove every combination of perturbations.
The claim remains **GEOMETRIC_ONLY**. Live opposing contacts, current-grasp
retention, collision, slip, release, cancellation and cleanup gates all remain
mandatory. Qualification fixtures preserve candidate/scene/model identities,
raw physical samples, planned states, mesh hashes, per-sample margins and
numeric rejection geometry under each executor's `grasp-qualification/`.

The 2026-09-27 offline regressions use explicitly synthetic fixtures. The
unavailable historical pre-lift state is still **INSUFFICIENT EVIDENCE**; the
preserved iteration-104485 post-failure fixture is not a substitute. Fresh
plan-only and six-gate commissioning remain blocked until bridge admission.
