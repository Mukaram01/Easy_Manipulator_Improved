# Humble MoveIt TEM teardown overlay

The Stage-A commissioning path requires a locally built, verified dependency
overlay. It grants no real-hardware authorization. The ordinary fake-hardware
workflow does not acquire a new MoveIt version or source-build requirement.

## Confirmed defect

The installed `ros-humble-moveit-ros-planning` package
`2.5.10-1jammy.20260908.081349` destroys its controller-manager plugin loader
before its retained controller node. That node's callback group still holds
expired weak client references whose control-block virtual functions are in
the unloaded `libmoveit_simple_controller_manager.so.2.5.10`.

The actual TEM reproducer in `scripts/moveit_teardown_repro` creates that
controller plugin and private executor, waits for readiness, receives an owned
SIGINT, and destroys TEM. It requires no robot model, controller server,
Fortress, task resolver, trajectory or hardware command. Baseline exit: -11.
GDB locates the fault at `CallbackGroup::~CallbackGroup`, `call *0x18(%rax)`;
`rax=0x7ffff05c8780` lies in the controller plugin's former read-only mapping
`0x7ffff05c8000–0x7ffff05ca000`, already unloaded at the fault. The stack passes
through Node and TEM destructors, matching the latest Stage-A MoveGroup crash.
The available apport MoveGroup core predates that run (September 21); it is not
represented as PID 22814's core. The new controlled baseline has its own full
GDB thread stacks, library mapping and instruction evidence.

The subsequent Stage-A run at `f275a844` passed Resolve/revalidation but
exposed the analogous second lifetime defect in MoveGroup's main node. Its
fresh PID 30576 core shows the default-capabilities library unloaded before
the node's callback groups, while the local commissioning library remained
mapped. An upstream-only one-joint MoveGroup launch, with execution disabled
and no Workcell plugins, reproduced this with the already patched TEM.

`patches/moveit_humble_capability_teardown.patch` retains the capability loader
in main scope, declared before the node. Ownership is retained immediately
when the loader is created, before capability initialization, covering both
normal teardown and exception unwinding. Capabilities still release their
objects normally; the loader unloads after main-node callback groups finish.
No library is intentionally leaked or pinned beyond its required lifetime.

The additional source package is `moveit_ros_move_group`, release commit
`66d37b40594e2b0ce8e8bd407122d20791d8c3b5`, with Debian packaging commit
`b3c911be97bd87141eaf6481497f4184cc202dd8`. Its source also matches upstream
2.5.10, and the Debian delta contains only packaging files.

## Narrow correction and source identity

`patches/moveit_humble_tem_teardown.patch` changes only TEM's destructor in
`moveit_ros_planning`. Its existing stop operation joins the execution worker;
the patch additionally joins any continuous worker while controller callbacks
can still run. It then cancels/joins the private executor, releases queued
contexts, handles, parameter callback, controller manager and controller node,
and finally releases the plugin loader. It does not change class layout,
public headers, SONAME, motion behavior, or shutdown timeouts.

The source is the ROS release package, not a current development branch:

- Release tag: `release/humble/moveit_ros_planning/2.5.10-1`.
- Release commit: `c62753946ae3629a8cb745767844f7e69ca51489` in
  [ros2-gbp/moveit2-release](https://github.com/ros2-gbp/moveit2-release/tree/c62753946ae3629a8cb745767844f7e69ca51489).
- Debian Jammy release: `4e3d4445099fccf3d3732e893bf461e71e86522d`; its changes
  consist only of added `debian/` packaging files, with no source patch.
- Upstream 2.5.10: `c283a36186a6f7a5985360e6674bf8fd0790e485`; the unpatched TEM
  implementation matches the release source byte for byte.
- The installed TEM header also matches the release header byte for byte.

## Build and proof

From this repository on the reviewed Ubuntu/Humble workstation:

```bash
scripts/build_moveit_teardown_overlay.sh ~/workcell_ws/moveit_teardown_overlay_new \
  /path/to/real.request.cdr /path/to/real.scene.cdr \
  /path/to/qualification.urdf /path/to/qualification.srdf
source /opt/ros/humble/setup.bash
source ~/workcell_ws/moveit_teardown_overlay_new/install/local_setup.bash
source ~/workcell_ws/install/local_setup.bash
```

The arguments select a new external overlay directory and the captured real UR5
request, scene, URDF and SRDF for active planning shutdown qualification.
The script pins/checks source, retains baseline and build evidence, applies
only the tracked patch, and builds only `moveit_ros_planning` and `moveit_ros_move_group` against installed
Humble with two build jobs. It never changes `/opt` or vendors MoveIt here.
It requires a baseline SIGSEGV followed by 20 consecutive patched controlled
shutdowns. Each patched cycle must return 0, print `TEM_DESTROYED`, load the
exact expected TEM library in `/proc/PID/maps`, leave no live process in its
owned group, and create no new/changed probe core file. This regression tests
the actual TEM binary, not a manual standalone plugin destruction sequence.

A second minimal regression starts the actual MoveGroup executable with a
one-joint URDF/SRDF and OMPL, waits for readiness, then sends owned SIGINT.
The installed executable with patched TEM must reproduce -11; the patched
executable must pass 20 consecutive cycles with no new core, nonzero exit,
or owned process remaining. `/proc/PID/exe` and mappings identify the binaries.

A third regression replays the real UR5 request with production planning
adapters, verifies the goal is accepted and naturally pending, and sends owned
SIGINT. Five cycles must complete the production certificate and plan after the
signal, return 0, and leave no new core or owned survivor. The adapter and probe
are archived before runtime; mappings bind the actual loaded artifact. No
planning delay or test barrier is introduced.

Only after all three proofs does the script publish `provenance.json` and the
package-share symlink `workcell_teardown_overlay.json`. They bind release and
upstream identities, archive hash, tracked patch hash, installed baseline,
patched TEM, executable and default-capability library paths/hashes, build logs
and the idle and active repetition results.
Runtime checks independently confirm actual MoveGroup executable, TEM and
default-capability library mappings/inodes before every gate; full-cycle
prerequisites require all three identities to match the current process and commissioning build.

Local independent proof for each defect: baseline -11, patched **20/20 exit 0**,
no timeout, changed core inventory or live owned processes. The patched GDB run also exits
normally. Evidence:
`~/workcell_ws/moveit_teardown_overlay/evidence/run-20260923-105813`;
expanded unload/mapping proof: `evidence/baseline-lifecycle-gdb.log`.
The second full Stage-A attempt is preserved at `~/workcell_ws/stage-a1-finish-20260923-104525`.
These independent regressions qualify dependency cleanup; Stage-A physical
gates still require their own live acceptance evidence.

### Restoration on 2026-09-27

The old MoveGroup installation resolved into its build tree. Its retained CMake
cache and build command showed symlink installation enabled; no current global
colcon defaults file was found. The builder now isolates colcon defaults and
explicitly sets `AMENT_CMAKE_SYMLINK_INSTALL=OFF`. TEM-only provenance stays in
the evidence directory. Final provenance and its discovery link are published
atomically only after both proofs; resolved binaries must be inside installation.

The clean overlay at
`/home/user/workcell_ws/moveit_teardown_overlay_stage_a1_20260927` passed both
baseline SIGSEGV proofs and both 20-cycle patched sequences. Its existing
runtime admission check accepts TEM SHA256
`39807461959d00318d8a4e00db99009b7f269ccbacba620d10b779448f1ff6fc` and MoveGroup
SHA256 `e79a53216e74c270702f12f881226bf07ad879b5fea723afd91e0c57e0fe1fe1`.
`evidence/run-20260927/artifact-mapping-audit.json` verifies all 42 process
executable/library mappings, devices/inodes, owned cleanup, pinned patches and
preservation of the old overlay. This completes MoveIt dependency qualification,
not the Stage-A1 simulator milestone.

### Additional FastDDS checks on 2026-09-29

The same qualified installation passed **20/20 TEM and 20/20 MoveGroup**
lifecycle checks with `rmw_fastrtps_cpp` after sourcing the qualified bridge
overlay. Every cycle reached readiness and exited 0 after owned SIGINT, with no
timeout, changed core inventory or owned process survivor. The original
September 27 baseline proofs, patched proofs and published provenance remain
byte-for-byte unchanged; these checks required no rebuild or republication.

Both qualified MoveIt artifacts are ordinary installed files. All 40 new
process maps match their executable/library paths, devices and inodes. FastDDS
was mapped in every cycle, the old MoveIt overlay was absent, and the audit
checked the mapped identities of 180 distinct shared libraries. The existing
MoveIt and bridge runtime admission checks both passed before the cycles.

The verified source order, starting in a clean shell, is:

```bash
source /opt/ros/humble/setup.bash
source /home/user/workcell_ws/moveit_teardown_overlay_stage_a1_20260927/install/local_setup.bash
source /home/user/workcell_ws/install/local_setup.bash
source /home/user/workcell_ws/ros_gz_bridge_shutdown_overlay_stage_a1_20260927/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
```

Use the workspace's `local_setup.bash`: its current `setup.bash` chains the old
`moveit_teardown_overlay` installation, whose qualification receipt is absent.
The additional tests used isolated domain 197 and the existing minimal TEM and
one-joint MoveGroup configurations. They do not establish Stage-A1 simulator
gate completion or authorize hardware motion.

The bounded [evidence bundle](../../evidence/stage_a1/moveit_closeout_20260929/README.md)
contains both cycle results and hashes of the complete workstation audit at
`/home/user/workcell_ws/diagnostics/moveit-closeout-20260929`.

### Active planning shutdown correction on 2026-09-30

A real Stage-A Resolve attempt exposed a separate failure before destructor
cleanup. Its PLAN_ONLY client stopped while the normal production certificate
was running. After owned SIGINT invalidated the ROS context, planning completed
successfully and `goal->succeed()` threw `Asked to publish result for goal that
does not exist`. The uncaught exception aborted MoveGroup. The preserved launch
log is `~/workcell_ws/stage-a1-closeout-20260930-hybrid-lift/01-resolve/launch.log`.
The earlier idle qualification remains evidence for its original scope.

The existing capability patch now makes the goal terminal locally and catches
terminal publication failure only when the ROS context is invalid and the goal
is already inactive. Valid-context exceptions and failed local transitions
remain visible. Feedback is skipped after context shutdown. Returning before
the terminal transition would be unsafe: the goal-handle destructor attempts a
terminal publication for an active goal. The callback finishes with local IDLE
state and releases its preemption flag.

The portable regression compiles the callback tail extracted from the tracked
patch and tests six context/terminal/feedback paths. Those are unit tests. The
runtime qualification uses the captured real request and production certificate
through `scripts/moveit_teardown_repro/verify_active_plan.py`.

The qualified installation is
`~/workcell_ws/moveit_teardown_overlay_active_plan_20260930`. Its TEM package is
an ordinary copy of the previously qualified installation with identical TEM
SHA256 and preserved original build proof. Only MoveGroup is rebuilt from the
same pinned release plus the extended capability patch. Publication requires
fresh installed-artifact idle cycles and five real active-plan shutdown cycles.
The September 27 overlay and original provenance are preserved.

The final host-native qualification passed **TEM20 + idle MoveGroup20 + real
active-plan5**. Every active cycle completed the same 1,647-interval production
certificate after the actual rclcpp signal-handler record and exited 0. All
45 cycles had no timeout, forced cleanup, changed core inventory or owned
survivor. The artifact audit checked 195 mapped shared libraries. The final
three-artifact MoveIt admission and refreshed bridge admission both pass.

Current source order:

```bash
source /opt/ros/humble/setup.bash
source /home/user/workcell_ws/moveit_teardown_overlay_active_plan_20260930/install/local_setup.bash
source /home/user/workcell_ws/install/local_setup.bash
source /home/user/workcell_ws/ros_gz_bridge_shutdown_overlay_stage_a1_20260927/setup.bash
```

The new overlay has standard generated colcon hooks and ordinary installed
binaries. The checked environment contains neither older MoveIt overlay.
See the [bounded active-shutdown evidence](../../evidence/stage_a1/moveit_active_plan_shutdown_20260930/README.md)
for hashes, exact proof paths, the real baseline, and preserved rejected
prequalification attempts. The frozen backend update also has a fresh bridge
admission review and native/Memcheck qualification recorded separately.
