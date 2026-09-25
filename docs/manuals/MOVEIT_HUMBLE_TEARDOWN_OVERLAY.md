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
scripts/build_moveit_teardown_overlay.sh
source /opt/ros/humble/setup.bash
source ~/workcell_ws/moveit_teardown_overlay/install/local_setup.bash
source ~/workcell_ws/install/local_setup.bash
```

An optional first script argument chooses the external overlay directory.
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

Only after both proofs does the script publish `provenance.json` and the
package-share symlink `workcell_teardown_overlay.json`. They bind release and
upstream identities, archive hash, tracked patch hash, installed baseline,
patched library and executable paths/hashes, build logs and repetition results.
Runtime checks independently confirm actual MoveGroup executable and TEM
mappings/inodes before every gate; full-cycle prerequisites require both
identities to match the current process and commissioning build.

Local independent proof for each defect: baseline -11, patched **20/20 exit 0**,
no timeout, changed core inventory or live owned processes. The patched GDB run also exits
normally. Evidence:
`~/workcell_ws/moveit_teardown_overlay/evidence/run-20260923-105813`;
expanded unload/mapping proof: `evidence/baseline-lifecycle-gdb.log`.
The second full Stage-A attempt is preserved at `~/workcell_ws/stage-a1-finish-20260923-104525`.
These independent regressions qualify dependency cleanup; Stage-A physical
gates still require their own live acceptance evidence.
