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
only the tracked patch, and builds only `moveit_ros_planning` against installed
Humble with two build jobs. It never changes `/opt` or vendors MoveIt here.
It requires a baseline SIGSEGV followed by 20 consecutive patched controlled
shutdowns. Each patched cycle must return 0, print `TEM_DESTROYED`, load the
exact expected TEM library in `/proc/PID/maps`, leave no live process in its
owned group, and create no new/changed probe core file. This regression tests
the actual TEM binary, not a manual standalone plugin destruction sequence.

Only after that proof does the script publish `provenance.json` and the
package-share symlink `workcell_teardown_overlay.json`. They bind release and
upstream identities, archive hash, tracked patch hash, installed baseline,
patched library path/hash, build log and repetition results. Runtime checks
must independently confirm actual MoveGroup mappings before commissioning.

Local independent proof: baseline -11, patched **20/20 exit 0**, no timeout,
changed core inventory or live owned processes. The patched GDB run also exits
normally. Evidence:
`~/workcell_ws/moveit_teardown_overlay/evidence/run-20260923-104258`;
expanded unload/mapping proof: `evidence/baseline-lifecycle-gdb.log`.
This closes the independent dependency reproducer, not Stage-A physical gates.
