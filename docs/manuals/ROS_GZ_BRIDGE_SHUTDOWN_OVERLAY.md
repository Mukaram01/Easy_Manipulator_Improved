# Humble bridge shutdown overlay

`ros-humble-ros-gz-bridge` 0.244.26 can return from `parameter_bridge` while
rclcpp's deferred signal thread is still shutting down logging. Main then runs
spdlog's static destructor concurrently with `rcl_logging_external_shutdown`.
The observed result was `free(): corrupted unsorted chunks` / SIGABRT.

The patch adds `rclcpp::shutdown()` after `rclcpp::spin()`. In installed rclcpp
16.0.21, this waits on the context shutdown mutex and uninstalls/joins the signal
handler before main returns. It does not disable logging, catch the abort,
change signal policy, or modify `/opt`.

The build uses the ROS release source at
`ros2-gbp/ros_ign-release` commit `90cdc5361059a6f949bc004658e7363df33bcffe`
(tag `release/humble/ros_gz_bridge/0.244.26-1`). The pristine main is identical to
upstream `gazebosim/ros_gz` commit `63793185bbbe58732b6eac8613937dc1bcb6ea2c`;
its SHA256 is `92c215bec11c013d403ae037b48b3aa670ca7e7aacc1d4904683e41ab017760d`.
The build verifies the installed version, installed public header, pinned main,
and exact source patch. Only the executable is rebuilt; it links the installed
`libros_gz_bridge.so` and existing transport/RMW libraries.

## Reproduce and qualify

Source the workstation ROS environment, then run:

```bash
source /home/ubuntu/workcell_ws/install/setup.bash
/usr/bin/python3 scripts/build_ros_gz_bridge_shutdown_overlay.py \
  --output /home/ubuntu/workcell_ws/ros_gz_bridge_shutdown_overlay \
  --valgrind /path/to/valgrind
```

For a locally extracted Valgrind package, also pass `--valgrind-lib` pointing to
its `usr/libexec/valgrind`. No package installation is required.

The verifier uses real Clock, Pose_V and 32 KB String transport traffic and a
real ROS consumer in isolated domain 196 and partition
`workcell_bridge_shutdown_proof`. It requires receipt of at least 300 strings
before one SIGINT per owned process. It records actual executable/library maps,
exit codes, core-inventory changes and surviving process-group members. There
is no Fortress instance or robot motion.

Qualification requires evidence that the installed baseline reproduced SIGABRT, followed by
20 consecutive patched native shutdowns and five patched Memcheck shutdowns
with zero reported memory errors. Any abnormal patched exit, forced cleanup,
new bridge core or surviving owned process fails the build. The manifest is
removed before building and emitted only after these checks pass.

Because the baseline race depends on scheduling, a bounded baseline run may
finish without reproducing it. Preserve that result. For subsequent builds,
`--baseline-results /path/to/preserved/results.json` can reuse a previously
captured SIGABRT proof only when its executable and library hashes still match.
Every rebuild still runs all 20 patched native and five patched Memcheck cycles.

`provenance.json` records source/patch/executable/library hashes, the installed
package version and hashed proof results. `setup.bash` exports
`ROS_GZ_BRIDGE_SHUTDOWN_OVERLAY`; it does not shadow the whole ROS package.
Commissioning must select the qualified absolute executable and independently
verify the actual loaded executable and installed bridge library.

## Original evidence

The actual Stage-A PID 78713 crash is preserved under
`/home/ubuntu/workcell_ws/bridge-abort-review-20260923-123100/apport`.
The independent traffic-only baseline reproduced the same abort on cycle 7.
`memcheck/valgrind-81563.log` identifies the use-after-free: the deferred signal
thread enters `spdlog::details::registry::drop` after the main thread frees the
registry during `__run_exit_handlers`. This establishes the shutdown race;
the bridge handle ownership cycle was investigated but is not the reason for
this patch.

The qualified run is
`/home/ubuntu/workcell_ws/ros_gz_bridge_shutdown_overlay/evidence/run-20260923-115301`:
20/20 native and 5/5 Memcheck cycles passed, with zero Memcheck errors, abnormal
exits, new bridge cores or surviving owned processes. The baseline directory
preserves the original PID 81044 abort and documents its evidence normalization;
two later bounded baseline runs that did not reproduce the race are retained
separately. The installed executable and library remained unchanged.

### Restoration and distinct shutdown exception (2026-09-27)

No compatible preserved logging-lifetime baseline was found in the local
workspaces/project artifacts; `/home/ubuntu` is absent. The earlier 20 native
non-reproductions remain preserved. A bounded five-cycle probe changing only
RMW selection to CycloneDDS also exited normally. Neither observation qualifies
the required installed-baseline logging-lifetime SIGABRT.

A separate patched diagnostic passed 20 native cycles but failed Memcheck cycle
5: the lazy bridge callback queried subscriptions after its node context became
invalid. The resulting `RCLError` caused SIGABRT, with zero Memcheck errors.
This is a distinct defect, **not** the required historical lifetime baseline.
The baseline verifier now rejects this unrelated exception/unclassified abort
and requires the preserved allocator symptom; that signature remains necessary,
not sufficient to establish the original cause without its lifetime evidence.

The independently reviewed executable patch now handles only
`RCL_RET_NODE_INVALID` when that node's own context is already invalid, then
performs the existing joined shutdown. All other errors still propagate. Three
synthetic C++ policy cases exercise the actual invalid-context graph query and
the two required rethrow cases. Synthetic tests are not baseline reproduction.
The corrected executable, SHA256
`156514f96ff73f3109224d711542b6c1f00cf2e5a69e4e8f1b1dea64214cf601`,
then passed 20 native and five Memcheck diagnostic cycles with zero memory
errors, forced kills, owned survivors or core changes. These results remain
**NON-QUALIFYING** because the required baseline is absent.
All fresh bridge evidence, including the failed cycle and new non-qualifying
checks, is under `/home/user/workcell_ws/diagnostics/stage-a1-20260927/bridge`.
No bridge admission provenance or qualified setup file is published while the
required baseline remains absent.
