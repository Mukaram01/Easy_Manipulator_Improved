# Independent observed shutdown review

Disposition: **accepted for the current mechanism and its admission implementation**.

Reviewer: `/root/bridge_history`, separate from producer `root-workcell-closeout-20260929` and the agents that implemented the observed validator, builder and backend changes. This reviewer wrote historical/core inspection artifacts and this review. It did not edit the production implementation or launch runtime/hardware processes.

This review precedes the fresh runtime gate. It does not itself qualify or publish the bridge overlay. The builder must subsequently complete the exact installed patched executable's 20 native and five Memcheck cycles, plus the shutdown-context policy regression, and the final validator must accept all raw results before publication.

## Evidence and scope

Reviewed `observed-evidence.json`, its recursively referenced sources, package/header comparisons, installed disassembly, real baseline and patched maps/events/logs, archived baseline core metadata, core and independent thread/register inspection. All 156 recursively referenced input files matched their recorded hashes during independent inspection. The review JSON additionally binds the 11 standard repository implementation/test files and independent inspection outputs.

The selected source is ros_gz_bridge release commit90cdc5361059a6f949bc004658e7363df33bcffe. Independent Git inspection confirmed the source diff exactly equals the tracked patch, and the patched main SHA256 is fa4a283ae634947384b7aed7f4bd84a97682a74856fc24164227990b33516617. The observed patched executable is SHA256156514f96ff73f3109224d711542b6c1f00cf2e5a69e4e8f1b1dea64214cf601. The bridge library and ROS/RMW/DDS/libc/libstdc++/logging/transport dependencies remain the recorded installed files. The installed bridge executable is f53832701e29bc0f0e2ffa8c3e925ec9de379bca7a43b5c26e324f05b64c3337.

The review concerns `observed-shutdown-order/v1`. Existing deterministic-source and historical qualification paths were not supplied new real-world qualification evidence by this review. Their prior files remain separate. Synthetic fixtures exercise admission rejection rules; they are not runtime or crash evidence.

## Causal adequacy

The real baseline PID7638 received300 strings. Its signal worker LWP7646 entered Context::shutdown and rcl_logging_fini; main LWP7638 then entered exit. The worker subsequently faulted in FastDDS RTPSWriter::new_change, reached through RMW publisher cleanup during logging finalization. The archived core independently confirms SIGSEGV and the same executable, PID/tracer, maps and fault instruction.

At the fault, the main thread was in `__run_exit_handlers` destroying FastDDS DomainParticipantFactory, participants, publishers and writers. The signal worker was still using FastDDS through rcl_logging_fini. The fault instruction dereferenced a zero pointer. Thus current process-static DDS teardown overlaps unfinished deferred ROS logging/RMW teardown. The record supports this lifecycle-order defect; it does not identify a particular freed object or prove the historical spdlog use-after-free.

An attributable failing baseline is a sufficient existential witness of this ordering defect. Requiring five clean baseline exits would exclude the observed failing schedule. The fatal branch is restricted to the baseline and requires the specific trace/core attribution, real traffic, exact identities and successful owned-process cleanup. The original row's null return code, clean_observation=false and core change are retained. The earlier debugger-killed attempt is not admitted.

## Correction and join

Pinned rclcpp16.0.21 source and installed disassembly show Context::shutdown holds its initialization mutex through logging finalization. The top-level rclcpp::shutdown waits for that call and then uninstalls signal handling for the default context. Uninstall notifies and joins the deferred signal thread. The global SignalHandler is heap-allocated and does not provide an automatic static-destructor join fallback.

The executable patch calls this shutdown before main returns and narrowly handles only an invalid-node error after that node's own context is already invalid. Other errors continue to propagate. Its separate three-case policy test remains required. No logging disable, broad exception suppression or altered signal policy was introduced.

All five final patched observations were individually inspected. Each received300 strings and showed registry drop completion, return from the same deferred handler, main-thread uninstall return, main exit, and destruction of the same registry object, followed by normal process exit. An uninstall return alone would not prove a join because uninstall has an early-return path. The same-handler completion, calling stacks and installed join implementation are necessary parts of this accepted argument. Nested tail-called finish-breakpoint callbacks are not used to infer an unreliable total order.

## Scheduling and isolation

The observer runs the exact real executables with identical reviewed GDB commands, automatic-continue breakpoints and stack/register reads. It contains no inferior function calls, register writes, injected SIGSEGV, scheduler locking or source delays. Continuing a pending natural fatal signal delivers that signal normally. GDB still perturbs scheduling; this is an observed ordering proof and GDB-observed SIGSEGV, not an uninstrumented native baseline crash.

Actual experiment environments use ROS domain196, localhost-only, the dedicated transport partition and rmw_fastrtps_cpp. Traffic is limited to the reviewed Clock, Pose_V and String transport harness. No Fortress, robot control, Stage-A or hardware operation was run by this reviewer. The final qualification must retain FastDDS and the bound dependency identities.

The review found and requested removal of an ambient-environment gap. The final builder rejects LD_PRELOAD, LD_AUDIT, VALGRIND_LIB and VALGRIND_OPTS for alternative qualification. The final validator binds the middleware/localhost environment, exact observer scripts/argv, all relevant raw mappings and their file identities. New rejection tests cover those guards. The final runtime gate requires20 native and5 Memcheck clean cycles after the bound review timestamp, checks raw logs/maps and zero Memcheck errors, and rejects forced kills, surviving owned processes and new patched cores. Publication remains contingent on complete validation and removes stale discovery on failure.

## Independent validation

Executed against the frozen final implementation:

`/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_bridge_shutdown_overlay.py tests/test_simulator_backend.py -k bridge`

Result:101 passed,61 deselected, exit0,4.80seconds. Complete output is `independent-observed-tests.log`. These focused tests exercise accepted records and rejection of missing/mismatched evidence, attribution, identity, join order, environmental interposition and incomplete runtime publication. Both actual baseline and patched batches also passed independent calls to the final `_observed_batch` validator. No ROS runtime or C++ test executable was launched by this review.

## Claim limits

The September23 PID81044 maps/results and Valgrind81563 log remain unavailable. Historical comparison stays categoryC. This record does not claim historical allocator SIGABRT reproduction, post-destruction spdlog registry access or proof that every possible future shutdown is safe. The accepted claim is that a current process-exit ordering defect is evidenced and that the exact joined-shutdown correction supplies the missing order, with final admission still requiring fresh native/Memcheck runtime validation. No Stage-A manipulation readiness follows from this review.
