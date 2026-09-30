# Independent bridge admission review after simulator wrapper shutdown correction

Disposition: **accepted for observed-shutdown-order admission on the bound implementation, pending fresh runtime qualification**.

Reviewer: `/root/moveit_evidence`. I did not author the new simulator wrapper change, its tests, the observed evidence, or the original causal review. I previously authored the unchanged MoveIt-only backend admission changes. This review covers the current `serve()` delta and reuses the bound prior independent causal assessment; it does not reopen the historical crash investigation.

## Source and evidence inspected

I read the full backend/test diff, the wrapper's surrounding control flow, its real-subprocess regression tests, the native simulator probe, and the preserved lifecycle report/results. I checked all 174 prior review bindings: exactly `scripts/simulator_backend.py` and `tests/test_simulator_backend.py` changed. Every other identity remains exact, including the bridge patch, builder, qualification code, observer, verifier, dependency sources/binaries, and previous causal evidence/reviews. An AST comparison with the reviewed repository parent confirms that only the top-level `serve` function changed; all bridge and MoveIt admission functions and shared imports remain identical.

The wrapper now marks shutdown before forwarding one SIGINT to its owned Gazebo child. Later SIGINT/SIGTERM handlers return while the finally block reaps that child. The first requested shutdown interrupts normal work and is caught only when that stop state is set. The existing ten-second reap timeout remains. A timeout still kills and reaps the child, but now reports failure explicitly. Normal completion returns the actual child status, and previous signal handlers are restored. This addresses the observed wrapper behavior in which a later signal could interrupt reaping and hide forced cleanup behind a clean wrapper status. It does not establish the cause or universal repair of the earlier long-run native Gazebo stall.

The two new subprocess tests cover a repeated shutdown signal during child reaping and a child that ignores shutdown. These are regression tests using a replacement child, not runtime bridge evidence. I inspected the three real Gazebo plus robot-state-publisher/controller lifecycle cycles: each reports four active controllers, SIGINT followed by SIGTERM, wrapper/RSP return zero, no live owned server, and no forced cleanup. Their source, logs, maps, model/configuration and result hashes all match the lifecycle report. These cycles sent no planning or motion goals and are not substitutes for bridge qualification.

Frozen backend SHA256: `255ff862959c5f81b837e99349000970775c46d6c2221f1c665d44e642d7f8c7`.
Frozen test SHA256: `8d12d9361f7e578385ec9ab8f99b3b4ef15ca6560aaf127577266fa07d3b1036`.

## Retained mechanism and claim limits

The prior independently reviewed, core-backed baseline attributes a current FastDDS/ROS shutdown SIGSEGV to main-thread process-static teardown overlapping deferred logging/RMW shutdown. The unchanged correction invokes shutdown before main returns. The source/disassembly argument and five corrected same-handler/same-registry observations establish the joined ordering; uninstall return alone does not. That evidence remains unchanged and bound here. I did not perform a new core or disassembly investigation and do not claim reproduction of the historical allocator SIGABRT or a particular spdlog use-after-free.

GDB changes scheduling; the claim remains observed shutdown ordering with exact executable/dependency identities and the preserved naturally faulting baseline. The current change neither alters that observer nor its event predicates. Bridge environment/middleware isolation, mapped-library checks, ambient-interposition rejection and publication requirements remain unchanged. This review makes no Stage-A, manipulation or hardware-readiness claim.

## Fresh independent validation and required next gate

I executed `/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_bridge_shutdown_overlay.py tests/test_simulator_backend.py -k bridge` after the two source files froze. Result: **101 passed, 66 deselected in 7.81 seconds; exit 0**. The exact command, timestamps and log hash are in `independent-bridge-tests-result.json`. Existing wrapper/backend and parent bridge test logs were inspected separately. The unchanged actual observed batches are mechanically revalidated by `validate_observed(..., include_runtime=False)` using the fresh bound review.

The new review does not publish the bridge overlay. The existing builder must complete 20 new native cycles and five new Memcheck cycles after this review timestamp, plus the shutdown-context regression, with the exact required maps/environment, zero Memcheck errors, no forced cleanup, survivors or new patched cores. Old runtime cycles cannot meet the new timestamp gate. Later changes to bound files invalidate this review.
