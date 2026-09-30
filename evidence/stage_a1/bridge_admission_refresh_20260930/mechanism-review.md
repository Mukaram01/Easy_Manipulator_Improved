# Independent bridge admission review after MoveIt qualification changes

Disposition: **accepted for observed-shutdown-order admission on the exact implementation bound below, pending fresh runtime qualification**.

Reviewer: `/root/stage_a_contract`. This reviewer did not author the new MoveIt/backend changes and performed this review after their author froze the backend and tests. The reviewer previously authored portions of the unchanged bridge qualification validator; this is therefore an independent review of the new admission delta, not a replacement claim of independence for the original validator. The separately authored prior causal/core review by `/root/bridge_history` is retained and explicitly reused.

## Scope and preserved evidence

The existing mechanism review JSON and Markdown in `diagnostics/bridge-closeout-20260929` remain unchanged. Their exact identities are bound by this new review, along with the unchanged observed evidence. No historical crash investigation was reopened, no raw evidence was relabelled, and no bridge or ROS runtime was launched here.

All 166 implementation identities from the prior review were checked against their current files. Exactly two changed: `scripts/simulator_backend.py` and `tests/test_simulator_backend.py`. All other bindings match, including the bridge patch, observer scripts, builder, verifier, qualification validator, context policy test, source/dependency records and retained evidence. Both actual observed batches passed fresh calls to the unchanged `_observed_batch` validator. This is mechanical validation of the preserved records, not a new runtime observation.

## Reviewed admission delta

The inspected diff changes exactly four top-level backend functions: `read_moveit_overlay`, `active_moveit_overlay`, `verify_moveit_overlay_maps` and `moveit_overlay_identity`. It requires the qualified MoveGroup default-capabilities library path/hash, requires that exact library in live mappings, includes its digest in the MoveIt identity tuple, and invokes the separate active-plan shutdown proof validator when admitting a MoveIt overlay. Its new import is local to MoveIt admission.

AST comparison confirms that all six bridge functions and all other backend top-level units are unchanged. In particular, observed-method dispatch, source/review binding, actual bridge executable/dependency identity checks and runtime publication requirements are unchanged. The changed tests extend MoveIt fixtures/rejection checks; existing bridge tests are unchanged. This review does not independently qualify the new MoveIt lifecycle fix or its active-plan runtime records.

Frozen backend SHA256: `dfb8ba90f222eaee9fddd7125e218b23efd4cf1eb1be677460836a15573b985d`.
Frozen backend-test SHA256: `7799b13d6e2137770856ab92db5f4a61e91ff497d7629c59981d53dc8bf9c604`.
Observed evidence SHA256: `225007d8f0d290bfee86a11da8b7278db1b96140dd3f71fc85c4ead9f87cdfbf`.

## Retained causal assessment and limits

The prior independent review attributes the real baseline SIGSEGV to overlap between main-thread process-static FastDDS teardown and unfinished deferred ROS logging/RMW shutdown. Its archived core, original null return code/failure state and raw stacks remain unchanged. That is an existential witness of the current lifecycle ordering defect; it is not a reproduction of the historical allocator SIGABRT or proof of a particular spdlog use-after-free.

The retained correction calls shutdown before main returns. The prior source/disassembly assessment establishes the Context shutdown and deferred-handler join relationship. Five corrected observations show registry drop, deferred-handler return, main-thread uninstall return, main exit and same-registry destruction in order. This new review relies on that bound independent causal assessment and freshly validates its unchanged records; it does not claim a new core/disassembly examination. Uninstall return alone and shared nested return PCs are not used as substitutes for the joined-order argument.

GDB changes scheduling. The retained claim remains an observed ordering proof with exact executables/dependencies and a naturally faulting baseline, not an uninstrumented baseline crash. Source/domain/environment isolation and ambient-interposition rejection remain unchanged. No manipulation, hardware or Stage-A readiness follows from this review.

## Fresh independent checks

Executed after backend freeze:

`/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_bridge_shutdown_overlay.py tests/test_simulator_backend.py -k bridge`

Confirmed result: **101 passed, 64 deselected, exit 0, 9.10 seconds**. The command, real start/end timestamps, exit status and exact log hash are in `independent-bridge-tests-result.json`. A first invocation also produced 101 passed; it was repeated once to preserve an explicit exit-status receipt. Both logs are retained. These are synthetic admission/regression tests, not real native or Memcheck cycles.

`admission-source-and-evidence-check.json` records the AST comparison, all prior binding comparisons and fresh actual baseline/patched batch validation. `simulator-backend.diff` preserves the reviewed source/test delta.

## Required next gate

This review does not publish or qualify an overlay by itself. Because the admission file's full hash changed, the builder must run **20 new native cycles and 5 new Memcheck cycles after this review's actual timestamp**, plus the existing shutdown-context regression. It must validate exact mappings, runtime environment, clean owned process exits, zero Memcheck errors, no forced cleanup, no survivors and no new patched cores. Prior 20+5 results cannot satisfy this new review timestamp. Any later change to a bound implementation file invalidates this review and requires renewed review/runtime admission.
