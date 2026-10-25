# Bounded bridge leak diagnostic — 29 September 2026

## Finding

The separate full leak-check run reproduced **336 definitely lost bytes in six blocks and 293 indirectly lost bytes in five blocks**. Allocation stacks attribute the 629 bytes to dynamic-loader bookkeeping during ROS typesupport loading and to the bridge component's class-loader registration metadata. The metadata is allocated by the installed bridge library's static initializer before `main`; the loader allocations occur during node/publisher startup. None of these allocation stacks originates in the patched shutdown/join code.

This is a **non-gating dependency leak finding for the existing shutdown qualification contract**. The evidence does not require altering the shutdown fix or its admission checks. It does establish that the executable is not leak-free. It does not establish that these allocations are intentional, harmless under every workload, or bounded during repeated in-process component load/unload. The finding remains recorded rather than suppressed.

## Qualification and diagnostic are distinct

All five intended Memcheck runs under `ros_gz_bridge_shutdown_overlay_stage_a1_20260927/evidence/run-20260929-212818-088111/memcheck` report `ERROR SUMMARY: 0` with the same 336/293-byte leak totals. Their reviewed command uses default leak-summary mode. Those runs satisfy the declared memory-access/lifecycle contract; they do not constitute a full leak check. Their logs/results and published provenance remain unchanged.

The one completed additional diagnostic used a copied verifier and byte-identical copied bridge configuration outside the repository. The only verifier change adds `--leak-check=full --show-leak-kinds=definite,indirect --default-suppressions=no`. The reviewed error-exit code of 99 remains in effect. No custom suppressions were supplied, default suppressions were disabled, and the log reports zero suppressed bytes/contexts.

The diagnostic received 300 real messages, then sent the same SIGINT shutdown sequence. Publisher PID 12185 returned 0; Valgrind/bridge PID 12186 returned **99**. The log reports **15 errors from 15 contexts**. The copied verifier correctly records `passed: false` and exits 1. There was no core change, forced cleanup, or surviving process in this completed cycle. No invalid read/write/free or uninitialized-value report appears in its log. Full leak checking counts definite and possible leak records as errors by default; selecting definite/indirect display does not change that error policy. The possible-leak stack records are therefore outside this requested display and have not been individually attributed.

Local Valgrind 3.18.1 documentation explicitly describes this distinction; the relevant excerpt is retained in `valgrind-leak-error-semantics.html.txt`. The additional failure is preserved in `cycle/results.json` and is not presented as another passing qualification run.

## Exact allocation ownership

Full stacks appear in `cycle/01-memcheck.log`.

| Lost allocation | Allocation stack / ownership |
| --- | --- |
| 64 direct bytes, one block | glibc `malloc -> resize_scopes (dl-open.c:295) -> dl_open_worker_begin -> dlopen -> rcutils_load_shared_library -> rcpputils::SharedLibrary -> librosidl_typesupport_c -> rmw_create_publisher -> rcl_publisher_init -> rcl_node_init`, while constructing the bridge node. Dynamic-loader scope bookkeeping. |
| 120 direct bytes, one block | glibc `malloc -> _dl_map_object_deps (dl-deps.c:438) -> dlopen -> rcutils/rcpputils SharedLibrary -> librosidl_typesupport_cpp -> rmw_fastrtps_shared_cpp::register_type_object -> rmw_create_publisher -> rclcpp::PublisherBase -> tf2_msgs publisher -> BridgeHandleGzToRos::StartPublisher`. Dynamic-loader dependency bookkeeping for typesupport. |
| 136 direct bytes, one block | The same `_dl_map_object_deps` allocation path through C++ typesupport and the FastDDS publisher creation path for the TF bridge. |
| Zero bytes, two blocks | glibc `malloc -> _dl_find_object_update (dl-find_object.c:791) -> dlopen`, through C++ typesupport loading for the String publisher and type-object registration. These explain the six-block count despite adding no bytes. |
| 16 direct bytes, one block | `operator new -> libros_gz_bridge.so + 0xa60fe3 -> call_init -> _dl_init`, the component registration factory. The corresponding installed disassembly allocates 0x10 bytes, calls `AbstractMetaObjectBase` construction, and adds an owning class-loader entry. |
| 293 indirect bytes, five blocks | The factory's metadata: 152-byte `AbstractMetaObjectBase` implementation; strings of 31, 35 and 67 bytes; eight bytes through `addOwningClassLoader`. Each stack comes from the same installed-library static initialization path. |

Direct total: `64 + 120 + 136 + 0 + 0 + 16 = 336`. Indirect total: `152 + 31 + 35 + 67 + 8 = 293`.

The checked-in release source ends in `RCLCPP_COMPONENTS_REGISTER_NODE(ros_gz_bridge::RosGzBridge)`. The installed registration macro expands to a class-loader `NodeFactoryTemplate` registration. Installed `class_loader_core.hpp` creates a `new MetaObject` and stores its raw pointer in the global factory map. Its comments also explain registration before `main` when a plugin-containing library is directly linked. This supports the factory attribution and allocation lifetime, without claiming its missing cleanup is intentional. Source excerpts, binary disassembly and identities are retained here.

## This predates the current qualification change

The five existing records under `diagnostics/stage-a1-20260927/bridge/fixed-memcheck-nonqualifying` also each report 336 direct bytes/six blocks, 293 indirect bytes/five blocks, and zero default Memcheck errors. Their results identify the same patched executable SHA256 `156514f96ff73f3109224d711542b6c1f00cf2e5a69e4e8f1b1dea64214cf601` and bridge library SHA256 `7806fe789ceba89ba203a3e055dae7e6a7e8e67a65931c2b877d596055200be3`. `sept27-memcheck-comparison.json` binds those records and summaries. This shows that these leak quantities predate the new observed-order qualification. It does not promote those earlier nonqualifying runs or prove the exact historical allocation stacks were identical.

## Identity, isolation and limits

The completed diagnostic used the real published patched binary, original compiled publisher and exact configuration content; no executable, source, qualification record, review or provenance was edited. All 21 declared qualified runtime dependencies were mapped with their expected file hashes. `final-identity-check.json` also confirms the input files and provenance were unchanged. `environment.json` records FastDDS, localhost-only, domain 196, Gazebo partition `workcell_bridge_shutdown_proof`, and empty LD_PRELOAD, LD_AUDIT, VALGRIND_LIB and VALGRIND_OPTS. The environment was constructed from an empty shell environment plus `/opt/ros/humble/setup.bash`; no hardware or simulator was started.

Two setup-only failed attempts are retained separately: ROS setup rejected nounset before any launch, then a relative output path caused Valgrind to reject its log filename before loading the bridge; the publisher from the latter attempt was cleaned. The corrected absolute-path command ran the single completed bridge diagnostic. No further runtime cycles were run.

The diagnostic also reports 3,834,672 possibly lost bytes and 7,285,320 still-reachable bytes. Their individual allocations were outside the bounded definite/indirect investigation. The result supports a narrow statement: shutdown qualification passed its specified native, ordering and default Memcheck gates, with separately recorded unresolved exit-time leaks. A general leak-free, memory-safe-for-all-workloads, or historical September 23 crash-reproduction claim is unsupported.
