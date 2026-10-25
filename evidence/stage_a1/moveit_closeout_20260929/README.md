# Additional MoveIt lifecycle evidence — 2026-09-29

**PASS: TEM20/20 and MoveGroup20/20 using FastDDS.** These are additional
dependency checks in the final qualified source environment. Stage-A1 simulator
gates require their own acceptance evidence.

The existing qualified installation is
`/home/user/workcell_ws/moveit_teardown_overlay_stage_a1_20260927`.
Its ordinary installed TEM library and MoveGroup executable retain SHA256s:

- TEM: `39807461959d00318d8a4e00db99009b7f269ccbacba620d10b779448f1ff6fc`.
- MoveGroup: `e79a53216e74c270702f12f881226bf07ad879b5fea723afd91e0c57e0fe1fe1`.

All 40 new cycles reached readiness and exited 0 after owned SIGINT, with no
timeout, changed core inventory or owned process survivors. The audit checked
every executable/library mapping against its current device and inode and
hashed 180 distinct mapped shared libraries. Every cycle mapped FastDDS and
excluded the old MoveIt overlay. Both runtime admission checks passed.

The original two baseline SIGSEGV proofs, 40 patched successes and published
MoveIt provenance were preserved byte-for-byte. No rebuild or publication was
performed. The TEM reproducer and one-joint MoveGroup configuration requested no
simulator or hardware motion.

## Source environment and evidence

A clean `env -i` shell sourced, in order:

1. `/opt/ros/humble/setup.bash`
2. `/home/user/workcell_ws/moveit_teardown_overlay_stage_a1_20260927/install/local_setup.bash`
3. `/home/user/workcell_ws/install/local_setup.bash`
4. `/home/user/workcell_ws/ros_gz_bridge_shutdown_overlay_stage_a1_20260927/setup.bash`

The checks used `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=197` and
`ROS_LOCALHOST_ONLY=1`. The workspace's `install/setup.bash` currently chains the
old MoveIt installation; the verified command used `install/local_setup.bash`.

- [summary.json](summary.json): identities, environment, audit counts and exact
  hashes/paths for the full audit, admissions and executed scripts.
- [tem-results.json](tem-results.json): all 20 TEM cycle records.
- [move-group-results.json](move-group-results.json): all 20 MoveGroup records.

Complete logs, process maps, script snapshots and verification remain local at
`/home/user/workcell_ws/diagnostics/moveit-closeout-20260929`. The two result
files above are byte-identical copies from that directory. Large logs and maps
are referenced by the bound audit instead of duplicated in this repository.
