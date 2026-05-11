# WORKCELL_BUILDER Robot + Tool Compatibility

Robot profiles define robot metadata (base_link, planning_group, mount link, controller family).
Tool profiles define tool metadata (tool_type, mount_link, tcp_frame, controller_hint, grasp/release defaults).
Pair profiles define known pair outcomes (COMPATIBLE, COMPATIBLE_WITH_WARNINGS, INCOMPATIBLE).

Compatibility statuses:
- COMPATIBLE
- COMPATIBLE_WITH_WARNINGS
- UNKNOWN_COMPATIBILITY
- INCOMPATIBLE
- MISSING_ROBOT_PROFILE
- MISSING_TOOL_PROFILE
- MISSING_TCP
- MISSING_MOUNT_LINK
- MISSING_CONTROLLER_METADATA

Auto-fill from profiles:
- base_link
- tool_mount_link
- tcp_frame
- gripper/tool_type
- grasp strategy default
- release strategy default

Warnings vs blockers:
- Unknown compatibility warns and allows generation by default.
- Known incompatible pairs and missing required TCP/mount_link can block generation.
- Manual Override remains available.

Safety note: fake-hardware-first only, generation-time/offline validation only, no motion planning/execution.

Operator flow:
1. Open workcell_builder
2. Select/create scene
3. Select robot asset
4. Select end effector asset
5. Open/check Robot / Tool Compatibility
6. Apply profile defaults
7. Review TCP/mount/controller warnings
8. Configure objects/layout
9. Generate Files
10. Build generated scene
11. Launch with use_fake_hardware:=true
