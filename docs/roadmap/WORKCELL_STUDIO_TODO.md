# Workcell Studio TODO Roadmap

This roadmap tracks prioritized implementation work from preview capability to live perception and later real hardware readiness.

## P0 - Must complete current camera/conveyor scenario
1. Live EPD feed bridge into Workcell Studio detection snapshot format.
2. Dry-run planning readiness check without execution.
3. Grasp strategy metadata: suction/finger, approach/retreat, object class mapping.
4. Class-to-place-zone routing.
5. Scenario wizard for conveyor pick.
6. End-to-end generated demo scene: camera + conveyor + robot + zones + EPD mapping + task intent preview.

## P1 - Industrial scenario coverage
1. Static table pick-place wizard.
2. Conveyor sorting wizard.
3. Bin picking metadata and demo.
4. Kitting/tray loading metadata and demo.
5. Inspection/reject scenario.
6. Machine tending scene template.
7. Palletizing pattern preview.
8. Multi-bin sorting.

## P2 - Asset library expansion
1. More conveyors.
2. More bins/totes/trays.
3. Fixtures.
4. Machine/CNC placeholder.
5. Pallet asset.
6. Safety fence/light curtain assets.
7. Camera mount variations.
8. More robot bases/pedestals.

## P3 - Runtime readiness
1. Fake execution runner.
2. MoveIt dry-run planning check.
3. EPD live topic subscription.
4. RealSense launch integration as optional adapter.
5. Hardware driver profiles later.
6. Real robot safety checklist later.

## P4 - Product polish
1. Scenario wizard UI.
2. Better visual layout editor.
3. Demo scene generator.
4. One-click readiness pack.
5. Export PDF/HTML report.
6. Screenshot/preview generation.
7. Better docs and onboarding.


## live_epd_feed_bridge update
- Added live EPD feed bridge preview path (metadata-only).
- EPD GUI remains separate.
- No robot/gripper/conveyor command, no MoveIt call, no forced RealSense launch.
