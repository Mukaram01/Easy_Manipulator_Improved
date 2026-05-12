# Workcell Builder Conveyor Sorting Golden Scenario

This scenario builds a complete conveyor-sorting preview workcell with upstream detection and downstream pick/sort routing.

## Template
- Name: `conveyor_sorting_live_epd_preview`
- Includes robot, 2F gripper, conveyor placeholder, RealSense metadata, camera mount, zones, conveyor flow, class routing, task/planning/grasp preview artifacts.

## Create from Workcell Builder
1. Open Workcell Builder.
2. Go to **Create Scenario Template**.
3. Select **Conveyor Sorting - Live EPD Preview**.
4. Generate scene and preview artifacts.

## Fake-hardware preview
- Keep fake hardware enabled.
- Safe launch pattern: `demo.launch.py use_fake_hardware:=true`.
- No robot execution, no gripper command, no conveyor hardware command.

## Sample EPD publisher
`python3 scripts/publish_sample_epd_snapshot.py --camera realsense_d435i_1 --zone detection_zone_1`

## Live EPD feed connection
- Bridge status may show `waiting_for_live_epd_snapshot` until a snapshot arrives.
- This is expected and should not be treated as failure.

## Class routing
- box -> place_zone_box
- bottle -> place_zone_bottle
- unknown -> reject_zone

## Artifact flow
Task intent preview, planning readiness report, dry-run planning request, grasp strategy, and EMD grasp planner request are generated in the preview folder and connected for dry-run readiness.

## Not real-hardware ready yet
Safety validation and hardware driver readiness are intentionally pending.
