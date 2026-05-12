# Workcell Builder Work Zones

Work zones add **metadata-only** semantic areas for scene authoring and preview.

- `camera_detection`: where a camera detects objects.
- `robot_pick`: where the robot may pick.
- `robot_place`: where the robot may place.
- `conveyor_flows`: upstream detection to downstream pick path metadata.

## Notes
- Static table picking can use overlapping detection and pick zones.
- Conveyor picking should use upstream detection zone and downstream pick zone.
- Place zone indicates release area.
- Conveyor flow in this PR is metadata-only and preview-only.
- Robot wait-until-object-reaches-pick-zone is future runtime work.
- Not safety-certified.
