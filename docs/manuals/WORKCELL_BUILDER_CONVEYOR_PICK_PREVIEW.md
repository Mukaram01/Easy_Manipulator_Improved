# Workcell Builder Conveyor Pick Preview

This feature adds **offline metadata-only** conveyor detection-to-pick preview.

- Camera detects object upstream in `camera_detection` zone.
- Conveyor flow metadata (`conveyor_flows`) estimates travel distance/time to `robot_pick` zone.
- `pick_ready` is preview metadata only.
- No MoveIt call, no robot command, no RealSense runtime launch, no EPD launch, no real conveyor command.
- No safety certification.
