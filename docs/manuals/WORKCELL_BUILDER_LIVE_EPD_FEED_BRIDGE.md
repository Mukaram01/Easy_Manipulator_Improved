# Workcell Builder Live EPD Feed Bridge

This feature adds a **metadata-only live perception bridge** for Workcell Studio.

- EPD GUI remains separate.
- Workcell Builder consumes `/workcell_studio/epd_detection_snapshot_json` (`std_msgs/String`) JSON snapshots.
- Bridge mode is `live_adapter_metadata_only`.
- No robot motion, no MoveIt plan calls, no gripper commands, no conveyor hardware commands.
- Workcell Builder does not auto-launch RealSense or EPD.

## Test quickly

```bash
python3 scripts/publish_sample_epd_snapshot.py --topic /workcell_studio/epd_detection_snapshot_json --camera realsense_d435i_1 --zone detection_zone_1
```

Live artifacts are written to `<scene>/preview/`:
- `live_epd_detection_snapshot.{yaml,json}`
- `live_epd_detection_mapping.{yaml,json}`
- `live_task_intent_preview.{yaml,json}`

If native EPD output differs, add a small exporter/shim to translate into the snapshot schema topic.
