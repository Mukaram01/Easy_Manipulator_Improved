# Live camera → world calibration (Stage B4)

This manual covers how a real D435i observation, already normalized by EPD in `camera_color_optical_frame`,
becomes a world-frame object in a Workcell Studio cell. Robot execution stays locked.

## Pieces

| Piece | Role |
|---|---|
| `scripts/camera_world_calibration.py` | Measures reference points live through EPD's ingress, fits world ← optical robustly, and qualifies or rejects the fit. It saves the record with the cell. |
| `scripts/live_world_objects.py` | Turns EPD live snapshots into `detected_objects/v1` world snapshots with refusals and virtual-counterpart binding. It also publishes RViz markers, TF and, optionally, MoveIt collision objects. |
| `scripts/live_perception_session.py` | The existing live session. It re-validates the calibration and the camera serial, then reports world XYZ and planning readiness per detection. |
| Workcell Builder, Live Perception panel | **Calibrate Camera** runs the capture. The table adds a *World XYZ* column, and the status shows calibration residuals. |

## Calibration procedure

1. **Bind the camera.** Bind the cell's live camera as before (Discover Cameras → Save Camera Binding).
   - A camera with only `depth/image_rect_raw` is bound with `depth_alignment: register`, so EPD's ingress registers the depth.
   - This uses EPD's `camera_source realsense_unaligned`.
2. **Place and measure the markers.**
   - Print ArUco markers (`DICT_4X4_50`).
   - Place at least 4, preferably 6, spread over the work area, not in a line, with at least one raised if possible.
   - Measure each marker centre in the cell **world** frame, in metres.
3. **Write the reference points file.** Save the measurements as `<cell>/config/camera_calibration/reference_points.yaml` (`workcell_camera_reference_points/v1`).
   - Every point needs a world position (`world_xyz`).
   - Every point names exactly one target: `aruco_id`, or `pixel` for a feature you identify yourself.
   - `measured_by` and `measurement_method` are required.
4. **Capture.** Click **Calibrate Camera**, or run:

   ```bash
   eval "$(python3 scripts/live_perception_session.py env)"
   python3 scripts/camera_world_calibration.py capture --cell <cell> \
     --points <cell>/config/camera_calibration/reference_points.yaml --output <evidence dir>
   ```

   - It reads the camera serial from the driver's `device_info` service and refuses to run without one.
   - It starts EPD's ingress only and averages each marker over 15 synchronized RGB-D frames.
   - It reads the depth as a median over a 5×5 window of the colour-registered depth.
5. **Check the result.** The fit is **qualified** only when all of these hold:

   | Check | Limit |
   |---|---|
   | Points agreeing within 15 mm | ≥ 4 |
   | Outliers | ≤ 25 % |
   | RMS residual | ≤ 10 mm |
   | Max residual | ≤ 20 mm |
   | Leave-one-out prediction error | ≤ 25 mm |
   | Spread of the points | ≥ 0.10 m |
   | Each point's variation between frames | ≤ 4 mm |

   - If qualified, the record goes to `<cell>/config/camera_calibration/<camera_id>.yaml`, and `perception.live_camera.calibration` references it by SHA-256.
   - A rejected fit is saved only in the evidence directory; the cell is not touched.
   - The record contains the camera serial, frames, the transform (matrix, translation, quaternion), every reference point (world and camera positions, pixel, frames measured, spread), residuals, limits and provenance (EMD commit, points-file hash, who measured and how, colour intrinsics, depth source).
6. **Re-validation on every use.** Each use re-solves from the saved reference points and checks:
   - the hash, camera ID and frames;
   - the current limits;
   - the running camera's serial.

   An edited, mismatched or out-of-limit calibration is `INVALID`. `solve --capture` re-solves a saved capture offline.

## Live handoff rules (`live_world_objects.world_snapshot`)

**Refusals for a whole snapshot:**
- the calibration is missing, unqualified or for another frame;
- the snapshot is not live `workcell_perception_snapshot/v1`;
- another camera or frame;
- a missing timestamp;
- older than 2 s, or stamped in the future;
- the cell has no workspace bounds.

**Refusals per object:**
- the object's frame differs;
- no finite 3D position;
- no extent;
- valid depth ratio below 0.5;
- a world position outside `workspace.bounds`.

**What every accepted object keeps:**
- its camera-frame position, orientation, extent, ROI and depth ratio;
- EPD's object ID and sensor stamp;
- the class and confidence, always `class_verified: false`.

**Planning objects come only from explicit virtual counterparts**, authored in the cell's `environment.yaml`:

```yaml
perception:
  live_counterparts:
    - {id: mouse_1, label: mouse, shape: box, size_m: [0.115, 0.063, 0.044], extent_tolerance_m: 0.02}
```

- A detection binds only when its label matches **and** its measured visible extent fits the authored box.
- Exactly one detection must match the counterpart.
- The planning object uses the counterpart's box at the detection's world pose.
- A mislabelled object, such as the laser meter EPD calls `cell phone`, does not fit a phone counterpart and stays BLOCKED.
- Real objects are never inserted into Gazebo automatically.

## Visualize

```bash
eval "$(python3 scripts/live_perception_session.py env)"
python3 scripts/live_world_objects.py publish --cell <cell> --output <dir>/world_objects.yaml [--apply-planning-scene]
```

- It publishes `/workcell_studio/live_world_objects_json` and `/workcell_studio/live_world_objects/markers`.
- Markers are drawn in `world`: green for a bound counterpart, orange for no counterpart, red for refused.
- It also publishes a static TF `world → calibrated_camera_color_optical_frame`.
- With `--apply-planning-scene`, bound counterparts become `live_<id>` MoveIt collision objects. This changes the planning scene only, never motion.
