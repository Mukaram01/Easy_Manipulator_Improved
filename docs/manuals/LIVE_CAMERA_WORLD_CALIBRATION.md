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

## Status (2026-10-26)

| Part | Status | Evidence |
|---|---|---|
| Implementation | **PASS** | 60 focused tests (calibration, world handoff, live session, replay) and the `workcell_builder` build |
| Physical calibration acceptance | **BLOCKED** | No measured reference coordinates exist yet for a tabletop that corresponds to a Gazebo cell |

Until that acceptance passes, every live object stays camera-frame only. The cell binding remains `uncalibrated` and robot planning is BLOCKED; that is the fail-closed default.

## Tabletop calibration setup (simplest repeatable procedure)

The physical table stands in for the cell's authored table. The reference markers' world coordinates come from the cell, not from ad-hoc floor measurements. The one physical measurement is where the printed board sits relative to a table corner that the cell also defines.

**Minimum physical setup**

1. **Camera.** A rigid D435i mount over a flat table, looking down at the work area from about 0.4–0.7 m. Don't move it after calibrating; moving it invalidates the calibration.
2. **Table corner in the cell.** A table corner that has a counterpart in the Gazebo cell: the cell's table or support-surface box defines that corner's world XY and the table-top height Z. Write that corner's world coordinates down from the cell.
3. **Board.**
   - Generate the A4 board and its reference-points file for the board pose you will use:

     ```bash
     python3 scripts/camera_world_calibration.py board --output <cell>/config/camera_calibration \
       --origin <X> <Y> <Z_table_top> --yaw-deg <yaw> --measured-by "<name>"
     ```

   - Print it at 100 %, and check with a ruler that each black square is **50.0 mm**.
   - Lay it flat with its top-left corner exactly at the authored point, e.g. the table corner offset by a ruler-measured amount; include that offset in `--origin`. Its x edge must lie at the authored yaw.
   - Markers 0–4 are fitted. Marker **5 is held out** as the independent check.
4. **Optional raised point.** Put one extra marker on a block of known height to improve the vertical constraint. Add it to `reference_points.yaml` by hand (`aruco_id`, `world_xyz`).
5. **Visibility.** Lighting good enough that all six markers are visible in the colour image.

**Calibrate**

1. Bind the cell's live camera (Discover Cameras → Save Camera Binding).
   - Raw `depth/image_rect_raw` binds with `depth_alignment: register`.
   - EPD's ingress then registers depth to colour (EPD `camera_source realsense_unaligned`).
2. Click **Calibrate Camera** in the Live Perception panel, or run:

   ```bash
   eval "$(python3 scripts/live_perception_session.py env)"
   python3 scripts/camera_world_calibration.py capture --cell <cell> \
     --points <cell>/config/camera_calibration/reference_points.yaml --output <evidence dir>
   ```

   - It reads the camera serial from the driver's `device_info` service and refuses to run without one.
   - It runs EPD's ingress only and averages every marker centre over 15 synchronized RGB-D frames.
   - Depth is a 5×5 median of the colour-registered depth.
3. **Gate:** the fit is **qualified** only when all of these hold:

   | Check | Limit |
   |---|---|
   | Fitted markers agreeing within 15 mm | ≥ 4 |
   | Outliers | ≤ 25 % |
   | RMS residual | ≤ 10 mm |
   | Max residual | ≤ 20 mm |
   | Leave-one-out prediction error | ≤ 25 mm |
   | Spread of the fitted markers | ≥ 0.10 m |
   | Each marker's variation between frames | ≤ 4 mm |
   | **Independent check:** held-out marker 5, never used in the fit, predicted within | **15 mm** |

   - Without a measured, agreeing check point the calibration is REJECTED. No calibrated object can then reach simulated robot planning.
   - A qualified record is saved as `<cell>/config/camera_calibration/<camera_id>.yaml`, and the live binding references it by SHA-256.
   - A rejected fit is saved only in the evidence directory; the cell is unchanged.
4. **Re-validation on every use.** The record is re-solved from its saved evidence and its hash, camera ID, frames and limits are checked. The running camera's serial must match. Anything else is INVALID.

**Before planning use: object-level acceptance**

1. Author a virtual counterpart for the test object in `environment.yaml` (`perception.live_counterparts`).
2. Place the object at a position you measure independently on the board or table, e.g. centred on a printed cross at a known board offset.
3. Run live perception with `live_world_objects.py publish`, and compare the transformed position with the measured one in RViz and in the world snapshot.
4. Record the difference and the camera-frame data in the evidence directory.

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
