# Feature 3B1 — selected-camera RGB-D preview and readiness diagnostics

## Scope and truth

The existing Workcell Builder **Task → Object input → Live camera source** now
has a **Capture RGB-D preview (read-only)** action for either
`simulated_rgbd` or `realsense_d435i`.

This is an inspection/diagnostic feature, **not** certified live perception,
a camera launcher, continuous video, or pick/place execution. It is stacked on
Feature 3A (the source selection and TaskIntent v2 persistence branch).

The command `scripts/workcell_camera_probe.py` inspects the **saved**
`config/workcell_builder_task_intent.yaml` and the scene's existing
`environment.yaml` camera metadata. There is no competing camera or task
configuration file. It subscribes to exactly the configured RGB/depth topics,
and a CameraInfo topic when available. It also listens to the existing
`/workcell_studio/epd_connector_status` as informational diagnostics.

It does not start Gazebo, a D435i driver, EPD, MoveIt, controllers or a robot.
It publishes no commands and cannot modify scene authoring or save task input.
The ROS connection is bounded (5 seconds in the GUI), and its temporary PNG
files are destroyed when the capture UI consumes them.

## Operation

1. Create/open a cell with an enabled physical camera asset and authored
   `camera.frame_id`, `camera.rgb_topic` and `camera.depth_topic`.
   `camera.camera_info_topic` is recommended; if absent, the probe *infers a
   diagnostic topic hint only*, explicitly flagging that it is not calibrated.
2. In **Task → Object input**, select **Live** and either **Simulated RGB-D**
   or **RealSense D435i**. **Save Task** or **Save Layout**.
3. Start the appropriate camera and EPD externally in their qualified environment.
   Preview does **not** launch either source.
4. Click **Capture RGB-D preview (read-only)**. The Task tab displays still
   RGB/depth images if received, and exact missing-topic/frame/CameraInfo/EPD
   readiness warnings. Depth preview is grayscale visualization only: 16UC1
   is displayed across a fixed 0–3000 raw-unit range, 32FC1 across 0–3m.
   Neither preview measures shape, calibrated distance or collision geometry.

For headless diagnostics (ROS Humble sourced):

```bash
python3 scripts/workcell_camera_probe.py \
  --scene scenes/ur5_2f_test \
  --camera-source realsense_d435i \
  --output-dir /tmp/workcell-camera-diagnostic \
  --timeout 5
```

This command requires the **saved task** to select `live_epd` +
`realsense_d435i`; otherwise it blocks. A different scene may be used.
It prints one JSON report with `PREVIEW_CAPTURED` or `BLOCKED`. Even when
transport succeeds it always reports `planning_ready: false` and
`robot_motion_commanded: false`. Actual calibration, TF and collision geometry
remain separate prerequisites.

## Fail-closed behaviour

- Unsupported/missing source and unsaved edits cannot start the probe.
- No inferred default camera sensor: source must match saved task exactly.
- No missing RGB/depth frames are replaced with stock/test images.
- Unsupported/truncated pixel encoding blocks the affected image.
- CameraInfo absence, unverified calibration, RGB/depth frame disagreement
  and missing matching EPD status are surfaced explicitly. ROS topics alone
  cannot attest that a physical D435i or simulator was the publisher.
- No camera preview causes a task to become plan-ready. The existing
  `detected_objects/v1` TaskIntent selection, measured collision geometry,
  freshness checks and planning authority are untouched.
- Simulated RGB-D camera clocks can differ from wall time; the probe does not
  rewrite image timestamps or declare simulated frames current for execution.

## Validation and limits

Focused no-ROS tests: `python3 -m pytest -q tests/test_workcell_camera_probe.py`.
Existing source persistence tests: `tests/test_task_intent_authoring.py`.
Qt TaskIntentEditor tests cover explicit camera choice, Save/reopen, and the
capture button's disabled/unsaved modes.

**Not yet demonstrated** in this PR: ROS workstation capture, continuous live
video, camera/EPD launch orchestration, D435i driver presence, Gazebo sensor
bridge qualification, TF/extrinsic calibration acceptance, EPD segmentation
purity, independent 3D geometry or grasp/collision planning from live pixels.
These need separate workstation evidence. Do not merge or advertise full
Feature 3B completion on static tests alone.
