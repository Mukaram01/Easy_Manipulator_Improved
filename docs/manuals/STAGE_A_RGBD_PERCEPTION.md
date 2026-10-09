# Stage-A RGB-D perception acceptance

This is an isolated, read-only perception slice of the existing Stage-A world.
It does not complete Stage A, prove grasping, commission a ROS bridge, or send
robot/controller goals. EPD remains external and owns inference/deprojection.

## Components and limits

- `stage_a_rgbd_world.py` derives the existing SDF, adding one static RGB-D
  sensor only when `--camera-pose` is present. Disabled output is byte-identical.
  `simulator_backend.prepare(..., rgbd_camera_pose=...)` uses the same derivation;
  no camera bridges are added to the executor's bridge configuration.
- Renderer is selectable (`ogre`, default; `ogre2`). On this workstation Ogre2
  published blank RGB and all-infinite depth; Ogre produced actual cubes.
  This finding is confined to this installed renderer stack.
- Sensor geometry is native 512×512, horizontal FOV 1.047 rad, near/far 0.02/5 m.
  RGB8 and float32 axial depth in metres share one sensor and optical frame
  `stage_a_camera_optical_frame` (+x right, +y down, +z forward).
- The optional C++ tool builds EPD's existing `ort_base.cpp` / `p3_ort_base.cpp`
  from a **read-only external source directory**. It creates no second inference
  implementation and never rewrites EPD session files. It requires the external
  TorchVision RGB01/full-image-mask P3 API and geometry-quality helper used in
  the verified cube experiment. Models and labels are command-line inputs.
- This profile is for the existing 25 mm cubes. Detection confidence is ≥0.80;
  masks are thresholded at 0.5. Finite 0.02–5 m mask depth is median/MAD filtered,
  requires ≥12 retained points, ≥16 mask pixels, ≥20% initially valid depth and
  ≥60% retained valid-depth support. Broad/depth-ambiguous masks are rejected.
  Observed per-axis span above 45 mm exceeds a 25 mm cube's diagonal and is
  rejected, rather than averaging merged instances into a fabricated position.
- EPD's `populateMaskedDepthCentroid` supplies metric deprojection and the
  centroid. Output explicitly means **visible-surface centroid**, not an
  inferred volume centre. No dimensions, shape or orientation are invented.
  PlanningScene collision insertion remains BLOCKED without observed dimensions.
- IDs are detection/capture scoped; no temporal tracking is claimed. The output
  is the existing `workcell_perception_snapshot/v1` contract. Timestamp is the
  simulation stamp in nanoseconds, with `source.clock_domain` explicit. CPU
  inference yields a **frozen snapshot**, not a current live ROS observation.
  Do not relabel simulation stamps as wall time or feed them to execution.
  The Workcell Studio source adapter rejects these simulation-clock snapshots
  in live mode and clears its previous observation; explicit replay preserves
  the original frames, timestamps and surface-centroid semantics.
- Acquisition requires exact RGB/depth/calibration timestamp equality and age
  ≤1 s against the same simulator clock, rejects future stamps, frame mismatch,
  missing calibration, nonzero distortion, incorrect image layouts and invalid
  geometry. Timeout is 30 s and rejected image/depth evidence is retained.
- World export is explicit. It checks live scene camera pose against the six
  configured pose values and requires a static camera in the live generated
  SDF. The generated camera link/sensor have identity relative poses; arbitrary
  imported sensor hierarchies are outside this profile. Pose-bearing normalized
  inputs are rejected by this centroid-only conversion.

## Exact workstation commands

Run from the isolated RGB-D checkout. Set paths to the external EPD package,
workspace and model files; production code embeds no workstation paths.
The build uses CMake, Humble, external EPD, OpenCV/PCL, Ignition transport11/
msgs8, jsoncpp and tinyxml2. Independent validation also uses protoc, Python
protobuf, NumPy and OpenCV.

```bash
cd /home/user/workcell_ws_rgbd
export EPD_WS=/home/user/epd_ros2_ws
export EPD_SOURCE_DIR="$EPD_WS/src/easy_perception_deployment/easy_perception_deployment"
export CUBE_MODEL=/home/user/stage_a_cube_model/cube_maskrcnn.onnx
export CUBE_LABELS=/home/user/stage_a_cube_model/labels.txt
source /opt/ros/humble/setup.bash
source "$EPD_WS/install/setup.bash"
export RGBD_RUN=$(mktemp -d /tmp/workcell-rgbd.XXXXXX)
export IGN_PARTITION="workcell-rgbd-$(basename "$RGBD_RUN")"
cmake -S scripts/stage_a_rgbd -B "$RGBD_RUN/build" \
  -DEPD_SOURCE_DIR="$EPD_SOURCE_DIR"
cmake --build "$RGBD_RUN/build" -j2
ctest --test-dir "$RGBD_RUN/build" --output-on-failure
python3 -m pytest -q tests/test_stage_a_rgbd.py \
  tests/test_simulator_backend.py tests/test_epd_snapshot_adapter.py
python3 scripts/stage_a_rgbd_world.py \
  --world scenes/ur5_2f_test/worlds/stage_a0.sdf \
  --output "$RGBD_RUN/world.sdf" --render-engine ogre \
  --camera-pose 0.4 -0.217 0.614 0 1.5707963267948966 0
ign gazebo -s -r "$RGBD_RUN/world.sdf" -v 2 > "$RGBD_RUN/gazebo.log" 2>&1 &
RGBD_SERVER_PID=$!
trap 'kill -INT "$RGBD_SERVER_PID"; wait "$RGBD_SERVER_PID"' EXIT
sleep 5
ign topic -l
# Ground truth acquisition is independent and used only for acceptance below.
timeout 10s ign topic -e -n 1 --json-output -t /world/a0/dynamic_pose/info \
  > "$RGBD_RUN/poses_before.json"
"$RGBD_RUN/build/stage_a_rgbd_capture" "$CUBE_MODEL" "$CUBE_LABELS" \
  "$RGBD_RUN/capture" a0 > "$RGBD_RUN/capture.log" 2>&1
timeout 10s ign topic -e -n 1 --json-output -t /world/a0/dynamic_pose/info \
  > "$RGBD_RUN/poses_after.json"
python3 scripts/stage_a_rgbd_snapshot.py "$RGBD_RUN/capture/snapshot.json" \
  --output "$RGBD_RUN/capture/world_snapshot.json" \
  --camera-pose 0.4 -0.217 0.614 0 1.5707963267948966 0
mkdir "$RGBD_RUN/protos"
protoc -I /usr/include/ignition/msgs8 --python_out="$RGBD_RUN/protos" \
  /usr/include/ignition/msgs8/ignition/msgs/*.proto
PYTHONPATH="$RGBD_RUN/protos" python3 scripts/stage_a_rgbd/validate_capture.py \
  "$RGBD_RUN/capture" --poses-before "$RGBD_RUN/poses_before.json" \
  --poses-after "$RGBD_RUN/poses_after.json" \
  > "$RGBD_RUN/independent_validation.json"
cat "$RGBD_RUN/capture/capture.json" "$RGBD_RUN/independent_validation.json"
sha256sum "$CUBE_MODEL" "$CUBE_LABELS"
kill -INT "$RGBD_SERVER_PID"
wait "$RGBD_SERVER_PID"
trap - EXIT
```

The validator uses actual timestamped dynamic poses, not the stale dynamic
model poses cached by `scene/info`. That service supplies only validation box
sizes and camera readback. Bracketing translation change must be ≤0.1 mm;
this is settled-pile acceptance, not moving-object accuracy qualification.
It ray-casts all ten independently measured boxes to count partial visibility
and reports depth residuals both at silhouettes and one-pixel-eroded interiors.
Nearest-surface matching is independent validation, not inferred track identity.
Volume-centre distance is reported separately because surface observations
are not volume-centre estimates.

## Evidence and gates

See `evidence/stage_a_rgbd/README.md` and its compact JSON evidence. Raw images,
depth buffers, protobuf samples, build products and the ONNX model remain local.

The user reports unresolved ROS–Gazebo bridge shutdown-overlay qualification.
Some committed roadmap text describes reviewed qualification; this slice does
not resolve that discrepancy or rerun it. **ROS bridge commissioning is BLOCKED
for this slice** pending existing source/receipt/qualification gates. No bridge
was started and no qualification code changed. The next product step is an
EPD-owned live perception connection through those gates, then independently
observed collision dimensions before PlanningScene/grasp planning.
