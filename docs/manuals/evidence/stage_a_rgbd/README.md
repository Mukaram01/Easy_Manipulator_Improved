# Stage-A RGB-D evidence — 2026-10-09

Scope: optional simulated cube perception only. Branch
`codex/stage-a-rgbd-perception`, based on published Stage-A1 commit `fe1f5ea3`.
The dirty Stage-A1 checkout and external EPD source were not edited.

| Acceptance | Result / evidence |
|---|---|
| A: RGB/depth/calibration | PASS — actual Fortress transport, 512×512 RGB8 / float32 metric depth; all stamps 36.940 s simulation time |
| B: aligned geometry | PASS for cube interiors — same sensor, intrinsics and optical frame; independent ray cast over 2,832 interior pixels: P95 depth residual 0.000149 mm. Including raster silhouette edges: 3,677 pixels, P95 12.074 mm; edge accuracy is not claimed |
| C: actual EPD segmentation | PASS — five detections ≥0.80 through external P3 C++ CPU inference |
| D: finite 3D observations | PASS — four accepted; one merged mask (58 mm observed span) rejected rather than averaged |
| E: camera/world transform | PASS — live camera pose matches configured pose; live generated SDF confirms static camera; link/sensor relative poses checked identity |
| F: normalized contract | PASS — existing `workcell_perception_snapshot/v1` validator accepts optical and world outputs, no invented dimensions/orientation |
| G: safe rejection | PASS — focused tests cover absent/invalid depth, calibration, nonfinite depth, ambiguous/broad masks, stale/mismatched/future stamps and unavailable/invalid transforms; live no-source timeout rejects without output observations |
| H: disabled behavior | PASS — byte-identical disabled world and focused existing simulator-backend regressions |
| I: no execution | PASS — physics world has no robot; capture calls only read-only transport APIs; no ROS bridge, ROS execution client or controller launch |
| ROS bridge commissioning | BLOCKED — existing shutdown-overlay qualification gates were not bypassed or rerun |
| Full Workcell Builder colcon build / GUI / task execution | NOT RUN — this slice built its optional C++ adapter only; no task or execution milestone claimed |

Measured nearest-box **surface** errors: **0.0031, 0.0064, 0.0103 and 0.0278 mm**.
These ideal-simulation errors do not establish real-camera precision. Distance
to each box's volume centre is **12.496–12.526 mm**, as expected for observed
surface centroids; no volume-centre/grasp accuracy is claimed.

Independent ray casting found all ten boxes at least partially visible, zero
fully occluded. Five detections produced four geometrically accepted instances;
six visible boxes have no accepted observation. This is not ten-object recall.
Ground truth came only from timestamped `dynamic_pose/info` samples bracketing
the capture; translation change was zero, orientation change 4.22e-8 rad.
`scene/info` dynamic poses were found stale and are not used for error estimates.

Build: PASS, CMake + external EPD sources with Humble; one CTest geometry
executable passes. Focused Python regression suite: **82 passed** on the published baseline.
CMake emitted a jsoncpp search-path warning from the existing EPD overlay;
the built executable ran successfully. ONNX Runtime emitted unused-initializer
warnings from the existing model. No model binary, raw image, depth buffer or
build artifact is committed.

Ogre2 on the installed stack produced a blank image and 262,144 infinite depth
pixels despite synchronized topics. The bounded capture rejected it. A
renderer-only comparison with Ogre produced the recorded working result. The
optional generator therefore defaults to Ogre and retains explicit Ogre2
selection. This does not qualify every renderer/platform combination.

Model SHA256: `21362d62d816bd684f2b5c7769d0bcd1cd568af86419788fca9d095c46f2bad2`.
Labels SHA256: `adeeb37af7c067d456ea0fb2978c9bc4a242bea4d1fbb5a7b53072d486d7e113`.
EPD workspace used `/home/user/epd_ros2_ws`, runtime inputs under
`/home/user/stage_a_cube_model`; these are evidence paths, not production defaults.
Local raw bundle: `/tmp/workcell_rgbd_release_capture`; build:
`/tmp/workcell_rgbd_build`; rejected no-source log:
`/tmp/workcell_rgbd_missing_source.log`. Temporary paths are not durable storage.

Exact acceptance commands, dependency requirements, quality policy, clock
semantics and next step: [manual](../../STAGE_A_RGBD_PERCEPTION.md).
Rollback: revert this feature commit; the canonical SDF and all existing bridge
qualification, task-resolution, preplanning and execution gates are unchanged.
