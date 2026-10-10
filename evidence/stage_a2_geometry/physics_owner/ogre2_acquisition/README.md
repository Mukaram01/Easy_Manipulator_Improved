# Native Ogre 2 acquisition witnesses

Baseline `a43dadb5b6547ca3a5178272dd052b72379cfdb7`. **Static CPU camera witness PASS; TOTAL registration/depth qualification BLOCKED.** No genuine Gazebo/EPD capture was attempted. This change instruments the existing opt-in native fixture, not installed libraries or production worlds.

## Implementation and single validation

Public `Ogre::Camera::Listener` callbacks read native camera state immediately before/after scene rendering. Each callback records session, batch, sequence, camera ID/name, authoritative attached-camera count and pointer correspondence, actual view, actual projection and render-system depth projection, native near/far and projection AB, configured dimensions, and the public last-viewport readback. No ideal-matrix substitution is used. Listener registration is removed before camera destruction.

The explicit loop brackets Render calls and all three image readbacks in each batch. Depth/segmentation events originate inside the actual data callbacks; RGB records its completed Copy. Three batches precede three scene PostRender completions. The diagnostic requires ordered complete callbacks/readbacks, stable camera identity/configuration and CPU shader-resource parameters, and exact agreement with final native matrices. A changed view, mismatched projection, viewport or dimensions, stale session/batch, missing callbacks or readbacks fails closed.

Exactly **one** bounded native llvmpipe fixture invocation was run (90-second process bound; completed successfully). It produced nonblank 512×512 RGB, 284 finite depth pixels and semantic labels 11/22. Actual trace: 63 events, 15 scene-pre callbacks, 15 scene-post callbacks and nine image readbacks. RGB used three scene passes in batch 1 and one in each subsequent batch; the validator permits multiple explicitly paired passes. Every observed viewport was [0,0,512,512]. This is last-viewport CPU state, not GPU pixel-centre or sample-position authority.

`acquisition_readback.json.gz` retains the full JSON trace. `rgb8.gz`, `depth.f32.gz` and `labels.rgb8.gz` retain exact native final-batch bytes; depth uses the workstation's little-endian binary32 representation. `provenance.json` records uncompressed image hashes, executable/source hashes, all observed loaded rendering/Ogre/GL/LLVM/driver library hashes, shader resource candidate hashes and session identity. Only final-batch image bytes are retained; callback counts alone do not attest GPU samples for earlier images.

## Separate numerical and authority decisions

| Quantity | Result / exact scope |
| --- | --- |
| Native scalar → binary64 report conversion | Exact for these binary32 camera entries; zero additional conversion error. This does not bound the original native matrix construction or GPU transforms. |
| Matrix-only coordinate discrepancy | ≤ **0.000035238669374079335 pixels**, exact rational evaluation rounded outward over the union of the centered camera frusta. Applies to recorded CPU coefficients before GPU arithmetic/rasterization. |
| Total pixel-registration error | **Unknown** |
| Depth conversion / total depth error | **Unknown** |
| Qualified pixel regions | **Empty**: 0 admitted, all 262144 excluded |
| Gazebo/physics timing and collision identity | **Unqualified**; static native fixture only |

Actual RGB/segmentation focal coefficient: 1.7320507764816284. Depth: 1.7320505380630493. Actual common view is stable across every observed callback. The render-system XY rows match the native projections, but depth/Z rows differ. The existing rational discrepancy evaluator is now gated on acquisition witnesses, rather than accepting an after-batch matrix snapshot alone.

Depth native clipping is approximately [0.017999999225139618, 5.5] m; CPU shader clamp uniforms are [0.019999999552965164, 5] m. CPU projectionParams readback is [-0.0032834731973707676, 0.003283472964540124]. These are measured values, not invented tolerances. Public MaterialManager lookup reads the named depth-camera clones and their selected fragment-program delegates/parameters. **Registry state does not establish that a compositor quad actually bound those values during GPU execution.** Source basenames are observed; on-disk shader hashes are candidates, not a captured GPU executable or resource-stream attestation.

The inspected rendering 6.6.4 source requests D32_FLOAT depth, uses quad far-corner directions and a division to linearize sampled depth, then packs float bits into RGBA32_UINT. The installed final shader uses texelFetch and repeats near/far clamping (including its own 1e-6 branch tolerance). These observations locate the required proof; they do not provide an active attachment-format, sampling, arithmetic or visibility bound. In particular, camera callbacks do not witness the subsequent compositor quad's GPU bindings. No observed residual or chosen border erosion is used as a worst-case bound. Without a bounded raster/visibility model even apparently interior pixels are excluded.

Source references: [Ogre 2.2.5 camera callback implementation](https://raw.githubusercontent.com/OGRECave/ogre-next/v2.2.5/OgreMain/src/OgreCamera.cpp), [rendering 6.6.4 depth camera](https://raw.githubusercontent.com/gazebosim/gz-rendering/ignition-rendering6_6.6.4/ogre2/src/Ogre2DepthCamera.cc). Installed shader paths/hashes are in provenance. Source inspection does not itself qualify the loaded binary.

## Focused tests and reproduction

```bash
cmake -S scripts/stage_a_rgbd -B /tmp/stage_a2_acquisition_build -DSTAGE_A_RENDER_PROBE_ONLY=ON
cmake --build /tmp/stage_a2_acquisition_build --target stage_a_render_identity_probe stage_a_ogre2_image_probe -j1
LIBGL_ALWAYS_SOFTWARE=1 timeout 90s /tmp/stage_a2_acquisition_build/stage_a_ogre2_image_probe ogre2 /tmp/new_acquisition.json --images
python3 -m pytest -q tests/test_stage_a_ogre2_registration.py tests/test_stage_a_render_identity_probe.py -k 'not ogre2_produces_nonempty_images'
```

Both targets build. **36 focused tests PASS, one existing graphics fixture test deliberately deselected** to avoid a second Ogre 2 fixture run. The one manual fixture and saved-trace replay validate the new native acquisition path. Rejection cases cover stale/missing acquisition events, session/order, identities, offset/projection disagreement, viewport/dimensions, incomplete shader resources or nonfinite/changing parameters, blank RGB, invalid/missing labels, nonfinite depth and incomplete formats/batches. Occlusion/touching labels, edge intensity, depth discontinuity and shifted depth/labels are **array-level perturbations**, not independent GPU rasterization experiments: every such uncertain image still admits zero pixels. They do not qualify visibility, subpixel coverage or a total bound. The historical blank SVGA report is replayed as rejection evidence; no second native SVGA fixture was run.

## Stop gate and next action

**Missing proof:** an acquisition-bound witness of the active compositor GPU program/uniforms, depth attachment and sample/raster state, followed by a conservative error/visibility enclosure for that actual path. Smallest next implementation: add read-only compositor-pass acquisition witnesses for those fields; retain unknown bounds until their operations can be enclosed. Camera listener / CPU registry evidence alone is insufficient.

Genuine Gazebo/EPD capture NOT RUN. Renderer-to-DART update alignment, collision-to-visual mapping, unique EPD-mask identity and a current-capture whole-shape penetration proof remain gated. Semantic fixture IDs confer no collision identity. Historical physics bounds were not reused. Original EPD poses/envelopes, 0.1 mm penetration threshold, bridge gates and native contact rules are unchanged. Zero MoveIt/controller/execution goals. Protected Stage-A1 and installed libraries untouched. PR #3176 remains draft. Rollback is the optional probe/diagnostic commit only.

Follow-up: [executing compositor/post-pass GL witness](../ogre2_compositor/README.md) captures actual post-pass GL uniforms, attachments and linked program binaries. Per-draw inputs and numerical/raster enclosure remain unqualified; total authority and the pixel mask remain blocked/empty.
