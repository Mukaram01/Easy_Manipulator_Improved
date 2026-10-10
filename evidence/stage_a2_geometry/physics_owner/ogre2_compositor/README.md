# Executing compositor and post-pass OpenGL witness

Baseline `b73ba340a99381554acbf4d5c49a87f3bf56246b`. **Working compositor/GL readback; registration authority BLOCKED.** Exactly one new native llvmpipe fixture run; no Gazebo/EPD, MoveIt, controller or execution goals. Installed libraries and protected Stage-A1 untouched. Original EPD geometry/envelopes and 0.1 mm physical penetration limit unchanged.

## Working implementation and API scope

The existing camera listener discovers the **current executing compositor pass** through public `Camera::getSceneManager()->getCurrentCompositorPass()`. Its public parent-node/workspace getters identify the actual live workspace. Only public listener subscription removes constness from the getter's view of this mutable workspace; no private/protected storage, ABI offset, registry search or guessed ID mapping is used. Subscription occurs during camera culling, after pre-pass listener iteration. The observer is removed before workspace destruction. It creates no workspace, camera, engine or render update.

`CompositorWorkspaceListener::passPosExecute` records the actual pass/workspace identities, pass type, render-target descriptor, executing quad's actual Ogre Pass and camera, and **OpenGL state at that post-execution boundary**. Read-only GL queries capture the bound linked program/pipeline, link status, scalar/vector uniform values, sampler objects/parameters, draw FBO and texture attachment dimensions/internal formats, viewport, clipping/depth state, sample buffers and subpixel capability. Accessible linked-program binaries are saved. No bind/use/active-texture or draw/scene setter is issued. `glGetError` retrieves/consumes diagnostic error flags; all recorded flags were zero.

CPU pass resources and actual GL query results are separate. The `fragment_program` field is a legacy name: when GL_CURRENT_PROGRAM is nonzero it identifies a **monolithic linked program**, not independently verified fragment-stage inventory. Scene-pass state describes the last bound program, not every object drawn by that scene pass. Clear/other passes need not use a program. Matrix uniforms, uniform blocks/arrays and non-active-unit texture object mappings remain UNKNOWN. Only the currently active texture unit's 2D binding is queried; sampler-unit indices and sampler objects are directly queried for each supported sampler uniform. Shader/resource names alone do not attest compiled executable operations.

All compositor records include session, batch, camera-call identity and acquisition sequence. The Python diagnostic checks each post-pass record lies inside that camera's matching Render call and precedes its readback. The first workspace discovery cannot witness earlier clear/pre-pass activity; all records are explicitly post-pass. Thus this is **post-pass binding evidence**, not a claim that public callbacks expose every draw-time input.

## Single measured fixture

Native image production PASS: 512×512 RGB, 284 finite depth pixels, semantic fixture labels 11/22. CPU acquisition witness remains PASS, with 63 events. New compositor witness: **29 post-pass callbacks** (15 scene, nine quad, five other), all with zero recorded GL query errors. Six distinct nonzero linked-program objects expose binary blobs; their hashes and compressed exact bytes are retained. Twenty-eight callbacks observed a nonzero current program; these are not 28 independently certified fragment executions.

Measured final-batch path:

| Boundary | Actual measured state |
| --- | --- |
| Main depth scene | FBO depth texture object 12, **GL_DEPTH_COMPONENT32F**, 512×512 |
| Depth conversion quad | Linked program 9; depthTexture sampler unit 0; active unit 0 has texture 12; RGBA32UI output object 10 |
| Final depth quad | Linked program 12; inputTexture sampler unit 0; active unit 0 has texture 10; RGBA32UI output object 11 |
| Segmentation scene | Linked program 15; RGBA8 output object 19; only last object's program/uniform state is observed |

Program/texture numbers are measured context-local handles bound to the recorded session, not stable cross-run IDs. Quad pass resources identify DepthCameraFS_GLSL and DepthCameraFinalFS_GLSL. GL depth uniforms read `near=0.019999999552965164`, `far=5`, `projectionParams=[-0.0032834731973707676,0.003283472964540124]`; they agree exactly with that executing Ogre Pass's CPU values. Final texResolution reads [512,512,1,512]. This is consistency evidence, **not a worst-case depth error bound**.

Main image viewports are [0,0,512,512]. A particle auxiliary clear uses 256×256 and is retained, not assumed to be an image mismatch. GL_SAMPLE_BUFFERS and GL_SAMPLES are zero even though GL_MULTISAMPLE is enabled. GL_SUBPIXEL_BITS is eight; this capability alone does not bound transformed vertex positions, clipping, interpolants, shader arithmetic or coverage. Quad depth attachments are shared renderbuffers whose formats were not queried; Ogre descriptors name D32_FLOAT_S8X24_UINT, which is kept separate from the actually queried sampled D32F texture. Disabled scissor coordinates are anomalous in the raw readback; no raster bound is inferred from them.

## Maximum defensible numerical authority

| Layer | Qualified result |
| --- | --- |
| Native binary32 camera entries → binary64 report | Exact additional scalar conversion for recorded entries; does not bound native matrix construction/GPU operations |
| Recorded matrix-only projection disagreement | ≤ **0.000035238669374079335 pixels**, existing exact rational/outward bound, before GPU transforms/rasterization |
| GL versus executing CPU depth uniforms | Observed exact agreement only; no universal numerical enclosure |
| Linked executable provenance | Exact post-pass driver binary bytes/hashes; opaque contents are not a verified operation/precision proof |
| Shader/depth arithmetic error | **UNKNOWN** |
| Coverage, visibility, segmentation correspondence | **UNKNOWN** |
| GPU readback/format conversion error | **UNKNOWN** as a complete end-to-end bound; saved CPU bytes are exact artifacts |
| Total registration/depth authority | **BLOCKED** |
| Qualified mask | **0 pixels qualified / 262144 excluded**, explicit uint8 mask |

No nonempty interior subset can be justified from these records: per-draw vertex matrices/inputs and complete texture bindings are missing, and no verified compiler/arithmetic/raster enclosure exists. Neither eight subpixel bits, nearest samplers, matching labels, observed uniform agreement nor arbitrary erosion supplies those missing bounds. `compositor_authority` always emits the empty mask and cannot be upgraded by caller-supplied PASS or precision fields.

## Tests, provenance and reproduction

**55 focused Python tests PASS; CPU uniform-storage CTest 1/1 PASS; native probe and storage test build PASS.** Safety tests preserve exclusion for wrong/unknown shader state, mismatched CPU/GL uniforms, unsupported attachment/viewport/sampling, stale or unassociated compositor records, projection/conversion changes, ambiguous edge/occlusion/shifted labels, incomplete batches and nonfinite depth. GPU-state perturbations are record/array-level tests, not additional native raster campaigns. Saved-trace replay checks real acquisition association, uniform consistency and program-blob/mask hashes. CLI schema tests cover missing captures, null/list/event/nested malformed JSON and require a BLOCKED result plus zero mask. CPU CTest rejects integer/double/scalar/array/short/overflow uniform layouts before reading float storage. Independent review identified the storage/schema issues; both were corrected and re-reviewed without further defects.

The sole graphics fixture preceded the final storage guards. The final guard build and CPU tests pass; it was **not rerun graphically**. Its hash is explicitly separate. The exact pre-guard fixture ELF hash was not retained before rebuilding and is **UNKNOWN**; no final-build hash is substituted. Actual loaded library paths and their file hashes, driver identity, saved GL program blobs and raw image/trace hashes are retained in provenance. This limitation independently prevents upgrading fixture binary qualification.

```bash
cmake -S scripts/stage_a_rgbd -B /tmp/stage_a2_compositor_build -DSTAGE_A_RENDER_PROBE_ONLY=ON
cmake --build /tmp/stage_a2_compositor_build --target stage_a_ogre2_image_probe stage_a_compositor_uniform_test -j1
ctest --test-dir /tmp/stage_a2_compositor_build --output-on-failure
python3 -m pytest -q tests/test_stage_a_ogre2_compositor.py tests/test_stage_a_ogre2_registration.py
# A future deliberate reproduction is bounded; do not infer authority from exit 0.
sha256sum /tmp/stage_a2_compositor_build/stage_a_ogre2_image_probe
LIBGL_ALWAYS_SOFTWARE=1 timeout 90s /tmp/stage_a2_compositor_build/stage_a_ogre2_image_probe ogre2 /tmp/new_compositor.json --images
python3 scripts/stage_a_ogre2_registration.py /tmp/new_compositor.json /tmp/new_registration.json
# Diagnostic exits 2, writes BLOCKED plus .mask.uint8, all zeros.
```

The compressed JSON retains original native paths/handles. `provenance.json` maps those captured program paths to compressed local artifacts and their decompressed SHA-256 values. All image bytes and the explicit empty mask are retained losslessly. OpenGL is a dependency only of the opt-in Ogre 2 probe; production builds are unchanged.

Primary API/source references: [Ogre 2.2.5 quad execution ordering](https://raw.githubusercontent.com/OGRECave/ogre-next/v2.2.5/OgreMain/src/Compositor/Pass/PassQuad/OgreCompositorPassQuad.cpp), [Ogre 2.2.5 scene execution](https://raw.githubusercontent.com/OGRECave/ogre-next/v2.2.5/OgreMain/src/Compositor/Pass/PassScene/OgreCompositorPassScene.cpp), [Khronos GL state-query specification](https://raw.githubusercontent.com/KhronosGroup/OpenGL-Refpages/main/gl4/glGet.xml). Source inspection is not an execution-time arithmetic certificate.

**Stop boundary:** post-pass public introspection does not establish per-draw bindings or a conservative numerical/raster enclosure. No further fixture or Gazebo/EPD run was attempted. DART/render alignment, physical collision-to-visual identity, genuine EPD association and current-capture penetration remain separately unqualified; fixture labels confer no collision identity.

**Exact next engineering action:** add a read-only draw-submission witness for actual vertex/matrix inputs and complete texture bindings on the pinned path, before attempting an operation/coverage enclosure or another fixture. PR #3176 remains draft; rollback is this optional probe/diagnostic change only.
