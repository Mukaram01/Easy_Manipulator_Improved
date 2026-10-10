# Same-fragment RGB / integer-ID fixture — runtime BLOCKED

Baseline `ab4be990c674b2259d82aa9896e9236a13970373`. **Supported API components compile; same-fragment output, occlusion and lossless runtime readback remain UNVERIFIED.** One bounded native fixture attempt failed before rendering. No Gazebo/EPD capture, MoveIt, controller or execution goals; no physics-owner, installed-library or protected Stage-A1 changes. No contact authority, envelope change or penetration-limit change.

## Implementation

Opt-in target `stage_a_fragment_mrt`, built only under existing `STAGE_A_RENDER_PROBE_ONLY`. Uses public Ogre2 SceneManager, v1 prefab cube/material, high-level GLSL program, TextureGpu and compositor RTV/workspace APIs. A single scene pass targets RGBA8_UNORM RGB and R32_UINT IDs with one D32_FLOAT depth attachment. One fragment shader writes both outputs; no second camera, segmentation pass, nominal projection or majority vote. Opaque depth-tested/depth-writing materials, no blending, no MSAA. The intended RGB is simple face-shaded red/green geometry rendering, not an ID-coloured segmentation replacement. This fixture does not claim equivalence to stock Gazebo PBS/RGB imagery or inference quality.

Two renderer-local integer IDs (16777217 and 4000000022) deliberately exceed exact binary32 integer range. They travel through uint uniforms and an integer output/readback; background 0, invalid 4294967295. They are **not Gazebo entities or collision IDs**. Native Entity inventory is enumerated and checked against the two allocated entities. An attachment callback verifies actual texture formats/dimensions, not per-draw GPU bindings. This is limited MRT attachment validation; the stopped draw-inspection route is not resumed. Source and loaded-library provenance are saved.

Ogre2.2 GL3Plus colour clear uses float clear operations. The implementation avoids them for the integer output: initialise the owned attachments with public typed `glClearTexImage`, then load both in the scene pass and clear only depth. Readback uses native R32_UINT and RGBA8, packs RGB without image transformation, and saves sibling buffers. The intended center pixel is covered by the near cube and also lies in the farther cube projection; both IDs must remain visible. These checks have **not reached execution**.

`stage_a_fragment_identity.py` supplies CPU consistency and strict mask diagnostics: reject unsupported formats/material contract, duplicate IDs/native entities, stale session/frame, missing/unknown visible IDs and changed sibling byte hashes. A nonempty boolean mask must contain exactly one non-background/non-invalid ID at unchanged RGB coordinates. It returns only a renderer-local integer, no planning pose or collision permission. Capture flags/hashes are input consistency checks, **not independent evidence that a GPU contract or EPD preprocessing is qualified**. The routine is not wired to EPD/native contact admission. It generates no masks.

## Sole run and corrected build

The executable SHA256 was saved **before** the one run: `37bc1a9cde0682a23816b1d8058aa5601bca200cde76889eb559ebcdeb8ac931`.

Command actually attempted:

```bash
LIBGL_ALWAYS_SOFTWARE=1 timeout 60s /tmp/stage_a2_fragment_build/stage_a_fragment_mrt /tmp/stage_a2_fragment_fixture.json
```

The run threw `Object already attached to a SceneNode or a Bone` before shader/material construction and before any RGB/ID image. Ogre2's `SceneManager::createCamera` already attaches the camera to the dynamic root, as documented in installed OgreCamera.h and [pinned SceneManager source](https://github.com/OGRECave/ogre-next/blob/0e0c47ed70091e7bdead5fb1ca01e1cae5857ef4/OgreMain/src/OgreSceneManager.cpp#L293). The probe incorrectly attached it again. The final source preserves and checks the existing root attachment. That correction **builds**, but was **not run** to obey the one-fixture maximum. The original failed-run ELF was copied before rebuilding; its pre-execution hash and the corrected ELF hash are separate. Failed-run source bytes are retained compressed. No corrected-build hash is substituted for the failed capture.

`failed_capture.json.gz`, `driver.log`, build logs and `provenance.json` preserve this failure and loaded libraries. Loaded swrast proves the software driver library was present; no successful image or llvmpipe rendering result is claimed from this attempt. **Zero qualified pixels; no RGB/ID images produced.** Previous camera/projection bounds do not qualify this new custom path.

## Validation and downstream gates

- Initial and corrected opt-in target builds: PASS (existing Ogre debug-configuration pragma note).
- `python3 -m pytest -q tests/test_stage_a_fragment_identity.py`: **23 PASS**. These are CPU record/array tests, not graphics/inference qualification. They cover two large integer IDs, mixed/empty/background masks, duplicate/missing IDs, duplicate native entity, incomplete inventory, stale session/frame, unsupported material/MSAA/depth/format, changed RGB/ID bytes and wrong dtype/shape.
- Native occlusion/shared-visibility/attachment/lossless-readback checks: **NOT REACHED**. No invented PASS or second fixture.
- Authoritative Gazebo visual → parent link/model → collision → owner ShapeNode mapping: **BLOCKED**, not implemented. Existing owner has collision-to-ShapeNode evidence, but this standalone fixture has no Gazebo visual inventory or current render/physics-step correspondence. No fixture label/name/order is substituted for that missing chain. Invalid mapping rejection cannot be presented as tested runtime behaviour of an absent mapping implementation.
- Genuine EPD association: **NOT RUN**, because the fixture and mapping prerequisites did not pass. Exact EPD preprocessing/image provenance is still required before use of the CPU mask diagnostic.
- Current-frame DART penetration, renderer/physics timing and continuous extraction: remain separate BLOCKED gates. Historical bounds are not imported. Original EPD poses/envelopes and the 0.1 mm threshold remain unchanged.

**One next action:** run the corrected, pre-hashed native MRT fixture once in the next explicitly bounded validation task; require actual RGB/uint-ID/occlusion evidence before implementing the Gazebo mapping or invoking EPD. No further GPU draw instrumentation is proposed. PR #3176 stays draft. Rollback removes this opt-in target, diagnostics and evidence only.
