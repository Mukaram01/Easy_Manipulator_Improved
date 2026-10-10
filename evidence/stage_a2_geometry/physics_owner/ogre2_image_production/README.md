# Ogre 2 image-production prerequisite

Baseline: `812487801f4955f22f89b0bae8900322aa842cd9`. **Image production PASS; registered capture/contact authority BLOCKED.** This is an actual native Ogre 2 rendering experiment with two synthetic static visual BOXes, not a Gazebo/EPD or physics capture.

## Controlled workstation result

The same probe ELF, fixture, camera settings and arguments produce:

| Graphics path | RGB | Finite depth pixels | Semantic IDs |
| --- | --- | --- | --- |
| Default VMware SVGA3D, Mesa 23.2.1, GL 4.3 | blank | 0 | none |
| `LIBGL_ALWAYS_SOFTWARE=1`, llvmpipe LLVM 15.0.7, GL 4.5 | nonblank | 284 | 11, 22 |

This identifies a working supported configuration change for this workstation and fixture; it does not prove a universal driver defect or a working Gazebo sensor pipeline. Installed libraries were unchanged. `provenance.json` records the shared executable hash, actual loaded rendering/Ogre libraries and hashes, and artifact hashes. The two driver logs and readbacks retain both observations.

All cameras use 512×512, requested HFOV π/3, aspect 1, near 0.02 m, far 5 m, antialiasing disabled, zero world pose. RGB is R8G8B8; native depth callback is FLOAT32/one channel; semantic callback is R8G8B8/three identical label channels, background 0. The two 25 mm visual BOXes are at (1, ±0.05, 0); labels are fixture identities, not verified physical collision IDs.

## Shared static rendering and calibration limitation

One explicit loop performs scene PreRender, all three camera Render calls, all three PostRender calls and scene PostRender, repeated three times. Depth and segmentation each deliver three callbacks. No external scene writer or simulation clock exists in this fixture. This establishes shared **static native fixture** state only; no applied Gazebo update or physics step is witnessed.

Generic depth/segmentation Camera matrix getters reconstruct ideal matrices and differ from the actual Ogre cameras. The probe therefore uses the supported public `Ogre2Node::Node()` API and its attached `Ogre::Camera` inventory to read actual native projection/view matrices. Each camera has exactly one attached camera. Readback occurs after the batches, not at exposure time.

Actual view matrices agree exactly. RGB and segmentation focal coefficients are 1.7320507764816284; depth is 1.7320505380630493. Corresponding pixel focal lengths are 443.4049987792969 and 443.4049377441406. The depth projection's clip coefficients also differ; the rendering 6.6.4 depth implementation expands near/far and performs shader depth conversion. A generic ideal projection cannot certify that pipeline.

For centered perspective and the common recorded view, over the union of camera frusta, `|x/z| <= 1/min(f)`. The exact rational expression `256*(max(f)-min(f))/min(f)`, rounded outward, bounds **recorded-matrix coordinate disagreement only** by **0.000035238669374079335 pixels**. It is not a total registration bound: acquisition-time native calibration provenance, rasterization and shader depth-linearization error remain unqualified. `registration.json` therefore reports `total_registration_bound_px: null`, `decision: BLOCKED`, and `contact_authority: false`.

`rgb.png` and `labels.png` preserve the native RGB and semantic values losslessly. `depth.f32.gz` preserves native float32 depth bytes (little-endian workstation). Native uncompressed sidecars are reproducible with the command below.

## Focused validation and reproduction

The two renderer executables are deliberately separate: the generic capability probe retains Ogre 1 compatibility; the OgreNext-linked image probe rejects other engines before loading. Loading Ogre 1 into an OgreNext-linked process is not a supported experiment.

```bash
cmake -S scripts/stage_a_rgbd -B /tmp/stage_a_render_probe -DSTAGE_A_RENDER_PROBE_ONLY=ON
cmake --build /tmp/stage_a_render_probe --target stage_a_render_identity_probe stage_a_ogre2_image_probe -j1
LIBGL_ALWAYS_SOFTWARE=1 timeout 90s /tmp/stage_a_render_probe/stage_a_ogre2_image_probe ogre2 /tmp/new_render_report.json --images
python3 -m pytest -q tests/test_stage_a_render_identity_probe.py tests/test_stage_a_ogre2_registration.py
```

Local results: four native checks and nine registration checks PASS (13 total, collected in focused runs). Includes real Ogre 1 unsupported segmentation, unknown engine, real software Ogre 2 image production, incompatible-engine rejection, blank RGB, infinite depth, absent labels, inconsistent batch count/resolution/format, different views and nonfinite calibration. The image probe exits 0 for image production while its authority decision remains BLOCKED. Both opt-in targets build. Independent code review found no blocking defects.

Physics mapping, stale physics frames, ambiguous EPD masks and penetration regressions for a new capture are **not reached** at this prerequisite. Historical six-state DART→ODE bounds are not reused. Zero genuine Gazebo/EPD captures in this change; no DART-to-render synchronization, new physical bound, native permissions, MoveIt or execution goals. Original envelopes, 0.1 mm limit, bridge restrictions, production worlds and protected Stage-A1 checkout remain unchanged.

**Exact next action:** add acquisition-time native camera/shader calibration witnesses and establish a conservative total RGB/depth/segmentation registration enclosure before the one bounded genuine Gazebo/EPD capture. This prerequisite, rather than missing image production, now blocks identity association.
