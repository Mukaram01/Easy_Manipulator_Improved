# Independent native 512 reference — one capture, no inference

Baseline `17f47975`; PR #3176 remains draft. Exactly **one** bounded Gazebo/Ogre2 diagnostic ran. Native process and child exit codes were **0**. No EPD inference, MoveIt, controllers or motion goals ran. No physical/contact/grasp/extraction qualification follows.

## Acquisition and comparability

A separate opt-in target compiles the unchanged Gazebo owner with its capture call intercepted. Production/fixture sources and strict `mask_identity()` remain unchanged. Two independently allocated, depth-tested native MRT passes use the same camera and original Ogre Items, geometry, material/ID bindings and DART collision joins: **256 RGB/uint32 IDs, then 512 RGB/uint32 IDs**. The extra 512 RGB is retained too. These are **two rendering passes**, not one pass across resolutions; the 512 IDs are never resized from 256.

Both targets are allocated before the sole `PreRender`/scene-graph preparation. Both acquisitions occur inside the same blocking step-2 `PostUpdate`; no RenderUtil/ECM update, physics step, `PreRender`, scene-graph update or `PostRender` occurs between them. The sole `PostRender` follows both readbacks. Before/between/after inventories, owner joins, actual Item transforms/AABBs/material identities, scene time, camera/projection matrices, simulation stamp and update epoch agree exactly. Their canonical fingerprints are all:

`aafbd1a7af2254f4db826eeb4df7cb93df60de60765ccacc18e3faf946ba613f`

Camera optics and 25 mm original BOX geometry are unchanged. Pixel sample `(x,y)` uses normalized camera coordinates `((x+.5)/N,(y+.5)/N)`, principal point `(N-1)/2` in array-index coordinates, and focal length `N/(2*tan(fovy/2))`. Thus corresponding continuous indices satisfy `u512=2*u256+.5`. Actual GL attachment dimensions/formats/full viewport and native camera rays are retained.

The new 256 RGB and ID bytes exactly match historical SHA256, and normalized camera, complete geometry inventories, material evidence, collision/renderer/physics joins and DART transforms match exactly. Only process-lifetime pointers and session-qualified material names are excluded from cross-run comparison; their within-run witnesses remain retained. The shared stable-scene fingerprint is:

`1fb6fcc16ae7fac9fb249271be589f5086b683b06b599be6580540ddeedee5ec`

**Direct historical comparison is established.** A mismatch would block it.

## Quantitative result

All mask counts below use unchanged strict `float32 > 0.5`; inference confidence threshold remains 0.8. Coordinates/distances are image pixels, not physical distances.

| Detection / visible ID | Confidence | Raw foreground | Native raw background | Native ROI background | Missing object, raw/ROI | IoU, raw / ROI | Max spill distance, raw / ROI |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0 / 65517 | 0.9756201 | 8762 | **456** | **303** | 75 / 75 | .939912 / .956472 | 3 / 2.828427 |
| 1 / 65523 | 0.8621253 | 9172 | **635** | **564** | 0 / 0 | .930768 / .938029 | 4 / 4 |
| 2 / 65517, below gate | 0.0504873 | 93 | 41 | 41 | 8329 / 8329 | .006174 / .006174 | 2.236068 / 2.236068 |

Cross-object foreground is **zero** for every detection. ROI extraction removes **153 / 71 / 0 foreground samples**, all background against native IDs; it adds **zero** samples and shifts none. Earlier instrumented evidence establishes bitwise unchanged ROI values at identical raw-output coordinates. Mean symmetric 8-connected boundary error for detections 0/1 is **1.440460 / 1.662152 px raw**, **1.032836 / 1.479993 px ROI**. Full support, component, distance, IoU and boundary metrics, plus affected `[y,x]` coordinates, are in `diagnostic.json.gz`.

The native raster has **8381 / 8537** object pixels for IDs 65517/65523; the repeated-2×2 proxy has **8316 / 8664**. They disagree at **376** pixels: proxy background becomes native object at **122 / 35**, and proxy object becomes native background at **57 / 162**. Consequently proxy raw spill **449 / 509** becomes native raw spill **456 / 635**; proxy ROI spill **296 / 438** becomes native ROI spill **303 / 564**. Renderer-resolution disagreement does not eliminate genuine exported-model spill.

The original conservative 256 mappings reproduce **2079 object + 123 background** and **2166 object + 160 background**. The third detection remains below 0.8. **Strict accepted associations remain zero.** Native IDs are measurement references only; no mask trimming, erosion, background allowance or synthetic promotion occurred.

**Classification: MODEL OUTPUT, at the exported ONNX-output boundary.** Foreground already exists on independently rendered background before external EPD ROI cropping. Internal learned segmentation versus resize/paste operations embedded in the exported ONNX graph remain **UNKNOWN**. This does not establish physical surface accuracy or any contact/motion qualification.

## Honest postcapture validation correction

Preflight passed 154 focused Python tests and the native synthetic test, and pinned the original world, qualified owner, diagnostic ELF and 1598 source/runtime dependencies. All observed native DSOs match preflight hashes.

The native run succeeded, but the initial Python wrapper exited **1**: its float64 analytic ray-direction tolerance of `2e-6` rejected a maximum difference `3.65256695e-6`; ray origins differ by less than `3e-8 m`. **There was no capture retry.** An offline arithmetic-only probe inspected the installed Ogre implementation and replayed its float32 inverse projection-view arithmetic (near clip z=-1, midpoint z=0). All eight native origins/directions reproduce **bit for bit**. Corrected offline validation requires exact float32 replay and separate analytic pinhole-origin checks, rather than the inappropriate float64 direction comparison. Changed direction, pixel centre and projection negative cases all reject. No Scene, Camera or Server is constructed by this replay.

The initial executed source snapshots, executed ELF, original failure record, DSO disassembly, offline correction sources/ELF, and replay/negative results are preserved separately. This correction changes analysis only; it does not alter captured rasters, masks, inference mathematics or safety gates.

## Hashes and replay

SHA256 of **uncompressed** artifacts:

- Historical/new RGB256: `92f1ba8ad58b2437fa5ccf55e73d53cc9dcba469b04efefbe624ce1505585793`
- Historical/new IDs256: `75246c2b20a8a0bcbbef866b6ade995535de6fa8f7be24f46bcb1d8197681a19`
- Native IDs512: `8365c3fc5df29278aa63b40a7a6ad0beb2c307235d30525537e4c7ed14603ee1`
- Executed native ELF: `f045931fe74142889e2e391408a4b0246de7d63756f69564da1aadef7b1228e0`
- Qualified owner ELF: `0f4a2dda8d7bfd67745b03493254a9d91ea66bb36dcb2d78440fbf5809ff072a`
- Paired capture header: `f06c5b1ca46208ce4e36578ebc8b028f66e12eb56ca363626b1955b1ba25516d`
- Native pass header: `7f459f3bf3a67ec90007e91f118935d392ba6c256f08d659260d1f5051571571`
- Diagnostic JSON: `6b5c4eaaf9606d5d044f8c9fd6a927dc26b41c994afb839e8cc3c0429cfe3ae6`
- [Artifact manifest](sha256.json): `927cabf64d305c73bbfd48226c009cebb3454ce62d0ef20b3ef69dc197d1a264`

The manifest pins 63 deterministic gzip artifacts. `retention_inventory.json.gz` binds 30 executed-source/build/lifecycle snapshots to preflight hashes. Preflight, loader hashes/maps, raw rasters, native/checked reports, camera matrices/rays, acquisition timestamps/order, owner traces, original scene input identity and process status are retained. The existing historical and instrumented-inference manifests are independently verified before mask analysis.

Offline regression (never launches Gazebo or inference):

```sh
python3 -m pytest -q tests/test_stage_a_paired_reference.py
```

Full scoped verification: **162 Python tests**, **1 native synthetic CTest**, and **3 offline arithmetic negative cases** pass. Negative coverage includes resolution, pixel-centre/viewport/projection shifts, scene/geometry changes, missing/duplicate identities, stale acquisitions, unexpected updates, changed runtime environment, disabled depth, malformed attachments, altered artifact bytes and manifest hashes. Existing EPD spill/retained/association regressions remain green.

**Next single action:** inspect the exported ONNX mask resize/paste subgraph offline using the retained 456/635 spill coordinates. No further capture or inference is authorized by this experiment.
