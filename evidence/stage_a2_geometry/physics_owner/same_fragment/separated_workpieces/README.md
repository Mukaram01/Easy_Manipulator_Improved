# Separated workpieces: capture PASS; EPD association BLOCKED

Baseline e3b0c2a7843354ccce36a614a26bc6016c7a4744. Exactly one disposable Gazebo capture and one genuine CPU EPD invocation were executed. No retries, model changes, planning or execution goals.

## Camera and validation

Profile `separated_workpieces`: position (0.40, -0.357, 0.0125) m, look-at (0.40, -0.217, 0.0125), Z up, vertical FOV 60 degrees, aspect 1, near 0.01 m, far 2 m, 256×256. The authored eight-corner pinhole calculation predicts both original 25 mm cubes in frame with 114.4383192247485 pixels between projected bounds. This is a framing diagnostic, not a GPU error enclosure or physical clearance measurement. The measured raster independently qualifies the profile.

Original cube model XML and physics XML were compared with the prior material-binding disposable world and are identical. Only camera framing changes. The existing overlap profile retains its near/far BOX-ray occlusion requirement. The separated profile independently requires two verified Items/mappings, at least 1000 pixels each and at least 12 blank columns. Frozen replay checks profile identity and recomputes the witness from actual retained uint32 pixels. Geometry/material/inventory/shared-pass/depth checks remain enforced.

## Actual capture

Native/supervisor exit 0; `PASS_TESTED_LIVE_GAZEBO_IDENTITY_ONLY`.

| ID | Pixels | Inclusive image bounds |
|---|---:|---|
| 65523 | 2166 | [21,106,70,149] |
| 65517 | 2079 | [186,106,233,149] |
| 0 (background) | 61291 | — |

Measured horizontal gap: **115 blank columns**. Both original Ogre Items and authoritative visual/collision/ShapeNode joins passed. RGBA8/R32_UINT attachments, one shared depth-tested/writing scene pass and nonblank RGB passed existing native and retained-byte validation. No unexpected image IDs.

Executed capture ELF SHA256: `fd7b545d345f3738818394e3d6b6763b9943be1a24b3cd77fd67086c524e31dc`.
Owner ELF SHA256: `0f4a2dda8d7bfd67745b03493254a9d91ea66bb36dcb2d78440fbf5809ff072a`.
RGB SHA256: `92f1ba8ad58b2437fa5ccf55e73d53cc9dcba469b04efefbe624ce1505585793`.
ID SHA256: `75246c2b20a8a0bcbbef866b6ade995535de6fa8f7be24f46bcb1d8197681a19`.

`capture/capture.json.gz` records actual native camera floats, Items, mappings, attachments and acquisition. `capture/preflight.json.gz`, process maps and result record source, executable and loaded-library provenance. Full stdout/stderr and exit evidence are retained.

## Genuine EPD result

Unchanged external P3OrtBase CPU inference, RGB-only input; RGB→BGR adapter, INTER_LINEAR 256→512, no padding; external BGR→RGB CHW float32/255; confidence 0.8, mask threshold 0.5. Original-coordinate mask is conservative union of the corresponding 2×2 samples, without ID trimming. One invocation completed in **2.348214161 seconds**.

Model `cube_maskrcnn.onnx` SHA256: `21362d62d816bd684f2b5c7769d0bcd1cd568af86419788fca9d095c46f2bad2`.
Labels SHA256: `adeeb37af7c067d456ea0fb2978c9bc4a242bea4d1fbb5a7b53072d486d7e113`.
Unchanged EPD ELF SHA256: `3bc8febf89c18133dfbac21e92b0fcb90ffd7d010f824eedd34e9fab18d49cc3`.

| Genuine class | Confidence | Mask support / rejection |
|---|---:|---|
| cube | 0.9756200909614563 | 2079 pixels ID65517 + 123 background; rejected |
| cube | 0.8621252775192261 | 2166 pixels ID65523 + 160 background; rejected |
| cube | 0.05048725754022598 | Below unchanged 0.8 threshold; rejected |

**Accepted associations: 0.** Separation removed cross-workpiece merging for the two confident masks, but both contain background. Strict single-ID association therefore fails closed. Diagnostic candidate mappings in `epd/mask_support.json.gz` are not accepted associations. Original float ROI masks and inverse-mapped boolean masks are retained, along with original class/confidence/bboxes, input bytes, preprocessing and loaded libraries. No RGB-D localization claim.

## Validation and reproduction

Build PASS; 117 focused Python tests PASS; 4 focused native tests PASS. See `validation/`. `source_sha256.json` pins changed code. Each capture/EPD file is retained byte-for-byte in deterministic gzip; `sha256.json` hashes the original uncompressed bytes. To inspect/replay, decompress each `.gz` to the same basename in a fresh directory. Retained preflight paths identify the original execution, not a new run. Do not rerun to reproduce the measured outcome under this one-run task.

The existing runner prepared `/tmp/stage_a2_separated_view_e3b0c2a7_20261010` with `--camera-profile separated_workpieces`, original two-cube world `/tmp/workcell_stage_a2_separated/physical_world.sdf`, capture `/tmp/stage_a2_live_fragment_build/stage_a_gazebo_fragment_capture` and unchanged owner library, then executed `--run-prepared` once with its 60-second bound. The retained EPD runner consumed that capture once using the hash-pinned model, labels and binary above.

Timing, physical penetration, contact and extraction authority remain BLOCKED. No simulator poses exported to planning; collision envelopes and 0.1 mm limit unchanged.

**Next action:** add this retained separated capture as an external EPD mask-boundary regression case to diagnose background spill, preserving the strict no-background association contract. No further camera sweep or inference was performed.
