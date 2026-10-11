# Genuine EPD inference on retained Gazebo RGB — associations BLOCKED

Baseline `2dcc1f8fe75b9cbeb5dd7c34255d068b50bcf2d0`; clean isolated worktree.
No Gazebo run, EPD GUI change, model modification or repeated inference.

## Inputs and external authority

The complete `../material_binding/attempt/raw_sha256.json` manifest was verified,
including original buffers, checked/native capture consistency, one complete
owner record, process result, original executable and owner loaded-library hashes.
The existing strict live visual/collision/ShapeNode validator passed. Original
256x256 RGB8 and little-endian uint32 ID bytes were used without replacement.

RGB SHA256: `920560881e06ae7285a33fd79d93e906a4001aac33f49583cf4af07016d1f526`.
ID SHA256: `cba6227c654aa690e9064057b51dd932162d604b5c5e0b112aa81b8c2ce60020`.
Session `b973ddbbca2c41639d5350db47bdac74`, frame 1; complete scene/world/step
metadata retained in preflight. These identities do not qualify physics timing.

Actual model: `/home/user/stage_a_cube_model/cube_maskrcnn.onnx`.
SHA256 `21362d62d816bd684f2b5c7769d0bcd1cd568af86419788fca9d095c46f2bad2`.
Labels: `/home/user/stage_a_cube_model/labels.txt`, `__background__`, `cube`.
SHA256 `adeeb37af7c067d456ea0fb2978c9bc4a242bea4d1fbb5a7b53072d486d7e113`.
Both match existing Stage-A evidence. ONNX metadata inspection (not inference)
confirmed input `images`, float32 `[1,3,512,512]` and boxes/labels/scores/masks outputs.

The new executable directly calls the external EPD P3OrtBase implementation.
It receives only model, labels, RGB path and output directory. No simulator IDs,
collision metadata, classes from the simulator, depth or planning poses enter it.
External EPD source was read-only, compiled without GPU, and hash-pinned alongside
the caller. `EPD_EXECUTION_BACKEND=cpu`, gpuIdx none, intra/inter threads 2/1,
sequential ONNX Runtime. No perception algorithm was duplicated in Workcell Builder.

## Exact preprocessing and inverse mask mapping

Retained RGB is resized 256→512 with OpenCV INTER_LINEAR, then converted RGB→BGR
for `P3OrtBase::infer`. EPD performs its configured 512→512 linear resize, BGR→RGB,
float32 division by 255 and CHW packing; no padding or letterbox is added. Model
input is NCHW with one batch. The full-image-mask option requires passing 512x512
to EPD, rather than passing 256 and expecting a 512 mask to fit it.

The exact prepared BGR bytes were retained and verified against an independently
computed hash. Python OpenCV 4.11.0 and native OpenCV 4.5.4 produced identical
bytes for this specific input; no universal cross-version equivalence is claimed.

EPD returns integer clipped bboxes and float32 ROI masks in the 512 coordinate
system. Original returned class, confidence, bbox and each ROI float mask are
retained unaltered. The mask threshold remains strictly >0.5. Coordinates obey
`u512=2*u256+0.5` for original pixel centres. Each foreground 512 sample maps to
its containing original pixel cell; the union of the four 2x2 samples is retained.
This conservatively preserves boundary support. It does not erode masks, select
interiors, trim using simulator IDs, use bboxes as masks or assert subpixel accuracy.
The original-scale bool masks are retained separately. Strict `mask_identity`
requires every selected pixel to have exactly one non-background renderer ID.

## Single measured inference

One bounded process (`timeout 120s`), one EPD infer call, normal exit 0.
ONNX Runtime 1.16.3; EPD infer duration (including preprocessing/postprocessing)
**3.429853999 seconds**. Loaded-library hashes and exact executable/source hashes
are retained. Executed ELF SHA256:
`3bc8febf89c18133dfbac21e92b0fcb90ffd7d010f824eedd34e9fab18d49cc3`.

Six genuine EPD outputs, all class 1 (`cube`):

| Detection | Confidence | EPD bbox (512 coordinates) | Association |
| --- | --- | --- | --- |
| 0 | 0.9839037657 | [210,219,311,301] | REJECTED: mixed/background IDs |
| 1 | 0.1852273643 | [286,281,315,307] | REJECTED: below unchanged 0.8 threshold |
| 2 | 0.1672583073 | [295,226,310,235] | REJECTED: below threshold |
| 3 | 0.1194306016 | [295,234,313,282] | REJECTED: below threshold |
| 4 | 0.1009240225 | [209,216,316,252] | REJECTED: below threshold |
| 5 | 0.0713838264 | [223,214,279,218] | REJECTED: below threshold |

Accepted associations: **0**. The high-confidence mask covers 2,024 original pixels:
1,724 with ID 65523, 210 with ID 65517 and 90 background pixels. Thus it merges
both workpieces and background, and cannot identify exactly one collision.
No majority vote, identity-based trimming, class substitution or threshold change.

Diagnostic candidate mappings (not accepted associations):
visual 6 → link 5 → model 4 → collision 7 → physics shape 6 → ShapeNode
`0x59205dd212e0`, renderer ID 65523; visual 10 → link 9 → model 8 → collision 11 →
physics shape 7 → ShapeNode `0x59205dd2a850`, renderer ID 65517.
`mask_support.json` records per-mask ID counts and these candidate identities.
Pointers are session-scoped identity witnesses, not persistent names.

## Validation and evidence

Focused build passed; 81 Python tests passed across the retained adapter, live
fragment, fragment and visual/collision contracts. New cases cover exact input
hashes/provenance, channel order, resize/padding/scale contract, ROI geometry and
pixel-cell inversion, nonfinite masks, mixed/background/unknown IDs, duplicate,
missing/stale metadata, duplicate associations and zero detections. Tests use
synthetic masks only as regressions; actual evidence is the one external EPD run.
The initial missing-module regression and an initial wrong-sized test fixture were
corrected before preflight. A pre-existing build cache belonged to another checkout;
it was left untouched and a fresh build directory was used.

`inference.json.gz` contains raw EPD detections and runtime versions;
`result.json` contains all association decisions and loaded-library hashes;
`mask_*.f32.gz` are genuine little-endian ROI floats; corresponding bool8 files
are the independent original-grid mask mapping. `preflight.json` pins image/model/
labels/source/ELF hashes and full preprocessing. Raw input and prepared BGR bytes,
complete stdout/stderr (including ONNX initializer warnings), CPU logs and the
single execution claim are retained. `raw_sha256.json` hashes uncompressed files.
Deterministic gzip is used. Source and ELF hashes were verified unchanged after run.

## Exact blocker and next action

The only confident instance merges two visible cube identities and background;
no genuine mask qualifies for a unique collision association on this image.
Next action: prepare a separately authorised capture with larger, non-overlapping
projected workpieces, keeping this model, preprocessing and strict single-ID
contract unchanged. No new capture or parameter search occurred in this task.

Timing, physical penetration, contact and extraction authority remain BLOCKED.
No RGB-D localisation claim: aligned depth/calibration are absent. Zero robot,
controller, MoveIt or execution goals; no ACM changes, installed-library or protected
Stage-A1 edits. Original EPD envelopes and the 0.1 mm limit unchanged. PR stays draft.
