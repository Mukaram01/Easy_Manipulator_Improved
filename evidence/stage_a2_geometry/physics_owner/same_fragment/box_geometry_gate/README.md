# Stage-A2 decisive segmentation recovery gate

**MODEL_IMPROVEMENT_REQUIRED_FOR_TESTED_SEARCH.** Baseline `6106fe83`;
PR #3176 stays draft and unmerged. CONFIRMED for this retained two-object test
case and this grid only. No global mathematical impossibility claim.

All 1,225 placements per confident detection were evaluated independently
against the independently rendered native 512×512 uint32 identity raster.
No tested placement has both zero background spill and zero missing visible
object pixels. This is an oracle diagnostic selected using offline truth;
none of these corrections is suitable for production integration.

## Exact retained baseline and provenance

All original evidence is preserved. The existing offline head-analysis validator
verifies every artifact in the observation manifest and the linked historical
retained, instrumented, static-graph and native-reference bundles before search.
Original boxes, labels, scores and exported masks compare byte for byte; the
mathematical reconstruction preserves every original binary foreground decision,
including the third low-confidence detection. Native truth is not the repeated
256 raster. Original strict accepted associations reproduce **zero**.

| Native ID | Raw background | Original EPD cropped background | Missing object, raw/cropped | Visible pixels |
|---|---:|---:|---:|---:|
| 65517 | 456 | 303 | 75 / 75 | 8381 |
| 65523 | 635 | 564 | 0 / 0 | 8537 |

Pinned SHA256 (uncompressed artifact bytes):

- Observation manifest: `d57a634d504f1de5114fdd4f43452d88fb45de5c298dea0bf622f8126f5eed40`
- Native reference manifest: `927cabf64d305c73bbfd48226c009cebb3454ce62d0ef20b3ef69dc197d1a264`
- Genuine 28×28 tensor: `eea5eb79fc9818a06fa56e3bfd33d7bb03d043628083a721a2f0b5c61e039ab1`
- Floating detection boxes: `533bfd3f60309ecd42b349a56269991316dcc7a782d91f79c690fec454783ca1`.
- Original exported masks: `3ffefec451c25c884b4a83e3e5075f027bb0a8947baa1adf89ed854567ef937e`
- Native 512 identity raster: `8365c3fc5df29278aa63b40a7a6ad0beb2c307235d30525537e4c7ed14603ee1`

## Search definition and metrics

For each original float box, scale its half-width and half-height independently
about its original centre, then translate that centre: dx/dy ∈
{-6,-4,-2,0,2,4,6}; sx/sy ∈ {0.90,0.95,1.00,1.05,1.10}. Centre/extent
arithmetic is float64, rounded once to float32 box inputs. The original box is
an explicit byte-exact control (candidate 612). Each candidate uses the unchanged
genuine 28×28 probabilities, one zero border, original graph float32 expansion
30/28, truncation to integer coordinates, inclusive +1 resize sizes, half-pixel
bilinear interpolation and strict probability >0.5. This reuses the previously
validated mathematical reconstruction, not a new ORT evaluation or a bit-exact
kernel claim. Both raw pasted masks and the existing candidate-specific EPD ROI path are
evaluated. ROI endpoints use float-box truncation/clipping and exclusive slicing;
this exactly reproduces original cropped controls. No extra cropping rule is introduced.

Background = foreground on ID0; missing = visible target pixels absent from
foreground; IoU = intersection/union with target. Cross-object contamination is
measured separately and is zero for every candidate. Boundary distances are
Euclidean pixel-centre distances between 8-connected boundary samples, pooled
in both directions (mean and maximum); spill distances are measured to the
nearest target pixel. Erosion only identifies boundary samples; it never edits a candidate. Distances
canonicalize the approximate OpenCV float32 squared distance to its nearest
integer (pixel-centre squared Euclidean distances are integers), require residual
<0.25, then use float64 square roots and means. This removes observed sub-micro-
pixel kernel roundoff and makes full report regeneration exact. Pixels are not millimetres.

Pareto objectives minimize spill and missing pixels simultaneously; every tied
geometry is retained. [pareto.csv](pareto.csv) lists all **4 / 9** raw Pareto candidates,
plus the separate cropped frontier (**8 / 9** candidates), including exact float32 xyxy coordinates,
parameters, expanded integer geometry,
IoU and boundary distances. Decimal representations round-trip exactly to the
float32 boxes. [results.json.gz](results.json.gz) contains **all 2,450 candidates**,
full paste geometry, raw/cropped metrics, original controls and all raw best-IoU residual `[y,x]`
coordinates. [sha256.json](sha256.json) pins uncompressed results and CSV.

## Quantitative decision

Best IoU means maximum raw target IoU, with lowest grid index breaking ties.

| ID | Baseline spill / missing | Baseline IoU | Best index | Best spill / missing | Best IoU | Best boundary mean / max px |
|---|---:|---:|---:|---:|---:|---:|
| 65517 | 456 / 75 | 0.939911735 | 436 | 103 / 198 | 0.964521452 | 0.834272572 / 3.162277660 |
| 65523 | 635 / 0 | 0.930767553 | 606 | 11 / 307 | 0.962798315 | 0.874178019 / 2.828427125 |

Box geometry improves IoU by **0.024609717 / 0.032030762** and reduces raw spill
by **353 (77.41%) / 624 (98.27%)** at the best-IoU placements, but increases
missing pixels by **123 / 307**. Compared with the original cropped masks,
spill is lower by 200 / 553, but original cropped IoU .956472 / .938029 still
has the same coverage advantage. This is a tradeoff, not strict qualification.

| ID | Least spill with full visible coverage | Least missing with zero spill | Coverage lost at zero spill |
|---|---:|---:|---:|
| 65517 | 806 | 520 | 6.20% |
| 65523 | 635 | 774 | 9.07% |

For ID65517, full coverage first appears on Pareto candidate438
(dx=-2,dy=0,sx=1,sy=1.05); for ID65523 the original control612 is the
full-coverage Pareto point. All exact coordinates are in the CSV.
No candidate meets the requested simultaneous zero-spill/full-coverage gate.
Zero-spill subsets are not promoted to qualified masks or passed to production
`mask_identity()`. The existing strict original replay still accepts zero.

At best-IoU correction, ID65517 retains 103 spill pixels in xy bounds
[370,215]–[374,297] near the left edge, while 198 missed pixels remain across
[375,212]–[467,299]. ID65523 retains an 11-pixel spill run x=42,y=215…225;
307 missed pixels remain across [43,212]–[141,299]. These bounds summarize exact
residual coordinate arrays, not filled rectangles. There is no cross-object
contamination after correction. Boundary mismatch remains, even where total
spill is small.

The heads have **781/784 (99.62%) / 725/784 (92.47%)** samples above threshold.
Their almost filled support admits only limited shape adjustment through
axis-aligned box geometry. In this tested grid, shrinking removes spill at the
cost of visible-object support; expansion restores support with spill. Thus the
observed heads plus tested box geometry are insufficient. Occupancy alone is
not a mathematical proof of failure for every possible continuous placement,
nor an exclusive attribution of all errors to head versus box prediction.

The cropped path independently fails the same simultaneous gate. At full visible
coverage its minimum spill is **752 / 564**; at zero spill its minimum missing
counts are **599 / 843**. Its own Pareto rows and best-IoU indices are retained
separately, so raw tradeoffs are not inferred to describe cropped behaviour.

| ID | Best cropped index | Cropped spill / missing | Cropped IoU | Boundary mean / max px |
|---|---:|---:|---:|---:|
| 65517 | 612 (original control) | 303 / 75 | 0.956471672 | 1.032836168 / 2.828427125 |
| 65523 | 607 | 190 / 77 | 0.969405294 | 0.715738761 / 2.000000000 |

The existing cropped path gains no IoU for ID65517; for ID65523 it gains
0.031375914 while trading 374 spill pixels for 77 missing pixels. Thus neither
raw nor cropped geometry results support integration.

## Concrete next milestone: supervised segmentation-model-v2

Prepare a versioned **EPD-owned** supervised dataset and training/evaluation
recipe, then run a separately authorized v2 experiment. No retraining or new
image acquisition is performed by this gate.

1. Collect actual RGB camera-render outputs with synchronized native instance-ID
   buffers and provenance: camera/intrinsics, scene seed, asset geometry,
   material/light/background and occlusion state. Use genuine RGB images as
   model inputs; do not paint masks into images or fabricate tensor fixtures.
   Renderer IDs provide visible-instance labels offline only. Keep the current
   two-object failure as a named regression, outside training.
2. Freeze at least 300 training, 100 validation and 100 held-out test scenes
   before fitting. Split by complete scene/layout and acquisition seed, including
   unseen object poses, projected sizes, materials, lighting and backgrounds;
   never split adjacent frames from one scene across partitions. Include
   separated, touching and partially occluded cubes and negative/background
   images. Preserve train/validation/test manifests and label/RGB hashes.
3. Fine-tune the segmentation model with visible-instance supervision and
   boundary-sensitive validation; save model/checkpoint, configuration, seeds,
   optimizer history and exported ONNX hash. Model inputs and runtime code
   receive RGB only. Keep confidence ≥0.8 and mask >0.5 fixed; no renderer IDs,
   truth-fitted box offsets, erosion, trimming or post-hoc threshold search in
   runtime inference. Use validation only for model selection, test once after
   freezing the export.
4. Gate v2 on both retained targets having **zero native background spill,
   zero cross-object pixels and zero missing visible pixels** using untouched
   exported/cropped masks and unchanged strict association. On the independent
   held-out set require ≥95% recall of visible labelled instances at confidence
   ≥0.8, ≥95% with IoU≥0.98, symmetric boundary p95≤1 px, **zero accepted
   associations containing background or another object**, and ≥90% strict
   association yield. Publish counts/denominators by occlusion/size strata and
   every rejection; no success from rejecting all detections. These are proposed
   measurable v2 exit criteria, not claims achieved here.
5. Only after this segmentation gate passes, advance to aligned depth/calibration
   and conservative grasp qualification on independent scenes. Preserve the
   original collision envelopes and **0.1 mm** physical qualification requirement;
   pixel accuracy alone grants no physical/contact/motion authority.

## Verification, safety and rollback

Offline reproduction (output directory must not exist):

```sh
python3 scripts/stage_a_box_gate.py --output /tmp/stage_a2_box_gate_new
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -c /dev/null --rootdir=. -q tests/test_stage_a_box_gate.py tests/test_stage_a_onnx_head_observe.py tests/test_stage_a_onnx_mask_graph.py tests/test_stage_a_epd_spill.py tests/test_stage_a_epd_retained.py tests/test_stage_a_epd_instrument.py tests/test_stage_a_paired_reference.py
```

The initial scoped run had 139 passes and one exact-regeneration failure caused
by OpenCV distance roundoff below 5×10⁻⁷ px. Counts, geometry, IoU and Pareto
sets were unchanged. The integer-squared-distance canonicalization above fixes
the diagnostic reproducibility issue; candidate-specific cropped evaluation was
also added after independent review.

**Final verification: 141 scoped tests pass, plus the final CSV hash/frontier/coordinate test passes (142 total distinct tests); no skips.** Full numerical report regeneration passes exact equality. See [verification.txt](verification.txt) and [provenance.json](provenance.json).

The focused tests check grid/control/centre scaling, real displaced-square
metrics, Pareto ties, threshold and zero padding, frozen baseline/hash rejection,
unchanged genuine heads and exact regeneration of every candidate result.
Original safety/acceptance code is untouched. No Gazebo capture, ONNX inference,
controllers, MoveIt, motion, production mask changes, retraining, threshold
relaxation or collision/physical qualification changes. Build/launch/task tests
are deliberately not run because this change is offline diagnostics only.
Rollback: revert this additive diagnostic/tests/report commit; preserve all
prior evidence. **Keep draft. Do not merge. Stop after this gate.**
