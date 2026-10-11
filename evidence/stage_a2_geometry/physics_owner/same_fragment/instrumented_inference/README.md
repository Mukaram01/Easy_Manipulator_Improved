# Single instrumented EPD inference: MODEL OUTPUT spill

**CONFIRMED at the exported ONNX graph boundary**, against the retained 256 ID
raster expanded into exact 2×2 cells. This reference is not independent 512
renderer ground truth. Strict accepted associations remain **0**.

Exactly one bounded CPU inference ran on 2026-10-11, 08:29:26–08:29:29 UTC,
exit **0**, no timeout. No inference retries, Gazebo, MoveIt or motion goals.
Baseline: `6640cea2`. Original evidence, external EPD checkout, weights, labels,
preprocessing, confidence 0.8, mask threshold 0.5, filtering, `mask_identity()`,
collision/contact restrictions and geometry remain unchanged. PR #3176 stays draft.

## Observation stage and safeguards

Temporary copies of hash-matched `ort_base.cpp` and `p3_ort_base.cpp` add only
const-read observers. Removing the marked observer blocks restores the original
sources exactly. Production sources/build trees were not edited.

1. `session_inputs_before_Run`: actual float32 `[1,3,512,512]` tensor.
2. `session_outputs_after_Run`: actual outputs immediately after
   `Ort::Session::Run`, before EPD pointer adaptation or P3 processing.
3. `detection_N.full.f32`: P3 full-mask memcpy result, before ROI extraction.
4. `detection_N.roi.f32`: cloned ROI immediately after original extraction;
   compared with the caller's separately saved `mask_N.f32`.

Floating boxes, scaled boxes, truncated pre-clamp boxes, exact clipped ROI
coordinates and raw→EPD index mappings are retained per detection. Raw tensor
bytes preserve actual types: boxes float32 `[3,4]`, labels int64 `[3]`, scores
float32 `[3]`, masks float32 `[3,1,512,512]`. Classes are `[1,1,1]`; raw/EPD
indices are `[0,1,2]`. The original P3 >0 score filter still returns all three;
the unchanged 0.8 association gate rejects the third.

“MODEL OUTPUT” means **exported graph output**, not an unprocessed mask head.
Static graph inspection shows `masks` from `/network/Unsqueeze_8`, fed by
`Loop_3207`; its body contains Resize/Slice/pasting operations. Those operations
are inside the pinned ONNX model. They were neither bypassed nor separately
instrumented. See retained `model_contract.json.gz` for exact producer metadata.

Before the genuine run, original and observed **actual P3 paths** ran against a
synthetic OrtBase that constructs no Session and opens no model. Input/output
bytes matched exactly. Fixtures cover fractional boxes, image-edge clamping,
zero-score filtering, threshold neighbours and strided ROIs. The tensor observer
also preserved signed zero, infinity, neighbouring float32 values and int64
4294967297, both in memory and in retained bytes. A first fixture assertion used
scalar-double scaling instead of OpenCV float rounding; its failure log is kept.
The corrected fixtures compare original/observed preprocessing bytes exactly.
These were synthetic preflight runs, **not additional model inferences**.

Source/input/model/labels and historical runtime hashes passed before execution.
The sealed preflight pins the instrumented ELF, source snapshots, synthetic
artifacts and loader closure. Symlink aliases were normalized; installed and
build JsonCpp copies have identical bytes. All **22 actually loaded library
hashes** match historical hashes, including dynamically loaded TBB malloc.
All 21 external src/include file hashes and the existing dirty Git status match
before/after. The previous preflight is retained to document Python-only guard
hardening after the C++/synthetic build; compiled source/header/ELF stayed unchanged.

## Exact attribution

| Detection / ID | Confidence | Raw ONNX object / BG | ROI object / BG | Foreground added / removed |
|---|---:|---:|---:|---:|
| 0 / 65517 | 0.9756200909614563 | 8313 / 449 | 8313 / 296 | 0 / 153 |
| 1 / 65523 | 0.8621252775192261 | 8663 / 509 | 8663 / 438 | 0 / 71 |
| 2 / 65517 | 0.05048725754022598 | 35 / 58 | 35 / 58 | 0 / 0 |

Raw→P3 full-mask copy changes **0 values and 0 foreground samples** for every
row. ROI extraction changes **0 values at identical retained coordinates**;
there is no mask resampling or shift. Its only foreground effect is removing
153/71/0 background samples outside the integer ROI. On zero re-embedding, all
probability-value changes are outside the ROI: 662/784/35 samples, respectively.
There are **0 postprocessing-added background samples**.

Actual floating boxes → truncated/clipped half-open ROIs:

- `[371.97686767578125,212.25132751464844,469.6667785644531,301.1519775390625]`
  → `[371,212,469,301]`
- `[38.25672149658203,212.29083251953125,146.5597686767578,301.1468505859375]`
  → `[38,212,146,301]`
- `[373.82806396484375,295.9470520019531,394.64251708984375,301.24237060546875]`
  → `[373,295,394,301]`

Clamping changes none of these boxes. Relative to a *diagnostic-only* floating
box crop using `(x+0.5,y+0.5)` centres, integer cropping adds 0 foreground samples
and removes 84/0/0. This counterfactual convention is not another processing
stage or a qualification of the continuous silhouette. The actual stage
removals above compare all raw foreground with the actual ROI directly.

Historical confidence and bbox differences: **0**. Historical ROI byte/value
and mapped-mask differences: **0** for all three. The ROI SHA256 values remain
`056d0ff2…af61fc2`, `cfd13ed6…afa442ab`, `f89ad4b1…b4d7411` (full hashes in
`analysis.json`). Original mapped counts remain 2079+123, 2166+160 and 10+19.
Conservative inverse-union filling remains the previously measured 196/202/18
native samples within already contaminated cells, adding zero background cells.
No mask is trimmed or promoted to acceptance.

## Retention and deterministic verification

`sha256.json` pins original uncompressed bytes for **92** deterministic gzip
artifacts (~350 KiB total). It includes all raw inputs/outputs, full/ROI masks,
process status/stdout/stderr, claims, build/synthetic logs, original/instrumented
source snapshots and pre/post library/checkout provenance. `retention_inventory`
maps historical absolute source paths to portable snapshots. Original binary
identity is pinned in preflight; the built ELF remains in the isolated `/tmp`
experiment directory. No binary or model is executed by the following replay:

```bash
python3 scripts/stage_a_epd_origin.py evidence/stage_a2_geometry/physics_owner/same_fragment/instrumented_inference > /tmp/epd_origin.json
python3 -m pytest -q tests/test_stage_a_epd_instrument.py tests/test_stage_a_epd_spill.py tests/test_stage_a_epd_retained.py tests/test_stage_a_fragment_identity.py tests/test_stage_a_live_fragment_identity.py tests/test_stage_a_fragment_camera.py tests/test_stage_a_gazebo_fragment_runner.py
```

**133 focused deterministic tests passed**, including byte-preservation,
stage-difference, classification, original-baseline and tamper-rejection cases.
The generated report matches `analysis.json`, including lossless raw background
coordinate runs. Manifest SHA256:
`11f9fe0899926a07f666b060df50c29eff6b89219a454bdcada03f707dfb19c6`.
Input RGB SHA256:
`92f1ba8ad58b2437fa5ccf55e73d53cc9dcba469b04efefbe624ce1505585793`.
Instrumented ELF SHA256:
`3a403529ff049f057240eec9ad29714780a60ce761deb25d8d467ef9248f6322`.

Remaining uncertainty: neural mask-head versus ONNX-internal paste/resize
contribution is unobserved; native 512 renderer ground truth is unavailable.
Neither uncertainty changes the observed exported-output-versus-external-EPD
classification. No timing, penetration, contact, grasp or extraction qualification.

**STOP:** the single experiment is complete; no further experiment was run.
Rollback is an additive-change revert; preserve all original evidence.
