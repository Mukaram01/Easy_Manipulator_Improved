# Offline ONNX mask resize/paste analysis

Baseline `cd1e372e`; PR #3176 stays draft. **No model evaluation, ONNX Runtime call, Gazebo run, planning, controller or motion goal occurred.** Only hash verification, ONNX protobuf/schema inspection, retained-byte analysis and synthetic NumPy arithmetic ran. All prior evidence, model bytes, genuine masks, thresholds, `mask_identity()` and collision/contact restrictions are unchanged.

**Result: internal origin of background spill remains UNKNOWN.** The graph confirms a learned 28×28 class-specific sigmoid mask, one-pixel zero padding to30×30, box expansion30/28, int64 truncation, inclusive endpoint sizes, linear half-pixel resizing and clipped placement into512×512. It does not expose the intermediate values needed to separate learned-head error from resize/paste alignment. The previous **MODEL OUTPUT at exported ONNX boundary** classification remains established against independently rendered native512 IDs.

[Operator-level diagrams, formulas and exact attributes](graph.md); `graph.json.gz` records named operators, constants, schemas and initializer dimensions/hashes. This is static analysis, not an observed execution trace.

## Retained measurements and coordinate inspection

All counts below use the unchanged genuine float32 masks and strict `>0.5`. The source capture, instrumented inference and paired native-reference manifests and their artifacts/provenance are verified **before** any coordinate interpretation.

| Detection / ID | Confidence | Raw foreground | Native raw background | Native ROI background | Missing object | Raw / ROI padding-touching background footprints |
|---|---:|---:|---:|---:|---:|---:|
| 0 / 65517 | .9756200909614563 | 8762 | **456** | **303** | **75** | 309 / 156 |
| 1 / 65523 | .8621252775192261 | 9172 | **635** | **564** | **0** | 224 / 153 |
| 2 / 65517, below0.8 | .05048725754022598 | 93 | 41 | 41 | 8329 | 0 / 0 |

Genuine raw background probabilities span **.5007120967–.9605980515** and **.5001886487–.9446411133** for detections0/1. All background pixels are within the derived paste windows; 147/411 raw background samples respectively have footprints wholly inside the unpadded head region. A footprint touching padding is a coordinate fact, not a claim that padding created foreground. The other background samples cannot be assigned to learned error solely from that fact either.

| Detection | Graph-derived expanded float box | Truncated box | Resize h×w | Exclusive paste window |
|---|---|---|---|---|
| 0 | [368.487976,209.076309,473.155701,304.326996] | [368,209,473,304] | 96×106 | [368,209,474,305] |
| 1 | [34.388760,209.117401,150.427734,304.320282] | [34,209,150,304] | 96×117 | [34,209,151,305] |

These intermediate boxes/dimensions are **DERIVED**, not runtime-observed. Exact float32 values and original boxes are retained in `coordinates.json.gz`. Every affected `[y,x]`, inverse padded-source `[sx,sy]`, probability extrema and coordinate-group count is included; coordinates outside a paste window have no invented source coordinate. The below-threshold third detection remains recorded separately and is never accepted.

All75 missing ID65517 pixels touch the zero-pad interpolation region. Given the graph-derived geometry, sigmoid probabilities ≤1, and zero border, the analytic maximum at a padded-source coordinate is:

```
U(sx,sy) = clip(min(sx,29-sx),0,1) * clip(min(sy,29-sy),0,1)
```

For **72 of75** missing pixels (`x=371`, `y=220…291`), U is **.49056603773584906**, below0.5 by .0094339623 regardless of the interior probabilities. Thus this conditional arithmetic establishes that padding/placement geometry prevents foreground at those72 samples; it does **not** establish the cause of background spill or recover the head values. The remaining3 missing pixels have U>0.5. All retained probabilities satisfy their bounds (a 2e-6 arithmetic comparison tolerance, never a segmentation allowance). The missing probabilities are .4251815677–.4984303713. All background samples have U>0.5, so the bound does not resolve their origin.

Prior independent native-reference measurements reproduce: zero cross-object contamination; raw→ROI cropping removes153/71 foreground samples and adds zero; native IDs differ from the repeated2×2 proxy at376 pixels. The proxy is not independent512 ground truth. Original256 mappings stay **2079 object +123 background** and **2166 +160**. Strict accepted associations remain **zero**, with the third detection below0.8. No physical accuracy, penetration, contact, grasp or extraction qualification follows.

## Reproduction and provenance

Model: `cube_maskrcnn.onnx`, **176310020 bytes**, SHA256
`21362d62d816bd684f2b5c7769d0bcd1cd568af86419788fca9d095c46f2bad2`.
It is read-only; the model is not copied into this evidence bundle. ONNX opset17 has2411 top-level nodes. The mask/coordinate ancestor slice retains745 plus nested graphs; the loop body has176 nodes. Neural weights are represented only by type/dimensions/hashes.

Original RGB SHA256: `92f1ba8ad58b2437fa5ccf55e73d53cc9dcba469b04efefbe624ce1505585793`.
Native512 IDs SHA256: `8365c3fc5df29278aa63b40a7a6ad0beb2c307235d30525537e4c7ed14603ee1`.
Instrumented-inference manifest SHA256: `11f9fe0899926a07f666b060df50c29eff6b89219a454bdcada03f707dfb19c6`.
Paired-reference manifest SHA256: `927cabf64d305c73bbfd48226c009cebb3454ce62d0ef20b3ef69dc197d1a264`.
The additional original source/artifact hashes, static-analysis source hashes, Python/parser identities and versions are in `provenance.json.gz`. `sha256.json` pins the uncompressed graph, coordinate and provenance artifacts; gzip uses mtime0. No production source was edited.

```sh
python3 scripts/stage_a_onnx_mask_graph.py --model /path/to/cube_maskrcnn.onnx --output /tmp/new_static_analysis_directory
python3 -m pytest -q tests/test_stage_a_onnx_mask_graph.py tests/test_stage_a_epd_spill.py tests/test_stage_a_epd_retained.py tests/test_stage_a_epd_instrument.py tests/test_stage_a_paired_reference.py
```

**Validation: 111 focused tests passed, including 26 new tests; no skips.** Scoped independent static review found no important correctness or safety issues.

The CLI refuses a changed model or previous evidence; it never creates an evaluator, session or simulator. It writes a new directory only. Uncompressed graph/coordinate hashes reproduce exactly under the recorded ONNX parser; provenance also binds the local source and parser identities. Arithmetic fixtures test linear half-pixel boundaries/singletons/clamping, nearest asymmetric floor, RoiAlign output-half-pixel alignment, image centre mapping, padding bounds, expansion/truncation/inclusive sizes, clipping and threshold boundaries. They are **TEST FIXTURES ONLY**, never genuine detections or acceptance evidence. Floating arithmetic helpers specify mathematical conventions and do not claim bit-exact ONNX Runtime kernels.

## Single recommended next experiment — not run

Separately authorize **one** bounded CPU observation of `/network/roi_heads/Slice_12_output_0`, the post-sigmoid selected-class tensor immediately before `/network/Pad`, on the exact retained input/model/runtime path. Expected shape for the historical N=3 is `[3,1,28,28]`: **2352 float32 values, 9408 bytes**. Preserve the original boxes, labels/index order, scores and exported masks; require exact historical outputs before attributing differences. This is the smallest missing internal probability tensor needed to check the downstream paste against observed values; no whole feature map or all-class logits are needed. Compare individual bilinear contributors at retained spill coordinates and require validated downstream arithmetic; unresolved kernel or spatial-support ambiguity must remain UNKNOWN. No future inference is performed or authorized by this offline analysis. Rollback: revert only this additive static-analysis/test/evidence change; preserve all preceding evidence. **STOP here; keep draft and unmerged.**
