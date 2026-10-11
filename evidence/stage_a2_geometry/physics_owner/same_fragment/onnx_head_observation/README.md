# One genuine 28×28 mask observation

Baseline `918659a0`; PR #3176 stays **draft and unmerged**. Exactly **one** bounded CPU ONNX Runtime inference ran, **2026-10-11 10:23:29.500859–10:23:32.577054 UTC**, process exit0, no timeout. All later work was offline. No retry, Gazebo, MoveIt, controllers or motion; no model tuning or physical qualification.

**Comparability: CONFIRMED.** Every original exported tensor—boxes, labels, scores and512 masks—is **byte-identical** to the historical retained output. The new genuine tensor `/network/roi_heads/Slice_12_output_0` is float32 `[3,1,28,28]`, **9408 bytes**. This is post-sigmoid class-selected head output before zero padding, not a new detection or a replacement mask.

**Attribution: UNKNOWN for an exclusive learned-mask-error versus box-placement-error diagnosis.** We now observe the actual contributing head probabilities and interpolation effects. Those mechanisms overlap, and a native512 raster does not independently label continuous28-grid support or establish a correct learned box. The established exported-boundary classification remains **MODEL OUTPUT**. This observation does establish padding-related missing support under the fixed geometry.

## Quantitative findings

| Detection / native ID | Head samples >0.5 /784 | Raw background | ROI background | Missing object | High head samples contributing to background, unique |
|---|---:|---:|---:|---:|---:|
| 0 /65517 | **781** | **456** | **303** | **75** | 117 |
| 1 /65523 | **725** | **635** | **564** | **0** | 111 |
| 2 /65517, below confidence gate | 471 | 41 | 41 | 8329 | 94 |

The two confident heads span **.4293611646–.9999976754** and **.0356183350–.9999939203**. Every actual background foreground pixel receives positive weight from at least one interior probability >0.5. **130/62** background pixels have every positive-weight contributor >0.5; **326/573** mix high and low/zero contributors. `analysis.json.gz` retains each affected native `[y,x]`, observed output probability, inverse padded-source coordinate, all four head `[row,column]` indices (null for padding), probabilities and weights.

**55/29** high head samples have centres whose nearest native512 ID pixel is background. This samples the retained raster cells; it is not independent continuous mask-head ground truth. **69/71** raw background pixels have image centres `(x+.5,y+.5)` outside the original floating box; **387/564** lie inside it. Full float boxes, derived expanded/int boxes and mappings are retained. These facts constrain spatial support; they do not partition error into learned-mask and box causes.

### Interpolation and padding: explicit offline counterfactuals

These arrays are analysis-only and never enter `mask_identity()` or any acceptance path. The genuine outputs remain untouched.

| Actual bilinear minus nearest counterfactual, same geometry/padding | ID65517 | ID65523 |
|---|---:|---:|
| Foreground added / removed | 7 /38 | 167 /48 |
| Background foreground added / removed | **6 /35** | **166 /48** |
| Net background change | −29 | +118 |

The nearest counterfactual samples the **same half-pixel coordinates**, choosing the nearest input index with ties to the lower index; it is not the graph's actual interpolation or the FPN asymmetric convention. Thus6/166 actual background pixels depend on bilinear interpolation relative to this precisely defined alternative. Other background pixels already exceed0.5 under nearest sampling. These overlapping counts are not an additive learned-error budget, and the alternative is not claimed correct.

At the **same paste geometry**, replacing the padded zero border with edge replication would add1324/664 foreground pixels:75/0 object pixels and1249/664 background pixels. Actual zero padding therefore suppresses support and creates **zero** foreground samples relative to that comparison. The edge-replicated counterfactual fills **all75** missing object samples; no genuine mask is changed.

Of those75 missing ID65517 samples, **72**, at `x=371,y=220…291`, have the previously derived maximum probability **.49056603773584906**, even with every interior probability1. The actual observation confirms this padding/placement bound. The remaining3 are `[y,x]=[212,465],[212,466],[212,467]`; they combine head probabilities **.8600227237/.6420754194** from head row0, columns26/27 with zero padding, producing **.4984303713/.4618058205/.4251815677**. Their bound is .59375, so the actual probability mixture is needed to explain their missing status. All75 are padding-dependent at this fixed geometry; this does not establish whether a different box would be correct.

### Numerical reconstruction, not a bitwise claim

Offline reconstruction follows the pinned graph: one zero border, float32 box expansion30/28, int64 truncation, inclusive `+1` sizes, linear half-pixel Resize, exclusive clipping and zero placement. Indices and bilinear arithmetic use float64 mathematics, with final float32 rounding. We explicitly compare this formulation with observed ORT rather than assuming kernel equivalence.

| Detection | Float values differing from ORT | Maximum absolute error | Smallest distance of ORT probability from0.5 | Foreground decisions differing |
|---|---:|---:|---:|---:|
| 0 | 4131 | **8.9406967163e-7** | 7.1209669113e-4 | **0** |
| 1 | 4904 | **9.8347663879e-7** | 1.8864870071e-4 | **0** |
| 2 | 113 | 5.9604644775e-7 | 5.5714845657e-3 | **0** |

These are reconstruction differences, not changed genuine outputs. Every binary decision is reproduced, and measured maximum errors are below the smallest threshold margins. Contributor arithmetic is therefore adequate for the reported support comparisons; it is not a bit-exact ONNX Runtime implementation.

External EPD cropping still removes153/71 background samples and adds none. Original conservative256 mappings remain **2079 object +123 background**, **2166 +160**; confidences are unchanged .9756200909614563/.8621252775192261, third .05048725754022598. **Strict accepted associations remain zero.** Native512 IDs are the previously independently rendered, exactly historically comparable reference; their376-pixel disagreement with the repeated2×2 proxy remains unchanged. No mask trimming, erosion, threshold adjustment, allowance or synthetic promotion occurred.

## Readiness, execution and hashes

The diagnostic model adds **one graph-output ValueInfo**, preserving original inputs, output order, nodes, initializers, attributes and all other protobuf fields. Removing that appended output restores the **exact original serialized model**, including its SHA256. The added output is last, FLOAT `['N',1,28,28]`, and its sole producer is the original `/network/roi_heads/Slice_12`. Static ONNX checking passed. No computation node was inserted or modified. The isolated diagnostic is65 bytes larger; the production file and external EPD checkout were never edited.

The standalone const-read C++ observer uses exact retained preprocessed CHW input bytes, eliminating new preprocessing. Same hash-pinned ORT1.16.3, CPU backend, intra/inter threads2/1, sequential execution and `ORT_ENABLE_ALL`. All **10** loaded DSOs match historical library hashes. Preflight pins source/header/compiler/tool, model/input, ELF and dependency hashes; after-attestation confirms input/model/EPD identity. Runtime optimizer warnings about unused initializers are retained in stderr; they do not change the on-disk models, and original outputs still match exactly.

Before inference: **123 focused tests passed**; typed C++ synthetic extraction preserved signed zero, adjacent float32 values and int64 4294967297 without constructing a Session. Structural negatives reject modified nodes/initializers/output order/tensor/shape/dtype. The initial compile warning and subsequent system-JsonCpp mismatch were corrected **before** inference; both preflight failures are retained and neither consumed an inference. Exact historical JsonCpp was linked. Independent static review found no blocker. Exclusive wrapper and native Run-entry markers prevent reuse; process exit, one entered/completed Run, timestamps, loader maps, stdout/stderr and session metadata are retained.

SHA256:

- Original model: `21362d62d816bd684f2b5c7769d0bcd1cd568af86419788fca9d095c46f2bad2`
- Diagnostic model: `ffc876c18fec9d07d52b661f00d44ef3f339c81563f47db057ec79f336e9ae82`
- Retained preprocessed input: `4dbd106f22bf0f7c77305e9a426afbf0102a91effe62c4442ccb95e22586d2fe`
- Genuine28×28 tensor: `eea5eb79fc9818a06fa56e3bfd33d7bb03d043628083a721a2f0b5c61e039ab1`
- Original/observed512 masks: `3ffefec451c25c884b4a83e3e5075f027bb0a8947baa1adf89ed854567ef937e`
- Native512 IDs: `8365c3fc5df29278aa63b40a7a6ad0beb2c307235d30525537e4c7ed14603ee1`
- ORT shared library: `ce3752ba35018ee6d8127ff4cba955b68b9c8b8b0fed8798a8f2e5c4c5a35fa5`
- Executed observer ELF: `c73d6e79fd50ac4f5387d7bf3912d03e16b5f76e4017046c7515efc50172b7f4`
- [38-artifact manifest](sha256.json): `d57a634d504f1de5114fdd4f43452d88fb45de5c298dea0bf622f8126f5eed40`

The deterministic gzip bundle (~214KiB) retains uncompressed-artifact hashes, all five genuine outputs, exact input, executable, executed-source snapshots, transform descriptor and appended ValueInfo bytes, build/synthetic/pre-inference logs, process/loader provenance and offline report. Both full model files remain preserved in their original/isolated locations; the176MB diagnostic is reproducible from the pinned original plus the retained ValueInfo without duplicating weights in Git. Such reconstruction is static and performs no inference. Captured test/source snapshots are the exact pre-inference versions; final offline regressions are committed separately.

## Focused verification and stop

**133 focused tests pass, including22 new tests; no skips.** Post-observation independent offline review exactly reproduces the report. Regression coverage includes frozen genuine tensor/report replay, graph preservation, extraction/indexing, coordinate arithmetic, runtime/count/type/order/hash rejection, an altered original output returning **NON-COMPARABLE without attribution**, and unchanged fail-closed association.

Safe offline reproduction only:

```sh
python3 scripts/stage_a_onnx_head_analysis.py evidence/stage_a2_geometry/physics_owner/same_fragment/onnx_head_observation --manifest-sha d57a634d504f1de5114fdd4f43452d88fb45de5c298dea0bf622f8126f5eed40 --output /tmp/new_head_analysis.json
python3 -m pytest -q tests/test_stage_a_onnx_head_observe.py tests/test_stage_a_onnx_mask_graph.py tests/test_stage_a_epd_spill.py tests/test_stage_a_epd_retained.py tests/test_stage_a_epd_instrument.py tests/test_stage_a_paired_reference.py
```

Do not rerun the execute command: the single authorized observation is complete. All previous worktrees/evidence, original masks, `mask_identity()`, collision envelopes and contact tolerances are preserved. No physical timing, penetration, contact, grasp or extraction qualification. Rollback: revert only this additive instrumentation/analysis/test/evidence change. **STOP; no further experiment or tuning. Keep PR draft; do not merge.**
