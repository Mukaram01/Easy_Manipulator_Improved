# Retained EPD background spill — offline regression

**CONFIRMED relative to the retained identity raster:** spill exists in the genuine
float32 EPD ROI before inverse mapping. Baseline `ad7a4655` reproduces exactly;
strict accepted associations remain **0**. This change adds diagnosis/tests only.
All original retained files and `mask_identity()` remain byte-for-byte unchanged.

Run from the repository root (no capture, inference, planning or motion):

```bash
python3 scripts/stage_a_epd_spill.py > /tmp/stage_a_epd_spill.json
python3 -m pytest -q tests/test_stage_a_epd_spill.py tests/test_stage_a_epd_retained.py tests/test_stage_a_fragment_identity.py tests/test_stage_a_live_fragment_identity.py tests/test_stage_a_fragment_camera.py tests/test_stage_a_gazebo_fragment_runner.py
```

Validation: **117 focused deterministic tests passed**, including 26 new cases.
The new baseline test was observed failing before the diagnostic existed.
The replay JSON matches [diagnostic.json](diagnostic.json), including lossless
background coordinate runs `[y, x_begin, x_end_exclusive]` at both resolutions,
exact squared pixel-centre distance histograms, component sizes and probabilities.
Rows/coordinates are zero-based; bbox upper bounds are exclusive.

## Provenance before analysis

Pinned SHA256 digests of both artifact manifests and the source manifest are
checked first. Every gzip is decompressed and its original-byte hash verified;
the entire capture then passes the existing provenance/identity validator.
EPD session/frame/acquisition/profile and RGB/ID bindings are checked, prepared
BGR bytes are recomputed, and mapped masks must equal retained boolean bytes.
All source hashes in the source manifest and repository sources in EPD preflight
are verified against the current checkout; the unchanged identity/contact
contracts are additionally pinned. Exact confidences, support and rejected
association results are assertions, not merely diagnostic output.

| Input | SHA256 |
|---|---|
| RGB bytes | `92f1ba8ad58b2437fa5ccf55e73d53cc9dcba469b04efefbe624ce1505585793` |
| uint32 ID bytes | `75246c2b20a8a0bcbbef866b6ade995535de6fa8f7be24f46bcb1d8197681a19` |
| ROI 0 float32 | `056d0ff27316dd11686b42e259894b5aead3df9325d3e5591434d4481af61fc2` |
| ROI 1 float32 | `cfd13ed6ab6ad70563e708fd92fee00bccb7458c314cfbf15eced739afa442ab` |
| ROI 2 float32 | `f89ad4b195807252b2f6b396cb5b8364eb6c2e8c87f984cd938c4a08fb4d7411` |
| `stage_a_epd_retained.py` | `fb23991be23f4b6c8d17d9d785edad75a336930fa1e8a2218c42864579e9b20f` |
| `stage_a_fragment_identity.py` | `3a4f4496fdc91775f9cebc49de10d6f818de4efcdd45d0e0a5103f96e4ba4127` |

The JSON includes **all** artifact, manifest and source hashes, plus recorded
external EPD hashes. During this diagnosis all three recorded external EPD source
files were also verified read-only against their preflight hashes. Portable
regressions do not require that external checkout. Loaded ELF/model/library
hashes are retained historical provenance, not a claim that those binaries ran
again or are available on every replay host. No weights, labels or external files
were changed.

## Pixel attribution

The native reference is the 256×256 uint32 ID image repeated into exact 2×2 cells.
It is **not** an independently rendered 512×512 silhouette.

| Detection / diagnostic ID | Confidence | 512 object / background | 256 object / background | Native fill: object / background |
|---|---:|---:|---:|---:|
| 0 / 65517 | 0.9756200909614563 | 8313 / 296 | 2079 / 123 | 3 / 196 |
| 1 / 65523 | 0.8621252775192261 | 8663 / 438 | 2166 / 160 | 1 / 202 |
| 2 / 65517 | 0.05048725754022598 | 35 / 58 | 10 / 19 | 5 / 18 |

Detection 2 is rejected below unchanged confidence 0.8. The first two retain all
2079/2166 visible object cells but also background; no genuine mask accepts.
For every detection, **new background cells from inverse mapping = 0**. Every
mapped background cell already contains a >0.5 native background sample.
Union fills remaining samples within those cells: `4 * mapped_bg - native_bg` =
196/202/18. These are additional covered *samples* on re-expansion, not newly
selected original pixels. No ID trimming, erosion or background allowance occurs.

Boundary means an 8-neighbour of the visible ID silhouette. Off-boundary pixels
can still belong to a background component that touches that silhouette.
All components/distances use pixel centres and 8-connectivity.

| Detection | 512 boundary / off-boundary BG | 256 boundary / off-boundary BG | BG component sizes at 512 → 256 |
|---|---:|---:|---|
| 0 | 227 / 69 | 114 / 9 | 67,87,142 → 19,104 |
| 1 | 266 / 172 | 135 / 25 | 54,41,316,27 → 20,129,11 |
| 2 | 22 / 36 | 11 / 8 | 58 → 19 |

Each complete foreground mask has one connected component at both resolutions.
Detached background components and cross-object contamination are **0 for all
three**. Maximum background distance is **3 native pixels**, **2 original pixels**.
Full exact distance histograms and affected coordinates are in the JSON.

## ROI and mapping semantics

The genuine ROIs are `[371,212,469,301]`, `[38,212,146,301]` and
`[373,295,394,301]`; float32 dimensions are 89×98, 89×108 and 6×21.
All bboxes are interior to the inference image, so none hits an image-edge clamp.
Foreground counts on ROI edges (top,bottom,left,right) are respectively
`[83,92,0,87]`, `[87,85,0,0]`, `[0,18,0,4]`. Probabilities outside the retained ROIs
and original floating bbox coordinates are unavailable; integer ROI cropping
loss cannot be quantified.

Above-threshold background probability ranges are **0.5007121–0.9520625**,
**0.5002435–0.8630255**, **0.5157212–0.9978737**. None equals 0.5. The spill is
therefore not solely floating threshold equality. Full object/background ranges
and selected counts are in the JSON.

Read-only inspection of hash-matched external `p3_ort_base.cpp` (SHA256
`7a72e42d4f1c1b0153f7e1b880d8b70c01b20504a81e151d1b563c39360c0d10`,
lines 237–263) shows integer truncation/clamping of boxes and an unresized clone
from the full-image model mask. It does not resize the mask in this configured
`full_image_masks` infer path. The adapter embeds that ROI without resampling,
uses strict `>0.5`, then unions the four samples per original pixel cell.

Controlled fixture tests verify centre mapping `(u512+0.5)/2-0.5`, all four sample
parities, edge cells, odd bbox placement, known RGB/BGR linear-interpolation bytes,
float32 neighbours of 0.5, and threshold-before-union versus averaging.
A single >0.5 sample survives union even when the interpolated average is below
0.5. Exactly 0.5 yields an empty mask and rejects. The existing confidence gate
remains inclusive at 0.8; immediately below rejects. Positive single-ID and
confidence-boundary acceptance tests use **TEST FIXTURES ONLY**. Background,
mixed IDs, duplicate associations/mappings, altered RGB/ID/hash/session/source,
manifest/artifact tampering and malformed ROI tests fail closed.

## Classification, limits and one next action

**CONFIRMED:** upstream EPD ROI spill relative to retained ID cells, with
conservative inverse-cell filling quantified separately. The inverse transform
introduces no new background cells. **UNKNOWN:** model versus earlier
postprocessing contribution, true independently rendered 512 silhouette, and
spill beyond integer ROI clipping. This evidence does not qualify physical
timing, penetration, contact or extraction.

**Single next action:** in a separately authorized bounded EPD experiment on the
same retained RGB bytes, retain raw full-image ONNX mask tensors and floating
boxes alongside postprocessed ROIs, then compare them offline at identical
coordinates and unchanged thresholds. This is the smallest missing observation
for separating model-output spill from postprocessing. It was **not run here**.

Rollback: revert this additive diagnostic/test/report change. PR #3176 remains
draft; original captures, inference artifacts, geometry and safety gates remain
unchanged.
