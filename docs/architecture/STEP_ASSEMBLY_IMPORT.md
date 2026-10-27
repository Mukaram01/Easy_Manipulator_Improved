# STEP assembly import — architecture and next-step specification

**Status:** design only. No STEP importer exists yet; nothing here is implemented. This document keeps the arrangement and mounting work compatible with a future Open CASCADE (OCCT)-backed importer.

## Where things stand

| Area | Today |
|---|---|
| Import | `external_asset_importer` takes one mesh file (STL/OBJ/DAE) and a flat request (name, category, box dimensions, pose, licence). It writes a catalog entry; there is no assembly tree, units, colours or topology. |
| Geometry for authoring | Layout items carry a mesh path plus calibration (`scale`, `rpy`, `origin_offset`, URDF visual-origin order). Bounds are read from STL vertices (`asset_alignment.part_box`). |
| Verified geometry | Reviewed datums (`plane`, `axis`, `point`) per catalog asset in `catalog/workcell_studio_asset_datums.yaml`. They are offered only while the mesh calibration matches, and copied into the asset's `mounting_anchors` when used. Datums are never inferred from triangles. |
| Relationships | One fixed parent per asset (`parent_object`, `parent_link` datum, `origin`, optional `mate`), resolved by `scripts/asset_mounting.py`. Multi-object arrangements are atomic edits on top of it (`asset_arrangement.py`). |
| Rendering | Web3D (three.js) loads STL/DAE/OBJ with materials from the export; native Scene3D is the fallback. |

## Principle

A STEP import must produce **the same contracts** the editor already trusts. It must not create a parallel geometry or constraint system:
- one asset per part instance;
- display meshes as derived artifacts;
- **exact topology kept as the source of verified datums**;
- the assembly tree as ordinary fixed mounts.

**Never:** STEP → STL only, presented as "CAD intelligence".

## Pipeline (proposed)

```text
STEP (AP203/AP214/AP242)
  └─ OCCT XDE (XCAFDoc: shapes, assembly labels, colours, materials, names, units)
       ├─ Assembly tree      → asset instances + fixed mounts (parent_object, origin)
       ├─ Per-part B-rep     → kept as the authoritative geometry (.brep / .step per part, hashed)
       ├─ Tessellation       → display mesh (glTF/OBJ with per-face colour) + bounds  [derived, regenerable]
       ├─ Topology index     → faces / edges / cylindrical axes with stable references
       ├─ Verified datums    → plane / axis / point candidates, reviewed before use
       └─ Collision geometry → separately generated and separately reviewed (box/convex/mesh)
```

### 1. Assembly tree and part instances

- Each XDE assembly component becomes an **asset instance** with a stable ID.
  - The ID derives from the assembly path and instance name, not array order.
  - Each instance records `source: {file, sha256, label_path}`.
- A shared part definition (one shape, many instances) is one catalog entry with several instances. Each instance carries its own placement.
- The tree maps to **fixed mounts**: a child instance gets `parent_object: <parent instance>`, `parent_link: origin` and `origin` = its XDE location. The existing resolver then yields world poses, cycles stay impossible, and Detach/Mount work as today.
  - Sub-assemblies without geometry become frame-only parents.
  - A `frame_only: true` flag needs a schema addition.

### 2. Colours and materials

- XCAFDoc colours (surface, curve, generic) and material names are read per shape and per face.
- The display mesh carries per-face colours (glTF materials, or OBJ `.mtl`).
- The layout records `material: {name, color}`, which `PreviewItem` already reads (`material_name`, `material_r/g/b/a`).
- Material names are **visual metadata only**; physical properties (density, friction) are not inferred.

### 3. Units and local transforms

- XDE length units, or the STEP `SI_UNIT` and `CONVERSION_BASED_UNIT` declarations, are read explicitly and converted to metres at import. The original unit is recorded.
- A part whose unit is missing is refused. The 50 m "mm without scale" guard in `asset_alignment` stays as a second line of defence.
- Placements are converted to ROS fixed-axis RPY with the same `angles()` used by the resolver. The import report lists any rotation that does not round-trip within 1e-9.

### 4. Exact faces, edges and cylindrical axes

- Every `TopoDS_Face`, `TopoDS_Edge` and vertex of a part gets an index entry:
  - its surface type (`GeomAbs_Plane`, `GeomAbs_Cylinder`, …);
  - its parameters, in the part frame: plane origin and normal, or cylinder axis, radius and location;
  - its area or length.
- **Datum candidates** come only from exact analytic surfaces:
  - plane faces → `plane` datums;
  - cylindrical and conical faces → `axis` datums (this is what enables the toolbar's *Concentric*);
  - circular edges → `axis` and `point` datums.
- Candidates are offered for review in the Mate UI. Once used, they are copied into `mounting_anchors` exactly like reviewed catalog datums. The pick policy (`mate_pick.hpp`) gains a `brep_face` geometry source, so a click on a tessellated triangle maps to its **source face index**, not the triangle.
- Free-form (NURBS) faces yield no datums.

### 5. Topological references and provenance

- Every datum derived from topology stores `{part_sha256, face_index, surface_type, parameters_hash}`.
- The face index is the deterministic OCCT explorer order on the stored B-rep, never on the tessellation.
- The import report lists, per part: source file and hash, unit, face/edge counts, analytic face counts, colours and materials found, and datums offered.

### 6. Reimport identity and broken references

- Reimporting the same STEP matches instances by assembly path and part by B-rep hash. Unchanged parts keep their IDs, datums and mounts.
- A changed part keeps its ID only if every **used** datum can be re-found: same surface type, parameters within tolerance, located by geometric match rather than by index.
- Otherwise each affected mount is marked **broken**, with the old and new parameters. The resolver refuses to generate until the user re-picks or detaches.
- Nothing is silently re-attached.

### 7. Visual and collision geometry, verified separately

- **Visual:** the display tessellation, used for rendering and box bounds. Its deflection is recorded.
- **Collision:** generated separately, either per part as a box or convex proxy, or as an exact-tessellation mesh with a triangle budget. It carries its own `review_required`, the same contract as `generate_moveit_collision_manifest.py`.
- Visual geometry is never promoted to collision without review.

## Compatibility with today's code

| Existing piece | STEP import plugs in by… |
|---|---|
| `asset_mounting.anchors` / `mounting_anchors` | Topology-derived datums use the same typed frames (`plane`/`axis`/`point`, +Z normal or axis), with an added `source: brep:<sha>#face<n>`. |
| `asset_alignment.part_box` | Reads bounds from the part's stored B-rep (`BRepBndLib`, oriented in the part frame) when present. Otherwise it reads the tessellation, as today. |
| `asset_arrangement` | Unchanged: it consumes boxes and edits. |
| Mate (`mate_pick`, `MateToolController`) | A `brep_face` pick resolves to `face_index` and then to a datum candidate. Concentric becomes available wherever cylinders exist. |
| Web3D | The display mesh carries a per-triangle face-index attribute, so picks report `face_index`. Colours come from glTF materials. |
| Save / reopen | Assets are ordinary layout items. The B-rep and tessellation live under `assets/imported/<part_sha>/` with hashes, so saved scenes never depend on the original STEP file. |

## Next step (smallest useful milestone)

**Read-only coloured STEP assembly import (no editing of topology):**

1. Load a STEP with OCCT XDE in a separate helper process, so that OCCT is not linked into `workcell_builder`.
2. Write, per part:
   - the B-rep;
   - a coloured glTF tessellation;
   - a topology index of plane and cylinder faces.
3. Show the assembly tree in the Scene Hierarchy as locked imported parts, with colours, units and provenance in the Inspector.
4. Offer analytic plane and axis datums as **candidates** in Mate. Nothing is used without an explicit pick.

**Acceptance:**
- A two-level assembly (for example a fixture with four bolts) imports with the correct tree, colours, units and instance placements.
- Reopen is identical.
- Concentric is enabled only between real cylindrical axes.
- A reimport with one modified part flags exactly the mounts that used its changed faces.

**Dependency:** OCCT 7.6+, from the Ubuntu 22.04 `libocct-*` packages. It is used only by the helper process. Licence: OCCT is LGPL 2.1 with an exception; record this in the dependency review before use.
