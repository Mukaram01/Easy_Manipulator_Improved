# Visual Mate Selection (R3.2)

Place a part by its faces, the way CAD assembly works: click the face of the part to move, click the face it should sit on, check the preview and Apply. For example, click the bottom of a bin, then the top of a table: the bin is seated flush on the table without typing any coordinates.

Visual Mate is another way to author the existing fixed mounting relation; it is not a new positioning system. It writes the same contract as Inspector **Mount To / Relative Pose…**:
- `parent_object` and `parent_link` (the parent datum);
- fixed `joint_type`;
- `origin`, the offset in the parent datum frame;
- optional `mate: {child_datum, alignment}`;
- reviewed datums copied into `mounting_anchors`.

`scripts/asset_mounting.py` owns all transform math, validation and persistence. Save, reopen, Generate, Validate, undo/redo, detach and reparent work exactly as they do for Mount To.

## Starting Mate

Any of these starts it:
- **Mate** on the Scene Builder toolbar;
- **Mate by Picking Faces…** in the Inspector, under Mount To;
- **Mate by Picking Faces…** in the Scene Hierarchy or 3D view context menu.

A selected asset becomes the first part. The Inspector shows the Mate panel; nothing changes until **Apply**. **Cancel** (or Esc) leaves without changes.

## Steps

1. **Part to move.** Click its mounting face in the 3D view. Datum markers show the verified mounting faces:
   - plane datums: a square with a normal arrow;
   - axes: a line;
   - origins and points: a small sphere with axes.
2. **Part it sits on.** Click its mounting face.
3. **Preview.** The panel shows:
   - where the part will go and how far it moves;
   - the relation and offset that will be stored;
   - which assets and destination zones follow;
   - whether the mate is valid, and if not, why (Apply is then disabled).

   The 3D view shows a translucent ghost of every part that will move. The preview is a dry run of the same resolver request that Apply sends.
4. **Alignment.**
   - **Flush:** plane faces touch with normals opposed. This is the default for two plane faces.
   - **Coincident:** datum frames line up. Use it for origins, points and axes.
5. **Position on the face.** The proposal keeps where the part is now, closes the gap and removes any tilt. Adjust **Along X / Y**, **Gap**, **Turn about normal** and **Tilt** as needed; **Reset** returns to the proposal.
6. **Apply.** One undoable edit.

Hovering shows what a click would pick, for example "Destination Bin: Bottom face (normal points down)" or "not a mounting surface".

## Hidden faces

A bin bottom sitting on a table cannot be seen. There are three ways to pick it:
- Datum markers are always drawn on top, so click the marker.
- **Isolate part** shows only the current part.
- **Look at a face ▾** turns the view to look straight at one of its datums from outside. After a pick, the view returns to the whole cell.

## What counts as a face

A click is resolved by one explicit policy (`include/mate_pick.hpp`), the same for every viewport:
- **Plane datum match.** The clicked face matches a plane datum only when both hold:
  - its normal is within 10° of the datum's outward normal;
  - the hit point is within 5 mm of the datum plane.

  The pick records the measured distance and angle.
- **Ambiguity.** Two equally close planes are ambiguous; choose one from the list.
- **Datum markers** name their datum exactly.
- **Refused with a reason:**
  - faces that match no verified datum (a bin side wall, a camera body);
  - placeholder shapes of meshes that are not loaded;
  - robots, generated previews and task zones;
  - views that draw the asset away from its authored pose.

  The panel then offers the datum list for that part, and **Choose datums in Mount To…** opens the existing dialog.

Datums come from the asset itself:
- its `origin`;
- its reviewed support surface (`surface_z_m`);
- its own `mounting_anchors`;
- reviewed catalog datums (`catalog/workcell_studio_asset_datums.yaml`) while its mesh calibration matches.

Arbitrary mesh triangles are never turned into faces.

## Rejected mates

The resolver rejects these. The panel shows the reason and Apply stays disabled; the scene and files are unchanged:
- self-mating;
- cycles (a table onto the bin that sits on it);
- flush without two planes;
- an axis coinciding with a plane;
- a missing datum or parent;
- over-constraint (a second parent);
- stale world poses;
- non-finite or out-of-range offsets;
- dynamic joints.

## Viewports

- **Embedded Web3D Product View (default):** full visual picking (surface hits, datum markers, hover, ghost, isolate, face view).
- **Native Scene3D fallback:** its picking only knows bounding boxes, not faces, so face picking is disabled there and the panel says so. Click a part (in 3D or the hierarchy) and choose its datum from the list; preview and Apply are the same.

## Known limits

- **No triangle-derived faces.** Only verified planar datums and their associated faces are pickable; there are no arbitrary mesh or STEP/BREP faces. The pick contract (`MateRawPick`) carries the geometry source and triangle so face-based datums can be added later.
- **Fixed, single-parent relations only.** No multi-constraint solver, no revolute or prismatic joints.
- **Camera stands are fixtures (fixed here).** Asset discovery and the Web3D export used to classify anything named "camera…" as a camera. A placed Camera Stand then became a second camera owner, so it was dropped from the Web3D scene, and reopening a cell with one failed Product View preparation ("exactly one physical camera owner"). Camera stands, mounts, brackets, frames, gantries and clamps are now physical fixtures; a real camera, and anything with `role: camera`, is unchanged.
- **Placeholder STLs wound inside out (fixed here).** The reviewed `camera_stand` STL and the other generated box placeholders (the Asset Library tables, bins, tray, tote, pallet, pedestal, workbench, conveyor and fence, and the camera frame/bracket/gantry/clamp packages) were wound inside out, so Web3D culled their outer faces and a click on the stand's top reached an inner wall and was refused. They are now wound outward with the same vertices and bounds, so the reviewed datums stay valid, and a click on the stand's top face resolves to its `top` datum. Cells saved before this fix keep their own inside-out copy under `assets/imported/catalog/`; place the asset again to pick its faces.
- **Newly placed library assets (fixed here).** A part placed from the Asset Library used to be drawn as a placeholder until the Product View was refreshed. Builder now copies its mesh into the Product View's mesh cache (`build/workcell_studio_web_scene/assets/<scene>/`, the same file a refresh writes) and the live view draws the real mesh at once, before and after Save. Face clicks on it work like on any other part. The copy is a cache: the cell keeps the original mesh path, and placing does not regenerate the Product View. A mesh outside the repository, or in a format Web3D cannot draw, stays a placeholder and the log says why.
- **Compound rotations in Web3D.** Web3D poses objects with three.js `XYZ` Euler order, while the resolver uses ROS fixed-axis RPY. Rotations about one axis, and yaw-only rotations, are drawn exactly. Combined tilt and yaw may be drawn differently, and surface picks on such assets are refused as a pose mismatch. Datum markers, the stored relation and the generated cell are unaffected.
