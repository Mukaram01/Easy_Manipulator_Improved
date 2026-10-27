# Align & Arrange (two-part alignment)

Line one placed part up with another without typing coordinates. For example, centre a destination bin on a workbench and rest it on the bench top, or push it against the bench's left edge with a 50 mm margin.

Align & Arrange is a positioning aid on the existing placement contract, not a new transform engine or a CAD constraint solver:
- `scripts/asset_alignment.py` measures both parts and plans the move.
- The move is an ordinary `scripts/asset_mounting.py` edit (`world_pose`, plus a fixed mount for **Keep relative**).
- Apply goes through the same undoable commit as **Mount To** and **Mate**.
- Save, reopen, Generate, Validate, undo and redo behave as they do for any other placement edit.

## Starting it

1. Select the part to move: a bin, fixture, table or other placed asset.
2. Start Align & Arrange in either of these ways:
   - Inspector → **Align & Arrange…**, under Mate by Picking Faces;
   - **Ctrl+Shift+A** in Scene Builder.
3. Choose the **Reference** in one of these ways:
   - click it in the 3D view;
   - click it in the Scene Hierarchy;
   - pick it from the list.

   While the panel is open, a click chooses the reference; the part to move stays selected. The 3D move gizmo is held off, so choosing a reference cannot move anything.

The Inspector shows the Align & Arrange panel. Nothing changes until **Apply**. **Cancel** or Esc leaves without changes.

## Operations

Every operation works in the **reference part's own axes**, so a rotated reference keeps its own sides:

| Button | Meaning (reference axes) |
|---|---|
| Centre X / Y / Z | Box centres match along that axis |
| Centre on reference | Centre X and Centre Y together (height kept) |
| Left / Right | The −X / +X box faces line up |
| Front / Back | The −Y / +Y box faces line up |
| Bottom / Top | The −Z / +Z box faces line up |
| Place on top | The part's box bottom rests on the reference: on its verified upward surface datum (for example a workbench `surface`) when it has one, otherwise on its box top |
| Mate faces… | Leaves for the Mate tool: flush on verified faces (datums), not boxes |

How operations combine:
- One operation per axis. Clicking an active operation clears it, and Clear removes all.
- Operations on different axes combine; for example, Centre on reference with Place on top.
- **Offset (mm)** adds an exact distance along each axis that has an operation. For example, Left with X +50 leaves a 50 mm margin. Offsets on axes without an operation are disabled.

## Preview

- The panel shows the planned move in numbers:
  - the distance;
  - the world ΔX/ΔY/ΔZ;
  - the target position.
- It states how each box was measured, for example "mesh vertices (sorting_bin.stl, 316 triangles, authored scale/RPY/offset)" or "authored box dimensions".
- It lists any parts mounted on the moving part, because they follow it.
- The 3D Product View shows a translucent ghost at the planned pose and highlights both parts. The native fallback view shows the numbers only.
- The orientation never changes.
- The preview is a dry run of exactly the request Apply sends.

## Align once versus Keep relative

- **Align once** (default) changes the pose only. The part does not follow the reference later. A part already mounted on another part stays mounted; only its offset changes.
- **Keep relative** also mounts the part on the reference with a fixed offset: the existing Mount To relation, with `parent_object` set to the reference and parent datum `origin`. The part then follows when the reference moves. Detach it later in Mount To. This option is unavailable when the reference is locked.

Neither option creates a CAD face constraint.

## What is measured, and what is rejected

Each part's box is the axis-aligned box of its **real authored geometry in its own frame**:
- **STL meshes:** vertices after the authored mesh calibration. Scale is applied first, then the mesh RPY, then `origin_offset`. This is the same convention the Product View and the MoveIt collision manifest use.
- **Box primitives:** their authored dimensions.

Pivots away from the geometry centre and mesh offsets are therefore honoured. When the moving part is turned relative to the reference by anything other than quarter turns, its box in the reference frame encloses the rotated box, and the preview says so.

Alignment is refused, with the reason and the next step, when:
- the part to move is locked, generated (robot or tool visuals) or a task zone;
- the reference is the part itself, a task zone, a generated robot or tool visual, or a part mounted on the moving part (it would move with it);
- a part has no readable geometry: a missing or unreadable mesh, a non-STL mesh, a cylinder or other primitive, or zero, negative or non-finite dimensions;
- mesh bounds larger than 50 m (usually a millimetre STL without scale 0.001);
- a part is not in the saved layout yet, or has unsaved size edits ("Save Layout, then align");
- no operation is chosen, or an offset is larger than 10 m.

Faces, holes, axes and mounting datums are never derived from mesh triangles. Collision checks and mounting validation are unchanged.

## Quick placement and Mate

- **Quick placement** in the Product View still places one part on the nearest table, edge or task area.
- **Mate** still seats a part by verified faces.
- Align & Arrange is the two-part tool between them.

### Recovering from a missed Mate click

A Mate click that hits no part (background, floor, a hidden part) changes nothing:
- the Mate panel says the click missed and lists what is still picked;
- both picks, the preview and Apply stay as they were;
- only Cancel or Esc ends the Mate.

## Checking it

```bash
python3 -m pytest -q tests/test_asset_alignment.py
```

Also run the `workcell_align_arrange_controller_test` gtest from the `workcell_builder` build.
