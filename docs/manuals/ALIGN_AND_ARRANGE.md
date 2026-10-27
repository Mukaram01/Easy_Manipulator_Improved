# Align & Arrange (two-part alignment)

Line one placed part up with another without typing coordinates. For example, centre a destination bin on a workbench and rest it on the bench top, or push it against the bench's left edge with a 50 mm margin.

Align & Arrange is a positioning aid on the existing placement contract, not a new transform engine or a CAD constraint solver:
- `scripts/asset_alignment.py` measures both parts and plans the move.
- The move is an ordinary `scripts/asset_mounting.py` edit (`world_pose`, plus a fixed mount for **Keep relative**).
- Apply goes through the same undoable commit as **Mount To** and **Mate**.
- Save, reopen, Generate, Validate, undo and redo behave as they do for any other placement edit.

## Where it lives now

The separate Inspector panel (Inspector → Align & Arrange…, Ctrl+Shift+A) was retired. The
[arrangement toolbar](MULTI_SELECT_ARRANGE.md) above the 3D view is the one place for alignment, and it uses the
same engine (`scripts/asset_alignment.py`):

1. Click the **reference** part, then Ctrl+click the part (or parts) to move.
2. Choose an Align tool: edges, Centre X/Y/Z, Centre in XY plane, Centre on reference or Place on top.
3. In the preview bar, set exact **ΔX/ΔY/ΔZ** offsets in millimetres (reference axes) and, if wanted,
   **Keep relative (mount on reference)**.
4. Check the ghosts and the numbers, then **Apply** (one undoable step) or **Cancel**.

Every rule works in the reference part's own axes, so a rotated reference keeps its own sides:
left/right −X/+X, front/back −Y/+Y, bottom/top −Z/+Z. The orientation of the moved parts never changes.

**Align once** (default) changes poses only. **Keep relative** also mounts each part on the reference with a
fixed offset (the existing Mount To relation), so it follows the reference; Detach removes it. Neither creates a
CAD face constraint: for verified faces use Flush / Coincident / Concentric (the Mate tool).

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
