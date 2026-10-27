# Multi-selection and the arrangement toolbar

Select several parts and arrange them against one reference part with a compact icon toolbar above the 3D view.

- Each operation is previewed as ghosts, then applied as **one** undoable step.
- The reference never moves.

The toolbar is an interaction pattern familiar from PLC/HMI and CAD layout editors, adapted to a 3D assembly editor. It reuses the existing contracts:
- geometry from `scripts/asset_alignment.py`;
- the transaction from `scripts/asset_arrangement.py`;
- every edit from `scripts/asset_mounting.py`;
- Mate for verified faces.

## Selecting

| Action | 3D view | Scene Hierarchy |
|---|---|---|
| Select one part | Click | Click |
| Add or remove a part | Ctrl+click (Cmd+click on macOS) | Ctrl+click |
| Add a range | — | Shift+click |
| Clear | Click empty space | — |

- The **first** part you select is the **reference** (anchor). It is outlined in orange in the 3D view; the other members are outlined in blue.
- Change the reference from the selection chip at the left of the toolbar, for example `4 selected · ref: Industrial Workbench ▾`.
- **Hierarchy:** every member is highlighted.
- **Inspector:** shows "N parts selected — reference X (stays put)" and the details of the reference.
- While two or more parts are selected, the single-part move gizmo is switched off, so a drag cannot move one member on its own.
- Locked parts may be the reference, but cannot be moved: tools that would move them are disabled and say why.
- Task zones and helper overlays never join a multi-selection.
- The selection survives previews, Cancel, Apply, Undo, Redo and Save. Parts that disappear (delete, reload) drop out, and the reference passes to the earliest remaining member.

## Tools

Every tool works in the **reference part's own axes** (left/right −X/+X, front/back −Y/+Y, bottom/top −Z/+Z), so a rotated reference keeps its own sides.
- Hover a tool for its description and shortcut.
- A disabled tool says why in its tooltip.
- When the 3D view is narrow, the remaining tools move to the **More (⋯)** menu.

### Align (one-time placement)

| Tool | Shortcut | Result |
|---|---|---|
| Align left / right | Alt+Shift+L / R | −X / +X box faces line up with the reference's |
| Align front / back | Alt+Shift+F / B | −Y / +Y box faces |
| Align bottom / top | Alt+Shift+G / T | −Z / +Z box faces |
| Centre X / Y / Z | Alt+Shift+X / Y / Z | box centres match on that axis |
| Centre in XY plane | Alt+Shift+C | centred in X and Y; heights kept |
| Centre on reference | Alt+Shift+O | box centres coincide in X, Y and Z |
| Place on top | Alt+Shift+P | resting on the reference's verified upward surface datum, or its box top |

### Distribute (one-time placement)

- **Distribute along X / Y / Z:** equal gaps between neighbours, in their current order along the axis. Needs three or more parts. The reference keeps its place: if it is in the middle of the row, the others are arranged around it.
- **Distribute with gap…:** an exact gap in millimetres along a chosen axis. Works with two or more parts.
- The preview states the order, for example `Destination Bin → Industrial Workbench → Reject Bin`, and the gap.

### Transform (one-time placement)

- **Move / rotate…:**
  - exact X/Y/Z in millimetres and Rx/Ry/Rz in degrees;
  - in world, reference or each part's local axes;
  - rotation about each part's origin or about the reference.
- **Snap to grid…:** snaps each part's origin to a horizontal world grid (X and Y; height kept).

### Assembly (relationships and verified constraints)

| Tool | Kind | Notes |
|---|---|---|
| Fixed mount on reference | persistent parent/child | Parts keep their place now and follow the reference later (the Mount To relation). |
| Detach | removes the relation | World positions are kept. |
| Flush / Coincident | verified geometric mate | Opens the Mate tool for exactly two parts: pick verified faces or datums. |
| Concentric | verified geometric mate | Enabled only when both parts have a reviewed axis datum (for example `bin_standard`'s `centre_axis`). |
| Group | — | Disabled: groups are not part of the scene contract yet. Use Fixed mount to keep parts together. |

These are three different kinds of operation, and the toolbar keeps them apart:
1. **One-time placement** (Align, Distribute, Transform) moves parts. It creates no relation.
2. **Persistent parent-relative positioning** (Fixed mount) stores a fixed offset; the part follows its parent.
3. **Verified geometric mating** (Flush, Coincident, Concentric, through Mate) is built only on reviewed datums, never on mesh triangles.

## Preview, Apply, Cancel

- Choosing a tool opens a preview bar under the toolbar: what will move, by how much, the reference, and any mounted followers.
- Translucent ghosts show every moving part.
- Fields such as the gap, move distances or grid step update the preview as you type.
- **Apply** (Enter) commits every part as one undoable step.
- **Cancel** (Esc) changes nothing.
- If anything changed the scene or the selection after the preview, Apply refuses and shows the updated preview.

## Refusals

The toolbar refuses, with the reason, when:
- a moving part is locked, generated (robot or tool visuals) or a task zone;
- a selected part is mounted on another selected part, or the reference would follow a moving part;
- geometry is missing or unreadable, or units look wrong (see [Align & Arrange](ALIGN_AND_ARRANGE.md));
- a part is not saved yet;
- equal spacing has fewer than three parts, or the parts are longer than the span;
- an offset is beyond 10 m.

## Checking it

```bash
python3 -m pytest -q tests/test_asset_arrangement.py tests/test_save_layout_fidelity.py
```

Also run these gtests:
- `workcell_scene_selection_set_test`;
- `workcell_arrange_controller_test`;
- `workcell_scene3d_mesh_offset_convention_test`.

The GUI runner is `workcell_builder --multi-select-acceptance --workspace WS --scene ur5_2f_test --smoke-output DIR --window-size 1280x720`, on a fixture built by `docs/acceptance/multi-select-arrange-v1/build_fixture.py`.
