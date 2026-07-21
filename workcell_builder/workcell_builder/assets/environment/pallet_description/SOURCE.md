# Pallet visual source

This directory contains one external visual-asset pilot used to validate the Workcell Studio asset import and Web3D rendering path.

- Creator: Kenney
- Pack: Retro Urban Kit 2.0
- Asset: `Models/OBJ format/pallet.obj`
- Source repository: `henrylewis2004/Code-Geass-SRPG`
- Source commit: `c3006c78bc764fc86a113316cbf2a0e5e48b7231`
- Source path: `assets/3d/kenney_retro-urban-kit/Models/OBJ format/pallet.obj`
- License: CC0 1.0
- Redistribution confirmed: yes

## Workcell Studio modifications

The source OBJ contained a duplicated face block and referenced an external material/texture. The pilot copy was normalized to keep only the seven original rectangular plank volumes, remove duplicate faces, and remove the material/texture dependency. The geometry remains equivalent to the source model.

The source model is Y-up and has approximate local bounds `1.0 x 0.15 x 1.0 m`. The Xacro rotates it into ROS Z-up and applies mesh scale `1.2 0.96 0.8`, producing a final footprint and height of `1.2 x 0.8 x 0.144 m`.

The previous generated `pallet.stl` is intentionally retained during this pilot as a rollback asset. It must not be deleted until the OBJ is visually confirmed in Product View/Web3D and the generated scene asset staging path is verified.
