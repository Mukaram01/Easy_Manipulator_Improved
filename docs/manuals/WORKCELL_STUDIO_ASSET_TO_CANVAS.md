# Workcell Studio Asset-to-Canvas Placement

This guide covers Asset Browser → Add/Place on Canvas → Configure Pose/Size → Save Layout → Validate → Generate/Preview.

## Asset Catalog panel in Scene Builder
Scene Builder includes:
- Scene Hierarchy
- Asset Catalog

Asset categories:
- Robots
- End Effectors
- Cameras
- Tables
- Conveyors
- Bins
- Fixtures
- Objects / STLs
- Pick/Place Zones
- Custom / Imported

Each asset row exposes: display name, category, source package/path, readiness status, short description, **Add to Canvas**, **Open Asset Folder**, **Copy Asset Path**.

## Add and place assets
Use **Add to Canvas** to create a canvas item with unique id and default pose near the active work area. Inspector pose/size fields are immediately editable.

## Custom object flow
- Import STL / URDF
- Add Existing STL to Canvas
- Generate Simple Box/Cylinder Placeholder
- Set role: pick_object, obstacle, fixture, bin, support_surface

## Save and persistence
Save Layout explicitly writes `layout/workcell_studio_layout.yaml` with id/type/category/role/source/pose/size and generated/imported markers.

## Task binding shortcuts
For the selected item:
- Use Selected as Pick Source
- Use Selected as Place Target
- Use Selected as Pick Zone
- Use Selected as Place Zone
- Use Selected as Camera

## Safety
No robot motion is commanded. Fake-hardware-first defaults remain required.
