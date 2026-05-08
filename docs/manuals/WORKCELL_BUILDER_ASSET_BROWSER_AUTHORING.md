# Workcell Builder Asset Browser Authoring

- Select an item in **Asset Browser** and use action buttons in **Ingredients**.
- Use **Set as Robot**, **Set as End Effector**, **Add as Support Surface**, and **Add as Pick Object** for core role assignment.
- Use **Import Custom STL** to add `.stl`, `.dae`, or `.obj` visual assets.
- Review added entries in **Current Cell Assets**.
- Edit pose and metadata in the inspector/layout tabs (XYZ/RPY, collision mode, runtime support status).
- Use **Duplicate Selected Asset**, **Remove Selected Asset**, and **Clear Cell Assets** to manage layout drafts.
- Validate from **Validate Cell**. Preview-only assets produce warnings, not hard failures.
- Generate exports from **Generate Canonical Files** / **Generate Studio Pack**.
- Generated YAML/JSON outputs include selected assets, source ids/paths, roles, pose, collision mode, and support status.
- EPD remains separate from workcell_builder GUI.
- Fake hardware remains the default safe mode.
