# Workcell Builder Operator Workflow

## Scope
Operator runbook for validating and generating a scene package from `workcell_builder` with fake-hardware-first safety.

## Steps
1. Open `workcell_builder` and select or create a scene.
2. Run **Run Offline Validation**.
3. Generate scene package artifacts.
4. In **Next Steps**, click:
   - **Copy Build Command**
   - **Copy Launch Command**
5. Build:
   - `colcon build --symlink-install --packages-select <scene>`
6. Launch:
   - `ros2 launch <scene> demo.launch.py use_fake_hardware:=true`

## Generated Artifacts
Each generated scene should include:
- `README.md` with build/launch/fake-hardware guidance.
- `builder_session_summary.json` with operator session command summary.
- Existing studio summary files (`workcell_studio_summary.json/.md`).

## Safety
- Fake hardware remains default.
- Real hardware requires explicit validation and `use_fake_hardware:=false`.
- Offline preview/metadata generation must not command robot motion.
