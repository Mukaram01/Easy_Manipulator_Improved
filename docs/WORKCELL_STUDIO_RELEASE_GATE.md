# Workcell Studio release gate

Run the integrated gate from the repository root to decide whether Workcell Studio is ready for a simulation-first release:

```bash
python3 scripts/run_workcell_studio_release_gate.py --profile static
```

Profiles:

- `static`: safe CI checks that do not require a ROS runtime.
- `runtime`: Ubuntu 22.04 / ROS 2 Humble workspace checks for Qt, Workcell Builder, fake hardware, MoveIt, pick/place smoke, perception replay, and safety defaults.
- `demo`: canonical `ur5_2f_test` end-to-end fake-hardware simulation acceptance.

The command writes JSON and Markdown reports under `build/workcell_studio_release_gate/` by default. Required `FAIL` and `BLOCKED` checks return non-zero. Missing ROS or Qt dependencies are reported as `BLOCKED`, not `PASS`.

## Manual runtime checklist

On an Ubuntu 22.04 ROS 2 Humble workstation or VM:

```bash
cd /home/user/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder
source install/setup.bash
cd src/easy_manipulation_deployment
python3 scripts/run_workcell_studio_release_gate.py --profile runtime --workspace-root /home/user/workcell_ws --scene ur5_2f_test
python3 scripts/run_workcell_studio_release_gate.py --profile demo --workspace-root /home/user/workcell_ws --scene ur5_2f_test
```

Confirm that every launched process terminates, `use_fake_hardware:=true` remains the exercised path, and no automatic real-robot execution was enabled.
