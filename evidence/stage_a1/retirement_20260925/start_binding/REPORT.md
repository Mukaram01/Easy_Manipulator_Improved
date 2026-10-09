# Stage A1 measured/controller start binding — runtime investigation

## Decision

**START_BINDING_UNQUALIFIED.** No start-binding certificate was enabled. No historical opening diagnostic was rerun, no R was issued, and no simulator recovery or physical acceptance was run. The repository remains at the required starting HEAD `076f4e552bbbcd2cb04a8229557e61be7e4f42af`; its production code, tests, saved evidence and collision policies are unchanged.

The runtime does not perform a discrete projection from measured follower positions M to the nominal mimic state C when a trajectory begins. It controls independently measured physical follower joints through a velocity-feedback command that the DART dynamic servo processes. The evidence supports this command law, but does not supply a conservative physical response/rounding enclosure over the first JTC interval. Treating the raw residuals as harmless coordinate differences would assume the missing bound.

## Required baseline verification

- `git status --short`: clean.
- `git rev-parse HEAD`: `076f4e552bbbcd2cb04a8229557e61be7e4f42af`.
- Latest commits: `076f4e55`, `30515090`, `9a32e5e3`.
- Read `docs/manuals/STAGE_A1_DETACHED_SEPARATION.md` before investigating.
- The completed continuous and detached certificate architecture was not reworked or re-audited.

## Actual command and state ownership

Saved runtime source: `/home/ubuntu/workcell_ws/stage-a1-finish-20260923-160609/05-contact-release/runtime`.

| Controller | Commanded joints | Command interface |
|---|---|---|
| `ur5_arm_controller` | shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint | position |
| `ur5_gripper_controller` | gripper_finger1_joint | position |

The five follower joints have position/velocity **state** interfaces and no command interface exposed to JTC. In the hardware plugin their exported state names have `_mimic` suffixes. They are not passive in the simulator: the hardware plugin computes velocity feedback for them.

The runtime URDF and hardware parameters agree on all five graph edges and multipliers, and every offset is zero. Nonzero offsets are rejected by the authored simulator generator; the installed 0.7.21 mimic feedback itself has no offset term. No follower is present in either JTC command joint list. The JTC source uses configured command joint names; it does not expand a URDF mimic graph into direct follower commands.

## Installed implementation and package binding

| Component | Installed version |
|---|---|
| JTC | ros-humble-joint-trajectory-controller 2.54.0-1jammy.20260908.124006 |
| gz/ign ros2_control | 0.7.21 (gz build 20260908.125238; ign build 20260909.012031) |
| Gazebo | 6.18.0-1~jammy |
| ignition-physics dartsim | 5.4.0-1~jammy |
| DART | 6.12.1+dfsg4-11build2 |

`dpkg -V` reported no modified files in the gz/ign control and JTC packages. The saved wrapper plugin SHA-256 matches the installed `/opt/ros/humble/lib/libign_ros2_control-system.so`: `2b4454cbcdd2e815f0a7fecfcfffdfdfccbd1994405401e7fba6e11ffed92913`.

The plugin XML resolves `ign_ros2_control/IgnitionSystem` to the installed `gz_hardware_plugins` library. The installed header aliases `IgnitionSystem` to `GazeboSimSystem`. The actual installed hardware library tested here has SHA-256 `42efd2fec6ab5d00d0018b4052d1ffeb126d99c297a5c944cca96a6d43101617`. The historical receipt recorded the wrapper hash, not a separate hardware-library hash; the installed-library tests establish current installed behavior, while saved configuration and initialization logs corroborate the historical configuration.

Exact-version source files and URLs are preserved in `sources/manifest.json`:

- [gz_ros2_control 0.7.21 hardware implementation](https://github.com/ros-controls/gz_ros2_control/blob/0.7.21/gz_ros2_control/src/gz_system.cpp)
- [gz_ros2_control 0.7.21 update scheduling](https://github.com/ros-controls/gz_ros2_control/blob/0.7.21/gz_ros2_control/src/gz_ros2_control_plugin.cpp)
- [JTC 2.54.0](https://github.com/ros-controls/ros2_controllers/blob/2.54.0/joint_trajectory_controller/src/joint_trajectory_controller.cpp)
- [Gazebo 6.18.0 command/reset dispatch](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/src/systems/physics/Physics.cc)
- [ignition-physics 5.4.0 DART command implementation](https://github.com/gazebosim/gz-physics/blob/ignition-physics5_5.4.0/dartsim/src/JointFeatures.cc)

## What happens to a residual

For this saved interface order (position first), the installed plugin selects position mimic feedback. For follower f with multiplier m:

```
v_command(f) = -100 * (q_measured(f) - m * q_measured(leader))
```

This is a velocity **command**, not a guaranteed position trajectory. The ordinary leader position command uses gain 0.1 and update rate 100 Hz. The Gazebo plugin writes hardware commands each 1 ms physics tick; controller read/update is at 10 ms. Saved initialization logs confirm these rates and gain.

- `read()` returns actual Gazebo `JointPosition`/`JointVelocity` components.
- Normal `write()` creates/updates `JointVelocityCmd`; it does not set follower `JointPosition` or create `JointPositionReset`.
- The reset path exists separately for initialization, not follower projection on trajectory acceptance.
- The physics system forwards velocity commands to ignition-physics.
- The DART backend selects `Joint::SERVO` and calls `setCommand`.
- Installed DART explicitly classifies SERVO as dynamic, unlike its kinematic velocity/acceleration/locked modes. Command assignment does not set position or velocity.

Nine direct installed-library tests confirm these points without launching a Gazebo server, advancing physics, constructing a trajectory, or sending an action. They use the saved URDF and measured joint values in a fresh in-memory ECM. No production or saved state is modified.

## Raw measured state versus controller state

Raw source epoch: run `4f31e734e14e4e9391c9bf65d7630245`, PID 153385, iteration 104485, simulation time 104485000000 ns. Complete raw coverage is 12 joint positions/velocities.

Telemetry writes Gazebo joint component values at 17 decimal digits. These residuals are approximately 408,149 to 2,221,526 floating-point ULPs. They are not explained by decimal serialization rounding. They can be numerical solver/control residuals of the simulated physical state; that does not make the distinct DOF values interchangeable with nominal mimic coordinates.

All names below have prefix `gripper_`. Delta means **C − M**.

| Follower | M (rad) | C (rad) | C − M (rad) |
|---|---:|---:|---:|
| finger2_joint | 0.55269734136811732 | 0.55269734141968274 | +5.1565418601740021e-11 |
| finger2_inner_knuckle_joint | 0.55269734136575255 | 0.55269734141968274 | +5.3930193644191604e-11 |
| finger2_finger_tip_joint | −0.55269734136579263 | −0.55269734141968274 | −5.3890114593002636e-11 |
| finger1_inner_knuckle_joint | 0.55269734117304381 | 0.55269734141968274 | +2.4663893150034255e-10 |
| finger1_finger_tip_joint | −0.55269734137436910 | −0.55269734141968274 | −4.5313641727773302e-11 |

### Why the prior diagnostic showed only three

A direct state-import test confirms that loading the saved PlanningScene message into MoveIt replaces the two finger-1 follower coordinates with their nominal values. The original raw sample and serialized message still contain them; neither file was edited. A separate full-vector `RobotState::setVariablePositions` test preserves all 12 raw coordinates exactly. No such replacement was introduced into production or the historical opening diagnostic in this task.

The prior geometry result obtained from the imported state is not a certificate for raw M: the overwritten finger-1 coordinates affect the initial-contact link chain.

A future binding proof must take the complete raw measured vector as M, validate its complete coverage and epoch, and must not use the partially canonicalized state as if it were complete physical measurement.

## Why neither start model is certified

### REPRESENTATION_ENCLOSURE

A static hull containing M and C would enclose those two configurations at one instant. Current evidence does not establish that all physical follower states during the first JTC interval remain inside that hull (or any already-certified propagated hull). Followers react to the leader's **measured** motion through dynamic feedback. Their nonzero measured velocities and the moving leader are part of the initial dynamic state. Absence of a reset is not proof that canonical C and measured M are merely two exact encodings of identical geometry.

### PHYSICAL_TRANSITION

The installed command law is known, but the actual response is computed by the constrained DART dynamic solver. It is not an algebraic assignment to C or a certified linear interpolation from M to C. A formula such as `q_next = q + dt * v_command` would assume that commanded velocity equals realized velocity and would omit the dynamic solver, constraints, effort limits and rounding. No such assumption was made.

Therefore no conservative transition/tube from the complete raw measured state through the first real JTC interval has been established. No epsilon test, state normalization, collision threshold change, or fallback to nominal C was added.

## Geometry and downstream status

| Gate | Result in this task |
|---|---|
| Start binding | FAIL — START_BINDING_UNQUALIFIED |
| Full start enclosure robot/world/self geometry | NOT CERTIFIED; no supportable propagated runtime enclosure |
| Exact initial pair lifecycle | No transition applied; remains ACTIVE |
| First real JTC interval for full representation | NOT CERTIFIED |
| Historical opening separation / continuous audit / expiry / endpoint | NOT RERUN — start-binding prerequisite failed |
| Recovery R | NOT ISSUED |
| Failed attempt A | Remains revoked |
| Simulation recovery / physical acceptance | NOT RUN |
| Baseline ACM / attachment / contact limits | Unchanged |

The requested start-binding acceptance cases (including new/self collisions, pair-identity changes, 100 µm exceedance, missing measurements, wrong graph and stale epoch) must test a supportable binding model. No acceptance path was implemented for an unsupported model merely to make those tests pass. Existing strict certificate and recovery regressions are rerun independently, with no architectural changes.

## Verification results

- Installed-runtime probes: **9 passed**, 0 failures.
- Existing C++ certificate regressions: **64 passed**, 0 failures.
- Existing Python recovery/runtime regressions: **342 passed**, 0 failures.
- Independent runtime-evidence review supports `START_BINDING_UNQUALIFIED`; see `independent-review.md`.
- Final repository verification: required HEAD unchanged, clean worktree; PR #3174 open and draft.

## Reproduction and artifacts

After sourcing `/opt/ros/humble/setup.bash`, the existing MoveIt teardown overlay, and the workspace overlay:

```
python3 /home/ubuntu/workcell_ws/stage-a1-start-binding-20260924/build_probe.py
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=214 \
  /home/ubuntu/workcell_ws/stage-a1-start-binding-20260924/runtime_mimic_probe
```

- `runtime_mimic_probe.cpp`: installed-library tests; no server/world step/trajectory.
- `build-command.json`: actual compiler/link invocation.
- `runtime-tests.txt` / `.xml`: fresh test results.
- `physical_residuals.json`: exact raw residuals and measured epoch.
- `recent_measurements.json`: untouched saved trace excerpt showing evolving residuals.
- `cpp-regression.txt` / `.xml`, `python-regression.txt` / `.xml`: existing regression gates.
- `report.json`: final test counts, hashes, runtime sources, and stopped decision.

**First remaining blocker:** a conservative physical follower-response enclosure from the complete raw measured state through the first JTC interval. Stop here; no withdrawal qualification is claimed.
