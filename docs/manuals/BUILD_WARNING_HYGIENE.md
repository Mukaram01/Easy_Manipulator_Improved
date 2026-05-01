# Build warning hygiene for developer colcon workspace

## Observed warnings
- `osqp_eigen` CMake developer warning: `OSQP_EIGEN_OSQP_TARGET_TO_LINK` cannot be set in parent scope.
- Tesseract overlays (`tesseract_geometry`, `tesseract_urdf`, `tesseract_examples`) surface Assimp's deprecated `pbrmaterial.h` warning.
- `run_grasp_execution` showed Boost.Bind global placeholders deprecation notes.

## Owned fix in this repo
`run_grasp_execution` now applies `BOOST_BIND_GLOBAL_PLACEHOLDERS` as a target-level compile definition for local targets (including `demo_node` and `test_explicit_release_pose_utils`). This removes the local Boost.Bind deprecation note without weakening standard warning flags.

## Known third-party warnings
`osqp_eigen` and Tesseract/Assimp warnings are classified as known third-party warnings. We do not patch third-party source overlays in this repository to keep vendor boundaries clean and upgrades simpler.

## Developer clean build workflow
Run:

```bash
scripts/build_workspace_dev_clean.sh
```

The script:
1. Runs a colcon build with `colcon/quiet_third_party_warnings.meta`.
2. Writes a timestamped log under `logs/colcon_builds/`.
3. Runs `scripts/check_colcon_build_warnings.py --fail-on-owned` on that log.

The workflow fails on owned or unknown warnings, but not on classified known third-party warnings.

## Manual log inspection
Inspect the latest log manually with tools like:

```bash
tail -n 200 logs/colcon_builds/<logfile>.log
rg -n "warning|note|deprecated|OSQP_EIGEN_OSQP_TARGET_TO_LINK|pbrmaterial" logs/colcon_builds/<logfile>.log
```

## Important note
This hygiene flow helps triage and noise-reduction; it is **not** a replacement for fixing legitimate project warnings.
