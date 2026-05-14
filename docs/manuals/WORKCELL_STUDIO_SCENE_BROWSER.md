# Workcell Studio Scene Browser

The Workcell Studio Qt shell now scans scene packages from `<workspace>/scenes` and renders real status in Dashboard, Existing Scenes, Scene Builder preview, inspector, and readiness/log sections.

## Scanned artifacts
- `environment.yaml`, `package.xml`, `launch/demo.launch.py`
- `urdf/scene.urdf.xacro` or `urdf/environment.urdf.xacro`
- `urdf/arm_hand.srdf.xacro`
- `config/task_recipe.yaml`, `config/workcell_builder_task_intent.yaml`
- `smoke/offline_smoke_report.json`, `smoke/offline_smoke_report.html`
- `preview/static_preview.svg`, `preview/static_preview.html`
- `scene_manifest.yaml`

## Status assignment
- `READY`: core package files + launch + URDF/SRDF + smoke/task artifacts present.
- `WARNINGS`: parse warnings or missing non-critical artifacts.
- `BLOCKED`: key files missing.
- `SCAFFOLD_ONLY`: environment exists but generated package assets are missing.
- `MISSING_ENVIRONMENT_YAML`: no `environment.yaml`.
- `MISSING_LAUNCH`: no `launch/demo.launch.py`.
- `PREVIEW_ONLY`: preview exists but launch is missing.

## Using preview and smoke reports
Use **Open Preview** and **Open Smoke Report** in Existing Scenes / Scene Builder context. Missing files report a clear message instead of failing silently.

## Safety
This browser is preview/readiness only:
- No robot motion commanded.
- No MoveIt runtime execution from browser actions.
- Launch copy uses fake hardware default:
  `ros2 launch <scene_name> demo.launch.py use_fake_hardware:=true`

EPD remains separate.
