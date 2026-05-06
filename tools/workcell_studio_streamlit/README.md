# Workcell Studio Streamlit Prototype

This prototype provides a lightweight orchestration/readiness/demo UI over existing Workcell Studio scripts.

## Safety notes

Workcell Studio defaults to offline validation and fake hardware. This UI does not command robot motion.
It does not run launch files and is not a replacement for runtime execution flows.

## Install

```bash
python3 -m pip install -r tools/workcell_studio_streamlit/requirements.txt
```

## Run

```bash
streamlit run tools/workcell_studio_streamlit/app.py
```

## Example builder import flow

1. Open **Builder scene import**.
2. Set scene package path, output directory, and project name.
3. Click **Import builder scene** to run:
   `python3 scripts/workcell_studio.py import-builder-scene --scene-package <scene-package> --output-dir <output-dir> --project-name <project-name> --validate --generate-project`
4. Review import summary JSON/Markdown plus safety/readiness fields.

`workcell_builder` remains the visual editor.
This Streamlit Studio prototype is currently an orchestration/readiness/demo UI.

## Demo Gallery

Open the Streamlit app and select **Demo Gallery**.

Generate one demo bundle:
- choose a demo id
- choose output directory
- click **Generate demo bundle**

Generate all demo bundles:
- click **Generate all demo bundles**

Safety notes:
- UI is offline-first and fake-hardware-first.
- No robot motion commands are issued.
- Runtime modes:
  - `runtime_ready`: intended runtime-capable configuration.
  - `fake_hardware_ready`: safe demo/validation default.
  - `preview_only`: concept/sales visualization only.

## Static preview artifacts

Demo Gallery and Builder Scene Import now surface static preview artifacts when generated. These previews are offline approximations for quick review only; RViz/MoveIt remains the real planning/visualisation path.

## Create Cell flow

The Streamlit app now includes **Create Cell** for selecting catalog robot/tool/sensor/task/grasp strategy and generating `cell_definition.yaml`, `environment_layout.yaml`, summary files, static preview, and optional bundle outputs. Incompatible combos can be marked preview-only with explicit warnings.
