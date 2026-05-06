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
