from __future__ import annotations

import json
from pathlib import Path

import backend
import streamlit as st

st.set_page_config(page_title="Workcell Studio Streamlit", layout="wide")
st.title("Workcell Studio Streamlit Prototype")
st.warning("Workcell Studio defaults to offline validation and fake hardware. This UI does not command robot motion.")

workflow = st.sidebar.radio(
    "Workflow",
    ["Catalog browser", "Cell definition validator", "Builder scene import"],
)

if workflow == "Catalog browser":
    st.header("Catalog browser")
    if st.button("Load catalog metadata"):
        caps = backend.load_capability_catalog()
        grasps = backend.load_grasp_strategy_catalog()
        for key in ["robots", "end_effectors", "sensors", "environment_assets", "tasks"]:
            st.subheader(key.replace("_", " ").title())
            st.dataframe(caps.get(key, []), use_container_width=True)
        st.subheader("Grasp strategies")
        st.dataframe(grasps, use_container_width=True)

elif workflow == "Cell definition validator":
    st.header("Cell definition validator")
    cell_def_path = st.text_input("Path to cell_definition.yaml", value="tests/fixtures/cell_definition_pick_place.yaml")
    env_layout_path = st.text_input("Optional path to environment_layout.yaml", value="")

    if st.button("Run validation"):
        result = backend.validate_cell_definition(cell_def_path)
        st.subheader("Cell definition status")
        st.json(result.get("json") or result)
        st.code((result.get("stdout") or "") + "\n" + (result.get("stderr") or ""))

        if env_layout_path.strip():
            layout_result = backend.validate_environment_layout(env_layout_path)
            st.subheader("Environment layout status")
            st.json(layout_result.get("json") or layout_result)
            st.code((layout_result.get("stdout") or "") + "\n" + (layout_result.get("stderr") or ""))

else:
    st.header("Builder scene import")
    scene_package = st.text_input("Scene package path")
    output_dir = st.text_input("Output directory", value="/tmp/workcell_studio_streamlit")
    project_name = st.text_input("Project name", value="streamlit_demo")

    if st.button("Import builder scene"):
        result = backend.import_builder_scene(scene_package, output_dir, project_name)
        if not result.get("ok"):
            st.error(result.get("error") or result.get("stderr") or "Import failed")
        st.subheader("Command output")
        st.code((result.get("stdout") or "") + "\n" + (result.get("stderr") or ""))

        summary_block = (result.get("summary") or {})
        st.subheader("Import summary JSON")
        st.json(summary_block.get("summary", {}))

        md_path = summary_block.get("summary_markdown_path")
        if md_path and Path(md_path).exists():
            st.subheader("Import summary Markdown")
            st.markdown(Path(md_path).read_text(encoding="utf-8"))

        summary = summary_block.get("summary", {})
        st.subheader("Key status")
        st.write(
            {
                "generated_project_path": summary.get("generated_project_path"),
                "dashboard_path": summary.get("dashboard_path"),
                "preflight_report_path": summary.get("preflight_report_path"),
                "safety_status": summary.get("safety_status"),
                "runtime_preview_blockers": (summary.get("validation", {}).get("builder_scene", {}).get("runtime_blockers", [])),
            }
        )
