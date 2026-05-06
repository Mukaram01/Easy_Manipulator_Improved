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
    ["Catalog browser", "Cell definition validator", "Builder scene import", "Demo Gallery", "Create Cell"],
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

elif workflow == "Builder scene import":
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
                "preview_svg": summary.get("preview_svg"),
                "preview_html": summary.get("preview_html"),
                "dashboard_path": summary.get("dashboard_path"),
                "preflight_report_path": summary.get("preflight_report_path"),
                "safety_status": summary.get("safety_status"),
                "runtime_preview_blockers": (summary.get("validation", {}).get("builder_scene", {}).get("runtime_blockers", [])),
            }
        )
        if summary.get("preview_svg") and Path(summary["preview_svg"]).exists():
            st.subheader("Static Preview")
            st.image(summary["preview_svg"])
        if summary.get("preview_warnings"):
            st.warning("\n".join(summary.get("preview_warnings", [])))


if workflow == "Demo Gallery":
    st.header("Demo Gallery")
    st.info("Preview-only demos are concept/demo artifacts only and are not runtime-ready.")
    catalog = backend.load_demo_catalog()
    demos = catalog.get("demos", [])
    st.caption(f"Catalog: {catalog.get('catalog_path','')}")
    st.dataframe(demos, use_container_width=True)
    demo_ids = [d.get("id") for d in demos if isinstance(d, dict) and d.get("id")]
    selected_demo = st.selectbox("Select demo", options=demo_ids) if demo_ids else ""
    output_dir = st.text_input("Output directory", value="/tmp/workcell_studio_demos")
    col1, col2 = st.columns(2)
    if col1.button("Generate demo bundle") and selected_demo:
        result = backend.generate_demo_bundle(output_dir=output_dir, demo_id=selected_demo, all_demos=False, force=True)
        st.json(result.get("json") or result)
        summary = backend.load_demo_bundle_summary(Path(output_dir) / selected_demo)
        st.subheader("Demo bundle summary")
        st.json(summary.get("summary", {}))
        if summary.get("markdown"):
            st.markdown(summary["markdown"])
        payload = summary.get("summary", {})
        if payload.get("preview_svg") and Path(payload["preview_svg"]).exists():
            st.subheader("Static Preview")
            st.image(payload["preview_svg"])
        if payload.get("preview_warnings"):
            st.warning("\n".join(payload.get("preview_warnings", [])))
    if col2.button("Generate all demo bundles"):
        result = backend.generate_demo_bundle(output_dir=output_dir, all_demos=True, force=True, continue_on_error=True)
        st.json(result.get("json") or result)

if workflow == "Create Cell":
    st.header("Create Cell")
    choices = backend.resolve_catalog_choices()
    robot = st.selectbox("Robot", [x["id"] for x in choices["robots"]])
    ee = st.selectbox("End effector", [x["id"] for x in choices["end_effectors"]])
    sensor = st.selectbox("Sensor", [x["id"] for x in choices["sensors"]])
    task = st.selectbox("Task", [x["id"] for x in choices["tasks"]])
    grasp = st.selectbox("Grasp strategy", [x["id"] for x in choices["grasp_strategies"]])
    cell_id = st.text_input("Cell ID", value="my_first_cell")
    output_dir = st.text_input("Output directory", value="/tmp/workcell_studio_create_cell")
    validate = st.checkbox("Validate", value=True)
    preview = st.checkbox("Generate static preview", value=True)
    bundle = st.checkbox("Generate project/demo bundle", value=False)
    allow = st.checkbox("Allow incompatible as preview-only", value=False)
    if st.button("Create cell"):
        result = backend.create_cell(cell_id, robot, ee, sensor, task, grasp, output_dir, validate=validate, preview=preview, generate_bundle=bundle, allow_incompatible=allow, force=True)
        st.json(result.get("json") or result)
        summary = result.get("summary", {})
        st.subheader("Create Cell Summary")
        st.json(summary.get("summary", {}))
        if summary.get("markdown"):
            st.markdown(summary["markdown"])
