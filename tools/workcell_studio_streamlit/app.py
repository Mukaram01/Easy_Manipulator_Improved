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
    ["Catalog browser", "Cell definition validator", "Builder scene import", "Builder Task Intent", "Demo Gallery", "RViz Plan Preview", "Planning Scene Readiness", "Create Cell", "Readiness Pack"],
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
                "generated_task_recipe_path": summary.get("generated_task_recipe_path"),
                "readiness": (summary.get("validation", {}).get("builder_scene", {}).get("readiness")),
            }
        )
        if summary.get("preview_svg") and Path(summary["preview_svg"]).exists():
            st.subheader("Static Preview")
            st.image(summary["preview_svg"])
        if summary.get("preview_warnings"):
            st.warning("\n".join(summary.get("preview_warnings", [])))



elif workflow == "Builder Task Intent":
    st.header("Builder Task Intent")
    st.info("This defines robot task intent only. It does not command robot motion.")
    scene_package = st.text_input("Scene package path", value="")
    found = backend.find_builder_task_intent(scene_package) if scene_package else ""
    task_path = st.text_input("Task intent YAML path", value=found or (str(Path(scene_package)/"generated"/"workcell_builder_task_intent.yaml") if scene_package else "workcell_builder_task_intent.yaml"))
    if st.button("Load task intent"):
        st.session_state["builder_task_intent"] = backend.load_builder_task_intent(task_path)
    if st.button("Create default task intent"):
        st.session_state["builder_task_intent"] = backend.default_builder_task_intent(Path(scene_package).name if scene_package else "")
    data = st.session_state.get("builder_task_intent", backend.default_builder_task_intent())
    data.setdefault("task", {})["id"] = st.text_input("Task ID", value=data.get("task", {}).get("id", "default_builder_task"))
    data["task"]["type"] = st.text_input("Task type", value=data.get("task", {}).get("type", "pick_place"))
    pick = data.setdefault("pick", {}).setdefault("source", {})
    pick["id"] = st.text_input("Pick source ID", value=pick.get("id", "pick_zone_main"))
    of = data.setdefault("pick", {}).setdefault("object_filter", {})
    of["class_id"] = st.text_input("Object class", value=of.get("class_id", "any"))
    of["color"] = st.text_input("Object color", value=of.get("color", "any"))
    grasps = [x["id"] for x in backend.resolve_catalog_choices().get("grasp_strategies", [])]
    g = data.setdefault("grasp", {})
    g["strategy_ref"] = st.selectbox("Grasp strategy", options=grasps, index=max(0, grasps.index(g.get("strategy_ref")) if g.get("strategy_ref") in grasps else 0)) if grasps else st.text_input("Grasp strategy", value=g.get("strategy_ref", "suction_top_basic"))
    g["approach_axis"] = st.text_input("Approach axis", value=g.get("approach_axis", "z_down"))
    g["approach_distance_m"] = st.number_input("Approach distance (m)", value=float(g.get("approach_distance_m", 0.1)))
    g["retreat_axis"] = st.text_input("Retreat axis", value=g.get("retreat_axis", "z_up"))
    g["retreat_distance_m"] = st.number_input("Retreat distance (m)", value=float(g.get("retreat_distance_m", 0.1)))
    place = data.setdefault("place", {}).setdefault("target", {})
    place["id"] = st.text_input("Place target ID", value=place.get("id", "bin_main"))
    data.setdefault("place", {})["release_strategy"] = st.text_input("Release strategy", value=data.get("place", {}).get("release_strategy", "tool_release"))
    if st.button("Save task intent"):
        backend.save_builder_task_intent(task_path, data)
        st.success(f"Saved: {task_path}")
    if st.button("Validate task intent"):
        result = backend.validate_builder_task_intent(task_path, scene_package if scene_package else None)
        st.json(result.get("json") or result)
    st.warning("This generates offline task recipe metadata only. It does not command robot motion.")
    if st.button("Generate task recipe"):
        out_recipe = str(Path(scene_package) / "generated" / "task_recipe_from_builder_intent.yaml") if scene_package else "/tmp/task_recipe_from_builder_intent.yaml"
        result = backend.convert_builder_task_intent_to_task_recipe(task_path, out_recipe, scene_package if scene_package else None)
        st.json(result.get("json") or result)
        st.json(backend.summarize_task_recipe(out_recipe))
    st.subheader("Safety metadata")
    st.json(data.get("safety", {}))

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

if workflow == "RViz Plan Preview":
    st.header("RViz/MoveIt Plan Preview")
    session_path = st.text_input("Session JSON path", value="/tmp/rviz_plan_preview_session/rviz_moveit_plan_preview_session.json")
    smoke_output = st.text_input("Smoke output dir", value="/tmp/fake_hardware_smoke")
    timeout_s = st.number_input("Timeout seconds", min_value=1, value=20)
    c1, c2 = st.columns(2)
    if c1.button("Dry-run smoke check"):
        result = backend.smoke_launch_preview(session_path, smoke_output, execute=False, timeout_s=int(timeout_s))
        st.json(result.get("json") or result)
    st.warning("This starts a fake-hardware ROS launch for a short timeout. It must not be used with real hardware.")
    confirm = st.checkbox("I understand this is fake-hardware-only and no robot motion will be commanded.")
    if c2.button("Execute fake-hardware smoke launch", disabled=not confirm):
        result = backend.smoke_launch_preview(session_path, smoke_output, execute=True, timeout_s=int(timeout_s))
        st.json(result.get("json") or result)
    report = backend.load_smoke_launch_report(smoke_output)
    if report:
        st.subheader("Smoke report")
        st.json({"status": (report.get("result") or {}).get("status"), "warnings": (report.get("result") or {}).get("warnings", []), "errors": (report.get("result") or {}).get("errors", []), "safety": report.get("safety", {})})
        logs = backend.read_smoke_launch_logs(smoke_output)
        st.code("STDOUT:\n" + logs.get("stdout", ""))
        st.code("STDERR:\n" + logs.get("stderr", ""))


if workflow == "Planning Scene Readiness":
    st.header("Planning Scene Readiness")
    st.warning("This is file/metadata readiness only. It does not call MoveIt, start ROS, or move the robot.")
    scene = st.text_input("Scene package path", value="scenes/ur5_2f_test")
    out = st.text_input("Output dir", value="/tmp/planning_scene_readiness")
    cell = st.text_input("Cell definition path (optional)", value="")
    recipe = st.text_input("Task recipe path (optional)", value="")
    req = st.text_input("Offline plan preview request path (optional)", value="")
    sess = st.text_input("RViz plan preview session path (optional)", value="")
    smoke = st.text_input("Smoke report path (optional)", value="")
    strict = st.checkbox("Strict mode", value=False)
    c1,c2=st.columns(2)
    if c1.button("Check planning-scene readiness"):
        result = backend.check_planning_scene_readiness(scene, out, cell_definition=cell or None, task_recipe=recipe or None, plan_preview_request=req or None, plan_preview_session=sess or None, smoke_report=smoke or None, strict=strict)
        st.json(result.get("json") or result)
    if c2.button("Validate readiness report"):
        result = backend.validate_planning_scene_readiness_report(Path(out)/"planning_scene_readiness_report.json")
        st.json(result.get("json") or result)
    loaded = backend.load_planning_scene_readiness_report(out)
    rep = loaded.get("report", {})
    if rep:
        st.subheader("Readiness")
        st.write({"readiness": ((rep.get("result") or {}).get("readiness")), "classification": ((rep.get("result") or {}).get("classification")), "blockers": ((rep.get("result") or {}).get("blockers")), "warnings": ((rep.get("result") or {}).get("warnings")), "suggested_next_actions": ((rep.get("result") or {}).get("suggested_next_actions"))})
        st.subheader("Key detected fields")
        checks = rep.get("checks") or {}
        st.write({"launch_file_detected": ((checks.get("scene_package") or {}).get("launch_file_exists")), "robot_detected": ((checks.get("robot") or {}).get("robot_model_detected")), "tool_detected": ((checks.get("tool") or {}).get("end_effector_detected")), "pick_place_grasp": {"pick": ((checks.get("task") or {}).get("pick_source_id")), "place": ((checks.get("task") or {}).get("place_target_id")), "grasp": ((checks.get("task") or {}).get("grasp_strategy"))}, "waypoint_count": ((checks.get("task") or {}).get("waypoint_count")), "plan_preview_session_status": ((checks.get("planning") or {}).get("rviz_expected")), "smoke_report_status": ((checks.get("smoke") or {}).get("smoke_status"))})
        st.subheader("Safety flags")
        st.json((checks.get("safety") or {}))

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


if workflow == "Readiness Pack":
    st.header("Readiness Pack")
    st.warning("This is an offline/fake-hardware readiness pack. It does not command robot motion or call MoveIt planning services.")
    scene = st.text_input("Scene package path", value="scenes/ur5_2f_test")
    out = st.text_input("Output dir", value="/tmp/workcell_readiness_pack")
    project = st.text_input("Project name", value="demo_cell")
    validate = st.checkbox("Validate", value=True)
    prep = st.checkbox("Prepare RViz/MoveIt preview", value=True)
    dry = st.checkbox("Smoke dry-run", value=True)
    strict = st.checkbox("Strict", value=False)
    coe = st.checkbox("Continue on error", value=False)
    smoke_exec = st.checkbox("Enable smoke execute (guarded)", value=False)
    confirm = st.checkbox("I confirm fake-hardware-only smoke execute") if smoke_exec else False
    c1,c2=st.columns(2)
    if c1.button("Generate readiness pack"):
        res = backend.generate_readiness_pack(scene, out, project, validate=validate, prepare_rviz_preview=prep, smoke_dry_run=dry, strict=strict, continue_on_error=coe, smoke_execute=(smoke_exec and confirm))
        st.json(res.get("json") or res)
    if c2.button("Validate readiness pack"):
        st.json(backend.validate_readiness_pack(Path(out)/"readiness_pack_manifest.json").get("json") or {})
    m = backend.load_readiness_pack_manifest(out)
    if m:
        st.write({"final_readiness": m.get("results",{}).get("final_readiness"), "classification": m.get("results",{}).get("classification"), "blockers": m.get("summary",{}).get("blockers",[]), "warnings": m.get("summary",{}).get("warnings",[]), "suggested_next_actions": m.get("summary",{}).get("suggested_next_actions",[])})
        st.json(m.get("artifacts",{}))
        st.markdown(backend.read_readiness_pack_summary(out))
        st.code(backend.read_readiness_pack_next_commands(out))
