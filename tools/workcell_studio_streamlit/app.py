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
    st.info("This defines task intent only. It does not command robot motion.")
    scene_package = st.text_input("Scene package path", value="scenes/ur5_2f_test")
    output_path = st.text_input("Task intent output path", value=str(Path(scene_package) / "generated" / "workcell_builder_task_intent.yaml") if scene_package else "")
    if st.button("Discover scene targets"):
        st.session_state["builder_targets"] = backend.list_builder_scene_authoring_targets(scene_package).get("json", {})
    st.subheader("Pick/Place Zones")
    st.caption("Pick/place zones are metadata only. Saving zones does not command robot motion.")
    env_path = st.text_input("environment_layout.yaml path", value=str(Path(scene_package)/"generated"/"environment_layout.yaml"))
    env_layout = backend.load_environment_layout(env_path)
    env_targets = backend.list_environment_targets(env_layout)
    ids = [t.get("id") for t in env_targets if t.get("id")]
    selected_id = st.selectbox("Target selector", options=["<create new>"] + ids)
    current = next((t for t in env_targets if t.get("id") == selected_id), {})
    pose = current.get("pose", {}) if isinstance(current, dict) else {}
    xyz = pose.get("xyz") if isinstance(pose.get("xyz"), list) else [0.45, 0.0, 0.08]
    rpy = pose.get("rpy") if isinstance(pose.get("rpy"), list) else [0.0, 0.0, 0.0]
    size = current.get("size") if isinstance(current.get("size"), list) else [0.3, 0.2, 0.1]
    zc1, zc2, zc3 = st.columns(3)
    zid = zc1.text_input("target id", value=(current.get("id") if current else "pick_zone_main"))
    ztype = zc2.selectbox("target type", options=["pick_zone","place_target","bin"], index=["pick_zone","place_target","bin"].index(current.get("type")) if current and current.get("type") in ["pick_zone","place_target","bin"] else 0)
    zlabel = zc3.text_input("label", value=(current.get("label") if current else "Main pick zone"))
    frame = st.text_input("frame", value=(pose.get("frame") if isinstance(pose, dict) and pose.get("frame") else "world"))
    cxyz = st.columns(3); x = cxyz[0].number_input("x", value=float(xyz[0]), step=0.01); y = cxyz[1].number_input("y", value=float(xyz[1]), step=0.01); z = cxyz[2].number_input("z", value=float(xyz[2]), step=0.01)
    crpy = st.columns(3); rr = crpy[0].number_input("roll", value=float(rpy[0]), step=0.01); pp = crpy[1].number_input("pitch", value=float(rpy[1]), step=0.01); yy = crpy[2].number_input("yaw", value=float(rpy[2]), step=0.01)
    csize = st.columns(3); sx = csize[0].number_input("size x", min_value=0.01, value=float(size[0]), step=0.01); sy = csize[1].number_input("size y", min_value=0.01, value=float(size[1]), step=0.01); sz = csize[2].number_input("size z", min_value=0.01, value=float(size[2]), step=0.01)
    svg = backend.render_topdown_targets_svg(env_targets)
    st.components.v1.html(svg, height=530, scrolling=False)
    zb1, zb2, zb3, zb4 = st.columns(4)
    if zb1.button("Save/update target"):
        st.json(backend.create_or_update_environment_target(env_path, zid, ztype, zlabel, frame, [x,y,z], [rr,pp,yy], [sx,sy,sz], output_path=env_path).get("json") or {})
        env_layout = backend.load_environment_layout(env_path)
        env_targets = backend.list_environment_targets(env_layout)
        st.components.v1.html(backend.render_topdown_targets_svg(env_targets), height=530, scrolling=False)
    if zb2.button("Refresh discovered targets"):
        st.session_state["builder_targets"] = backend.list_builder_scene_authoring_targets(scene_package).get("json", {})
    if zb3.button("Generate static preview"):
        preview_dir = str(Path(scene_package) / "generated" / "static_preview")
        cell_path = str(Path(scene_package) / "generated" / "cell_definition.yaml")
        st.json(backend.generate_static_preview_with_task_flow(cell_path, preview_dir, "Builder Task Intent Preview", task_intent_path=output_path, environment_layout_path=env_path).get("json") or {})
        st.caption(f"Preview HTML: {Path(preview_dir) / 'static_preview.html'}")
    if zb4.button("Generate readiness pack"):
        st.json(backend.generate_readiness_pack(scene_package, "/tmp/workcell_readiness_pack", "builder_intent_demo", validate=True, prepare_rviz_preview=False, smoke_dry_run=True).get("json") or {})
    targets = st.session_state.get("builder_targets", {})
    st.json(targets)
    grasps = [x["id"] for x in backend.resolve_catalog_choices().get("grasp_strategies", [])]
    discovered = backend.summarize_environment_targets(env_path) if Path(env_path).exists() else {"pick_sources": [], "place_targets": []}
    pick_opts = discovered.get("pick_sources") or targets.get("pick_sources") or ["pick_zone_main"]
    place_opts = discovered.get("place_targets") or targets.get("place_targets") or ["bin_main"]
    task_id = st.text_input("Task id", value="sorting_task_001")
    task_type = st.text_input("Task type", value="pick_place")
    pick_source = st.selectbox("Pick source", options=pick_opts)
    place_target = st.selectbox("Place target", options=place_opts)
    grasp = st.selectbox("Grasp strategy", options=grasps or ["finger_pinch_basic"])
    object_class = st.text_input("Object class", value="any")
    object_color = st.text_input("Object color", value="red")
    approach_axis = st.text_input("Approach axis", value="z_down")
    approach_dist = st.number_input("Approach distance (m)", value=0.12)
    retreat_axis = st.text_input("Retreat axis", value="z_up")
    retreat_dist = st.number_input("Retreat distance (m)", value=0.10)
    release_strategy = st.text_input("Release strategy", value="tool_release")
    c1,c2,c3,c4 = st.columns(4)
    if c1.button("Save task intent"):
        st.json(backend.create_or_update_builder_task_intent(scene_package, task_id, task_type, pick_source, place_target, grasp, output_path=output_path, approach_axis=approach_axis, approach_distance_m=approach_dist, retreat_axis=retreat_axis, retreat_distance_m=retreat_dist, release_strategy=release_strategy, object_class=object_class, object_color=object_color, validate=False).get("json") or {})
    if discovered.get("pick_sources") and pick_source not in discovered.get("pick_sources", []):
        st.warning(f"Selected pick source '{pick_source}' is not present in environment_layout.yaml")
    if discovered.get("place_targets") and place_target not in discovered.get("place_targets", []):
        st.warning(f"Selected place target '{place_target}' is not present in environment_layout.yaml")
    if c2.button("Validate task intent"):
        st.json(backend.validate_builder_task_intent(output_path, scene_package).get("json") or {})
    if c3.button("Generate task recipe"):
        out_recipe = str(Path(scene_package)/"generated"/"task_recipe_from_builder_intent.yaml") if scene_package else "/tmp/task_recipe_from_builder_intent.yaml"
        st.json(backend.convert_builder_task_intent_to_task_recipe(output_path, out_recipe, scene_package).get("json") or {})
    if c4.button("Generate readiness pack"):
        st.json(backend.generate_readiness_pack(scene_package, "/tmp/workcell_readiness_pack", "builder_intent_demo", validate=True, prepare_rviz_preview=False, smoke_dry_run=True).get("json") or {})

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
        dash_path = backend.dashboard_path_from_manifest(m)
        if dash_path:
            st.write({'dashboard_path': dash_path})
        if st.button('Generate dashboard'):
            target = Path(out)/'readiness_dashboard.html'
            st.json(backend.generate_readiness_dashboard(Path(out)/'readiness_pack_manifest.json', target).get('json') or {})
            dash_path = str(target)
        with st.expander('Preview dashboard HTML'):
            st.warning('Dashboard is a review artifact only. It does not execute commands.')
            if dash_path and Path(dash_path).exists():
                html_text = backend.read_readiness_dashboard(dash_path)
                try:
                    import streamlit.components.v1 as components
                    components.html(html_text, height=500, scrolling=True)
                except Exception:
                    st.code(html_text[:3000])
                st.caption(dash_path)
            else:
                st.caption('No dashboard file available yet.')
