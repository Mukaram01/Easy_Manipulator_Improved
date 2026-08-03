from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEWER = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
INDEX = (ROOT / "workcell_studio_web/viewer/index.html").read_text(encoding="utf-8")
STYLE = (ROOT / "workcell_studio_web/viewer/style.css").read_text(encoding="utf-8")
CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")
HDR = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.h").read_text(encoding="utf-8")
VIEWPORT_CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp").read_text(encoding="utf-8")


def test_browser_editor_api_v1_contract_and_bounded_events():
    assert "window.__WORKCELL_EDITOR_API_V1__" in VIEWER
    for method in ["getState", "selectItem", "clearSelection", "setMode", "setSnap", "undo", "redo", "fitScene", "getEditPatch", "drainEvents"]:
        assert f"{method}:" in VIEWER
    for field in ["ready", "sceneId", "selectedItemId", "uiSelectionItemId", "selectedItemType", "selectedEditable", "dirty", "dirtyCount", "canUndo", "canRedo", "mode", "error"]:
        assert field in VIEWER
    for event in ["selection_changed", "dirty_changed", "transform_committed", "editor_error"]:
        assert event in VIEWER
    assert "state.editorEvents.length > 100" in VIEWER
    assert "splice(0, state.editorEvents.length - 100)" in VIEWER


def test_browser_api_reuses_existing_editor_functions_and_preserves_locks():
    for token in ["selectObject(String(id || ''))", "refreshGizmoSnap()", "undoPreviewEdit()", "redoPreviewEdit()", "resetView()", "buildEditPatch()"]:
        assert token in VIEWER
    assert "function canEditItem(item)" in VIEWER
    assert "source.includes('generated')" in VIEWER
    assert "item?.locked || item?.editable !== true" in VIEWER
    assert "emitTransformCommitted(rendered)" in VIEWER
    assert "dragging-changed" in VIEWER


def test_embedded_mode_hides_standalone_browser_ui_only_when_requested():
    assert "params.get('embedded') === '1'" in VIEWER
    assert "document.body.classList.add('embedded-mode')" in VIEWER
    assert "body.embedded-mode .topbar" in STYLE
    assert "body.embedded-mode .object-panel" in STYLE
    assert "body.embedded-mode .details-panel" in STYLE
    assert "body.embedded-mode .toolbar" in STYLE
    assert "body.embedded-mode .app-shell" in STYLE
    assert "class=\"topbar\"" in INDEX


def test_qt_loads_embedded_view_and_routes_controls_without_reload():
    assert 'viewer_query.addQueryItem(QStringLiteral("embedded"), QStringLiteral("1"))' in CPP
    for helper in ["run_embedded_editor_command", "poll_embedded_editor_events", "apply_embedded_editor_state", "embedded_snap_command"]:
        assert helper in CPP and helper in HDR
    for command in ["setMode", "setSnap", "fitScene", "undo()", "redo()", "selectItem"]:
        assert command in CPP
    assert "QTimer::singleShot(200, this, [this, identity]()" in CPP
    assert "refresh_embedded_web_product_view();" not in CPP.split('connect(gizmo_mode_selector_', 1)[1].split('connect(mesh_preview_mode_selector_', 1)[0]


def test_outer_qt_authoring_controls_use_typed_web3d_bridge():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    for method in ["set_authoring_mode", "undo_authoring_edit", "redo_authoring_edit", "request_authoring_save"]:
        assert method in HDR
        assert f"scene_preview_widget_->{method}" in main
    assert "authoring_mode_changed" in CPP
    assert "authoring_mode_changed" in main
    assert 'make_primary_button("Rotate")' in main


def test_authoring_mode_round_trip_cleanup_and_selection_preservation():
    mode_body = VIEWER.split("function setEditorMode", 1)[1].split("function setEditorSnap", 1)[0]
    for mode in ["'move'", "'rotate'", "'select'"]:
        assert mode in mode_body
    assert "cancelDirectMoveDrag('Move cancelled')" in mode_body
    assert "cancelDirectRotateDrag('Rotation cancelled')" in mode_body
    assert "gizmo.reset?.()" in mode_body
    assert "gizmo.detach()" in mode_body
    assert "state.three.controls.enabled = true" in mode_body
    assert "state.selected =" not in mode_body
    assert "return state.editorMode" in mode_body


def test_browser_mode_state_synchronizes_all_qt_controls_without_stale_callbacks():
    apply_body = CPP.split("void ScenePreviewWidget::apply_embedded_editor_state", 1)[1].split("void ScenePreviewWidget::poll_embedded_editor_events", 1)[0]
    assert "QSignalBlocker blocker(gizmo_mode_selector_)" in apply_body
    assert "QSignalBlocker blocker(interaction_mode_selector_)" in apply_body
    assert "emit authoring_mode_changed(mode)" in apply_body
    assert "state_request_token != embedded_editor_state_request_token_" in CPP
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    for control in ["select_mode_button", "move_mode_button", "rotate_mode_button"]:
        assert f"QSignalBlocker {control.removesuffix('_mode_button')}_blocker({control})" in main


def test_move_mode_first_click_selects_and_starts_drag_without_bypassing_locks():
    handler = VIEWER.split("function onCanvasPointerDown", 1)[1].split("function onCanvasPointerMove", 1)[0]
    assert "const rendered = hitId ? renderedById(hitId) : null" in handler
    assert "hitId === state.selected" not in handler
    assert "beginDirectMoveDrag(event, rendered)" in handler
    assert "!selectionIsEditable(rendered)" in VIEWER.split("function beginDirectMoveDrag", 1)[1].split("function updateDirectMoveDrag", 1)[0]


def test_qt_compact_toolbar_for_embedded_web3d():
    for member in ["embedded_undo_button_", "embedded_redo_button_", "embedded_fit_button_"]:
        assert member in CPP and member in HDR
    assert "set_visible(view_actions_label_, !embedded_web_active)" in CPP
    assert "set_visible(view_actions_selector_, !embedded_web_active)" in CPP
    assert "set_visible(labels_label_, !embedded_web_active)" in CPP
    assert "set_visible(labels_selector_, !embedded_web_active)" in CPP
    assert "set_visible(mesh_preview_mode_label_, !embedded_web_active)" in CPP
    assert "set_visible(mesh_preview_mode_selector_, !embedded_web_active)" in CPP
    assert "set_visible(interaction_mode_label_, !embedded_web_active)" in CPP
    assert "set_visible(interaction_mode_selector_, !embedded_web_active)" in CPP
    assert "set_visible(overlays_selector_, !embedded_web_active)" in CPP
    assert "set_visible(gizmo_mode_label_, embedded_web_active)" in CPP
    assert "set_visible(gizmo_mode_selector_, embedded_web_active)" in CPP
    assert "set_visible(snap_mode_selector_, embedded_web_active)" in CPP
    assert 'state.value(QStringLiteral("canUndo")).toBool()' in CPP
    assert 'state.value(QStringLiteral("canRedo")).toBool()' in CPP


def test_product_view_selection_uses_stable_identity_and_filters_helpers():
    assert "function isExpandedUrdfInspectionPick(rendered)" in VIEWER
    assert "function isCanvasSelectableRendered(rendered)" in VIEWER
    assert "state.pickRecords.includes(rendered)" in VIEWER
    assert "rendered?.authoritativePhysicalPick === true" in VIEWER
    assert "function isNormalSelectableRendered(rendered)" in VIEWER
    assert "item.selectable !== false" in VIEWER
    assert "state.debugOverlaysVisible && (isTaskOnlyHelperItem(item)" in VIEWER
    assert "!isTaskOnlyHelperItem(item) && !isOverlayPolicyItem(item)" in VIEWER
    assert "!isDebugOverlayItem(item)" in VIEWER
    assert "renderedById(requestedId)" in VIEWER
    assert "missing_render_identity" in VIEWER
    assert "diagnostic_helper_or_non_selectable" in VIEWER
    assert "canonicalSelectionRendered" in VIEWER
    assert "selectObjectFromRender(canonicalId, selectedCandidate.rendered);" in VIEWER
    assert "itemLabel(item)" not in VIEWER.split("function pickObject", 1)[1].split("function beginDirectMoveDrag", 1)[0]


def test_expanded_urdf_inspection_selection_policy_executes_in_browser_harness(tmp_path):
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',className:'',innerHTML:'',classList:{toggle(){}},querySelector(){return null;},querySelectorAll(){return[];},addEventListener(){},setAttribute(){},appendChild(){},getBoundingClientRect(){return {left:0,top:0,width:100,height:100}}});
const context={console,assert,window:{location:{search:''},dispatchEvent(){},parent:{postMessage(){}}},document:{getElementById(){return element()},querySelectorAll(){return[]},createElement(){return element()}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){},setTimeout(){},clearTimeout(){}};
vm.createContext(context); vm.runInContext(source+`
updateLabels=()=>{}; populateInspector=()=>{}; attachTransformGizmo=()=>{}; detachTransformGizmo=()=>{}; removeSelectionHighlight=()=>{};
const highlighted=[]; refreshSelectionHighlight=record=>highlighted.push(record);
const object=name=>({name,visible:true,userData:{},children:[],parent:null});
const normal=id=>{const item={id,editable:true,source_layer:'editable_layout'};const object3d=object(id);object3d.userData.item=item;return {item,object3d};};
const table=normal('support_surface_table'),camera=normal('realsense_overhead'),bin=normal('target_bin_default');
state.objects=[table,camera,bin]; state.sceneJson={scene:{id:'ur5_2f_test'},ui_selection_owners:[{id:'ur5'},{id:'robotiq_85_gripper'}]}; rebuildSelectionIdentityIndex();
for(const rendered of [table,camera,bin]){assert.strictEqual(itemFromRaycastHit({object:rendered.object3d}),rendered);selectObject(rendered.item.id);assert.strictEqual(state.selected,rendered.item.id);}
const register=(id,link,owner)=>{const node=object(link);return registerPickRecord({id,link_name:link,locked:true,editable:false,selectable:true},node,node,{pickRecordSource:'expanded_urdf_inspection',uiSelectionOwnerId:owner});};
const wrist=register('scene::inspection::wrist_3_link','wrist_3_link','ur5');
const finger=register('scene::inspection::robotiq_85_right_finger_link','robotiq_85_right_finger_link','robotiq_85_gripper');
for(const [record,owner] of [[wrist,'ur5'],[finger,'robotiq_85_gripper']]){assert.strictEqual(isCanvasSelectableRendered(record),true);selectObject(record.item.id);assert.strictEqual(editorState().selectedItemId,owner);assert.strictEqual(editorState().uiSelectionItemId,owner);assert.strictEqual(editorState().selectedEditable,false);assert.strictEqual(highlighted.at(-1),record);}
const excluded=object('selection_subtle_bounds_highlight'); excluded.userData.selection_highlight=true;
state.three={pointer:{},camera:{},raycaster:{setFromCamera(){},intersectObjects(){return this.hits;},hits:[]}};
for(const [record,canonical,editable] of [[wrist,'ur5',false],[finger,'robotiq_85_gripper',false],[camera,'realsense_overhead',true],[table,'support_surface_table',true],[bin,'target_bin_default',true]]){
  state.three.raycaster.hits=[{object:excluded,distance:0.5},{object:record.object3d,distance:1}];
  assert.strictEqual(pickObject({clientX:5,clientY:5}),canonical,'excluded first hit must not reject '+canonical);
  assert.strictEqual(editorState().selectedItemId,canonical);
  assert.strictEqual(editorState().selectedEditable,editable);
}
const diagnostic=normal('ordinary_diagnostic'); diagnostic.item.diagnostic_only=true; state.objects.push(diagnostic); assert.strictEqual(isCanvasSelectableRendered(diagnostic),false); assert.strictEqual(selectObject(diagnostic.item.id),'');
assert.strictEqual(state.objects.includes(wrist),false);assert.strictEqual(state.objects.includes(finger),false);assert.strictEqual(state.dirtyTransforms.size,0);assert.strictEqual(buildEditPatch().edits.length,0);assert.strictEqual(state.undoStack.length,0);assert.strictEqual(state.redoStack.length,0);
`,context);
"""
    import subprocess
    subprocess.run(["node", "-e", harness, str(ROOT / "workcell_studio_web/viewer/viewer.js")], cwd=ROOT, check=True, capture_output=True, text=True)


def test_qt_selection_clears_stale_scene_and_missing_id_callbacks():
    assert "selected_preview_item_id_.clear();" in CPP
    assert "scene_context_changed" in CPP
    assert "selection_missing_after_refresh" in CPP
    assert "Preview selection cleared after refresh (id missing):" in CPP
    assert "if (!embedded_web_identity_is_current(identity)) return;" in CPP
    assert "if (valid_browser_selection && browser_ui_selected_id != selected_preview_item_id_)" in CPP
    assert "selection_update_guard_" in (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def test_missing_nonempty_selection_is_preserved_and_warning_is_bounded():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    body = main.split("void MainWindow::apply_scene_selection", 1)[1].split("void MainWindow::mark_layout_dirty", 1)[0]
    assert "Ignored selection id absent from active scene payload; existing selection preserved:" in body
    assert "emitted_scene_diagnostic_log_keys_.contains(warning_key)" in body
    assert body.find("if (!active_scene_item_present)") < body.find("current_selected_scene_item_id_ = selected_id")
    assert "apply_scene_selection(QString(), selected_role, true, false)" not in body


def test_selection_diagnostics_are_exposed_to_qt_bridge():
    assert "function currentSelectionDiagnostics()" in VIEWER
    assert "selectionDiagnostics: currentSelectionDiagnostics()" in VIEWER
    assert "selectionDiagnostics: () => currentSelectionDiagnostics()" in VIEWER
    for field in ["renderIdentity", "sourceLayer", "activeVisualSource", "diagnosticOnly", "helperOrOverlay", "objectPresent"]:
        assert field in VIEWER


def test_qt_poll_accepts_only_final_valid_browser_selection_and_explicit_clear():
    poll = CPP.split("void ScenePreviewWidget::poll_embedded_editor_events()", 1)[1].split("#else", 1)[0]
    for token in [
        'editor_state.value(QStringLiteral("selectedItemId"))',
        'editor_state.value(QStringLiteral("uiSelectionItemId"))',
        'editor_state.value(QStringLiteral("selectionDiagnostics"))',
        'editor_state.value(QStringLiteral("sceneId"))',
        'selection_diagnostics.value(QStringLiteral("objectPresent")).toBool()',
        'preview_item_by_id(browser_ui_selected_id) != nullptr',
        'browser_scene_id == identity.scene_id',
    ]:
        assert token in poll
    assert "selected_preview_item_id_ = id" not in poll
    assert 'event.value(QStringLiteral("uiItemId"))' in poll
    assert "browser_ui_selected_id = browser_selected_id" in poll
    assert "selected_preview_item_id_ = browser_ui_selected_id" in poll
    assert "browser_selected_id.isEmpty()" in poll
    assert "selected_preview_item_id_.clear();" in poll


def test_qt_poll_treats_current_browser_state_as_authoritative_without_same_cycle_event():
    poll = CPP.split("void ScenePreviewWidget::poll_embedded_editor_events()", 1)[1].split("#else", 1)[0]
    assert "matching_selection_event" not in poll
    assert "const bool valid_browser_selection = scene_identity_matches && !browser_ui_selected_id.isEmpty()" in poll
    assert 'editor_state.value(QStringLiteral("selectedItemType"))' in poll
    assert "browser_ui_selected_id != selected_preview_item_id_" in poll
    assert "emit preview_item_selected(browser_ui_selected_id, matching_item_type);" in poll
    assert "browser_selected_id.isEmpty() && scene_identity_matches" in poll
    assert "state_request_token != embedded_editor_state_request_token_" in poll
    assert "preview_item_by_id(browser_ui_selected_id) != nullptr" in poll
    assert "Embedded Product View selection rejected:" in poll
    assert "emitted_scene_diagnostic_keys_.contains(rejection_key)" in poll


def test_qt_inventory_preserves_explicit_locked_robot_and_tool_owner_rows():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    body = main.split("QString selection_robot_owner_id", 1)[1].split("const bool filtered_payload_changed", 1)[0]
    assert 'value(QStringLiteral("selection_robot_owner_id"))' in body
    assert 'value(QStringLiteral("selection_tool_owner_id"))' in body
    assert "item.id.trimmed() == owner_id" in body
    assert "filtered_items.push_back(*owner)" in body
    assert "selection_robot_owner_id.isEmpty() && !explicit_robot_owner" in body
    assert "selection_tool_owner_id.isEmpty() && !explicit_tool_owner" in body
    assert "used unique category fallback" in body
    assert "item.locked && !item.editable" in body
    assert 'source_layer != QStringLiteral("locked_generated_urdf_visual")' in body
    assert "if (candidates.size() != 1) return;" in body
    assert 'QStringLiteral("robot_arm")' in body
    assert 'QStringLiteral("end_effector")' in body
    assert "filtered_items.push_back(candidates.front())" in body


def test_qt_reconciles_exported_selection_owner_registry_as_identity_only_rows():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    body = main.split("const QString selection_owner_contract_path", 1)[1].split(
        "apply_scene3d_product_view_layer_defaults_and_commit();", 1
    )[0]
    for token in [
        'value(QStringLiteral("ui_selection_owners"))',
        'value(QStringLiteral("robot_preview"))',
        'value(QStringLiteral("selection_robot_owner_id"))',
        'value(QStringLiteral("selection_tool_owner_id"))',
        'QStringLiteral("robot")',
        'QStringLiteral("end_effector")',
    ]:
        assert token in body
    assert 'source_layer == QStringLiteral("selection_owner_registry")' in body
    assert "all_scene_preview_items_.removeAt(i)" in body
    assert "item.id.trimmed() == declaration.id" in body
    assert "if (id_already_owned) continue;" in body
    assert 'owner.source_layer = QStringLiteral("selection_owner_registry")' in body
    assert "owner.locked = true" in body
    assert "owner.editable = false" in body
    assert "owner.mesh_available = false" in body
    assert "owner.has_mesh_metadata = false" in body
    assert "owner.primitive_geometry_type.clear()" in body
    assert "owner.sx = owner.sy = owner.sz = 0.0" in body
    assert "all_scene_preview_items_.push_back(owner)" in body
    assert "add_tree_node(owner)" in body
    assert "retained usable preview state" in body


def test_selection_owner_registry_is_forwarded_for_lookup_but_not_geometry():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    filters = main.split("void MainWindow::apply_scene3d_preview_layer_filters", 1)[1].split(
        "void MainWindow::refresh_scene3d_product_view_status_and_audit", 1
    )[0]
    registry_branch = filters.split('p.source_layer == QStringLiteral("selection_owner_registry")', 1)[1]
    registry_branch = registry_branch.split("if (workcell_builder::include_preview_item_for_scene3d", 1)[0]
    assert "filtered_items.push_back(p)" in registry_branch
    assert "continue;" in registry_branch
    assert "preview_item_by_id(browser_ui_selected_id) != nullptr" in CPP
    ingest = VIEWPORT_CPP.split("void Scene3DViewportWidget::ingest_preview_items", 1)[1]
    ingest = ingest.split("void Scene3DViewportWidget::", 1)[0]
    assert 'item.source_layer == QStringLiteral("selection_owner_registry")' in ingest
    assert "continue;" in ingest


def test_qt_exact_ur5_and_robotiq_owner_ids_remain_selectable_in_preview_inventory():
    main = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    environment = (ROOT / "scenes/ur5_2f_test/environment.yaml").read_text(encoding="utf-8")
    owner_body = main.split("auto preserve_explicit_inspection_owner", 1)[1].split("// Compatibility fallback", 1)[0]
    poll = CPP.split("void ScenePreviewWidget::poll_embedded_editor_events()", 1)[1].split("#else", 1)[0]
    assert "all_scene_preview_items_.cbegin()" in owner_body
    assert "item.id.trimmed() == owner_id" in owner_body
    assert "filtered_items.push_back(*owner)" in owner_body
    for owner_id in ("ur5", "robotiq_85_gripper"):
        assert f"id: {owner_id}" in environment
    assert "preview_item_by_id(browser_ui_selected_id) != nullptr" in poll
