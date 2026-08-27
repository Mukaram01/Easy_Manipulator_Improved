import json
import subprocess
from pathlib import Path

import pytest
import xml.etree.ElementTree as ET

from scripts import export_workcell_studio_web_scene as exporter


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "ur5_airpick4_test"
VIEWER = ROOT / "workcell_studio_web" / "viewer" / "viewer.js"
ALL_ACCEPTANCE_SCENES = (
    "ur5_2f_test",
    "suction_test",
    "ur5_2f_sorting_test",
    "ur5_3f_test",
    "ur5_airpick4_test",
    "ur10_2f_test",
    "ur3_suction_test",
    "ur5_2f_builder_pick_place_demo",
)


def _payload(tmp_path: Path) -> dict:
    return exporter.build_web_scene(
        SCENE,
        stage_assets=True,
        output_path=tmp_path / "ur5_airpick4_test.web_scene.json",
    )


def test_airpick_scene_exports_one_canonical_owner_for_each_physical_assembly(tmp_path):
    payload = _payload(tmp_path)
    owners = {item["id"]: item for item in payload["ui_selection_owners"]}

    assert payload["robot_preview"]["selection_robot_owner_id"] == "ur5"
    assert payload["robot_preview"]["selection_tool_owner_id"] == "onrobot_airpick4"
    assert {"ur5", "onrobot_airpick4", "realsense_overhead", "table_main"} <= owners.keys()
    assert owners["realsense_overhead"]["locked"] is True
    assert owners["realsense_overhead"]["editable"] is False

    d435 = [item for item in payload["sensors"] if "d435.dae" in item.get("mesh_uri", "")]
    assert len(d435) == 1
    assert d435[0]["canonical_scene_item_id"] == "realsense_overhead"
    assert d435[0]["camera_id"] == "realsense_overhead"
    assert d435[0]["render_policy"] == "primary"


def test_commissioning_fallback_is_only_in_the_debug_overlay_layer(tmp_path):
    payload = _payload(tmp_path)
    helper = next(item for item in payload["zones"] if item["id"] == "commissioning_object")
    assert helper["render_policy"] == "overlay"
    assert helper["render_owner"] == "task_overlay"
    assert helper["readiness_category"] == ""


def test_canonical_hierarchy_selection_resolves_back_to_physical_records(tmp_path):
    payload_path = tmp_path / "scene.json"
    payload_path.write_text(json.dumps(_payload(tmp_path)), encoding="utf-8")
    harness = r"""
const fs=require('fs'),vm=require('vm'),assert=require('assert');
let source=fs.readFileSync(process.argv[1],'utf8').replace(/boot\(\);\s*$/,'');
const element=()=>({hidden:false,checked:false,disabled:false,textContent:'',className:'',innerHTML:'',classList:{toggle(){}},querySelector(){return null},querySelectorAll(){return[]},appendChild(){},addEventListener(){},removeEventListener(){},setAttribute(){}});
const context={console,assert,window:{location:{search:''},dispatchEvent(){},addEventListener(){},removeEventListener(){},parent:{postMessage(){}}},document:{getElementById(){return element()},createElement(){return element()},querySelectorAll(){return[]}},URLSearchParams,CustomEvent:function(){},requestAnimationFrame(){return 0},cancelAnimationFrame(){},setTimeout(){return 0},clearTimeout(){}};
vm.createContext(context);
vm.runInContext(source+`
state.sceneJson=JSON.parse(${JSON.stringify(fs.readFileSync(process.argv[2],'utf8'))});
rebuildSelectionIdentityIndex(); state.objects=[]; state.pickRecords=[]; state.editorEvents=[];
updateLabels=()=>{}; populateInspector=()=>{}; detachTransformGizmo=()=>{}; attachTransformGizmo=()=>{};
let highlighted=[]; refreshSelectionHighlight=record=>highlighted.push(record); removeSelectionHighlight=()=>{};
const physical=(id,owner,link)=>({item:{id,link_name:link,locked:true,editable:false,selectable:true,source_kind:'generated_preview'},object3d:{visible:true},authoritativePhysicalPick:true,uiSelectionOwnerId:owner,pickRecordSource:'test'});
const records=[physical('urdf_visual_robot','ur5','base_link'),physical('urdf_visual_tool','onrobot_airpick4','gripper_base_link'),physical('urdf_visual_camera','realsense_overhead','camera_link')];
state.pickRecords.push(...records);
for(const record of records){
  const owner=record.uiSelectionOwnerId;
  assert.strictEqual(explicitUiSelectionItemId(record),owner,'physical mesh to canonical owner');
  selectObjectFromRender(record.item.id,record);
  assert.strictEqual(state.selected,owner,'3D mesh click selects canonical owner');
  assert.strictEqual(state.selectedRenderIdentityId,record.item.id);
  selectObject(owner);
  assert.strictEqual(state.selected,owner,'hierarchy row retains canonical owner');
  assert.strictEqual(state.selectedRenderIdentityId,record.item.id,'hierarchy row resolves physical mesh/group');
  assert.strictEqual(highlighted.at(-1),record,'physical record is highlighted');
}
`,context);
"""
    result = subprocess.run(
        ["node", "-e", harness, str(VIEWER), str(payload_path)],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_native_hierarchy_materializes_camera_owner_when_physical_id_already_exists():
    source = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
    assert "id_already_owned_by_hierarchy_item" in source
    assert "item.id.trimmed() == declaration.id && is_user_facing_scene_hierarchy_item(item)" in source
    assert 'role == QStringLiteral("camera")' in source


@pytest.mark.parametrize("scene_id", ("suction_test", "ur3_suction_test"))
def test_suction_transform_chain_matches_package_truth(scene_id):
    xacro = (ROOT / "scenes" / scene_id / "urdf/scene.urdf.xacro").read_text(encoding="utf-8")
    assert 'parent="tool0"' in xacro
    assert (
        'rpy="1.5707 -1.5707 0"' in xacro
        or 'name="suction_mount_rpy" default="1.5707 -1.5707 0"' in xacro
    )

    package_xacro = (
        ROOT / "assets/end_effectors/single_suction_gripper/single_suction_description/urdf/single_suction_gripper.urdf.xacro"
    ).read_text(encoding="utf-8")
    assert 'xyz="-0.22836 0.053 0" rpy="1.5708 0 -1.5708"' in package_xacro

    expanded = ET.parse(ROOT / "scenes" / scene_id / "generated/expanded_scene_preview.urdf").getroot()
    joints = {joint.attrib["name"]: joint for joint in expanded.findall("joint")}
    mount = joints["gripper_base_joint"].find("origin")
    cup = joints["virtual_ee_joint"].find("origin")
    assert mount is not None and mount.attrib == {"rpy": "1.5707 -1.5707 0", "xyz": "0 0 0"}
    assert cup is not None and cup.attrib == {"rpy": "1.5708 0 -1.5708", "xyz": "-0.22836 0.053 0"}

    suction_visuals = [
        visual
        for link in expanded.findall("link")
        if link.attrib.get("name") == "suction_cup_link"
        for visual in link.findall("visual")
    ]
    assert len(suction_visuals) == 1


@pytest.mark.parametrize("scene_id", ALL_ACCEPTANCE_SCENES)
def test_all_scene_exports_have_unique_physical_selection_owners(tmp_path, scene_id):
    payload = exporter.build_web_scene(
        ROOT / "scenes" / scene_id,
        stage_assets=True,
        output_path=tmp_path / f"{scene_id}.web_scene.json",
    )
    owners = payload["ui_selection_owners"]
    owner_ids = [owner["id"] for owner in owners]
    owner_types = {owner["type"] for owner in owners}

    assert len(owner_ids) == len(set(owner_ids))
    assert not any(owner_id.startswith("urdf_visual_") for owner_id in owner_ids)
    assert {"robot", "end_effector", "support_surface"} <= owner_types
    assert payload["robot_preview"]["selection_robot_owner_id"] in owner_ids
    assert payload["robot_preview"]["selection_tool_owner_id"] in owner_ids

    authoritative_tables = [
        item
        for section in ("assets", "sensors")
        for item in payload.get(section, [])
        if item.get("semantic_role") == "support_surface"
        and item.get("render_policy") == "primary"
    ]
    assert len(authoritative_tables) == 1
    assert authoritative_tables[0].get("canonical_scene_item_id", authoritative_tables[0]["id"]) in owner_ids

    d435 = [item for item in payload["sensors"] if "d435.dae" in item.get("mesh_uri", "")]
    if d435:
        camera_owners = [owner for owner in owners if owner["type"] == "camera"]
        assert len(camera_owners) == 1
        camera_owner_id = camera_owners[0]["id"]
        primary_d435 = [item for item in d435 if item["render_policy"] == "primary"]
        assert len(primary_d435) == 1
        assert (
            primary_d435[0]["id"] == camera_owner_id
            or primary_d435[0].get("camera_id") == camera_owner_id
            or primary_d435[0].get("canonical_scene_item_id") == camera_owner_id
        )

    helpers = [
        item for item in payload["zones"]
        if item.get("id") == "commissioning_object"
    ]
    assert all(item["render_policy"] == "overlay" for item in helpers)
