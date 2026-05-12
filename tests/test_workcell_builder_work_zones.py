from pathlib import Path
import yaml


def sample():
    return {
        "work_zones": [
            {"name": "detection_zone_1", "type": "camera_detection", "linked_camera": "realsense_d435i_1", "runtime_mode": "metadata_only"},
            {"name": "pick_zone_1", "type": "robot_pick", "linked_robot": "ur5", "runtime_mode": "metadata_only"},
            {"name": "place_zone_1", "type": "robot_place", "destination": "bin_left", "runtime_mode": "metadata_only"},
        ],
        "conveyor_flows": [
            {"name": "conveyor_flow_1", "detection_zone": "detection_zone_1", "pick_zone": "pick_zone_1", "runtime_mode": "metadata_only"}
        ],
    }


def test_work_zone_yaml_roundtrip():
    d = sample()
    s = yaml.safe_dump(d)
    loaded = yaml.safe_load(s)
    assert loaded["work_zones"][0]["type"] == "camera_detection"
    assert loaded["work_zones"][1]["type"] == "robot_pick"
    assert loaded["work_zones"][2]["type"] == "robot_place"


def test_conveyor_flow_yaml_roundtrip():
    loaded = yaml.safe_load(yaml.safe_dump(sample()))
    flow = loaded["conveyor_flows"][0]
    assert flow["detection_zone"] == "detection_zone_1"
    assert flow["pick_zone"] == "pick_zone_1"


def test_ui_static_strings_present():
    ui = Path("workcell_builder/workcell_builder/gui/addscene.ui").read_text()
    for text in ["Add Work Zone", "Edit Work Zone", "Remove Work Zone", "Add Conveyor Flow", "camera_detection", "robot_pick", "robot_place", "metadata_only"]:
        assert text in ui


def test_docs_work_zones_mentions():
    body = Path("docs/manuals/WORKCELL_BUILDER_WORK_ZONES.md").read_text()
    assert "metadata-only" in body
    assert "not safety-certified" in body.lower()



def test_bundle_metadata_fields_preserved():
    data = sample()
    manifest_like = {"environment": data}
    cloned = yaml.safe_load(yaml.safe_dump(manifest_like))
    assert "work_zones" in cloned["environment"]
    assert "conveyor_flows" in cloned["environment"]
