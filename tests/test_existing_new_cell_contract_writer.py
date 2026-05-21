from pathlib import Path
import yaml
from scripts.export_builder_scene_to_cell_definition import export_scene

def _seed_scene(scene: Path) -> None:
    (scene / "layout").mkdir(parents=True, exist_ok=True)
    (scene / "generated").mkdir(parents=True, exist_ok=True)
    (scene / "environment.yaml").write_text(yaml.safe_dump({"scene_name": scene.name, "robot": {"name": "ur5", "base_link": "base_link"}, "end_effector": {"name": "robotiq_85"}, "task_zones": [{"id": "pick_zone_01", "type": "pick_zone", "frame": "world", "dimensions": [0.3,0.2,0.15]}, {"id": "place_zone_01", "type": "place_zone", "frame": "world", "dimensions": [0.3,0.2,0.15]}], "objects": {}, "metadata": {}, "fake_hardware_first": True}, sort_keys=False), encoding="utf-8")
    (scene / "layout" / "workcell_studio_layout.yaml").write_text(yaml.safe_dump({"schema": "workcell_studio_layout/v1", "items": [{"id": "pick_zone_01", "type": "pick_zone", "editable": True}, {"id": "place_zone_01", "type": "place_zone", "editable": True}], "zones": [{"id": "pick_zone_01", "type": "pick_zone"}, {"id": "place_zone_01", "type": "place_zone"}], "targets": [{"id": "place_zone_01", "type": "bin"}]}, sort_keys=False), encoding="utf-8")

def test_minimal_new_cell_export_writes_core_contract_files(tmp_path: Path):
    scene = tmp_path / "src" / "scenes" / "demo_cell"; scene.mkdir(parents=True); _seed_scene(scene)
    result = export_scene(scene, scene / "generated", validate=False)
    assert (scene / "generated" / "cell_definition.yaml").is_file()
    assert (scene / "generated" / "environment_layout.yaml").is_file()
    assert result["generated_by"] == "workcell_builder"

def test_generated_contract_is_fake_hardware_first_and_zone_bound(tmp_path: Path):
    scene = tmp_path / "src" / "scenes" / "safe_cell"; scene.mkdir(parents=True); _seed_scene(scene)
    result = export_scene(scene, scene / "generated", validate=False)
    cell_text = (scene / "generated" / "cell_definition.yaml").read_text(encoding="utf-8").lower()
    assert "pick_zone_01" in cell_text and "place_zone_01" in cell_text
    assert "use_fake_hardware:=false" not in cell_text and "fake_hardware:=false" not in cell_text
    assert "ur_robot_driver" not in cell_text and "ethercat" not in cell_text and "canopen" not in cell_text
    assert any("incomplete" in w.lower() for w in result.get("warnings", []))
