from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]

def test_golden_template_present():
    demos=yaml.safe_load((ROOT/"catalog/workcell_studio_demos.yaml").read_text())["demos"]
    demo=next(d for d in demos if d["id"]=="ur5_2f_table_pick_place")
    assert demo["runtime_mode"]=="fake_hardware_ready"
    assert demo.get("status")=="supported"
