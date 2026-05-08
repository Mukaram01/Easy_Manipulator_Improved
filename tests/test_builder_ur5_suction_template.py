from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]

def test_suction_template_warn_not_false_supported():
    demos=yaml.safe_load((ROOT/"catalog/workcell_studio_demos.yaml").read_text())["demos"]
    demo=next(d for d in demos if d["id"]=="ur5_suction_table_pick_place")
    assert "warn" in str(demo.get("compatibility_status","")).lower()
