from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]

def test_preview_templates_flagged():
    demos=yaml.safe_load((ROOT/"catalog/workcell_studio_demos.yaml").read_text())["demos"]
    preview=[d for d in demos if d.get("preview_only")]
    assert preview
    for d in preview:
        assert d["runtime_mode"]=="preview_only"
