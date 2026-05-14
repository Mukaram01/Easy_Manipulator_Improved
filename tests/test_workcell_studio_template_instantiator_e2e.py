from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_instantiator_contract_tokens_present():
    h = ROOT / "workcell_builder/workcell_builder/include/workcell_studio_template_instantiator.hpp"
    c = ROOT / "workcell_builder/workcell_builder/src_workcell_studio_template_instantiator.cpp"
    assert h.exists()
    assert c.exists()
    text = c.read_text(encoding="utf-8")
    for token in [
        "environment.yaml",
        "package.xml",
        "task_recipe.yaml",
        "workcell_builder_task_intent.yaml",
        "-1.5708 -1.5708 0",
        "suction_top",
        "PREVIEW_ONLY",
        "use_fake_hardware:=true",
    ]:
        assert token in text
