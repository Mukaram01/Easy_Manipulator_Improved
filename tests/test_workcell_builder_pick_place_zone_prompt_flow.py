from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"


def _body(fn: str, text: str) -> str:
    start = text.index(f"void MainWindow::{fn}()")
    brace = text.index("{", start)
    depth = 0
    for i in range(brace, len(text)):
        ch = text[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[brace:i+1]
    raise AssertionError("function body not found")


def test_pick_zone_no_decline_does_not_mutate_source_or_zone_fields():
    text = CPP.read_text(encoding="utf-8")
    body = _body("bind_selected_item_as_pick_zone", text)

    prompt_idx = body.index("QMessageBox::question")
    src_update_idx = body.index('update_selected_scene_task_intent_binding("Pick Source"')
    zone_update_idx = body.index('update_selected_scene_task_intent_binding("Pick Zone"')
    assert prompt_idx < src_update_idx
    assert prompt_idx < zone_update_idx

    else_block = body.split("else", 1)[1]
    assert 'update_selected_scene_task_intent_binding("Pick Source"' not in else_block
    assert 'update_selected_scene_task_intent_binding("Pick Zone"' not in else_block


def test_place_zone_no_decline_does_not_mutate_target_or_zone_fields():
    text = CPP.read_text(encoding="utf-8")
    body = _body("bind_selected_item_as_place_zone", text)

    prompt_idx = body.index("QMessageBox::question")
    target_update_idx = body.index('update_selected_scene_task_intent_binding("Place Target"')
    zone_update_idx = body.index('update_selected_scene_task_intent_binding("Place Zone"')
    assert prompt_idx < target_update_idx
    assert prompt_idx < zone_update_idx

    else_block = body.split("else", 1)[1]
    assert 'update_selected_scene_task_intent_binding("Place Target"' not in else_block
    assert 'update_selected_scene_task_intent_binding("Place Zone"' not in else_block
