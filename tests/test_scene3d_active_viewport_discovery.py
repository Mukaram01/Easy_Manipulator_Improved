from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
HDR = Path('workcell_builder/workcell_builder/gui/mainwindow.h').read_text(encoding='utf-8')


def test_stable_widget_object_names_are_used_for_viewport_discovery():
    assert '"scene3dViewportWidget"' in CPP
    assert '"scenePreviewWidget"' in CPP or 'ScenePreviewWidget' in HDR


def test_active_resolver_contract_and_candidate_enumeration_tokens_exist():
    for token in [
        'active_scene_preview_widget()',
        'active_viewport_counter_handoff_failed',
        'findChild<Scene3DViewportWidget *>("scene3dViewportWidget")',
    ]:
        assert token in (CPP + "\n" + HDR)


def test_json_keys_for_viewport_candidate_tracking_and_source_are_present():
    for token in [
        'viewport_candidates',
        'active_viewport_candidate_index',
        'viewport_counter_source',
    ]:
        assert token in CPP
