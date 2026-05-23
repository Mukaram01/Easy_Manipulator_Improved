from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')


def test_stable_widget_object_names_present_in_construction_and_discovery_paths():
    assert 'scenePreviewWidget' in CPP
    assert 'scene3dViewportWidget' in CPP


def test_active_viewport_resolver_and_candidate_enumeration_structures_exist():
    resolver_tokens = [
        'resolve_active_scene3d_viewport',
        'Scene3DViewportCandidate',
        'resolution.candidates',
        'add_candidate',
    ]
    for token in resolver_tokens:
        assert token in CPP


def test_candidate_selection_precedence_prefers_visible_nonzero_over_hidden_zero():
    # Selection score grants visibility points and additional points for non-zero counters.
    assert 'if (c.is_visible) s += 100000;' in CPP
    assert 's += c.non_zero_counter_count * 10;' in CPP
    assert 'candidate.non_zero_counter_count' in CPP
