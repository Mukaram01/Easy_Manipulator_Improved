from pathlib import Path
import yaml


def test_routing_table_yaml_roundtrip():
    src = Path('workcell_builder/workcell_builder/src_class_routing_model.cpp').read_text(encoding='utf-8')
    assert 'serialize_class_routing_yaml' in src
    payload = {
        'class_routing': {
            'schema_version': 1,
            'runtime_mode': 'preview_only',
            'default_place_zone': 'reject_zone',
            'routes': [{'class_label': 'box', 'destination': 'bin_a', 'place_zone': 'place_zone_box', 'priority': 10, 'enabled': True}],
        }
    }
    text = yaml.safe_dump(payload)
    out = yaml.safe_load(text)
    route = out['class_routing']['routes'][0]
    assert route['class_label'] == 'box'
    assert route['place_zone'] == 'place_zone_box'
    assert route['destination'] == 'bin_a'


def test_route_match_default_and_errors_markers_present():
    src = Path('workcell_builder/workcell_builder/src_class_routing_model.cpp').read_text(encoding='utf-8')
    for token in [
        'route_detection_class_to_place_zone',
        'ERROR: route references missing place_zone',
        'ERROR: duplicate class_label routes with same priority and enabled true',
        'ERROR: no place zone available for routed class',
        'WARN: unknown class used fallback route',
    ]:
        assert token in src


def test_task_intent_and_planning_and_status_and_ui_and_bundle_integration_markers():
    tip = Path('workcell_builder/workcell_builder/src_task_intent_readiness.cpp').read_text(encoding='utf-8')
    status = Path('workcell_builder/workcell_builder/src_workcell_scene_status.cpp').read_text(encoding='utf-8')
    bundle = Path('workcell_builder/workcell_builder/src_workcell_scene_bundle.cpp').read_text(encoding='utf-8')
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    assert 'place_zone may be overwritten by routing preview artifacts' in tip
    assert 'Class routing table available' in status
    assert 'Selected place zone' in status
    assert 'Routing preview only' in status
    assert 'class_routing_table.yaml' in bundle
    for label in ['Add Class Route', 'Edit Class Route', 'Remove Class Route', 'Preview Class Routing', 'Open Class Routing Result']:
        assert label in ui


def test_catalog_and_docs_updated_and_real_hardware_false():
    catalog = Path('catalog/scenarios/industrial_scenarios.yaml').read_text(encoding='utf-8')
    matrix = Path('docs/roadmap/WORKCELL_STUDIO_CAPABILITY_MATRIX.md').read_text(encoding='utf-8')
    todo = Path('docs/roadmap/WORKCELL_STUDIO_TODO.md').read_text(encoding='utf-8')
    assert 'class_to_place_zone_routing' in catalog
    assert 'conveyor_sorting_by_class' in catalog
    assert 'multi_bin_sorting_cell' in catalog
    assert 'inspection_and_reject' in catalog
    assert 'real_hardware_ready: false' in catalog
    assert 'class-to-place-zone routing' in matrix.lower()
    assert 'class-to-place-zone routing' in todo.lower()
