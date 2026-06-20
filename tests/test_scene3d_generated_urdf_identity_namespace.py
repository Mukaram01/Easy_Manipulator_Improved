from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def test_generated_urdf_visual_ids_are_namespaced_and_row_indexed():
    assert 'QStringLiteral("generated_urdf::%1::%2::%3")' in MAIN
    assert 'visual_item_final_identity_key(v, source_row_index)' in MAIN
    assert 'preview_ids.contains(id)' in MAIN
    assert '::dedupe_%2' in MAIN
    assert 'continue;\n          }\n          const QString geometry_type' not in MAIN


def test_robot_base_regression_assertion_keeps_generated_links_visible():
    assert 'regression_preview_ids.insert(QStringLiteral("robot_base"));' in MAIN
    assert 'generated_urdf::base_link::base_visual::0' in MAIN
    assert 'generated_urdf::shoulder_link::shoulder_visual::1' in MAIN
    assert 'generated_urdf::upper_arm_link::upper_arm_visual::2' in MAIN
    assert 'editable robot_base must not suppress generated URDF visuals' in MAIN
