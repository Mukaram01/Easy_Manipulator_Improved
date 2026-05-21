from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
CONTRACT_DOC = (ROOT / 'docs/architecture/SCENE3D_CANVAS_CONTRACT.md').read_text(encoding='utf-8')


def test_apply_transform_path_targets_layout_and_never_generated_artifacts_tokens():
    assert 'Save Layout' in MAIN_CPP
    assert 'Locked/generated item edit rejected' in MAIN_CPP


def test_contract_documents_transform_editing_guards():
    assert 'Selection and transform editing rules' in CONTRACT_DOC
    for tok in ['editable_layout', 'locked_generated_urdf_visual', 'mesh_preview', 'primitive_fallback']:
        assert tok in CONTRACT_DOC
    assert 'must not mutate generated artifacts' in CONTRACT_DOC


def test_invalid_or_locked_edit_is_rejected_safely():
    assert 'Locked/generated item edit rejected' in MAIN_CPP
    assert 'if(inspector_update_guard_ || !digital_twin_scene_ || digital_twin_scene_->selectedItems().isEmpty()) return;' in MAIN_CPP


def test_safety_tokens_absent_in_docs_and_tests_fixture_scope():
    corpus = '\n'.join([
        CONTRACT_DOC,
        (ROOT / 'docs/manuals/WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md').read_text(encoding='utf-8'),
        (ROOT / 'tests/test_scene3d_canvas_contract.py').read_text(encoding='utf-8'),
    ]).lower()
    for forbidden in ['use_fake_hardware:=false', 'fake_hardware:=false', 'ur_robot_driver', 'ethercat', 'canopen']:
        assert forbidden not in corpus
