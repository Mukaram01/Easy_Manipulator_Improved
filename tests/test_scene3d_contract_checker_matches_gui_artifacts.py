from pathlib import Path

CHK = Path('scripts/check_scene3d_canvas_contract.py').read_text(encoding='utf-8')


def test_contract_checker_counts_generated_fallback_aliases():
    assert 'generated_urdf_visual' in CHK
    assert 'locked_generated_urdf_visual' in CHK
    assert 'primitive_fallback' in CHK
