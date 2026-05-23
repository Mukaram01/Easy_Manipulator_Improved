from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
SMOKE_PY = (ROOT / 'scripts/run_workcell_builder_scene3d_gui_smoke.py').read_text(encoding='utf-8')


def test_wrapper_uses_required_smoke_flags():
    for token in ["--scene3d-smoke", "--smoke-output", "--smoke-screenshot", "--exit-after-smoke"]:
        assert token in SMOKE_PY


def test_cpp_handles_scene3d_smoke_flags_and_output_write():
    for token in ["--scene3d-smoke", "--smoke-output", "--smoke-screenshot", "--exit-after-smoke"]:
        assert token in MAIN_CPP
    assert "QFile out(opts_.smoke_output)" in MAIN_CPP
    assert "visual_quality_failed" in MAIN_CPP
