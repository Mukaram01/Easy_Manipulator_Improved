from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp"
HDR = ROOT / "workcell_builder/workcell_builder/gui/scene_select.h"
README = ROOT / "workcell_studio_web/viewer/README.md"
PATCH_DOC = ROOT / "docs/WORKCELL_STUDIO_WEB_3D_EDIT_PATCH.md"


def test_web_viewer_action_registered_and_wired_to_export_slot():
    cpp = CPP.read_text(encoding="utf-8")
    hdr = HDR.read_text(encoding="utf-8")
    assert "Export & Open Web 3D Viewer" in cpp
    assert "export_open_web_3d_viewer_action" in cpp
    assert "on_export_open_web_3d_viewer_clicked" in cpp
    assert "void on_export_open_web_3d_viewer_clicked();" in hdr
    assert "QDesktopServices::openUrl" in cpp


def test_web_viewer_action_exports_to_build_not_scenes_or_generated():
    cpp = CPP.read_text(encoding="utf-8")
    assert '"scripts" / "export_workcell_studio_web_scene.py"' in cpp
    assert 'repo_root / "build" / "workcell_studio_web_scene"' in cpp
    assert 'scene_id + ".web_scene.json"' in cpp
    assert 'QProcess::execute("python3", args)' in cpp
    assert '"--scene", QString::fromStdString(scene_dir.string())' in cpp
    assert '"--output", QString::fromStdString(output_path.string())' in cpp
    assert 'repo_root / "workcell_studio_web" / "viewer" / "index.html"' in cpp


def test_user_facing_failures_and_local_server_fallback_are_present():
    cpp = CPP.read_text(encoding="utf-8")
    for text in [
        "No scene selected",
        "Exporter script missing",
        "Scene path missing",
        "Web 3D scene export failed",
        "Viewer file missing",
        "Browser open failed",
        "python3 -m http.server 8765",
        "http://localhost:8765/workcell_studio_web/viewer/index.html",
    ]:
        assert text in cpp


def test_docs_describe_builder_action_and_safety_boundaries():
    docs = README.read_text(encoding="utf-8") + "\n" + PATCH_DOC.read_text(encoding="utf-8")
    assert "Export & Open Web 3D Viewer" in docs
    assert "build/workcell_studio_web_scene/<scene_id>.web_scene.json" in docs
    assert "does not write under `scenes/`" in docs
    assert "does not apply edit patches" in docs
    assert "RViz/MoveIt" in docs
    assert "python3 -m http.server 8765" in docs
