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


def test_web_viewer_action_exports_fresh_scene_to_build_before_opening_viewer():
    cpp = CPP.read_text(encoding="utf-8")
    assert '"scripts" / "run_workcell_web3d_visual_acceptance.py"' in cpp
    assert 'repo_root / "build" / "workcell_studio_web_scene"' in cpp
    assert 'scene_id + ".web_scene.json"' in cpp
    assert 'scene_dir_for_current_selection()' in cpp
    assert '"--scene", QString::fromStdString(scene_dir.string())' in cpp
    assert '"--output", QString::fromStdString(output_path.string())' in cpp
    assert '"--port", "8765"' in cpp
    assert 'repo_root / "workcell_studio_web" / "viewer" / "index.html"' in cpp
    assert (
        "Web 3D scene export failed; viewer not opened with stale generated artifacts."
        in cpp
    )
    assert "QProcess::startDetached" in cpp
    assert "python3 -m http.server 8765 --bind 127.0.0.1" in cpp
    assert "http://localhost:8765/workcell_studio_web/viewer/index.html?scene=" in cpp

    export_success_index = cpp.index('append_success("Exported Web 3D scene JSON: "')
    server_start_index = cpp.index("QProcess::startDetached", export_success_index)
    browser_open_index = cpp.index("QDesktopServices::openUrl(QUrl(viewer_url))", export_success_index)
    assert export_success_index < server_start_index
    assert export_success_index < browser_open_index


def test_user_facing_failures_and_local_server_fallback_are_present():
    cpp = CPP.read_text(encoding="utf-8")
    for text in [
        "No scene selected",
        "Exporter script missing",
        "Scene path missing",
        "Web 3D scene export failed",
        "Viewer file missing",
        "Browser open failed",
        "reused existing server",
        "Local static asset server",
        "Viewer URL opened",
        "Exported web scene JSON",
        "python3 -m http.server 8765 --bind 127.0.0.1",
        "http://localhost:8765/workcell_studio_web/viewer/index.html?scene=",
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
