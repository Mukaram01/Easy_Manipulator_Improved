from pathlib import Path
import ast


APP_PATH = Path("tools/workcell_studio_streamlit/app.py")


def _source() -> str:
    return APP_PATH.read_text(encoding="utf-8")


def test_app_py_compiles() -> None:
    ast.parse(_source())


def test_high_risk_buttons_have_explicit_keys() -> None:
    src = _source()
    required = {
        'Discover scene targets", key="builder_discover_scene_targets"',
        'Save/update target", key="zones_save_update_target"',
        'Refresh discovered targets", key="zones_refresh_discovered_targets"',
        'Generate static preview", key="zones_generate_static_preview"',
        'Generate readiness pack from current scene", key="zones_generate_readiness_pack"',
        'Save task intent", key="task_intent_save"',
        'Validate task intent", key="task_intent_validate"',
        'Generate task recipe", key="task_intent_generate_recipe"',
        'Generate readiness pack after task intent", key="task_intent_generate_readiness_pack"',
    }
    for token in required:
        assert token in src, f"missing keyed widget token: {token}"


def test_both_readiness_buttons_have_different_keys() -> None:
    src = _source()
    assert 'key="zones_generate_readiness_pack"' in src
    assert 'key="task_intent_generate_readiness_pack"' in src


def test_task_intent_readiness_options_present() -> None:
    src = _source()
    assert "prepare_rviz_preview=readiness_prepare_rviz_preview" in src
    assert "smoke_dry_run=readiness_smoke_dry_run" in src


def test_dashboard_preview_guidance_present() -> None:
    src = _source()
    assert "backend.load_readiness_pack_manifest(Path(manifest_path).parent)" in src
    assert "http://localhost:8767/readiness_dashboard.html" in src
    assert "may not work with sandboxed Firefox/Snap" in src
    assert "xdg-open" in src
