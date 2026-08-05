import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
COALESCER_PATH = REPO_ROOT / "scripts/workcell_studio_scene_refresh_coalescer.py"
spec = importlib.util.spec_from_file_location("workcell_studio_scene_refresh_coalescer", COALESCER_PATH)
coalescer = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = coalescer
spec.loader.exec_module(coalescer)


def _write(path: Path, text: str = "x") -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def _fake_impl(tmp_path: Path):
    scene = tmp_path / "scenes" / "demo_scene"
    source = _write(scene / "scene_manifest.yaml", "scene: demo_scene\n")
    mesh_index = _write(scene / "generated" / "scene_visual_mesh_index.json", "{}\n")
    build = tmp_path / "build" / "workcell_studio_web_scene"
    output = _write(
        build / "demo_scene.web_scene.json",
        json.dumps({"schema_version": "workcell_studio_web_scene/v1", "scene_id": "demo_scene"}),
    )
    asset_dir = build / "assets" / "demo_scene"
    _write(asset_dir / "mesh.stl", "solid mesh\nendsolid mesh\n")
    impl = SimpleNamespace(
        MESH_INDEX_REL=Path("generated/scene_visual_mesh_index.json"),
        ASSET_BUILD_ROOT=build / "assets",
        existing_inputs=lambda _scene_dir: [source],
        normalize_scene=lambda _arg: ("demo_scene", scene),
        normalize_output=lambda arg: Path(arg),
    )
    return impl, scene, source, mesh_index, output


def _argv(output: Path, *extra: str):
    return [
        "--scene",
        "demo_scene",
        "--output",
        str(output),
        "--stage-assets",
        *extra,
    ]


def _freshener_result(output: Path, calls: list, mutate=None):
    def main(argv):
        calls.append(list(argv))
        if mutate is not None:
            mutate()
        print(
            json.dumps(
                {
                    "schema_version": "workcell_studio_web_scene_freshener/v1",
                    "status": "rebuilt",
                    "scene_id": "demo_scene",
                    "output": str(output),
                    "fingerprint": "test",
                    "staged_asset_diagnostics": {"ok": True},
                }
            )
        )
        return 0

    return main


def test_startup_request_with_empty_scene_identity_is_rejected(tmp_path, capsys):
    impl, _scene, _source, _mesh_index, output = _fake_impl(tmp_path)
    calls = []
    rc = coalescer.run_coalesced(
        impl,
        _freshener_result(output, calls),
        ["--scene", "   ", "--output", str(output)],
    )
    assert rc == 2
    assert calls == []
    assert "non-empty scene identity" in capsys.readouterr().err


def test_one_scene_opening_produces_one_asset_index_ingestion_pass(tmp_path, capsys):
    impl, _scene, _source, _mesh_index, output = _fake_impl(tmp_path)
    calls = []
    original = _freshener_result(output, calls)

    assert coalescer.run_coalesced(impl, original, _argv(output)) == 0
    first = json.loads(capsys.readouterr().out)
    assert first["cache_hit"] is False
    assert first["asset_ingestion"] == "rebuilt"

    assert coalescer.run_coalesced(impl, original, _argv(output)) == 0
    second = json.loads(capsys.readouterr().out)
    assert second["cache_hit"] is True
    assert second["asset_ingestion"] == "cache_reused"
    assert len(calls) == 1


def test_browser_only_transform_edit_does_not_regenerate_before_save(tmp_path, capsys):
    impl, _scene, _source, _mesh_index, output = _fake_impl(tmp_path)
    calls = []
    original = _freshener_result(output, calls)

    assert coalescer.run_coalesced(impl, original, _argv(output)) == 0
    capsys.readouterr()
    # A local Web3D transform changes browser edit state only. No authored file
    # changes here, so an accidental duplicate refresh must remain a cache hit.
    browser_only_transform = {"xyz": [0.55, -0.31, 0.19], "rpy": [0.0, 0.0, 0.4]}
    assert browser_only_transform["xyz"][0] == 0.55

    assert coalescer.run_coalesced(impl, original, _argv(output)) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["cache_hit"] is True
    assert len(calls) == 1


def test_external_file_change_triggers_exactly_one_new_refresh(tmp_path, capsys):
    impl, _scene, source, _mesh_index, output = _fake_impl(tmp_path)
    calls = []
    original = _freshener_result(output, calls)

    assert coalescer.run_coalesced(impl, original, _argv(output)) == 0
    capsys.readouterr()
    source.write_text("scene: demo_scene\nrevision: 2\n", encoding="utf-8")

    assert coalescer.run_coalesced(impl, original, _argv(output)) == 0
    changed = json.loads(capsys.readouterr().out)
    assert changed["cache_hit"] is False

    assert coalescer.run_coalesced(impl, original, _argv(output)) == 0
    follower = json.loads(capsys.readouterr().out)
    assert follower["cache_hit"] is True
    assert len(calls) == 2


def test_newer_revision_replaces_preparation_that_finishes_stale(tmp_path, capsys):
    impl, _scene, source, _mesh_index, output = _fake_impl(tmp_path)
    calls = []
    changed = False

    def mutate_once():
        nonlocal changed
        if not changed:
            source.write_text("scene: demo_scene\nrevision: newer\n", encoding="utf-8")
            changed = True

    assert coalescer.run_coalesced(
        impl,
        _freshener_result(output, calls, mutate=mutate_once),
        _argv(output),
    ) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["preparation_attempt"] == 2
    assert payload["superseded_preparation_count"] == 1
    assert len(calls) == 2


def test_force_refresh_bypasses_revision_cache(tmp_path, capsys):
    impl, _scene, _source, _mesh_index, output = _fake_impl(tmp_path)
    calls = []
    original = _freshener_result(output, calls)

    assert coalescer.run_coalesced(impl, original, _argv(output)) == 0
    capsys.readouterr()
    assert coalescer.run_coalesced(impl, original, _argv(output, "--force")) == 0
    forced = json.loads(capsys.readouterr().out)
    assert forced["cache_hit"] is False
    assert len(calls) == 2


def test_qt_request_side_retains_identity_coalescing_and_save_boundary():
    source = (REPO_ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(
        encoding="utf-8"
    )
    assert "context_ready = !request_key.scene_id.isEmpty()" in source
    assert "duplicate_active" in source
    assert "duplicate_pending" in source
    assert "duplicate_preparing" in source
    assert "cancelled_superseded" in source
    assert "request_post_save_product_view_refresh" in source
    assert "A save changes authored inputs" in source
