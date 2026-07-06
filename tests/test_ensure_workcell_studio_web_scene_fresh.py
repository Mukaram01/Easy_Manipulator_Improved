import importlib.util
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts/ensure_workcell_studio_web_scene_fresh.py"
spec = importlib.util.spec_from_file_location("ensure_workcell_studio_web_scene_fresh", SCRIPT_PATH)
ensure = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = ensure
spec.loader.exec_module(ensure)


def _write(path: Path, text: str = "x") -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def _touch(path: Path, when: int) -> None:
    os.utime(path, (when, when))


def _minimal_repo(tmp_path, monkeypatch):
    repo = tmp_path / "repo"
    scene = repo / "scenes" / "demo_scene"
    build = repo / "build" / "workcell_studio_web_scene"
    extractor = _write(
        repo / "scripts" / "extract_scene_urdf_visual_mesh_index.py",
        'EXTRACTOR_VERSION = "expected-v1"\n',
    )
    exporter = _write(repo / "scripts" / "export_workcell_studio_web_scene.py", "# exporter\n")
    source = _write(scene / "scene_manifest.yaml", "scene: demo\n")
    (scene / "generated").mkdir(parents=True)

    monkeypatch.setattr(ensure, "REPO_ROOT", repo)
    monkeypatch.setattr(ensure, "SCENES_ROOT", repo / "scenes")
    monkeypatch.setattr(ensure, "WEB_BUILD_ROOT", build)
    monkeypatch.setattr(ensure, "ASSET_BUILD_ROOT", build / "assets")
    monkeypatch.setattr(ensure, "SCENE_INPUT_RELS", ("scene_manifest.yaml",))
    monkeypatch.setattr(
        ensure,
        "GENERATOR_INPUT_RELS",
        (
            "scripts/extract_scene_urdf_visual_mesh_index.py",
            "scripts/export_workcell_studio_web_scene.py",
        ),
    )
    monkeypatch.setattr(ensure, "real_xacro_is_discoverable", lambda: False)
    return repo, scene, source, extractor, exporter, build


def test_missing_mesh_index_runs_extractor_then_exporter(tmp_path, monkeypatch):
    _repo, scene, _source, _extractor, _exporter, build = _minimal_repo(tmp_path, monkeypatch)
    output = build / "demo_scene.web_scene.json"
    calls = []

    def fake_run(command):
        calls.append(command)
        script = command[1]
        if script.endswith("extract_scene_urdf_visual_mesh_index.py"):
            _write(
                scene / "generated" / "scene_visual_mesh_index.json",
                json.dumps({"extractor_version": "expected-v1", "visual_items": []}),
            )
        elif script.endswith("export_workcell_studio_web_scene.py"):
            _write(output, json.dumps({"schema_version": "workcell_studio_web_scene/v1"}))

    monkeypatch.setattr(ensure, "run_checked", fake_run)

    assert ensure.main(["--scene", str(scene), "--output", str(output)]) == 0
    assert [Path(call[1]).name for call in calls] == [
        "extract_scene_urdf_visual_mesh_index.py",
        "export_workcell_studio_web_scene.py",
    ]


def test_older_mesh_index_than_source_or_generator_input_triggers_regeneration(tmp_path, monkeypatch):
    _repo, scene, source, _extractor, _exporter, build = _minimal_repo(tmp_path, monkeypatch)
    mesh_index = _write(
        scene / "generated" / "scene_visual_mesh_index.json",
        json.dumps({"extractor_version": "expected-v1", "visual_items": []}),
    )
    output = _write(build / "demo_scene.web_scene.json", "{}")
    _touch(mesh_index, 100)
    _touch(source, 200)
    _touch(output, 300)
    calls = []

    def fake_run(command):
        calls.append(Path(command[1]).name)
        if command[1].endswith("extract_scene_urdf_visual_mesh_index.py"):
            _write(mesh_index, json.dumps({"extractor_version": "expected-v1", "visual_items": []}))

    monkeypatch.setattr(ensure, "run_checked", fake_run)

    assert ensure.main(["--scene", "demo_scene", "--output", str(output)]) == 0
    assert calls == [
        "extract_scene_urdf_visual_mesh_index.py",
        "export_workcell_studio_web_scene.py",
    ]


def test_older_web_scene_than_mesh_index_or_source_input_triggers_export_only(tmp_path, monkeypatch):
    _repo, scene, source, _extractor, _exporter, build = _minimal_repo(tmp_path, monkeypatch)
    mesh_index = _write(
        scene / "generated" / "scene_visual_mesh_index.json",
        json.dumps({"extractor_version": "expected-v1", "visual_items": []}),
    )
    output = _write(build / "demo_scene.web_scene.json", "{}")
    _touch(output, 100)
    _touch(mesh_index, 300)
    _touch(source, 250)
    for generator_input in (
        scene.parents[1] / "scripts" / "extract_scene_urdf_visual_mesh_index.py",
        scene.parents[1] / "scripts" / "export_workcell_studio_web_scene.py",
    ):
        _touch(generator_input, 200)
    calls = []
    monkeypatch.setattr(
        ensure,
        "run_checked",
        lambda command: calls.append(Path(command[1]).name),
    )

    assert ensure.main(["--scene", "demo_scene", "--output", str(output)]) == 0
    assert calls == ["export_workcell_studio_web_scene.py"]


def test_fresh_mesh_web_scene_and_staged_assets_do_not_rerun(tmp_path, monkeypatch):
    _repo, scene, source, extractor, exporter, build = _minimal_repo(tmp_path, monkeypatch)
    mesh_index = _write(
        scene / "generated" / "scene_visual_mesh_index.json",
        json.dumps({"extractor_version": "expected-v1", "visual_items": []}),
    )
    output = _write(build / "demo_scene.web_scene.json", "{}")
    asset_dir = build / "assets" / "demo_scene"
    asset_file = _write(asset_dir / "mesh.stl", "solid mesh\nendsolid mesh\n")
    for path in (source, extractor, exporter):
        _touch(path, 100)
    for path in (mesh_index, output, asset_dir, asset_file):
        _touch(path, 300)
    calls = []
    monkeypatch.setattr(ensure, "run_checked", lambda command: calls.append(command))

    assert ensure.main(["--scene", "demo_scene", "--output", str(output), "--stage-assets"]) == 0
    assert calls == []


def test_mismatched_extractor_version_marks_mesh_index_stale(tmp_path, monkeypatch):
    _repo, scene, _source, _extractor, _exporter, build = _minimal_repo(tmp_path, monkeypatch)
    mesh_index = _write(
        scene / "generated" / "scene_visual_mesh_index.json",
        json.dumps({"extractor_version": "old-v0", "visual_items": []}),
    )
    output = _write(build / "demo_scene.web_scene.json", "{}")
    for path in (mesh_index, output):
        _touch(path, 300)
    calls = []

    def fake_run(command):
        calls.append(Path(command[1]).name)
        if command[1].endswith("extract_scene_urdf_visual_mesh_index.py"):
            _write(mesh_index, json.dumps({"extractor_version": "expected-v1", "visual_items": []}))

    monkeypatch.setattr(ensure, "run_checked", fake_run)

    assert ensure.main(["--scene", "demo_scene", "--output", str(output)]) == 0
    assert calls == [
        "extract_scene_urdf_visual_mesh_index.py",
        "export_workcell_studio_web_scene.py",
    ]


def test_sourced_real_xacro_requires_real_expansion_status(tmp_path, monkeypatch):
    _repo, scene, _source, _extractor, _exporter, build = _minimal_repo(tmp_path, monkeypatch)
    monkeypatch.setattr(ensure, "real_xacro_is_discoverable", lambda: True)
    output = build / "demo_scene.web_scene.json"
    calls = []

    def fake_run(command):
        calls.append(command)
        if command[1].endswith("extract_scene_urdf_visual_mesh_index.py"):
            assert "--require-xacro" in command
            _write(
                scene / "generated" / "scene_visual_mesh_index.json",
                json.dumps({
                    "extractor_version": "expected-v1",
                    "extraction_mode": "real_xacro_expanded",
                    "xacro_real_command_succeeded": True,
                    "visual_items": [],
                }),
            )
        elif command[1].endswith("export_workcell_studio_web_scene.py"):
            _write(output, "{}")

    monkeypatch.setattr(ensure, "run_checked", fake_run)

    assert ensure.main(["--scene", str(scene), "--output", str(output), "--force"]) == 0
    assert [Path(call[1]).name for call in calls] == [
        "extract_scene_urdf_visual_mesh_index.py",
        "export_workcell_studio_web_scene.py",
    ]


def test_ur5_mesh_index_regeneration_has_no_legacy_static_fallback_transform_statuses(tmp_path):
    scene = Path("scenes/ur5_2f_test")
    mesh_index = scene / "generated" / "scene_visual_mesh_index.json"
    output = tmp_path / "ur5_2f_test.web_scene.json"
    subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--scene",
            str(scene),
            "--output",
            str(output),
            "--stage-assets",
            "--force",
        ],
        text=True,
        capture_output=True,
        check=True,
    )
    copied = tmp_path / "scene_visual_mesh_index.json"
    shutil.copy2(mesh_index, copied)
    payload = json.loads(copied.read_text(encoding="utf-8"))

    forbidden = "legacy_static_fallback_resolved_ur5_mesh_pose"
    assert payload.get("extractor_version")
    for item in payload.get("visual_items", []):
        assert item.get("baked_world_visual_transform_source") != forbidden
        assert item.get("transform_status") != forbidden
        assert item.get("link_transform_status") != forbidden


def test_real_script_reports_fresh_for_temporary_minimal_scene(tmp_path):
    scene = tmp_path / "minimal_scene"
    _write(scene / "scene_manifest.yaml", "scene: minimal\n")
    extractor_version = ensure.read_extractor_version(Path("scripts/extract_scene_urdf_visual_mesh_index.py"))
    mesh_index = _write(
        scene / "generated" / "scene_visual_mesh_index.json",
        json.dumps({"extractor_version": extractor_version, "visual_items": []}),
    )
    output = _write(tmp_path / "build" / "minimal_scene.web_scene.json", "{}")
    future = int(time.time()) + 60
    _touch(mesh_index, future)
    _touch(output, future)

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "--scene", str(scene), "--output", str(output)],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "is fresh" in result.stdout
    assert "Refreshing" not in result.stdout


def test_real_ur5_2f_freshness_helper_recreates_missing_mesh_index(tmp_path):
    scene = Path("scenes/ur5_2f_test")
    mesh_index = scene / "generated" / "scene_visual_mesh_index.json"
    backup = tmp_path / "scene_visual_mesh_index.backup.json"
    had_existing = mesh_index.exists()
    if had_existing:
        backup.write_bytes(mesh_index.read_bytes())
    mesh_index.unlink(missing_ok=True)

    try:
        output = tmp_path / "ur5_2f_test.web_scene.json"
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT_PATH),
                "--scene",
                str(scene),
                "--output",
                str(output),
                "--stage-assets",
                "--force",
            ],
            text=True,
            capture_output=True,
            check=False,
        )

        assert result.returncode == 0, result.stderr
        assert mesh_index.exists(), "freshness helper should recreate the local scene visual mesh index"
        assert output.exists(), "freshness helper should also export the requested web scene"
        payload = json.loads(mesh_index.read_text(encoding="utf-8"))
        assert payload.get("extractor_version")
    finally:
        if had_existing:
            mesh_index.write_bytes(backup.read_bytes())
        else:
            mesh_index.unlink(missing_ok=True)
