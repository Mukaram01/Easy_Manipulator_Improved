import json
from pathlib import Path

import scripts.run_scene3d_generated_canvas_acceptance as target


def test_generated_canvas_acceptance_exports_artifacts(monkeypatch, tmp_path: Path):
    scene_name = "scene3d_generated_canvas_acceptance"
    out_dir = tmp_path / "out"
    gen_root = out_dir / "generated_scenes"
    scene_dir = gen_root / scene_name

    def fake_run(cmd, cwd):
        cmd_s = " ".join(cmd)
        if "generate_scratch_cell_acceptance.py" in cmd_s:
            (scene_dir / "config").mkdir(parents=True, exist_ok=True)
            (scene_dir / "urdf").mkdir(parents=True, exist_ok=True)
            (scene_dir / "launch").mkdir(parents=True, exist_ok=True)
            (scene_dir / "scene_manifest.yaml").write_text("x", encoding="utf-8")
            (scene_dir / "environment_layout.yaml").write_text("x", encoding="utf-8")
            (scene_dir / "config/scene3d_mesh_index.json").write_text("{}", encoding="utf-8")
            (scene_dir / "urdf/scene.urdf.xacro").write_text("x", encoding="utf-8")
            (scene_dir / "launch/demo.launch.py").write_text("x", encoding="utf-8")
            (scene_dir / "package.xml").write_text("x", encoding="utf-8")
            (scene_dir / "CMakeLists.txt").write_text("x", encoding="utf-8")
            (scene_dir / "cell_definition.yaml").write_text("x", encoding="utf-8")
            gen_json = Path(cmd[cmd.index("--json-out") + 1])
            gen_json.parent.mkdir(parents=True, exist_ok=True)
            gen_json.write_text(json.dumps({"scene_dir": str(scene_dir), "run_id": cmd[cmd.index("--run-id") + 1]}), encoding="utf-8")
            return 0, "ok", ""
        if "run_workcell_builder_scene3d_gui_smoke.py" in cmd_s:
            smoke_json = Path(cmd[cmd.index("--output") + 1])
            smoke_png = Path(cmd[cmd.index("--screenshot") + 1])
            smoke_json.write_text(json.dumps({"counters": {"assembled_preview_item_count": 1, "filtered_visible_candidate_count": 1, "forwarded_to_viewport_count": 1, "viewport_received_count": 1, "rendered_count": 1, "selectable_count": 1, "hierarchy_rows": 1}}), encoding="utf-8")
            smoke_png.write_bytes(b"png")
            return 0, "ok", ""
        if "validate_scene3d_runtime_acceptance.py" in cmd_s:
            runtime_json = Path(cmd[cmd.index("--json") + 1])
            runtime_md = Path(cmd[cmd.index("--markdown") + 1])
            runtime_json.write_text("{}", encoding="utf-8")
            runtime_md.write_text("# runtime", encoding="utf-8")
            return 0, "ok", ""
        raise AssertionError(cmd)

    monkeypatch.setattr(target, "_run", fake_run)
    monkeypatch.setattr(target, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(target.sys, "argv", ["prog", "--output-dir", str(out_dir), "--scene-name", scene_name])

    rc = target.main()
    assert rc == 0
    artifact = json.loads((out_dir / f"{scene_name}_acceptance.json").read_text(encoding="utf-8"))
    assert artifact["status"] == "PASS"
    assert artifact["missing_outputs"] == []
    assert artifact["missing_optional_outputs"] == []
    assert artifact["key_counts"]["assembled_preview_item_count"] > 0
    assert artifact["key_counts"]["filtered_visible_candidate_count"] > 0
    assert artifact["key_counts"]["forwarded_to_viewport_count"] > 0
    assert artifact["key_counts"]["viewport_received_count"] > 0
    assert artifact["key_counts"]["rendered_count"] > 0
    assert artifact["key_counts"]["selectable_count"] > 0
    assert artifact["key_counts"]["hierarchy_rows_count"] > 0
    assert Path(artifact["gui_smoke"]["screenshot"]).stat().st_size > 0


def test_generated_canvas_acceptance_allows_optional_geometry_when_runtime_counters_pass(monkeypatch, tmp_path: Path):
    scene_name = "scene3d_generated_canvas_acceptance"
    out_dir = tmp_path / "out"
    gen_root = out_dir / "generated_scenes"
    scene_dir = gen_root / scene_name

    def fake_run(cmd, cwd):
        cmd_s = " ".join(cmd)
        if "generate_scratch_cell_acceptance.py" in cmd_s:
            (scene_dir / "launch").mkdir(parents=True, exist_ok=True)
            (scene_dir / "scene_manifest.yaml").write_text("x", encoding="utf-8")
            (scene_dir / "environment_layout.yaml").write_text("x", encoding="utf-8")
            (scene_dir / "launch/demo.launch.py").write_text("x", encoding="utf-8")
            (scene_dir / "package.xml").write_text("x", encoding="utf-8")
            (scene_dir / "CMakeLists.txt").write_text("x", encoding="utf-8")
            (scene_dir / "cell_definition.yaml").write_text("x", encoding="utf-8")
            gen_json = Path(cmd[cmd.index("--json-out") + 1])
            gen_json.parent.mkdir(parents=True, exist_ok=True)
            gen_json.write_text(json.dumps({"scene_dir": str(scene_dir), "run_id": cmd[cmd.index("--run-id") + 1]}), encoding="utf-8")
            return 0, "ok", ""
        if "run_workcell_builder_scene3d_gui_smoke.py" in cmd_s:
            smoke_json = Path(cmd[cmd.index("--output") + 1])
            smoke_png = Path(cmd[cmd.index("--screenshot") + 1])
            smoke_json.write_text(json.dumps({"counters": {"assembled_preview_item_count": 1, "filtered_visible_candidate_count": 1, "forwarded_to_viewport_count": 1, "viewport_received_count": 1, "rendered_count": 1, "selectable_count": 1, "hierarchy_rows": 1}}), encoding="utf-8")
            smoke_png.write_bytes(b"png")
            return 0, "ok", ""
        if "validate_scene3d_runtime_acceptance.py" in cmd_s:
            runtime_json = Path(cmd[cmd.index("--json") + 1])
            runtime_md = Path(cmd[cmd.index("--markdown") + 1])
            runtime_json.write_text("{}", encoding="utf-8")
            runtime_md.write_text("# runtime", encoding="utf-8")
            return 0, "ok", ""
        raise AssertionError(cmd)

    monkeypatch.setattr(target, "_run", fake_run)
    monkeypatch.setattr(target, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(target.sys, "argv", ["prog", "--output-dir", str(out_dir), "--scene-name", scene_name])
    rc = target.main()
    artifact = json.loads((out_dir / f"{scene_name}_acceptance.json").read_text(encoding="utf-8"))
    assert rc == 0
    assert artifact["status"] == "PASS"
    assert artifact["missing_outputs"] == []
    assert sorted(artifact["missing_optional_outputs"]) == ["config/scene3d_mesh_index.json", "urdf/scene.urdf.xacro"]


def test_generated_canvas_acceptance_failure_reports_schema_blockers(monkeypatch, tmp_path: Path):
    scene_name = 'scene3d_generated_canvas_acceptance'
    out_dir = tmp_path / 'out'

    def fake_run(cmd, cwd):
        if 'generate_scratch_cell_acceptance.py' in ' '.join(cmd):
            gen_json = Path(cmd[cmd.index('--json-out') + 1])
            gen_json.parent.mkdir(parents=True, exist_ok=True)
            gen_json.write_text(json.dumps({
                'failure_step': 'schema_preflight',
                'failure_summary': 'schema invalid',
                'blockers': ['schema_preflight: Missing required top-level key: objects', 'schema_preflight: Missing required top-level key: commissioning'],
                'schema_preflight': {
                    'validator_command': 'python3 scripts/validate_cell_definition.py /tmp/cell.yaml --json',
                    'schema_blockers': ['Missing required top-level key: objects', 'Missing required top-level key: commissioning'],
                    'next_failed_step': 'schema_preflight'
                },
                'run_id': cmd[cmd.index("--run-id") + 1],
                'generated_files': ['cell_definition.yaml'],
                'missing_files': ['scene_manifest.yaml', 'package.xml', 'CMakeLists.txt', 'launch/demo.launch.py']
            }), encoding='utf-8')
            return 1, '', 'failed'
        raise AssertionError(cmd)

    monkeypatch.setattr(target, '_run', fake_run)
    monkeypatch.setattr(target.sys, 'argv', ['prog', '--output-dir', str(out_dir), '--scene-name', scene_name])
    rc = target.main()
    assert rc == 1


def test_generation_crash_reports_missing_or_stale(monkeypatch, tmp_path: Path, capsys):
    out_dir = tmp_path / "out"

    def fake_run(cmd, cwd):
        if "generate_scratch_cell_acceptance.py" in " ".join(cmd):
            return 2, "", "traceback"
        raise AssertionError(cmd)

    monkeypatch.setattr(target, "_run", fake_run)
    monkeypatch.setattr(target.sys, "argv", ["prog", "--output-dir", str(out_dir)])
    rc = target.main()
    assert rc == 2
    payload = json.loads(capsys.readouterr().out)
    assert "generation_report_missing_or_stale" in payload["blockers"]
    assert payload["step"] == "generation"


def test_stale_generation_json_not_reused(monkeypatch, tmp_path: Path, capsys):
    out_dir = tmp_path / "out"

    def fake_run(cmd, cwd):
        if "generate_scratch_cell_acceptance.py" in " ".join(cmd):
            gen_json = Path(cmd[cmd.index("--json-out") + 1])
            gen_json.parent.mkdir(parents=True, exist_ok=True)
            gen_json.write_text(json.dumps({"scene_dir": str(out_dir / "x"), "run_id": "old-run"}), encoding="utf-8")
            return 0, "ok", ""
        raise AssertionError(cmd)

    monkeypatch.setattr(target, "_run", fake_run)
    monkeypatch.setattr(target.sys, "argv", ["prog", "--output-dir", str(out_dir)])
    rc = target.main()
    assert rc == 1
    payload = json.loads(capsys.readouterr().out)
    assert payload["blockers"] == ["generation_report_missing_or_stale"]


def test_generated_canvas_acceptance_requires_core_package_contract_outputs(monkeypatch, tmp_path: Path):
    scene_name = "scene3d_generated_canvas_acceptance"
    out_dir = tmp_path / "out"
    scene_dir = out_dir / "generated_scenes" / scene_name

    def fake_run(cmd, cwd):
        cmd_s = " ".join(cmd)
        if "generate_scratch_cell_acceptance.py" in cmd_s:
            (scene_dir / "launch").mkdir(parents=True, exist_ok=True)
            (scene_dir / "scene_manifest.yaml").write_text("x", encoding="utf-8")
            (scene_dir / "launch/demo.launch.py").write_text("x", encoding="utf-8")
            (scene_dir / "package.xml").write_text("x", encoding="utf-8")
            (scene_dir / "CMakeLists.txt").write_text("x", encoding="utf-8")
            (scene_dir / "cell_definition.yaml").write_text("x", encoding="utf-8")
            gen_json = Path(cmd[cmd.index("--json-out") + 1])
            gen_json.parent.mkdir(parents=True, exist_ok=True)
            gen_json.write_text(json.dumps({"scene_dir": str(scene_dir), "run_id": cmd[cmd.index("--run-id") + 1]}), encoding="utf-8")
            return 0, "ok", ""
        if "run_workcell_builder_scene3d_gui_smoke.py" in cmd_s:
            smoke_json = Path(cmd[cmd.index("--output") + 1])
            smoke_png = Path(cmd[cmd.index("--screenshot") + 1])
            smoke_json.write_text(json.dumps({"counters": {"assembled_preview_item_count": 1, "filtered_visible_candidate_count": 1, "forwarded_to_viewport_count": 1, "viewport_received_count": 1, "rendered_count": 1, "selectable_count": 1, "hierarchy_rows": 1}}), encoding="utf-8")
            smoke_png.write_bytes(b"png")
            return 0, "ok", ""
        if "validate_scene3d_runtime_acceptance.py" in cmd_s:
            runtime_json = Path(cmd[cmd.index("--json") + 1])
            runtime_md = Path(cmd[cmd.index("--markdown") + 1])
            runtime_json.write_text("{}", encoding="utf-8")
            runtime_md.write_text("# runtime", encoding="utf-8")
            return 0, "ok", ""
        raise AssertionError(cmd)

    monkeypatch.setattr(target, "_run", fake_run)
    monkeypatch.setattr(target, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(target.sys, "argv", ["prog", "--output-dir", str(out_dir), "--scene-name", scene_name])
    rc = target.main()
    artifact = json.loads((out_dir / f"{scene_name}_acceptance.json").read_text(encoding="utf-8"))
    assert rc == 1
    assert artifact["status"] == "FAIL"
    assert artifact["missing_outputs"] == ["environment_layout.yaml"]
