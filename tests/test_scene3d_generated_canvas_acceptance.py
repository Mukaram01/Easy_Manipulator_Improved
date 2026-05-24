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
            gen_json = Path(cmd[cmd.index("--json-out") + 1])
            gen_json.parent.mkdir(parents=True, exist_ok=True)
            gen_json.write_text(json.dumps({"scene_dir": str(scene_dir)}), encoding="utf-8")
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
    assert artifact["key_counts"]["assembled_preview_item_count"] > 0
    assert artifact["key_counts"]["filtered_visible_candidate_count"] > 0
    assert artifact["key_counts"]["forwarded_to_viewport_count"] > 0
    assert artifact["key_counts"]["viewport_received_count"] > 0
    assert artifact["key_counts"]["rendered_count"] > 0
    assert artifact["key_counts"]["selectable_count"] > 0
    assert artifact["key_counts"]["hierarchy_rows_count"] > 0
    assert Path(artifact["gui_smoke"]["screenshot"]).stat().st_size > 0
