import argparse
from pathlib import Path

import pytest

import scripts.run_scene3d_generated_canvas_acceptance as generated
import scripts.run_workcell_builder_scene3d_gui_smoke as smoke
import scripts.validate_scene3d_runtime_acceptance as runtime


def test_gui_smoke_all_scenes_output_dir_without_output_parses(monkeypatch, tmp_path: Path):
    monkeypatch.setattr(smoke.sys, "argv", ["prog", "--all-scenes", "--output-dir", str(tmp_path)])

    def fake_discover(_repo_root):
        return []

    monkeypatch.setattr(smoke, "_discover_scene_targets", fake_discover)
    assert smoke.main() == 0


def test_gui_smoke_single_scene_requires_output(monkeypatch):
    monkeypatch.setattr(smoke.sys, "argv", ["prog", "--scene", "ur5_2f_test"])
    with pytest.raises(SystemExit, match="--output is required unless --all-scenes is used"):
        smoke.main()


def test_runtime_acceptance_all_scenes_smoke_dir_parses(monkeypatch, tmp_path: Path):
    out_json = tmp_path / "out.json"
    out_md = tmp_path / "out.md"
    monkeypatch.setattr(
        runtime.sys,
        "argv",
        ["prog", "--all-scenes", "--smoke-dir", str(tmp_path), "--json", str(out_json), "--markdown", str(out_md)],
    )
    assert runtime.main() in (0, 1)
    assert out_json.exists()


def test_runtime_acceptance_all_scenes_smoke_json_fails_clearly(monkeypatch):
    parser_errors = []

    def fake_error(msg):
        parser_errors.append(msg)
        raise SystemExit(2)

    monkeypatch.setattr(argparse.ArgumentParser, "error", lambda self, msg: fake_error(msg))
    monkeypatch.setattr(runtime.sys, "argv", ["prog", "--all-scenes", "--smoke-json", "x.json"])
    with pytest.raises(SystemExit):
        runtime.main()
    assert parser_errors[-1] == "--smoke-json is only valid for single-scene mode; use --smoke-dir with --all-scenes"


def test_generated_canvas_acceptance_accepts_repo_root_and_forwards_timeout(monkeypatch, tmp_path: Path):
    calls = []
    out_dir = tmp_path / "out"

    def fake_run(cmd, cwd):
        calls.append((cmd, cwd))
        cmd_s = " ".join(cmd)
        if "generate_scratch_cell_acceptance.py" in cmd_s:
            scene_name = cmd[cmd.index("--scene-name") + 1]
            scene_dir = out_dir / "generated_scenes" / scene_name
            (scene_dir / "config").mkdir(parents=True, exist_ok=True)
            (scene_dir / "urdf").mkdir(parents=True, exist_ok=True)
            (scene_dir / "launch").mkdir(parents=True, exist_ok=True)
            for rel in [
                "scene_manifest.yaml",
                "environment_layout.yaml",
                "config/scene3d_mesh_index.json",
                "urdf/scene.urdf.xacro",
                "launch/demo.launch.py",
                "package.xml",
                "CMakeLists.txt",
            ]:
                (scene_dir / rel).write_text("x", encoding="utf-8")
            gen_json = Path(cmd[cmd.index("--json-out") + 1])
            gen_json.parent.mkdir(parents=True, exist_ok=True)
            gen_json.write_text('{"scene_dir": "%s"}' % scene_dir, encoding="utf-8")
            return 0, "", ""
        if "run_workcell_builder_scene3d_gui_smoke.py" in cmd_s:
            Path(cmd[cmd.index("--output") + 1]).write_text('{"counters":{"visible_count":1,"rendered_count":1,"selectable_count":1,"hierarchy_rows":1}}', encoding="utf-8")
            Path(cmd[cmd.index("--screenshot") + 1]).write_bytes(b"png")
            return 0, "", ""
        if "validate_scene3d_runtime_acceptance.py" in cmd_s:
            Path(cmd[cmd.index("--json") + 1]).write_text("{}", encoding="utf-8")
            Path(cmd[cmd.index("--markdown") + 1]).write_text("# ok\n", encoding="utf-8")
            return 0, "", ""
        return 0, "", ""

    monkeypatch.setattr(generated, "_run", fake_run)
    monkeypatch.setattr(generated.sys, "argv", ["prog", "--repo-root", str(tmp_path), "--output-dir", str(out_dir), "--timeout-sec", "45"])
    assert generated.main() == 0

    smoke_call = next(cmd for cmd, _ in calls if "run_workcell_builder_scene3d_gui_smoke.py" in " ".join(cmd))
    assert "--timeout-sec" in smoke_call
    assert smoke_call[smoke_call.index("--timeout-sec") + 1] == "45.0"
