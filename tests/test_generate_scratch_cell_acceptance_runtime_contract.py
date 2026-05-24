from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml
import scripts.run_scene3d_generated_canvas_acceptance as scene3d_acceptance

ROOT = Path(__file__).resolve().parents[1]
GEN_SCRIPT = ROOT / "scripts" / "generate_scratch_cell_acceptance.py"
VALIDATOR = ROOT / "scripts" / "validate_cell_definition.py"


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run([sys.executable, *args], capture_output=True, text=True, check=False)


def _extract_cell_template() -> str:
    text = GEN_SCRIPT.read_text(encoding="utf-8")
    start = text.index("CELL_TMPL='''") + len("CELL_TMPL='''")
    end = text.index("'''", start)
    return text[start:end].encode("utf-8").decode("unicode_escape")


def test_scratch_template_validates_with_zero_schema_errors(tmp_path: Path) -> None:
    scene_name = "scratch_validator_pass"
    template = _extract_cell_template().format(scene=scene_name)
    parsed = yaml.safe_load(template)
    cell_definition = tmp_path / "cell_definition.yaml"
    cell_definition.write_text(yaml.safe_dump(parsed, sort_keys=False), encoding="utf-8")
    validate_proc = _run(str(VALIDATOR), str(cell_definition), "--json")
    assert validate_proc.returncode == 0, validate_proc.stdout + validate_proc.stderr

    payload = json.loads(validate_proc.stdout)
    assert payload["errors"] == []


def test_scratch_template_objects_and_task_linkage_are_validator_compliant(tmp_path: Path) -> None:
    scene_name = "scratch_validator_objects"
    template = _extract_cell_template().format(scene=scene_name)
    cell_definition = tmp_path / "cell_definition.yaml"
    cell_definition.write_text(template, encoding="utf-8")
    payload = json.loads(_run(str(VALIDATOR), str(cell_definition), "--json").stdout)

    assert not any("objects[" in err for err in payload["errors"])
    assert not any("task.destinations" in err and "does not exist" in err for err in payload["errors"])


def test_intentionally_broken_definition_reports_multiple_missing_keys(tmp_path: Path) -> None:
    broken = tmp_path / "broken_cell_definition.yaml"
    broken.write_text("schema_version: cell_definition/v1\ncell: {id: bad}\n", encoding="utf-8")

    proc = _run(str(VALIDATOR), str(broken), "--json")
    assert proc.returncode != 0
    payload = json.loads(proc.stdout)

    missing_key_errors = [e for e in payload["errors"] if e.startswith("Missing required top-level key")]
    assert len(missing_key_errors) >= 3


def test_generated_acceptance_artifact_records_package_generation_command(monkeypatch, tmp_path: Path) -> None:
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
            smoke_json.write_text(json.dumps({"counters": {"visible_count": 1, "rendered_count": 1, "selectable_count": 1, "hierarchy_rows": 1}}), encoding="utf-8")
            smoke_png.write_bytes(b"png")
            return 0, "ok", ""
        if "validate_scene3d_runtime_acceptance.py" in cmd_s:
            runtime_json = Path(cmd[cmd.index("--json") + 1])
            runtime_md = Path(cmd[cmd.index("--markdown") + 1])
            runtime_json.write_text("{}", encoding="utf-8")
            runtime_md.write_text("# runtime", encoding="utf-8")
            return 0, "ok", ""
        raise AssertionError(cmd)

    monkeypatch.setattr(scene3d_acceptance, "_run", fake_run)
    monkeypatch.setattr(scene3d_acceptance, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(scene3d_acceptance.sys, "argv", ["prog", "--output-dir", str(out_dir), "--scene-name", scene_name])

    rc = scene3d_acceptance.main()
    assert rc == 0
    artifact = json.loads((out_dir / f"{scene_name}_acceptance.json").read_text(encoding="utf-8"))
    assert "generate_scratch_cell_acceptance.py" in artifact["commands"]["generation"]


def test_generated_scene3d_acceptance_fails_when_any_positive_counter_missing(monkeypatch, tmp_path: Path) -> None:
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
            gen_json.write_text(json.dumps({"scene_dir": str(scene_dir)}), encoding="utf-8")
            return 0, "ok", ""
        if "run_workcell_builder_scene3d_gui_smoke.py" in cmd_s:
            smoke_json = Path(cmd[cmd.index("--output") + 1])
            smoke_png = Path(cmd[cmd.index("--screenshot") + 1])
            smoke_json.write_text(json.dumps({"counters": {"visible_count": 1, "rendered_count": 0, "selectable_count": 1, "hierarchy_rows": 1}}), encoding="utf-8")
            smoke_png.write_bytes(b"png")
            return 0, "ok", ""
        if "validate_scene3d_runtime_acceptance.py" in cmd_s:
            runtime_json = Path(cmd[cmd.index("--json") + 1])
            runtime_md = Path(cmd[cmd.index("--markdown") + 1])
            runtime_json.write_text("{}", encoding="utf-8")
            runtime_md.write_text("# runtime", encoding="utf-8")
            return 0, "ok", ""
        raise AssertionError(cmd)

    monkeypatch.setattr(scene3d_acceptance, "_run", fake_run)
    monkeypatch.setattr(scene3d_acceptance, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(scene3d_acceptance.sys, "argv", ["prog", "--output-dir", str(out_dir), "--scene-name", scene_name])

    rc = scene3d_acceptance.main()
    assert rc == 1
    artifact = json.loads((out_dir / f"{scene_name}_acceptance.json").read_text(encoding="utf-8"))
    assert artifact["status"] == "FAIL"
    assert any("runtime counter not > 0: rendered=0" in b for b in artifact["blockers"])
