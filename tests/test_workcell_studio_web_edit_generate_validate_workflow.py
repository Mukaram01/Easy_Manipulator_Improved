import importlib.util
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
WORKFLOW_PATH = REPO_ROOT / "scripts" / "run_workcell_studio_web_edit_workflow.py"


def _load_workflow():
    spec = importlib.util.spec_from_file_location("workflow_under_test", WORKFLOW_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def workflow(monkeypatch):
    module = _load_workflow()
    monkeypatch.setattr(module, "build_web_scene", lambda scene: {"schema_version": "workcell_studio_web_scene/v1", "scene_id": scene.name, "items": []})
    return module


@pytest.fixture()
def scene(tmp_path):
    scene_dir = tmp_path / "scenes" / "ur5_2f_test"
    (scene_dir / "generated").mkdir(parents=True)
    return scene_dir


def _patch(tmp_path):
    patch_path = tmp_path / "patch.json"
    patch_path.write_text('{"schema_version":"workcell_studio_web_scene_edit_patch/v1"}\n', encoding="utf-8")
    return patch_path


class RunRecorder:
    def __init__(self, returncodes=None):
        self.commands = []
        self.returncodes = list(returncodes or [])

    def __call__(self, cmd, **kwargs):
        self.commands.append([str(part) for part in cmd])
        rc = self.returncodes.pop(0) if self.returncodes else 0
        return subprocess.CompletedProcess(cmd, rc, stdout=f"stub rc={rc}\n", stderr="")

    def joined(self):
        return "\n".join(" ".join(cmd) for cmd in self.commands)


def test_default_guided_workflow_does_not_generate_or_validate_automatically(workflow, scene, tmp_path, monkeypatch, capsys):
    recorder = RunRecorder([0])
    monkeypatch.setattr(workflow.subprocess, "run", recorder)
    monkeypatch.setattr(workflow, "_validate_patch", lambda before, patch_path: (True, {"scene_id": scene.name}, []))

    rc = workflow.main(["--scene", str(scene), "--patch", str(_patch(tmp_path)), "--output-dir", str(tmp_path / "out")])

    out = capsys.readouterr().out
    assert rc == 0
    assert "generation requested: False" in out
    assert "validation requested: False" in out
    assert "write/apply result: SKIPPED" in out
    assert "scene generation" not in out
    assert "selected-scene validation" not in out
    assert len(recorder.commands) == 1


def test_generate_and_validate_triggers_generation_validation_path(workflow, scene, tmp_path, monkeypatch, capsys):
    recorder = RunRecorder([0, 0])
    monkeypatch.setattr(workflow.subprocess, "run", recorder)

    rc = workflow.main(["--scene", str(scene), "--generate-and-validate", "--output-dir", str(tmp_path / "out")])

    out = capsys.readouterr().out
    assert rc == 0
    assert "generation requested: True" in out
    assert "validation requested: True" in out
    assert "== scene generation ==" in out
    assert "== selected-scene validation ==" in out
    assert any("validate_builder_generated_scene.py" in " ".join(cmd) for cmd in recorder.commands)


def test_generation_not_attempted_if_patch_validation_fails(workflow, scene, tmp_path, monkeypatch):
    recorder = RunRecorder()
    monkeypatch.setattr(workflow.subprocess, "run", recorder)
    monkeypatch.setattr(workflow, "_validate_patch", lambda before, patch_path: (False, None, ["bad patch"]))

    rc = workflow.main(["--scene", str(scene), "--patch", str(_patch(tmp_path)), "--write", "--generate-and-validate"])

    assert rc == 1
    assert "scene generation" not in recorder.joined()
    assert recorder.commands == []


def test_generation_not_attempted_if_patch_apply_fails(workflow, scene, tmp_path, monkeypatch):
    recorder = RunRecorder([23])
    monkeypatch.setattr(workflow.subprocess, "run", recorder)
    monkeypatch.setattr(workflow, "_validate_patch", lambda before, patch_path: (True, {"scene_id": scene.name}, []))

    rc = workflow.main(["--scene", str(scene), "--patch", str(_patch(tmp_path)), "--write", "--generate-and-validate"])

    assert rc == 23
    assert len(recorder.commands) == 1
    assert "apply_workcell_studio_web_scene_edit_patch.py" in recorder.joined()
    assert "scene generation" not in recorder.joined()


def test_generation_not_attempted_if_persistence_verification_fails(workflow, scene, tmp_path, monkeypatch):
    recorder = RunRecorder([0, 0, 24])
    monkeypatch.setattr(workflow.subprocess, "run", recorder)
    monkeypatch.setattr(workflow, "_validate_patch", lambda before, patch_path: (True, {"scene_id": scene.name}, []))

    rc = workflow.main(["--scene", str(scene), "--patch", str(_patch(tmp_path)), "--write", "--generate-and-validate"])

    assert rc == 24
    assert "verify_workcell_studio_web_scene_edit_persistence.py" in recorder.joined()
    assert "scene generation" not in recorder.joined()


def test_validation_failure_returns_nonzero_and_emits_clear_failure_summary(workflow, scene, monkeypatch, capsys):
    recorder = RunRecorder([0, 42])
    monkeypatch.setattr(workflow.subprocess, "run", recorder)

    rc = workflow.main(["--scene", str(scene), "--generate-and-validate"])

    out = capsys.readouterr().out
    assert rc == 42
    assert "validation command/result:" in out
    assert "-> FAIL" in out
    assert "Review the failed command output above" in out


def test_generate_only_current_scene_path_works_when_supported(workflow, scene, monkeypatch, capsys):
    recorder = RunRecorder([0])
    monkeypatch.setattr(workflow.subprocess, "run", recorder)

    rc = workflow.main(["--scene", str(scene), "--generate"])

    out = capsys.readouterr().out
    assert rc == 0
    assert "patch applied/skipped: SKIPPED (no --patch provided)" in out
    assert "== scene generation ==" in out
    assert "validation command/result: SKIPPED" in out
    assert len(recorder.commands) == 1


def test_output_summary_includes_generated_and_readiness_paths_where_applicable(workflow, scene, monkeypatch, capsys):
    recorder = RunRecorder([0, 0])
    monkeypatch.setattr(workflow.subprocess, "run", recorder)

    rc = workflow.main(["--scene", str(scene), "--generate", "--run-readiness"])

    out = capsys.readouterr().out
    assert rc == 0
    assert "generated output paths:" in out
    assert str(scene / "generated" / "cell_definition.yaml") in out
    assert "readiness output path:" in out
    assert "readiness output paths:" in out
    assert "scene_readiness_summary.json" in out


def test_no_ros_rviz_moveit_gazebo_launch_command_is_introduced():
    source = WORKFLOW_PATH.read_text(encoding="utf-8")
    forbidden = ("ros2 launch", "launch_rviz", "rviz2", "move_group", "gazebo", "ign gazebo")
    for token in forbidden:
        assert token not in source


def test_no_qt_scene3d_visual_topology_screenshot_wording_is_reintroduced():
    source = WORKFLOW_PATH.read_text(encoding="utf-8")
    forbidden = ("Qt Scene3D visual", "topology", "screenshot")
    for token in forbidden:
        assert token not in source
