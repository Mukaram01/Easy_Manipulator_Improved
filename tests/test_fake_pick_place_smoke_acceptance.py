from __future__ import annotations

import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import run_fake_pick_place_smoke_acceptance as s
from scripts.supported_scene_catalog import load_supported_scene_catalog, default_catalog_path


def _entry(scene: str):
    _, entries, errors = load_supported_scene_catalog(default_catalog_path(Path.cwd()))
    assert not errors
    return next(e for e in entries if e.scene_name == scene)


def test_full_gripper_sequence_resolves_from_metadata() -> None:
    meta, res, errors = s.resolve(_entry("ur5_2f_test"), Path.cwd())
    assert errors == []
    stages = s.build_stages(res, execute=False)
    assert [x["stage"] for x in stages] == s.STAGES
    assert all(x["status"] == s.PASS for x in stages)
    assert "close command verified" in "\n".join(d for x in stages for d in x["details"])
    assert meta["pick_zone"] == "commissioning_pick_pose"


def test_full_suction_sequence_simulates_attach_release() -> None:
    _, res, errors = s.resolve(_entry("suction_test"), Path.cwd())
    assert errors == []
    stages = s.build_stages(res, execute=False)
    details = "\n".join(d for x in stages for d in x["details"])
    assert all(x["status"] == s.PASS for x in stages)
    assert "simulated suction attach state true" in details
    assert "simulated suction attach state false" in details
    assert "no real I/O" in details


def test_missing_zone_tool_metadata_fails(tmp_path: Path) -> None:
    scene = tmp_path / "scenes" / "bad_scene"
    scene.mkdir(parents=True)
    (scene / "environment.yaml").write_text("task_zones: []\n", encoding="utf-8")
    e = SimpleNamespace(scene_path="scenes/bad_scene", tool="", task_smoke={"pick_zone":"missing"})
    _, _, errors = s.resolve(e, tmp_path)
    assert any("missing task-smoke metadata: planning_group" in x for x in errors)
    assert any("missing task-smoke metadata: tool_type" in x for x in errors)
    assert any("pick zone 'missing' could not be resolved" in x for x in errors)


def test_unreachable_stage_fails_precisely() -> None:
    res = {"pick":[2.0,0.0,0.1], "place":[0.5,0.0,0.1], "bounds":{"x_min":-1,"x_max":1,"y_min":-1,"y_max":1,"z_min":0,"z_max":1}, "tool_type":"parallel_gripper"}
    stages = s.build_stages(res, execute=False)
    failed = [x for x in stages if x["status"] == s.FAIL]
    assert failed
    assert any("pre_pick pose outside workspace bounds" in "\n".join(x["details"]) for x in failed)


def test_real_hardware_execution_rejected() -> None:
    status, blockers, diag = s.fake_active("ros2 launch demo demo.launch.py use_fake_hardware:=false", 1)
    assert status == s.FAIL
    assert "non-fake hardware token" in blockers[0]
    assert diag["terminated"] is True


def test_unavailable_ros_reports_blocked(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(s.shutil, "which", lambda name: None)
    status, blockers, diag = s.fake_active("ros2 launch suction_test demo.launch.py use_fake_hardware:=true", 1)
    assert status == s.BLOCKED
    assert "ros2 executable not found" in blockers[0]
    assert diag["terminated"] is True


def test_process_always_shut_down(monkeypatch: pytest.MonkeyPatch) -> None:
    class Proc:
        pid = 9
        def communicate(self, timeout=None):
            return "", ""
    monkeypatch.setattr(s.shutil, "which", lambda name: "/usr/bin/ros2")
    monkeypatch.setattr(s.subprocess, "Popen", lambda *a, **k: Proc())
    monkeypatch.setattr(s.os, "getpgid", lambda pid: pid)
    killed = []
    monkeypatch.setattr(s.os, "killpg", lambda pgid, sig: killed.append((pgid, sig)))
    monkeypatch.setattr(s.time, "sleep", lambda n: None)
    monkeypatch.setattr(s.subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0, "true", ""))
    status, blockers, diag = s.fake_active("ros2 launch suction_test demo.launch.py use_fake_hardware:=true", 3)
    assert status == s.PASS
    assert blockers == []
    assert diag["terminated"] is True
    assert killed
