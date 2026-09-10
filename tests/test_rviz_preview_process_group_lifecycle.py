from __future__ import annotations

import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = REPO_ROOT / "scripts" / "workcell_preview_process_group.py"
RUNNER_CPP = REPO_ROOT / "workcell_builder" / "workcell_builder" / "src_rviz_preview_runner.cpp"
META_RE = re.compile(r"leader_pid=(\d+) pgid=(\d+) sid=(\d+)")

pytestmark = pytest.mark.skipif(os.name != "posix", reason="R1.3 preview ownership is POSIX-only")


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _start_supervised_python(code: str):
    process = subprocess.Popen(
        [sys.executable, str(SUPERVISOR), "--", sys.executable, "-c", code],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    metadata = process.stderr.readline().strip()
    match = META_RE.search(metadata)
    assert match, metadata
    leader_pid, pgid, sid = (int(value) for value in match.groups())
    return process, leader_pid, pgid, sid


def _cleanup(process: subprocess.Popen, pgid: int) -> None:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1)
    if _group_alive(pgid):
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def test_preview_group_is_isolated_and_stop_removes_descendants_without_touching_unrelated_process():
    code = (
        "import subprocess,sys,time; "
        "subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); "
        "print('preview-ready', flush=True); time.sleep(60)"
    )
    process, leader_pid, pgid, sid = _start_supervised_python(code)
    unrelated = subprocess.Popen(["sleep", "30"], start_new_session=True)
    try:
        assert process.stdout.readline().strip() == "preview-ready"
        assert leader_pid == pgid == sid
        assert pgid != os.getpgrp()
        assert _group_alive(pgid)

        # QProcess::terminate() is SIGTERM on Linux. The supervisor converts it
        # into graceful SIGINT for the isolated preview group and waits for every
        # descendant before the tracked supervisor exits.
        process.terminate()
        assert process.wait(timeout=5) == 0
        assert not _group_alive(pgid)
        assert unrelated.poll() is None

        diagnostics = process.stderr.read()
        assert "signal=SIGINT" in diagnostics
        assert "preview group stopped" in diagnostics
    finally:
        _cleanup(process, pgid)
        if unrelated.poll() is None:
            unrelated.terminate()
            unrelated.wait(timeout=2)


def test_preview_group_escalates_to_term_and_kill_when_children_ignore_graceful_signals():
    child = (
        "import signal,time; "
        "signal.signal(signal.SIGINT, signal.SIG_IGN); "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "print('child-ready', flush=True); time.sleep(60)"
    )
    code = (
        "import subprocess,sys,signal,time; "
        "signal.signal(signal.SIGINT, signal.SIG_IGN); "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        f"subprocess.Popen([sys.executable,'-c',{child!r}]); "
        "print('parent-ready', flush=True); time.sleep(60)"
    )
    process, _leader_pid, pgid, _sid = _start_supervised_python(code)
    try:
        ready = {process.stdout.readline().strip(), process.stdout.readline().strip()}
        assert ready == {"parent-ready", "child-ready"}
        process.terminate()
        assert process.wait(timeout=6) == 0
        diagnostics = process.stderr.read()
        assert "signal=SIGINT" in diagnostics
        assert "signal=SIGTERM" in diagnostics
        assert "signal=SIGKILL" in diagnostics
        assert not _group_alive(pgid)
    finally:
        _cleanup(process, pgid)


def test_leader_exit_with_live_descendant_is_cleaned_and_not_reported_as_success():
    child = (
        "import signal,time; "
        "signal.signal(signal.SIGINT, signal.SIG_IGN); "
        "time.sleep(60)"
    )
    code = f"import subprocess,sys; subprocess.Popen([sys.executable,'-c',{child!r}])"
    process, _leader_pid, pgid, _sid = _start_supervised_python(code)
    try:
        assert process.wait(timeout=6) == 75
        diagnostics = process.stderr.read()
        assert "leader exited" in diagnostics
        assert "still had descendants" in diagnostics
        assert not _group_alive(pgid)
    finally:
        _cleanup(process, pgid)


def test_second_preview_launch_after_stop_gets_a_fresh_owned_group():
    groups = []
    for _ in range(2):
        process, _leader_pid, pgid, _sid = _start_supervised_python(
            "import time; print('preview-ready', flush=True); time.sleep(60)"
        )
        groups.append(pgid)
        try:
            assert process.stdout.readline().strip() == "preview-ready"
            process.terminate()
            assert process.wait(timeout=5) == 0
            assert not _group_alive(pgid)
        finally:
            _cleanup(process, pgid)
    assert groups[0] != groups[1]


def test_product_launch_command_routes_only_the_preview_through_supervisor():
    source = RUNNER_CPP.read_text(encoding="utf-8")
    assert "workcell_preview_process_group.py" in source
    assert "exec python3 '%2' -- %3" in source
    assert "use_fake_hardware:=true" in source
    assert "launch_rviz:=true" in source
    assert "[c]ontroller_manager/ros2_control_node" in source
    assert "killall" not in source
    assert "pkill ros2" not in source
