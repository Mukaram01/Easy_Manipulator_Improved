from pathlib import Path


CPP = Path("workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")


def _between(start: str, end: str) -> str:
    begin = CPP.index(start)
    return CPP[begin:CPP.index(end, begin)]


def test_scene_select_has_no_unchecked_shell_execution():
    assert "std::system" not in CPP
    assert 'QStringLiteral("/bin/sh")' not in CPP
    assert 'QStringLiteral("/bin/bash")' not in CPP


def test_checked_launcher_preserves_space_containing_arguments_and_captures_results():
    launcher = _between("CheckedProcessResult run_process_checked", "bool process_succeeded")
    assert "process.setProgram(executable)" in launcher
    assert "process.setArguments(arguments)" in launcher
    assert "QProcessEnvironment::systemEnvironment()" in launcher
    assert "QProcess::SeparateChannels" in launcher
    assert "waitForStarted" in launcher
    assert "waitForFinished" in launcher
    assert "process.exitStatus()" in launcher
    assert "process.exitCode()" in launcher
    assert "readAllStandardOutput()" in launcher
    assert "readAllStandardError()" in launcher

    # Each filesystem value remains one QStringList element, so a path such as
    # "/tmp/Workcell Studio/scenes" is not split or interpreted by a shell.
    metadata = _between("const fs::path metadata_script", "write_builder_validation_helper")
    assert "const QStringList arguments" in metadata
    assert "QString::fromStdString(scene_dir.string())" in metadata
    assert "run_process_checked(executable, arguments, working_directory)" in metadata


def test_failed_and_nonzero_launches_have_actionable_diagnostics():
    success = _between("bool process_succeeded", "QString process_failure_message")
    assert "result.started" in success
    assert "QProcess::NormalExit" in success
    assert "result.exit_code == 0" in success

    failure = _between("QString process_failure_message", "bool emit_perception_contract_warning_once")
    for detail in (
        "executable=%2",
        "arguments=%3",
        "working_directory=%4",
        "started=%5",
        "exit_status=%7",
        "exit_code=%8",
        "error=%1",
        "stderr=%1",
        "stdout=%1",
        "Next step: %1",
    ):
        assert detail in failure

    readiness = _between(
        "void SceneSelect::on_run_all_scenes_readiness_clicked",
        "\n}",
    )
    assert "process_failure_message(" in readiness
    assert "verify Python 3 and the readiness validator" in readiness
