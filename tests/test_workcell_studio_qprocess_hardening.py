from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_qprocess_stdout_stderr_and_finished_hooks():
    for needle in ['readyReadStandardOutput', 'readyReadStandardError', 'QProcess::finished']:
        assert needle in MAIN


def test_qprocess_stop_path_uses_terminate_and_kill():
    assert 'terminate()' in MAIN
    assert 'kill()' in MAIN
    assert 'waitForFinished(2000)' in MAIN
