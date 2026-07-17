from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")


@dataclass(frozen=True)
class Identity:
    scene: str
    generation: int
    revision: int


class PreparationDiagnostics:
    """Behavioral contract: immutable preparation identities log once."""

    def __init__(self):
        self.results = {}

    def terminal(self, identity, outcome):
        self.results.setdefault(identity, outcome)


def test_every_started_preparation_terminal_path_records_exactly_one_result():
    diagnostics = PreparationDiagnostics()
    paths = ["success", "command_failure", "process_error", "cancelled", "stale_discarded"]
    for generation, outcome in enumerate(paths, 1):
        identity = Identity("ur5_2f_test", generation, 40 + generation)
        diagnostics.terminal(identity, outcome)
        # A queued finished/error callback after the terminal event is ignored.
        diagnostics.terminal(identity, "success")

    assert len(diagnostics.results) == len(paths)
    assert list(diagnostics.results.values()) == paths


def test_cpp_records_bounded_tails_and_all_qprocess_terminal_signals():
    start = CPP.index("void ScenePreviewWidget::start_embedded_web_prepare")
    end = CPP.index("void ScenePreviewWidget::on_embedded_web_prepare_finished", start)
    prepare = CPP[start:end]
    for signal in [
        "&QProcess::started",
        "&QProcess::errorOccurred",
        "&QProcess::readyReadStandardOutput",
        "&QProcess::readyReadStandardError",
        "&QProcess::finished",
    ]:
        assert signal in prepare
    assert "kPreparationOutputTailBytes = 4096" in CPP
    assert "tail = tail.right(kPreparationOutputTailBytes)" in CPP

    terminal = CPP[CPP.index("void ScenePreviewWidget::record_embedded_web_prepare_terminal"):CPP.index("void ScenePreviewWidget::refresh_embedded_web_product_view")]
    for token in ["scene=%2", "generation=%3", "payload_revision=%4", "exit_status=%5", "exit_code=%6", "expected_json_exists=%7"]:
        assert token in terminal
    assert "terminal_recorded" in terminal


def test_cancellation_and_stale_completion_record_without_mutating_current_ui():
    cancel = CPP[CPP.index("void ScenePreviewWidget::cancel_embedded_web_lifecycle"):CPP.index("void ScenePreviewWidget::request_embedded_web_product_view_refresh")]
    assert 'QStringLiteral("cancelled")' in cancel
    assert cancel.index("record_embedded_web_prepare_terminal") < cancel.index("disconnect(process")

    finished = CPP[CPP.index("void ScenePreviewWidget::on_embedded_web_prepare_finished"):CPP.index("void ScenePreviewWidget::start_embedded_web_readiness_polling")]
    stale = finished.index("if (!embedded_web_identity_is_current(identity))")
    assert 'QStringLiteral("stale_discarded")' in finished[stale:]
    assert finished.index("record_embedded_web_prepare_terminal", stale) < finished.index("maybe_start_next_embedded_web_prepare();", stale)
