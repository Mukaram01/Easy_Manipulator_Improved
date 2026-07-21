from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GUI = ROOT / "workcell_builder/workcell_builder/gui"
EDIT_SAVE = GUI / "embedded_web_edit_save_controller.hpp"
CURATED_ADD = GUI / "embedded_web_curated_add_controller.hpp"


def test_edit_save_controller_resets_qurl_explicitly_for_qt5():
    source = EDIT_SAVE.read_text(encoding="utf-8")
    assert "#include <QUrl>" in source
    assert "last_polled_url_ = QUrl();" in source
    assert "last_polled_url_ = {};" not in source


def test_curated_add_controller_uses_qt5_qjsonarray_api_with_shape_guards():
    source = CURATED_ADD.read_text(encoding="utf-8")

    assert 'dimensions.size() != 3' in source
    assert 'xyz.size() != 3' in source

    for token in [
        "choice.dimensions.at(0).toDouble()",
        "choice.dimensions.at(1).toDouble()",
        "choice.dimensions.at(2).toDouble()",
        "xyz.at(0).toDouble()",
        "xyz.at(1).toDouble()",
        "xyz.at(2).toDouble()",
    ]:
        assert token in source

    for incompatible in [
        "choice.dimensions.value(0)",
        "choice.dimensions.value(1)",
        "choice.dimensions.value(2)",
        "xyz.value(0)",
        "xyz.value(1)",
        "xyz.value(2)",
    ]:
        assert incompatible not in source


def test_curated_add_controller_has_unambiguous_process_and_reload_control_flow():
    source = CURATED_ADD.read_text(encoding="utf-8")

    assert "if (process == process_) {\n          process_ = nullptr;\n        }\n        process->deleteLater();" in source
    assert "if (view_) {\n      view_->reload();\n    }\n    QTimer::singleShot" in source
    assert "if (process == process_) process_ = nullptr; process->deleteLater();" not in source
    assert "if (view_) view_->reload(); QTimer::singleShot" not in source
