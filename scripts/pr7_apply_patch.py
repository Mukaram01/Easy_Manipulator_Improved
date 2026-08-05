#!/usr/bin/env python3
"""Apply the focused PR7 C++ warning repairs, then remove this helper."""

from pathlib import Path


HEADER_PATH = Path("workcell_builder/workcell_builder/gui/mainwindow.h")
SOURCE_PATH = Path("workcell_builder/workcell_builder/gui/scene_select.cpp")
TEST_PATH = Path("tests/test_workcell_builder_cpp_warning_repairs.py")
WORKFLOW_PATH = Path(".github/workflows/pr7-autopatch.yml")
SELF_PATH = Path(__file__)


def replace_once(text: str, old: str, new: str, description: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"expected exactly one {description}, found {count}")
    return text.replace(old, new, 1)


def patch_header() -> None:
    text = HEADER_PATH.read_text(encoding="utf-8")
    text = replace_once(
        text,
        "  QVector<CanonicalAssetEditorLayoutItemSnapshot> preview_items;",
        "  QVector<CanonicalAssetEditorLayoutItemSnapshot> preview_items{};",
        "CanvasEditCommand preview_items member",
    )
    HEADER_PATH.write_text(text, encoding="utf-8")


def patch_scene_select() -> None:
    text = SOURCE_PATH.read_text(encoding="utf-8")

    namespace_anchor = "\n\n}  // namespace\n\nstruct RobotHomeJoint"
    helper = r'''

struct CheckedProcessResult
{
  bool ok{false};
  int exit_code{-1};
  QProcess::ExitStatus exit_status{QProcess::NormalExit};
  QString stdout_text;
  QString stderr_text;
  QString diagnostic;
};

QString quote_process_argument(const QString & argument)
{
  if (argument.isEmpty()) {
    return QStringLiteral("\"\"");
  }
  QString escaped = argument;
  escaped.replace('\\', QStringLiteral("\\\\"));
  escaped.replace('"', QStringLiteral("\\\""));
  if (argument.contains(QRegularExpression(QStringLiteral("[\\s\"]")))) {
    return QStringLiteral("\"") + escaped + QStringLiteral("\"");
  }
  return escaped;
}

QString process_command_text(const QString & program, const QStringList & arguments)
{
  QStringList parts{quote_process_argument(program)};
  for (const QString & argument : arguments) {
    parts.push_back(quote_process_argument(argument));
  }
  return parts.join(' ');
}

CheckedProcessResult run_checked_process(
  const QString & operation,
  const QString & program,
  const QStringList & arguments,
  const QString & working_directory = QString(),
  int timeout_ms = 30000)
{
  CheckedProcessResult result;
  QProcess process;
  process.setProgram(program);
  process.setArguments(arguments);
  process.setProcessChannelMode(QProcess::SeparateChannels);
  if (!working_directory.trimmed().isEmpty()) {
    process.setWorkingDirectory(working_directory);
  }

  const QString command = process_command_text(program, arguments);
  const QString shown_working_directory = working_directory.isEmpty()
    ? QStringLiteral("<inherit>")
    : working_directory;
  process.start();
  if (!process.waitForStarted(timeout_ms)) {
    result.diagnostic = QStringLiteral(
      "%1 failed to start. command=%2 working_directory=%3 process_error=%4. "
      "Check that the executable exists, is on PATH, and the working directory is accessible.")
      .arg(operation)
      .arg(command)
      .arg(shown_working_directory)
      .arg(process.errorString());
    const QByteArray diagnostic = result.diagnostic.toUtf8();
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", diagnostic.constData());
    return result;
  }

  if (!process.waitForFinished(timeout_ms)) {
    process.terminate();
    if (!process.waitForFinished(2000)) {
      process.kill();
      process.waitForFinished(2000);
    }
    result.stdout_text = QString::fromLocal8Bit(process.readAllStandardOutput());
    result.stderr_text = QString::fromLocal8Bit(process.readAllStandardError());
    result.diagnostic = QStringLiteral(
      "%1 timed out after %2 ms. command=%3 working_directory=%4 "
      "process_error=%5 stdout=%6 stderr=%7")
      .arg(operation)
      .arg(timeout_ms)
      .arg(command)
      .arg(shown_working_directory)
      .arg(process.errorString())
      .arg(result.stdout_text.trimmed().isEmpty()
        ? QStringLiteral("<empty>") : result.stdout_text.trimmed())
      .arg(result.stderr_text.trimmed().isEmpty()
        ? QStringLiteral("<empty>") : result.stderr_text.trimmed());
    const QByteArray diagnostic = result.diagnostic.toUtf8();
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", diagnostic.constData());
    return result;
  }

  result.stdout_text = QString::fromLocal8Bit(process.readAllStandardOutput());
  result.stderr_text = QString::fromLocal8Bit(process.readAllStandardError());
  result.exit_status = process.exitStatus();
  result.exit_code = process.exitCode();
  result.ok = result.exit_status == QProcess::NormalExit && result.exit_code == 0;
  if (!result.ok) {
    const QString exit_status = result.exit_status == QProcess::NormalExit
      ? QStringLiteral("normal")
      : QStringLiteral("crashed");
    result.diagnostic = QStringLiteral(
      "%1 failed. command=%2 working_directory=%3 exit_status=%4 "
      "exit_code=%5 process_error=%6 stdout=%7 stderr=%8")
      .arg(operation)
      .arg(command)
      .arg(shown_working_directory)
      .arg(exit_status)
      .arg(result.exit_code)
      .arg(process.errorString())
      .arg(result.stdout_text.trimmed().isEmpty()
        ? QStringLiteral("<empty>") : result.stdout_text.trimmed())
      .arg(result.stderr_text.trimmed().isEmpty()
        ? QStringLiteral("<empty>") : result.stderr_text.trimmed());
    const QByteArray diagnostic = result.diagnostic.toUtf8();
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", diagnostic.constData());
  }
  return result;
}
'''
    text = replace_once(
        text,
        namespace_anchor,
        helper + namespace_anchor,
        "anonymous-namespace process-helper insertion point",
    )

    metadata_old = r'''    const fs::path metadata_script = tool_root / "scripts" / "render_workcell_builder_metadata.py";
    const std::string cmd =
      "python3 \"" + metadata_script.string() + "\" --robot \"" + robot_name +
      "\" --end-effector \"" + ee_name + "\" --scene-path \"" + scene_dir.string() +
      "\" --output \"" + metadata_path.string() + "\"";
    const int metadata_rc = std::system(cmd.c_str());
    if (metadata_rc != 0) {
      append_warning("Workcell Studio metadata generation command failed with exit code " + std::to_string(metadata_rc) + ".");
    }'''
    metadata_new = r'''    const fs::path metadata_script = tool_root / "scripts" / "render_workcell_builder_metadata.py";
    const QStringList metadata_arguments{
      QString::fromStdString(metadata_script.string()),
      QStringLiteral("--robot"), QString::fromStdString(robot_name),
      QStringLiteral("--end-effector"), QString::fromStdString(ee_name),
      QStringLiteral("--scene-path"), QString::fromStdString(scene_dir.string()),
      QStringLiteral("--output"), QString::fromStdString(metadata_path.string())};
    const CheckedProcessResult metadata_result = run_checked_process(
      QStringLiteral("Workcell Studio metadata generation"),
      QStringLiteral("python3"),
      metadata_arguments,
      QString::fromStdString(tool_root.string()));
    if (!metadata_result.ok) {
      append_warning(metadata_result.diagnostic.toStdString());
    } else if (!metadata_result.stdout_text.trimmed().isEmpty()) {
      append_info(metadata_result.stdout_text.trimmed().toStdString());
    }'''
    text = replace_once(text, metadata_old, metadata_new, "metadata std::system block")

    readiness_old = r'''  const fs::path workspace_root = derive_ros_workspace_root(workcell_path);
  const std::string cmd = "python3 scripts/validate_supported_scenes_readiness.py --repo-root "" + workcell_path.string() + "" --workspace-root "" + workspace_root.string() + "" --json --skip-build --skip-launch-smoke";
  append_info("Run All-Scenes Readiness: " + cmd);
  const int rc = std::system(cmd.c_str());
  if (rc != 0) { append_error("Run All-Scenes Readiness failed with exit code " + std::to_string(rc)); return; }
  append_success("Run All-Scenes Readiness completed.");'''
    readiness_new = r'''  const fs::path workspace_root = derive_ros_workspace_root(workcell_path);
  const QStringList readiness_arguments{
    QStringLiteral("scripts/validate_supported_scenes_readiness.py"),
    QStringLiteral("--repo-root"), QString::fromStdString(workcell_path.string()),
    QStringLiteral("--workspace-root"), QString::fromStdString(workspace_root.string()),
    QStringLiteral("--json"),
    QStringLiteral("--skip-build"),
    QStringLiteral("--skip-launch-smoke")};
  append_info(
    "Run All-Scenes Readiness: " +
    process_command_text(QStringLiteral("python3"), readiness_arguments).toStdString());
  const CheckedProcessResult readiness_result = run_checked_process(
    QStringLiteral("Run All-Scenes Readiness"),
    QStringLiteral("python3"),
    readiness_arguments,
    QString::fromStdString(workcell_path.string()),
    120000);
  if (!readiness_result.stdout_text.trimmed().isEmpty()) {
    append_info(readiness_result.stdout_text.trimmed().toStdString());
  }
  if (!readiness_result.stderr_text.trimmed().isEmpty()) {
    append_warning(readiness_result.stderr_text.trimmed().toStdString());
  }
  if (!readiness_result.ok) {
    append_error(readiness_result.diagnostic.toStdString());
    return;
  }
  append_success("Run All-Scenes Readiness completed.");'''
    text = replace_once(text, readiness_old, readiness_new, "readiness std::system block")

    if "std::system(" in text:
        raise SystemExit("unchecked std::system remains in scene_select.cpp")
    SOURCE_PATH.write_text(text, encoding="utf-8")


def write_tests() -> None:
    TEST_PATH.write_text(
        '''from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.h"
SOURCE = ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp"


def test_canvas_edit_command_default_initializes_preview_items():
    header = HEADER.read_text(encoding="utf-8")
    assert "QVector<CanonicalAssetEditorLayoutItemSnapshot> preview_items{};" in header
    assert "QVector<CanonicalAssetEditorLayoutItemSnapshot> preview_items;" not in header


def test_scene_select_uses_checked_qprocess_instead_of_std_system():
    source = SOURCE.read_text(encoding="utf-8")
    assert "std::system(" not in source
    assert "CheckedProcessResult run_checked_process(" in source
    for token in (
        "process.waitForStarted(timeout_ms)",
        "process.waitForFinished(timeout_ms)",
        "process.exitStatus()",
        "process.exitCode()",
        "process.errorString()",
        "process.readAllStandardOutput()",
        "process.readAllStandardError()",
        "process.terminate()",
        "process.kill()",
    ):
        assert token in source


def test_process_failures_are_actionable_and_argument_safe():
    source = SOURCE.read_text(encoding="utf-8")
    for token in (
        "failed to start",
        "timed out after",
        "working_directory=%3 exit_status=%4",
        "exit_code=%5 process_error=%6",
        "stdout=%7 stderr=%8",
    ):
        assert token in source
    assert "process.setProgram(program);" in source
    assert "process.setArguments(arguments);" in source
    assert "QProcess::SeparateChannels" in source
    assert 'QStringLiteral("--repo-root"), QString::fromStdString(workcell_path.string())' in source
    assert 'QStringLiteral("--workspace-root"), QString::fromStdString(workspace_root.string())' in source
    assert '\"\" + workcell_path.string() + \"\"' not in source


def test_all_qprocess_call_results_are_checked():
    source = SOURCE.read_text(encoding="utf-8")
    assert "const CheckedProcessResult metadata_result = run_checked_process(" in source
    assert "if (!metadata_result.ok)" in source
    assert "const CheckedProcessResult readiness_result = run_checked_process(" in source
    assert "if (!readiness_result.ok)" in source
''',
        encoding="utf-8",
    )


def cleanup_scaffolding() -> None:
    WORKFLOW_PATH.unlink(missing_ok=False)
    SELF_PATH.unlink(missing_ok=False)


def main() -> None:
    patch_header()
    patch_scene_select()
    write_tests()
    cleanup_scaffolding()


if __name__ == "__main__":
    main()
