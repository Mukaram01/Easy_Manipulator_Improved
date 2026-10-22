#pragma once

// Command construction and outcome classification for "Generate Readiness Pack".
//
// CLI contract of scripts/generate_workcell_studio_readiness_pack.py:
//   --scene-package DIR --output-dir DIR --project-name NAME   (all required)
//   optional: --json (prints {"result": PASS|WARN|FAIL, "manifest": ...})
// A positional scene path is rejected by argparse with exit code 2, which is
// what the GUI used to send. Exit code 0 means the pack was written and the
// final readiness is not FAIL; the manifest file is the artefact that proves it.

#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
#include <QStringList>

namespace workcell_builder
{

struct ReadinessPackCommand
{
  QString program{QStringLiteral("python3")};
  QStringList arguments;
  QString output_dir;     // <scene_dir>/readiness
  QString manifest_path;  // <output_dir>/readiness_pack_manifest.json
};

inline ReadinessPackCommand readiness_pack_command(
  const QString & script_path, const QString & scene_dir, const QString & scene_name)
{
  ReadinessPackCommand command;
  command.output_dir = QDir(scene_dir).filePath(QStringLiteral("readiness"));
  command.manifest_path = QDir(command.output_dir).filePath(QStringLiteral("readiness_pack_manifest.json"));
  command.arguments = QStringList{
    script_path,
    QStringLiteral("--scene-package"), scene_dir,
    QStringLiteral("--output-dir"), command.output_dir,
    QStringLiteral("--project-name"), scene_name,
    QStringLiteral("--json")};
  return command;
}

struct ReadinessPackOutcome
{
  bool success{false};
  QString kind{QStringLiteral("error")};  // banner kind: success | warning | error
  QString message;
  QString result;                         // PASS | WARN | FAIL | empty
};

// `manifest_written` must mean "the manifest exists and was written by this
// run" (checked by the caller against the start time), never merely "exists".
inline ReadinessPackOutcome classify_readiness_pack_outcome(
  bool started, bool timed_out, bool normal_exit, int exit_code,
  const QString & stdout_text, const QString & stderr_text,
  bool manifest_written, const QString & manifest_path, int timeout_seconds)
{
  ReadinessPackOutcome outcome;
  const QString last_stderr = stderr_text.trimmed().section(QLatin1Char('\n'), -1).trimmed().left(220);
  const QJsonObject report = QJsonDocument::fromJson(stdout_text.trimmed().toUtf8()).object();
  outcome.result = report.value(QStringLiteral("result")).toString().trimmed();

  if (!started) {
    outcome.kind = QStringLiteral("warning");
    outcome.message = QStringLiteral("Generate Readiness Pack did not start (python3 or the script could not be launched%1). Nothing was written.")
      .arg(last_stderr.isEmpty() ? QString() : QStringLiteral(": %1").arg(last_stderr));
    return outcome;
  }
  if (timed_out) {
    outcome.kind = QStringLiteral("warning");
    outcome.message = QStringLiteral("Generate Readiness Pack timed out after %1 s and was stopped. The pack is incomplete; run it again or inspect the Studio Log.")
      .arg(timeout_seconds);
    return outcome;
  }
  if (!normal_exit || exit_code != 0) {
    outcome.kind = QStringLiteral("error");
    if (outcome.result == QStringLiteral("FAIL")) {
      // The script exits 2 when the final readiness is FAIL; the pack it wrote
      // holds the failing checks, so point the user at it instead of hiding it.
      outcome.message = QStringLiteral("Readiness pack reports FAIL (exit code %1): final readiness is FAIL. %2 Fix the failing checks, then run it again.")
        .arg(exit_code)
        .arg(manifest_written ? QStringLiteral("The failing checks are listed in %1.").arg(manifest_path)
                              : QStringLiteral("No manifest was written."));
      return outcome;
    }
    outcome.message = QStringLiteral("Generate Readiness Pack failed (exit code %1)%2 No readiness pack was published; fix the reported items and run it again.")
      .arg(exit_code).arg(last_stderr.isEmpty() ? QStringLiteral(".") : QStringLiteral(": %1.").arg(last_stderr));
    return outcome;
  }
  if (!manifest_written) {
    outcome.kind = QStringLiteral("error");
    outcome.message = QStringLiteral("Generate Readiness Pack finished but wrote no manifest at %1. Treat the pack as missing.")
      .arg(manifest_path);
    return outcome;
  }
  outcome.success = true;
  outcome.kind = outcome.result == QStringLiteral("WARN") ? QStringLiteral("warning") : QStringLiteral("success");
  outcome.message = QStringLiteral("Readiness pack generated (final readiness %1). Manifest: %2. Open Readiness Dashboard from More Actions.")
    .arg(outcome.result.isEmpty() ? QStringLiteral("reported") : outcome.result, manifest_path);
  return outcome;
}

}  // namespace workcell_builder
