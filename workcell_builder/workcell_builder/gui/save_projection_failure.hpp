#pragma once

// Human-readable reason for a failed authored-layout projection
// (scripts/workcell_studio_layout_merge.py --save-authored). The helper
// reports blocking authoring problems as uncaught Python exceptions, so the
// user used to see a 1.6 kB traceback; the reason is its last line.

#include <QString>
#include <QStringList>

namespace workcell_builder
{

inline QString projection_failure_reason(const QString & stderr_text, const QString & stdout_text)
{
  const auto last_line = [](const QString & text) {
    const QStringList lines = text.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    for (int index = lines.size() - 1; index >= 0; --index) {
      const QString line = lines.at(index).trimmed();
      if (!line.isEmpty()) return line;
    }
    return QString();
  };
  QString reason = last_line(stderr_text);
  if (reason.isEmpty()) reason = last_line(stdout_text);
  if (reason.isEmpty()) return QStringLiteral("the projection helper reported no detail");
  // "ValueError: ..." / "RuntimeError: ..." / "error: ..." -> message only.
  const int colon = reason.indexOf(QStringLiteral(": "));
  if (colon > 0 && colon < 40 && !reason.left(colon).contains(QLatin1Char(' '))) reason = reason.mid(colon + 2).trimmed();
  if (reason.contains(QStringLiteral("outside target usable placement geometry"))) {
    reason += QStringLiteral(" (move the item back inside its destination zone's usable placement area)");
  }
  return reason.left(400);
}

inline QString projection_failure_message(const QString & stderr_text, const QString & stdout_text)
{
  return QStringLiteral("Save failed: authored layout projection rejected the edit: %1.")
    .arg(projection_failure_reason(stderr_text, stdout_text));
}

}  // namespace workcell_builder
