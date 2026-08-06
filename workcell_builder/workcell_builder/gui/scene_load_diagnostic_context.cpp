#include "scene_load_diagnostic_context.h"

#include <QCryptographicHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace {
QJsonValue normalized_json(const QJsonValue & value)
{
  if (value.isObject()) {
    const QJsonObject input = value.toObject();
    QStringList keys = input.keys();
    keys.sort();
    QJsonObject output;
    for (const QString & key : keys) output.insert(key, normalized_json(input.value(key)));
    return output;
  }
  if (value.isArray()) {
    QJsonArray output;
    for (const QJsonValue & item : value.toArray()) output.append(normalized_json(item));
    return output;
  }
  return value;
}
}

QString SceneLoadDiagnosticContext::normalized_content_hash(const QJsonValue & content)
{
  QJsonArray wrapper;
  wrapper.append(normalized_json(content));
  const QByteArray bytes = QJsonDocument(wrapper).toJson(QJsonDocument::Compact);
  return QString::fromLatin1(QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
}

SceneLoadDiagnosticContext::Result SceneLoadDiagnosticContext::observe(
  const Identity & identity, const QString & report_type, const QJsonValue & content, bool terminal)
{
  const QString scene = identity.scene_id.trimmed().isEmpty() ? QStringLiteral("unknown") : identity.scene_id.trimmed();
  const QString source = identity.source_identity.trimmed().isEmpty() ? QStringLiteral("unknown") : identity.source_identity.trimmed();
  const QString type = report_type.trimmed().isEmpty() ? QStringLiteral("unknown") : report_type.trimmed();
  const QString load_key = QStringLiteral("%1\x1f%2\x1f%3").arg(scene, source).arg(identity.navigation_token);
  const QString report_key = load_key + QLatin1Char('\x1f') + type;
  const QString hash = normalized_content_hash(content);
  const bool changed = last_hash_by_report_.value(report_key) != hash;

  // A terminal transition is always visible. Polling the same terminal state
  // is still an identical observation and remains quiet outside diagnostics.
  QString terminal_state;
  if (terminal && content.isObject()) terminal_state = content.toObject().value(QStringLiteral("state")).toString();
  const bool terminal_transition = terminal && last_terminal_state_by_load_.value(load_key) != terminal_state;
  const bool emit = debug_enabled_ || changed || terminal_transition;
  last_hash_by_report_.insert(report_key, hash);
  if (terminal) last_terminal_state_by_load_.insert(load_key, terminal_state);

  return {emit,
    QStringLiteral("Scene load: scene=%1 source=%2 navigation=%3 type=%4 hash=%5")
      .arg(scene, source).arg(identity.navigation_token).arg(type, hash.left(12)),
    hash};
}
