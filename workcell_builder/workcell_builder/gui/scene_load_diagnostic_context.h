#pragma once

#include <QHash>
#include <QJsonValue>
#include <QString>

// One lifecycle-scoped gate for Product View load diagnostics.  Producers in
// the builder, native canvas, and embedded viewer bridge use the same identity
// and hash rules rather than maintaining independent "last message" caches.
class SceneLoadDiagnosticContext
{
public:
  struct Identity
  {
    QString scene_id;
    QString source_identity;
    quint64 navigation_token{0};
  };

  struct Result
  {
    bool emit{false};
    QString summary;
    QString content_hash;
  };

  void set_debug_enabled(bool enabled) { debug_enabled_ = enabled; }
  bool debug_enabled() const { return debug_enabled_; }
  Result observe(const Identity & identity, const QString & report_type,
                 const QJsonValue & content, bool terminal = false);
  static QString normalized_content_hash(const QJsonValue & content);

private:
  QHash<QString, QString> last_hash_by_report_;
  QHash<QString, QString> last_terminal_state_by_load_;
  bool debug_enabled_{false};
};
