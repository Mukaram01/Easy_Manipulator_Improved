#include "epd_detection_readiness_model.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <cmath>

namespace workcell_builder {
namespace {
const QString kSchema = QStringLiteral("workcell_epd_detection_readiness/v1");

bool positioned_state(const QString & s)
{
  return s == "LOCALISED" || s == "POSE_VALID" || s == "GRASP_CANDIDATE";
}

bool oriented_state(const QString & s) { return s == "POSE_VALID" || s == "GRASP_CANDIDATE"; }

EpdReadinessView unknown(const QString & reason)
{
  EpdReadinessView v;
  v.connection_state = "UNKNOWN";
  v.connection_reason = reason;
  return v;
}
}  // namespace

QString epd_readiness_artifact_path(const QString & scene_path)
{
  if (scene_path.isEmpty()) return {};
  return QDir(scene_path).filePath("preview/epd_detection_readiness.json");
}

EpdReadinessView load_epd_readiness_view(
  const QString & artifact_path, double now_wall_s, double file_stale_s)
{
  if (artifact_path.isEmpty()) return unknown("no scene selected");
  QFile file(artifact_path);
  if (!file.exists()) {
    return unknown("readiness monitor not running (no preview/epd_detection_readiness.json)");
  }
  if (!file.open(QIODevice::ReadOnly)) return unknown("readiness artifact unreadable");
  QJsonParseError error;
  const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &error);
  if (error.error != QJsonParseError::NoError || !doc.isObject()) {
    return unknown("malformed readiness artifact");
  }
  const QJsonObject o = doc.object();
  if (o.value("schema_version").toString() != kSchema) return unknown("unsupported readiness schema");

  EpdReadinessView v;
  v.connection_state = o.value("connection_state").toString("UNKNOWN");
  v.connection_reason = o.value("connection_reason").toString();
  v.camera_id = o.value("camera_id").toString();
  v.frame_id = o.value("frame_id").toString();
  v.report_age_s = o.value("last_report_age_s").toDouble(-1.0);
  v.epd_session = o.value("epd_session").toInt();
  for (const auto & e : o.value("contract_errors").toArray()) v.contract_errors << e.toString();

  const double written = o.value("written_wall_time").toDouble(0.0);
  if (!(written > 0.0) || now_wall_s - written > file_stale_s) {
    v.connection_state = "STALE";
    v.connection_reason = QString("readiness monitor not updating (artifact %1 s old)")
      .arg(written > 0.0 ? now_wall_s - written : -1.0, 0, 'f', 1);
  }
  if (v.connection_state != "FRESH") {
    v.contract_errors.clear();
    return v;  // never present stale objects as current
  }

  for (const auto & item : o.value("objects").toArray()) {
    const QJsonObject d = item.toObject();
    EpdReadinessRow r;
    r.index = d.value("index").toInt(-1);
    r.class_label = d.value("class").toString();
    r.confidence = d.value("confidence").isDouble() ? d.value("confidence").toDouble() : -1.0;
    r.track_id = d.value("track_id").isString() ? d.value("track_id").toString() : QString();
    r.state = d.value("state").toString("UNKNOWN");
    const QJsonArray p = d.value("position_m").toArray();
    if (positioned_state(r.state) && p.size() == 3) {
      bool finite = true;
      for (int i = 0; i < 3; ++i) {
        r.position_m[i] = p.at(i).toDouble(NAN);
        finite = finite && std::isfinite(r.position_m[i]);
      }
      r.has_position = finite;
    }
    r.orientation_valid = oriented_state(r.state) && d.value("orientation_valid").toBool(false);
    QStringList reasons;
    for (const auto & reason : d.value("reasons").toArray()) reasons << reason.toString();
    r.reasons = reasons.join(", ");
    r.note = d.value("note").toString();
    v.rows.push_back(r);
  }
  return v;
}

QString format_epd_readiness_position(const EpdReadinessRow & row)
{
  if (!row.has_position) return QStringLiteral("not validated");
  return QString("%1, %2, %3")
    .arg(row.position_m[0], 0, 'f', 3).arg(row.position_m[1], 0, 'f', 3).arg(row.position_m[2], 0, 'f', 3);
}

QString format_epd_readiness_summary(const EpdReadinessView & view)
{
  QString text = QString("EPD readiness: %1 — %2").arg(view.connection_state, view.connection_reason);
  if (!view.camera_id.isEmpty() || !view.frame_id.isEmpty()) {
    text += QString("\nCamera: %1  Frame: %2").arg(view.camera_id, view.frame_id);
  }
  if (view.report_age_s >= 0.0) text += QString("  Last report: %1 s ago").arg(view.report_age_s, 0, 'f', 1);
  if (view.connection_state == "FRESH") {
    int candidates = 0;
    for (const auto & r : view.rows) candidates += r.state == "GRASP_CANDIDATE" ? 1 : 0;
    text += QString("\nObjects: %1 (grasp candidates: %2)").arg(view.rows.size()).arg(candidates);
  }
  if (view.epd_session > 0) text += QString("\nEPD restarted %1x; track IDs restart with EPD.").arg(view.epd_session);
  if (!view.contract_errors.isEmpty()) text += "\nContract warnings: " + view.contract_errors.join("; ");
  text += "\nDiagnostic only: planning uses validated EPD poses; candidates are not reachability or collision checked here.";
  return text;
}

}  // namespace workcell_builder
