#pragma once

#include <QString>
#include <QStringList>
#include <QVector>

#include <array>

namespace workcell_builder {

// Operator view of EPD per-detection readiness, read from the status artifact written
// by epd_readiness_monitor_node.py. Display only: nothing here feeds grasp planning.
struct EpdReadinessRow {
  int index{-1};
  QString class_label;
  double confidence{-1.0};       // < 0 when unavailable
  QString track_id;              // empty unless EPD tracking supplied one
  QString state;                 // DETECTED | LOCALISED | POSE_VALID | GRASP_CANDIDATE | REJECTED | UNKNOWN
  bool has_position{false};
  std::array<double, 3> position_m{{0.0, 0.0, 0.0}};
  bool orientation_valid{false};
  QString reasons;
  QString note;
};

struct EpdReadinessView {
  QString connection_state{"UNKNOWN"};  // FRESH | STALE | UNKNOWN | UNAVAILABLE
  QString connection_reason;
  QString camera_id;
  QString frame_id;
  double report_age_s{-1.0};
  int epd_session{0};
  QVector<EpdReadinessRow> rows;        // empty unless connection_state == FRESH
  QStringList contract_errors;
};

QString epd_readiness_artifact_path(const QString & scene_path);

// `file_stale_s` guards against a stopped monitor leaving a frozen "FRESH" artifact.
EpdReadinessView load_epd_readiness_view(
  const QString & artifact_path, double now_wall_s, double file_stale_s = 3.0);

QString format_epd_readiness_summary(const EpdReadinessView & view);
QString format_epd_readiness_position(const EpdReadinessRow & row);

}  // namespace workcell_builder
