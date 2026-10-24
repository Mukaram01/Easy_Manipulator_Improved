#pragma once

// View model for the Operator HMI. It only translates the read-only projection
// (scripts/operator_preview_projection.py) and existing dry-run reports into
// display state. It never invents values: anything missing stays empty and is
// rendered as UNAVAILABLE / UNKNOWN.

#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <QVector>

#include <array>
#include <optional>

namespace workcell_builder
{

enum class OperatorMode { Draft, Replay, Offline, Simulation, Live };
QString operator_mode_label(OperatorMode mode);

struct OperatorZone
{
  QString id;
  QString type;
  std::array<double, 3> xyz{};
  std::array<double, 3> dimensions{};
  bool valid{false};
};

struct OperatorObjectRow
{
  QString object_id;
  QString class_id;
  std::optional<double> confidence;
  bool accepted{false};
  QString reason_code;
  QString reason;
  std::array<double, 3> xyz{};
  std::array<double, 3> dimensions{};
  bool has_geometry{false};
};

struct OperatorEvent
{
  QString level;
  QString code;
  QString message;
};

struct OperatorPreviewView
{
  bool usable{false};
  QStringList errors;

  QString scene_name;
  QString scene_path;
  bool draft{false};
  std::optional<bool> draft_differs;
  bool layout_unsaved{false};
  QString intent_sha256;
  QString saved_intent_sha256;
  QString intent_file_schema;
  bool intent_file_exists{false};

  QString approval_status;
  QString approval_by;
  QString approval_at;

  QString robot;
  QString planning_group;
  QString tool;
  QString tool_type;
  QStringList capabilities;
  QString camera;
  QString perception_mode;
  QString perception_contract;

  QString task_id;
  QString task_type;
  QString target_class;
  QString target_color;
  std::optional<double> min_confidence;
  std::optional<double> max_age_seconds;
  QString pick_zone;
  QString source;
  QString not_targeted_rule;
  QStringList destinations;
  QStringList routing_rules;
  QString grasp;

  QString validation_status;
  QString readiness_classification;
  QStringList validation_errors;
  QStringList validation_warnings;
  QStringList next_actions;

  QString resolution_status;
  QString resolution_code;
  QString resolution_reason;

  QString scene_validation_status;
  std::optional<double> scene_validation_modified;
  QString offline_validation_status;
  std::optional<double> offline_validation_modified;
  std::optional<bool> offline_validation_current;

  QString input_kind;  // replay | live | unavailable
  QString input_path;
  QString input_blocker;

  bool replay_evaluated{false};
  QString replay_error;
  QString replay_description;
  QString selected_object;
  QVector<OperatorObjectRow> objects;

  OperatorZone pick_zone_geometry;
  OperatorZone place_zone_geometry;

  bool dry_run_available{false};
  QString dry_run_unavailable_reason;
  QString execution_reason;
  QString simulation_note;
  QString live_note;
  bool motion_authorised{false};

  QVector<OperatorEvent> events;
};

OperatorPreviewView parse_operator_preview(const QJsonObject & projection, bool layout_unsaved);

// DRAFT outranks REPLAY: unsaved engineering edits are never mistaken for an
// operating configuration. SIMULATION and LIVE are reported only when a real
// backend says so; the preview never sets them itself.
OperatorMode operator_mode(const OperatorPreviewView & view, bool replay_shown);

struct OperatorReadiness
{
  bool offline_checks_allowed{false};
  QStringList offline_blockers;
  QStringList production_blockers;  // never empty in this release
};
OperatorReadiness operator_readiness(const OperatorPreviewView & view);

struct GatedDryRunResult
{
  bool parsed{false};
  QString status;
  QString preflight_status;
  QStringList blockers;
  QStringList warnings;
  QString cycle_status;
  QString cycle_note;
  QString next_action;
  QString selected_object;
  QString selected_destination;
  bool safe_for_robot_motion{false};
  QStringList stages;
};
GatedDryRunResult parse_gated_dry_run(const QJsonObject & report);

}  // namespace workcell_builder
