#include "operator_hmi_model.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>

namespace workcell_builder
{

namespace
{

QString text(const QJsonValue & value)
{
  if (value.isString()) return value.toString();
  if (value.isDouble()) return QString::number(value.toDouble());
  if (value.isBool()) return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
  return {};
}

std::optional<double> number(const QJsonValue & value)
{
  if (value.isDouble()) return value.toDouble();
  return std::nullopt;
}

QStringList strings(const QJsonValue & value)
{
  QStringList out;
  for (const auto & item : value.toArray()) {
    if (item.isString()) {
      out << item.toString();
    } else if (item.isObject()) {
      const auto object = item.toObject();
      const QString message = object.value("message").toString();
      out << (message.isEmpty() ? QString::fromUtf8(QJsonDocument(object).toJson(QJsonDocument::Compact)) : message);
    }
  }
  return out;
}

bool vector3(const QJsonValue & value, std::array<double, 3> & out)
{
  const auto array = value.toArray();
  if (array.size() < 3) return false;
  for (int i = 0; i < 3; ++i) {
    if (!array.at(i).isDouble()) return false;
    out[static_cast<size_t>(i)] = array.at(i).toDouble();
  }
  return true;
}

OperatorZone zone(const QJsonValue & value)
{
  OperatorZone out;
  if (!value.isObject()) return out;
  const auto object = value.toObject();
  out.id = object.value("id").toString();
  out.type = object.value("type").toString();
  out.valid = vector3(object.value("pose_xyz"), out.xyz) && vector3(object.value("dimensions"), out.dimensions);
  return out;
}

}  // namespace

QString operator_mode_label(OperatorMode mode)
{
  switch (mode) {
    case OperatorMode::Draft: return QStringLiteral("DRAFT");
    case OperatorMode::Replay: return QStringLiteral("REPLAY");
    case OperatorMode::Offline: return QStringLiteral("OFFLINE");
    case OperatorMode::Simulation: return QStringLiteral("SIMULATION");
    case OperatorMode::Live: return QStringLiteral("LIVE");
  }
  return QStringLiteral("UNKNOWN");
}

OperatorPreviewView parse_operator_preview(const QJsonObject & projection, bool layout_unsaved)
{
  OperatorPreviewView view;
  view.layout_unsaved = layout_unsaved;
  view.errors = strings(projection.value("errors"));
  if (projection.value("schema").toString() != QStringLiteral("workcell_operator_preview/v1")) {
    view.errors.prepend(QStringLiteral("PROJECTION_INVALID: the operator projection was not produced."));
    return view;
  }

  const auto scene = projection.value("scene").toObject();
  view.scene_name = scene.value("name").toString();
  view.scene_path = scene.value("path").toString();

  const auto revision = projection.value("revision").toObject();
  view.draft = revision.value("kind").toString() == QStringLiteral("draft");
  if (revision.value("draft_differs_from_saved").isBool()) {
    view.draft_differs = revision.value("draft_differs_from_saved").toBool();
  }
  view.intent_sha256 = revision.value("normalized_intent_sha256").toString();
  view.saved_intent_sha256 = revision.value("saved_normalized_intent_sha256").toString();
  view.intent_file_schema = revision.value("intent_file_schema").toString();
  view.intent_file_exists = revision.value("intent_file_exists").toBool();

  const auto approval = projection.value("approval").toObject();
  view.approval_status = approval.value("status").toString();
  view.approval_by = approval.value("approved_by").toString();
  view.approval_at = approval.value("approved_at").toString();

  const auto cell = projection.value("cell").toObject();
  const auto robot = cell.value("robot").toObject();
  view.robot = robot.value("model").toString();
  view.planning_group = robot.value("planning_group").toString();
  const auto tool = cell.value("end_effector").toObject();
  view.tool = tool.value("model").toString();
  view.tool_type = tool.value("type").toString();
  view.capabilities = strings(tool.value("capabilities"));
  view.camera = cell.value("camera").toObject().value("id").toString();
  view.perception_mode = cell.value("perception").toObject().value("mode").toString();
  view.perception_contract = cell.value("perception").toObject().value("contract").toString();

  const auto task = projection.value("task").toObject();
  view.task_id = task.value("id").toString();
  view.task_type = task.value("type").toString();
  const auto target = task.value("target").toObject();
  view.target_class = target.value("class_id").toString();
  view.target_color = target.value("color").toString();
  view.min_confidence = number(target.value("min_confidence"));
  view.max_age_seconds = number(target.value("max_age_seconds"));
  view.pick_zone = target.value("pick_zone").toString();
  view.source = target.value("source_type").toString();
  if (!target.value("source_ref").toString().isEmpty()) {
    view.source += QStringLiteral(" · ") + target.value("source_ref").toString();
  }
  view.not_targeted_rule = task.value("not_targeted").toObject().value("rule").toString();
  for (const auto & item : task.value("destinations").toArray()) {
    const auto object = item.toObject();
    const QString asset = object.value("target_ref").toString();
    view.destinations << (asset.isEmpty() ? object.value("id").toString()
                                          : object.value("id").toString() + QStringLiteral(" → ") + asset);
  }
  for (const auto & item : task.value("routing_rules").toArray()) {
    const auto rule = item.toObject();
    const bool always = rule.value("when").toObject().value("always").toBool();
    view.routing_rules << QString("%1: %2 → %3").arg(
      rule.value("id").toString(),
      always ? QStringLiteral("every selected object") : QString::fromUtf8(
        QJsonDocument(rule.value("when").toObject()).toJson(QJsonDocument::Compact)),
      rule.value("destination").toString());
  }
  const auto grasp = task.value("grasp").toObject();
  view.grasp = QStringList{grasp.value("policy").toString(), grasp.value("strategy_ref").toString()}
                 .filter(QRegularExpression(".+")).join(QStringLiteral(" · "));

  const auto validation = projection.value("validation").toObject();
  view.validation_status = validation.value("status").toString();
  view.readiness_classification = validation.value("readiness_classification").toString();
  view.validation_errors = strings(validation.value("errors"));
  view.validation_warnings = strings(validation.value("warnings"));
  view.next_actions = strings(validation.value("suggested_next_actions"));

  const auto resolution = projection.value("resolution").toObject();
  view.resolution_status = resolution.value("status").toString();
  view.resolution_code = resolution.value("primary_code").toString();
  view.resolution_reason = resolution.value("reason").toString();

  const auto reports = projection.value("reports").toObject();
  const auto scene_validation = reports.value("validate_generated_scene").toObject();
  view.scene_validation_status = scene_validation.value("status").toString();
  view.scene_validation_modified = number(scene_validation.value("modified"));
  const auto offline = reports.value("offline_validation").toObject();
  view.offline_validation_status = offline.value("status").toString();
  view.offline_validation_modified = number(offline.value("modified"));
  if (offline.value("current").isBool()) view.offline_validation_current = offline.value("current").toBool();

  const auto input = projection.value("input").toObject();
  view.input_kind = input.value("kind").toString();
  view.input_path = input.value("path").toString();
  view.input_blocker = input.value("blocker").toString();

  if (projection.value("observations").isObject()) {
    const auto observations = projection.value("observations").toObject();
    view.replay_evaluated = true;
    view.replay_error = observations.value("error").toString();
    view.replay_description = observations.value("description").toString();
    view.selected_object = observations.value("selected_object").toString();
    for (const auto & item : observations.value("objects").toArray()) {
      const auto object = item.toObject();
      OperatorObjectRow row;
      row.object_id = object.value("object_id").toString();
      row.class_id = object.value("class_id").toString();
      row.confidence = number(object.value("confidence"));
      row.accepted = object.value("accepted").toBool();
      row.reason_code = object.value("reason_code").toString();
      row.reason = object.value("reason").toString();
      row.has_geometry = vector3(object.value("pose_xyz"), row.xyz) &&
        vector3(object.value("dimensions"), row.dimensions);
      view.objects << row;
    }
  }

  const auto zones = projection.value("zones").toObject();
  view.pick_zone_geometry = zone(zones.value("pick"));
  view.place_zone_geometry = zone(zones.value("place"));

  const auto bundle = projection.value("generated_bundle").toObject();
  view.dry_run_available = bundle.value("dry_run_available").toBool();
  view.dry_run_unavailable_reason = bundle.value("reason").toString();
  const auto execution = projection.value("execution").toObject();
  view.execution_reason = execution.value("reason").toString();
  view.simulation_note = execution.value("simulation").toString();
  view.live_note = execution.value("live").toString();
  view.motion_authorised = execution.value("motion_authorised").toBool(false);

  for (const auto & item : projection.value("events").toArray()) {
    const auto object = item.toObject();
    view.events << OperatorEvent{object.value("level").toString(), object.value("code").toString(),
                                 object.value("message").toString()};
  }
  view.usable = !view.scene_name.isEmpty() && !task.isEmpty();
  return view;
}

OperatorMode operator_mode(const OperatorPreviewView & view, bool replay_shown)
{
  if (view.draft) return OperatorMode::Draft;
  if (replay_shown && view.replay_evaluated) return OperatorMode::Replay;
  return OperatorMode::Offline;
}

OperatorReadiness operator_readiness(const OperatorPreviewView & view)
{
  OperatorReadiness readiness;
  if (!view.usable) readiness.offline_blockers << QStringLiteral("The cell configuration could not be read.");
  if (view.validation_status != QStringLiteral("PASS")) {
    readiness.offline_blockers << (view.validation_errors.isEmpty()
      ? QStringLiteral("Task intent is not valid.") : view.validation_errors.join(QStringLiteral("; ")));
  }
  readiness.offline_checks_allowed = readiness.offline_blockers.isEmpty();

  // Production needs every engineering gate plus a qualified backend. In this
  // release the backend gate never opens, so Start stays NOT AUTHORISED.
  readiness.production_blockers = readiness.offline_blockers;
  if (view.draft) readiness.production_blockers << QStringLiteral("Unsaved draft: save the task in Workcell Builder.");
  if (view.layout_unsaved) readiness.production_blockers << QStringLiteral("Unsaved layout edits in Workcell Builder.");
  if (view.approval_status != QStringLiteral("APPROVED")) {
    readiness.production_blockers << QStringLiteral("No approved generated bundle for this cell.");
  }
  if (view.resolution_status != QStringLiteral("READY") && view.resolution_status != QStringLiteral("WARNING")) {
    readiness.production_blockers << QString("Task resolution %1%2").arg(
      view.resolution_status.isEmpty() ? QStringLiteral("UNKNOWN") : view.resolution_status,
      view.resolution_code.isEmpty() ? QString() : QStringLiteral(" (") + view.resolution_code + QStringLiteral(")"));
  }
  if (view.input_kind != QStringLiteral("live")) {
    readiness.production_blockers << QStringLiteral("No live perception input is bound.");
  }
  if (!view.motion_authorised) {
    readiness.production_blockers << (view.execution_reason.isEmpty()
      ? QStringLiteral("No qualified execution backend.") : view.execution_reason);
  }
  return readiness;
}

GatedDryRunResult parse_gated_dry_run(const QJsonObject & report)
{
  GatedDryRunResult result;
  if (report.isEmpty()) return result;
  result.parsed = true;
  const auto summary = report.value("operator_summary").toObject();
  const auto preflight = report.value("preflight").toObject();
  const auto cycle = report.value("cycle").toObject();
  result.status = summary.value("status").toString();
  result.next_action = summary.value("next_recommended_action").toString();
  result.safe_for_robot_motion = summary.value("safe_for_robot_motion").toBool(false);
  result.selected_object = text(summary.value("selected_object"));
  result.selected_destination = text(summary.value("selected_destination"));
  result.preflight_status = preflight.value("status").toString();
  result.blockers = strings(preflight.value("blockers")) + strings(cycle.value("blockers"));
  result.warnings = strings(preflight.value("warnings")) + strings(cycle.value("warnings"));
  result.cycle_status = cycle.value("status").toString();
  result.cycle_note = cycle.value("note").toString();
  if (result.selected_object.isEmpty()) result.selected_object = text(cycle.value("selected_object"));
  if (result.selected_destination.isEmpty()) result.selected_destination = text(cycle.value("selected_destination"));
  for (const auto & step : cycle.value("task_flow_preview").toObject().value("steps").toArray()) {
    result.stages << step.toObject().value("name").toString(step.toObject().value("step").toString());
  }
  return result;
}

}  // namespace workcell_builder
