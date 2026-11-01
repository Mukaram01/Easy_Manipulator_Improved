#pragma once

// Plan / Simulate execution monitor: presentation of the runtime's own stage events and failure codes.
//
// This is not a state machine. A presentation category is "reached" only when a real stage event that the
// executor or a runner printed (--stream-status) maps to it; nothing is inferred from elapsed time. Planning
// stages (PREPLAN_*) are counted as planned, never as executed; ATTACH/DETACH before any EXECUTE_* stage are
// planning-scene bookkeeping, not a physical grasp or release.

#include <QDateTime>
#include <QJsonArray>
#include <QJsonObject>
#include <QRegularExpression>
#include <QSet>
#include <QString>
#include <QStringList>

namespace workcell_builder
{

inline const QStringList & monitor_categories()
{
  static const QStringList categories{"Preflight", "Perception", "Planning", "Approach", "Grasp", "Lift",
    "Transfer", "Place", "Release", "Home", "Verification", "Shutdown"};
  return categories;
}

// The nine MoveIt planning stages of a complete cycle (perceived_object_grasp_execute.STAGES).
inline const QStringList & planning_stages()
{
  static const QStringList stages{"PREPLAN_APPROACH", "PREPLAN_GRASP", "PREPLAN_CLOSE_GRIPPER", "PREPLAN_LIFT",
    "PREPLAN_TRANSFER", "PREPLAN_PLACE", "PREPLAN_OPEN_GRIPPER", "PREPLAN_RETREAT", "PREPLAN_HOME"};
  return stages;
}

// Category of a real stage name ("" for session markers such as EXECUTE_FULL_CYCLE that are no category).
inline QString monitor_category(const QString & stage, bool executing)
{
  static const QSet<QString> preflight{"IDLE", "PREFLIGHT", "BUILD_RUNNING", "PACKAGE_CHECK_RUNNING",
    "LAUNCHING_FAKE_HARDWARE", "SIMULATOR_LAUNCH", "SIMULATOR_READY"};
  static const QSet<QString> perception{"PERCEIVING", "ACQUIRE_OBJECTS", "FILTER_TARGETS", "ENUMERATE_TARGETS"};
  static const QSet<QString> planning{"UPDATE_PLANNING_SCENE", "GENERATE_GRASPS", "RESOLVE_TASK", "REVALIDATE_PLAN",
    "GRASP_GEOMETRY", "CANDIDATE_READY", "VERIFY_PREPLAN_UNCHANGED"};
  if (preflight.contains(stage)) return "Preflight";
  if (perception.contains(stage)) return "Perception";
  if (planning.contains(stage) || stage.startsWith("PREPLAN_") || stage.startsWith("SEQUENCE_OBJECT_")) return "Planning";
  if (stage == "ATTACH") return executing ? "Grasp" : "Planning";
  if (stage == "DETACH") return executing ? "Release" : "Planning";
  if (stage == "EXECUTE_APPROACH") return "Approach";
  if (stage == "EXECUTE_GRASP" || stage == "EXECUTE_CLOSE_GRIPPER" || stage == "COMMISSION_GRASP_HOLD") return "Grasp";
  if (stage == "EXECUTE_LIFT") return "Lift";
  if (stage == "EXECUTE_TRANSFER") return "Transfer";
  if (stage == "EXECUTE_PLACE") return "Place";
  if (stage == "EXECUTE_OPEN_GRIPPER") return "Release";
  if (stage == "EXECUTE_RETREAT" || stage == "EXECUTE_HOME") return "Home";
  if (stage == "COMPLETE") return "Verification";
  if (stage == "STOPPING_SIMULATION" || stage == "SIMULATOR_SHUTDOWN") return "Shutdown";
  return {};
}

struct MonitorTimeline
{
  QStringList reached;            // categories with real evidence, in the order first reached
  QString current;                // category of the latest event
  QString failed;                 // category the run failed in (set from the result, never guessed)
  QString interrupted;            // category that was running when the operator asked to cancel
  bool executing{false};          // an EXECUTE_* stage was reported (physics execution started)
  QSet<QString> planned;          // the nine PREPLAN_* stages reported by the latest planning pass
  QSet<QString> executed;         // EXECUTE_* stages reported
  QString last_stage, last_phase;
  bool last_motion{false};
  QDateTime last_time;

  void observe(const QString & stage, const QString & phase, bool motion, const QDateTime & when)
  {
    if (stage == "FAILED") { failed = current; last_stage = stage; last_time = when; return; }
    if (stage.startsWith("EXECUTE_") && stage != "EXECUTE_FULL_CYCLE") { executing = true; executed.insert(stage); }
    if (stage == "PREPLAN_APPROACH") planned.clear();            // a new planning pass starts over
    if (planning_stages().contains(stage)) planned.insert(stage);
    const QString category = monitor_category(stage, executing);
    if (!category.isEmpty()) {
      if (!reached.contains(category)) reached << category;
      current = category;
    }
    last_stage = stage; last_phase = phase; last_motion = motion; last_time = when;
  }
  void reach(const QString & category)
  {
    if (!reached.contains(category)) reached << category;
    current = category;
  }
  int planned_count() const { return static_cast<int>(planned.size()); }

  // One line: reached (✓), current (▶), failed (✗), pending (○); a plan-only session never shows execution.
  QString html(bool physics, bool finished) const
  {
    QStringList parts;
    for (const auto & category : monitor_categories()) {
      const int index = monitor_categories().indexOf(category);
      const bool execution = index >= monitor_categories().indexOf("Approach") && index <= monitor_categories().indexOf("Home");
      if (!physics && execution) {
        if (category == "Approach") parts << "<span style='color:#6b7280'>Approach…Home: not executed (plan only)</span>";
        continue;
      }
      QString label = category;
      if (category == "Planning" && !planned.isEmpty()) label += QString(" (%1/9 planned)").arg(planned_count());
      QString mark = "○", colour = "#9ca3af";
      if (!failed.isEmpty() && category == failed) { mark = "✗"; colour = "#b91c1c"; }
      else if (!interrupted.isEmpty() && category == interrupted) { mark = "■"; colour = "#b45309"; label += " (interrupted)"; }
      else if (reached.contains(category) && (category != current || finished)) { mark = "✓"; colour = "#15803d"; }
      else if (category == current) { mark = "▶"; colour = "#1d4ed8"; }
      parts << QString("<span style='color:%1'>%2 %3</span>").arg(colour, mark, label.toHtmlEscaped());
    }
    return parts.join(" › ");
  }
};

struct FailureDiagnosis
{
  QString code;      // the runtime's own code (or UNCLASSIFIED)
  QString meaning;   // what it means for this run
  QString action;    // what the engineer can do (empty when nothing is to be done)
};

// The runtime's structured code in a failure text: the first CODE_NAME, preferably one followed by ':'.
inline QString failure_code(const QString & text)
{
  static const QRegularExpression leading("\\b([A-Z][A-Z0-9]*(?:_[A-Z0-9]+)+):");
  static const QRegularExpression any("\\b([A-Z][A-Z0-9]*(?:_[A-Z0-9]+)+)\\b");
  auto match = leading.match(text);
  if (!match.hasMatch()) match = any.match(text);
  return match.hasMatch() ? match.captured(1) : QString();
}

inline FailureDiagnosis diagnose_failure(const QString & text)
{
  const QString code = failure_code(text);
  const auto in = [&code](std::initializer_list<const char *> codes) {
    for (const char * c : codes) if (code == QLatin1String(c)) return true;
    return false;
  };
  if (text.contains("cancelled by the operator") || code == "OPERATOR_CANCEL")
    return {"CANCELLED", "Stopped by the operator.", {}};
  if (in({"PERCEPTION_RUNTIME_UNAVAILABLE", "PERCEPTION_RUNTIME_UNREGISTERED"}))
    return {code, "The EPD perception runtime this replayed-perception task detects with is not registered or no longer valid for this workspace.",
      "Register it once: scripts/live_perception_session.py register --epd-setup <EPD install>/setup.bash --ros-domain-id <id>."};
  if (in({"PLACE_OBJECT_DOES_NOT_FIT", "DESTINATION_CONTAINMENT", "PLACE_LOCAL_POSE_OUTSIDE_REGION", "PHYSICAL_DESTINATION_INVALID"}))
    return {code, "The selected object cannot be placed inside the configured destination with this clearance and the planner's placement tolerance.",
      "Task > How to place: reduce the placement clearance (the message names the largest that fits) or choose another destination, then Save, Generate and Validate."};
  if (in({"TARGET_ASSET_NOT_FOUND", "PLACE_REGION_NOT_FOUND", "PLACE_REGION_TARGET_MISMATCH", "R19_DESTINATION_INVALID"}))
    return {code, "The task's destination is not a placement region of its physical target, or its saved geometry is stale.",
      "Scene Builder: select the destination bin in the 3D view and use Inspector > Use Selected as Task Destination "
      "(or Task > Where), then Save, Generate and Validate."};
  if (in({"TASK_ROUTE_DESTINATION_INVALID"}))
    return {code, "A class routing rule names a placement region that is missing, not inside its target, or on a target without collision geometry.",
      "Task > Where > Route by class: name an existing placement region of a collision-modelled bin for every class, then Save, Generate and Validate."};
  if (in({"TASK_ROUTE_CHANGED"}))
    return {code, "The resolved object no longer routes to the destination it was planned for; no other bin is substituted.",
      "Resolve the saved task again (Plan Saved Task), then Generate and Validate."};
  if (in({"TASK_ROUTING_PHYSICS_UNQUALIFIED"}))
    return {code, "Class routing is qualified for fake-hardware planning only; the qualified physics world binds one destination.",
      "Use Plan Saved Task (fake hardware). Physics execution of a routed task needs a separately qualified multi-destination world."};
  if (in({"QUALIFIED_RUNTIME_UNAVAILABLE", "MOVEIT_OVERLAY_REJECTED", "SIMULATOR_IDENTITY_REJECTED", "NO_ACTIVE_SIMULATION",
          "SIMULATOR_EXECUTION_UNCOMMISSIONED"}))
    return {code, "No qualified physics simulator runtime is available, or it no longer matches its qualification.",
      "Register the workspace's qualified runtime from passing Stage-A evidence (scripts/qualified_physics_runtime.py register); physics never runs on an unqualified runtime."};
  if (in({"TASK_HANDOFF_STALE", "TASK_RESOLUTION_STALE"}))
    return {code, "The saved task or its resolution changed after the last Generate, so the generated handoff no longer matches it.",
      "Generate the cell again (Home > More > Generate Package), Validate, then plan."};
  if (in({"OBSERVATION_STALE", "PERCEPTION_STALE", "OBSERVATION_TIME_INVALID", "TIMESTAMP_INVALID", "CLOCK_DOMAIN_MISMATCH"}))
    return {code, "The object observation is too old or carries an invalid time, so it may not be planned against.",
      "Refresh perception (record and detect the replay again, or rerun the live session), then plan again."};
  if (in({"OBSERVATION_UNAVAILABLE", "OBSERVATION_INVALID", "NO_ELIGIBLE_OBSERVATIONS", "TASK_INPUT_UNAVAILABLE", "REPLAY_INPUT_REQUIRED",
          "PERCEPTION_REPLAY_MISSING", "RECORDING_MISSING", "RECORDING_INVALID", "RECORDING_WORLD_MISMATCH", "PERCEPTION_WORLD_MISMATCH",
          "REPLAY_INPUT_INVALID", "CALIBRATION_INVALID", "CALIBRATION_MISSING"}))
    return {code, "There is no valid observation of an object this task may pick.",
      "Task > Object input: choose a valid observation snapshot; after moving objects or the camera, record and detect the replay again."};
  if (in({"OBSERVATION_PRODUCER_REJECTED", "LIVE_INPUT_REQUIRED", "CAMERA_SOURCE_REQUIRED", "CAMERA_SOURCE_MISMATCH"}))
    return {code, "The saved Perception mode does not accept this kind of observation input.",
      "Task > Object input: to plan from a recording on fake hardware choose Recorded observations (replay); for Live EPD, "
      "capture objects with the selected camera. Then Save, Generate and Validate."};
  if (in({"NO_FEASIBLE_CYCLE", "SEARCH_BUDGET_EXHAUSTED", "CANDIDATE_SLICE_EXHAUSTED", "RESOLVED_CYCLE_BLOCKED", "NO_VALID_EXTRACTION",
          "TASK_APPROACH_IK_UNBOUND", "TASK_TRANSFER_IK_UNBOUND", "TARGET_NOT_ELIGIBLE"}))
    return {code, "MoveIt found no collision-free, certified pick-and-place cycle for an eligible grasp: the object or destination is unreachable or blocked.",
      "Keep the object within the robot's reach and clear of obstacles on both grasp axes; the evidence's candidate_attempts name the stage that failed."};
  if (code.contains("COLLISION") || code.contains("CERTIF") || in({"JOINT_LIMIT", "PRECISION_OR_DEPTH_LIMIT", "PLANNING_DEADLINE_EXCEEDED"}))
    return {code, "The collision check or path certificate rejected a motion; that motion was not executed.",
      "Inspect the evidence for the rejected stage. Collision and certificate checks are never relaxed."};
  if (code.startsWith("GRASP_HOLD") || code.contains("RETENTION"))
    return {code, "The simulated fingers did not hold the object (contact lost or not verified); the run stopped.",
      "Keep the object inside the qualified grasp envelope (box about 30 mm wide, at most 45 mm tall, qualified material)."};
  if (code.startsWith("RELEASE_"))
    return {code, "The release could not be verified against the measured arm and gripper state.", "Inspect the evidence before retrying."};
  if (code == "SHUTDOWN_UNCLEAN" || code.startsWith("RECOVERY_"))
    return {code, "The session did not shut down cleanly or needs recovery.",
      "Check the remaining processes listed below before starting another run."};
  if (code == "AUTHORED_TASK_NOT_SIMULATOR")
    return {code, "The saved task does not select the qualified physics simulation.", "Task > Run > Execution backend: Qualified physics simulation, then Save and Generate."};
  if (in({"SCENE_PACKAGE_UNINSTALLED", "SCENE_PACKAGE_FROM_OTHER_CHECKOUT"}))
    return {code, "The installed scene launch is missing or belongs to another checkout.", "Build the workspace with colcon --symlink-install, then retry."};
  if (text.contains("deadline exceeded", Qt::CaseInsensitive) || text.contains("timed out", Qt::CaseInsensitive) ||
      text.contains("TimeoutExpired"))
    return {code.isEmpty() ? QString("EXECUTION_TIMEOUT") : code, "A stage exceeded its time budget.",
      "Check host load (for example another virtual machine), then retry once; the evidence names the stage."};
  return {code.isEmpty() ? QString("UNCLASSIFIED") : code, text.left(300), "See the run's evidence directory for its logs."};
}

// The finished run's outcome lines, from the runner's own result only (cycle_result / plan audit). A value
// the run did not report is "Not available", never assumed.
inline QStringList monitor_outcome_lines(const QJsonObject & result, bool physics)
{
  QStringList lines;
  const auto yes_no = [](const QJsonValue & v) { return v.isBool() ? QString(v.toBool() ? "yes" : "NO") : QString("Not available"); };
  if (physics) {
    const auto e = result.value("physical_evidence").toObject();
    const auto executed = e.value("executed_stages").toArray();
    int ok = 0; for (const auto & s : executed) ok += s.toObject().value("code").toInt() == 1 && s.toObject().value("action_status").toInt() == 4;
    lines << (executed.isEmpty() ? QString("Executed motion: Not available (no executed stage reported)")
                                 : QString("Executed motion: %1/%2 stages succeeded (MoveIt code 1, controller SUCCEEDED)").arg(ok).arg(executed.size()));
    const auto force = e.value("fingertip_force_n").toObject();
    QStringList forces; for (auto it = force.begin(); it != force.end(); ++it) forces << QString::number(it.value().toDouble(), 'f', 1);
    const auto peak = e.value("retention_peak").toObject(), limit = e.value("retention_limits").toObject();
    lines << QString("Grasp (measured in simulation): fingertip force %1 · retention: %2%3")
      .arg(forces.isEmpty() ? QString("Not available") : forces.join(" / ") + " N",
           e.value("strict_retention_compliant").isBool() ? (e.value("strict_retention_compliant").toBool() ? "strict, compliant" : "NOT compliant") : QString("Not available"),
           peak.isEmpty() ? QString() : QString(" (peak %1 mm / %2 mrad; limits %3 mm / %4 mrad)")
             .arg(peak.value("translation_m").toDouble() * 1000, 0, 'f', 2).arg(peak.value("rotation_rad").toDouble() * 1000, 0, 'f', 2)
             .arg(limit.value("translation_m").toDouble() * 1000, 0, 'f', 1).arg(limit.value("rotation_rad").toDouble() * 1000, 0, 'f', 1));
    lines << QString("Release: detached %1 · attached objects left: %2 · ACM restored: %3 · final collision check valid: %4")
      .arg(yes_no(e.value("detached")), e.contains("attached_ids") ? QString::number(e.value("attached_ids").toArray().size()) : QString("Not available"),
           yes_no(e.value("baseline_acm_restored")), yes_no(e.value("final_collision_valid")));
    lines << (e.contains("placement_error_to_planned_m")
      ? QString("Placement: %1 mm from the planned resting pose%2").arg(e.value("placement_error_to_planned_m").toDouble() * 1000, 0, 'f', 3)
          .arg(e.contains("release_disturbance_m") ? QString(" · release disturbance %1 mm").arg(e.value("release_disturbance_m").toDouble() * 1000, 0, 'f', 1) : QString())
      : QString("Placement: Not available (not measured in this run)"));
    const auto s = result.value("session_shutdown").toObject();
    if (s.isEmpty()) lines << "Session shutdown: Not available (no session shutdown record)";
    else {
      QStringList abnormal; for (const auto & a : s.value("abnormal").toArray()) abnormal << QString("%1 exit %2").arg(a.toObject().value("name").toString()).arg(a.toObject().value("returncode").toInt());
      lines << QString("Session shutdown: %1 · %2 owned processes · escalation: %3 · remaining owned processes: %4%5")
        .arg(s.value("clean").toBool() ? "clean" : "NOT clean").arg(s.value("processes").toInt())
        .arg(s.value("escalation").isNull() ? QString("none") : s.value("escalation").toVariant().toString(),
             s.value("remaining_owned_processes").toBool() ? QString("YES") : QString("none"),
             abnormal.isEmpty() ? QString() : " · abnormal: " + abnormal.join(", "));
    }
  } else {
    int planned = 0; for (const auto & p : result.value("plan_metadata").toArray()) planned += p.toObject().value("success").toBool();
    lines << QString("Planned: %1/9 MoveIt stages · Executed: none (plan only; execution goals sent: %2)")
      .arg(planned).arg(result.contains("execution_action_goals") ? QString::number(result.value("execution_action_goals").toArray().size()) : QString("Not available"));
    lines << "Gripper and attach/detach: planned only (planning-scene bookkeeping; no gripper moved, nothing physically held)";
    QStringList crashes; for (const auto & x : result.value("shutdown_crashes").toArray()) crashes << QString("%1 exit %2").arg(x.toObject().value("process").toString()).arg(x.toObject().value("exit_code").toInt());
    lines << QString("Session shutdown: %1 · %2 owned process exits reported · remaining owned process groups: %3%4")
      .arg(result.value("shutdown_clean").isBool() ? (result.value("shutdown_clean").toBool() ? "clean" : "NOT clean") : QString("Not available"))
      .arg(result.value("process_exits").toArray().size())
      .arg(result.contains("remaining_owned_process_groups") ? QString::number(result.value("remaining_owned_process_groups").toArray().size()) : QString("Not available"),
           crashes.isEmpty() ? QString() : " · crashed: " + crashes.join(", "));
  }
  return lines;
}

}  // namespace workcell_builder
