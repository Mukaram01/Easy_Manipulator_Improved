#include <gtest/gtest.h>

#include <QFile>
#include <QJsonDocument>

#include "runtime_monitor.hpp"

using workcell_builder::MonitorTimeline;
using workcell_builder::diagnose_failure;
using workcell_builder::monitor_category;

namespace
{
const QDateTime kWhen = QDateTime::fromMSecsSinceEpoch(1793188749000);

void observe_all(MonitorTimeline & timeline, const QStringList & stages, const QString & phase = {})
{
  for (const auto & stage : stages) timeline.observe(stage, phase, stage.startsWith("EXECUTE_"), kWhen);
}

// Real executor stage events of a recorded fake-hardware plan-only run (no EXECUTE_* stage).
const QStringList kPlanOnly{"IDLE", "PREFLIGHT", "ACQUIRE_OBJECTS", "FILTER_TARGETS", "UPDATE_PLANNING_SCENE", "GENERATE_GRASPS",
  "PREPLAN_APPROACH", "PREPLAN_GRASP", "PREPLAN_CLOSE_GRIPPER", "GRASP_GEOMETRY", "ATTACH", "PREPLAN_LIFT", "PREPLAN_TRANSFER",
  "PREPLAN_PLACE", "PREPLAN_OPEN_GRIPPER", "DETACH", "PREPLAN_RETREAT", "PREPLAN_HOME", "CANDIDATE_READY", "VERIFY_PREPLAN_UNCHANGED",
  "COMPLETE"};
}  // namespace

TEST(RuntimeMonitor, PlanOnlyIsPlannedNeverExecuted)
{
  MonitorTimeline timeline;
  observe_all(timeline, kPlanOnly);
  EXPECT_FALSE(timeline.executing);
  EXPECT_EQ(timeline.planned_count(), 9);
  EXPECT_TRUE(timeline.executed.isEmpty());
  EXPECT_EQ(timeline.reached, (QStringList{"Preflight", "Perception", "Planning", "Verification"}));  // ATTACH/DETACH: planning scene only
  const auto html = timeline.html(false, true);
  EXPECT_TRUE(html.contains("Planning (9/9 planned)"));
  EXPECT_TRUE(html.contains("not executed (plan only)"));
  EXPECT_FALSE(html.contains("Transfer"));
}

TEST(RuntimeMonitor, PhysicsExecutionIsReachedOnlyByExecutedStages)
{
  MonitorTimeline timeline;
  observe_all(timeline, {"PERCEIVING", "SIMULATOR_LAUNCH", "SIMULATOR_READY", "RESOLVE_TASK"});
  observe_all(timeline, kPlanOnly.mid(0, 20), "resolve");
  EXPECT_FALSE(timeline.reached.contains("Grasp"));                      // a planned ATTACH is not a grasp
  observe_all(timeline, {"EXECUTE_FULL_CYCLE", "PREPLAN_APPROACH", "EXECUTE_APPROACH", "EXECUTE_GRASP", "EXECUTE_CLOSE_GRIPPER", "ATTACH"}, "execute");
  EXPECT_TRUE(timeline.executing);
  EXPECT_EQ(timeline.current, "Grasp");
  EXPECT_EQ(timeline.planned_count(), 1);                               // a new planning pass started over
  EXPECT_FALSE(timeline.reached.contains("Lift"));
  EXPECT_EQ(monitor_category("EXECUTE_FULL_CYCLE", true), "");           // session marker, not a category
  EXPECT_EQ(monitor_category("DETACH", true), "Release");
  EXPECT_TRUE(timeline.html(true, false).contains("▶ Grasp"));
}

TEST(RuntimeMonitor, AFailureMarksTheCategoryItHappenedIn)
{
  MonitorTimeline timeline;
  observe_all(timeline, {"PREFLIGHT", "ACQUIRE_OBJECTS", "PREPLAN_APPROACH", "FAILED"});
  EXPECT_EQ(timeline.failed, "Planning");
  EXPECT_TRUE(timeline.html(true, true).contains("✗ Planning"));
  EXPECT_TRUE(timeline.html(true, true).contains("○ Approach"));         // never reached: pending, not done
}

TEST(RuntimeMonitor, ACancelledCategoryIsInterruptedNotDone)
{
  MonitorTimeline timeline;
  observe_all(timeline, {"PERCEIVING", "SIMULATOR_READY", "RESOLVE_TASK", "PREPLAN_APPROACH"}, "resolve");
  timeline.interrupted = timeline.current;                                // Stop pressed while planning
  observe_all(timeline, {"SIMULATOR_SHUTDOWN"});
  const auto html = timeline.html(true, true);
  EXPECT_TRUE(html.contains("■ Planning (1/9 planned) (interrupted)")) << html.toStdString();
  EXPECT_FALSE(html.contains("✓ Planning"));
  EXPECT_TRUE(html.contains("✓ Shutdown"));
}

TEST(RuntimeMonitor, RuntimeFailureCodesAreExplainedWithAnAction)
{
  auto d = diagnose_failure("PERCEPTION_RUNTIME_UNAVAILABLE: no perception runtime is registered for workspace /w");
  EXPECT_EQ(d.code, "PERCEPTION_RUNTIME_UNAVAILABLE");
  EXPECT_TRUE(d.action.contains("live_perception_session.py register"));
  d = diagnose_failure("BLOCKED\nPLACE_OBJECT_DOES_NOT_FIT: runtime::sample-cup cannot be placed at default_drop_zone: z ...");
  EXPECT_EQ(d.code, "PLACE_OBJECT_DOES_NOT_FIT");
  EXPECT_TRUE(d.action.contains("clearance"));
  EXPECT_EQ(diagnose_failure("SIMULATOR_IDENTITY_REJECTED: live process mismatch").code, "SIMULATOR_IDENTITY_REJECTED");
  EXPECT_EQ(diagnose_failure("OBSERVATION_STALE: runtime::cup is 4.1 s old").code, "OBSERVATION_STALE");
  // A stale generated handoff is a Generate step, not a perception refresh.
  EXPECT_TRUE(diagnose_failure("TASK_HANDOFF_STALE: Generate the current resolution before consuming it").action.contains("Generate"));
  EXPECT_FALSE(diagnose_failure("TASK_HANDOFF_STALE: Generate the current resolution").meaning.contains("observation"));
  EXPECT_TRUE(diagnose_failure("NO_FEASIBLE_CYCLE: 13/13 candidates").meaning.contains("unreachable"));
  EXPECT_TRUE(diagnose_failure("GRASP_HOLD_CONTACT_LOST at lift").meaning.contains("did not hold"));
  EXPECT_EQ(diagnose_failure("cancelled by the operator (SIGTERM)").code, "CANCELLED");
  EXPECT_EQ(diagnose_failure("launch/planning deadline exceeded").code, "EXECUTION_TIMEOUT");
  EXPECT_EQ(diagnose_failure("SHUTDOWN_UNCLEAN: move_group exit -11 after the owned stop").code, "SHUTDOWN_UNCLEAN");
  // R2.4A class routing: a wrong route, a substituted bin, and physics outside its one-destination qualification.
  EXPECT_TRUE(diagnose_failure("TASK_ROUTE_DESTINATION_INVALID: rule route_2: destination 'x': unknown").action.contains("Route by class"));
  EXPECT_TRUE(diagnose_failure("TASK_ROUTE_CHANGED: cup no longer routes").meaning.contains("no other bin is substituted"));
  EXPECT_TRUE(diagnose_failure("BLOCKED\nTASK_ROUTING_PHYSICS_UNQUALIFIED: class routing ...").action.contains("Plan Saved Task"));
  const auto unknown = diagnose_failure("something unexpected");
  EXPECT_EQ(unknown.code, "UNCLASSIFIED");
  EXPECT_EQ(unknown.meaning, "something unexpected");                     // the runtime's own text, never replaced
}

namespace
{
QJsonObject fixture(const char * name)
{
  QFile file(QString(RUNTIME_MONITOR_FIXTURES) + "/" + name);
  EXPECT_TRUE(file.open(QIODevice::ReadOnly)) << name;
  return QJsonDocument::fromJson(file.readAll()).object();
}
}  // namespace

// The recorded Stage-B GUI physics cycle (cycle_result of its own report and summary, 2026-10-28).
TEST(RuntimeMonitor, PhysicsOutcomeComesFromTheRecordedRunsMeasurements)
{
  const auto lines = workcell_builder::monitor_outcome_lines(fixture("physics_cycle_result_stage_b_20261028.json"), true).join("\n");
  EXPECT_TRUE(lines.contains("Executed motion: 9/9 stages succeeded")) << lines.toStdString();
  EXPECT_TRUE(lines.contains("fingertip force 20.0 / 20.1 N")) << lines.toStdString();
  EXPECT_TRUE(lines.contains("strict, compliant (peak 0.11 mm / 0.11 mrad; limits 2.0 mm / 10.0 mrad)")) << lines.toStdString();
  EXPECT_TRUE(lines.contains("detached yes · attached objects left: 0 · ACM restored: yes · final collision check valid: yes"));
  EXPECT_TRUE(lines.contains("Placement: 0.028 mm from the planned resting pose")) << lines.toStdString();
  EXPECT_TRUE(lines.contains("Session shutdown: clean · 10 owned processes · escalation: none · remaining owned processes: none"));
  // The same result without measurements says so instead of inventing them.
  auto unmeasured = fixture("physics_cycle_result_stage_b_20261028.json");
  unmeasured.remove("physical_evidence"); unmeasured.remove("session_shutdown");
  const auto bare = workcell_builder::monitor_outcome_lines(unmeasured, true).join("\n");
  EXPECT_TRUE(bare.contains("Executed motion: Not available"));
  EXPECT_TRUE(bare.contains("fingertip force Not available ·"));
  EXPECT_TRUE(bare.contains("Placement: Not available"));
  EXPECT_TRUE(bare.contains("Session shutdown: Not available"));
}

// A recorded fake-hardware Plan Saved Task (its acceptance record, 2026-10-28): planned, never executed.
TEST(RuntimeMonitor, PlanOnlyOutcomeIsPlannedNotExecuted)
{
  const auto lines = workcell_builder::monitor_outcome_lines(fixture("plan_only_result_cup_20261028.json"), false).join("\n");
  EXPECT_TRUE(lines.contains("Planned: 9/9 MoveIt stages · Executed: none (plan only; execution goals sent: 0)")) << lines.toStdString();
  EXPECT_TRUE(lines.contains("nothing physically held"));
  EXPECT_TRUE(lines.contains("Session shutdown: clean · 9 owned process exits reported · remaining owned process groups: 0"));
  EXPECT_FALSE(lines.contains("fingertip"));
}
