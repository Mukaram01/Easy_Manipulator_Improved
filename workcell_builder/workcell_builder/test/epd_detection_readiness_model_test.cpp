#include <gtest/gtest.h>

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include "epd_detection_readiness_model.hpp"

using namespace workcell_builder;

namespace {
QJsonObject object(const QString & cls, const QString & state, bool position, bool orientation,
                   const QString & reason = QString(), const QString & track = QString())
{
  QJsonObject o{{"index", 0}, {"class", cls}, {"confidence", 0.95}, {"state", state},
                {"orientation_valid", orientation}, {"note", ""}, {"planning_input", false}};
  o["track_id"] = track.isEmpty() ? QJsonValue() : QJsonValue(track);
  o["position_m"] = position ? QJsonValue(QJsonArray{0.0426, 0.0013, 0.4734}) : QJsonValue();
  o["reasons"] = reason.isEmpty() ? QJsonArray{} : QJsonArray{reason};
  return o;
}

QString write(const QTemporaryDir & dir, const QJsonObject & root)
{
  QDir(dir.path()).mkpath("preview");
  const QString path = epd_readiness_artifact_path(dir.path());
  QFile f(path);
  f.open(QIODevice::WriteOnly);
  f.write(QJsonDocument(root).toJson());
  return path;
}

QJsonObject artifact(const QString & state, double written, const QJsonArray & objects)
{
  return QJsonObject{{"schema_version", "workcell_epd_detection_readiness/v1"},
                     {"written_wall_time", written}, {"connection_state", state},
                     {"connection_reason", "test"}, {"camera_id", "camera"},
                     {"frame_id", "camera_color_optical_frame"}, {"last_report_age_s", 0.2},
                     {"epd_session", 0}, {"objects", objects}, {"contract_errors", QJsonArray{}}};
}
}  // namespace

TEST(EpdReadinessModel, ShowsCandidateLocalisedAndRejectedStates)
{
  QTemporaryDir dir;
  const auto path = write(dir, artifact("FRESH", 100.0, {
    object("banana", "GRASP_CANDIDATE", true, true, "", "8"),
    object("apple", "LOCALISED", true, false, "ambiguous_axis_no_pose", "6"),
    object("banana", "REJECTED", false, false, "overlapping_ownership", "7")}));
  const auto v = load_epd_readiness_view(path, 100.5);
  ASSERT_EQ(v.connection_state, "FRESH");
  ASSERT_EQ(v.rows.size(), 3);
  EXPECT_TRUE(v.rows[0].orientation_valid);
  EXPECT_EQ(v.rows[0].track_id, "8");
  EXPECT_EQ(v.rows[1].state, "LOCALISED");
  EXPECT_TRUE(v.rows[1].has_position);
  EXPECT_FALSE(v.rows[1].orientation_valid);
  EXPECT_EQ(v.rows[1].reasons, "ambiguous_axis_no_pose");
  EXPECT_EQ(format_epd_readiness_position(v.rows[1]), "0.043, 0.001, 0.473");
  EXPECT_FALSE(v.rows[2].has_position);
  EXPECT_EQ(format_epd_readiness_position(v.rows[2]), "not validated");
  EXPECT_NE(format_epd_readiness_summary(v).indexOf("grasp candidates: 1"), -1);
  EXPECT_NE(format_epd_readiness_summary(v).indexOf("Diagnostic only"), -1);
}

TEST(EpdReadinessModel, NeverShowsGeometryBeyondState)
{
  QTemporaryDir dir;
  // A malformed artifact claiming geometry for weaker states is not believed.
  const auto path = write(dir, artifact("FRESH", 100.0, {
    object("apple", "DETECTED", true, true), object("apple", "LOCALISED", true, true)}));
  const auto v = load_epd_readiness_view(path, 100.0);
  ASSERT_EQ(v.rows.size(), 2);
  EXPECT_FALSE(v.rows[0].has_position);
  EXPECT_FALSE(v.rows[0].orientation_valid);
  EXPECT_FALSE(v.rows[1].orientation_valid);
}

TEST(EpdReadinessModel, FrozenArtifactIsStaleAndHidesObjects)
{
  QTemporaryDir dir;
  const auto path = write(dir, artifact("FRESH", 100.0, {object("apple", "GRASP_CANDIDATE", true, true)}));
  const auto v = load_epd_readiness_view(path, 110.0);
  EXPECT_EQ(v.connection_state, "STALE");
  EXPECT_TRUE(v.rows.isEmpty());
}

TEST(EpdReadinessModel, MonitorReportedStaleOrUnavailableHidesObjects)
{
  QTemporaryDir dir;
  for (const QString state : {"STALE", "UNAVAILABLE", "UNKNOWN"}) {
    const auto path = write(dir, artifact(state, 100.0, {object("apple", "GRASP_CANDIDATE", true, true)}));
    const auto v = load_epd_readiness_view(path, 100.0);
    EXPECT_EQ(v.connection_state, state);
    EXPECT_TRUE(v.rows.isEmpty());
  }
}

TEST(EpdReadinessModel, MissingOrMalformedArtifactIsUnknown)
{
  QTemporaryDir dir;
  EXPECT_EQ(load_epd_readiness_view(epd_readiness_artifact_path(dir.path()), 1.0).connection_state, "UNKNOWN");
  EXPECT_EQ(load_epd_readiness_view(QString(), 1.0).connection_state, "UNKNOWN");
  QDir(dir.path()).mkpath("preview");
  QFile f(epd_readiness_artifact_path(dir.path()));
  f.open(QIODevice::WriteOnly);
  f.write("{not json");
  f.close();
  const auto v = load_epd_readiness_view(f.fileName(), 1.0);
  EXPECT_EQ(v.connection_state, "UNKNOWN");
  EXPECT_NE(v.connection_reason.indexOf("malformed"), -1);
}
