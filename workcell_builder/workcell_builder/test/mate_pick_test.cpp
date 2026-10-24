// Shared semantic-pick contract: one deterministic policy maps a viewport hit to
// an authored datum, whichever backend produced the hit.

#include <gtest/gtest.h>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <cmath>

#include "mate_pick.hpp"

using workcell_builder::MatePickGeometry;
using workcell_builder::MateRawPick;
using workcell_builder::parse_mate_frames;
using workcell_builder::resolve_mate_pick;
using workcell_builder::seated_mate_offset;

namespace
{

constexpr double kPi = 3.14159265358979323846;

// Shape of scripts/asset_mounting.py `datum_frames` for the ur5_2f_test bin and
// table (bin bottom/rim are the reviewed catalog datums).
QJsonObject frames_json()
{
  const auto datum = [](const char * type, const char * label, QJsonArray xyz, QJsonArray normal) {
    return QJsonObject{{"datum_type", type}, {"label", label}, {"source", "test"}, {"xyz", xyz},
      {"rpy", QJsonArray{0., 0., 0.}}, {"x_axis", QJsonArray{1., 0., 0.}}, {"normal", normal}};
  };
  return {
    {"bin", QJsonObject{{"world", QJsonObject{{"xyz", QJsonArray{0.45, 0.22, 0.1}}, {"rpy", QJsonArray{0., 0., 0.}}}},
      {"datums", QJsonObject{
        {"origin", datum("origin", "", QJsonArray{0.45, 0.22, 0.1}, QJsonArray{0., 0., 1.})},
        {"bottom", datum("plane", "Bottom face (normal points down)", QJsonArray{0.45, 0.22, 0.0}, QJsonArray{0., 0., -1.})},
        {"rim", datum("plane", "Rim plane (normal points up)", QJsonArray{0.45, 0.22, 0.2}, QJsonArray{0., 0., 1.})},
        {"centre_axis", datum("axis", "Vertical centre axis", QJsonArray{0.45, 0.22, 0.1}, QJsonArray{0., 0., 1.})}}}}},
    {"table", QJsonObject{{"world", QJsonObject{{"xyz", QJsonArray{0.71, -0.2, -0.0009}}, {"rpy", QJsonArray{0., 0., 0.}}}},
      {"datums", QJsonObject{
        {"origin", datum("origin", "", QJsonArray{0.71, -0.2, -0.0009}, QJsonArray{0., 0., 1.})},
        {"surface", datum("plane", "", QJsonArray{0.71, -0.2, -0.0009}, QJsonArray{0., 0., 1.})}}}}},
    {"camera", QJsonObject{{"world", QJsonObject{{"xyz", QJsonArray{0.4, -0.36, 0.85}}, {"rpy", QJsonArray{0., 1.57, 0.}}}},
      {"datums", QJsonObject{{"origin", datum("origin", "", QJsonArray{0.4, -0.36, 0.85}, QJsonArray{1., 0., 0.})}}}}},
  };
}

MateRawPick surface(const QString & asset, std::array<double, 3> point, std::array<double, 3> normal,
  std::array<double, 3> ray = {{0., 0., -1.}})
{
  MateRawPick pick;
  pick.backend = "test"; pick.asset_id = asset; pick.point_world = point; pick.normal_world = normal;
  pick.ray_direction_world = ray; pick.geometry_source = "test mesh";
  return pick;
}

}  // namespace

TEST(MatePick, TableTopMatchesTheReviewedSurfaceWithinTolerance)
{
  const auto frames = parse_mate_frames(frames_json());
  // The table mesh top is 0.9 mm above the reviewed surface datum.
  const auto r = resolve_mate_pick(surface("table", {0.5, -0.1, 0.0}, {0., 0., 1.}), frames);
  ASSERT_TRUE(r.valid) << r.reason.toStdString();
  EXPECT_EQ(r.reason_code, "MATCHED");
  EXPECT_EQ(r.datum_name, "surface");
  EXPECT_NEAR(r.plane_distance_m, 0.0009, 1e-12);
  EXPECT_NEAR(r.normal_angle_deg, 0., 1e-9);
  EXPECT_NEAR(r.point_local[2], 0.0009, 1e-12);
}

TEST(MatePick, BinBottomFromBelowMatchesBottomNotRim)
{
  const auto frames = parse_mate_frames(frames_json());
  // Seen from below: the ray goes up; the stored triangle normal may point either way.
  const auto r = resolve_mate_pick(surface("bin", {0.46, 0.2, 0.0}, {0., 0., 1.}, {0., 0., 1.}), frames);
  ASSERT_TRUE(r.valid) << r.reason.toStdString();
  EXPECT_EQ(r.datum_name, "bottom");
  EXPECT_NEAR(r.normal_world[2], -1., 1e-12);
}

TEST(MatePick, SideWallAndInnerFloorAreNotInventedAsDatums)
{
  const auto frames = parse_mate_frames(frames_json());
  const auto side = resolve_mate_pick(surface("bin", {0.626, 0.22, 0.1}, {1., 0., 0.}, {-1., 0., 0.}), frames);
  EXPECT_FALSE(side.valid);
  EXPECT_EQ(side.reason_code, "NO_MATCHING_DATUM");
  EXPECT_TRUE(side.reason.contains("not a verified mounting surface"));
  // Inner floor faces up, 2 mm above the bottom plane: neither the bottom (opposite normal) nor the rim.
  const auto floor = resolve_mate_pick(surface("bin", {0.45, 0.22, 0.002}, {0., 0., 1.}), frames);
  EXPECT_FALSE(floor.valid);
}

TEST(MatePick, DatumMarkersNameTheDatumExactly)
{
  const auto frames = parse_mate_frames(frames_json());
  MateRawPick pick;
  pick.asset_id = "camera"; pick.geometry = MatePickGeometry::DatumMarker; pick.datum_name = "origin";
  pick.point_world = {0.4, -0.36, 0.85};
  const auto r = resolve_mate_pick(pick, frames);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.reason_code, "DATUM_MARKER");
  EXPECT_EQ(r.datum_type, "origin");
  pick.datum_name = "bottom";
  EXPECT_EQ(resolve_mate_pick(pick, frames).reason_code, "STALE_DATUM");
}

TEST(MatePick, CameraBodyHasNoPlaneDatums)
{
  const auto frames = parse_mate_frames(frames_json());
  const auto r = resolve_mate_pick(surface("camera", {0.4, -0.36, 0.83}, {0., 0., -1.}, {0., 0., 1.}), frames);
  EXPECT_FALSE(r.valid);
  EXPECT_EQ(r.reason_code, "NO_PLANE_DATUMS");
}

TEST(MatePick, RejectsUnknownAssetsStaleViewsAndDegenerateNormals)
{
  const auto frames = parse_mate_frames(frames_json());
  EXPECT_EQ(resolve_mate_pick(surface("pick_zone_main", {0., 0., 0.}, {0., 0., 1.}), frames).reason_code, "NOT_A_MATING_ASSET");
  EXPECT_EQ(resolve_mate_pick(surface("", {0., 0., 0.}, {0., 0., 1.}), frames).reason_code, "NO_ASSET");
  EXPECT_EQ(resolve_mate_pick(surface("table", {0.5, -0.1, 0.0}, {0., 0., 0.}), frames).reason_code, "DEGENERATE_NORMAL");
  EXPECT_EQ(resolve_mate_pick(surface("table", {0.5, -0.1, 0.0}, {std::nan(""), 0., 1.}), frames).reason_code, "DEGENERATE_NORMAL");
  auto stale = surface("table", {0.5, -0.1, 0.0}, {0., 0., 1.});
  stale.has_object_pose = true;
  stale.object_pose_world = {0.76, -0.2, -0.0009, 0., 0., 0.};  // drawn 5 cm away from the authored pose
  EXPECT_EQ(resolve_mate_pick(stale, frames).reason_code, "STALE_VIEW");
  stale.object_pose_world = {0.71, -0.2, -0.0009, 0., 0., 0.};
  EXPECT_TRUE(resolve_mate_pick(stale, frames).valid);
  stale.object_pose_world = {0.71, -0.2, -0.0009, 0., 0., 0.01};  // turned 0.57 degrees
  EXPECT_EQ(resolve_mate_pick(stale, frames).reason_code, "STALE_VIEW");
  auto placeholder = surface("table", {0.5, -0.1, 0.0}, {0., 0., 1.});
  placeholder.geometry_source = "fallback_primitive";
  EXPECT_EQ(resolve_mate_pick(placeholder, frames).reason_code, "PLACEHOLDER_GEOMETRY");
}

TEST(MatePick, CoincidentPlanesAreAmbiguousAndAngleLimitIsExplicit)
{
  auto json = frames_json();
  auto table = json.value("table").toObject();
  auto datums = table.value("datums").toObject();
  datums.insert("top_copy", datums.value("surface"));
  table.insert("datums", datums); json.insert("table", table);
  const auto frames = parse_mate_frames(json);
  EXPECT_EQ(resolve_mate_pick(surface("table", {0.5, -0.1, 0.0}, {0., 0., 1.}), frames).reason_code, "AMBIGUOUS_DATUM");

  const auto plain = parse_mate_frames(frames_json());
  const double tilt = 9.0 * kPi / 180.;
  EXPECT_TRUE(resolve_mate_pick(surface("table", {0.5, -0.1, 0.0}, {std::sin(tilt), 0., std::cos(tilt)}), plain).valid);
  const double steep = 11.0 * kPi / 180.;
  EXPECT_FALSE(resolve_mate_pick(surface("table", {0.5, -0.1, 0.0}, {std::sin(steep), 0., std::cos(steep)}), plain).valid);
}

TEST(MatePick, SeatedOffsetKeepsInPlanePositionAndYawOnly)
{
  const auto seated = seated_mate_offset({0.12, -0.3, 0.0009, 0.002, -0.001, 0.4});
  EXPECT_DOUBLE_EQ(seated[0], 0.12);
  EXPECT_DOUBLE_EQ(seated[1], -0.3);
  EXPECT_DOUBLE_EQ(seated[2], 0.);
  EXPECT_DOUBLE_EQ(seated[3], 0.);
  EXPECT_DOUBLE_EQ(seated[4], 0.);
  EXPECT_DOUBLE_EQ(seated[5], 0.4);
}

TEST(MatePick, ViewerCountersRebaseWhenTheViewerRestartsCounting)
{
  using workcell_builder::MateViewerCounters;
  using workcell_builder::mate_viewer_counters_rebase;
  const MateViewerCounters seen{4, 30, 1};
  EXPECT_FALSE(mate_viewer_counters_rebase(seen, {5, 31, 1}, true));  // a new click and hover: act on them
  EXPECT_FALSE(mate_viewer_counters_rebase(seen, {4, 30, 1}, true));  // nothing new
  EXPECT_TRUE(mate_viewer_counters_rebase(seen, {0, 0, 0}, true));    // a reloaded viewer counts from zero
  EXPECT_TRUE(mate_viewer_counters_rebase(seen, {4, 30, 0}, true));   // any count going backwards
  EXPECT_TRUE(mate_viewer_counters_rebase(seen, {4, 30, 1}, false));  // an inactive viewer records nothing
}
