#include <gtest/gtest.h>

#include "datum_mating_text.hpp"

using namespace workcell_builder;

TEST(DatumMatingText, DescribesAFlushPlaneMate)
{
  const auto relation = parse_mount_relation(QStringLiteral(
    "{parent_object: support_surface_table, parent_link: surface, joint_type: fixed,"
    " origin: {x: -0.2607, y: 0.42, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 1.5707963267948966},"
    " mate: {child_datum: bottom, alignment: flush}}"));
  ASSERT_TRUE(relation.mounted);
  EXPECT_TRUE(relation.datum_mate);
  EXPECT_EQ(relation.child_datum, QStringLiteral("bottom"));
  EXPECT_EQ(mount_relation_short(relation), QStringLiteral("bottom flush on surface"));
  const QString summary = mount_relation_summary(relation, QStringLiteral("Industrial Workbench"));
  EXPECT_TRUE(summary.startsWith(QStringLiteral("Mated: bottom flush (face to face) to Industrial Workbench · surface")));
  EXPECT_TRUE(summary.contains(QStringLiteral("X -0.261 · Y 0.420 · Z 0.000 m")));
  EXPECT_TRUE(summary.contains(QStringLiteral("yaw 90.0°")));
  EXPECT_EQ(summary.count(QLatin1Char('\n')), 2);  // short lines; the Inspector never widens
  const auto negative_zero = parse_mount_relation(QStringLiteral(
    "{parent_object: t, parent_link: surface, origin: {x: -0.0, y: -0.0001, z: 0, roll: -0.0, pitch: 0, yaw: -0.0001}}"));
  EXPECT_TRUE(mount_relation_summary(negative_zero, QString()).contains(QStringLiteral("Offset X 0.000 · Y 0.000 · Z 0.000 m\nRoll 0.0° · pitch 0.0° · yaw 0.0°")));
}

TEST(DatumMatingText, KeepsV1MountsAndWorldItemsDistinct)
{
  const auto v1 = parse_mount_relation(QStringLiteral(
    "{parent_object: camera_01, parent_link: origin, joint_type: fixed, origin: {x: 0.15, y: 0, z: 1.2, roll: 0, pitch: 1.5707963267948966, yaw: 0}}"));
  ASSERT_TRUE(v1.mounted);
  EXPECT_FALSE(v1.datum_mate);
  EXPECT_EQ(mount_relation_short(v1), QStringLiteral("mounted at origin"));
  EXPECT_TRUE(mount_relation_summary(v1, QString()).startsWith(QStringLiteral("Mounted to camera_01 · origin")));
  for (const auto & world : {QStringLiteral("{}"), QStringLiteral("surface_z_m: 0.0"), QStringLiteral("{parent_object: world}"), QStringLiteral(": not yaml [")}) {
    const auto relation = parse_mount_relation(world);
    EXPECT_FALSE(relation.mounted);
    EXPECT_TRUE(mount_relation_short(relation).isEmpty());
    EXPECT_EQ(mount_relation_summary(relation, QString()), QStringLiteral("Not mounted · placed in world coordinates"));
  }
  EXPECT_EQ(datum_choice_text(QStringLiteral("top"), QStringLiteral("plane"), QStringLiteral("Top mounting face")),
    QStringLiteral("top · plane — Top mounting face"));
}

TEST(DatumMatingText, ComparesRelationsIndependentOfYamlStyle)
{
  const QString flow = QStringLiteral(
    "{\"parent_object\": \"t\", \"parent_link\": \"surface\", \"origin\": {\"x\": 0.15, \"y\": -0.0, \"z\": 1e-18},"
    " \"mate\": {\"child_datum\": \"bottom\", \"alignment\": \"flush\"}}");
  const QString block = QStringLiteral(
    "parent_object: t\nparent_link: surface\norigin:\n  x: 0.15\n  y: 0.0\n  z: 0\nmate:\n  child_datum: bottom\n  alignment: flush\n");
  EXPECT_TRUE(same_mounting_metadata(flow, block));
  EXPECT_FALSE(same_mounting_metadata(flow, QString(block).replace(QStringLiteral("flush"), QStringLiteral("coincident"))));
  EXPECT_FALSE(same_mounting_metadata(flow, QString(block).replace(QStringLiteral("x: 0.15"), QStringLiteral("x: 0.151"))));
  EXPECT_FALSE(same_mounting_metadata(QStringLiteral("{}"), block));
}
