#include "preview_item_suppression.h"

#include <gtest/gtest.h>

#include <QHash>
#include <QStringList>

namespace
{
ScenePreviewWidget::PreviewItem make_generated_visual(int visual_index, const QString & link_name, const QString & category, const QString & role, const QString & mesh_uri)
{
  ScenePreviewWidget::PreviewItem item;
  item.id = QStringLiteral("generated_urdf::%1::visual_%2::%2").arg(link_name).arg(visual_index);
  item.display_name = link_name;
  item.category = category;
  item.role = role;
  item.source_layer = QStringLiteral("locked_generated_urdf_visual");
  item.active_visual_source = QStringLiteral("mesh_preview");
  item.mesh_path = mesh_uri;
  item.package_uri = mesh_uri;
  item.visual_index_mesh_uri = mesh_uri;
  item.visual_index_package_uri = mesh_uri;
  item.visual_index_link_name = link_name;
  item.visual_index_link = link_name;
  item.visual_index_visual = QStringLiteral("visual_%1").arg(visual_index);
  item.visual_index_visual_name = item.visual_index_visual;
  item.visual_index_value = visual_index;
  item.source_row_index = visual_index;
  item.mesh_available = true;
  item.has_mesh_metadata = true;
  item.locked = true;
  item.editable = false;
  item.selectable = true;
  item.linked_to_editable_layout_state = false;
  item.x = visual_index * 0.01;
  item.y = visual_index * 0.02;
  item.z = visual_index * 0.03;
  return item;
}

ScenePreviewWidget::PreviewItem make_helper_placeholder(const ScenePreviewWidget::PreviewItem & authoritative, const QString & id_suffix)
{
  ScenePreviewWidget::PreviewItem helper = authoritative;
  helper.id = QStringLiteral("semantic_helper::%1::%2_placeholder").arg(authoritative.visual_index_link_name, id_suffix);
  helper.source_layer = QStringLiteral("primitive_fallback");
  helper.active_visual_source = QStringLiteral("primitive_fallback");
  helper.mesh_path.clear();
  helper.package_uri.clear();
  helper.visual_index_mesh_uri.clear();
  helper.visual_index_package_uri.clear();
  helper.visual_index_link.clear();
  helper.visual_index_link_name.clear();
  helper.visual_index_value = -1;
  helper.source_row_index = -1;
  helper.mesh_available = false;
  helper.has_mesh_metadata = false;
  helper.locked = true;
  helper.editable = false;
  helper.linked_to_editable_layout_state = false;
  helper.lock_reason = QStringLiteral("legacy generated_bounds placeholder");
  helper.warnings = QStringList{QStringLiteral("mesh_metadata_missing_or_legacy primitive_preview")};
  return helper;
}

QVector<ScenePreviewWidget::PreviewItem> make_visual_index_rows()
{
  QVector<ScenePreviewWidget::PreviewItem> rows;
  rows << make_generated_visual(0, QStringLiteral("base_link_inertia"), QStringLiteral("robot/ur5"), QStringLiteral("robot"), QStringLiteral("package://ur_description/meshes/ur5/visual/base.dae"));
  rows << make_generated_visual(1, QStringLiteral("shoulder_link"), QStringLiteral("robot/ur5"), QStringLiteral("robot"), QStringLiteral("package://ur_description/meshes/ur5/visual/shoulder.dae"));
  rows << make_generated_visual(2, QStringLiteral("upper_arm_link"), QStringLiteral("robot/ur5"), QStringLiteral("robot"), QStringLiteral("package://ur_description/meshes/ur5/visual/upperarm.dae"));
  rows << make_generated_visual(3, QStringLiteral("forearm_link"), QStringLiteral("robot/ur5"), QStringLiteral("robot"), QStringLiteral("package://ur_description/meshes/ur5/visual/forearm.dae"));
  rows << make_generated_visual(4, QStringLiteral("wrist_1_link"), QStringLiteral("robot/ur5"), QStringLiteral("robot"), QStringLiteral("package://ur_description/meshes/ur5/visual/wrist1.dae"));
  rows << make_generated_visual(5, QStringLiteral("wrist_2_link"), QStringLiteral("robot/ur5"), QStringLiteral("robot"), QStringLiteral("package://ur_description/meshes/ur5/visual/wrist2.dae"));
  rows << make_generated_visual(6, QStringLiteral("wrist_3_link"), QStringLiteral("robot/ur5"), QStringLiteral("robot"), QStringLiteral("package://ur_description/meshes/ur5/visual/wrist3.dae"));
  rows << make_generated_visual(7, QStringLiteral("workbench_top"), QStringLiteral("table/workbench"), QStringLiteral("workbench"), QStringLiteral("package://workbench_description/meshes/workbench_top.dae"));
  rows << make_generated_visual(8, QStringLiteral("camera_link"), QStringLiteral("camera"), QStringLiteral("camera"), QStringLiteral("package://realsense2_description/meshes/d435.dae"));
  for (int i = 9; i <= 17; ++i) {
    rows << make_generated_visual(i, QStringLiteral("robotiq_visual_%1_link").arg(i), QStringLiteral("gripper"), QStringLiteral("gripper"), QStringLiteral("package://robotiq_85_description/meshes/visual/robotiq_%1.dae").arg(i));
  }
  return rows;
}
}  // namespace

TEST(PreviewItemSuppression, PreservesAuthoritativeGeneratedUrdfMeshesAndSuppressesSemanticHelpers)
{
  QVector<ScenePreviewWidget::PreviewItem> items = make_visual_index_rows();
  for (int i = 0; i < items.size(); ++i) {
    if (i <= 6 || i >= 9) {
      items << make_helper_placeholder(items[i], QStringLiteral("matching_pose"));
    }
  }

  ASSERT_TRUE(workcell_builder::is_authoritative_generated_urdf_mesh_preview_item(items.front()));
  ASSERT_TRUE(workcell_builder::is_lower_fidelity_generated_placeholder(items.back()));

  const auto result = workcell_builder::suppress_lower_fidelity_preview_items(items, true);

  QHash<int, int> visual_counts;
  QHash<QString, int> id_counts;
  for (const auto & item : result.items) {
    if (item.visual_index_value >= 0) ++visual_counts[item.visual_index_value];
    ++id_counts[item.id];
    EXPECT_FALSE(item.id.contains(QStringLiteral("::dedupe_1"))) << item.id.toStdString();
  }

  for (int visual = 0; visual <= 17; ++visual) {
    EXPECT_EQ(visual_counts.value(visual), 1) << "visual_" << visual << " should be retained exactly once";
  }

  EXPECT_EQ(id_counts.value(QStringLiteral("generated_urdf::base_link_inertia::visual_0::0")), 1);
  for (int visual = 1; visual <= 6; ++visual) {
    EXPECT_EQ(visual_counts.value(visual), 1) << "UR5 visual_" << visual << " should survive";
  }
  for (int visual = 9; visual <= 17; ++visual) {
    const QString expected_id = QStringLiteral("generated_urdf::robotiq_visual_%1_link::visual_%1::%1").arg(visual);
    EXPECT_EQ(id_counts.value(expected_id), 1) << expected_id.toStdString();
  }

  for (const auto & item : result.items) {
    EXPECT_FALSE(item.id.startsWith(QStringLiteral("semantic_helper::"))) << item.id.toStdString();
  }
  EXPECT_GT(result.suppressed_preview_placeholder_count, 0);
}

TEST(PreviewItemSuppression, ExplicitlyPreservesUr5VisualMeshIndexRowsAgainstSemanticPlaceholders)
{
  QVector<ScenePreviewWidget::PreviewItem> items = make_visual_index_rows();

  ScenePreviewWidget::PreviewItem robot_base;
  robot_base.id = QStringLiteral("robot_base");
  robot_base.display_name = QStringLiteral("Robot Base");
  robot_base.category = QStringLiteral("robot");
  robot_base.role = QStringLiteral("robot_base");
  robot_base.source_layer = QStringLiteral("primitive_fallback");
  robot_base.active_visual_source = QStringLiteral("primitive_fallback");
  robot_base.locked = true;
  robot_base.editable = false;
  robot_base.selectable = true;
  robot_base.mesh_available = false;
  robot_base.has_mesh_metadata = false;
  robot_base.warnings = QStringList{QStringLiteral("semantic placeholder")};
  items << robot_base;

  const auto result = workcell_builder::suppress_lower_fidelity_preview_items(items, true);

  QSet<QString> retained_ur5_links;
  bool retained_robot_base_placeholder = false;
  for (const auto & item : result.items) {
    if (item.package_uri.startsWith(QStringLiteral("package://ur_description/meshes/ur5/visual/")) &&
        item.source_layer == QStringLiteral("locked_generated_urdf_visual") &&
        item.active_visual_source == QStringLiteral("mesh_preview")) {
      retained_ur5_links.insert(item.visual_index_link_name);
      EXPECT_TRUE(item.mesh_available);
      EXPECT_TRUE(item.has_mesh_metadata);
      EXPECT_TRUE(item.selectable);
    }
    if (item.id == QStringLiteral("robot_base")) retained_robot_base_placeholder = true;
  }

  EXPECT_EQ(retained_ur5_links.size(), 7);
  EXPECT_FALSE(retained_robot_base_placeholder);
}
