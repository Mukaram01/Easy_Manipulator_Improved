#include <gtest/gtest.h>
#include "scene3d_visual_classification.h"

using workcell_builder::scene3d_visual_classification::has_actionable_visual_warning;

TEST(Scene3DActionableWarning, HealthyLockedAndSemanticItemsAreNotDefects)
{
  ScenePreviewWidget::PreviewItem item;
  item.locked = true;
  item.lock_reason = QStringLiteral("item is locked");
  item.status = QStringLiteral("ready");
  item.source_layer = QStringLiteral("locked_generated_urdf_visual");
  EXPECT_FALSE(has_actionable_visual_warning(item));
  item.source_layer = QStringLiteral("overlay");
  item.semantic_task_zone_helper = true;
  EXPECT_FALSE(has_actionable_visual_warning(item));
}

TEST(Scene3DActionableWarning, HelpersNeverMaskActualErrors)
{
  ScenePreviewWidget::PreviewItem item;
  item.source_layer = QStringLiteral("overlay");
  item.semantic_task_zone_helper = true;
  item.status = QStringLiteral("warning");
  EXPECT_TRUE(has_actionable_visual_warning(item));
  item.status = QStringLiteral("ready");
  for (const auto & warning : {"helper transform is invalid", "unsupported primitive geometry",
                              "overlay metadata missing", "locked item references missing frame"}) {
    item.warnings = QStringList{QString::fromUtf8(warning)};
    EXPECT_TRUE(has_actionable_visual_warning(item)) << warning;
  }
  item.warnings.clear();
  item.mesh_load_warning = QStringLiteral("mesh resolution failed");
  EXPECT_TRUE(has_actionable_visual_warning(item));
}
