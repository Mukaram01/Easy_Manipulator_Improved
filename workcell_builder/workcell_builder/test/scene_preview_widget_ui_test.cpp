#include <gtest/gtest.h>

#include <QApplication>
#include <QComboBox>

#include "scene_preview_widget.h"

namespace {
QApplication * ensure_app()
{
  if (QApplication::instance()) return qobject_cast<QApplication *>(QApplication::instance());
  static int argc = 0;
  static char * argv[] = { nullptr };
  static QApplication app(argc, argv);
  return &app;
}
}

TEST(ScenePreviewWidgetUi, MeshPreviewControlDefaultsToAuto)
{
  ASSERT_NE(ensure_app(), nullptr);

  ScenePreviewWidget widget;
  const auto combos = widget.findChildren<QComboBox *>();
  QComboBox * mesh_combo = nullptr;
  for (auto * combo : combos) {
    if (combo && combo->findText("Auto") >= 0 && combo->findText("Meshes") >= 0 && combo->findText("Primitives") >= 0) {
      mesh_combo = combo;
      break;
    }
  }

  ASSERT_NE(mesh_combo, nullptr);
  EXPECT_EQ(mesh_combo->currentText(), QString("Auto"));
  EXPECT_EQ(widget.mesh_preview_mode(), ScenePreviewWidget::MeshPreviewMode::Auto);
}

TEST(ScenePreviewWidgetUi, MeshPreviewModeToggles)
{
  ASSERT_NE(ensure_app(), nullptr);

  ScenePreviewWidget widget;
  const auto combos = widget.findChildren<QComboBox *>();
  QComboBox * mesh_combo = nullptr;
  for (auto * combo : combos) {
    if (combo && combo->findText("Auto") >= 0 && combo->findText("Meshes") >= 0 && combo->findText("Primitives") >= 0) {
      mesh_combo = combo;
      break;
    }
  }
  ASSERT_NE(mesh_combo, nullptr);

  mesh_combo->setCurrentText("Meshes");
  EXPECT_EQ(widget.mesh_preview_mode(), ScenePreviewWidget::MeshPreviewMode::Meshes);

  mesh_combo->setCurrentText("Primitives");
  EXPECT_EQ(widget.mesh_preview_mode(), ScenePreviewWidget::MeshPreviewMode::Primitives);

  mesh_combo->setCurrentText("Auto");
  EXPECT_EQ(widget.mesh_preview_mode(), ScenePreviewWidget::MeshPreviewMode::Auto);
}


TEST(ScenePreviewWidgetUi, LabelModeDefaultsToSelected)
{
  ASSERT_NE(ensure_app(), nullptr);

  ScenePreviewWidget widget;
  const auto combos = widget.findChildren<QComboBox *>();
  QComboBox * label_combo = nullptr;
  for (auto * combo : combos) {
    if (combo && combo->findText("Important") >= 0 && combo->findText("Selected") >= 0 && combo->findText("All") >= 0) {
      label_combo = combo;
      break;
    }
  }
  ASSERT_NE(label_combo, nullptr);
  EXPECT_EQ(label_combo->currentText(), QString("Selected"));
}

TEST(ScenePreviewWidgetUi, ProductViewDefaultsToIsometric)
{
  ASSERT_NE(ensure_app(), nullptr);

  ScenePreviewWidget widget;
  const auto combos = widget.findChildren<QComboBox *>();
  QComboBox * view_combo = nullptr;
  for (auto * combo : combos) {
    if (combo && combo->findText("Top") >= 0 && combo->findText("Front") >= 0 &&
        combo->findText("Side") >= 0 && combo->findText("Isometric") >= 0 &&
        combo->findText("Fit View") >= 0) {
      view_combo = combo;
      break;
    }
  }

  ASSERT_NE(view_combo, nullptr);
  EXPECT_EQ(view_combo->currentText(), QString("Isometric"));

  view_combo->setCurrentText("Top");
  widget.apply_product_view_defaults();
  EXPECT_EQ(view_combo->currentText(), QString("Isometric"));
}

TEST(ScenePreviewWidgetUi, EquivalentPreviewPayloadsDoNotPrepareProductViewAgain)
{
  ASSERT_NE(ensure_app(), nullptr);

  ScenePreviewWidget::PreviewItem robot;
  robot.id = "robot/base";
  robot.display_name = "UR5 base";
  robot.role = "robot";
  robot.mesh_path = "package://ur_description/meshes/ur5/visual/base.dae";
  robot.mesh_available = true;
  robot.has_material_color = true;
  robot.locked = true;
  robot.editable = false;
  robot.source_layer = "generated_urdf";
  robot.x = 0.25;
  robot.warnings = {"mesh staged"};

  ScenePreviewWidget::PreviewItem table;
  table.id = "layout/table";
  table.display_name = "Workbench";
  table.role = "environment";
  table.linked_to_editable_layout_state = true;
  table.sx = 1.2;

  ScenePreviewWidget widget;
  widget.set_preview_items({robot, table});
  widget.select_preview_item(robot.id);
  const int revision = widget.preview_payload_revision();
  const quint64 generation = widget.preview_payload_generation();
  const quint64 preparations = widget.embedded_web_preparation_request_count();

  // Reordered values are the same effective render/edit payload and a fresh
  // QVector must still be ingested without restarting Web3D preparation.
  widget.set_preview_items({table, robot});
  EXPECT_EQ(widget.preview_payload_revision(), revision);
  EXPECT_EQ(widget.preview_payload_generation(), generation);
  EXPECT_EQ(widget.embedded_web_preparation_request_count(), preparations);
  EXPECT_EQ(widget.selected_preview_item_id(), robot.id);
  ASSERT_NE(widget.preview_item_by_id(robot.id), nullptr);
  EXPECT_EQ(widget.preview_item_by_id(robot.id)->mesh_path, robot.mesh_path);

  robot.material_r = 0.45;  // Material color affects the Scene3D/Web3D render.
  widget.set_preview_items({table, robot});
  EXPECT_EQ(widget.preview_payload_revision(), revision + 1);
  EXPECT_EQ(widget.preview_payload_generation(), generation + 1);
  EXPECT_EQ(widget.embedded_web_preparation_request_count(), preparations + 1);
  EXPECT_EQ(widget.selected_preview_item_id(), robot.id);
}
