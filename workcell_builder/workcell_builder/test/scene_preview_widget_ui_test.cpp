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
