#include <gtest/gtest.h>

#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QTemporaryDir>
#include <QUrlQuery>

#define private public
#include "scene_preview_widget.h"
#undef private
#include "scene3d_viewport_widget.h"

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
#include <QWebEngineView>
#endif

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

TEST(ScenePreviewWidgetUi, EquivalentPreviewContextsPrepareProductViewOnlyOnce)
{
  ASSERT_NE(ensure_app(), nullptr);

  QTemporaryDir temporary_root;
  ASSERT_TRUE(temporary_root.isValid());
  const QString repo_root = temporary_root.path();
  const QString scene_dir = QDir(repo_root).filePath("scenes/ur5_2f_test");
  ASSERT_TRUE(QDir().mkpath(scene_dir));

  ScenePreviewWidget::PreviewContext first_context;
  first_context.scene_id = "  ur5_2f_test  ";
  first_context.absolute_scene_dir = QDir(repo_root).filePath("scenes/./ur5_2f_test");
  first_context.absolute_repo_root = repo_root + "/.";

  ScenePreviewWidget::PreviewContext equivalent_context;
  equivalent_context.scene_id = "ur5_2f_test";
  equivalent_context.absolute_scene_dir = scene_dir;
  equivalent_context.absolute_repo_root = repo_root;

  ScenePreviewWidget widget;
  const quint64 preparations = widget.embedded_web_preparation_request_count();
  widget.set_preview_context(first_context);
  EXPECT_EQ(widget.embedded_web_preparation_request_count(), preparations + 1);

  widget.set_preview_context(equivalent_context);
  widget.set_preview_context(first_context);
  widget.set_preview_scene_name(" ur5_2f_test ");
  EXPECT_EQ(widget.embedded_web_preparation_request_count(), preparations + 1);
}

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
TEST(ScenePreviewWidgetUi, EmbeddedWebViewerUrlPreservesScenePathAndPayloadRevision)
{
  ASSERT_NE(ensure_app(), nullptr);

  const QList<quint64> revisions{1, 2, 3, 4, 9, 10, 12, 38, 125};
  for (const quint64 revision : revisions) {
    ScenePreviewWidget widget;
    widget.preview_scene_name_ = QStringLiteral("ur5_2f_test");
    widget.embedded_web_request_generation_ = 1;
    widget.embedded_web_server_lifecycle_ = ScenePreviewWidget::EmbeddedWebServerLifecycle::ServerReady;
    widget.embedded_web_server_port_ = 8765;

    ScenePreviewWidget::EmbeddedWebRequestIdentity identity;
    identity.scene_id = QStringLiteral("ur5_2f_test");
    identity.payload_revision = revision;
    identity.generation = widget.embedded_web_request_generation_;
    widget.load_prepared_embedded_web_scene(identity);

    const QString logical_scene_path = QStringLiteral("build/workcell_studio_web_scene/ur5_2f_test.web_scene.json");
    const QString expected_revision = QString::number(revision);
    const QUrl viewer_url = widget.embedded_web_expected_viewer_url_;
    const QUrlQuery query(viewer_url);
    const QList<QPair<QString, QString>> query_items = query.queryItems(QUrl::FullyDecoded);
    const auto query_item_count = [&query_items](const QString & key) {
      int count = 0;
      for (const auto & item : query_items) {
        if (item.first == key) ++count;
      }
      return count;
    };
    EXPECT_EQ(viewer_url.scheme(), QStringLiteral("http"));
    EXPECT_EQ(viewer_url.host(), QStringLiteral("127.0.0.1"));
    EXPECT_EQ(viewer_url.port(), 8765);
    EXPECT_EQ(viewer_url.path(), QStringLiteral("/workcell_studio_web/viewer/index.html"));
    EXPECT_EQ(query.queryItemValue(QStringLiteral("scene"), QUrl::FullyDecoded), logical_scene_path);
    EXPECT_EQ(query.queryItemValue(QStringLiteral("builderRevision"), QUrl::FullyDecoded), expected_revision);
    EXPECT_EQ(query.queryItemValue(QStringLiteral("embedded"), QUrl::FullyDecoded), QStringLiteral("1"));
    EXPECT_EQ(query_item_count(QStringLiteral("scene")), 1);
    EXPECT_EQ(query_item_count(QStringLiteral("builderRevision")), 1);
    EXPECT_EQ(query_item_count(QStringLiteral("embedded")), 1);

    const QString encoded_url = viewer_url.toString(QUrl::FullyEncoded);
    EXPECT_EQ(encoded_url.indexOf(QStringLiteral("build1F")), -1);
    EXPECT_EQ(encoded_url.indexOf(QStringLiteral("build2F")), -1);
    EXPECT_EQ(encoded_url.indexOf(QStringLiteral("build3F")), -1);
    EXPECT_EQ(encoded_url.indexOf(QStringLiteral("build4F")), -1);
    EXPECT_EQ(encoded_url.indexOf(QStringLiteral("build12F")), -1);
    EXPECT_EQ(encoded_url.indexOf(QStringLiteral("%252F")), -1);
  }
}

TEST(ScenePreviewWidgetUi, PreparationFailureUsesPopulatedCompatibilityViewport)
{
  ASSERT_NE(ensure_app(), nullptr);

  ScenePreviewWidget::PreviewItem table;
  table.id = "layout/table";
  table.display_name = "Workbench";
  table.role = "environment";
  table.linked_to_editable_layout_state = true;
  table.sx = 1.2;
  table.sy = 0.8;
  table.sz = 0.75;

  ScenePreviewWidget widget;
  widget.resize(800, 600);
  widget.show();
  widget.set_preview_items({table});
  widget.select_preview_item(table.id);
  widget.activate_native_compatibility_preview("test preparation failure");
  QApplication::processEvents();

  auto * web_view = widget.findChild<QWebEngineView *>("embeddedWeb3dProductView");
  auto * compatibility_viewport = widget.findChild<Scene3DViewportWidget *>("nativeCompatibilityScene3dViewport");
  ASSERT_NE(web_view, nullptr);
  ASSERT_NE(compatibility_viewport, nullptr);
  EXPECT_TRUE(web_view->isHidden());
  EXPECT_TRUE(compatibility_viewport->isVisible());
  EXPECT_EQ(compatibility_viewport->items.size(), 1);
  EXPECT_EQ(compatibility_viewport->selected_id, table.id);
  EXPECT_EQ(widget.selected_preview_item_id(), table.id);
  EXPECT_GT(compatibility_viewport->render_debug_counters().viewport_received_count, 0);
  EXPECT_EQ(widget.runtime_preview_status_text(), QStringLiteral("Preview available in compatibility mode"));
  EXPECT_NE(widget.runtime_preview_status_text(), QStringLiteral("Preview failed"));
}
#endif
