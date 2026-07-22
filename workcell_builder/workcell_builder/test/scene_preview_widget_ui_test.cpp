#include <gtest/gtest.h>

#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QFile>
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
  robot.warnings = QStringList{QStringLiteral("mesh staged")};

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
TEST(ScenePreviewWidgetUi, EmbeddedWebNavigationRequiresVerifiedServerReadiness)
{
  ASSERT_NE(ensure_app(), nullptr);

  ScenePreviewWidget widget;
  widget.preview_scene_name_ = QStringLiteral("ur5_2f_test");
  widget.embedded_web_request_generation_ = 1;
  ScenePreviewWidget::EmbeddedWebRequestIdentity identity;
  identity.scene_id = QStringLiteral("ur5_2f_test");
  identity.absolute_repo_root = QDir::currentPath();
  identity.selected_server_port = 18765;
  identity.payload_revision = 1;
  identity.generation = widget.embedded_web_request_generation_;
  widget.embedded_web_active_identity_ = identity;
  widget.embedded_web_has_active_identity_ = true;

  widget.embedded_web_server_lifecycle_ = ScenePreviewWidget::EmbeddedWebServerLifecycle::ServerProbing;
  widget.load_prepared_embedded_web_scene(identity);
  EXPECT_TRUE(widget.embedded_web_expected_viewer_url_.isEmpty());

  widget.embedded_web_server_lifecycle_ = ScenePreviewWidget::EmbeddedWebServerLifecycle::ServerReady;
  widget.load_prepared_embedded_web_scene(identity);
  EXPECT_EQ(widget.embedded_web_expected_viewer_url_.port(), identity.selected_server_port);
}

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
    identity.absolute_repo_root = QDir::currentPath();
    identity.selected_server_port = 8765;
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


TEST(SceneBuilderWorkspaceSource, CompactTopCommandRowKeepsActionsAndPathAccess)
{
  QFile source(QStringLiteral("workcell_builder/workcell_builder/gui/mainwindow.cpp"));
  if (!source.exists()) source.setFileName(QStringLiteral("../workcell_builder/gui/mainwindow.cpp"));
  ASSERT_TRUE(source.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(source.readAll());

  const int row_index = text.indexOf(QStringLiteral("sceneBuilderCompactCommandRow"));
  ASSERT_GE(row_index, 0);
  const QString row_block = text.mid(row_index, 5200);
  EXPECT_TRUE(row_block.contains(QStringLiteral("new QHBoxLayout")));
  EXPECT_TRUE(row_block.contains(QStringLiteral("setWordWrap(false)")));
  EXPECT_TRUE(row_block.contains(QStringLiteral("sceneBuilderCompactSceneIdentity")));
  EXPECT_TRUE(text.contains(QStringLiteral("ElideMiddle")));
  EXPECT_TRUE(text.contains(QStringLiteral("setToolTip(cleaned_path)")));
  EXPECT_FALSE(row_block.contains(QStringLiteral("sceneBuilderHeaderFilesButton")));
  EXPECT_FALSE(row_block.contains(QStringLiteral("sceneBuilderHeaderSaveLayoutButton")));
  EXPECT_FALSE(row_block.contains(QStringLiteral("sceneBuilderHeaderRunNextButton")));
  EXPECT_FALSE(row_block.contains(QStringLiteral("<h2>Scene Builder</h2>")));
  EXPECT_FALSE(row_block.contains(QStringLiteral("setWordWrap(true)")));

  EXPECT_EQ(text.count(QStringLiteral("sceneBuilderCompactCommandRow")), 1);
  EXPECT_FALSE(text.contains(QStringLiteral("scene_builder_preview_chip_")));
  const int focus_action_index = text.indexOf(QStringLiteral("scene_builder_focus_3d_action_"));
  ASSERT_GE(focus_action_index, 0);
  const QString focus_block = text.mid(focus_action_index, 2200);
  EXPECT_FALSE(focus_block.contains(QStringLiteral("sceneBuilderCompactCommandRow")));
  EXPECT_FALSE(focus_block.contains(QStringLiteral("sceneBuilderHeaderSaveLayoutButton")));
}

TEST(SceneBuilderWorkspaceSource, StatusInformationHasAuthoritativeLocationsOnly)
{
  QFile source(QStringLiteral("workcell_builder/workcell_builder/gui/mainwindow.cpp"));
  if (!source.exists()) source.setFileName(QStringLiteral("../workcell_builder/gui/mainwindow.cpp"));
  ASSERT_TRUE(source.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(source.readAll());
  const QString header = text.mid(text.indexOf(QStringLiteral("sceneBuilderCompactCommandRow")), 5200);
  const QString bottom = text.mid(text.indexOf(QStringLiteral("sceneBuilderBottomStatusBar")), 1900);

  EXPECT_EQ(text.count(QStringLiteral("sceneBuilderCompactSceneIdentity")), 1);
  EXPECT_TRUE(text.contains(QStringLiteral("setToolTip(cleaned_path)")));
  EXPECT_TRUE(text.contains(QStringLiteral("Copy full scene path")));
  EXPECT_FALSE(text.contains(QStringLiteral("selection_scene_path_label_")));
  EXPECT_FALSE(text.contains(QStringLiteral("Selected scene path:")));
  EXPECT_FALSE(header.contains(QStringLiteral("Run Next:")));
  EXPECT_FALSE(bottom.contains(QStringLiteral("Run Next:")));
  EXPECT_FALSE(bottom.contains(QStringLiteral("Selected:")));
  EXPECT_FALSE(bottom.contains(QStringLiteral("No item selected")));
  EXPECT_TRUE(bottom.contains(QStringLiteral("sceneBuilderIssueCount")));
  EXPECT_TRUE(bottom.contains(QStringLiteral("sceneBuilderLogsButton")));
  EXPECT_FALSE(text.contains(QStringLiteral("scene_builder_preview_chip_")));
  EXPECT_FALSE(text.contains(QStringLiteral("scene_preview_label_")));
  EXPECT_FALSE(text.contains(QStringLiteral("canvas_header_label_")));
  EXPECT_TRUE(text.contains(QStringLiteral("studio_log_->append(message)")));
  EXPECT_EQ(text.count(QStringLiteral("connect_button(save_layout_button_, &MainWindow::save_layout_changes)")), 1);
  EXPECT_EQ(text.count(QStringLiteral("connect_if(scene_workflow_recommendation_button_")), 1);
  EXPECT_EQ(text.count(QStringLiteral("if (save_layout_button_) save_layout_button_->click()")), 1);
}

TEST(SceneBuilderWorkspaceSource, ResizablePanelActionsAndFocusAreWired)
{
  QFile source(QStringLiteral("workcell_builder/workcell_builder/gui/mainwindow.cpp"));
  if (!source.exists()) source.setFileName(QStringLiteral("../workcell_builder/gui/mainwindow.cpp"));
  ASSERT_TRUE(source.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(source.readAll());

  EXPECT_TRUE(text.contains(QStringLiteral("sceneBuilderMainSplitter")));
  EXPECT_TRUE(text.contains(QStringLiteral("sceneBuilderLeftPanel")));
  EXPECT_TRUE(text.contains(QStringLiteral("sceneBuilderProductViewPanel")));
  EXPECT_TRUE(text.contains(QStringLiteral("sceneBuilderRightPanel")));
  EXPECT_TRUE(text.contains(QStringLiteral("setStretchFactor(1, 8)")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_splitter->setSizes({320, 1000, 0})")));
  EXPECT_TRUE(text.contains(QStringLiteral("right_panel->setVisible(false)")));
  EXPECT_TRUE(text.contains(QStringLiteral("Show Left Panel")));
  EXPECT_TRUE(text.contains(QStringLiteral("Show Right Panel")));
  EXPECT_TRUE(text.contains(QStringLiteral("Focus 3D View")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder/main_splitter_sizes")));
}

TEST(SceneBuilderWorkspaceSource, FocusModeHidesPanelsWithoutReloadingProductView)
{
  QFile source(QStringLiteral("workcell_builder/workcell_builder/gui/mainwindow.cpp"));
  if (!source.exists()) source.setFileName(QStringLiteral("../workcell_builder/gui/mainwindow.cpp"));
  ASSERT_TRUE(source.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(source.readAll());

  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder_left_panel_->hide()")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder_right_panel_->hide()")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder_left_panel_->setVisible(scene_builder_focus_restore_left_visible_)")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder_right_panel_->setVisible(scene_builder_focus_restore_right_visible_)")));
  const int focus_action_index = text.indexOf(QStringLiteral("scene_builder_focus_3d_action_"));
  ASSERT_GE(focus_action_index, 0);
  const QString focus_block = text.mid(focus_action_index, 2200);
  EXPECT_FALSE(focus_block.contains(QStringLiteral("new ScenePreviewWidget")));
  EXPECT_FALSE(focus_block.contains(QStringLiteral("request_embedded_web_product_view_refresh")));
  EXPECT_FALSE(focus_block.contains(QStringLiteral("reload_meshes")));
}

TEST(SceneBuilderWorkspaceSource, CompactBottomStatusBarUsesSingleRowAndExistingLogDrawer)
{
  QFile source(QStringLiteral("workcell_builder/workcell_builder/gui/mainwindow.cpp"));
  if (!source.exists()) source.setFileName(QStringLiteral("../workcell_builder/gui/mainwindow.cpp"));
  ASSERT_TRUE(source.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(source.readAll());

  const int bar_index = text.indexOf(QStringLiteral("sceneBuilderBottomStatusBar"));
  ASSERT_GE(bar_index, 0);
  const QString bar_block = text.mid(bar_index, 1900);
  EXPECT_EQ(text.count(QStringLiteral("sceneBuilderBottomStatusBar")), 1);
  EXPECT_EQ(text.count(QStringLiteral("sceneBuilderLatestStatus")), 1);
  EXPECT_EQ(text.count(QStringLiteral("sceneBuilderLogDrawer")), 1);
  EXPECT_FALSE(text.contains(QStringLiteral("sceneBuilderSelectionSummary")));
  EXPECT_FALSE(bar_block.contains(QStringLiteral("Selection:")));
  EXPECT_FALSE(bar_block.contains(QStringLiteral("Warnings: 0 | Errors: 0")));
  EXPECT_FALSE(bar_block.contains(QStringLiteral("Product View preview-only; fake hardware remains default")));
  EXPECT_TRUE(bar_block.contains(QStringLiteral("new QHBoxLayout")));
  EXPECT_TRUE(bar_block.contains(QStringLiteral("setContentsMargins(8, 2, 8, 2)")));
  EXPECT_TRUE(bar_block.contains(QStringLiteral("setWordWrap(false)")));
  EXPECT_TRUE(bar_block.contains(QStringLiteral("setVisible(false)")));
  EXPECT_TRUE(bar_block.contains(QStringLiteral("setCheckable(true)")));

  const int append_index = text.indexOf(QStringLiteral("void MainWindow::append_studio_log"));
  ASSERT_GE(append_index, 0);
  const QString append_block = text.mid(append_index, 2300);
  EXPECT_TRUE(append_block.contains(QStringLiteral("message.simplified()")));
  EXPECT_TRUE(append_block.contains(QStringLiteral("setToolTip(concise)")));
  EXPECT_TRUE(append_block.contains(QStringLiteral("elidedText(concise, Qt::ElideRight")));
  EXPECT_TRUE(append_block.contains(QStringLiteral("scene_builder_warning_count_")));
  EXPECT_TRUE(append_block.contains(QStringLiteral("scene_builder_error_count_")));
  EXPECT_TRUE(append_block.contains(QStringLiteral("lowered.contains(\"warn\")")));
  EXPECT_TRUE(append_block.contains(QStringLiteral("lowered.contains(\"error\")")));
  EXPECT_TRUE(append_block.contains(QStringLiteral("lowered.contains(\"failed\")")));
  EXPECT_TRUE(append_block.contains(QStringLiteral("setVisible(!visible_parts.isEmpty())")));
  EXPECT_FALSE(append_block.contains(QStringLiteral("scene_builder_log_panel_->setVisible(true)")));

  const int toggle_index = text.indexOf(QStringLiteral("connect_button(scene_builder_log_toggle_button_"));
  ASSERT_GE(toggle_index, 0);
  const QString toggle_block = text.mid(toggle_index, 650);
  EXPECT_TRUE(toggle_block.contains(QStringLiteral("const bool show = !scene_builder_log_panel_->isVisible()")));
  EXPECT_TRUE(toggle_block.contains(QStringLiteral("scene_builder_log_panel_->setVisible(show)")));
  EXPECT_TRUE(toggle_block.contains(QStringLiteral("studio_log_->setVisible(show)")));
  EXPECT_TRUE(toggle_block.contains(QStringLiteral("setChecked(show)")));
  EXPECT_TRUE(toggle_block.contains(QStringLiteral("setText(\"Logs\")")));
  EXPECT_FALSE(toggle_block.contains(QStringLiteral("new QTextEdit")));
  EXPECT_FALSE(toggle_block.contains(QStringLiteral("clear()")));

  EXPECT_EQ(text.count(QStringLiteral("sceneBuilderLogCopyButton")), 1);
  EXPECT_EQ(text.count(QStringLiteral("sceneBuilderLogClearButton")), 1);
  EXPECT_EQ(text.count(QStringLiteral("connect_button(clear_log")), 1);
  EXPECT_EQ(text.count(QStringLiteral("connect_button(copy_log")), 1);

  const int focus_index = text.indexOf(QStringLiteral("scene_builder_focus_3d_action_"));
  ASSERT_GE(focus_index, 0);
  const QString focus_block = text.mid(focus_index, 2200);
  EXPECT_FALSE(focus_block.contains(QStringLiteral("sceneBuilderBottomStatusBar")));
  EXPECT_FALSE(focus_block.contains(QStringLiteral("scene_builder_log_panel_->setVisible")));
  EXPECT_FALSE(focus_block.contains(QStringLiteral("request_embedded_web_product_view_refresh")));
}
