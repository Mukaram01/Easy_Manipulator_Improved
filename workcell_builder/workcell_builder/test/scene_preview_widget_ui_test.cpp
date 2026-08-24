#include <gtest/gtest.h>

#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QProcess>
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

TEST(ScenePreviewWidgetUi, AuthoringOverlayPreservesCatalogIdentityScaleAndDistinctInstances)
{
  ScenePreviewWidget::PreviewItem first;
  first.id = QStringLiteral("object_01");
  first.display_name = QStringLiteral("Imported test asset");
  first.category = QStringLiteral("Imported");
  first.catalog_asset_id = QStringLiteral("imported_test_asset");
  first.mesh_type = QStringLiteral("stl");
  first.mesh_scale_x = first.mesh_scale_y = first.mesh_scale_z = 0.001;

  ScenePreviewWidget::PreviewItem second = first;
  second.id = QStringLiteral("object_02");

  const QJsonObject first_overlay = ScenePreviewWidget::authoring_overlay_item(
    first, QStringLiteral("/tmp/imported_test_asset.stl"));
  const QJsonObject second_overlay = ScenePreviewWidget::authoring_overlay_item(
    second, QStringLiteral("/tmp/imported_test_asset.stl"));

  EXPECT_EQ(first_overlay.value(QStringLiteral("id")).toString(), QStringLiteral("object_01"));
  EXPECT_EQ(second_overlay.value(QStringLiteral("id")).toString(), QStringLiteral("object_02"));
  EXPECT_NE(first_overlay.value(QStringLiteral("id")), second_overlay.value(QStringLiteral("id")));
  EXPECT_EQ(first_overlay.value(QStringLiteral("asset_id")).toString(), QStringLiteral("imported_test_asset"));
  EXPECT_EQ(second_overlay.value(QStringLiteral("asset_id")).toString(), QStringLiteral("imported_test_asset"));
  const QJsonArray scale = first_overlay.value(QStringLiteral("mesh_scale")).toArray();
  ASSERT_EQ(scale.size(), 3);
  EXPECT_DOUBLE_EQ(scale.at(0).toDouble(), 0.001);
  EXPECT_DOUBLE_EQ(scale.at(1).toDouble(), 0.001);
  EXPECT_DOUBLE_EQ(scale.at(2).toDouble(), 0.001);
  EXPECT_EQ(first_overlay.value(QStringLiteral("source_mesh_path")).toString(),
    QStringLiteral("/tmp/imported_test_asset.stl"));
}

TEST(ScenePreviewWidgetUi, DirtySessionRefreshPreservesImportedCatalogMetadataThroughOverlay)
{
  using workcell_builder::WorkcellStudioCanvasItem;
  using workcell_builder::WorkcellStudioCanvasModel;
  using workcell_builder::WorkcellStudioItemProvenance;

  auto imported_instance = [](const std::string & id) {
    WorkcellStudioCanvasItem item;
    item.id = id;
    item.label = "Imported test asset";
    item.category = "Imported";
    item.catalog_asset_id = "imported_test_asset";
    item.source_file = "/tmp/imported_test_asset.stl";
    item.mesh_path = item.source_file;
    item.mesh_type = "stl";
    item.mesh_scale_x = item.mesh_scale_y = item.mesh_scale_z = 0.001;
    item.provenance = WorkcellStudioItemProvenance::EditableLayout;
    item.editable = true;
    return item;
  };

  const WorkcellStudioCanvasItem object_01 = imported_instance("object_01");
  const WorkcellStudioCanvasItem object_02 = imported_instance("object_02");
  WorkcellStudioCanvasModel rebuilt_disk_model;
  workcell_builder::merge_dirty_editable_layout_session(
    rebuilt_disk_model, {object_01, object_02}, {});
  ASSERT_EQ(rebuilt_disk_model.items.size(), 2U);

  for (const auto & rebuilt : rebuilt_disk_model.items) {
    const auto preview = ScenePreviewWidget::preview_item_from_canvas_item(rebuilt);
    const auto overlay = ScenePreviewWidget::authoring_overlay_item(preview, preview.mesh_path);
    EXPECT_EQ(preview.id, QString::fromStdString(rebuilt.id));
    EXPECT_EQ(preview.catalog_asset_id, QStringLiteral("imported_test_asset"));
    EXPECT_DOUBLE_EQ(preview.mesh_scale_x, 0.001);
    EXPECT_DOUBLE_EQ(preview.mesh_scale_y, 0.001);
    EXPECT_DOUBLE_EQ(preview.mesh_scale_z, 0.001);
    EXPECT_EQ(overlay.value(QStringLiteral("id")).toString(), preview.id);
    EXPECT_EQ(overlay.value(QStringLiteral("asset_id")).toString(),
      QStringLiteral("imported_test_asset"));
    EXPECT_NE(overlay.value(QStringLiteral("asset_id")).toString(), QStringLiteral("Imported"));
  }
  EXPECT_NE(rebuilt_disk_model.items[0].id, rebuilt_disk_model.items[1].id);
  EXPECT_EQ(rebuilt_disk_model.items[0].catalog_asset_id,
    rebuilt_disk_model.items[1].catalog_asset_id);
}

#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
ScenePreviewWidget::PreviewItem stale_editable_item(const QString & id, double x, double y, double z)
{
  ScenePreviewWidget::PreviewItem item;
  item.id = id;
  item.catalog_asset_id = QStringLiteral("shared_catalog_asset");
  item.source_layer = QStringLiteral("editable_layout");
  item.mesh_path = QStringLiteral("/tmp/shared_catalog_asset.stl");
  item.editable = true;
  item.locked = false;
  item.x = x;
  item.y = y;
  item.z = z;
  return item;
}

void stop_preparation(ScenePreviewWidget & widget)
{
  if (!widget.embedded_web_prepare_process_) return;
  widget.embedded_web_prepare_process_->kill();
  widget.embedded_web_prepare_process_->waitForFinished();
}

ScenePreviewWidget::EmbeddedWebRequestIdentity publication_identity(
  const QString & root, const QString & scene, quint64 revision, quint64 generation,
  const QByteArray & fingerprint)
{
  ScenePreviewWidget::EmbeddedWebRequestIdentity identity;
  identity.scene_id = scene;
  identity.absolute_scene_dir = QDir(root).filePath(QStringLiteral("scenes/%1").arg(scene));
  identity.absolute_repo_root = root;
  identity.selected_server_port = 18765;
  identity.product_view_backend = QStringLiteral("embedded_web3d");
  identity.request_owned_output_path = QStringLiteral("build/workcell_studio_web_scene/requests/%1-r%2-g%3-%4.web_scene.json")
    .arg(scene).arg(revision).arg(generation).arg(QString::fromLatin1(fingerprint.toHex()));
  identity.generated_web_scene_path = QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(scene);
  identity.payload_revision = revision;
  identity.generation = generation;
  identity.payload_fingerprint = fingerprint;
  return identity;
}

QProcess * completed_preparation(
  ScenePreviewWidget & widget, const ScenePreviewWidget::EmbeddedWebRequestIdentity & identity,
  const QString & marker)
{
  const QString absolute_output = QDir(identity.absolute_repo_root).filePath(identity.request_owned_output_path);
  EXPECT_TRUE(QDir().mkpath(QFileInfo(absolute_output).absolutePath()));
  QFile output(absolute_output);
  EXPECT_TRUE(output.open(QIODevice::WriteOnly));
  output.write(QJsonDocument(QJsonObject{
    {QStringLiteral("schema_version"), QStringLiteral("workcell_studio_web_scene/v1")},
    {QStringLiteral("scene_id"), identity.scene_id},
    {QStringLiteral("marker"), marker}
  }).toJson(QJsonDocument::Compact));
  output.close();

  auto * process = new QProcess(&widget);
  ScenePreviewWidget::EmbeddedWebPreparationDiagnostic diagnostic;
  diagnostic.identity = identity;
  diagnostic.expected_output_path = identity.request_owned_output_path;
  diagnostic.expected_output_absolute_path = absolute_output;
  diagnostic.started = true;
  diagnostic.stdout_tail = QJsonDocument(QJsonObject{
    {QStringLiteral("schema_version"), QStringLiteral("workcell_studio_web_scene_freshener/v1")},
    {QStringLiteral("status"), QStringLiteral("rebuilt")},
    {QStringLiteral("scene_id"), identity.scene_id},
    {QStringLiteral("output"), identity.request_owned_output_path}
  }).toJson(QJsonDocument::Compact);
  const QString key = widget.embedded_web_preparation_diagnostic_key(identity);
  widget.embedded_web_preparation_diagnostics_.insert(key, diagnostic);
  widget.embedded_web_preparation_process_keys_.insert(process, key);
  return process;
}

void select_identity(ScenePreviewWidget & widget, const ScenePreviewWidget::EmbeddedWebRequestIdentity & identity)
{
  widget.preview_scene_name_ = identity.scene_id;
  widget.preview_context_.scene_id = identity.scene_id;
  widget.preview_context_.absolute_scene_dir = identity.absolute_scene_dir;
  widget.preview_context_.absolute_repo_root = identity.absolute_repo_root;
  widget.embedded_web_active_identity_ = identity;
  widget.embedded_web_has_active_identity_ = true;
  widget.embedded_web_repo_root_ = identity.absolute_repo_root;
}
#endif
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
TEST(ScenePreviewWidgetUi, NormalPreparationIncludesAuthoringSessionOverlay)
{
  ASSERT_NE(ensure_app(), nullptr);
  const QString root = QDir(QStringLiteral(WORKCELL_BUILDER_REPO_ROOT)).absolutePath();
  const QString scene = QStringLiteral("ur5_2f_test");

  ScenePreviewWidget widget;
  widget.preview_items_ = {stale_editable_item(QStringLiteral("object_01"), 0.1, 0.45, 0.1)};
  const auto identity = publication_identity(root, scene, 4, 8, QByteArrayLiteral("normal-overlay"));
  select_identity(widget, identity);
  widget.start_embedded_web_prepare(
    identity, true, ScenePreviewWidget::EmbeddedWebSourcePolicy::AuthoringSession);

  ASSERT_NE(widget.embedded_web_prepare_process_, nullptr);
  const QStringList arguments = widget.embedded_web_prepare_process_->arguments();
  EXPECT_TRUE(arguments.contains(QStringLiteral("--authoring-session-overlay")));
  EXPECT_TRUE(arguments.contains(QStringLiteral("--force")));
  stop_preparation(widget);
}

TEST(ScenePreviewWidgetUi, PostSaveRefreshUsesPersistedCanonicalSourcesWithoutStaleOverlay)
{
  ASSERT_NE(ensure_app(), nullptr);
  const QString root = QDir(QStringLiteral(WORKCELL_BUILDER_REPO_ROOT)).absolutePath();
  const QString scene = QStringLiteral("ur5_2f_test");

  ScenePreviewWidget widget;
  stop_preparation(widget);
  widget.preview_scene_name_ = scene;
  widget.preview_context_.scene_id = scene;
  widget.preview_context_.absolute_scene_dir = QDir(root).filePath(QStringLiteral("scenes/%1").arg(scene));
  widget.preview_context_.absolute_repo_root = root;
  // These deliberately stale transforms represent A/B while disk contains the
  // just-saved A'/B'. Both instances share one catalog asset identity, so only
  // their scene-instance IDs may distinguish them.
  widget.preview_items_ = {
    stale_editable_item(QStringLiteral("object_01"), 0.1, 0.45, 0.1),
    stale_editable_item(QStringLiteral("object_02"), 1.1, -0.15, 0.1)};
  const int prior_revision = widget.preview_payload_revision_;
  const quint64 prior_generation = widget.preview_payload_generation_;

  const int requested_revision = widget.request_post_save_product_view_refresh();

  ASSERT_GT(requested_revision, 0);
  ASSERT_NE(widget.embedded_web_prepare_process_, nullptr);
  const QStringList arguments = widget.embedded_web_prepare_process_->arguments();
  EXPECT_FALSE(arguments.contains(QStringLiteral("--authoring-session-overlay")));
  EXPECT_TRUE(arguments.contains(QStringLiteral("--force")));
  EXPECT_EQ(widget.preview_payload_revision_, prior_revision + 1);
  EXPECT_EQ(widget.preview_payload_generation_, prior_generation + 1);
  EXPECT_EQ(widget.post_save_refresh_payload_revision_, requested_revision);
  EXPECT_EQ(widget.post_save_refresh_generation_, widget.embedded_web_request_generation_);
  EXPECT_EQ(widget.preview_items_.at(0).x, 0.1);
  EXPECT_EQ(widget.preview_items_.at(1).y, -0.15);
  EXPECT_EQ(widget.preview_items_.at(0).catalog_asset_id,
    widget.preview_items_.at(1).catalog_asset_id);
  stop_preparation(widget);
}

TEST(ScenePreviewWidgetUi, CurrentPreparationPublishesAtomicallyToCanonicalBrowserPath)
{
  ASSERT_NE(ensure_app(), nullptr);
  QTemporaryDir root;
  ASSERT_TRUE(root.isValid());
  const QString scene = QStringLiteral("publication_scene");
  ASSERT_TRUE(QDir().mkpath(QDir(root.path()).filePath(QStringLiteral("scenes/%1").arg(scene))));

  ScenePreviewWidget widget;
  const auto identity = publication_identity(root.path(), scene, 7, 11, QByteArrayLiteral("single"));
  select_identity(widget, identity);
  auto * process = completed_preparation(widget, identity, QStringLiteral("CURRENT_MARKER"));
  widget.embedded_web_prepare_process_ = process;
  widget.embedded_web_server_process_ = new QProcess(&widget);
  widget.embedded_web_server_process_->start(
    QStringLiteral("/bin/sh"), QStringList{QStringLiteral("-c"), QStringLiteral("sleep 30")});
  ASSERT_TRUE(widget.embedded_web_server_process_->waitForStarted());
  widget.embedded_web_server_is_owned_ = true;
  widget.embedded_web_server_session_repo_root_ = root.path();
  widget.embedded_web_server_session_port_ = identity.selected_server_port;
  widget.on_embedded_web_prepare_finished(identity, process, 0, QProcess::NormalExit);

  QFile canonical(QDir(root.path()).filePath(identity.generated_web_scene_path));
  ASSERT_TRUE(canonical.open(QIODevice::ReadOnly));
  EXPECT_TRUE(canonical.readAll().contains("CURRENT_MARKER"));
  EXPECT_EQ(widget.embedded_web_canonical_publications_, 1u);
  EXPECT_EQ(widget.embedded_web_prepared_identity_, identity);
  EXPECT_EQ(widget.embedded_web_loading_identity_, identity);
  EXPECT_EQ(QUrlQuery(widget.embedded_web_expected_viewer_url_).queryItemValue(QStringLiteral("scene")),
    identity.generated_web_scene_path);
  EXPECT_EQ(widget.embedded_web_preparation_diagnostics_.value(
    widget.embedded_web_preparation_diagnostic_key(identity)).terminal_outcome, QStringLiteral("success"));
}

TEST(ScenePreviewWidgetUi, LateSupersededCompletionCannotPublishOrNavigate)
{
  ASSERT_NE(ensure_app(), nullptr);
  QTemporaryDir root;
  ASSERT_TRUE(root.isValid());
  const QString scene = QStringLiteral("publication_scene");
  ASSERT_TRUE(QDir().mkpath(QDir(root.path()).filePath(QStringLiteral("scenes/%1").arg(scene))));

  ScenePreviewWidget widget;
  const auto identity_a = publication_identity(root.path(), scene, 21, 31, QByteArrayLiteral("fingerprint-a"));
  const auto identity_b = publication_identity(root.path(), scene, 22, 32, QByteArrayLiteral("fingerprint-b"));
  EXPECT_NE(identity_a.request_owned_output_path, identity_b.request_owned_output_path);

  select_identity(widget, identity_a);
  auto * process_a = completed_preparation(widget, identity_a, QStringLiteral("MARKER_A"));
  widget.embedded_web_prepare_process_ = process_a;  // A starts.

  select_identity(widget, identity_b);  // B supersedes A.
  auto * process_b = completed_preparation(widget, identity_b, QStringLiteral("MARKER_B"));
  widget.embedded_web_prepare_process_ = process_b;

  // Reuse a verified owned server so successful B publication immediately
  // reaches the canonical browser-loading lifecycle without network probes.
  widget.embedded_web_server_process_ = new QProcess(&widget);
  widget.embedded_web_server_process_->start(
    QStringLiteral("/bin/sh"), QStringList{QStringLiteral("-c"), QStringLiteral("sleep 30")});
  ASSERT_TRUE(widget.embedded_web_server_process_->waitForStarted());
  widget.embedded_web_server_is_owned_ = true;
  widget.embedded_web_server_session_repo_root_ = root.path();
  widget.embedded_web_server_session_port_ = identity_b.selected_server_port;

  widget.on_embedded_web_prepare_finished(identity_b, process_b, 0, QProcess::NormalExit);
  QApplication::processEvents();
  const auto prepared_b = widget.embedded_web_prepared_identity_;
  const auto loading_b = widget.embedded_web_loading_identity_;
  const auto readiness_state_b = widget.embedded_product_view_state_;
  const QUrl browser_url_b = widget.embedded_web_expected_viewer_url_;
  const quint64 publications_after_b = widget.embedded_web_canonical_publications_;
  const quint64 navigations_after_b = widget.embedded_web_browser_navigations_started_;

  widget.on_embedded_web_prepare_finished(identity_a, process_a, 0, QProcess::NormalExit);  // A completes late.

  QFile canonical(QDir(root.path()).filePath(identity_b.generated_web_scene_path));
  ASSERT_TRUE(canonical.open(QIODevice::ReadOnly));
  const QByteArray canonical_payload = canonical.readAll();
  EXPECT_TRUE(canonical_payload.contains("MARKER_B"));
  EXPECT_FALSE(canonical_payload.contains("MARKER_A"));
  EXPECT_EQ(widget.embedded_web_preparation_diagnostics_.value(
    widget.embedded_web_preparation_diagnostic_key(identity_a)).terminal_outcome, QStringLiteral("stale_discarded"));
  EXPECT_EQ(widget.embedded_web_prepared_identity_, prepared_b);
  EXPECT_EQ(widget.embedded_web_loading_identity_, loading_b);
  EXPECT_EQ(widget.embedded_web_prepared_identity_, identity_b);
  EXPECT_EQ(widget.embedded_web_loading_identity_, identity_b);
  EXPECT_EQ(widget.embedded_product_view_state_, readiness_state_b);
  EXPECT_EQ(widget.embedded_web_expected_viewer_url_, browser_url_b);
  EXPECT_EQ(widget.embedded_web_active_identity_, identity_b);
  EXPECT_EQ(widget.embedded_web_canonical_publications_, publications_after_b);
  EXPECT_EQ(publications_after_b, 1u);
  EXPECT_EQ(widget.embedded_web_browser_navigations_started_, navigations_after_b);
  EXPECT_EQ(navigations_after_b, 1u);
}

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


TEST(SceneBuilderWorkspaceSource, ImportedPlacementPropagatesCatalogIdentityAndAuthoredScale)
{
  QFile source(QStringLiteral("workcell_builder/workcell_builder/gui/mainwindow.cpp"));
  if (!source.exists()) source.setFileName(QStringLiteral("../workcell_builder/gui/mainwindow.cpp"));
  ASSERT_TRUE(source.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(source.readAll());

  EXPECT_TRUE(text.contains(QStringLiteral("ui_entry.scale = source_entry.scale")));
  EXPECT_TRUE(text.contains(QStringLiteral("armed_asset_scale_ = match->scale")));
  EXPECT_TRUE(text.contains(QStringLiteral("preview_item.catalog_asset_id = asset_id")));
  EXPECT_TRUE(text.contains(QStringLiteral("preview_item.mesh_scale_x = armed_asset_scale_")));
  EXPECT_TRUE(text.contains(QStringLiteral("preview_item.mesh_scale_y = armed_asset_scale_")));
  EXPECT_TRUE(text.contains(QStringLiteral("preview_item.mesh_scale_z = armed_asset_scale_")));
  EXPECT_FALSE(text.contains(QStringLiteral("preview_item.mesh_scale_x = 1.0; preview_item.mesh_scale_y = 1.0")));
}

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
  EXPECT_TRUE(text.contains(QStringLiteral("scene_splitter->setSizes({320, 900, 360})")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder/preferred_right_width")));
  EXPECT_TRUE(text.contains(QStringLiteral("Show Left Panel")));
  EXPECT_TRUE(text.contains(QStringLiteral("Show Right Panel")));
  EXPECT_TRUE(text.contains(QStringLiteral("Focus 3D View")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder/main_splitter_sizes")));
}

TEST(SceneBuilderWorkspaceSource, MinimapPresentationTracksProductViewBackendWithoutRemovingFallback)
{
  QFile source(QStringLiteral("workcell_builder/workcell_builder/gui/mainwindow.cpp"));
  if (!source.exists()) source.setFileName(QStringLiteral("../workcell_builder/gui/mainwindow.cpp"));
  ASSERT_TRUE(source.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(source.readAll());

  EXPECT_TRUE(text.contains(QStringLiteral("setObjectName(\"digital_twin_minimap\")")));
  EXPECT_TRUE(text.contains(QStringLiteral("void MainWindow::update_minimap_backend_presentation()")));
  EXPECT_TRUE(text.contains(QStringLiteral("ProductViewBackend::EmbeddedWeb3D")));
  EXPECT_TRUE(text.contains(QStringLiteral("embedded_web_product_view_presented()")));
  EXPECT_TRUE(text.contains(QStringLiteral("minimap_view_->setMinimumSize(0, 0)")));
  EXPECT_TRUE(text.contains(QStringLiteral("minimap_view_->setMaximumSize(0, 0)")));
  EXPECT_TRUE(text.contains(QStringLiteral("minimap_view_->setMinimumSize(150, 90)")));
  EXPECT_TRUE(text.contains(QStringLiteral("minimap_view_->setMaximumSize(150, 90)")));
  EXPECT_TRUE(text.contains(QStringLiteral("&ScenePreviewWidget::embedded_product_view_runtime_state_changed")));
  EXPECT_GE(text.count(QStringLiteral("update_minimap_backend_presentation();")), 3);

  // The hidden minimap is presentation-only. The authoritative Qt authoring
  // scene and its 2D fallback view must still be built and wired to Product View.
  EXPECT_TRUE(text.contains(QStringLiteral("digital_twin_canvas_ = new QGraphicsView(scene_builder)")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_preview_widget_->set_fallback_2d_view(digital_twin_canvas_)")));
  EXPECT_TRUE(text.contains(QStringLiteral("digital_twin_scene_ = new QGraphicsScene(digital_twin_canvas_)")));
}

TEST(SceneBuilderWorkspaceSource, FocusModeHidesPanelsWithoutReloadingProductView)
{
  QFile source(QStringLiteral("workcell_builder/workcell_builder/gui/mainwindow.cpp"));
  if (!source.exists()) source.setFileName(QStringLiteral("../workcell_builder/gui/mainwindow.cpp"));
  ASSERT_TRUE(source.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(source.readAll());

  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder_left_panel_->hide()")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder_right_panel_->hide()")));
  EXPECT_TRUE(text.contains(QStringLiteral("apply_scene_builder_panel_visibility(\n        scene_builder_focus_restore_left_visible_, scene_builder_focus_restore_right_visible_, false)")));
  const int focus_action_index = text.indexOf(QStringLiteral("scene_builder_focus_3d_action_"));
  ASSERT_GE(focus_action_index, 0);
  const QString focus_block = text.mid(focus_action_index, 2200);
  EXPECT_FALSE(focus_block.contains(QStringLiteral("new ScenePreviewWidget")));
  EXPECT_FALSE(focus_block.contains(QStringLiteral("request_embedded_web_product_view_refresh")));
  EXPECT_FALSE(focus_block.contains(QStringLiteral("reload_meshes")));
}

TEST(SceneBuilderWorkspaceSource, PanelLayoutV3RepairsAndRestoresInspector)
{
  QFile source(QStringLiteral("workcell_builder/workcell_builder/gui/mainwindow.cpp"));
  if (!source.exists()) source.setFileName(QStringLiteral("../workcell_builder/gui/mainwindow.cpp"));
  ASSERT_TRUE(source.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(source.readAll());

  // Fresh settings and old v2 {320, 1000, 0} both obtain a non-zero default.
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder_preferred_right_width_{360}")) ||
    text.contains(QStringLiteral("scene_builder_preferred_right_width_ = 360")));
  EXPECT_TRUE(text.contains(QStringLiteral("if (scene_builder_preferred_right_width_ <= 0) scene_builder_preferred_right_width_ = 360")));
  EXPECT_TRUE(text.contains(QStringLiteral("native_layout_version\"), 3")));
  EXPECT_TRUE(text.contains(QStringLiteral("right = qMin(qMax(240, right), right_available)")));

  // Hiding cannot overwrite a preferred width with the splitter's zero.
  EXPECT_TRUE(text.contains(QStringLiteral("!scene_builder_right_panel_->isHidden() && sizes[2] > 0")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder/preferred_left_width")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder/preferred_right_width")));

  // Menu toggles, selection auto-open, startup, and Focus 3D exit share one path.
  EXPECT_GE(text.count(QStringLiteral("apply_scene_builder_panel_visibility(")), 6);
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder_focus_restore_right_visible_, false")));
  EXPECT_TRUE(text.contains(QStringLiteral("scene_builder_left_panel_ && !scene_builder_left_panel_->isHidden(), true")));
  EXPECT_TRUE(text.contains(QStringLiteral("QSignalBlocker blocker(scene_builder_show_right_panel_action_)")));

  // Responsive rebuilding always puts the persistent View actions back.
  const int overflow = text.indexOf(QStringLiteral("void MainWindow::update_scene_builder_top_controls_overflow()"));
  ASSERT_GE(overflow, 0);
  const QString overflow_block = text.mid(overflow, 2600);
  EXPECT_TRUE(overflow_block.contains(QStringLiteral("addMenu(\"View\")")));
  EXPECT_TRUE(overflow_block.contains(QStringLiteral("addAction(scene_builder_show_left_panel_action_)")));
  EXPECT_TRUE(overflow_block.contains(QStringLiteral("addAction(scene_builder_show_right_panel_action_)")));
  EXPECT_TRUE(overflow_block.contains(QStringLiteral("addAction(scene_builder_focus_3d_action_)")));
  EXPECT_TRUE(text.contains(QStringLiteral("new QAction(\"Show Right Panel\", this)")));
  EXPECT_TRUE(text.contains(QStringLiteral("new QAction(\"Focus 3D View\", this)")));
  EXPECT_TRUE(text.contains(QStringLiteral("setText(\"Panels & Tools\")")));
}

TEST(SceneBuilderWorkspaceSource, CompactBottomStatusBarUsesSingleRowAndExistingLogDrawer)
{
  QFile source(QStringLiteral("workcell_builder/workcell_builder/gui/mainwindow.cpp"));
  if (!source.exists()) source.setFileName(QStringLiteral("../workcell_builder/gui/mainwindow.cpp"));
  ASSERT_TRUE(source.open(QIODevice::ReadOnly | QIODevice::Text));
  const QString text = QString::fromUtf8(source.readAll());

  const int bar_index = text.indexOf(QStringLiteral(
    "bottom_status_bar->setObjectName(\"sceneBuilderBottomStatusBar\")"));
  ASSERT_GE(bar_index, 0);
  const QString bar_block = text.mid(bar_index, 1900);
  EXPECT_EQ(text.count(QStringLiteral(
    "bottom_status_bar->setObjectName(\"sceneBuilderBottomStatusBar\")")), 1);
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
  EXPECT_TRUE(append_block.contains(QStringLiteral("studio_log_issue_tracker_.warning_count()")));
  EXPECT_TRUE(append_block.contains(QStringLiteral("studio_log_issue_tracker_.error_count()")));
  EXPECT_FALSE(append_block.contains(QStringLiteral("lowered.contains")));
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
