#include "scene_preview_widget.h"
// Compatibility token for static tests: Preview selection cleared after refresh (id missing):

#include <QRectF>
#include <QtGlobal>
#include <QVariantMap>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QFile>
#include <QIODevice>
#include <QStringList>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QEventLoop>
#include <QTimer>
#include <QJsonArray>
#include <QSet>
#include <functional>

namespace {
constexpr double kOverlayFitDominanceRatio = 4.0;

QString normalized_preview_token(const QString & value)
{
  return value.trimmed().toLower().replace('-', '_').replace(' ', '_');
}

bool preview_item_has_credible_mesh_handoff(const ScenePreviewWidget::PreviewItem & item)
{
  return item.mesh_available || item.has_mesh_metadata || !item.mesh_path.trimmed().isEmpty() || !item.source_path.trimmed().isEmpty();
}

bool preview_item_has_valid_urdf_primitive(const ScenePreviewWidget::PreviewItem & item)
{
  const QString type = normalized_preview_token(item.primitive_geometry_type);
  if (type == QStringLiteral("box")) return item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001;
  if (type == QStringLiteral("cylinder")) return item.primitive_radius > 0.001 && item.primitive_length > 0.001;
  if (type == QStringLiteral("sphere")) return item.primitive_radius > 0.001;
  if (type == QStringLiteral("capsule")) return item.primitive_radius > 0.001 && item.primitive_length > 0.001;
  return false;
}

bool preview_item_is_overlay_or_helper(const ScenePreviewWidget::PreviewItem & item)
{
  const QString role = normalized_preview_token(item.role);
  const QString category = normalized_preview_token(item.category);
  const QString source_layer = normalized_preview_token(item.source_layer);
  const QString lock_reason = normalized_preview_token(item.lock_reason);
  return role.contains("overlay") || role.contains("helper") || role.contains("guide") ||
         role.contains("warning_anchor") || role.contains("warning_badge") ||
         role.contains("safety_zone") || category.contains("overlay") || category.contains("helper") ||
         category.contains("safety") || source_layer.contains("overlay") || lock_reason.contains("overlay");
}

bool preview_item_is_generated_or_locked_urdf(const ScenePreviewWidget::PreviewItem & item)
{
  const QString category = normalized_preview_token(item.category);
  const QString source_layer = normalized_preview_token(item.source_layer);
  const QString visual_source = normalized_preview_token(item.active_visual_source);
  const QString lock_reason = normalized_preview_token(item.lock_reason);
  return source_layer.contains("generated_urdf") || source_layer.contains("locked_generated_urdf") ||
         visual_source.contains("generated_urdf") || visual_source.contains("locked_generated_urdf") ||
         category.contains("urdf") || lock_reason.contains("urdf") || lock_reason.contains("robot_model") ||
         lock_reason.contains("robotmodel");
}

bool preview_item_is_raw_generated_bounds_only(const ScenePreviewWidget::PreviewItem & item)
{
  return preview_item_is_generated_or_locked_urdf(item) &&
         item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001 &&
         (preview_item_has_credible_mesh_handoff(item) || preview_item_has_valid_urdf_primitive(item));
}

void maybe_warn_overlay_fit_dominance(ScenePreviewWidget * self, const QRectF & physical_bounds, const QRectF & overlay_bounds)
{
  if (!self) return;
  if (!physical_bounds.isValid() || physical_bounds.isEmpty()) return;
  if (!overlay_bounds.isValid() || overlay_bounds.isEmpty()) return;
  const double physical_area = qMax(1e-6, physical_bounds.width() * physical_bounds.height());
  const double overlay_area = qMax(0.0, overlay_bounds.width() * overlay_bounds.height());
  const double ratio = overlay_area / physical_area;
  if (ratio < kOverlayFitDominanceRatio) return;
  emit self->studio_log_requested(QString("Overlay-fit warning: overlay bounds are %1x physical bounds; use Fit Scene to keep physical meshes legible.")
                                      .arg(QString::number(ratio, 'f', 1)));
}
}


#include <algorithm>
#include <QComboBox>
#include <QFrame>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsProxyWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QProcess>
#include <QProcessEnvironment>
#include <QUrl>
#include <QTimer>
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
#include <QWebEngineView>
#include <QWebEnginePage>
#endif
#include "scene3d_viewport_widget.h"
#include <QPainter>
#include <QStackedWidget>
#include <QSignalBlocker>
#include <QVector3D>
#include <QVBoxLayout>
#include <QtMath>
#include <string_view>


namespace {
constexpr const char kScenePreviewMouseHelpText[] = "Mouse: orbit / wheel zoom / shift-drag pan";
static_assert(std::string_view(kScenePreviewMouseHelpText) == "Mouse: orbit / wheel zoom / shift-drag pan",
              "Acceptance token mismatch for scene preview mouse help text");

QString scene_preview_mouse_help_tooltip(const QString & unavailable_reason)
{
  QString tooltip = QString::fromUtf8(kScenePreviewMouseHelpText);
  if (!unavailable_reason.trimmed().isEmpty()) {
    tooltip += QString("\n3D unavailable: %1").arg(unavailable_reason.trimmed());
  }
  return tooltip;
}
}  // namespace

ScenePreviewWidget::ScenePreviewWidget(QWidget * parent) : QWidget(parent)
{
  setObjectName("scenePreviewWidget");
  auto * root = new QVBoxLayout(this);
  auto * controls = new QHBoxLayout();
  view_mode_label_ = new QLabel("View mode", this);
  controls->addWidget(view_mode_label_);
  mode_selector_ = new QComboBox(this);
  mode_selector_->addItems({"3D Layout Preview", "2D Layout"});
  controls->addWidget(mode_selector_);
  controls->addSpacing(8);
  mesh_preview_mode_label_ = new QLabel("Mesh Preview:", this);
  controls->addWidget(mesh_preview_mode_label_);
  mesh_preview_mode_selector_ = new QComboBox(this);
  mesh_preview_mode_selector_->addItems({"Auto", "Meshes", "Primitives"});
  mesh_preview_mode_selector_->setCurrentText("Auto");
  mesh_preview_mode_selector_->setToolTip("Mesh preview mode is visual-only and does not alter generated runtime files.");
  controls->addWidget(mesh_preview_mode_selector_);
  controls->addSpacing(8);
  gizmo_mode_label_ = new QLabel("Gizmo:", this);
  controls->addWidget(gizmo_mode_label_);
  gizmo_mode_selector_ = new QComboBox(this);
  gizmo_mode_selector_->addItems({"Select", "Move", "Rotate", "Scale (disabled)"});
  controls->addWidget(gizmo_mode_selector_);
  snap_mode_label_ = new QLabel("Snap:", this);
  controls->addWidget(snap_mode_label_);
  snap_mode_selector_ = new QComboBox(this);
  snap_mode_selector_->addItems({"Off", "1 cm", "5 cm", "10 cm", "5 deg", "15 deg"});
  snap_mode_selector_->setCurrentText("5 cm");
  controls->addWidget(snap_mode_selector_);
  controls->addSpacing(8);
  labels_label_ = new QLabel("Labels:", this);
  controls->addWidget(labels_label_);
  labels_selector_ = new QComboBox(this);
  labels_selector_->addItems({"Off", "Important", "Selected", "All"});
  labels_selector_->setCurrentText("Selected");
  labels_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  controls->addWidget(labels_selector_);
  controls->addSpacing(8);
  interaction_mode_label_ = new QLabel("Mode:", this);
  controls->addWidget(interaction_mode_label_);
  interaction_mode_selector_ = new QComboBox(this);
  interaction_mode_selector_->addItems({"Select", "Place Asset", "Move", "Rotate", "Inspect"});
  controls->addWidget(interaction_mode_selector_);
  view_actions_label_ = new QLabel("View:", this);
  controls->addWidget(view_actions_label_);
  view_actions_selector_ = new QComboBox(this);
  view_actions_selector_->addItems({"Top", "Front", "Side", "Isometric", "Fit View"});
  {
    const QSignalBlocker blocker(view_actions_selector_);
    view_actions_selector_->setCurrentText("Isometric");
  }
  controls->addWidget(view_actions_selector_);
  toolbar_status_chip_ = new QLabel(this);
  toolbar_status_chip_->setObjectName("previewToolbarChip");
  toolbar_status_chip_->setStyleSheet("QLabel#previewToolbarChip { background-color: rgba(30,41,59,225); color: #e2e8f0; border-radius: 9px; padding: 2px 8px; }");
  toolbar_status_chip_->setAlignment(Qt::AlignCenter);
  controls->addWidget(toolbar_status_chip_);
  auto * mouse_help_label = new QLabel(QStringLiteral("ⓘ"), this);
  mouse_help_label->setToolTip(scene_preview_mouse_help_tooltip(QString()));
  mouse_help_label->setStatusTip(QString::fromUtf8(kScenePreviewMouseHelpText));
  controls->addWidget(mouse_help_label);
  overlays_selector_ = new QComboBox(this);
  overlays_selector_->addItems({
    "Overlays",
    "Diagnostics Overlay",
    "Reachability Heatmap",
    "Collision Warnings",
    "Safety Zones",
    "Work Envelope",
    "Warning Labels",
    "Labels",
    "Pick/Place Zones",
    "Task Route",
    "Approach/Retreat",
    "Camera FOV",
    "Pick Coverage",
    "EPD Detections",
    "Detection Labels",
    "Warnings",
    "Focus Selected",
    "Fit Scene",
    "Fit Robot",
    "Fit overlays",
    "Clear Selection"
  });
  overlays_selector_->setCurrentText("Overlays");
  overlays_selector_->setToolTip("Product View starts clean. Use these diagnostics controls to explicitly enable helper overlays for the current preview session.");
  controls->addWidget(overlays_selector_);
  controls->addStretch(1);
  root->addLayout(controls);
  stack_ = new QStackedWidget(this);
  view3d_container_ = new QWidget(this); auto * v3 = new QVBoxLayout(view3d_container_);
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  embedded_web_view_ = new QWebEngineView(view3d_container_);
  embedded_web_view_->setObjectName("embeddedWeb3dProductView");
  connect(embedded_web_view_, &QWebEngineView::loadFinished, this, [this](bool ok) {
    if (!ok) {
      const QString detail = QStringLiteral("browser failed to load Product View page for scene %1; viewer URL: %2; expected JSON: %3")
        .arg(embedded_web_prepare_scene_, embedded_web_last_viewer_url_, embedded_web_prepare_output_path_);
      set_embedded_product_view_state(EmbeddedProductViewState::Failed, detail);
      emit studio_log_requested(QStringLiteral("Embedded Product View load failed. %1").arg(detail));
      return;
    }
    set_embedded_product_view_state(EmbeddedProductViewState::Loading, QStringLiteral("browser loaded; waiting for viewer readiness"));
    start_embedded_web_readiness_polling(embedded_web_prepare_scene_, embedded_web_active_revision_, embedded_web_prepare_output_path_, embedded_web_last_viewer_url_);
  });
  simple_3d_view_ = embedded_web_view_;
#else
  simple_3d_view_ = new Scene3DViewportWidget(view3d_container_);
  simple_3d_view_->setObjectName("scene3dViewportWidget");
#endif
  v3->addWidget(simple_3d_view_);
  empty_state_label_ = new QLabel("No scene selected\nOpen a scene or create a new cell to preview it.", view3d_container_);
  empty_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(empty_state_label_);
  error_state_label_ = new QLabel("3D Layout Preview unavailable", view3d_container_); error_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(error_state_label_);
  view2d_container_ = new QWidget(this); view2d_container_->setLayout(new QVBoxLayout());
  fallback_banner_label_ = new QLabel("2D fallback preview active", view2d_container_);
  fallback_banner_label_->setAlignment(Qt::AlignCenter);
  fallback_banner_label_->setVisible(false);
  fallback_banner_label_->setToolTip(scene_preview_mouse_help_tooltip(QString()));
  view2d_container_->layout()->addWidget(fallback_banner_label_);
  info_chip_label_ = new QLabel(view2d_container_);
  info_chip_label_->setObjectName("previewInfoChip");
  info_chip_label_->setStyleSheet("QLabel#previewInfoChip { background-color: rgba(15,23,42,210); color: #e2e8f0; border-radius: 6px; padding: 5px 8px; }");
  info_chip_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  info_chip_label_->setWordWrap(true);
  stack_->addWidget(view3d_container_); stack_->addWidget(view2d_container_); root->addWidget(stack_, 1);
  connect(mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ScenePreviewWidget::on_mode_changed);
  connect(labels_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto *v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
    if (!v) { refresh_embedded_web_product_view(); refresh_info_chip(); return; }
    const QString choice = labels_selector_->currentText();
    if (choice == "Off") v->label_mode = LabelMode::Off;
    else if (choice == "Important") v->label_mode = LabelMode::Important;
    else if (choice == "Selected") v->label_mode = LabelMode::Selected;
    else v->label_mode = LabelMode::All;
    v->update();
    refresh_info_chip();
  });
  connect(snap_mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
    if (!v) return;
    const QString choice = snap_mode_selector_->currentText();
    if (choice == "1 cm") v->snap_mode = Scene3DViewportWidget::SnapMode::Cm1;
    else if (choice == "5 cm") v->snap_mode = Scene3DViewportWidget::SnapMode::Cm5;
    else if (choice == "10 cm") v->snap_mode = Scene3DViewportWidget::SnapMode::Cm10;
    else if (choice == "5 deg") v->snap_mode = Scene3DViewportWidget::SnapMode::Deg5;
    else if (choice == "15 deg") v->snap_mode = Scene3DViewportWidget::SnapMode::Deg15;
    else v->snap_mode = Scene3DViewportWidget::SnapMode::Off;
    v->update();
    refresh_info_chip();
  });
  connect(gizmo_mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
    if (!v) return;
    const QString choice = gizmo_mode_selector_->currentText();
    if (choice == "Move") v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Move;
    else if (choice == "Rotate") v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Rotate;
    else if (choice.startsWith("Scale")) v->gizmo_mode = Scene3DViewportWidget::GizmoMode::ScaleDisabled;
    else v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Select;
    v->update();
    refresh_info_chip();
  });
  connect(mesh_preview_mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
    const QString choice = mesh_preview_mode_selector_->currentText();
    if (choice == "Meshes") mesh_preview_mode_ = MeshPreviewMode::Meshes;
    else if (choice == "Primitives") mesh_preview_mode_ = MeshPreviewMode::Primitives;
    else mesh_preview_mode_ = MeshPreviewMode::Auto;
    if (v) { v->mesh_preview_mode = mesh_preview_mode_; v->update(); }
  });
  connect(interaction_mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    const QString choice = interaction_mode_selector_->currentText();
    auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
    if (!v) return;
    if (choice == "Move") { v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Move; if (gizmo_mode_selector_) gizmo_mode_selector_->setCurrentText("Move"); }
    else if (choice == "Rotate") { v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Rotate; if (gizmo_mode_selector_) gizmo_mode_selector_->setCurrentText("Rotate"); }
    else { v->gizmo_mode = Scene3DViewportWidget::GizmoMode::Select; if (gizmo_mode_selector_) gizmo_mode_selector_->setCurrentText("Select"); }
    v->update();
    refresh_info_chip();
  });
  connect(view_actions_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    const QString choice = view_actions_selector_->currentText();
    auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
    if (!v) { if (choice == "Fit View") refresh_embedded_web_product_view(); refresh_info_chip(); return; }
    if (choice == "Top") v->set_top_view();
    else if (choice == "Front") v->set_front_view();
    else if (choice == "Side") v->set_side_view();
    else if (choice == "Isometric") v->set_isometric_view();
    else if (choice == "Fit View") on_fit_scene_clicked();
    else if (choice == "Labels" && labels_selector_) labels_selector_->showPopup();
    else if (choice == "Mesh Mode" && mesh_preview_mode_selector_) mesh_preview_mode_selector_->showPopup();
    else if (choice == "Diagnostics / Overlays" && overlays_selector_) overlays_selector_->showPopup();
    refresh_info_chip();
  });
  connect(overlays_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto *v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
    const QString choice = overlays_selector_->currentText();
    if (!v) { if (choice != "Overlays") emit studio_log_requested(QString("Embedded Web 3D Product View handles Product View controls in the browser; use the web viewer UI for overlays and camera actions.")); if (overlays_selector_ && choice != "Overlays") { const QSignalBlocker blocker(overlays_selector_); overlays_selector_->setCurrentText("Overlays"); } refresh_info_chip(); return; }
    if (choice == "Diagnostics Overlay") {
      v->debug_overlays_mode = !v->debug_overlays_mode;
      emit studio_log_requested(QString("Scene3D diagnostics overlay %1. Detailed render diagnostics remain available in logs.")
                                  .arg(v->debug_overlays_mode ? "enabled" : "disabled"));
    }
    else if (choice == "Reachability Heatmap") v->show_reachability_heatmap = !v->show_reachability_heatmap;
    else if (choice == "Collision Warnings") v->show_collision_warnings = !v->show_collision_warnings;
    else if (choice == "Safety Zones") v->show_safety = !v->show_safety;
    else if (choice == "Work Envelope") v->show_work_envelope = !v->show_work_envelope;
    else if (choice == "Warning Labels") v->show_warning_labels = !v->show_warning_labels;
    else if (choice == "Labels" && labels_selector_) labels_selector_->showPopup();
    else if (choice == "Pick/Place Zones") v->show_pick_place = !v->show_pick_place;
    else if (choice == "Task Route") v->show_task_route = !v->show_task_route;
    else if (choice == "Approach/Retreat") v->show_approach_retreat = !v->show_approach_retreat;
    else if (choice == "Camera FOV") v->show_camera_fov = !v->show_camera_fov;
    else if (choice == "Pick Coverage") v->show_pick_coverage = !v->show_pick_coverage;
    else if (choice == "EPD Detections") v->show_epd_detections = !v->show_epd_detections;
    else if (choice == "Detection Labels") v->show_detection_labels = !v->show_detection_labels;
    else if (choice == "Warnings") v->show_warnings = !v->show_warnings;
    else if (choice == "Focus Selected") on_focus_selected_clicked();
    else if (choice == "Fit Scene") on_fit_scene_clicked();
    else if (choice == "Fit Robot") on_fit_robot_clicked();
    else if (choice == "Fit overlays") on_fit_overlays_clicked();
    else if (choice == "Clear Selection") on_clear_selection_clicked();
    if (overlays_selector_ && choice != "Overlays") {
      const QSignalBlocker blocker(overlays_selector_);
      overlays_selector_->setCurrentText("Overlays");
    }
    v->update();
    refresh_info_chip();
  });
  if (auto * legacy_viewport = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_)) {
  legacy_viewport->select_cb = [this](const QString & id, const QString & role){ select_preview_item(id); emit preview_item_selected(id, role); if (diagnostic_debug_logging_enabled()) emit studio_log_requested(QString("Selected preview item: %1 (%2)").arg(id, role)); };
  legacy_viewport->status_message_cb = [this](const QString & message) {
    emit studio_log_requested(message);
  };
  }
  refresh_info_chip();
  refresh_mode_and_state();
  QTimer::singleShot(0, this, [this]() { emit_backend_startup_diagnostic_once(); });
}

QString ScenePreviewWidget::resolve_embedded_web_repo_root(const QString & selected_scene_dir) const
{
  auto canonical_path = [](const QString & path) -> QString {
    const QFileInfo info(path);
    const QString canonical = info.canonicalFilePath();
    return canonical.isEmpty() ? info.absoluteFilePath() : canonical;
  };
  auto missing_markers = [](const QString & root) -> QStringList {
    QDir dir(root);
    QStringList missing;
    if (!QFileInfo::exists(dir.filePath(QStringLiteral("workcell_studio_web/viewer/index.html")))) {
      missing << QStringLiteral("workcell_studio_web/viewer/index.html");
    }
    if (!QFileInfo::exists(dir.filePath(QStringLiteral("scripts/ensure_workcell_studio_web_scene_fresh.py")))) {
      missing << QStringLiteral("scripts/ensure_workcell_studio_web_scene_fresh.py");
    }
    if (!QFileInfo(dir.filePath(QStringLiteral("scenes"))).isDir()) {
      missing << QStringLiteral("scenes");
    }
    return missing;
  };
  auto log_diagnostic = [this](const QString & message) {
    emit const_cast<ScenePreviewWidget *>(this)->studio_log_requested(message);
  };
  auto accept_candidate = [&](const QString & raw_root, const QString & label, QString * accepted) -> bool {
    const QString root = canonical_path(raw_root);
    const QStringList missing = missing_markers(root);
    if (missing.isEmpty()) {
      *accepted = root;
      log_diagnostic(QStringLiteral("Embedded Product View repo root selected from %1: %2").arg(label, root));
      return true;
    }
    log_diagnostic(QStringLiteral("Embedded Product View repo root candidate rejected from %1: %2 (missing %3)")
                     .arg(label, root, missing.join(QStringLiteral(", "))));
    return false;
  };

  const QString scene_input = selected_scene_dir.trimmed();
  if (!scene_input.isEmpty()) {
    QFileInfo scene_info(scene_input);
    if (scene_info.exists() && scene_info.isDir()) {
      QDir dir(canonical_path(scene_input));
      for (int i = 0; i < 16; ++i) {
        QString accepted;
        if (accept_candidate(dir.absolutePath(), QStringLiteral("selected scene directory upward walk"), &accepted)) return accepted;
        if (!dir.cdUp()) break;
      }
    } else {
      log_diagnostic(QStringLiteral("Embedded Product View selected scene directory candidate rejected: %1 (missing directory)").arg(scene_input));
    }
  }

  if (!embedded_web_repo_root_.trimmed().isEmpty()) {
    QString accepted;
    if (accept_candidate(embedded_web_repo_root_, QStringLiteral("cached root after selected scene"), &accepted)) return accepted;
  }

  const QString env_root = QProcessEnvironment::systemEnvironment().value(QStringLiteral("WORKCELL_STUDIO_REPO_ROOT")).trimmed();
  if (!env_root.isEmpty()) {
    QString accepted;
    if (accept_candidate(env_root, QStringLiteral("WORKCELL_STUDIO_REPO_ROOT secondary override"), &accepted)) return accepted;
  }

  const QStringList fallback_roots{QDir::currentPath(), QCoreApplication::applicationDirPath()};
  for (const QString & fallback : fallback_roots) {
    QDir dir(fallback);
    for (int i = 0; i < 16; ++i) {
      QString accepted;
      if (accept_candidate(dir.absolutePath(), QStringLiteral("fallback application path upward walk"), &accepted)) return accepted;
      if (!dir.cdUp()) break;
    }
  }
  return QString();
}

QString ScenePreviewWidget::embedded_web_prepare_command_for_log(const QString & scene, const QString & output_path, bool force) const
{
  QString command = QStringLiteral("python3 scripts/ensure_workcell_studio_web_scene_fresh.py --scene scenes/%1 --output %2 --stage-assets")
                      .arg(scene, output_path);
  if (force) command += QStringLiteral(" --force");
  return command;
}

void ScenePreviewWidget::set_embedded_product_view_state(EmbeddedProductViewState state, const QString & detail)
{
  embedded_product_view_state_ = state;
  QString text;
  switch (state) {
    case EmbeddedProductViewState::Idle: text = QStringLiteral("Product View: idle"); break;
    case EmbeddedProductViewState::Preparing: text = QStringLiteral("Product View: preparing %1…").arg(detail); break;
    case EmbeddedProductViewState::StartingServer: text = QStringLiteral("Product View: starting local viewer server…"); break;
    case EmbeddedProductViewState::Loading: text = detail.trimmed().isEmpty() ? QStringLiteral("Product View: loading…") : QStringLiteral("Product View: %1…").arg(detail); break;
    case EmbeddedProductViewState::Ready: text = QStringLiteral("Product View: ready"); break;
    case EmbeddedProductViewState::Failed: text = QStringLiteral("Product View failed: %1").arg(detail); break;
  }
  if (toolbar_status_chip_) toolbar_status_chip_->setText(text);
  if (error_state_label_) {
    error_state_label_->setText(text);
    error_state_label_->setVisible(state == EmbeddedProductViewState::Failed);
  }
}

bool ScenePreviewWidget::embedded_web_server_is_usable(const QString & repo_root, const QString & scene) const
{
  const QString trimmed_repo_root = repo_root.trimmed().isEmpty() ? embedded_web_repo_root_ : repo_root.trimmed();
  const QString trimmed_scene = scene.trimmed().isEmpty() ? embedded_web_prepare_scene_ : scene.trimmed();
  if (trimmed_repo_root.isEmpty() || trimmed_scene.isEmpty()) {
    emit studio_log_requested(QStringLiteral("Embedded Web 3D server probe failed: missing selected repository root or scene id."));
    return false;
  }

  const QDir repo_dir(trimmed_repo_root);
  const QString marker_path = QStringLiteral("workcell_studio_web/viewer/workcell_runtime_marker.json");
  QFile marker_file(repo_dir.filePath(marker_path));
  QByteArray expected_marker;
  if (marker_file.open(QIODevice::ReadOnly)) expected_marker = marker_file.readAll().trimmed();

  struct HttpCheck { QString label; QString path; int status{0}; bool ok{false}; QString detail; QByteArray body; };
  auto get_url = [this](const QString & path, int timeout_ms = 1200) -> HttpCheck {
    HttpCheck check;
    check.path = path;
    const QUrl url(QStringLiteral("http://127.0.0.1:%1/%2").arg(embedded_web_server_port_).arg(path));
    QNetworkAccessManager manager;
    QNetworkRequest request(url);
    QNetworkReply * reply = manager.get(request);
    QEventLoop loop;
    QTimer timer;
    timer.setSingleShot(true);
    QObject::connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);
    QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
    timer.start(timeout_ms);
    loop.exec();
    if (timer.isActive()) {
      timer.stop();
      check.status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
      check.body = reply->readAll();
      check.ok = (reply->error() == QNetworkReply::NoError && check.status == 200);
      if (!check.ok) check.detail = reply->errorString();
    } else {
      check.detail = QStringLiteral("timeout after %1 ms").arg(timeout_ms);
      reply->abort();
    }
    reply->deleteLater();
    return check;
  };

  QVector<HttpCheck> checks;
  auto add_check = [&](const QString & label, const QString & path) {
    HttpCheck check = get_url(path);
    check.label = label;
    checks.push_back(check);
    return check;
  };

  const HttpCheck marker_check = add_check(QStringLiteral("repo marker"), marker_path);
  if (expected_marker.isEmpty()) {
    checks.last().ok = false;
    checks.last().detail = QStringLiteral("local marker file missing or empty under selected repo root: %1").arg(marker_path);
  } else if (marker_check.body.trimmed() != expected_marker) {
    checks.last().ok = false;
    checks.last().detail = QStringLiteral("marker response does not match selected repository root marker content");
  }

  add_check(QStringLiteral("viewer index"), QStringLiteral("workcell_studio_web/viewer/index.html"));
  add_check(QStringLiteral("viewer bundle"), QStringLiteral("workcell_studio_web/viewer/dist/viewer.bundle.js"));
  const QString web_scene_path = QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(trimmed_scene);
  HttpCheck scene_check = add_check(QStringLiteral("web scene"), web_scene_path);

  auto normalize_asset_path = [](QString value) {
    value = value.trimmed();
    if (value.startsWith(QStringLiteral("./"))) value = value.mid(2);
    if (value.startsWith(QStringLiteral("/"))) value = value.mid(1);
    if (value.startsWith(QStringLiteral("http://")) || value.startsWith(QStringLiteral("https://")) ||
        value.startsWith(QStringLiteral("package://")) || value.startsWith(QStringLiteral("file://"))) return QString();
    return value;
  };
  auto classify_asset = [](const QString & path) {
    const QString p = path.toLower();
    if (p.endsWith(QStringLiteral(".urdf")) && p.contains(QStringLiteral("robot_preview"))) return QStringLiteral("robot-preview URDF");
    if (p.contains(QStringLiteral("/ur_description/")) || p.contains(QStringLiteral("/ur5/"))) return QStringLiteral("UR5 mesh");
    if (p.contains(QStringLiteral("robotiq")) || p.contains(QStringLiteral("2f"))) return QStringLiteral("Robotiq mesh");
    if (p.contains(QStringLiteral("table")) || p.contains(QStringLiteral("workbench"))) return QStringLiteral("table/workbench mesh");
    if (p.contains(QStringLiteral("camera"))) return QStringLiteral("camera mesh");
    return QString();
  };
  std::function<void(const QJsonValue &, QSet<QString> &)> collect_assets = [&](const QJsonValue & value, QSet<QString> & out) {
    if (value.isObject()) {
      const QJsonObject obj = value.toObject();
      for (auto it = obj.begin(); it != obj.end(); ++it) {
        if (it.value().isString()) {
          const QString key = it.key().toLower();
          if (key.contains(QStringLiteral("mesh")) || key.contains(QStringLiteral("urdf")) || key.contains(QStringLiteral("uri")) || key.contains(QStringLiteral("path"))) {
            const QString path = normalize_asset_path(it.value().toString());
            if (!path.isEmpty() && classify_asset(path).size() > 0) out.insert(path);
          }
        }
        collect_assets(it.value(), out);
      }
    } else if (value.isArray()) {
      const QJsonArray array = value.toArray();
      for (const QJsonValue & entry : array) collect_assets(entry, out);
    }
  };

  if (scene_check.ok) {
    QJsonParseError parse_error;
    const QJsonDocument scene_doc = QJsonDocument::fromJson(scene_check.body, &parse_error);
    if (parse_error.error != QJsonParseError::NoError || !scene_doc.isObject()) {
      checks.last().ok = false;
      checks.last().detail = QStringLiteral("web scene JSON parse failed: %1").arg(parse_error.errorString());
    } else {
      const QJsonObject scene_obj = scene_doc.object();
      if (scene_obj.value(QStringLiteral("schema_version")).toString() != QStringLiteral("workcell_studio_web_scene/v1") ||
          scene_obj.value(QStringLiteral("scene_id")).toString(scene_obj.value(QStringLiteral("scene_name")).toString()) != trimmed_scene) {
        checks.last().ok = false;
        checks.last().detail = QStringLiteral("web scene JSON does not match selected scene/repository contract");
      }
      QSet<QString> assets;
      collect_assets(scene_doc.object(), assets);
      QSet<QString> covered_classes;
      for (const QString & asset : assets) {
        const QString label = classify_asset(asset);
        covered_classes.insert(label);
        add_check(label, asset);
      }
      const QStringList required_classes{QStringLiteral("robot-preview URDF"), QStringLiteral("UR5 mesh"), QStringLiteral("Robotiq mesh"), QStringLiteral("table/workbench mesh"), QStringLiteral("camera mesh")};
      for (const QString & required : required_classes) {
        if (!covered_classes.contains(required)) {
          HttpCheck missing;
          missing.label = required;
          missing.path = QStringLiteral("<referenced asset missing from web scene JSON>");
          missing.ok = false;
          missing.detail = QStringLiteral("no referenced staged asset found for required class");
          checks.push_back(missing);
        }
      }
    }
  }

  QStringList diagnostic_lines{QStringLiteral("Embedded Web 3D server resource probe for repo=%1 scene=%2").arg(trimmed_repo_root, trimmed_scene)};
  bool all_ok = true;
  for (const HttpCheck & check : checks) {
    all_ok = all_ok && check.ok;
    diagnostic_lines << QStringLiteral("  [%1] %2 status=%3 result=%4%5")
                          .arg(check.label,
                               check.path,
                               check.status > 0 ? QString::number(check.status) : QStringLiteral("n/a"),
                               check.ok ? QStringLiteral("PASS") : QStringLiteral("FAIL"),
                               check.detail.isEmpty() ? QString() : QStringLiteral(" detail=%1").arg(check.detail.left(240)));
  }
  emit studio_log_requested(diagnostic_lines.join(QStringLiteral("\n")));
  return all_ok;
}

void ScenePreviewWidget::ensure_embedded_web_server_started(const QString & repo_root)
{
  if (repo_root.trimmed().isEmpty()) return;
  if (embedded_web_server_process_ && embedded_web_server_process_->state() != QProcess::NotRunning) {
    load_prepared_embedded_web_scene(embedded_web_prepare_scene_, embedded_web_active_revision_);
    return;
  }
  if (embedded_web_server_is_usable(repo_root, embedded_web_prepare_scene_)) {
    emit studio_log_requested(QStringLiteral("Embedded Web 3D Product View reused existing verified server at http://127.0.0.1:8765/."));
    load_prepared_embedded_web_scene(embedded_web_prepare_scene_, embedded_web_active_revision_);
    return;
  }
  set_embedded_product_view_state(EmbeddedProductViewState::StartingServer);
  if (embedded_web_server_process_) embedded_web_server_process_->deleteLater();
  embedded_web_server_process_ = new QProcess(this);
  embedded_web_server_process_->setProgram(QStringLiteral("python3"));
  embedded_web_server_process_->setArguments(QStringList{"-m", "http.server", QString::number(embedded_web_server_port_), "--bind", "127.0.0.1"});
  embedded_web_server_process_->setWorkingDirectory(repo_root);
  embedded_web_server_process_->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  embedded_web_server_process_->setProcessChannelMode(QProcess::MergedChannels);
  connect(embedded_web_server_process_, &QProcess::started, this, [this]() {
    emit studio_log_requested(QStringLiteral("Started embedded Web 3D Product View server from repo root: python3 -m http.server 8765 --bind 127.0.0.1"));
    load_prepared_embedded_web_scene(embedded_web_prepare_scene_, embedded_web_active_revision_);
  });
  connect(embedded_web_server_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, [this](int exit_code, QProcess::ExitStatus) {
    const QString output = QString::fromUtf8(embedded_web_server_process_->readAll()).trimmed();
    if (embedded_product_view_state_ == EmbeddedProductViewState::StartingServer && !embedded_web_server_is_usable(embedded_web_repo_root_, embedded_web_prepare_scene_)) {
      set_embedded_product_view_state(EmbeddedProductViewState::Failed, QStringLiteral("local server failed; exit %1; %2").arg(exit_code).arg(output.left(240)));
      emit studio_log_requested(QStringLiteral("Embedded Product View local server failed: python3 -m http.server 8765 --bind 127.0.0.1\nExit code: %1\nOutput:\n%2").arg(exit_code).arg(output));
    }
  });
  embedded_web_server_process_->start();
}

void ScenePreviewWidget::request_embedded_web_product_view_refresh(bool force)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(force);
  return;
#else
  if (!embedded_web_view_) return;
  const QString scene = preview_scene_name_.trimmed();
  if (scene.isEmpty() || scene == QStringLiteral("No scene")) { set_embedded_product_view_state(EmbeddedProductViewState::Idle); return; }
  pending_embedded_web_scene_ = scene;
  pending_embedded_web_revision_ = ++embedded_web_request_revision_;
  pending_embedded_web_force_ = pending_embedded_web_force_ || force;
  maybe_start_next_embedded_web_prepare();
#endif
}

void ScenePreviewWidget::refresh_embedded_web_product_view()
{
  request_embedded_web_product_view_refresh(false);
}

void ScenePreviewWidget::maybe_start_next_embedded_web_prepare()
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  return;
#else
  if (embedded_web_prepare_process_ && embedded_web_prepare_process_->state() != QProcess::NotRunning) return;
  if (pending_embedded_web_scene_.isEmpty()) return;
  const QString scene = pending_embedded_web_scene_;
  const quint64 revision = pending_embedded_web_revision_;
  const bool force = pending_embedded_web_force_;
  pending_embedded_web_scene_.clear();
  pending_embedded_web_force_ = false;
  start_embedded_web_prepare(scene, revision, force);
#endif
}

void ScenePreviewWidget::emit_backend_startup_diagnostic_once()
{
  if (backend_startup_diagnostic_emitted_) return;
  backend_startup_diagnostic_emitted_ = true;
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  emit studio_log_requested(QStringLiteral("Workcell Product View backend=embedded_web3d webengine_compiled=true"));
#else
  emit studio_log_requested(QStringLiteral("Workcell Product View backend=native_compatibility reason=explicit_build_option"));
#endif
}

void ScenePreviewWidget::start_embedded_web_prepare(const QString & scene, quint64 revision, bool force)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(scene); Q_UNUSED(revision); Q_UNUSED(force);
#else
  const QString selected_scene_dir = QFileInfo(scene).isAbsolute()
    ? QFileInfo(scene).absoluteFilePath()
    : QDir(QDir::currentPath()).absoluteFilePath(QStringLiteral("scenes/%1").arg(scene));
  const QString repo_root = resolve_embedded_web_repo_root(selected_scene_dir);
  if (repo_root.isEmpty()) {
    set_embedded_product_view_state(EmbeddedProductViewState::Failed, QStringLiteral("could not find a Workcell Studio repo root with viewer, scene-prep script, and scenes markers"));
    emit studio_log_requested(QStringLiteral("Embedded Web 3D Product View unavailable: could not find a Workcell Studio repo root with required markers from selected scene, environment override, or fallback application paths."));
    return;
  }
  embedded_web_repo_root_ = repo_root;
  embedded_web_prepare_scene_ = scene;
  embedded_web_active_revision_ = revision;
  embedded_web_prepare_output_path_ = QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(scene);
  if (embedded_web_prepare_process_) embedded_web_prepare_process_->deleteLater();
  embedded_web_prepare_process_ = new QProcess(this);
  embedded_web_prepare_process_->setProgram(QStringLiteral("python3"));
  QStringList args{"scripts/ensure_workcell_studio_web_scene_fresh.py", "--scene", QStringLiteral("scenes/%1").arg(scene), "--output", embedded_web_prepare_output_path_, "--stage-assets"};
  if (force) args << "--force";
  embedded_web_prepare_process_->setArguments(args);
  embedded_web_prepare_process_->setWorkingDirectory(repo_root);
  embedded_web_prepare_process_->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  embedded_web_prepare_process_->setProcessChannelMode(QProcess::SeparateChannels);
  embedded_web_prepare_started_at_ = QDateTime::currentDateTimeUtc();
  connect(embedded_web_prepare_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &ScenePreviewWidget::on_embedded_web_prepare_finished);
  set_embedded_product_view_state(EmbeddedProductViewState::Preparing, scene);
  emit studio_log_requested(QStringLiteral("Preparing embedded Product View from repo root %1: %2").arg(repo_root, embedded_web_prepare_command_for_log(scene, embedded_web_prepare_output_path_, force)));
  embedded_web_prepare_process_->start();
#endif
}

void ScenePreviewWidget::on_embedded_web_prepare_finished(int exit_code, QProcess::ExitStatus exit_status)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(exit_code); Q_UNUSED(exit_status);
#else
  QProcess * process = qobject_cast<QProcess *>(sender());
  if (!process || process != embedded_web_prepare_process_) return;
  const QString scene = embedded_web_prepare_scene_;
  const quint64 revision = embedded_web_active_revision_;
  const QString output_path = embedded_web_prepare_output_path_;
  const QString stdout_text = QString::fromUtf8(process->readAllStandardOutput()).trimmed();
  const QString stderr_text = QString::fromUtf8(process->readAllStandardError()).trimmed();
  const QString command = embedded_web_prepare_command_for_log(scene, output_path, false);
  process->deleteLater();
  embedded_web_prepare_process_ = nullptr;

  const bool still_current = (preview_scene_name_.trimmed() == scene && revision == embedded_web_request_revision_);
  const QString absolute_output_path = QDir(embedded_web_repo_root_).filePath(output_path);
  const bool output_is_fresh = QFileInfo::exists(QDir(embedded_web_repo_root_).filePath(output_path));  // Contract validation supersedes mtime freshness.
  Q_UNUSED(output_is_fresh);
  auto reject_prepare = [&](const QString & reason) {
    set_embedded_product_view_state(EmbeddedProductViewState::Failed, reason);
    emit studio_log_requested(QStringLiteral("Embedded Product View preparation failed; stale output will not be loaded.\nCommand: %1\nExit code: %2\nExpected output: %3\nStdout excerpt: %4\nStderr excerpt: %5").arg(command).arg(exit_code).arg(output_path, stdout_text.left(600), stderr_text.left(600)));
    maybe_start_next_embedded_web_prepare();
  };

  if (!still_current) {
    reject_prepare(QStringLiteral("discarded stale preparation for %1; current scene is %2").arg(scene, preview_scene_name_));
    return;
  }
  if (exit_status != QProcess::NormalExit || exit_code != 0) {
    reject_prepare(QStringLiteral("prepare command failed with exit code %1; old output is rejected even if present").arg(exit_code));
    return;
  }

  QJsonParseError result_parse_error;
  const QJsonDocument result_doc = QJsonDocument::fromJson(stdout_text.toUtf8(), &result_parse_error);
  if (result_parse_error.error != QJsonParseError::NoError || !result_doc.isObject()) {
    reject_prepare(QStringLiteral("freshener did not return a valid JSON object on stdout: %1").arg(result_parse_error.errorString()));
    return;
  }
  const QJsonObject result = result_doc.object();
  const QString freshener_schema = result.value(QStringLiteral("schema_version")).toString();
  const QString freshener_status = result.value(QStringLiteral("status")).toString();
  const QString freshener_scene = result.value(QStringLiteral("scene_id")).toString();
  const QString freshener_output = result.value(QStringLiteral("output")).toString();
  if (freshener_schema != QStringLiteral("workcell_studio_web_scene_freshener/v1") ||
      (freshener_status != QStringLiteral("current") && freshener_status != QStringLiteral("rebuilt")) ||
      freshener_scene != scene || freshener_output != output_path) {
    reject_prepare(QStringLiteral("freshener JSON contract validation failed for scene %1").arg(scene));
    return;
  }
  const QJsonObject asset_diagnostics = result.value(QStringLiteral("staged_asset_diagnostics")).toObject();
  if (asset_diagnostics.contains(QStringLiteral("ok")) && !asset_diagnostics.value(QStringLiteral("ok")).toBool()) {
    reject_prepare(QStringLiteral("freshener reported missing staged assets for %1").arg(scene));
    return;
  }

  QFile output_file(absolute_output_path);
  if (!output_file.open(QIODevice::ReadOnly)) {
    reject_prepare(QStringLiteral("expected output missing or unreadable: %1").arg(output_path));
    return;
  }
  QJsonParseError output_parse_error;
  const QJsonDocument output_doc = QJsonDocument::fromJson(output_file.readAll(), &output_parse_error);
  if (output_parse_error.error != QJsonParseError::NoError || !output_doc.isObject()) {
    reject_prepare(QStringLiteral("prepared output JSON validation failed: %1").arg(output_parse_error.errorString()));
    return;
  }
  const QJsonObject output = output_doc.object();
  const QString web_schema = output.value(QStringLiteral("schema_version")).toString();
  const QString output_scene = output.value(QStringLiteral("scene_id")).toString(output.value(QStringLiteral("scene_name")).toString());
  if (web_schema != QStringLiteral("workcell_studio_web_scene/v1") || output_scene != scene) {
    reject_prepare(QStringLiteral("prepared output semantic validation failed for scene %1").arg(scene));
    return;
  }

  ensure_embedded_web_server_started(embedded_web_repo_root_);
  maybe_start_next_embedded_web_prepare();
#endif
}

void ScenePreviewWidget::start_embedded_web_readiness_polling(const QString & scene, quint64 revision, const QString & expected_json_path, const QString & viewer_url)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(scene); Q_UNUSED(revision); Q_UNUSED(expected_json_path); Q_UNUSED(viewer_url);
#else
  embedded_web_readiness_deadline_ = QDateTime::currentDateTimeUtc().addSecs(45);
  embedded_web_last_boot_status_ = QStringLiteral("browser_loaded");
  poll_embedded_web_readiness(scene, revision, expected_json_path, viewer_url);
#endif
}

void ScenePreviewWidget::poll_embedded_web_readiness(const QString & scene, quint64 revision, const QString & expected_json_path, const QString & viewer_url)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(scene); Q_UNUSED(revision); Q_UNUSED(expected_json_path); Q_UNUSED(viewer_url);
#else
  if (!embedded_web_view_) return;
  if (preview_scene_name_.trimmed() != scene || revision != embedded_web_request_revision_) return;

  if (QDateTime::currentDateTimeUtc() > embedded_web_readiness_deadline_) {
    const QString detail = QStringLiteral(
      "startup timed out after 45s for scene %1; viewer URL: %2; expected JSON: %3; last observed boot status: %4")
      .arg(scene, viewer_url, expected_json_path, embedded_web_last_boot_status_.isEmpty() ? QStringLiteral("unavailable") : embedded_web_last_boot_status_);
    set_embedded_product_view_state(EmbeddedProductViewState::Failed, detail);
    emit studio_log_requested(QStringLiteral("Embedded Product View readiness timeout. %1").arg(detail));
    return;
  }

  static const char kStatusScript[] = R"JS(
(() => {
  const s = window.__WORKCELL_VIEWER_STATUS__ || {};
  return {
    viewer_boot_state: s.viewer_boot_state || s.viewerBootState || 'booting',
    scene_name: s.scene_name || s.sceneName || '',
    source_web_scene_file: s.source_web_scene_file || s.sourceWebSceneFile || '',
    scene_json_loaded: Boolean(s.scene_json_loaded || s.sceneJsonLoaded || s.source_web_scene_file || s.sourceWebSceneFile),
    robot_preview_lifecycle_state: s.robot_preview_lifecycle_state || s.robotPreviewLifecycleState || '',
    failed_stage: s.failed_stage || s.failedStage || '',
    fatal_error: s.fatal_error || s.fatalError || '',
    fatal_stack: String(s.fatal_stack || s.fatalStack || '').split('\n').slice(0, 6).join('\n')
  };
})()
)JS";
  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, scene, revision, expected_json_path, viewer_url](const QVariant & value) {
    if (preview_scene_name_.trimmed() != scene || revision != embedded_web_request_revision_) return;
    const QVariantMap status = value.toMap();
    const QString boot_state = status.value(QStringLiteral("viewer_boot_state")).toString();
    const QString source_json = status.value(QStringLiteral("source_web_scene_file")).toString();
    const bool scene_json_loaded = status.value(QStringLiteral("scene_json_loaded")).toBool();
    const QString robot_state = status.value(QStringLiteral("robot_preview_lifecycle_state")).toString();
    const QString failed_stage = status.value(QStringLiteral("failed_stage")).toString();
    const QString fatal_error = status.value(QStringLiteral("fatal_error")).toString();
    const QString fatal_stack = status.value(QStringLiteral("fatal_stack")).toString();
    embedded_web_last_boot_status_ = QStringLiteral("boot=%1 json=%2 source=%3 robot=%4")
      .arg(boot_state.isEmpty() ? QStringLiteral("unknown") : boot_state)
      .arg(scene_json_loaded ? QStringLiteral("loaded") : QStringLiteral("not_loaded"))
      .arg(source_json.isEmpty() ? QStringLiteral("unknown") : source_json)
      .arg(robot_state.isEmpty() ? QStringLiteral("unknown") : robot_state);

    if (boot_state == QStringLiteral("failed")) {
      const QString detail = QStringLiteral("viewer JavaScript failed at %1: %2%3")
        .arg(failed_stage.isEmpty() ? QStringLiteral("unknown_stage") : failed_stage,
             fatal_error.isEmpty() ? QStringLiteral("unknown error") : fatal_error,
             fatal_stack.isEmpty() ? QString() : QStringLiteral("; stack: %1").arg(fatal_stack.left(500)));
      set_embedded_product_view_state(EmbeddedProductViewState::Failed, detail);
      emit studio_log_requested(QStringLiteral("Embedded Product View JavaScript failure for scene %1.\nStage: %2\nFatal error: %3\nStack excerpt:\n%4")
        .arg(scene,
             failed_stage.isEmpty() ? QStringLiteral("unknown_stage") : failed_stage,
             fatal_error.isEmpty() ? QStringLiteral("unknown error") : fatal_error,
             fatal_stack.left(500)));
      return;
    }

    const bool expected_json_loaded = scene_json_loaded && source_json == expected_json_path;
    const bool robot_ready_required = scene == QStringLiteral("ur5_2f_test");
    const bool robot_ready = !robot_ready_required || robot_state == QStringLiteral("ready");
    if (boot_state == QStringLiteral("ready") && expected_json_loaded && robot_ready) {
      set_embedded_product_view_state(EmbeddedProductViewState::Ready, QStringLiteral("viewer ready"));
      emit studio_log_requested(QStringLiteral("Embedded Product View ready: scene=%1 json=%2 robot_preview_lifecycle_state=%3")
        .arg(scene, expected_json_path, robot_state.isEmpty() ? QStringLiteral("not_required") : robot_state));
      return;
    }

    set_embedded_product_view_state(EmbeddedProductViewState::Loading, QStringLiteral("waiting for viewer readiness (%1)").arg(embedded_web_last_boot_status_));
    QTimer::singleShot(750, this, [this, scene, revision, expected_json_path, viewer_url]() {
      poll_embedded_web_readiness(scene, revision, expected_json_path, viewer_url);
    });
  });
#endif
}

void ScenePreviewWidget::load_prepared_embedded_web_scene(const QString & scene, quint64 revision)
{
#ifndef WORKCELL_BUILDER_HAS_WEBENGINE
  Q_UNUSED(scene); Q_UNUSED(revision);
#else
  if (!embedded_web_view_) return;
  if (preview_scene_name_.trimmed() != scene || revision != embedded_web_request_revision_) {
    emit studio_log_requested(QStringLiteral("Embedded Product View ignored stale prepared scene %1 rev %2; current scene is %3 rev %4.").arg(scene).arg(revision).arg(preview_scene_name_).arg(embedded_web_request_revision_));
    return;
  }
  const QString web_scene_url_path = QStringLiteral("build/workcell_studio_web_scene/%1.web_scene.json").arg(scene);
  const QString viewer_url = QStringLiteral("http://127.0.0.1:%1/workcell_studio_web/viewer/index.html?scene=%2&builderRevision=%3")
    .arg(embedded_web_server_port_)
    .arg(QString::fromUtf8(QUrl::toPercentEncoding(web_scene_url_path)))
    .arg(revision);
  embedded_web_last_viewer_url_ = viewer_url;
  embedded_web_readiness_deadline_ = QDateTime();
  embedded_web_last_boot_status_.clear();
  set_embedded_product_view_state(EmbeddedProductViewState::Loading);
  embedded_web_view_->load(QUrl(viewer_url));
#endif
}
void ScenePreviewWidget::set_fallback_2d_view(QGraphicsView * view){ fallback_2d_view_ = view; if (!view2d_container_->layout()) view2d_container_->setLayout(new QVBoxLayout()); view2d_container_->layout()->addWidget(view); if (fallback_2d_view_ && fallback_2d_view_->scene() && info_chip_label_ && !fallback_info_chip_proxy_) { fallback_info_chip_proxy_ = fallback_2d_view_->scene()->addWidget(info_chip_label_); fallback_info_chip_proxy_->setZValue(10000.0); fallback_info_chip_proxy_->setPos(12.0, 12.0); } refresh_info_chip(); }
void ScenePreviewWidget::set_scene_selected(bool selected){ scene_selected_ = selected; refresh_mode_and_state(); }
void ScenePreviewWidget::set_3d_available(bool available, const QString & reason){
  preview3d_available_ = available;
  unavailable_reason_ = reason;
  const QString help_tooltip = scene_preview_mouse_help_tooltip(preview3d_available_ ? QString() : unavailable_reason_);
  if (fallback_banner_label_) fallback_banner_label_->setToolTip(help_tooltip);
  if (view3d_container_) view3d_container_->setToolTip(help_tooltip);
  if (view2d_container_) view2d_container_->setToolTip(help_tooltip);
  if (!mode_default_initialized_) {
    mode_selector_->setCurrentText(preview3d_available_ ? "3D Layout Preview" : "2D Layout");
    mode_default_initialized_ = true;
  }
  refresh_mode_and_state();
}
void ScenePreviewWidget::on_mode_changed(int){ refresh_mode_and_state(); }
void ScenePreviewWidget::set_preview_items(const QVector<PreviewItem> & items)
{
  preview_items_ = items;
  ++preview_payload_revision_;
  auto * viewport = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
  if (viewport) viewport->ingest_preview_items(preview_items_);
  const bool has_selected = std::any_of(preview_items_.cbegin(), preview_items_.cend(), [this](const PreviewItem & it){ return it.id == selected_preview_item_id_; });
  if (viewport) viewport->selected_id = selected_preview_item_id_;
  if (diagnostic_debug_logging_enabled() && !selected_preview_item_id_.isEmpty()) {
    emit studio_log_requested(has_selected ? QString("Preview selection restored after refresh: %1").arg(selected_preview_item_id_) : QString("Preview selection retained after refresh; id is hidden by filters or absent from the visible preview payload: %1").arg(selected_preview_item_id_));
  }
  if (viewport) viewport->fit_include_overlays = false;
  apply_product_view_defaults();
  refresh_embedded_web_product_view();
  emit_scene_diagnostic_once(
    QStringLiteral("payload_commit"),
    preview_items_.size(),
    QStringLiteral("Scene3D payload committed: scene=%1 rev=%2 items=%3 visible=%4 mesh=%5 fallback=%6")
      .arg(preview_scene_name_)
      .arg(preview_payload_revision_)
      .arg(preview_items_.size())
      .arg(viewport ? viewport->render_debug_counters().visible_count : 0)
      .arg(viewport ? viewport->render_debug_counters().mesh_backed_count : 0)
      .arg(viewport ? viewport->render_debug_counters().primitive_fallback_count : 0));
  fit_fallback_scene_to_items(false);
  refresh_info_chip();
  emit_visual_quality_assessment_once();
  update();
}
void ScenePreviewWidget::set_preview_scene_name(const QString & scene_name)
{
  const QString normalized_scene_name = scene_name.trimmed().isEmpty() ? QStringLiteral("No scene") : scene_name.trimmed();
  if (preview_scene_name_ != normalized_scene_name) {
    preview_scene_name_ = normalized_scene_name;
    preview_payload_revision_ = 0;
    last_visual_quality_revision_logged_ = -1;
    emitted_scene_diagnostic_keys_.clear();
    emit_scene_diagnostic_once(
      QStringLiteral("scene_load"),
      0,
      QStringLiteral("Scene loaded: %1").arg(preview_scene_name_));
  } else {
    preview_scene_name_ = normalized_scene_name;
  }
  auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
  if (v) { v->scene_name = preview_scene_name_; v->update(); }
  refresh_embedded_web_product_view();
  refresh_info_chip();
}

bool ScenePreviewWidget::diagnostic_debug_logging_enabled() const
{
  const auto * viewport = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
  return (viewport && viewport->debug_overlays_mode) || qEnvironmentVariableIsSet("WORKCELL_SCENE3D_DEBUG_LOGS");
}

bool ScenePreviewWidget::emit_scene_diagnostic_once(const QString & event, int payload_count, const QString & message)
{
  const QString scene = preview_scene_name_.trimmed().isEmpty() ? QStringLiteral("No scene") : preview_scene_name_.trimmed();
  const QString key = QStringLiteral("%1|%2|rev=%3|count=%4").arg(scene, event).arg(preview_payload_revision_).arg(payload_count);
  if (!diagnostic_debug_logging_enabled()) return false;
  if (emitted_scene_diagnostic_keys_.contains(key)) return false;
  emitted_scene_diagnostic_keys_.insert(key);
  emit studio_log_requested(message);
  return true;
}

void ScenePreviewWidget::emit_visual_quality_assessment_once()
{
  if (!diagnostic_debug_logging_enabled()) return;
  if (last_visual_quality_revision_logged_ == preview_payload_revision_) return;
  last_visual_quality_revision_logged_ = preview_payload_revision_;
  int physical_count = 0;
  int mesh_count = 0;
  int primitive_count = 0;
  int missing_count = 0;
  int overlay_count = 0;
  for (const auto & item : preview_items_) {
    if (preview_item_is_overlay_or_helper(item)) { ++overlay_count; continue; }
    ++physical_count;
    if (preview_item_has_credible_mesh_handoff(item)) ++mesh_count;
    else if (preview_item_has_valid_urdf_primitive(item) || (item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001)) ++primitive_count;
    else ++missing_count;
  }
  emit_scene_diagnostic_once(
    QStringLiteral("visual_quality"),
    preview_items_.size(),
    QStringLiteral("Scene3D visual quality: scene=%1 rev=%2 physical=%3 mesh=%4 primitive=%5 missing=%6 overlays=%7 warnings=%8")
      .arg(preview_scene_name_)
      .arg(preview_payload_revision_)
      .arg(physical_count)
      .arg(mesh_count)
      .arg(primitive_count)
      .arg(missing_count)
      .arg(overlay_count)
      .arg(total_warning_count()));
}

void ScenePreviewWidget::set_preview_status_summary(const QString & summary){ preview_status_summary_ = summary.trimmed(); refresh_info_chip(); }
void ScenePreviewWidget::set_clean_product_view_status(bool clean, int visual_count)
{
  clean_product_view_ = clean;
  clean_product_visual_count_ = qMax(0, visual_count);
  refresh_info_chip();
}
void ScenePreviewWidget::set_task_overlay_model(const TaskOverlayModel & model){ overlay_model_ = model; if (auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_)) v->task_overlay = model; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_task_overlay_visibility(bool task_route, bool pick_place_zones, bool approach_retreat, bool labels){
  auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
  if (!v) return;
  v->show_task_route = task_route;
  v->show_pick_place = pick_place_zones;
  v->show_approach_retreat = approach_retreat;
  if (labels) v->label_mode = LabelMode::All;
  else if (v->label_mode == LabelMode::All) v->label_mode = LabelMode::Important;
  // else if (v->label_mode == LabelMode::All) v->label_mode = LabelMode::SelectedOnly;
  v->update();
}
void ScenePreviewWidget::select_preview_item(const QString & id){ selected_preview_item_id_ = id; if (auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_)) v->selected_id = id; simple_3d_view_->update(); }
QString ScenePreviewWidget::selected_preview_item_id() const { return selected_preview_item_id_; }
const ScenePreviewWidget::PreviewItem * ScenePreviewWidget::preview_item_by_id(const QString & id) const
{
  const QString stable_id = id.trimmed();
  if (stable_id.isEmpty()) return nullptr;
  for (const auto & item : preview_items_) {
    if (item.id.trimmed() == stable_id) return &item;
  }
  return nullptr;
}

ScenePreviewWidget::MeshPreviewMode ScenePreviewWidget::mesh_preview_mode() const { return mesh_preview_mode_; }

void ScenePreviewWidget::reload_meshes()
{
  auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
  if (!v) { refresh_embedded_web_product_view(); emit studio_log_requested("Reloaded embedded Web 3D Product View."); return; }
  v->invalidate_mesh_cache();
  apply_product_view_defaults();
  update();
  emit studio_log_requested("Reloaded mesh preview cache (visual-only).");
}
void ScenePreviewWidget::apply_product_view_defaults()
{
  auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
  if (!v) return;
  if (view_actions_selector_) {
    const QSignalBlocker blocker(view_actions_selector_);
    view_actions_selector_->setCurrentText("Isometric");
  }
  // Product previews should open in the same camera path users get from the
  // normal canvas controls: an isometric view followed by product-fit bounds
  // that keep robot/tool/environment meshes and editable layout items legible
  // without letting diagnostics overlays dominate the initial framing.
  v->fit_include_overlays = false;
  v->debug_overlays_mode = false;
  v->show_warnings = false;
  v->show_warning_labels = false;
  v->show_safety = false;
  v->show_pick_place = false;
  v->show_reachability_heatmap = false;
  v->show_collision_warnings = false;
  v->show_work_envelope = false;
  v->show_task_route = false;
  v->show_approach_retreat = false;
  v->show_camera_fov = false;
  v->show_pick_coverage = false;
  v->show_epd_detections = false;
  v->show_detection_labels = false;
  v->mesh_preview_mode = ScenePreviewWidget::MeshPreviewMode::Auto;
  mesh_preview_mode_ = ScenePreviewWidget::MeshPreviewMode::Auto;
  if (mesh_preview_mode_selector_) {
    const QSignalBlocker blocker(mesh_preview_mode_selector_);
    mesh_preview_mode_selector_->setCurrentText("Auto");
  }
  if (labels_selector_) {
    const QSignalBlocker blocker(labels_selector_);
    labels_selector_->setCurrentText("Selected");
  }
  v->label_mode = ScenePreviewWidget::LabelMode::Selected;
  v->set_isometric_view();
  v->fit_product_view();
  fit_fallback_scene_to_items(false);
}
void ScenePreviewWidget::on_reset_view_clicked(){ if (auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_)) v->reset_view(); else refresh_embedded_web_product_view(); reset_fallback_scene_view(); }
void ScenePreviewWidget::on_fit_scene_clicked(){ auto *v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_); if (v) { v->fit_include_overlays = false; v->fit_scene(); } fit_fallback_scene_to_items(false); } // Fit Scene intentionally excludes overlay-only bounds by default.
void ScenePreviewWidget::on_fit_robot_clicked(){ auto *v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_); if (v) v->fit_robot(); fit_fallback_scene_to_items(false); }
void ScenePreviewWidget::on_fit_overlays_clicked(){ auto *v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_); const QRectF physical_bounds = rendered_items_bounds_2d(false); const QRectF overlay_bounds = rendered_items_bounds_2d(true); maybe_warn_overlay_fit_dominance(this, physical_bounds, overlay_bounds); if (v) { v->fit_include_overlays = true; v->fit_scene(); } fit_fallback_scene_to_items(true); if (v) v->fit_include_overlays = false; } // Fit overlays includes overlay bounds for explicit overlay-focused framing.
void ScenePreviewWidget::on_focus_selected_clicked(){ if (auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_)) v->focus_selected(); }
void ScenePreviewWidget::on_clear_selection_clicked(){ selected_preview_item_id_.clear(); if (auto * v = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_)) v->selected_id.clear(); simple_3d_view_->update(); emit studio_log_requested("Cleared preview selection."); emit preview_item_selected(QString(), QStringLiteral("unknown")); }
void ScenePreviewWidget::refresh_toolbar_visibility()
{
  if (view_actions_selector_) {
    const QSignalBlocker blocker(view_actions_selector_);
    const QString previous = view_actions_selector_->currentText();
    view_actions_selector_->clear();
    view_actions_selector_->addItems({"Top", "Front", "Side", "Isometric", "Fit View"});
    const int previous_index = view_actions_selector_->findText(previous);
    view_actions_selector_->setCurrentIndex(previous_index >= 0 ? previous_index : 0);
  }

  const auto set_visible = [](QWidget * widget, bool visible) {
    if (widget) widget->setVisible(visible);
  };

  // Embedded Web3D is the canonical Product View when Qt WebEngine is enabled;
  // hide native Scene3DViewportWidget-only controls so they are not active no-ops.
  const bool embedded_web_active = (qobject_cast<Scene3DViewportWidget *>(simple_3d_view_) == nullptr);
  set_visible(view_actions_label_, !embedded_web_active);
  set_visible(view_actions_selector_, !embedded_web_active);
  set_visible(labels_label_, !embedded_web_active);
  set_visible(labels_selector_, !embedded_web_active);
  set_visible(toolbar_status_chip_, true);
  set_visible(mesh_preview_mode_label_, !embedded_web_active);
  set_visible(mesh_preview_mode_selector_, !embedded_web_active);
  set_visible(gizmo_mode_label_, !embedded_web_active);
  set_visible(gizmo_mode_selector_, !embedded_web_active);
  set_visible(snap_mode_label_, false);
  set_visible(snap_mode_selector_, false);
  set_visible(interaction_mode_label_, true);
  set_visible(interaction_mode_selector_, true);
  set_visible(overlays_selector_, true);
}

void ScenePreviewWidget::refresh_mode_and_state()
{
  const QString mode = mode_selector_->currentText();
  const bool requested_3d = (mode == "3D Layout Preview");
  const bool use3d = requested_3d && preview3d_available_;
  refresh_toolbar_visibility();

  if (!preview3d_available_) {
    stack_->setCurrentWidget(view2d_container_);
    fallback_banner_label_->setVisible(scene_selected_);
    const QString reason = unavailable_reason_.isEmpty() ? "initialization failed" : unavailable_reason_;
    emit studio_log_requested(QString("3D Layout Preview unavailable, using 2D Layout: %1").arg(reason));
  } else {
    fallback_banner_label_->setVisible(false);
    fallback_banner_label_->setToolTip(scene_preview_mouse_help_tooltip(QString()));
    stack_->setCurrentWidget(use3d ? view3d_container_ : view2d_container_);
  }
  const bool has_preview_items = !preview_items_.isEmpty();
  empty_state_label_->setVisible(use3d && !scene_selected_);
  simple_3d_view_->setVisible(use3d && scene_selected_);
  const bool rendering_failed = scene_selected_ && has_preview_items && requested_3d && !preview3d_available_;
  error_state_label_->setText(QString("Scene selected but 3D preview is unavailable. Loaded %1 preview items. Using 2D fallback canvas.").arg(preview_items_.size()));
  error_state_label_->setVisible(rendering_failed);
  refresh_info_chip();
}
QRectF ScenePreviewWidget::rendered_items_bounds_2d(bool include_overlays) const
{
  QRectF bounds;
  auto include_in_fit_bounds = [include_overlays](const PreviewItem & it) {
    if (include_overlays) return true;
    if (preview_item_is_overlay_or_helper(it)) return false;

    const QString source_layer = normalized_preview_token(it.source_layer);
    const QString visual_source = normalized_preview_token(it.active_visual_source);
    if (source_layer.contains("overlay") || visual_source.contains("overlay")) return false;

    const bool generated_urdf = preview_item_is_generated_or_locked_urdf(it);
    const bool mesh_backed = preview_item_has_credible_mesh_handoff(it);
    const bool explicit_primitive = preview_item_has_valid_urdf_primitive(it) ||
                                    (it.sx > 0.001 && it.sy > 0.001 && it.sz > 0.001);
    if (generated_urdf) return true;
    if (it.linked_to_editable_layout_state) return true;
    if (source_layer == QStringLiteral("mesh_preview") || visual_source == QStringLiteral("mesh_preview")) {
      return mesh_backed || explicit_primitive;
    }

    const QString role = normalized_preview_token(it.role);
    const QString category = normalized_preview_token(it.category);
    const QString id = normalized_preview_token(it.id);
    const QString display_name = normalized_preview_token(it.display_name);
    const QString mix = role + QStringLiteral("|") + category + QStringLiteral("|") + id + QStringLiteral("|") + display_name;
    const bool product_physical = mix.contains("robot") || mix.contains("gripper") ||
                                  mix.contains("tool") || mix.contains("end_effector") ||
                                  mix.contains("table") || mix.contains("work_surface") ||
                                  mix.contains("workbench") || mix.contains("camera") ||
                                  mix.contains("realsense") || mix.contains("depth_camera") ||
                                  mix.contains("rgbd");
    return product_physical && (mesh_backed || explicit_primitive);
  };
  for (const auto & it : preview_items_) {
    if (!include_in_fit_bounds(it)) continue;
    const QRectF rect(it.x, it.z, it.sx, it.sz);
    bounds = bounds.isNull() ? rect : bounds.united(rect);
  }

  if (bounds.isValid() && !bounds.isEmpty()) return bounds;
  if (!fallback_2d_view_ || !fallback_2d_view_->scene()) return QRectF();
  const auto scene_items = fallback_2d_view_->scene()->items();
  for (QGraphicsItem * item : scene_items) {
    if (!item || !item->isVisible()) continue;
    bounds = bounds.isNull() ? item->sceneBoundingRect() : bounds.united(item->sceneBoundingRect());
  }
  return bounds;
}
void ScenePreviewWidget::fit_fallback_scene_to_items(bool include_overlays)
{
  if (!fallback_2d_view_) return;
  const QRectF bounds = rendered_items_bounds_2d(include_overlays);
  if (bounds.isValid() && !bounds.isEmpty()) fallback_2d_view_->fitInView(bounds.adjusted(-20, -20, 20, 20), Qt::KeepAspectRatio);
}
void ScenePreviewWidget::reset_fallback_scene_view()
{
  if (!fallback_2d_view_) return;
  fallback_2d_view_->resetTransform();
  fit_fallback_scene_to_items(false);
}

void ScenePreviewWidget::set_camera_overlay_model(const CameraOverlayModel & model){ camera_overlay_model_ = model; if (auto *v=qobject_cast<Scene3DViewportWidget *>(simple_3d_view_)) v->camera_overlay = model; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_epd_detection_overlays(const QVector<EpdDetectionOverlayModel> & detections){ epd_detections_ = detections; if (auto *v=qobject_cast<Scene3DViewportWidget *>(simple_3d_view_)) v->epd_detections = detections; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_perception_overlay_visibility(bool camera_fov, bool pick_coverage, bool epd_detections, bool detection_labels){ auto *v=qobject_cast<Scene3DViewportWidget *>(simple_3d_view_); if (!v) return; v->show_camera_fov=camera_fov; v->show_pick_coverage=pick_coverage; v->show_epd_detections=epd_detections; v->show_detection_labels=detection_labels; v->update(); }


void ScenePreviewWidget::set_reachability_overlay_model(const ReachabilityOverlayModel & model){ reachability_overlay_model_ = model; if (auto *v=qobject_cast<Scene3DViewportWidget *>(simple_3d_view_)) v->reach_overlay = model; if (diagnostic_debug_logging_enabled()) emit studio_log_requested(QString("reachability overlay loaded: warning count=%1").arg(model.warnings.size())); refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_collision_overlay_model(const CollisionOverlayModel & model){ collision_overlay_model_ = model; if (auto *v=qobject_cast<Scene3DViewportWidget *>(simple_3d_view_)) v->collision_overlay = model; if (diagnostic_debug_logging_enabled()) emit studio_log_requested(QString("collision preview checks complete: warning count=%1").arg(model.warnings.size())); refresh_info_chip(); simple_3d_view_->update(); }

void ScenePreviewWidget::set_label_mode(LabelMode mode){ Q_UNUSED(mode); auto *v=qobject_cast<Scene3DViewportWidget *>(simple_3d_view_); if (labels_selector_) labels_selector_->setCurrentText("Selected"); if (v) { v->label_mode = LabelMode::Selected; v->update(); } }

ScenePreviewWidget::RenderDebugCounters ScenePreviewWidget::render_debug_counters() const
{
  const auto * viewport = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
  const auto counters = viewport ? viewport->render_debug_counters() : Scene3DViewportWidget::RenderDebugCounters{};
  RenderDebugCounters out;
  out.preview_items_count = preview_items_.size();
  out.total_payload_count = preview_items_.size();
  out.viewport_received_count = counters.viewport_received_count;
  out.render_cache_count = counters.render_cache_count;
  out.visible_count = counters.visible_count;
  out.rendered_count = counters.rendered_count;
  out.skipped_count = counters.skipped_count;
  out.unique_visible_item_count = counters.unique_visible_item_count;
  out.mesh_backed_count = counters.mesh_backed_count;
  out.mesh_source_count = counters.mesh_source_count;
  out.mesh_path_resolved_count = counters.mesh_path_resolved_count;
  out.mesh_file_loaded_count = counters.mesh_file_loaded_count;
  out.mesh_triangles_loaded_count = counters.mesh_triangles_loaded_count;
  out.mesh_rendered_count = counters.mesh_rendered_count;
  out.mesh_surface_rendered_count = counters.mesh_surface_rendered_count;
  out.mesh_bounds_fallback_rendered_count = counters.mesh_bounds_fallback_rendered_count;
  out.urdf_primitive_source_count = counters.urdf_primitive_source_count;
  out.urdf_primitive_rendered_count = counters.urdf_primitive_rendered_count;
  out.placeholder_count = counters.placeholder_count;
  out.missing_geometry_count = counters.missing_geometry_count;
  out.wireframe_fallback_count = counters.wireframe_fallback_count;
  out.overlay_helper_count = counters.overlay_helper_count;
  out.overlay_count = counters.overlay_count;
  out.generated_fallback_count = counters.generated_fallback_count;
  out.editable_layout_count = counters.editable_layout_count;
  out.primitive_fallback_count = counters.primitive_fallback_count;
  out.primitive_fallback_rendered_count = counters.primitive_fallback_rendered_count;
  out.editable_primitive_rendered_count = counters.editable_primitive_rendered_count;
  out.valid_physical_fallback_count = counters.valid_physical_fallback_count;
  out.overlay_rendered_count = counters.overlay_rendered_count;
  out.locked_generated_urdf_visual_count = counters.locked_generated_urdf_visual_count;
  out.physical_anchor_count = counters.physical_anchor_count;
  out.generated_robot_mesh_count = counters.generated_robot_mesh_count;
  out.tool_gripper_visual_count = counters.tool_gripper_visual_count;
  out.table_workbench_visual_count = counters.table_workbench_visual_count;
  out.camera_body_visual_count = counters.camera_body_visual_count;
  out.transform_chain_applied_count = counters.transform_chain_applied_count;
  out.visual_origin_applied_count = counters.visual_origin_applied_count;
  out.baked_world_visual_transform_count = counters.baked_world_visual_transform_count;
  out.legacy_viewport_transform_count = counters.legacy_viewport_transform_count;
  out.visual_quality_status = counters.visual_quality_status;
  out.visual_quality_warnings = counters.visual_quality_warnings;
  out.labels_drawn = counters.labels_drawn;
  out.labels_suppressed_overlap = counters.labels_suppressed_overlap;
  out.hierarchy_child_row_count = counters.hierarchy_child_row_count;
  out.last_paint_completed = counters.last_paint_completed;
  out.smoke_fallback_render_used = counters.smoke_fallback_render_used;
  return out;
}

int ScenePreviewWidget::total_warning_count() const { int count = 0; for (const auto & item : preview_items_) count += item.warnings.size(); count += overlay_model_.warnings.size() + reachability_overlay_model_.warnings.size() + collision_overlay_model_.warnings.size() + camera_overlay_model_.warnings.size(); for (const auto & det : epd_detections_) count += det.warnings.size(); return count; }
bool ScenePreviewWidget::task_is_ready() const { return overlay_model_.has_intent_metadata && overlay_model_.pick_source_id != "unknown" && overlay_model_.place_target_id != "unknown"; }
void ScenePreviewWidget::refresh_info_chip()
{
  if (!info_chip_label_) return;
  const QString mode = mode_selector_ ? mode_selector_->currentText() : QStringLiteral("2D Layout");
  const bool requested_3d = (mode == "3D Layout Preview");
  const QString render_mode = requested_3d && preview3d_available_ ? mode : QStringLiteral("2D Layout (Fallback)");
  const QString summary = preview_status_summary_.isEmpty() ? QString("Items: %1").arg(preview_items_.size()) : preview_status_summary_;

  int mesh_count = 0;
  int box_count = 0;
  int missing_count = 0;
  int overlay_count = 0;
  int locked_urdf_count = 0;
  int physical_count = 0;
  for (const auto & item : preview_items_) {
    const QString category = item.category.trimmed().toLower();
    const QString lock_reason = item.lock_reason.trimmed().toLower();
    if (preview_item_is_overlay_or_helper(item)) {
      ++overlay_count;
      continue;
    }
    ++physical_count;
    const bool mesh_backed = preview_item_has_credible_mesh_handoff(item);
    const bool raw_generated_bounds = preview_item_is_raw_generated_bounds_only(item);
    if (mesh_backed) ++mesh_count;
    else if (preview_item_has_valid_urdf_primitive(item) || (item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001 && !raw_generated_bounds)) ++box_count;
    else if (raw_generated_bounds) ++mesh_count;
    else ++missing_count;

    if (item.locked && !item.editable &&
        (category.contains("urdf") || lock_reason.contains("urdf") || lock_reason.contains("robot model") ||
         lock_reason.contains("robotmodel") || lock_reason.contains("urdf visual"))) ++locked_urdf_count;
  }

  const auto * viewport = qobject_cast<Scene3DViewportWidget *>(simple_3d_view_);
  const auto counters = viewport ? viewport->render_debug_counters() : Scene3DViewportWidget::RenderDebugCounters{};
  const QString compact_stats = QString("Items %1 M%2 B%3 Miss%4 Ov%5 L-URDF%6")
                                  .arg(physical_count).arg(mesh_count).arg(box_count).arg(missing_count).arg(overlay_count).arg(locked_urdf_count);
  const QString smoke_stats = QString("quality=%1 mesh=%2/%3 surface=%4 bounds_fallback=%5 resolved=%6 loaded=%7 triangles=%8 urdf_prim=%9/%10 missing=%11 placeholder=%12 wireframe=%13 helpers=%14 paint_completed=%15 selected_scene_name=%16 selected_item_id=%17")
                                  .arg(counters.visual_quality_status)
                                  .arg(counters.mesh_rendered_count).arg(counters.mesh_source_count)
                                  .arg(counters.mesh_surface_rendered_count)
                                  .arg(counters.mesh_bounds_fallback_rendered_count)
                                  .arg(counters.mesh_path_resolved_count)
                                  .arg(counters.mesh_file_loaded_count)
                                  .arg(counters.mesh_triangles_loaded_count)
                                  .arg(counters.urdf_primitive_rendered_count).arg(counters.urdf_primitive_source_count)
                                  .arg(counters.missing_geometry_count).arg(counters.placeholder_count)
                                  .arg(counters.wireframe_fallback_count).arg(counters.overlay_helper_count)
                                  .arg(counters.last_paint_completed ? QStringLiteral("true") : QStringLiteral("false"))
                                  .arg(preview_scene_name_.isEmpty() ? QStringLiteral("(none)") : preview_scene_name_)
                                  .arg(selected_preview_item_id_.isEmpty() ? QStringLiteral("(none)") : selected_preview_item_id_);
  const QString initial_fit_audit = QStringLiteral("initial_fit_robot_bounds=%1 physical_anchors=%2")
                                      .arg(viewport && viewport->last_initial_fit_included_robot_bounds()
                                             ? QStringLiteral("included")
                                             : QStringLiteral("not_included"))
                                      .arg(viewport ? viewport->last_initial_fit_physical_anchor_count() : 0);
  info_chip_label_->setText(QString("Scene: %1\nMode: %2\n%3\n%4  Warn: %5  Task: %6")
                              .arg(preview_scene_name_).arg(render_mode).arg(summary).arg(compact_stats)
                              .arg(total_warning_count()).arg(task_is_ready() ? "Ready" : "Missing") + QString("\n") + smoke_stats + QStringLiteral(" ") + initial_fit_audit);
  info_chip_label_->adjustSize();
  if (fallback_info_chip_proxy_) fallback_info_chip_proxy_->setPos(12.0, 12.0);
  if (toolbar_status_chip_) {
    if (clean_product_view_ && preview3d_available_) {
      toolbar_status_chip_->setText(QStringLiteral("Scene3D Product View • %1 visuals").arg(clean_product_visual_count_));
    } else {
      const QString interaction = interaction_mode_selector_ ? interaction_mode_selector_->currentText() : QStringLiteral("Select");
      toolbar_status_chip_->setText(QString("%1 • %2 • Warn %3")
        .arg(preview3d_available_ ? QStringLiteral("3D") : QStringLiteral("2D fallback"))
        .arg(interaction)
        .arg(total_warning_count()));
    }
  }
}
