#include "scene_preview_widget.h"

#include <algorithm>
#include <QComboBox>
#include <QFrame>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsProxyWidget>
#include <QHBoxLayout>
#include <functional>
#include <QLabel>
#include <QMouseEvent>
#include "scene3d_viewport_widget.h"
#include <QPainter>
#include <QPushButton>
#include <QStackedWidget>
#include <QVector3D>
#include <QVBoxLayout>
#include <QtMath>


ScenePreviewWidget::ScenePreviewWidget(QWidget * parent) : QWidget(parent)
{
  auto * root = new QVBoxLayout(this);
  auto * controls = new QHBoxLayout();
  controls->addWidget(new QLabel("View mode", this));
  mode_selector_ = new QComboBox(this);
  mode_selector_->addItems({"3D Layout Preview", "2D Layout", "Debug Overlays"});
  controls->addWidget(mode_selector_);
  controls->addSpacing(8);
  controls->addWidget(new QLabel("Mesh Preview:", this));
  mesh_preview_mode_selector_ = new QComboBox(this);
  mesh_preview_mode_selector_->addItems({"Auto", "Meshes", "Primitives"});
  mesh_preview_mode_selector_->setCurrentText("Auto");
  mesh_preview_mode_selector_->setToolTip("Mesh preview mode is visual-only and does not alter generated runtime files.");
  controls->addWidget(mesh_preview_mode_selector_);
  controls->addSpacing(8);
  controls->addWidget(new QLabel("Labels:", this));
  labels_selector_ = new QComboBox(this);
  labels_selector_->addItems({"Off", "Important", "Selected", "All"});
  labels_selector_->setCurrentText("Important");
  labels_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  controls->addWidget(labels_selector_);
  reset_view_button_ = new QPushButton("Reset View", this); controls->addWidget(reset_view_button_);
  isometric_view_button_ = new QPushButton("Isometric", this); controls->addWidget(isometric_view_button_);
  top_view_button_ = new QPushButton("Top", this); controls->addWidget(top_view_button_);
  front_view_button_ = new QPushButton("Front", this); controls->addWidget(front_view_button_);
  side_view_button_ = new QPushButton("Side", this); controls->addWidget(side_view_button_);
  fit_scene_button_ = new QPushButton("Fit Scene", this); controls->addWidget(fit_scene_button_);
  focus_selected_button_ = new QPushButton("Focus Selected", this); controls->addWidget(focus_selected_button_);
  clear_selection_button_ = new QPushButton("Clear Selection", this); controls->addWidget(clear_selection_button_);
  overlays_selector_ = new QComboBox(this); overlays_selector_->addItems({"Overlays", "Reachability Heatmap", "Collision Warnings", "Safety Zones", "Work Envelope", "Warning Labels", "Labels", "Pick/Place Zones", "Task Route", "Approach/Retreat", "Camera FOV", "Pick Coverage", "EPD Detections", "Detection Labels", "Warnings", "Focus Selected", "Fit Scene", "Fit overlays", "Clear Selection"}); controls->addWidget(overlays_selector_);
  controls->addStretch(1);
  root->addLayout(controls);
  stack_ = new QStackedWidget(this);
  view3d_container_ = new QWidget(this); auto * v3 = new QVBoxLayout(view3d_container_);
  simple_3d_view_ = new Scene3DViewportWidget(view3d_container_); v3->addWidget(simple_3d_view_);
  empty_state_label_ = new QLabel("No scene selected\nOpen a scene or create a new cell to preview it.", view3d_container_);
  empty_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(empty_state_label_);
  error_state_label_ = new QLabel("3D Layout Preview unavailable", view3d_container_); error_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(error_state_label_);
  view2d_container_ = new QWidget(this); view2d_container_->setLayout(new QVBoxLayout());
  fallback_banner_label_ = new QLabel("2D fallback preview active", view2d_container_);
  fallback_banner_label_->setAlignment(Qt::AlignCenter);
  fallback_banner_label_->setVisible(false);
  view2d_container_->layout()->addWidget(fallback_banner_label_);
  info_chip_label_ = new QLabel(view2d_container_);
  info_chip_label_->setObjectName("previewInfoChip");
  info_chip_label_->setStyleSheet("QLabel#previewInfoChip { background-color: rgba(15,23,42,210); color: #e2e8f0; border-radius: 6px; padding: 5px 8px; }");
  info_chip_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  info_chip_label_->setWordWrap(true);
  stack_->addWidget(view3d_container_); stack_->addWidget(view2d_container_); root->addWidget(stack_, 1);
  connect(mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ScenePreviewWidget::on_mode_changed);
  connect(reset_view_button_, &QPushButton::clicked, this, &ScenePreviewWidget::on_reset_view_clicked);
  connect(labels_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto *v = static_cast<Scene3DViewportWidget *>(simple_3d_view_);
    const QString choice = labels_selector_->currentText();
    if (choice == "Off") v->label_mode = LabelMode::Off;
    else if (choice == "Important") v->label_mode = LabelMode::Important;
    else if (choice == "Selected") v->label_mode = LabelMode::Selected;
    else v->label_mode = LabelMode::All;
    v->update();
  });
  connect(mesh_preview_mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto * v = static_cast<Scene3DViewportWidget *>(simple_3d_view_);
    const QString choice = mesh_preview_mode_selector_->currentText();
    if (choice == "Meshes") mesh_preview_mode_ = MeshPreviewMode::Meshes;
    else if (choice == "Primitives") mesh_preview_mode_ = MeshPreviewMode::Primitives;
    else mesh_preview_mode_ = MeshPreviewMode::Auto;
    v->mesh_preview_mode = mesh_preview_mode_;
    v->update();
  });
  connect(fit_scene_button_, &QPushButton::clicked, this, &ScenePreviewWidget::on_fit_scene_clicked);
  connect(isometric_view_button_, &QPushButton::clicked, this, [this](){ static_cast<Scene3DViewportWidget *>(simple_3d_view_)->set_isometric_view(); });
  connect(top_view_button_, &QPushButton::clicked, this, [this](){ static_cast<Scene3DViewportWidget *>(simple_3d_view_)->set_top_view(); });
  connect(front_view_button_, &QPushButton::clicked, this, [this](){ static_cast<Scene3DViewportWidget *>(simple_3d_view_)->set_front_view(); });
  connect(side_view_button_, &QPushButton::clicked, this, [this](){ static_cast<Scene3DViewportWidget *>(simple_3d_view_)->set_side_view(); });
  connect(focus_selected_button_, &QPushButton::clicked, this, &ScenePreviewWidget::on_focus_selected_clicked);
  connect(clear_selection_button_, &QPushButton::clicked, this, &ScenePreviewWidget::on_clear_selection_clicked);
  connect(overlays_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto *v = static_cast<Scene3DViewportWidget *>(simple_3d_view_);
    const QString choice = overlays_selector_->currentText();
    if (choice == "Reachability Heatmap") v->show_reachability_heatmap = !v->show_reachability_heatmap;
    else if (choice == "Collision Warnings") v->show_collision_warnings = !v->show_collision_warnings;
    else if (choice == "Safety Zones") v->show_safety = !v->show_safety;
    else if (choice == "Work Envelope") v->show_work_envelope = !v->show_work_envelope;
    else if (choice == "Warning Labels") v->show_warning_labels = !v->show_warning_labels;
    else if (choice == "Focus Selected") on_focus_selected_clicked();
    else if (choice == "Fit Scene") on_fit_scene_clicked();
    else if (choice == "Fit overlays") on_fit_overlays_clicked();
    else if (choice == "Clear Selection") on_clear_selection_clicked();
    v->update();
  });
  static_cast<Scene3DViewportWidget *>(simple_3d_view_)->select_cb = [this](const QString & id, const QString & role){ select_preview_item(id); emit preview_item_selected(id, role); emit studio_log_requested(QString("Selected preview item: %1 (%2)").arg(id, role)); };
  refresh_info_chip();
  refresh_mode_and_state();
}
void ScenePreviewWidget::set_fallback_2d_view(QGraphicsView * view){ fallback_2d_view_ = view; if (!view2d_container_->layout()) view2d_container_->setLayout(new QVBoxLayout()); view2d_container_->layout()->addWidget(view); if (fallback_2d_view_ && fallback_2d_view_->scene() && info_chip_label_ && !fallback_info_chip_proxy_) { fallback_info_chip_proxy_ = fallback_2d_view_->scene()->addWidget(info_chip_label_); fallback_info_chip_proxy_->setZValue(10000.0); fallback_info_chip_proxy_->setPos(12.0, 12.0); } refresh_info_chip(); }
void ScenePreviewWidget::set_scene_selected(bool selected){ scene_selected_ = selected; refresh_mode_and_state(); }
void ScenePreviewWidget::set_3d_available(bool available, const QString & reason){
  preview3d_available_ = available;
  unavailable_reason_ = reason;
  if (!mode_default_initialized_) {
    mode_selector_->setCurrentText(preview3d_available_ ? "3D Layout Preview" : "2D Layout");
    mode_default_initialized_ = true;
  }
  refresh_mode_and_state();
}
void ScenePreviewWidget::on_mode_changed(int){ refresh_mode_and_state(); }
void ScenePreviewWidget::set_preview_items(const QVector<PreviewItem> & items){ preview_items_ = items; static_cast<Scene3DViewportWidget *>(simple_3d_view_)->items = preview_items_; const bool has_selected = std::any_of(preview_items_.cbegin(), preview_items_.cend(), [this](const PreviewItem & it){ return it.id == selected_preview_item_id_; }); if (!has_selected && !selected_preview_item_id_.isEmpty()) { emit studio_log_requested(QString("Preview selection id no longer present: %1").arg(selected_preview_item_id_)); } else { static_cast<Scene3DViewportWidget *>(simple_3d_view_)->selected_id = selected_preview_item_id_; } emit studio_log_requested(QString("Loaded %1 preview items.").arg(preview_items_.size())); fit_fallback_scene_to_items(); refresh_info_chip(); update(); }
void ScenePreviewWidget::set_preview_scene_name(const QString & scene_name){ preview_scene_name_ = scene_name.trimmed().isEmpty() ? "No scene" : scene_name.trimmed(); auto * v = static_cast<Scene3DViewportWidget *>(simple_3d_view_); v->scene_name = preview_scene_name_; refresh_info_chip(); v->update(); }
void ScenePreviewWidget::set_task_overlay_model(const TaskOverlayModel & model){ overlay_model_ = model; static_cast<Scene3DViewportWidget *>(simple_3d_view_)->task_overlay = model; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_task_overlay_visibility(bool task_route, bool pick_place_zones, bool approach_retreat, bool labels){
  auto * v = static_cast<Scene3DViewportWidget *>(simple_3d_view_);
  v->show_task_route = task_route;
  v->show_pick_place = pick_place_zones;
  v->show_approach_retreat = approach_retreat;
  if (labels) v->label_mode = LabelMode::All;
  else if (v->label_mode == LabelMode::All) v->label_mode = LabelMode::Important;
  v->update();
}
void ScenePreviewWidget::select_preview_item(const QString & id){ selected_preview_item_id_ = id; static_cast<Scene3DViewportWidget *>(simple_3d_view_)->selected_id = id; simple_3d_view_->update(); }
QString ScenePreviewWidget::selected_preview_item_id() const { return selected_preview_item_id_; }

ScenePreviewWidget::MeshPreviewMode ScenePreviewWidget::mesh_preview_mode() const { return mesh_preview_mode_; }

void ScenePreviewWidget::reload_meshes()
{
  auto * v = static_cast<Scene3DViewportWidget *>(simple_3d_view_);
  v->invalidate_mesh_cache();
  v->update();
  update();
  emit studio_log_requested("Reloaded mesh preview cache (visual-only).");
}
void ScenePreviewWidget::on_reset_view_clicked(){ static_cast<Scene3DViewportWidget *>(simple_3d_view_)->reset_view(); reset_fallback_scene_view(); }
void ScenePreviewWidget::on_fit_scene_clicked(){ auto *v = static_cast<Scene3DViewportWidget *>(simple_3d_view_); v->fit_include_overlays = false; v->fit_scene(); fit_fallback_scene_to_items(false); }
void ScenePreviewWidget::on_fit_overlays_clicked(){ auto *v = static_cast<Scene3DViewportWidget *>(simple_3d_view_); v->fit_include_overlays = true; v->fit_scene(); fit_fallback_scene_to_items(true); v->fit_include_overlays = false; }
void ScenePreviewWidget::on_focus_selected_clicked(){ static_cast<Scene3DViewportWidget *>(simple_3d_view_)->focus_selected(); }
void ScenePreviewWidget::on_clear_selection_clicked(){ selected_preview_item_id_.clear(); static_cast<Scene3DViewportWidget *>(simple_3d_view_)->selected_id.clear(); simple_3d_view_->update(); emit studio_log_requested("Cleared preview selection."); emit preview_item_selected(QString(), QStringLiteral("unknown")); }
void ScenePreviewWidget::refresh_mode_and_state()
{
  const QString mode = mode_selector_->currentText();
  const bool requested_3d = (mode == "3D Layout Preview") || (mode == "Debug Overlays");
  const bool use3d = requested_3d && preview3d_available_;
  auto * viewport = static_cast<Scene3DViewportWidget *>(simple_3d_view_);
  viewport->debug_overlays_mode = (mode == "Debug Overlays");

  if (!preview3d_available_) {
    stack_->setCurrentWidget(view2d_container_);
    fallback_banner_label_->setVisible(scene_selected_);
    const QString reason = unavailable_reason_.isEmpty() ? "initialization failed" : unavailable_reason_;
    emit studio_log_requested(QString("3D Layout Preview unavailable, using 2D Layout: %1").arg(reason));
  } else {
    fallback_banner_label_->setVisible(false);
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
    const QString role = it.role.trimmed().toLower();
    const QString category = it.category.trimmed().toLower();
    const QString mix = role + "|" + category;
    return !mix.contains("safety_zone") && !mix.contains("safety") && !mix.contains("warning_anchor") && !mix.contains("warning_badge");
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

void ScenePreviewWidget::set_camera_overlay_model(const CameraOverlayModel & model){ camera_overlay_model_ = model; auto *v=static_cast<Scene3DViewportWidget *>(simple_3d_view_); v->camera_overlay = model; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_epd_detection_overlays(const QVector<EpdDetectionOverlayModel> & detections){ epd_detections_ = detections; auto *v=static_cast<Scene3DViewportWidget *>(simple_3d_view_); v->epd_detections = detections; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_perception_overlay_visibility(bool camera_fov, bool pick_coverage, bool epd_detections, bool detection_labels){ auto *v=static_cast<Scene3DViewportWidget *>(simple_3d_view_); v->show_camera_fov=camera_fov; v->show_pick_coverage=pick_coverage; v->show_epd_detections=epd_detections; v->show_detection_labels=detection_labels; v->update(); }


void ScenePreviewWidget::set_reachability_overlay_model(const ReachabilityOverlayModel & model){ reachability_overlay_model_ = model; auto *v=static_cast<Scene3DViewportWidget *>(simple_3d_view_); v->reach_overlay = model; emit studio_log_requested(QString("reachability overlay loaded: warning count=%1").arg(model.warnings.size())); refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_collision_overlay_model(const CollisionOverlayModel & model){ collision_overlay_model_ = model; auto *v=static_cast<Scene3DViewportWidget *>(simple_3d_view_); v->collision_overlay = model; emit studio_log_requested(QString("collision preview checks complete: warning count=%1").arg(model.warnings.size())); refresh_info_chip(); simple_3d_view_->update(); }

void ScenePreviewWidget::set_label_mode(LabelMode mode){ auto *v=static_cast<Scene3DViewportWidget *>(simple_3d_view_); v->label_mode = mode; if (labels_selector_) { if (mode == LabelMode::Off) labels_selector_->setCurrentText("Off"); else if (mode == LabelMode::Important) labels_selector_->setCurrentText("Important"); else if (mode == LabelMode::Selected) labels_selector_->setCurrentText("Selected"); else labels_selector_->setCurrentText("All"); } v->update(); }

int ScenePreviewWidget::total_warning_count() const { int count = 0; for (const auto & item : preview_items_) count += item.warnings.size(); count += overlay_model_.warnings.size() + reachability_overlay_model_.warnings.size() + collision_overlay_model_.warnings.size() + camera_overlay_model_.warnings.size(); for (const auto & det : epd_detections_) count += det.warnings.size(); return count; }
bool ScenePreviewWidget::task_is_ready() const { return overlay_model_.has_intent_metadata && overlay_model_.pick_source_id != "unknown" && overlay_model_.place_target_id != "unknown"; }
void ScenePreviewWidget::refresh_info_chip() { if (!info_chip_label_) return; const QString mode = mode_selector_ ? mode_selector_->currentText() : QStringLiteral("2D Layout"); const bool requested_3d = (mode == "3D Layout Preview") || (mode == "Debug Overlays"); const QString render_mode = requested_3d && preview3d_available_ ? mode : QStringLiteral("2D Layout (Fallback)"); info_chip_label_->setText(QString("Scene: %1\nMode: %2\nItems: %3  Warn: %4  Task: %5").arg(preview_scene_name_).arg(render_mode).arg(preview_items_.size()).arg(total_warning_count()).arg(task_is_ready() ? "Ready" : "Missing")); info_chip_label_->adjustSize(); if (fallback_info_chip_proxy_) fallback_info_chip_proxy_->setPos(12.0, 12.0); }
