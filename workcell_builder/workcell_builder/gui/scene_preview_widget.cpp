#include "scene_preview_widget.h"

#include <QComboBox>
#include <QFrame>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QHBoxLayout>
#include <functional>
#include <QLabel>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QPainter>
#include <QPushButton>
#include <QStackedWidget>
#include <QVector3D>
#include <QVBoxLayout>
#include <QtMath>

namespace {
class SimplePreview3DView : public QOpenGLWidget {
public:
  explicit SimplePreview3DView(QWidget * parent=nullptr) : QOpenGLWidget(parent) { setMinimumHeight(420); }
  QVector<ScenePreviewWidget::PreviewItem> items;
  QString selected_id;
  bool show_labels{true}, show_warnings{true}, show_safety{true}, show_pick_place{true};
  bool show_reachability_heatmap{true}, show_collision_warnings{true}, show_work_envelope{true}, show_warning_labels{true};
  bool show_task_route{true}, show_approach_retreat{true};
  bool show_camera_fov{true}, show_pick_coverage{true}, show_epd_detections{true}, show_detection_labels{true};
  ScenePreviewWidget::TaskOverlayModel task_overlay;
  ScenePreviewWidget::ReachabilityOverlayModel reach_overlay;
  ScenePreviewWidget::CollisionOverlayModel collision_overlay;
  ScenePreviewWidget::CameraOverlayModel camera_overlay;
  QVector<ScenePreviewWidget::EpdDetectionOverlayModel> epd_detections;
  std::function<void(const QString&)> select_cb;
  void reset_view() { yaw_ = -0.9; pitch_ = 0.7; zoom_ = 1.0; update(); }
  void fit_scene() { zoom_ = 1.0; update(); }
  void focus_selected() { if (!selected_id.isEmpty()) zoom_ = 0.85; update(); }
protected:
  void mousePressEvent(QMouseEvent * e) override {
    last_ = e->pos();
    if (e->button() != Qt::LeftButton) return;
    QString best; double bestd = 1e18;
    for (const auto & item : items) {
      if (!item.selectable) continue;
      const QPointF c = project(item.x, item.y, item.z);
      const double d = QLineF(c, e->pos()).length();
      if (d < bestd) { bestd = d; best = item.id; }
    }
    if (!best.isEmpty() && bestd < 50.0 && select_cb) select_cb(best);
  }
  void mouseMoveEvent(QMouseEvent * e) override {
    auto d = e->pos() - last_; last_ = e->pos();
    if (e->buttons() & Qt::LeftButton) { yaw_ += d.x() * 0.01; pitch_ = qBound(-1.4, pitch_ + d.y() * 0.01, 1.4); update(); }
  }
  void wheelEvent(QWheelEvent * e) override { zoom_ = qBound(0.4, zoom_ + (e->angleDelta().y() > 0 ? -0.1 : 0.1), 2.2); update(); }
  QPointF project(double x, double y, double z) const {
    double cy = qCos(yaw_), sy = qSin(yaw_), cp = qCos(pitch_), sp = qSin(pitch_);
    double rx = cy * x + sy * z; double rz = -sy * x + cy * z;
    double ry = cp * y - sp * rz; double zz = sp * y + cp * rz + 6.0;
    double s = 220.0 / (zz * zoom_);
    return QPointF(width()*0.5 + rx*s, height()*0.6 - ry*s);
  }
  void paintEvent(QPaintEvent *) override {
    QPainter p(this); p.fillRect(rect(), QColor("#0b1020")); p.setRenderHint(QPainter::Antialiasing, true);
    p.setPen(QPen(QColor("#1f2937"),1));
    for (int i=-10;i<=10;++i){ p.drawLine(project(i*0.4,0,-4), project(i*0.4,0,4)); p.drawLine(project(-4,0,i*0.4), project(4,0,i*0.4)); }
    for (const auto & it : items) {
      if (!show_safety && it.category.contains("safety", Qt::CaseInsensitive)) continue;
      const bool selected = it.id == selected_id;
      QColor c = QColor("#94a3b8");
      if (it.category.contains("robot",Qt::CaseInsensitive)) c=QColor("#a78bfa");
      else if (it.category.contains("camera",Qt::CaseInsensitive)) c=QColor("#38bdf8");
      else if (it.category.contains("conveyor",Qt::CaseInsensitive)) c=QColor("#06b6d4");
      else if (it.category.contains("bin",Qt::CaseInsensitive) || it.role.contains("pick", Qt::CaseInsensitive)) c=QColor("#34d399");
      else if (it.role.contains("place", Qt::CaseInsensitive)) c=QColor("#fb7185");
      QPointF a=project(it.x,it.y,it.z), b=project(it.x+it.sx,it.y,it.z), c1=project(it.x+it.sx,it.y+it.sy,it.z), d=project(it.x,it.y+it.sy,it.z);
      p.setPen(QPen(selected ? QColor("#fde047") : c, selected ? 3 : 2));
      QPolygonF poly;  // Qt5-compatible polygon construction (initializer-list ctor is not available).
      poly << a << b << c1 << d;
      p.drawPolygon(poly);
      if (show_labels || selected) p.drawText(project(it.x,it.y+it.sy+0.08,it.z), selected ? (it.display_name + " [selected]") : it.display_name);
      if (show_warnings && it.status == "warning") p.drawText(project(it.x,it.y+it.sy+0.2,it.z), "metadata incomplete");
      if (show_pick_place && (it.role.contains("pick") || it.role.contains("place"))) p.drawEllipse(project(it.x,it.y+it.sy+0.1,it.z), 4, 4);
    }

    if (show_reachability_heatmap || show_work_envelope) {
      const double base_x = -0.2, base_y = 0.0, base_z = -2.2;
      QPointF center = project(base_x, base_y + 0.05, base_z);
      const QPointF edgeMin = project(base_x + reach_overlay.approximate_reach_min_m, base_y + 0.05, base_z);
      const QPointF edgePref = project(base_x + reach_overlay.preferred_work_zone_radius_m, base_y + 0.05, base_z);
      const QPointF edgeMax = project(base_x + reach_overlay.approximate_reach_max_m, base_y + 0.05, base_z);
      const double rMin = QLineF(center, edgeMin).length();
      const double rPref = QLineF(center, edgePref).length();
      const double rMax = QLineF(center, edgeMax).length();
      if (show_work_envelope) {
        p.setPen(QPen(QColor("#22c55e"), 2, Qt::DashLine));
        p.drawEllipse(center, rPref, rPref);
        p.drawText(project(base_x + 0.1, base_y + 0.25, base_z), "Work Envelope (preview-only)");
      }
      if (show_reachability_heatmap) {
        p.setPen(QPen(QColor("#f59e0b"), 2, Qt::DotLine)); p.drawEllipse(center, rMin, rMin);
        p.setPen(QPen(QColor("#ef4444"), 2)); p.drawEllipse(center, rMax, rMax);
        p.drawText(project(base_x + 0.12, base_y + 0.4, base_z), "Reachability Heatmap (approximate, preview-only)");
      }
    }

    if (show_pick_place) {
      p.setPen(QPen(QColor("#00d1b2"), 2, Qt::DashLine)); p.drawText(project(-0.8, 0.5, -0.7), "pick source zone");
      p.setPen(QPen(QColor("#ff7b72"), 2, Qt::DashLine)); p.drawText(project(1.3, 0.5, -0.1), "place target zone");
      if (task_overlay.reject_target_id != "unknown") { p.setPen(QPen(QColor("#f59e0b"), 2, Qt::DashLine)); p.drawText(project(1.7, 0.5, 0.3), "reject target zone"); }
    }
    if (show_task_route) {
      p.setPen(QPen(QColor("#38bdf8"), 2, Qt::DashDotLine));
      p.drawLine(project(-0.8,0.3,-0.7), project(1.3,0.3,-0.1));
      p.drawText(project(0.3, 0.45, -0.4), "Task Route");
      if (task_overlay.reject_target_id != "unknown") p.drawLine(project(-0.8,0.32,-0.7), project(1.8,0.32,0.3));
    }
    if (show_approach_retreat) {
      p.setPen(QPen(QColor("#22c55e"), 2)); p.drawLine(project(-0.8,0.65,-0.7), project(-0.8,1.1,-0.7));
      p.drawText(project(-0.7, 1.1, -0.7), "Approach/Retreat");
      p.setPen(QPen(QColor("#ef4444"), 2)); p.drawLine(project(-0.7,1.05,-0.7), project(-0.7,0.72,-0.7));
      p.drawText(project(-0.55, 1.18, -0.7), QString("grasp=%1").arg(task_overlay.grasp_strategy));
    }
    if (!task_overlay.has_intent_metadata && show_warnings) p.drawText(project(-2.2, 1.2, -0.8), "Task overlay unavailable: missing task intent");

    if (show_camera_fov) {
      const QPointF c0 = project(camera_overlay.x, camera_overlay.y, camera_overlay.z);
      const double h = qDegreesToRadians(camera_overlay.horizontal_fov_deg * 0.5);
      const double v = qDegreesToRadians(camera_overlay.vertical_fov_deg * 0.5);
      const double n = qMax(0.05, camera_overlay.range_min_m);
      const double f = qMax(n + 0.05, camera_overlay.range_max_m);
      auto rayPoint=[&](double r,double ah,double av){ return QVector3D(camera_overlay.x + r, camera_overlay.y + qTan(av)*r, camera_overlay.z + qTan(ah)*r); };
      QVector3D nfl=rayPoint(n,-h,-v), nfr=rayPoint(n,h,-v), nbl=rayPoint(n,-h,v), nbr=rayPoint(n,h,v);
      QVector3D ffl=rayPoint(f,-h,-v), ffr=rayPoint(f,h,-v), fbl=rayPoint(f,-h,v), fbr=rayPoint(f,h,v);
      auto pp=[&](const QVector3D &v3){ return project(v3.x(), v3.y(), v3.z()); };
      QColor camC = camera_overlay.status == "ready" ? QColor("#22c55e") : (camera_overlay.status == "warning" ? QColor("#f59e0b") : QColor("#94a3b8"));
      p.setPen(QPen(camC,2));
      p.drawLine(c0, pp(ffl)); p.drawLine(c0, pp(ffr)); p.drawLine(c0, pp(fbl)); p.drawLine(c0, pp(fbr));
      QPolygonF nearP; nearP << pp(nfl) << pp(nfr) << pp(nbr) << pp(nbl); p.drawPolygon(nearP);
      QPolygonF farP; farP << pp(ffl) << pp(ffr) << pp(fbr) << pp(fbl); p.drawPolygon(farP);
      p.drawLine(c0, project(camera_overlay.x + f, camera_overlay.y, camera_overlay.z));
      p.drawText(project(camera_overlay.x, camera_overlay.y + 0.25, camera_overlay.z), QString("%1 [%2]").arg(camera_overlay.display_name, camera_overlay.frame_id));
    }
    if (show_pick_coverage && show_warnings) {
      for (int wi = 0; wi < camera_overlay.warnings.size(); ++wi) p.drawText(project(-2.2, 1.0 - 0.16*wi, -0.7), camera_overlay.warnings[wi]);
    }
    if (show_epd_detections) {
      for (const auto & det : epd_detections) {
        QColor dc = det.status == "ready" ? QColor("#22c55e") : (det.status == "warning" ? QColor("#f59e0b") : QColor("#64748b"));
        p.setPen(QPen(dc,2));
        p.drawEllipse(project(det.x, det.y, det.z), 6, 6);
        if (show_detection_labels) p.drawText(project(det.x + 0.05, det.y + 0.08, det.z), QString("%1 %2").arg(det.label, det.confidence >= 0.0 ? QString("(%1)").arg(det.confidence,0,'f',2) : QString("")));
      }
    }

    auto drawBox=[&](double x,double y,double z,double sx,double sy,double sz,QColor c,const QString &name){
      QPointF a=project(x,y,z), b=project(x+sx,y,z), c1=project(x+sx,y+sy,z), d=project(x,y+sy,z);
      Q_UNUSED(sz);
      QPolygonF poly;
      poly << a << b << c1 << d;
      p.setPen(QPen(c,2));
      p.drawPolygon(poly);
      p.drawText(project(x,y+sy+0.1,z), name);
    };
    p.setPen(QPen(QColor("#ef4444"),2)); p.drawLine(project(0,0,0), project(1.2,0,0)); p.drawText(project(1.25,0,0), "X");
    p.setPen(QPen(QColor("#22c55e"),2)); p.drawLine(project(0,0,0), project(0,1.2,0)); p.drawText(project(0,1.3,0), "Y");
    p.setPen(QPen(QColor("#3b82f6"),2)); p.drawLine(project(0,0,0), project(0,0,1.2)); p.drawText(project(0,0,1.3), "Z");
    drawBox(-1.2,0,-1.0,2.4,0.15,1.6,QColor("#64748b"),"table");
    drawBox(1.6,0,-0.8,0.8,0.8,0.8,QColor("#f59e0b"),"bin");
    drawBox(-2.4,0,-0.2,1.2,0.1,0.6,QColor("#06b6d4"),"conveyor");
    drawBox(-0.2,0,-2.2,0.5,0.8,0.5,QColor("#a78bfa"),"robot base");
    drawBox(0.8,0.8,-1.8,0.2,0.2,0.2,QColor("#38bdf8"),"camera");
    drawBox(-0.8,0.15,-0.7,0.3,0.3,0.3,QColor("#34d399"),"pick src");
    drawBox(1.3,0.15,-0.1,0.3,0.3,0.3,QColor("#fb7185"),"place tgt");
  }
private:
  QPoint last_;
  double yaw_{-0.9}, pitch_{0.7}, zoom_{1.0};
};
}

ScenePreviewWidget::ScenePreviewWidget(QWidget * parent) : QWidget(parent)
{
  auto * root = new QVBoxLayout(this);
  auto * controls = new QHBoxLayout();
  controls->addWidget(new QLabel("View mode", this));
  mode_selector_ = new QComboBox(this);
  mode_selector_->addItems({"3D Preview", "2D Layout"});
  controls->addWidget(mode_selector_);
  reset_view_button_ = new QPushButton("Reset View", this); controls->addWidget(reset_view_button_);
  fit_scene_button_ = new QPushButton("Fit Scene", this); controls->addWidget(fit_scene_button_);
  overlays_selector_ = new QComboBox(this); overlays_selector_->addItems({"Overlays", "Reachability Heatmap", "Collision Warnings", "Safety Zones", "Work Envelope", "Warning Labels", "Labels", "Pick/Place Zones", "Task Route", "Approach/Retreat", "Camera FOV", "Pick Coverage", "EPD Detections", "Detection Labels", "Warnings", "Focus Selected", "Fit Selected", "Clear Selection"}); controls->addWidget(overlays_selector_);
  controls->addStretch(1);
  root->addLayout(controls);
  stack_ = new QStackedWidget(this);
  view3d_container_ = new QWidget(this); auto * v3 = new QVBoxLayout(view3d_container_);
  simple_3d_view_ = new SimplePreview3DView(view3d_container_); v3->addWidget(simple_3d_view_);
  empty_state_label_ = new QLabel("No scene selected\nOpen a scene or create a new cell to preview it.", view3d_container_);
  empty_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(empty_state_label_);
  error_state_label_ = new QLabel("3D preview unavailable", view3d_container_); error_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(error_state_label_);
  view2d_container_ = new QWidget(this); view2d_container_->setLayout(new QVBoxLayout());
  fallback_banner_label_ = new QLabel("2D fallback preview active", view2d_container_);
  fallback_banner_label_->setAlignment(Qt::AlignCenter);
  fallback_banner_label_->setVisible(false);
  view2d_container_->layout()->addWidget(fallback_banner_label_);
  stack_->addWidget(view3d_container_); stack_->addWidget(view2d_container_); root->addWidget(stack_, 1);
  connect(mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ScenePreviewWidget::on_mode_changed);
  connect(reset_view_button_, &QPushButton::clicked, this, &ScenePreviewWidget::on_reset_view_clicked);
  connect(fit_scene_button_, &QPushButton::clicked, this, &ScenePreviewWidget::on_fit_scene_clicked);
  connect(overlays_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    auto *v = static_cast<SimplePreview3DView *>(simple_3d_view_);
    const QString choice = overlays_selector_->currentText();
    if (choice == "Reachability Heatmap") v->show_reachability_heatmap = !v->show_reachability_heatmap;
    else if (choice == "Collision Warnings") v->show_collision_warnings = !v->show_collision_warnings;
    else if (choice == "Safety Zones") v->show_safety = !v->show_safety;
    else if (choice == "Work Envelope") v->show_work_envelope = !v->show_work_envelope;
    else if (choice == "Warning Labels") v->show_warning_labels = !v->show_warning_labels;
    else if (choice == "Focus Selected") on_focus_selected_clicked();
    else if (choice == "Fit Selected") on_fit_scene_clicked();
    else if (choice == "Clear Selection") on_clear_selection_clicked();
    v->update();
  });
  static_cast<SimplePreview3DView *>(simple_3d_view_)->select_cb = [this](const QString & id){ select_preview_item(id); emit preview_item_selected(id); emit studio_log_requested(QString("Selected preview item: %1").arg(id)); };
  refresh_mode_and_state();
}
void ScenePreviewWidget::set_fallback_2d_view(QGraphicsView * view){ fallback_2d_view_ = view; if (!view2d_container_->layout()) view2d_container_->setLayout(new QVBoxLayout()); view2d_container_->layout()->addWidget(view); }
void ScenePreviewWidget::set_scene_selected(bool selected){ scene_selected_ = selected; refresh_mode_and_state(); }
void ScenePreviewWidget::set_3d_available(bool available, const QString & reason){ preview3d_available_ = available; unavailable_reason_ = reason; refresh_mode_and_state(); }
void ScenePreviewWidget::on_mode_changed(int){ refresh_mode_and_state(); }
void ScenePreviewWidget::set_preview_items(const QVector<PreviewItem> & items){ preview_items_ = items; static_cast<SimplePreview3DView *>(simple_3d_view_)->items = preview_items_; emit studio_log_requested(QString("Loaded %1 preview items.").arg(preview_items_.size())); fit_fallback_scene_to_items(); update(); }
void ScenePreviewWidget::set_task_overlay_model(const TaskOverlayModel & model){ overlay_model_ = model; static_cast<SimplePreview3DView *>(simple_3d_view_)->task_overlay = model; simple_3d_view_->update(); }
void ScenePreviewWidget::set_task_overlay_visibility(bool task_route, bool pick_place_zones, bool approach_retreat, bool labels){
  auto * v = static_cast<SimplePreview3DView *>(simple_3d_view_);
  v->show_task_route = task_route;
  v->show_pick_place = pick_place_zones;
  v->show_approach_retreat = approach_retreat;
  v->show_labels = labels;
  v->update();
}
void ScenePreviewWidget::select_preview_item(const QString & id){ selected_preview_item_id_ = id; static_cast<SimplePreview3DView *>(simple_3d_view_)->selected_id = id; simple_3d_view_->update(); }
QString ScenePreviewWidget::selected_preview_item_id() const { return selected_preview_item_id_; }
void ScenePreviewWidget::on_reset_view_clicked(){ static_cast<SimplePreview3DView *>(simple_3d_view_)->reset_view(); reset_fallback_scene_view(); }
void ScenePreviewWidget::on_fit_scene_clicked(){ static_cast<SimplePreview3DView *>(simple_3d_view_)->fit_scene(); fit_fallback_scene_to_items(); }
void ScenePreviewWidget::on_focus_selected_clicked(){ static_cast<SimplePreview3DView *>(simple_3d_view_)->focus_selected(); }
void ScenePreviewWidget::on_clear_selection_clicked(){ selected_preview_item_id_.clear(); static_cast<SimplePreview3DView *>(simple_3d_view_)->selected_id.clear(); simple_3d_view_->update(); emit studio_log_requested("Cleared preview selection."); emit preview_item_selected(QString()); }
void ScenePreviewWidget::refresh_mode_and_state()
{
  if (!preview3d_available_) {
    mode_selector_->setCurrentText("2D Layout");
    stack_->setCurrentWidget(view2d_container_);
    fallback_banner_label_->setVisible(scene_selected_);
    emit studio_log_requested(QString("3D preview fallback to 2D: %1").arg(unavailable_reason_.isEmpty() ? "initialization failed" : unavailable_reason_));
    return;
  }
  fallback_banner_label_->setVisible(false);
  bool use3d = mode_selector_->currentText() == "3D Preview";
  stack_->setCurrentWidget(use3d ? view3d_container_ : view2d_container_);
  const bool has_preview_items = !preview_items_.isEmpty();
  empty_state_label_->setVisible(use3d && !scene_selected_);
  simple_3d_view_->setVisible(use3d && scene_selected_);
  const bool rendering_failed = scene_selected_ && has_preview_items && !simple_3d_view_->isVisible() && !preview3d_available_;
  error_state_label_->setText(QString("Scene selected but preview rendering failed. Loaded %1 preview items. Check fallback 2D canvas.").arg(preview_items_.size()));
  error_state_label_->setVisible(rendering_failed);
}
QRectF ScenePreviewWidget::rendered_items_bounds_2d() const
{
  if (!fallback_2d_view_ || !fallback_2d_view_->scene()) return QRectF();
  QRectF bounds;
  const auto items = fallback_2d_view_->scene()->items();
  for (QGraphicsItem * item : items) {
    if (!item || !item->isVisible()) continue;
    bounds = bounds.isNull() ? item->sceneBoundingRect() : bounds.united(item->sceneBoundingRect());
  }
  return bounds;
}
void ScenePreviewWidget::fit_fallback_scene_to_items()
{
  if (!fallback_2d_view_) return;
  const QRectF bounds = rendered_items_bounds_2d();
  if (bounds.isValid() && !bounds.isEmpty()) fallback_2d_view_->fitInView(bounds.adjusted(-20, -20, 20, 20), Qt::KeepAspectRatio);
}
void ScenePreviewWidget::reset_fallback_scene_view()
{
  if (!fallback_2d_view_) return;
  fallback_2d_view_->resetTransform();
  fit_fallback_scene_to_items();
}

void ScenePreviewWidget::set_camera_overlay_model(const CameraOverlayModel & model){ camera_overlay_model_ = model; auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->camera_overlay = model; simple_3d_view_->update(); }
void ScenePreviewWidget::set_epd_detection_overlays(const QVector<EpdDetectionOverlayModel> & detections){ epd_detections_ = detections; auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->epd_detections = detections; simple_3d_view_->update(); }
void ScenePreviewWidget::set_perception_overlay_visibility(bool camera_fov, bool pick_coverage, bool epd_detections, bool detection_labels){ auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->show_camera_fov=camera_fov; v->show_pick_coverage=pick_coverage; v->show_epd_detections=epd_detections; v->show_detection_labels=detection_labels; v->update(); }


void ScenePreviewWidget::set_reachability_overlay_model(const ReachabilityOverlayModel & model){ reachability_overlay_model_ = model; auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->reach_overlay = model; emit studio_log_requested(QString("reachability overlay loaded: warning count=%1").arg(model.warnings.size())); simple_3d_view_->update(); }
void ScenePreviewWidget::set_collision_overlay_model(const CollisionOverlayModel & model){ collision_overlay_model_ = model; auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->collision_overlay = model; emit studio_log_requested(QString("collision preview checks complete: warning count=%1").arg(model.warnings.size())); simple_3d_view_->update(); }
