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
  ScenePreviewWidget::LabelMode label_mode{ScenePreviewWidget::LabelMode::SelectedOnly};
  bool show_warnings{true}, show_safety{true}, show_pick_place{true};
  bool show_reachability_heatmap{true}, show_collision_warnings{true}, show_work_envelope{true}, show_warning_labels{true};
  bool show_task_route{true}, show_approach_retreat{true};
  bool show_camera_fov{true}, show_pick_coverage{true}, show_epd_detections{true}, show_detection_labels{true};
  ScenePreviewWidget::TaskOverlayModel task_overlay;
  ScenePreviewWidget::ReachabilityOverlayModel reach_overlay;
  ScenePreviewWidget::CollisionOverlayModel collision_overlay;
  QString scene_name{"No scene"};
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
    QString tooltip;
    QString hovered_id;
    double bestd = 1e18;
    for (const auto & item : items) {
      const QPointF c = project(item.x, item.y, item.z);
      const double dist = QLineF(c, e->pos()).length();
      if (dist < bestd) { bestd = dist; hovered_id = item.id; }
    }
    if (bestd < 45.0) {
      for (const auto & item : items) {
        if (item.id != hovered_id || item.warnings.isEmpty()) continue;
        tooltip = QString("id: %1\nrole: %2\nwarnings:\n- %3")
          .arg(item.id, item.role, item.warnings.join("\n- "));
        break;
      }
    }
    setToolTip(tooltip);
  }
  void wheelEvent(QWheelEvent * e) override { zoom_ = qBound(0.4, zoom_ + (e->angleDelta().y() > 0 ? -0.1 : 0.1), 2.2); update(); }
  QPointF project(double x, double y, double z) const {
    double cy = qCos(yaw_), sy = qSin(yaw_), cp = qCos(pitch_), sp = qSin(pitch_);
    double rx = cy * x + sy * z; double rz = -sy * x + cy * z;
    double ry = cp * y - sp * rz; double zz = sp * y + cp * rz + 6.0;
    double s = 220.0 / (zz * zoom_);
    return QPointF(width()*0.5 + rx*s, height()*0.6 - ry*s);
  }
  enum class RenderLayer {
    BackgroundGrid,
    RobotReachLayer,
    SafetyZones,
    SupportSurfaces,
    BinsFixturesObjects,
    CamerasAndFov,
    PickPlaceZones,
    SelectionHighlight,
    CompactLabels,
    WarningBadges,
  };

  // Render order is intentional: base geometry first, selection highlight next,
  // then labels and warning badges on top for readability.
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.fillRect(rect(), QColor("#0b1020"));
    p.setRenderHint(QPainter::Antialiasing, true);

    drawBackgroundGrid(p);
    drawRobotReachLayer(p);
    drawSafetyZones(p);
    drawSupportSurfaces(p);
    drawBinsFixturesObjects(p);
    drawCamerasAndFov(p);
    drawPickPlaceZones(p);
    drawSelectionHighlight(p);
    drawCompactLabels(p);
    drawWarningBadges(p);
    drawInfoChip(p);
  }

  QColor itemColor(const ScenePreviewWidget::PreviewItem & it) const {
    if (it.category.contains("robot",Qt::CaseInsensitive)) return QColor("#a78bfa");
    if (it.category.contains("camera",Qt::CaseInsensitive)) return QColor("#38bdf8");
    if (it.category.contains("conveyor",Qt::CaseInsensitive)) return QColor("#06b6d4");
    if (it.category.contains("bin",Qt::CaseInsensitive) || it.role.contains("pick", Qt::CaseInsensitive)) return QColor("#34d399");
    if (it.role.contains("place", Qt::CaseInsensitive)) return QColor("#fb7185");
    return QColor("#94a3b8");
  }

  void drawItemPolygon(QPainter & p, const ScenePreviewWidget::PreviewItem & it, const QColor & color, int width) const {
    QPointF a=project(it.x,it.y,it.z), b=project(it.x+it.sx,it.y,it.z), c1=project(it.x+it.sx,it.y+it.sy,it.z), d=project(it.x,it.y+it.sy,it.z);
    p.setPen(QPen(color, width));
    QPolygonF poly;
    poly << a << b << c1 << d;
    p.drawPolygon(poly);
  }

  void drawBackgroundGrid(QPainter & p) const {
    p.setPen(QPen(QColor("#1f2937"),1));
    for (int i=-10;i<=10;++i){ p.drawLine(project(i*0.4,0,-4), project(i*0.4,0,4)); p.drawLine(project(-4,0,i*0.4), project(4,0,i*0.4)); }
  }

  void drawRobotReachLayer(QPainter & p) const {
    if (!(show_reachability_heatmap || show_work_envelope)) return;
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
    }
    if (show_reachability_heatmap) {
      p.setPen(QPen(QColor("#f59e0b"), 2, Qt::DotLine)); p.drawEllipse(center, rMin, rMin);
      p.setPen(QPen(QColor("#ef4444"), 2)); p.drawEllipse(center, rMax, rMax);
    }
  }

  void drawSafetyZones(QPainter & p) const {
    if (!show_safety) return;
    for (const auto & it : items) {
      if (!it.category.contains("safety", Qt::CaseInsensitive)) continue;
      drawItemPolygon(p, it, itemColor(it), 2);
    }
  }

  void drawSupportSurfaces(QPainter & p) const {
    auto drawBox=[&](double x,double y,double z,double sx,double sy,double sz,QColor c){
      Q_UNUSED(sz);
      QPointF a=project(x,y,z), b=project(x+sx,y,z), c1=project(x+sx,y+sy,z), d=project(x,y+sy,z);
      QPolygonF poly; poly << a << b << c1 << d;
      p.setPen(QPen(c,2)); p.drawPolygon(poly);
    };
    drawBox(-1.2,0,-1.0,2.4,0.15,1.6,QColor("#64748b"));
    drawBox(-2.4,0,-0.2,1.2,0.1,0.6,QColor("#06b6d4"));
  }

  void drawBinsFixturesObjects(QPainter & p) const {
    for (const auto & it : items) {
      if (!show_safety && it.category.contains("safety", Qt::CaseInsensitive)) continue;
      if (it.id == selected_id) continue;
      drawItemPolygon(p, it, itemColor(it), 2);
    }
  }

  void drawCamerasAndFov(QPainter & p) const {
    auto drawBox=[&](double x,double y,double z,double sx,double sy,double sz,QColor c){
      Q_UNUSED(sz);
      QPointF a=project(x,y,z), b=project(x+sx,y,z), c1=project(x+sx,y+sy,z), d=project(x,y+sy,z);
      QPolygonF poly; poly << a << b << c1 << d;
      p.setPen(QPen(c,2)); p.drawPolygon(poly);
    };
    drawBox(0.8,0.8,-1.8,0.2,0.2,0.2,QColor("#38bdf8"));

    if (!show_camera_fov) return;
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
  }

  void drawPickPlaceZones(QPainter & p) const {
    auto drawBox=[&](double x,double y,double z,double sx,double sy,double sz,QColor c){
      Q_UNUSED(sz);
      QPointF a=project(x,y,z), b=project(x+sx,y,z), c1=project(x+sx,y+sy,z), d=project(x,y+sy,z);
      QPolygonF poly; poly << a << b << c1 << d;
      p.setPen(QPen(c,2)); p.drawPolygon(poly);
    };
    drawBox(-0.8,0.15,-0.7,0.3,0.3,0.3,QColor("#34d399"));
    drawBox(1.3,0.15,-0.1,0.3,0.3,0.3,QColor("#fb7185"));
    if (!show_pick_place) return;
    p.setPen(QPen(QColor("#00d1b2"), 2, Qt::DashLine));
    p.setPen(QPen(QColor("#ff7b72"), 2, Qt::DashLine));
  }

  void drawSelectionHighlight(QPainter & p) const {
    for (const auto & it : items) {
      if (it.id != selected_id) continue;
      drawItemPolygon(p, it, QColor("#fde047"), 3);
      break;
    }
  }

  QString shortRoleLabel(const ScenePreviewWidget::PreviewItem & it) const {
    const QString category = it.category.toLower();
    const QString role = it.role.toLower();
    if (category.contains("robot")) return "Robot";
    if (category.contains("table")) return "Table";
    if (category.contains("conveyor")) return "Conveyor";
    if (category.contains("camera")) return "Camera";
    if (role.contains("pick") || category.contains("pick")) return "Pick";
    if (role.contains("place") || category.contains("place")) return "Place";
    if (category.contains("bin")) return "Bin";
    return QString();
  }

  void drawCompactLabels(QPainter & p) const {
    QVector<QRectF> used_label_rects;
    for (const auto & it : items) {
      if (!show_safety && it.category.contains("safety", Qt::CaseInsensitive)) continue;
      const bool selected = (it.id == selected_id);
      if (label_mode == ScenePreviewWidget::LabelMode::Off) continue;
      if (label_mode == ScenePreviewWidget::LabelMode::SelectedOnly && !selected) continue;
      const QString short_label = shortRoleLabel(it);
      if (short_label.isEmpty()) continue;
      const QPointF anchor = project(it.x, it.y + it.sy + 0.08, it.z);
      const QRectF label_rect = QFontMetricsF(p.font()).boundingRect(short_label).translated(anchor);
      bool intersects_existing = false;
      for (const QRectF & used : used_label_rects) {
        if (used.intersects(label_rect)) {
          intersects_existing = true;
          break;
        }
      }
      if (intersects_existing) continue;
      p.drawText(anchor, selected ? (short_label + " [selected]") : short_label);
      used_label_rects.push_back(label_rect);
    }
    if (show_work_envelope) p.drawText(project(-0.1, 0.25, -2.2), "Work Envelope (preview-only)");
    if (show_reachability_heatmap) p.drawText(project(-0.08, 0.4, -2.2), "Reachability Heatmap (approximate, preview-only)");
    if (show_pick_place) {
      p.setPen(QPen(QColor("#00d1b2"), 2, Qt::DashLine)); p.drawText(project(-0.8, 0.5, -0.7), "pick source zone");
      p.setPen(QPen(QColor("#ff7b72"), 2, Qt::DashLine)); p.drawText(project(1.3, 0.5, -0.1), "place target zone");
      if (task_overlay.reject_target_id != "unknown") { p.setPen(QPen(QColor("#f59e0b"), 2, Qt::DashLine)); p.drawText(project(1.7, 0.5, 0.3), "reject target zone"); }
    }
    if (show_task_route) { p.setPen(QPen(QColor("#38bdf8"), 2, Qt::DashDotLine)); p.drawLine(project(-0.8,0.3,-0.7), project(1.3,0.3,-0.1)); p.drawText(project(0.3, 0.45, -0.4), "Task Route"); }
    if (show_approach_retreat) { p.setPen(QPen(QColor("#22c55e"), 2)); p.drawLine(project(-0.8,0.65,-0.7), project(-0.8,1.1,-0.7)); p.drawText(project(-0.7, 1.1, -0.7), "Approach/Retreat"); }
    if (show_camera_fov) p.drawText(project(camera_overlay.x, camera_overlay.y + 0.25, camera_overlay.z), QString("%1 [%2]").arg(camera_overlay.display_name, camera_overlay.frame_id));
    if (show_epd_detections) {
      for (const auto & det : epd_detections) {
        QColor dc = det.status == "ready" ? QColor("#22c55e") : (det.status == "warning" ? QColor("#f59e0b") : QColor("#64748b"));
        p.setPen(QPen(dc,2));
        p.drawEllipse(project(det.x, det.y, det.z), 6, 6);
        if (show_detection_labels) p.drawText(project(det.x + 0.05, det.y + 0.08, det.z), QString("%1 %2").arg(det.label, det.confidence >= 0.0 ? QString("(%1)").arg(det.confidence,0,'f',2) : QString("")));
      }
    }
  }

  void drawWarningBadges(QPainter & p) const {
    if (!show_warnings) return;
    for (const auto & it : items) {
      if (it.warnings.isEmpty()) continue;
      const QPointF badge_center = project(it.x + it.sx * 0.5, it.y + it.sy + 0.08, it.z);
      p.setPen(Qt::NoPen);
      p.setBrush(QColor("#f59e0b"));
      p.drawEllipse(badge_center, 8, 8);
      p.setPen(QPen(QColor("#111827"), 2));
      p.drawText(QRectF(badge_center.x() - 8, badge_center.y() - 8, 16, 16), Qt::AlignCenter, it.warnings.size() > 1 ? QString::number(it.warnings.size()) : "!");
    }
  }
  void drawInfoChip(QPainter & p) const {
    int warning_count = 0;
    for (const auto & it : items) warning_count += it.warnings.size();
    warning_count += task_overlay.warnings.size() + reach_overlay.warnings.size() + collision_overlay.warnings.size() + camera_overlay.warnings.size();
    for (const auto & det : epd_detections) warning_count += det.warnings.size();
    const bool task_ready = task_overlay.has_intent_metadata && task_overlay.pick_source_id != "unknown" && task_overlay.place_target_id != "unknown";
    const QString chip_text = QString("Scene: %1\nItems: %2  Warn: %3  Task: %4").arg(scene_name, QString::number(items.size()), QString::number(warning_count), task_ready ? "Ready" : "Missing");
    const QRectF text_rect = QFontMetricsF(p.font()).boundingRect(QRectF(0, 0, width() * 0.6, 100), Qt::TextWordWrap, chip_text);
    const QRectF chip_rect(12, height() - text_rect.height() - 16, text_rect.width() + 16, text_rect.height() + 10);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(15, 23, 42, 210));
    p.drawRoundedRect(chip_rect, 6, 6);
    p.setPen(QColor("#e2e8f0"));
    p.drawText(chip_rect.adjusted(8, 5, -8, -5), Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap, chip_text);
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
  mode_selector_->addItems({"3D View", "2D Layout", "Debug Overlays"});
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
  error_state_label_ = new QLabel("3D View unavailable", view3d_container_); error_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(error_state_label_);
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
  refresh_info_chip();
  refresh_mode_and_state();
}
void ScenePreviewWidget::set_fallback_2d_view(QGraphicsView * view){ fallback_2d_view_ = view; if (!view2d_container_->layout()) view2d_container_->setLayout(new QVBoxLayout()); view2d_container_->layout()->addWidget(view); if (fallback_2d_view_ && fallback_2d_view_->scene() && info_chip_label_ && !fallback_info_chip_proxy_) { fallback_info_chip_proxy_ = fallback_2d_view_->scene()->addWidget(info_chip_label_); fallback_info_chip_proxy_->setZValue(10000.0); fallback_info_chip_proxy_->setPos(12.0, 12.0); } refresh_info_chip(); }
void ScenePreviewWidget::set_scene_selected(bool selected){ scene_selected_ = selected; refresh_mode_and_state(); }
void ScenePreviewWidget::set_3d_available(bool available, const QString & reason){
  preview3d_available_ = available;
  unavailable_reason_ = reason;
  if (!mode_default_initialized_) {
    mode_selector_->setCurrentText(preview3d_available_ ? "3D View" : "2D Layout");
    mode_default_initialized_ = true;
  }
  refresh_mode_and_state();
}
void ScenePreviewWidget::on_mode_changed(int){ refresh_mode_and_state(); }
void ScenePreviewWidget::set_preview_items(const QVector<PreviewItem> & items){ preview_items_ = items; static_cast<SimplePreview3DView *>(simple_3d_view_)->items = preview_items_; const bool has_selected = std::any_of(preview_items_.cbegin(), preview_items_.cend(), [this](const PreviewItem & it){ return it.id == selected_preview_item_id_; }); if (!has_selected && !selected_preview_item_id_.isEmpty()) { emit studio_log_requested(QString("Preview selection id no longer present: %1").arg(selected_preview_item_id_)); } else { static_cast<SimplePreview3DView *>(simple_3d_view_)->selected_id = selected_preview_item_id_; } emit studio_log_requested(QString("Loaded %1 preview items.").arg(preview_items_.size())); fit_fallback_scene_to_items(); refresh_info_chip(); update(); }
void ScenePreviewWidget::set_preview_scene_name(const QString & scene_name){ preview_scene_name_ = scene_name.trimmed().isEmpty() ? "No scene" : scene_name.trimmed(); auto * v = static_cast<SimplePreview3DView *>(simple_3d_view_); v->scene_name = preview_scene_name_; refresh_info_chip(); v->update(); }
void ScenePreviewWidget::set_task_overlay_model(const TaskOverlayModel & model){ overlay_model_ = model; static_cast<SimplePreview3DView *>(simple_3d_view_)->task_overlay = model; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_task_overlay_visibility(bool task_route, bool pick_place_zones, bool approach_retreat, bool labels){
  auto * v = static_cast<SimplePreview3DView *>(simple_3d_view_);
  v->show_task_route = task_route;
  v->show_pick_place = pick_place_zones;
  v->show_approach_retreat = approach_retreat;
  v->label_mode = labels ? LabelMode::All : LabelMode::Off;
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
    emit studio_log_requested(QString("3D View unavailable, using 2D Layout: %1").arg(unavailable_reason_.isEmpty() ? "initialization failed" : unavailable_reason_));
    return;
  }
  fallback_banner_label_->setVisible(false);
  bool use3d = mode_selector_->currentText() == "3D View";
  if (mode_selector_->currentText() == "Debug Overlays") use3d = false;
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

void ScenePreviewWidget::set_camera_overlay_model(const CameraOverlayModel & model){ camera_overlay_model_ = model; auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->camera_overlay = model; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_epd_detection_overlays(const QVector<EpdDetectionOverlayModel> & detections){ epd_detections_ = detections; auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->epd_detections = detections; refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_perception_overlay_visibility(bool camera_fov, bool pick_coverage, bool epd_detections, bool detection_labels){ auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->show_camera_fov=camera_fov; v->show_pick_coverage=pick_coverage; v->show_epd_detections=epd_detections; v->show_detection_labels=detection_labels; v->update(); }


void ScenePreviewWidget::set_reachability_overlay_model(const ReachabilityOverlayModel & model){ reachability_overlay_model_ = model; auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->reach_overlay = model; emit studio_log_requested(QString("reachability overlay loaded: warning count=%1").arg(model.warnings.size())); refresh_info_chip(); simple_3d_view_->update(); }
void ScenePreviewWidget::set_collision_overlay_model(const CollisionOverlayModel & model){ collision_overlay_model_ = model; auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->collision_overlay = model; emit studio_log_requested(QString("collision preview checks complete: warning count=%1").arg(model.warnings.size())); refresh_info_chip(); simple_3d_view_->update(); }

void ScenePreviewWidget::set_label_mode(LabelMode mode){ auto *v=static_cast<SimplePreview3DView *>(simple_3d_view_); v->label_mode = mode; v->update(); }

int ScenePreviewWidget::total_warning_count() const { int count = 0; for (const auto & item : preview_items_) count += item.warnings.size(); count += overlay_model_.warnings.size() + reachability_overlay_model_.warnings.size() + collision_overlay_model_.warnings.size() + camera_overlay_model_.warnings.size(); for (const auto & det : epd_detections_) count += det.warnings.size(); return count; }
bool ScenePreviewWidget::task_is_ready() const { return overlay_model_.has_intent_metadata && overlay_model_.pick_source_id != "unknown" && overlay_model_.place_target_id != "unknown"; }
void ScenePreviewWidget::refresh_info_chip() { if (!info_chip_label_) return; info_chip_label_->setText(QString("Scene: %1\nItems: %2  Warn: %3  Task: %4").arg(preview_scene_name_).arg(preview_items_.size()).arg(total_warning_count()).arg(task_is_ready() ? "Ready" : "Missing")); info_chip_label_->adjustSize(); if (fallback_info_chip_proxy_) fallback_info_chip_proxy_->setPos(12.0, 12.0); }
