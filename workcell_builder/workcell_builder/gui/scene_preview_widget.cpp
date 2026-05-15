#include "scene_preview_widget.h"

#include <QComboBox>
#include <QFrame>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <functional>
#include <QLabel>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QPainter>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QtMath>

namespace {
class SimplePreview3DView : public QOpenGLWidget {
public:
  explicit SimplePreview3DView(QWidget * parent=nullptr) : QOpenGLWidget(parent) { setMinimumHeight(420); }
  QVector<ScenePreviewWidget::PreviewItem> items;
  QString selected_id;
  bool show_labels{true}, show_warnings{true}, show_safety{true}, show_pick_place{true};
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
      p.setPen(QPen(selected ? QColor("#fde047") : c, selected ? 3 : 2)); p.drawPolygon(QPolygonF{a,b,c1,d});
      if (show_labels || selected) p.drawText(project(it.x,it.y+it.sy+0.08,it.z), selected ? (it.display_name + " [selected]") : it.display_name);
      if (show_warnings && it.status == "warning") p.drawText(project(it.x,it.y+it.sy+0.2,it.z), "metadata incomplete");
      if (show_pick_place && (it.role.contains("pick") || it.role.contains("place"))) p.drawEllipse(project(it.x,it.y+it.sy+0.1,it.z), 4, 4);
    }
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
  overlays_selector_ = new QComboBox(this); overlays_selector_->addItems({"Overlays", "Labels", "Safety Zones", "Pick/Place Arrows", "Warnings", "Focus Selected", "Fit Selected", "Clear Selection"}); controls->addWidget(overlays_selector_);
  controls->addStretch(1);
  root->addLayout(controls);
  stack_ = new QStackedWidget(this);
  view3d_container_ = new QWidget(this); auto * v3 = new QVBoxLayout(view3d_container_);
  simple_3d_view_ = new SimplePreview3DView(view3d_container_); v3->addWidget(simple_3d_view_);
  empty_state_label_ = new QLabel("No scene selected\nOpen a scene or create a new cell to preview it.", view3d_container_);
  empty_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(empty_state_label_);
  error_state_label_ = new QLabel("3D preview unavailable", view3d_container_); error_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(error_state_label_);
  view2d_container_ = new QWidget(this); view2d_container_->setLayout(new QVBoxLayout());
  stack_->addWidget(view3d_container_); stack_->addWidget(view2d_container_); root->addWidget(stack_, 1);
  connect(mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ScenePreviewWidget::on_mode_changed);
  connect(reset_view_button_, &QPushButton::clicked, this, &ScenePreviewWidget::on_reset_view_clicked);
  connect(fit_scene_button_, &QPushButton::clicked, this, &ScenePreviewWidget::on_fit_scene_clicked);
  connect(overlays_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int){
    const QString choice = overlays_selector_->currentText();
    if (choice == "Focus Selected") on_focus_selected_clicked();
    else if (choice == "Fit Selected") on_fit_scene_clicked();
    else if (choice == "Clear Selection") on_clear_selection_clicked();
  });
  static_cast<SimplePreview3DView *>(simple_3d_view_)->select_cb = [this](const QString & id){ select_preview_item(id); emit preview_item_selected(id); emit studio_log_requested(QString("Selected preview item: %1").arg(id)); };
  refresh_mode_and_state();
}
void ScenePreviewWidget::set_fallback_2d_view(QGraphicsView * view){ fallback_2d_view_ = view; if (!view2d_container_->layout()) view2d_container_->setLayout(new QVBoxLayout()); view2d_container_->layout()->addWidget(view); }
void ScenePreviewWidget::set_scene_selected(bool selected){ scene_selected_ = selected; refresh_mode_and_state(); }
void ScenePreviewWidget::set_3d_available(bool available, const QString & reason){ preview3d_available_ = available; unavailable_reason_ = reason; refresh_mode_and_state(); }
void ScenePreviewWidget::on_mode_changed(int){ refresh_mode_and_state(); }
void ScenePreviewWidget::set_preview_items(const QVector<PreviewItem> & items){ preview_items_ = items; static_cast<SimplePreview3DView *>(simple_3d_view_)->items = preview_items_; emit studio_log_requested(QString("Loaded %1 preview items.").arg(preview_items_.size())); update(); }
void ScenePreviewWidget::select_preview_item(const QString & id){ selected_preview_item_id_ = id; static_cast<SimplePreview3DView *>(simple_3d_view_)->selected_id = id; simple_3d_view_->update(); }
QString ScenePreviewWidget::selected_preview_item_id() const { return selected_preview_item_id_; }
void ScenePreviewWidget::on_reset_view_clicked(){ static_cast<SimplePreview3DView *>(simple_3d_view_)->reset_view(); }
void ScenePreviewWidget::on_fit_scene_clicked(){ static_cast<SimplePreview3DView *>(simple_3d_view_)->fit_scene(); }
void ScenePreviewWidget::on_focus_selected_clicked(){ static_cast<SimplePreview3DView *>(simple_3d_view_)->focus_selected(); }
void ScenePreviewWidget::on_clear_selection_clicked(){ selected_preview_item_id_.clear(); static_cast<SimplePreview3DView *>(simple_3d_view_)->selected_id.clear(); simple_3d_view_->update(); emit studio_log_requested("Cleared preview selection."); emit preview_item_selected(QString()); }
void ScenePreviewWidget::refresh_mode_and_state()
{
  if (!preview3d_available_) {
    mode_selector_->setCurrentText("2D Layout");
    stack_->setCurrentWidget(view2d_container_);
    emit studio_log_requested(QString("3D preview fallback to 2D: %1").arg(unavailable_reason_.isEmpty() ? "initialization failed" : unavailable_reason_));
    return;
  }
  bool use3d = mode_selector_->currentText() == "3D Preview";
  stack_->setCurrentWidget(use3d ? view3d_container_ : view2d_container_);
  empty_state_label_->setVisible(use3d && !scene_selected_);
  simple_3d_view_->setVisible(use3d && scene_selected_);
  error_state_label_->setVisible(false);
}
