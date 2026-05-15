#include "scene_preview_widget.h"

#include <QComboBox>
#include <QFrame>
#include <QGraphicsView>
#include <QHBoxLayout>
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
  void reset_view() { yaw_ = -0.9; pitch_ = 0.7; zoom_ = 1.0; update(); }
  void fit_scene() { zoom_ = 1.0; update(); }
protected:
  void mousePressEvent(QMouseEvent * e) override { last_ = e->pos(); }
  void mouseMoveEvent(QMouseEvent * e) override {
    auto d = e->pos() - last_; last_ = e->pos();
    if (e->buttons() & Qt::LeftButton) { yaw_ += d.x() * 0.01; pitch_ = qBound(-1.4, pitch_ + d.y() * 0.01, 1.4); update(); }
  }
  void wheelEvent(QWheelEvent * e) override { zoom_ = qBound(0.4, zoom_ + (e->angleDelta().y() > 0 ? -0.1 : 0.1), 2.2); update(); }
  void paintEvent(QPaintEvent *) override {
    QPainter p(this); p.fillRect(rect(), QColor("#0b1020")); p.setRenderHint(QPainter::Antialiasing, true);
    auto project = [&](double x, double y, double z){
      double cy = qCos(yaw_), sy = qSin(yaw_), cp = qCos(pitch_), sp = qSin(pitch_);
      double rx = cy * x + sy * z; double rz = -sy * x + cy * z;
      double ry = cp * y - sp * rz; double zz = sp * y + cp * rz + 6.0;
      double s = 220.0 / (zz * zoom_);
      return QPointF(width()*0.5 + rx*s, height()*0.6 - ry*s);
    };
    p.setPen(QPen(QColor("#1f2937"),1));
    for (int i=-10;i<=10;++i){ p.drawLine(project(i*0.4,0,-4), project(i*0.4,0,4)); p.drawLine(project(-4,0,i*0.4), project(4,0,i*0.4)); }
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
  overlays_selector_ = new QComboBox(this); overlays_selector_->addItems({"Overlays", "Labels", "Warnings"}); controls->addWidget(overlays_selector_);
  controls->addStretch(1);
  root->addLayout(controls);

  stack_ = new QStackedWidget(this);
  view3d_container_ = new QWidget(this); auto * v3 = new QVBoxLayout(view3d_container_);
  simple_3d_view_ = new SimplePreview3DView(view3d_container_); v3->addWidget(simple_3d_view_);
  empty_state_label_ = new QLabel("No scene selected\nOpen a scene or create a new cell to preview it.", view3d_container_);
  empty_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(empty_state_label_);
  error_state_label_ = new QLabel("3D preview unavailable", view3d_container_); error_state_label_->setAlignment(Qt::AlignCenter); v3->addWidget(error_state_label_);

  view2d_container_ = new QWidget(this); view2d_container_->setLayout(new QVBoxLayout());
  stack_->addWidget(view3d_container_);
  stack_->addWidget(view2d_container_);
  root->addWidget(stack_, 1);

  connect(mode_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, &ScenePreviewWidget::on_mode_changed);
  connect(reset_view_button_, &QPushButton::clicked, this, [this]{ static_cast<SimplePreview3DView *>(simple_3d_view_)->reset_view(); });
  connect(fit_scene_button_, &QPushButton::clicked, this, [this]{ static_cast<SimplePreview3DView *>(simple_3d_view_)->fit_scene(); });
  refresh_mode_and_state();
}

void ScenePreviewWidget::set_fallback_2d_view(QGraphicsView * view)
{
  fallback_2d_view_ = view;
  if (!view2d_container_->layout()) view2d_container_->setLayout(new QVBoxLayout());
  view2d_container_->layout()->addWidget(view);
}

void ScenePreviewWidget::set_scene_selected(bool selected){ scene_selected_ = selected; refresh_mode_and_state(); }
void ScenePreviewWidget::set_3d_available(bool available, const QString & reason){ preview3d_available_ = available; unavailable_reason_ = reason; refresh_mode_and_state(); }
void ScenePreviewWidget::on_mode_changed(int){ refresh_mode_and_state(); }

void ScenePreviewWidget::refresh_mode_and_state()
{
  if (!preview3d_available_) {
    mode_selector_->setCurrentText("2D Layout");
    stack_->setCurrentWidget(view2d_container_);
    emit studio_log_requested(QString("3D preview unavailable: %1").arg(unavailable_reason_.isEmpty() ? "initialization failed" : unavailable_reason_));
    return;
  }
  bool use3d = mode_selector_->currentText() == "3D Preview";
  stack_->setCurrentWidget(use3d ? view3d_container_ : view2d_container_);
  empty_state_label_->setVisible(use3d && !scene_selected_);
  simple_3d_view_->setVisible(use3d && scene_selected_);
  error_state_label_->setVisible(false);
}
