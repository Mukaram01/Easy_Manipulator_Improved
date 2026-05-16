#pragma once

#include "scene_preview_widget.h"

#include <QOpenGLFunctions>
#include <QOpenGLWidget>

#include <functional>

class QVector3D;

class Scene3DViewportWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
  Q_OBJECT
public:
  explicit Scene3DViewportWidget(QWidget * parent = nullptr);

  QVector<ScenePreviewWidget::PreviewItem> items;
  QString selected_id;
  ScenePreviewWidget::LabelMode label_mode{ ScenePreviewWidget::LabelMode::SelectedOnly };
  bool show_warnings{ true }, show_safety{ true }, show_pick_place{ true };
  bool show_reachability_heatmap{ true }, show_collision_warnings{ true }, show_work_envelope{ true }, show_warning_labels{ true };
  bool show_task_route{ true }, show_approach_retreat{ true };
  bool show_camera_fov{ true }, show_pick_coverage{ true }, show_epd_detections{ true }, show_detection_labels{ true };
  ScenePreviewWidget::TaskOverlayModel task_overlay;
  ScenePreviewWidget::ReachabilityOverlayModel reach_overlay;
  ScenePreviewWidget::CollisionOverlayModel collision_overlay;
  QString scene_name{ "No scene" };
  ScenePreviewWidget::CameraOverlayModel camera_overlay;
  QVector<ScenePreviewWidget::EpdDetectionOverlayModel> epd_detections;
  std::function<void(const QString &)> select_cb;

  void reset_view();
  void fit_scene();
  void focus_selected();
  void set_top_view();
  void set_front_view();
  void set_side_view();
  void set_isometric_view();

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent * e) override;
  void mouseMoveEvent(QMouseEvent * e) override;
  void wheelEvent(QWheelEvent * e) override;

private:
  QPointF project_to_screen(double x, double y, double z) const;
  void draw_box(double cx, double cy, double cz, double sx, double sy, double sz, const QColor & color, bool translucent = false);
  void draw_cylinder(double cx, double cy, double cz, double radius, double height, const QColor & color, bool translucent = false);
  void draw_frustum(const QColor & color, bool translucent = true);
  QPoint last_;
  QVector3D orbit_offset_{ 0.0f, 0.0f, 0.0f };
  double yaw_{ -0.9 };
  double pitch_{ 0.7 };
  double distance_{ 6.0 };
  const double min_distance_{ 0.35 };
  const double max_distance_{ 80.0 };
};
