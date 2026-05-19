#pragma once

#include "scene_preview_widget.h"

#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QHash>
#include <QSet>
#include <QVector3D>
#include <QJsonObject>

#include <array>

#include <functional>


class Scene3DViewportWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
  Q_OBJECT
public:
  struct InternalTriangleMesh
  {
    struct Triangle
    {
      std::array<QVector3D, 3> vertices;
      QVector3D normal;
    };
    QVector<Triangle> triangles;
  };

  explicit Scene3DViewportWidget(QWidget * parent = nullptr);

  QVector<ScenePreviewWidget::PreviewItem> items;
  QString selected_id;
  ScenePreviewWidget::LabelMode label_mode{ ScenePreviewWidget::LabelMode::Important };
  bool show_warnings{ true }, show_safety{ true }, show_pick_place{ true };
  bool show_reachability_heatmap{ true }, show_collision_warnings{ true }, show_work_envelope{ true }, show_warning_labels{ true };
  bool show_task_route{ true }, show_approach_retreat{ true };
  bool show_camera_fov{ true }, show_pick_coverage{ true }, show_epd_detections{ true }, show_detection_labels{ true };
  bool debug_overlays_mode{ false };
  ScenePreviewWidget::MeshPreviewMode mesh_preview_mode{ ScenePreviewWidget::MeshPreviewMode::Auto };
  bool fit_include_overlays{ false };
  ScenePreviewWidget::TaskOverlayModel task_overlay;
  ScenePreviewWidget::ReachabilityOverlayModel reach_overlay;
  ScenePreviewWidget::CollisionOverlayModel collision_overlay;
  QString scene_name{ "No scene" };
  ScenePreviewWidget::CameraOverlayModel camera_overlay;
  QVector<ScenePreviewWidget::EpdDetectionOverlayModel> epd_detections;
  std::function<void(const QString &, const QString &)> select_cb;
  std::function<void(const QString &)> status_message_cb;
  std::function<void(const QJsonObject &, double, double, double, bool)> asset_drop_cb;
  std::function<void(const QString &, double, double, double, double, double, double)> transform_changed_cb;
  enum class GizmoMode { Select, Move, Rotate, ScaleDisabled };
  enum class SnapMode { Off, Cm1, Cm5, Cm10, Deg5, Deg15 };
  enum class GizmoHandle
  {
    None,
    MoveX,
    MoveY,
    MoveZ,
    Roll,
    Pitch,
    Yaw
  };
  GizmoMode gizmo_mode{ GizmoMode::Select };
  SnapMode snap_mode{ SnapMode::Cm5 };

  void reset_view();
  void fit_scene();
  void focus_selected();
  void set_top_view();
  void set_front_view();
  void set_side_view();
  void set_isometric_view();
  void invalidate_mesh_cache();

  static bool parse_stl_bytes_for_test(const QByteArray & bytes, const QString & source_hint,
                                       InternalTriangleMesh & out_mesh, QString & out_error,
                                       int triangle_limit = 100000);
  static bool compute_mesh_bounds_for_test(const InternalTriangleMesh & mesh, QVector3D & out_min, QVector3D & out_max);
  static bool should_attempt_mesh_draw_for_mode_for_test(ScenePreviewWidget::MeshPreviewMode mode,
                                                         bool cache_loaded, bool cache_valid);

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent * e) override;
  void mouseMoveEvent(QMouseEvent * e) override;
  void mouseReleaseEvent(QMouseEvent * e) override;
  void wheelEvent(QWheelEvent * e) override;
  void keyPressEvent(QKeyEvent * e) override;
  void dragEnterEvent(QDragEnterEvent * event) override;
  void dragMoveEvent(QDragMoveEvent * event) override;
  void dragLeaveEvent(QDragLeaveEvent * event) override;
  void dropEvent(QDropEvent * event) override;

private:
  struct ItemBounds { double x, y, z, sx, sy, sz; };
  QPointF project_to_screen(double x, double y, double z) const;
  bool pick_item_at_screen(const QPoint & pos, QString & out_id, QString & out_role, QString * out_tooltip = nullptr) const;
  bool pick_gizmo_axis_at_screen(const QPoint & pos, QString & out_axis, double * out_score = nullptr) const;
  bool pick_gizmo_rotation_ring_at_screen(const QPoint & pos, QString & out_axis, double * out_score = nullptr) const;
  ItemBounds item_bounds_for_role(const ScenePreviewWidget::PreviewItem & item) const;
  bool mesh_world_bounds_for_item(const ScenePreviewWidget::PreviewItem & item, ItemBounds & out_bounds) const;
  bool ray_intersects_aabb(const QVector3D & ray_origin, const QVector3D & ray_dir,
                           const ScenePreviewWidget::PreviewItem & item, float & out_t) const;
  void camera_matrices(QMatrix4x4 & out_proj, QMatrix4x4 & out_view) const;
  void draw_box(double cx, double cy, double cz, double sx, double sy, double sz, const QColor & color, bool translucent = false);
  void draw_box_outline(double cx, double cy, double cz, double sx, double sy, double sz, const QColor & color, float line_width = 2.5f);
  void draw_cylinder(double cx, double cy, double cz, double radius, double height, const QColor & color, bool translucent = false);
  void draw_frustum(const QColor & color, bool translucent = true);
  void draw_ground_grid_pass();
  void draw_world_axes_pass();
  bool scene_bounds_from_visible_items(QVector3D & out_min, QVector3D & out_max, bool include_overlays) const;
  QString gizmo_mode_label() const;
  void draw_robot_base_with_axis(const ScenePreviewWidget::PreviewItem & it);
  void draw_table_slab(const ScenePreviewWidget::PreviewItem & it);
  void draw_conveyor(const ScenePreviewWidget::PreviewItem & it);
  void draw_camera_body_with_frustum(const ScenePreviewWidget::PreviewItem & it);
  void draw_pick_zone(const ScenePreviewWidget::PreviewItem & it);
  void draw_place_target_bin(const ScenePreviewWidget::PreviewItem & it);
  void draw_object_cube(const ScenePreviewWidget::PreviewItem & it);
  void draw_safety_zone(const ScenePreviewWidget::PreviewItem & it);
  void draw_warning_badge_anchor(const ScenePreviewWidget::PreviewItem & it);
  bool draw_mesh_preview_if_available(const ScenePreviewWidget::PreviewItem & it, const QColor & color, bool preview_path = true);
  void draw_unit_cube_triangles(const QColor & color);
  QPoint last_;
  QPoint drag_start_screen_;
  bool dragging_gizmo_{ false };
  QString active_axis_;
  struct DragPose
  {
    QString item_id;
    double x{ 0.0 }, y{ 0.0 }, z{ 0.0 };
    double roll{ 0.0 }, pitch{ 0.0 }, yaw{ 0.0 };
  };
  DragPose drag_start_pose_;
  bool drag_in_progress_{ false };
  bool drag_cancelled_{ false };
  GizmoHandle drag_active_handle_{ GizmoHandle::None };
  // active_gizmo_handle token preserved for static validator compatibility.
  GizmoHandle hovered_gizmo_handle_{ GizmoHandle::None };
  QString hovered_id_;
  struct MeshCacheEntry
  {
    bool loaded{ false };
    bool valid{ false };
    bool oversized{ false };
    QString warning;
    InternalTriangleMesh mesh;
    bool has_bounds{ false };
    QVector3D local_min;
    QVector3D local_max;
  };
  QHash<QString, MeshCacheEntry> mesh_cache_;
  QSet<QString> warned_mesh_fallbacks_;
  bool try_resolve_canonical_mesh_path(const QString & path, QString & out_canonical) const;
  bool warn_mesh_fallback_once(const QString & item_id, const QString & reason, const QString & path);
  const MeshCacheEntry & ensure_mesh_cached(const QString & path);
  QVector3D orbit_offset_{ 0.0f, 0.0f, 0.0f };
  double yaw_{ -0.9 };
  double pitch_{ 0.7 };
  double distance_{ 6.0 };
  double scene_radius_{ 2.0 };
  const double min_distance_{ 0.35 };
  const double max_distance_{ 80.0 };
  bool drag_asset_preview_visible_{ false };
  QString drag_asset_label_;
  QString drag_asset_drop_status_;
  QPoint drag_asset_screen_pos_;
  QJsonObject drag_asset_payload_;
};
