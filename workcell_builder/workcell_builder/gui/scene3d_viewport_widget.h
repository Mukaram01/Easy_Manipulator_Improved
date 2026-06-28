#pragma once

#include "scene_preview_widget.h"

#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QColor>
#include <QHash>
#include <QSet>
#include <QVector3D>
#include <QJsonObject>
#include <QJsonArray>

#include <array>

#include <functional>

class QImage;
class QPainter;

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
  ScenePreviewWidget::LabelMode label_mode{ ScenePreviewWidget::LabelMode::Selected };
  bool show_warnings{ false }, show_safety{ false }, show_pick_place{ false };
  bool show_reachability_heatmap{ false }, show_collision_warnings{ false }, show_work_envelope{ false }, show_warning_labels{ false };
  bool show_task_route{ false }, show_approach_retreat{ false };
  bool show_camera_fov{ false }, show_pick_coverage{ false }, show_epd_detections{ false }, show_detection_labels{ false };
  bool debug_overlays_mode{ false };
  bool diagnostic_transparency_mode{ false };
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
  void fit_product_view();
  void fit_robot();
  void focus_selected();
  void set_top_view();
  void set_front_view();
  void set_side_view();
  void set_isometric_view();
  void invalidate_mesh_cache();
  void ingest_preview_items(const QVector<ScenePreviewWidget::PreviewItem> & preview_items);

  struct RenderDebugCounters
  {
    int preview_items_count{ 0 };
    int total_payload_count{ 0 };
    int viewport_received_count{ 0 };
    int render_cache_count{ 0 };
    int visible_count{ 0 };
    int rendered_count{ 0 };
    int skipped_count{ 0 };
    int unique_visible_item_count{ 0 };
    int mesh_backed_count{ 0 };
    int mesh_source_count{ 0 };
    int mesh_path_resolved_count{ 0 };
    int mesh_file_loaded_count{ 0 };
    int mesh_triangles_loaded_count{ 0 };
    int mesh_rendered_count{ 0 };
    int mesh_surface_rendered_count{ 0 };
    int mesh_bounds_fallback_rendered_count{ 0 };
    int generated_mesh_bounds_fallback_rendered_count{ 0 };
    int urdf_primitive_source_count{ 0 };
    int urdf_primitive_rendered_count{ 0 };
    int placeholder_count{ 0 };
    int missing_geometry_count{ 0 };
    int generated_missing_geometry_count{ 0 };
    int wireframe_fallback_count{ 0 };
    int overlay_helper_count{ 0 };
    int generated_fallback_count{ 0 };
    int editable_layout_count{ 0 };
    int primitive_fallback_count{ 0 };
    int primitive_fallback_rendered_count{ 0 };
    int editable_primitive_rendered_count{ 0 };
    int valid_physical_fallback_count{ 0 };
    int overlay_rendered_count{ 0 };
    int locked_generated_urdf_visual_count{ 0 };
    int transform_chain_applied_count{ 0 };
    int visual_origin_applied_count{ 0 };
    int baked_world_visual_transform_count{ 0 };
    int legacy_viewport_transform_count{ 0 };
    int overlay_count{ 0 };
    QString visual_quality_status{ QStringLiteral("UNAVAILABLE") };
    QStringList visual_quality_warnings;
    int labels_drawn{ 0 };
    int labels_suppressed_overlap{ 0 };
    int hierarchy_child_row_count{ 0 };
    bool last_paint_completed{ false };
    bool smoke_fallback_render_used{ false };
  };
  RenderDebugCounters last_render_counters;
  RenderDebugCounters render_debug_counters() const;
  bool last_initial_fit_included_robot_bounds() const;
  int last_initial_fit_physical_anchor_count() const;
  QStringList last_initial_fit_anchor_roles() const;
  QString last_camera_fit_target() const;
  bool render_smoke_fallback_frame(QImage * out_image = nullptr);
  QJsonArray mesh_diagnostics_export() const;
  QJsonArray generated_robot_final_draw_candidate_diagnostics_export() const;
  QJsonArray final_draw_visual_items_export() const;
  QJsonArray final_draw_diagnostics_export() const;

  static bool parse_stl_bytes_for_test(const QByteArray & bytes, const QString & source_hint,
                                       InternalTriangleMesh & out_mesh, QString & out_error,
                                       int triangle_limit = 100000);
  static bool parse_collada_bytes_for_test(const QByteArray & bytes, const QString & source_hint,
                                           InternalTriangleMesh & out_mesh, QString & out_error,
                                           double * out_unit_meter = nullptr,
                                           int triangle_limit = 100000);
  static bool parse_obj_bytes_for_test(const QByteArray & bytes, const QString & source_hint,
                                       InternalTriangleMesh & out_mesh, QString & out_error,
                                       int triangle_limit = 100000);
  static bool compute_mesh_bounds_for_test(const InternalTriangleMesh & mesh, QVector3D & out_min, QVector3D & out_max);
  static bool should_attempt_mesh_draw_for_mode_for_test(ScenePreviewWidget::MeshPreviewMode mode,
                                                         bool cache_loaded, bool cache_valid);
  static QString render_role_for_test(const ScenePreviewWidget::PreviewItem & item);
  static bool should_include_in_default_fit_for_test(const ScenePreviewWidget::PreviewItem & item);
  static bool should_draw_as_solid_for_test(const ScenePreviewWidget::PreviewItem & item,
                                            ScenePreviewWidget::MeshPreviewMode mode);
  static bool should_draw_as_wireframe_for_test(const ScenePreviewWidget::PreviewItem & item,
                                                ScenePreviewWidget::MeshPreviewMode mode);
  static bool should_draw_clean_semantic_primitive_for_test(const ScenePreviewWidget::PreviewItem & item);
  static bool should_suppress_missing_geometry_marker_for_test(const ScenePreviewWidget::PreviewItem & item);
  static bool has_mesh_surface_candidate_for_test(const ScenePreviewWidget::PreviewItem & item);
  static QColor material_color_for_test(const ScenePreviewWidget::PreviewItem & item, bool diagnostic_transparency_mode = false);
  static bool is_raw_generated_bounds_only_for_test(const ScenePreviewWidget::PreviewItem & item);
  static QMatrix4x4 final_mesh_transform_matrix_for_test(const ScenePreviewWidget::PreviewItem & item);
  static QMatrix4x4 baked_mesh_asset_local_correction_matrix_for_test(const ScenePreviewWidget::PreviewItem & item);

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
  void draw_cylinder(double cx, double cy, double cz, double radius, double height, const QColor & color,
                     bool translucent = false, int segment_count = 32);
  void draw_sphere(double cx, double cy, double cz, double radius, const QColor & color,
                   bool translucent = false, int slice_count = 24, int stack_count = 12);
  void draw_frustum(const QColor & color, bool translucent = true);
  void configure_product_render_state();
  void draw_viewport_quality_overlay(QPainter & painter, int visible_item_count, int physical_item_count) const;
  void draw_ground_grid_pass();
  void draw_world_axes_pass();
  bool scene_bounds_from_visible_items(QVector3D & out_min, QVector3D & out_max, bool include_overlays) const;
  bool initial_physical_fit_bounds(QVector3D & out_min, QVector3D & out_max,
                                   bool * out_robot_included = nullptr,
                                   int * out_anchor_count = nullptr,
                                   QStringList * out_anchor_roles = nullptr) const;
  bool robot_bounds_from_rendered_visuals(QVector3D & out_min, QVector3D & out_max) const;
  bool item_has_explicit_dimensions(const ScenePreviewWidget::PreviewItem & item) const;
  QString placeholder_reason_for_item(const ScenePreviewWidget::PreviewItem & item) const;
  bool draw_urdf_primitive_geometry(const ScenePreviewWidget::PreviewItem & it, const QColor & color);
  bool draw_required_generated_robot_emergency_fallback(const ScenePreviewWidget::PreviewItem & it, const QColor & color);
  bool draw_truthful_item_geometry(const ScenePreviewWidget::PreviewItem & it, int * out_placeholder_count = nullptr,
                                   int * out_mesh_count = nullptr, int * out_wireframe_count = nullptr,
                                   int * out_urdf_primitive_count = nullptr, int * out_missing_geometry_count = nullptr,
                                   int * out_primitive_fallback_count = nullptr,
                                   int * out_editable_primitive_count = nullptr);
  QString gizmo_mode_label() const;
  void draw_robot_base_with_axis(const ScenePreviewWidget::PreviewItem & it);
  void draw_table_slab(const ScenePreviewWidget::PreviewItem & it);
  void draw_conveyor(const ScenePreviewWidget::PreviewItem & it);
  void draw_camera_body_with_frustum(const ScenePreviewWidget::PreviewItem & it);
  void draw_realsense_d435_visual_surrogate(const ScenePreviewWidget::PreviewItem & it);
  void draw_pick_zone(const ScenePreviewWidget::PreviewItem & it);
  void draw_place_zone(const ScenePreviewWidget::PreviewItem & it);
  void draw_place_target_bin(const ScenePreviewWidget::PreviewItem & it);
  void draw_home_pose_marker(const ScenePreviewWidget::PreviewItem & it);
  void draw_object_cube(const ScenePreviewWidget::PreviewItem & it);
  void draw_missing_geometry_marker(const ScenePreviewWidget::PreviewItem & it);
  void draw_safety_zone(const ScenePreviewWidget::PreviewItem & it);
  void draw_warning_badge_anchor(const ScenePreviewWidget::PreviewItem & it);
  bool draw_clean_semantic_primitive(const ScenePreviewWidget::PreviewItem & it);
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
    QString parser_type;
    QString parse_status;
    QString parse_error;
    QString load_failure_reason;
    QString failure_reason_code;
    QString requested_path;
    bool path_resolved{ false };
    QString package_uri;
    QString resolved_source_path_original;
    QString source_path_resolution_outcome;
    bool resolved_source_path_stale{ false };
    InternalTriangleMesh mesh;
    bool has_bounds{ false };
    QVector3D local_min;
    QVector3D local_max;
    QVector3D local_span;
    double dae_unit_meter{ 1.0 };
    bool dae_has_pre_unit_bounds{ false };
    bool visual_surrogate_available{ false };
    QString visual_surrogate_type;
    QString visual_surrogate_reason;
    QVector3D dae_pre_unit_min;
    QVector3D dae_pre_unit_max;
    QVector3D dae_pre_unit_span;
  };
  QHash<QString, MeshCacheEntry> mesh_cache_;
  QHash<QString, QString> last_mesh_rejection_reasons_;
  QSet<QString> warned_mesh_fallbacks_;
  int last_scene_load_summary_item_count_{ -1 };
  QString last_scene_load_summary_scene_name_;
  QString last_scene_load_summary_warning_signature_;
  bool try_resolve_canonical_mesh_path(const QString & path, QString & out_canonical,
                                       const ScenePreviewWidget::PreviewItem * item = nullptr,
                                       QString * out_failure_reason = nullptr) const;
  bool warn_mesh_fallback_once(const QString & item_id, const QString & reason, const QString & path);
  QString mesh_rejection_diagnostic_detail(const ScenePreviewWidget::PreviewItem & item,
                                          const QString & mesh_source,
                                          const QString & canonical_mesh_source = QString(),
                                          const MeshCacheEntry * entry = nullptr,
                                          const QString & extra_detail = QString()) const;
  bool item_should_surface_mesh_warning(const ScenePreviewWidget::PreviewItem & item) const;
  void remember_mesh_rejection_reason(const QString & item_id, const QString & reason);
  QString last_mesh_rejection_reason_for_item(const QString & item_id) const;
  const MeshCacheEntry & ensure_mesh_cached(const ScenePreviewWidget::PreviewItem & item, const QString & path);
  bool validate_mesh_final_span(const ScenePreviewWidget::PreviewItem & it,
                                const MeshCacheEntry & entry,
                                const QString & mesh_source,
                                QString & out_reason,
                                QVector3D * out_raw_span = nullptr,
                                QVector3D * out_final_span = nullptr) const;
  QVector3D orbit_offset_{ 0.0f, 0.0f, 0.0f };
  double yaw_{ -0.9 };
  double pitch_{ -0.7 };
  double distance_{ 6.0 };
  double scene_radius_{ 2.0 };
  const double min_distance_{ 0.35 };
  const double max_distance_{ 80.0 };
  bool drag_asset_preview_visible_{ false };
  QString drag_asset_label_;
  QString drag_asset_drop_status_;
  QPoint drag_asset_screen_pos_;
  QJsonObject drag_asset_payload_;
  QString last_camera_fit_target_{ "scene" };
  QVector3D last_camera_fit_bounds_min_{ 0.0f, 0.0f, 0.0f };
  QVector3D last_camera_fit_bounds_max_{ 0.0f, 0.0f, 0.0f };
  QVector3D last_camera_fit_bounds_span_{ 0.0f, 0.0f, 0.0f };
  QString last_camera_fit_margin_{ "unset" };
  double last_camera_fit_margin_value_{ 0.0 };
  bool last_initial_fit_included_robot_bounds_{ false };
  int last_initial_fit_physical_anchor_count_{ 0 };
  QStringList last_initial_fit_anchor_roles_;
  bool has_robot_aabb_diag_{ false };
  QVector3D last_robot_aabb_min_;
  QVector3D last_robot_aabb_max_;
};
