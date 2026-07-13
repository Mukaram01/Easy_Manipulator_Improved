#pragma once

#include <QWidget>
#include <QVector>
#include <QStringList>
#include <QSet>
#include <QMatrix4x4>
#include <QProcess>
#include <QDateTime>

class QComboBox;
class QLabel;
class QStackedWidget;
class QComboBox;
class QGraphicsView;
class QGraphicsScene;
class QGraphicsItem;
class QGraphicsProxyWidget;
class QTcpSocket;
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
class QWebEngineView;
#endif

class ScenePreviewWidget : public QWidget
{
  Q_OBJECT
public:
  struct RenderDebugCounters;
  struct PreviewItem
  {
    QString id;
    QString display_name;
    QString category;
    double x{ 0.0 }, y{ 0.0 }, z{ 0.0 };
    double roll{ 0.0 }, pitch{ 0.0 }, yaw{ 0.0 };
    double sx{ 0.3 }, sy{ 0.3 }, sz{ 0.3 };
    QString status{ "unknown" };
    QString source_path;
    QString role;
    QString mesh_path;
    QString mesh_type;
    QString primitive_geometry_type;
    double primitive_radius{ 0.0 };
    double primitive_length{ 0.0 };
    bool has_material_color{ false };
    double material_r{ 0.0 }, material_g{ 0.0 }, material_b{ 0.0 }, material_a{ 1.0 };
    QString material_name;
    double mesh_scale_x{ 1.0 }, mesh_scale_y{ 1.0 }, mesh_scale_z{ 1.0 };
    double mesh_roll{ 0.0 }, mesh_pitch{ 0.0 }, mesh_yaw{ 0.0 };
    bool mesh_available{ false };
    QString mesh_load_warning;
    bool selectable{ true };
    bool editable{ true };
    bool locked{ false };
    QString lock_reason;
    QString metadata_tags;
    QString source_layer;
    QString active_visual_source;
    bool linked_to_editable_layout_state{ false };
    bool metadata_complete{ true };
    QStringList warnings;
    bool has_mesh_metadata{ false };
    double mesh_r{ 0.0 }, mesh_p{ 0.0 }, mesh_y{ 0.0 };
    bool has_origin_offset{ false };
    double origin_offset_x{ 0.0 }, origin_offset_y{ 0.0 }, origin_offset_z{ 0.0 };
    QString camera_id;
    QString frame_id;
    QString detection_label;
    double confidence{ -1.0 };
    QString tracking_id;
    QString snapshot_source_file;
    QString alignment_warning;
    bool resolved_source_path_stale{ false };
    QString resolved_source_path_original;
    QString package_uri;
    QString source_path_resolution_outcome;
    double base_pose_x{ 0.0 }, base_pose_y{ 0.0 }, base_pose_z{ 0.0 };
    double base_pose_roll{ 0.0 }, base_pose_pitch{ 0.0 }, base_pose_yaw{ 0.0 };
    double chain_pose_x{ 0.0 }, chain_pose_y{ 0.0 }, chain_pose_z{ 0.0 };
    double chain_pose_roll{ 0.0 }, chain_pose_pitch{ 0.0 }, chain_pose_yaw{ 0.0 };
    double visual_origin_x{ 0.0 }, visual_origin_y{ 0.0 }, visual_origin_z{ 0.0 };
    double visual_origin_roll{ 0.0 }, visual_origin_pitch{ 0.0 }, visual_origin_yaw{ 0.0 };
    bool transform_chain_applied{ false };
    bool visual_origin_applied{ false };
    bool has_baked_world_visual_transform{ false };
    QString baked_world_visual_transform_source;
    bool has_baked_world_visual_matrix{ false };
    QMatrix4x4 baked_world_visual_matrix;
    bool robot_candidate{ false };
    QString robot_classification_source;
    QString robot_base_frame;
    QString robot_world_pose;
    QString visual_index_link;
    QString visual_index_link_name;
    QString visual_index_object_name;
    QString visual_index_visual;
    QString visual_index_visual_name;
    int visual_index_value{ -1 };
    int source_row_index{ -1 };
    QString visual_index_parent_link;
    QStringList visual_index_link_chain;
    QString visual_index_mesh_uri;
    QString visual_index_package_uri;
    QString visual_index_source;
  };
  struct CameraOverlayModel
  {
    QString camera_id{"unknown"};
    QString display_name{"camera"};
    double x{0.0}, y{0.0}, z{0.0};
    double roll{0.0}, pitch{0.0}, yaw{0.0};
    QString frame_id{"unknown"};
    double horizontal_fov_deg{69.0};
    double vertical_fov_deg{42.0};
    double range_min_m{0.2};
    double range_max_m{2.0};
    QString source_path;
    QString metadata_source{"preview"};
    QString status{"unknown"};
    QStringList warnings;
  };
  struct EpdDetectionOverlayModel
  {
    QString detection_id{"unknown"};
    QString label{"unknown"};
    double confidence{-1.0};
    double x{0.0}, y{0.0}, z{0.0};
    double dx{0.08}, dy{0.08}, dz{0.08};
    QString source_path;
    QString status{"unknown"};
    QStringList warnings;
    bool selectable{true};
  };
  enum class ReachStatus { Reachable, NearLimit, OutOfReach, Unknown };
  enum class LabelMode { Off, Important, Selected, All };
  enum class MeshPreviewMode { Auto, Meshes, Primitives };
  struct ReachabilityOverlayModel
  {
    QString robot_base_id{"unknown"};
    QString planning_group{"unknown"};
    double approximate_reach_min_m{0.25};
    double approximate_reach_max_m{1.25};
    double preferred_work_zone_radius_m{0.85};
    QString pick_source_status{"unknown"};
    QString place_target_status{"unknown"};
    QString reject_target_status{"unknown"};
    QString selected_item_status{"unknown"};
    QString metadata_source{"preview"};
    QStringList warnings;
    bool has_mesh_metadata{ false };
    double mesh_r{ 0.0 }, mesh_p{ 0.0 }, mesh_y{ 0.0 };
    double mesh_scale_x{ 1.0 }, mesh_scale_y{ 1.0 }, mesh_scale_z{ 1.0 };
    bool has_origin_offset{ false };
    double origin_offset_x{ 0.0 }, origin_offset_y{ 0.0 }, origin_offset_z{ 0.0 };
    QString camera_id;
    QString frame_id;
    QString detection_label;
    double confidence{ -1.0 };
    QString tracking_id;
    QString snapshot_source_file;
    QString alignment_warning;
  };
  struct CollisionOverlayModel
  {
    QString metadata_source{"preview"};
    QStringList colliding_items;
    QStringList near_miss_items;
    QStringList warnings;
    bool has_mesh_metadata{ false };
    double mesh_r{ 0.0 }, mesh_p{ 0.0 }, mesh_y{ 0.0 };
    double mesh_scale_x{ 1.0 }, mesh_scale_y{ 1.0 }, mesh_scale_z{ 1.0 };
    bool has_origin_offset{ false };
    double origin_offset_x{ 0.0 }, origin_offset_y{ 0.0 }, origin_offset_z{ 0.0 };
    QString camera_id;
    QString frame_id;
    QString detection_label;
    double confidence{ -1.0 };
    QString tracking_id;
    QString snapshot_source_file;
    QString alignment_warning;
  };

  struct TaskOverlayModel
  {
    QString task_type{"unknown"};
    QString pick_source_id{"unknown"};
    QString place_target_id{"unknown"};
    QString reject_target_id{"unknown"};
    QString grasp_strategy{"unknown"};
    QString approach_axis{"unknown"};
    QString approach_distance{"unknown"};
    QString retreat_axis{"unknown"};
    QString retreat_distance{"unknown"};
    QString object_class{"unknown"};
    QStringList warnings;
    bool has_intent_metadata{ false };
  };
  explicit ScenePreviewWidget(QWidget * parent = nullptr);

  void set_fallback_2d_view(QGraphicsView * view);
  void set_scene_selected(bool selected);
  void set_3d_available(bool available, const QString & reason = QString());
  void set_preview_items(const QVector<PreviewItem> & items);
  void set_preview_scene_name(const QString & scene_name);
  void set_preview_status_summary(const QString & summary);
  void set_clean_product_view_status(bool clean, int visual_count);
  void set_task_overlay_model(const TaskOverlayModel & model);
  void set_reachability_overlay_model(const ReachabilityOverlayModel & model);
  void set_collision_overlay_model(const CollisionOverlayModel & model);
  void set_camera_overlay_model(const CameraOverlayModel & model);
  void set_epd_detection_overlays(const QVector<EpdDetectionOverlayModel> & detections);
  void set_perception_overlay_visibility(bool camera_fov, bool pick_coverage, bool epd_detections, bool detection_labels);
  void set_task_overlay_visibility(bool task_route, bool pick_place_zones, bool approach_retreat, bool labels);
  void set_label_mode(LabelMode mode);
  void select_preview_item(const QString & id);
  QString selected_preview_item_id() const;
  const PreviewItem * preview_item_by_id(const QString & id) const;
  MeshPreviewMode mesh_preview_mode() const;
  void reload_meshes();
  void apply_product_view_defaults();
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
    int urdf_primitive_source_count{ 0 };
    int urdf_primitive_rendered_count{ 0 };
    int placeholder_count{ 0 };
    int missing_geometry_count{ 0 };
    int wireframe_fallback_count{ 0 };
    int overlay_helper_count{ 0 };
    int overlay_count{ 0 };
    int generated_fallback_count{ 0 };
    int editable_layout_count{ 0 };
    int primitive_fallback_count{ 0 };
    int primitive_fallback_rendered_count{ 0 };
    int editable_primitive_rendered_count{ 0 };
    int valid_physical_fallback_count{ 0 };
    int overlay_rendered_count{ 0 };
    int locked_generated_urdf_visual_count{ 0 };
    int physical_anchor_count{ 0 };
    int generated_robot_mesh_count{ 0 };
    int tool_gripper_visual_count{ 0 };
    int table_workbench_visual_count{ 0 };
    int camera_body_visual_count{ 0 };
    int transform_chain_applied_count{ 0 };
    int visual_origin_applied_count{ 0 };
    int baked_world_visual_transform_count{ 0 };
    int legacy_viewport_transform_count{ 0 };
    QString visual_quality_status{ QStringLiteral("UNAVAILABLE") };
    QStringList visual_quality_warnings;
    int labels_drawn{ 0 };
    int labels_suppressed_overlap{ 0 };
    int hierarchy_child_row_count{ 0 };
    bool last_paint_completed{ false };
    bool smoke_fallback_render_used{ false };
  };
  RenderDebugCounters render_debug_counters() const;
  int total_warning_count() const;

signals:
  void studio_log_requested(const QString & message);
  void preview_item_selected(const QString & id, const QString & role);

private slots:
  void on_mode_changed(int index);
  void on_reset_view_clicked();
  void on_fit_scene_clicked();
  void on_fit_robot_clicked();
  void on_focus_selected_clicked();
  void on_fit_overlays_clicked();
  void on_clear_selection_clicked();

private:
  void refresh_mode_and_state();
  QRectF rendered_items_bounds_2d(bool include_overlays) const;
  void fit_fallback_scene_to_items(bool include_overlays = false);
  void reset_fallback_scene_view();
  void refresh_info_chip();
  void refresh_toolbar_visibility();
  enum class EmbeddedProductViewState { Idle, Preparing, StartingServer, Loading, Ready, Failed };
  void refresh_embedded_web_product_view();
  void request_embedded_web_product_view_refresh(bool force = false);
  void maybe_start_next_embedded_web_prepare();
  void start_embedded_web_prepare(const QString & scene, quint64 revision, bool force);
  void on_embedded_web_prepare_finished(int exit_code, QProcess::ExitStatus exit_status);
  void ensure_embedded_web_server_started(const QString & repo_root);
  bool embedded_web_server_is_usable() const;
  void load_prepared_embedded_web_scene(const QString & scene, quint64 revision);
  void start_embedded_web_readiness_polling(const QString & scene, quint64 revision, const QString & expected_json_path, const QString & viewer_url);
  void poll_embedded_web_readiness(const QString & scene, quint64 revision, const QString & expected_json_path, const QString & viewer_url);
  void set_embedded_product_view_state(EmbeddedProductViewState state, const QString & detail = QString());
  QString embedded_web_prepare_command_for_log(const QString & scene, const QString & output_path, bool force = false) const;
  QString resolve_embedded_web_repo_root() const;
  void emit_backend_startup_diagnostic_once();
  bool diagnostic_debug_logging_enabled() const;
  bool emit_scene_diagnostic_once(const QString & event, int payload_count, const QString & message);
  void emit_visual_quality_assessment_once();
  bool task_is_ready() const;

  QComboBox * mode_selector_{ nullptr };
  QComboBox * interaction_mode_selector_{ nullptr };
  QComboBox * view_actions_selector_{ nullptr };
  QLabel * view_mode_label_{ nullptr };
  QLabel * mesh_preview_mode_label_{ nullptr };
  QLabel * gizmo_mode_label_{ nullptr };
  QLabel * snap_mode_label_{ nullptr };
  QLabel * labels_label_{ nullptr };
  QLabel * interaction_mode_label_{ nullptr };
  QLabel * view_actions_label_{ nullptr };
  QLabel * toolbar_status_chip_{ nullptr };
  QComboBox * overlays_selector_{ nullptr };
  QComboBox * labels_selector_{ nullptr };
  QComboBox * mesh_preview_mode_selector_{ nullptr };
  QComboBox * gizmo_mode_selector_{ nullptr };
  QComboBox * snap_mode_selector_{ nullptr };
  QStackedWidget * stack_{ nullptr };
  QWidget * view3d_container_{ nullptr };
  QWidget * view2d_container_{ nullptr };
  QLabel * empty_state_label_{ nullptr };
  QLabel * error_state_label_{ nullptr };
  QLabel * fallback_banner_label_{ nullptr };
  QWidget * simple_3d_view_{ nullptr };
#ifdef WORKCELL_BUILDER_HAS_WEBENGINE
  QWebEngineView * embedded_web_view_{ nullptr };
#endif
  QProcess * embedded_web_server_process_{ nullptr };
  QProcess * embedded_web_prepare_process_{ nullptr };
  EmbeddedProductViewState embedded_product_view_state_{ EmbeddedProductViewState::Idle };
  QString embedded_web_repo_root_;
  QString embedded_web_prepare_scene_;
  QString embedded_web_prepare_output_path_;
  QString embedded_web_last_viewer_url_;
  QString embedded_web_last_boot_status_;
  QDateTime embedded_web_readiness_deadline_;
  QString pending_embedded_web_scene_;
  quint64 embedded_web_request_revision_{ 0 };
  quint64 embedded_web_active_revision_{ 0 };
  quint64 pending_embedded_web_revision_{ 0 };
  bool pending_embedded_web_force_{ false };
  bool backend_startup_diagnostic_emitted_{ false };
  QDateTime embedded_web_prepare_started_at_;
  int embedded_web_server_port_{ 8765 };
  QGraphicsView * fallback_2d_view_{ nullptr };
  QLabel * info_chip_label_{ nullptr };
  QGraphicsProxyWidget * fallback_info_chip_proxy_{ nullptr };
  QString preview_scene_name_{"No scene"};
  bool scene_selected_{ false };
  bool preview3d_available_{ true };
  bool mode_default_initialized_{ false };
  QString unavailable_reason_;
  QVector<PreviewItem> preview_items_;
  TaskOverlayModel overlay_model_;
  ReachabilityOverlayModel reachability_overlay_model_;
  CollisionOverlayModel collision_overlay_model_;
  CameraOverlayModel camera_overlay_model_;
  QVector<EpdDetectionOverlayModel> epd_detections_;
  QString selected_preview_item_id_;
  QString preview_status_summary_;
  bool clean_product_view_{ false };
  int clean_product_visual_count_{ 0 };
  MeshPreviewMode mesh_preview_mode_{ MeshPreviewMode::Auto };
  int preview_payload_revision_{ 0 };
  int last_visual_quality_revision_logged_{ -1 };
  QSet<QString> emitted_scene_diagnostic_keys_;
};
