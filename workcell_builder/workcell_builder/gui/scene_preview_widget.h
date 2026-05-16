#pragma once

#include <QWidget>
#include <QVector>
#include <QStringList>

class QComboBox;
class QLabel;
class QPushButton;
class QStackedWidget;
class QGraphicsView;
class QGraphicsScene;
class QGraphicsItem;
class QGraphicsProxyWidget;

class ScenePreviewWidget : public QWidget
{
  Q_OBJECT
public:
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
    bool selectable{ true };
    bool metadata_complete{ true };
    QStringList warnings;
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
  enum class LabelMode { Off, SelectedOnly, All };
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
  };
  struct CollisionOverlayModel
  {
    QString metadata_source{"preview"};
    QStringList colliding_items;
    QStringList near_miss_items;
    QStringList warnings;
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

signals:
  void studio_log_requested(const QString & message);
  void preview_item_selected(const QString & id);

private slots:
  void on_mode_changed(int index);
  void on_reset_view_clicked();
  void on_fit_scene_clicked();
  void on_focus_selected_clicked();
  void on_clear_selection_clicked();

private:
  void refresh_mode_and_state();
  QRectF rendered_items_bounds_2d() const;
  void fit_fallback_scene_to_items();
  void reset_fallback_scene_view();
  void refresh_info_chip();
  int total_warning_count() const;
  bool task_is_ready() const;

  QComboBox * mode_selector_{ nullptr };
  QPushButton * reset_view_button_{ nullptr };
  QPushButton * fit_scene_button_{ nullptr };
  QComboBox * overlays_selector_{ nullptr };
  QStackedWidget * stack_{ nullptr };
  QWidget * view3d_container_{ nullptr };
  QWidget * view2d_container_{ nullptr };
  QLabel * empty_state_label_{ nullptr };
  QLabel * error_state_label_{ nullptr };
  QLabel * fallback_banner_label_{ nullptr };
  QWidget * simple_3d_view_{ nullptr };
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
};
