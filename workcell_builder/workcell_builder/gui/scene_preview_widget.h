#pragma once

#include <QWidget>
#include <QVector>
#include <QStringList>

class QComboBox;
class QLabel;
class QPushButton;
class QStackedWidget;
class QGraphicsView;

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
  void set_task_overlay_model(const TaskOverlayModel & model);
  void set_task_overlay_visibility(bool task_route, bool pick_place_zones, bool approach_retreat, bool labels);
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

  QComboBox * mode_selector_{ nullptr };
  QPushButton * reset_view_button_{ nullptr };
  QPushButton * fit_scene_button_{ nullptr };
  QComboBox * overlays_selector_{ nullptr };
  QStackedWidget * stack_{ nullptr };
  QWidget * view3d_container_{ nullptr };
  QWidget * view2d_container_{ nullptr };
  QLabel * empty_state_label_{ nullptr };
  QLabel * error_state_label_{ nullptr };
  QWidget * simple_3d_view_{ nullptr };
  QGraphicsView * fallback_2d_view_{ nullptr };
  bool scene_selected_{ false };
  bool preview3d_available_{ true };
  QString unavailable_reason_;
  QVector<PreviewItem> preview_items_;
  TaskOverlayModel overlay_model_;
  QString selected_preview_item_id_;
};
