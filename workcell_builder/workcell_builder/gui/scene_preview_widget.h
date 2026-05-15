#pragma once

#include <QWidget>

class QComboBox;
class QLabel;
class QPushButton;
class QStackedWidget;
class QGraphicsView;

class ScenePreviewWidget : public QWidget
{
  Q_OBJECT
public:
  explicit ScenePreviewWidget(QWidget * parent = nullptr);

  void set_fallback_2d_view(QGraphicsView * view);
  void set_scene_selected(bool selected);
  void set_3d_available(bool available, const QString & reason = QString());

signals:
  void studio_log_requested(const QString & message);

private slots:
  void on_mode_changed(int index);

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
};
