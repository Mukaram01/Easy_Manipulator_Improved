#pragma once

// Inspector panel for the visual Mate tool: the two picks (with Replace and the
// manual datum selector), alignment, offset, live resolver preview and the
// Apply / Reset / Cancel actions. It only presents MateToolController state.

#include <QFrame>
#include <QPointer>

#include <array>

class QButtonGroup;
class QCheckBox;
class QToolButton;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QRadioButton;

namespace workcell_builder
{

class MateToolController;

class MateToolPanel : public QFrame
{
  Q_OBJECT

public:
  explicit MateToolPanel(MateToolController * controller, QWidget * parent = nullptr);
  void refresh();

signals:
  void manual_dialog_requested();

private:
  struct Row
  {
    QLabel * title{nullptr};
    QLabel * value{nullptr};
    QComboBox * datums{nullptr};
    QPushButton * replace{nullptr};
  };
  Row build_row(const QString & title, const QString & object_prefix);
  void fill_datums(QComboBox * combo, const QString & asset_id, const QString & selected);

  QPointer<MateToolController> controller_;
  QLabel * intro_{nullptr};
  QLabel * status_{nullptr};
  QLabel * backend_note_{nullptr};
  Row child_;
  Row parent_;
  QCheckBox * isolate_{nullptr};
  QToolButton * face_view_{nullptr};
  QRadioButton * flush_{nullptr};
  QRadioButton * coincident_{nullptr};
  QLabel * alignment_note_{nullptr};
  std::array<QDoubleSpinBox *, 6> offset_{};
  QLabel * preview_{nullptr};
  QPushButton * apply_{nullptr};
  QPushButton * reset_{nullptr};
  QPushButton * cancel_{nullptr};
  QPushButton * manual_{nullptr};
  bool refreshing_{false};
  QString face_view_key_;  // rebuild the menu only when its datums change
};

}  // namespace workcell_builder
