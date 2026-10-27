#pragma once

// Inspector panel for Align & Arrange: the part to move, the reference, the
// Centre / Align / Place operations, offsets in the reference frame, Align once
// versus Keep relative, the numerical preview and Apply / Clear / Cancel. It
// only presents AlignArrangeController state.

#include <QFrame>
#include <QList>
#include <QPair>
#include <QPointer>

#include <array>

class QAbstractButton;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QRadioButton;
class QToolButton;

namespace workcell_builder
{

class AlignArrangeController;

class AlignArrangePanel : public QFrame
{
  Q_OBJECT

public:
  explicit AlignArrangePanel(AlignArrangeController * controller, QWidget * parent = nullptr);
  // Parts that may be chosen as the reference: (id, display name).
  void set_reference_candidates(const QList<QPair<QString, QString>> & candidates);
  // How the active view shows the preview (empty: the ghost is drawn).
  void set_view_note(const QString & note);
  void refresh();

signals:
  // The user wants verified-face mating instead (the existing Mate tool).
  void mate_requested();

private:
  struct Operation
  {
    QToolButton * button{nullptr};
    int axis{0};
    QString rule;
  };
  QToolButton * add_operation(const QString & text, int axis, const QString & rule, const QString & tooltip,
    const QString & object_name);

  QPointer<AlignArrangeController> controller_;
  QLabel * intro_{nullptr};
  QLabel * view_note_{nullptr};
  QLabel * moving_{nullptr};
  QComboBox * reference_{nullptr};
  QList<Operation> operations_;
  QToolButton * centre_both_{nullptr};
  std::array<QDoubleSpinBox *, 3> offset_{};
  QRadioButton * once_{nullptr};
  QRadioButton * keep_{nullptr};
  QLabel * preview_{nullptr};
  QLabel * method_{nullptr};
  QLabel * status_{nullptr};
  QPushButton * apply_{nullptr};
  QPushButton * clear_{nullptr};
  QPushButton * cancel_{nullptr};
  QPushButton * mate_{nullptr};
  bool refreshing_{false};
};

}  // namespace workcell_builder
