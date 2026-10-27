#pragma once

// Compact arrangement toolbar above the 3D view (TIA-style interaction for a 3D
// assembly editor): a selection chip naming the count and the reference, icon
// tools grouped as Align / Distribute / Transform / Assembly with an overflow
// menu when the view is narrow, and an inline preview bar (parameters, Apply,
// Cancel) while an operation is pending. Tools that the current selection or
// geometry cannot support are disabled with the reason in their tooltip.

#include <QFrame>
#include <QHash>
#include <QJsonObject>
#include <QKeySequence>
#include <QList>
#include <QPair>
#include <QPointer>

#include <functional>

class QAction;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QToolButton;
class QWidget;

namespace workcell_builder
{

class ArrangeController;

class ArrangeToolbar : public QFrame
{
  Q_OBJECT

public:
  explicit ArrangeToolbar(ArrangeController * controller, QWidget * parent = nullptr);

  // The selection chip: members as (id, name), in selection order, and the reference.
  void set_selection(const QList<QPair<QString, QString>> & members, const QString & reference);
  // Extra availability for tools the controller does not plan (Mate hand-offs, group).
  // Returns the reason a tool is unavailable, or an empty string.
  void set_extra_availability(std::function<QString(const QString & tool)> check) { extra_ = std::move(check); }
  void refresh();
  QAction * action(const QString & tool) const { return actions_.value(tool); }
  QWidget * tools() const { return tools_host_; }
  QToolButton * more_button() const { return more_; }

protected:
  bool eventFilter(QObject * watched, QEvent * event) override;

signals:
  void reference_chosen(const QString & id);
  // Flush, coincident or concentric: verified-face mating through the Mate tool.
  void mate_requested(const QString & kind);

private:
  QAction * add_tool(const QString & tool, const QString & icon, const QString & text, const QString & tip,
    const QString & op, const QJsonObject & params, const QKeySequence & shortcut = QKeySequence());
  void start(const QString & tool);
  QJsonObject current_params() const;
  void update_parameter_visibility();

  QPointer<ArrangeController> controller_;
  std::function<QString(const QString &)> extra_;
  QToolButton * chip_{nullptr};
  // Tool buttons in order (nullptr marks a group separator); tools that do not fit
  // the view's width move to the More menu.
  void add_separator();
  void relayout_tools();
  QWidget * tools_host_{nullptr};
  QList<QWidget *> tool_widgets_;
  QList<QAction *> tool_actions_;
  QToolButton * more_{nullptr};
  QHash<QString, QAction *> actions_;
  QHash<QString, QPair<QString, QJsonObject>> plans_;  // tool -> (op, params)
  QHash<QString, QString> tips_;
  QString pending_tool_;
  QFrame * preview_bar_{nullptr};
  QLabel * summary_{nullptr};
  QWidget * gap_box_{nullptr};
  QDoubleSpinBox * gap_{nullptr};
  QWidget * move_box_{nullptr};
  QComboBox * frame_{nullptr};
  QComboBox * pivot_{nullptr};
  QDoubleSpinBox * move_[6]{};
  QWidget * grid_box_{nullptr};
  QWidget * align_box_{nullptr};
  QDoubleSpinBox * align_offset_[3]{};
  QCheckBox * keep_relative_{nullptr};
  QDoubleSpinBox * grid_{nullptr};
  QPushButton * apply_{nullptr};
  QPushButton * cancel_{nullptr};
  bool refreshing_{false};
};

}  // namespace workcell_builder
