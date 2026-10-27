#include "arrange_toolbar.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QLabel>
#include <QMenu>
#include <QPushButton>
#include <QSignalBlocker>
#include <QToolButton>
#include <QVBoxLayout>

#include "arrange_controller.hpp"
#include "arrange_icons.hpp"
#include "workcell_builder_ui_utils.hpp"

namespace workcell_builder
{
namespace
{

QJsonObject rules(std::initializer_list<std::pair<const char *, const char *>> values)
{
  QJsonObject result;
  for (const auto & value : values) result.insert(value.first, value.second);
  return QJsonObject{{"rules", result}};
}

QDoubleSpinBox * spin(QWidget * parent, const QString & name, const QString & prefix, const QString & suffix,
  double minimum, double maximum, double value, int decimals = 1)
{
  auto * box = new QDoubleSpinBox(parent);
  box->setObjectName(name);
  box->setPrefix(prefix);
  box->setSuffix(suffix);
  box->setRange(minimum, maximum);
  box->setDecimals(decimals);
  box->setValue(value);
  box->setKeyboardTracking(false);
  box->setAccelerated(true);
  box->setMinimumWidth(78);
  return box;
}

}  // namespace

ArrangeToolbar::ArrangeToolbar(ArrangeController * controller, QWidget * parent)
: QFrame(parent), controller_(controller)
{
  setObjectName(QStringLiteral("sceneBuilderArrangeToolbar"));
  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(4);

  auto * row = new QHBoxLayout();
  row->setSpacing(6);
  chip_ = new QToolButton(this);
  chip_->setObjectName(QStringLiteral("arrangeSelectionChip"));
  chip_->setPopupMode(QToolButton::InstantPopup);
  chip_->setToolButtonStyle(Qt::ToolButtonTextOnly);
  chip_->setMenu(new QMenu(chip_));
  chip_->setAccessibleName(QStringLiteral("Selection and reference"));
  row->addWidget(chip_);
  chip_->setMaximumWidth(240);
  tools_host_ = new QWidget(this);
  tools_host_->setObjectName(QStringLiteral("arrangeTools"));
  tools_host_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);  // the More menu, never a wider view
  auto * tools_layout = new QHBoxLayout(tools_host_);
  tools_layout->setContentsMargins(0, 0, 0, 0);
  tools_layout->setSpacing(2);
  tools_host_->installEventFilter(this);
  // Compact icon buttons (the application sheet sizes text buttons, not a tool strip).
  tools_host_->setStyleSheet(QStringLiteral(
    "QToolButton { padding: 3px; margin: 0px; }"));
  row->addWidget(tools_host_, 1);
  more_ = new QToolButton(this);
  more_->setObjectName(QStringLiteral("arrangeMore"));
  more_->setIcon(studioIcon(QStringLiteral("more")));
  more_->setToolTip(QStringLiteral("More arrangement tools (the view is too narrow to show them all)"));
  more_->setAccessibleName(QStringLiteral("More arrangement tools"));
  more_->setPopupMode(QToolButton::InstantPopup);
  more_->setMenu(new QMenu(more_));
  more_->hide();
  row->addWidget(more_);
  outer->addLayout(row);

  const auto alt = [](char key) { return QKeySequence(Qt::ALT | Qt::SHIFT | key); };
  add_tool("align_left", "align_left", "Align left", "Line up the −X box faces with the reference's −X face.", "align", rules({{"x", "min"}}), alt('L'));
  add_tool("align_right", "align_right", "Align right", "Line up the +X box faces with the reference's +X face.", "align", rules({{"x", "max"}}), alt('R'));
  add_tool("align_front", "align_front", "Align front", "Line up the −Y box faces with the reference's −Y face.", "align", rules({{"y", "min"}}), alt('F'));
  add_tool("align_back", "align_back", "Align back", "Line up the +Y box faces with the reference's +Y face.", "align", rules({{"y", "max"}}), alt('B'));
  add_tool("align_bottom", "align_bottom", "Align bottom", "Line up the box bottoms (−Z) with the reference's bottom.", "align", rules({{"z", "min"}}), alt('G'));
  add_tool("align_top", "align_top", "Align top", "Line up the box tops (+Z) with the reference's top.", "align", rules({{"z", "max"}}), alt('T'));
  add_tool("centre_x", "centre_x", "Centre X", "Match box centres along the reference's X axis.", "align", rules({{"x", "centre"}}), alt('X'));
  add_tool("centre_y", "centre_y", "Centre Y", "Match box centres along the reference's Y axis.", "align", rules({{"y", "centre"}}), alt('Y'));
  add_tool("centre_z", "centre_z", "Centre Z", "Match box centres along the reference's Z axis.", "align", rules({{"z", "centre"}}), alt('Z'));
  add_tool("centre_xy", "centre_xy", "Centre in XY plane", "Centre X and Y on the reference; heights are kept.", "align", rules({{"x", "centre"}, {"y", "centre"}}), alt('C'));
  add_tool("centre_ref", "centre_ref", "Centre on reference", "Box centres coincide with the reference's box centre (X, Y and Z).", "align", rules({{"x", "centre"}, {"y", "centre"}, {"z", "centre"}}), alt('O'));
  add_tool("place_top", "place_top", "Place on top", "Rest each part on the reference: on its verified upward surface datum when it has one, otherwise on its box top.", "align", rules({{"z", "on_top"}}), alt('P'));
  add_separator();
  add_tool("dist_x", "dist_x", "Distribute along X", "Equal gaps along the reference's X axis, in current order. The reference stays put.", "distribute", {{"axis", "x"}, {"mode", "equal"}});
  add_tool("dist_y", "dist_y", "Distribute along Y", "Equal gaps along the reference's Y axis, in current order. The reference stays put.", "distribute", {{"axis", "y"}, {"mode", "equal"}});
  add_tool("dist_z", "dist_z", "Distribute along Z", "Equal gaps along the reference's Z axis, in current order. The reference stays put.", "distribute", {{"axis", "z"}, {"mode", "equal"}});
  add_tool("gap", "gap", "Distribute with gap…", "An exact gap in millimetres between neighbours along one reference axis, starting at the reference.", "distribute", {{"axis", "x"}, {"mode", "gap"}});
  add_separator();
  add_tool("move", "move", "Move / rotate…", "Move and rotate the parts by exact millimetres and degrees in world, reference or local axes.", "move", {}, alt('M'));
  add_tool("snap_grid", "snap_grid", "Snap to grid…", "Snap each part's origin to a horizontal world grid (X and Y; height kept).", "snap_grid", {});
  add_separator();
  add_tool("flush", "flush", "Flush (verified faces)…", "Mate the part's verified face flush on the reference's verified face (Mate tool).", QString(), {});
  add_tool("coincident", "coincident", "Coincident (verified datums)…", "Make verified datums coincide: points, axes or frames (Mate tool). Also snaps to a datum.", QString(), {});
  add_tool("concentric", "concentric", "Concentric (verified axes)…", "Line up verified cylindrical axes (Mate tool, coincident axes).", QString(), {});
  add_tool("mount", "mount", "Fixed mount on reference", "Mount the parts on the reference with a fixed offset: they keep their place and follow it (parent/child).", "mount", {});
  add_tool("detach", "detach", "Detach", "Remove the selected parts' mounts; world positions are kept.", "detach", {});
  add_tool("group", "group", "Group", "Groups are not part of the scene contract yet.", QString(), {});
  // Tools keep their natural spacing; spare width stays at the end, not between icons.
  static_cast<QHBoxLayout *>(tools_host_->layout())->addStretch(1);

  // Inline preview bar: what will happen, its parameters, Apply / Cancel.
  preview_bar_ = new QFrame(this);
  preview_bar_->setObjectName(QStringLiteral("arrangePreviewBar"));
  preview_bar_->setFrameShape(QFrame::StyledPanel);
  auto * bar = new QVBoxLayout(preview_bar_);
  bar->setContentsMargins(8, 6, 8, 6);
  bar->setSpacing(4);
  summary_ = new QLabel(preview_bar_);
  summary_->setObjectName(QStringLiteral("arrangeSummary"));
  summary_->setWordWrap(true);
  summary_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  summary_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  bar->addWidget(summary_);

  gap_box_ = new QWidget(preview_bar_);
  auto * gap_row = new QHBoxLayout(gap_box_);
  gap_row->setContentsMargins(0, 0, 0, 0);
  auto * gap_axis = new QComboBox(gap_box_);
  gap_axis->setObjectName(QStringLiteral("arrangeGapAxis"));
  gap_axis->addItem(QStringLiteral("Along X"), "x");
  gap_axis->addItem(QStringLiteral("Along Y"), "y");
  gap_axis->addItem(QStringLiteral("Along Z"), "z");
  gap_ = spin(gap_box_, QStringLiteral("arrangeGap"), QStringLiteral("Gap "), QStringLiteral(" mm"), -1000., 10000., 10.);
  gap_row->addWidget(gap_axis);
  gap_row->addWidget(gap_);
  gap_row->addStretch(1);
  bar->addWidget(gap_box_);

  move_box_ = new QWidget(preview_bar_);
  auto * move_grid = new QHBoxLayout(move_box_);
  move_grid->setContentsMargins(0, 0, 0, 0);
  frame_ = new QComboBox(move_box_);
  frame_->setObjectName(QStringLiteral("arrangeFrame"));
  frame_->addItem(QStringLiteral("World axes"), "world");
  frame_->addItem(QStringLiteral("Reference axes"), "reference");
  frame_->addItem(QStringLiteral("Local axes"), "local");
  pivot_ = new QComboBox(move_box_);
  pivot_->setObjectName(QStringLiteral("arrangePivot"));
  pivot_->addItem(QStringLiteral("Turn each about its origin"), "each");
  pivot_->addItem(QStringLiteral("Turn about the reference"), "reference");
  const char * labels[6] = {"X ", "Y ", "Z ", "Rx ", "Ry ", "Rz "};
  auto * values = new QWidget(move_box_);
  auto * values_row = new QHBoxLayout(values);
  values_row->setContentsMargins(0, 0, 0, 0);
  for (int k = 0; k < 6; ++k) {
    move_[k] = spin(values, QStringLiteral("arrangeMove%1").arg(k), QString::fromLatin1(labels[k]),
      k < 3 ? QStringLiteral(" mm") : QStringLiteral("°"), k < 3 ? -10000. : -360., k < 3 ? 10000. : 360., 0.);
    move_[k]->setMinimumWidth(64);
    values_row->addWidget(move_[k], 1);
  }
  auto * move_column = new QVBoxLayout();
  auto * combos = new QHBoxLayout();
  combos->addWidget(frame_);
  combos->addWidget(pivot_);
  combos->addStretch(1);
  move_column->addLayout(combos);
  move_column->addWidget(values);
  move_grid->addLayout(move_column, 1);
  bar->addWidget(move_box_);

  // Align: exact offsets along the reference axes, and optionally keep the parts relative
  // to the reference afterwards (a fixed mount: they then follow it).
  align_box_ = new QWidget(preview_bar_);
  auto * align_row = new QHBoxLayout(align_box_);
  align_row->setContentsMargins(0, 0, 0, 0);
  const char * offset_labels[3] = {"ΔX ", "ΔY ", "ΔZ "};
  for (int k = 0; k < 3; ++k) {
    align_offset_[k] = spin(align_box_, QStringLiteral("arrangeAlignOffset%1").arg(QLatin1Char("XYZ"[k])),
      QString::fromUtf8(offset_labels[k]), QStringLiteral(" mm"), -10000., 10000., 0.);
    align_offset_[k]->setToolTip(QStringLiteral("Extra distance along the reference %1 axis, applied on axes the tool aligns.")
      .arg(QLatin1Char("XYZ"[k])));
    align_row->addWidget(align_offset_[k]);
  }
  keep_relative_ = new QCheckBox(QStringLiteral("Keep relative (mount on reference)"), align_box_);
  keep_relative_->setObjectName(QStringLiteral("arrangeKeepRelative"));
  keep_relative_->setToolTip(QStringLiteral("Also mount the parts on the reference with a fixed offset, so they follow it "
    "later. Off: the alignment changes their pose only."));
  align_row->addWidget(keep_relative_);
  align_row->addStretch(1);
  bar->addWidget(align_box_);

  grid_box_ = new QWidget(preview_bar_);
  auto * grid_row = new QHBoxLayout(grid_box_);
  grid_row->setContentsMargins(0, 0, 0, 0);
  grid_ = spin(grid_box_, QStringLiteral("arrangeGrid"), QStringLiteral("Grid "), QStringLiteral(" mm"), 0.1, 1000., 10.);
  grid_row->addWidget(grid_);
  grid_row->addStretch(1);
  bar->addWidget(grid_box_);

  auto * buttons = new QHBoxLayout();
  buttons->addStretch(1);
  cancel_ = new QPushButton(QStringLiteral("Cancel"), preview_bar_);
  cancel_->setObjectName(QStringLiteral("arrangeCancel"));
  cancel_->setToolTip(QStringLiteral("Discard the preview; nothing changes (Esc)."));
  apply_ = new QPushButton(QStringLiteral("Apply"), preview_bar_);
  apply_->setObjectName(QStringLiteral("arrangeApply"));
  apply_->setToolTip(QStringLiteral("Apply to every previewed part as one undoable step (Enter)."));
  buttons->addWidget(cancel_);
  buttons->addWidget(apply_);
  bar->addLayout(buttons);
  outer->addWidget(preview_bar_);
  preview_bar_->hide();

  const auto params_changed = [this]() {
    if (refreshing_ || !controller_ || !controller_->active()) return;
    controller_->set_params(current_params());
  };
  connect(gap_axis, qOverload<int>(&QComboBox::currentIndexChanged), this, params_changed);
  connect(gap_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, params_changed);
  connect(frame_, qOverload<int>(&QComboBox::currentIndexChanged), this, params_changed);
  connect(pivot_, qOverload<int>(&QComboBox::currentIndexChanged), this, params_changed);
  for (auto * box : move_) connect(box, qOverload<double>(&QDoubleSpinBox::valueChanged), this, params_changed);
  connect(grid_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, params_changed);
  for (auto * box : align_offset_) connect(box, qOverload<double>(&QDoubleSpinBox::valueChanged), this, params_changed);
  connect(keep_relative_, &QCheckBox::toggled, this, params_changed);
  connect(apply_, &QPushButton::clicked, this, [this]() {
    QString error;
    if (controller_ && !controller_->apply(&error)) summary_->setText(error.toHtmlEscaped());
  });
  connect(cancel_, &QPushButton::clicked, this, [this]() { if (controller_) controller_->cancel(); });
  if (controller_) connect(controller_, &ArrangeController::changed, this, &ArrangeToolbar::refresh);
  set_selection({}, QString());
}

QAction * ArrangeToolbar::add_tool(const QString & tool, const QString & icon, const QString & text, const QString & tip,
  const QString & op, const QJsonObject & params, const QKeySequence & shortcut)
{
  auto * action = new QAction(arrangeIcon(icon), text, this);
  action->setObjectName(QStringLiteral("arrange_%1").arg(tool));
  action->setCheckable(!op.isEmpty());
  if (!shortcut.isEmpty()) {
    action->setShortcut(shortcut);
    action->setShortcutContext(Qt::WindowShortcut);
  }
  const QString full = shortcut.isEmpty() ? QStringLiteral("<b>%1</b><br/>%2").arg(text.toHtmlEscaped(), tip.toHtmlEscaped())
    : QStringLiteral("<b>%1</b> (%2)<br/>%3").arg(text.toHtmlEscaped(), shortcut.toString(QKeySequence::NativeText), tip.toHtmlEscaped());
  tips_.insert(tool, full);
  action->setToolTip(full);
  action->setStatusTip(tip);
  if (!op.isEmpty()) plans_.insert(tool, {op, params});
  connect(action, &QAction::triggered, this, [this, tool]() { start(tool); });
  addAction(action);  // shortcuts work whether the button is shown or in the More menu
  actions_.insert(tool, action);
  auto * button = new QToolButton(tools_host_);
  button->setDefaultAction(action);
  button->setObjectName(QStringLiteral("arrangeButton_%1").arg(tool));
  button->setAccessibleName(text);
  button->setToolButtonStyle(Qt::ToolButtonIconOnly);
  button->setIconSize(QSize(16, 16));
  button->setAutoRaise(true);
  button->setFixedSize(28, 28);
  button->setFocusPolicy(Qt::TabFocus);
  tools_host_->layout()->addWidget(button);
  tool_widgets_ << button;
  tool_actions_ << action;
  return action;
}

void ArrangeToolbar::add_separator()
{
  auto * line = new QFrame(tools_host_);
  line->setFrameShape(QFrame::VLine);
  line->setFixedWidth(9);
  tools_host_->layout()->addWidget(line);
  tool_widgets_ << line;
  tool_actions_ << nullptr;
}

bool ArrangeToolbar::eventFilter(QObject * watched, QEvent * event)
{
  if (watched == tools_host_ && (event->type() == QEvent::Resize || event->type() == QEvent::Show)) relayout_tools();
  return QFrame::eventFilter(watched, event);
}

void ArrangeToolbar::relayout_tools()
{
  // Show tools in order while they fit; the rest go to the More menu (in order,
  // with group separators), so every tool stays reachable at any width.
  const int spacing = tools_host_->layout()->spacing();
  const int more_width = more_->sizeHint().width() + spacing;
  int total = 0;
  for (auto * widget : tool_widgets_) total += widget->sizeHint().width() + spacing;
  const int available = tools_host_->width() - (total > tools_host_->width() ? more_width : 0);
  int used = 0;
  bool overflow = false;
  QMenu * menu = more_->menu();
  menu->clear();
  for (int k = 0; k < tool_widgets_.size(); ++k) {
    auto * widget = tool_widgets_.at(k);
    const int width = widget->sizeHint().width() + spacing;
    overflow = overflow || used + width > available;
    widget->setVisible(!overflow);
    if (!overflow) { used += width; continue; }
    if (QAction * moved = tool_actions_.at(k)) menu->addAction(moved);
    else if (!menu->isEmpty()) menu->addSeparator();
  }
  more_->setVisible(overflow);
}

void ArrangeToolbar::set_selection(const QList<QPair<QString, QString>> & members, const QString & reference)
{
  QMenu * menu = chip_->menu();
  menu->clear();
  QString reference_name;
  for (const auto & member : members) {
    if (member.first == reference) reference_name = member.second;
    auto * item = menu->addAction(member.second);
    item->setCheckable(true);
    item->setChecked(member.first == reference);
    const QString id = member.first;
    connect(item, &QAction::triggered, this, [this, id]() { emit reference_chosen(id); });
  }
  if (members.size() >= 2) {
    menu->insertSection(menu->actions().value(0), QStringLiteral("Reference (stays put)"));
    const QString full = QStringLiteral("%1 selected · ref: %2").arg(members.size()).arg(reference_name);
    // Elide the part name, never the count: "4 selected · ref: Industrial Wor…".
    chip_->setText(chip_->fontMetrics().elidedText(full, Qt::ElideRight, chip_->maximumWidth() - 44));
    chip_->setAccessibleName(full);
    chip_->setToolTip(QStringLiteral("%1 parts selected. The reference %2 never moves; choose another from this menu.")
      .arg(members.size()).arg(reference_name));
    chip_->setEnabled(true);
  } else {
    chip_->setText(members.isEmpty() ? QStringLiteral("Arrange: select parts") : QStringLiteral("1 selected · Ctrl+click to add"));
    chip_->setToolTip(QStringLiteral("Click the reference part, then Ctrl+click the parts to arrange "
      "(3D view or Scene Hierarchy; Shift+click selects a range in the hierarchy)."));
    chip_->setEnabled(false);
  }
  refresh();
}

QJsonObject ArrangeToolbar::current_params() const
{
  const auto plan = plans_.value(pending_tool_);
  QJsonObject params = plan.second;
  if (plan.first == QStringLiteral("distribute") && params.value("mode").toString() == QStringLiteral("gap")) {
    auto * axis = gap_box_->findChild<QComboBox *>(QStringLiteral("arrangeGapAxis"));
    params.insert("axis", axis ? axis->currentData().toString() : QStringLiteral("x"));
    params.insert("gap_m", gap_->value() / 1000.);
  } else if (plan.first == QStringLiteral("move")) {
    params.insert("frame", frame_->currentData().toString());
    params.insert("pivot", pivot_->currentData().toString());
    params.insert("translate_m", QJsonArray{move_[0]->value() / 1000., move_[1]->value() / 1000., move_[2]->value() / 1000.});
    params.insert("rotate_deg", QJsonArray{move_[3]->value(), move_[4]->value(), move_[5]->value()});
  } else if (plan.first == QStringLiteral("snap_grid")) {
    params.insert("step_m", grid_->value() / 1000.);
  } else if (plan.first == QStringLiteral("align")) {
    params.insert("offset_m", QJsonArray{align_offset_[0]->value() / 1000., align_offset_[1]->value() / 1000.,
      align_offset_[2]->value() / 1000.});
    params.insert("keep_relative", keep_relative_->isChecked());
  }
  return params;
}

void ArrangeToolbar::start(const QString & tool)
{
  if (!controller_) return;
  if (!plans_.contains(tool)) {
    const QString reason = extra_ ? extra_(tool) : QString();
    if (reason.isEmpty()) emit mate_requested(tool);
    refresh();
    return;
  }
  if (controller_->active()) controller_->cancel(QStringLiteral("Switched tool; nothing changed."));
  pending_tool_ = tool;
  update_parameter_visibility();
  QString error;
  if (!controller_->start(plans_.value(tool).first, current_params(), actions_.value(tool)->text().remove(QChar(0x2026)), &error)) {
    pending_tool_.clear();
    summary_->setText(error.toHtmlEscaped());
  }
  refresh();
}

void ArrangeToolbar::update_parameter_visibility()
{
  const auto plan = plans_.value(pending_tool_);
  gap_box_->setVisible(plan.first == QStringLiteral("distribute") && plan.second.value("mode").toString() == QStringLiteral("gap"));
  move_box_->setVisible(plan.first == QStringLiteral("move"));
  grid_box_->setVisible(plan.first == QStringLiteral("snap_grid"));
  align_box_->setVisible(plan.first == QStringLiteral("align"));
  if (plan.first == QStringLiteral("align")) {
    // Offsets are per operation: a new tool starts from zero, never from a previous tool's values.
    for (auto * box : align_offset_) { const QSignalBlocker blocker(box); box->setValue(0.); }
    const QSignalBlocker blocker(keep_relative_);
    keep_relative_->setChecked(false);
  }
}

void ArrangeToolbar::refresh()
{
  if (!controller_) return;
  refreshing_ = true;
  for (auto it = actions_.constBegin(); it != actions_.constEnd(); ++it) {
    const QString tool = it.key();
    QString reason;
    if (plans_.contains(tool)) {
      const auto plan = plans_.value(tool);
      reason = controller_->availability(plan.first, plan.second);
    } else {
      reason = extra_ ? extra_(tool) : QStringLiteral("Unavailable.");
    }
    it.value()->setEnabled(reason.isEmpty());
    it.value()->setToolTip(reason.isEmpty() ? tips_.value(tool)
      : QStringLiteral("%1<br/><i>Unavailable: %2</i>").arg(tips_.value(tool), reason.toHtmlEscaped()));
    it.value()->setChecked(controller_->active() && tool == pending_tool_);
  }
  const bool active = controller_->active();
  if (!active) pending_tool_.clear();
  preview_bar_->setVisible(active);
  if (active) {
    const auto & preview = controller_->preview();
    QString text = QStringLiteral("<b>%1</b> — %2").arg(controller_->label().toHtmlEscaped(),
      (preview.ok ? preview.summary : preview.error).toHtmlEscaped());
    if (preview.ok) text += QStringLiteral("<br/>") + controller_->status().toHtmlEscaped();
    summary_->setText(text);
    apply_->setEnabled(preview.ok);
    apply_->setToolTip(preview.ok ? QStringLiteral("Apply to every previewed part as one undoable step (Enter).")
                                  : preview.error);
  }
  refreshing_ = false;
}

}  // namespace workcell_builder
