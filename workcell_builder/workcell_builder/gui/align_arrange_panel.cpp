#include "align_arrange_panel.hpp"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QSignalBlocker>
#include <QToolButton>
#include <QVBoxLayout>

#include "align_arrange_controller.hpp"
#include "workcell_builder_ui_utils.hpp"

namespace workcell_builder
{
namespace
{

QLabel * section(const QString & text, QWidget * parent)
{
  auto * label = new QLabel(QStringLiteral("<b>%1</b>").arg(text), parent);
  label->setWordWrap(true);
  return label;
}

QLabel * wrapped(QWidget * parent, const QString & object_name)
{
  auto * label = new QLabel(parent);
  label->setObjectName(object_name);
  label->setWordWrap(true);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);  // wraps, never widens the panel
  return label;
}

}  // namespace

AlignArrangePanel::AlignArrangePanel(AlignArrangeController * controller, QWidget * parent)
: QFrame(parent), controller_(controller)
{
  setObjectName(QStringLiteral("sceneBuilderAlignPanel"));
  setFrameShape(QFrame::NoFrame);
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setSpacing(6);

  // Like the Mate tool: title and the Apply / Cancel actions first.
  auto * header = new QHBoxLayout();
  header->addWidget(new QLabel(QStringLiteral("<b>Align &amp; Arrange</b>"), this), 1);
  clear_ = new QPushButton(QStringLiteral("C&lear"), this);
  clear_->setObjectName(QStringLiteral("alignClear"));
  clear_->setToolTip(QStringLiteral("Clear every operation and offset; the reference stays chosen."));
  cancel_ = new QPushButton(QStringLiteral("Cancel"), this);
  cancel_->setObjectName(QStringLiteral("alignCancel"));
  cancel_->setToolTip(QStringLiteral("Leave Align & Arrange without changing the scene (Esc)."));
  apply_ = new QPushButton(QStringLiteral("&Apply"), this);
  apply_->setObjectName(QStringLiteral("alignApply"));
  header->addWidget(clear_);
  header->addWidget(cancel_);
  header->addWidget(apply_);
  layout->addLayout(header);

  intro_ = wrapped(this, QStringLiteral("alignIntro"));
  intro_->setText(QStringLiteral("Lines the part up with a reference by bounding boxes, in the reference's own axes "
    "(left/right −X/+X, front/back −Y/+Y). Not a CAD face mate: use <i>Mate faces</i> for verified faces."));
  intro_->setTextInteractionFlags(Qt::NoTextInteraction);
  layout->addWidget(intro_);
  view_note_ = wrapped(this, QStringLiteral("alignViewNote"));
  view_note_->hide();
  layout->addWidget(view_note_);

  auto * parts = new QGridLayout();
  parts->setHorizontalSpacing(8);
  parts->addWidget(new QLabel(QStringLiteral("Part to move"), this), 0, 0);
  moving_ = wrapped(this, QStringLiteral("alignMoving"));
  parts->addWidget(moving_, 0, 1);
  auto * reference_label = new QLabel(QStringLiteral("&Reference"), this);
  parts->addWidget(reference_label, 1, 0);
  reference_ = new QComboBox(this);
  reference_->setObjectName(QStringLiteral("alignReference"));
  reference_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  reference_->setMinimumContentsLength(14);
  reference_->setToolTip(QStringLiteral("The part to line up with. You can also click it in the 3D view or the Scene "
    "Hierarchy. Robots, generated tool visuals and task zones cannot be references."));
  reference_label->setBuddy(reference_);
  parts->addWidget(reference_, 1, 1);
  parts->setColumnStretch(1, 1);
  layout->addLayout(parts);

  // The numbers come first, like the Mate panel: visible without scrolling at 1280 x 720.
  status_ = wrapped(this, QStringLiteral("alignStatus"));
  status_->setAccessibleName(QStringLiteral("Align & Arrange status"));
  status_->setStyleSheet(QStringLiteral("font-weight: 600;"));
  preview_ = wrapped(this, QStringLiteral("alignPreview"));
  method_ = wrapped(this, QStringLiteral("alignMethod"));
  method_->setStyleSheet(studioQss(QStringLiteral("color: @textSecondary;")));
  layout->addWidget(status_);
  layout->addWidget(preview_);
  layout->addWidget(method_);

  layout->addWidget(section(QStringLiteral("Centre"), this));
  centre_both_ = new QToolButton(this);
  centre_both_->setText(QStringLiteral("Centre on reference"));
  centre_both_->setObjectName(QStringLiteral("alignCentreBoth"));
  centre_both_->setCheckable(true);
  centre_both_->setToolTip(QStringLiteral("Centre horizontally: the box centres match along the reference X and Y. "
    "Height is kept unless you also choose a Z operation."));
  centre_both_->setProperty("baseToolTip", centre_both_->toolTip());
  centre_both_->setFocusPolicy(Qt::StrongFocus);
  connect(centre_both_, &QToolButton::clicked, this, [this]() { if (controller_) controller_->centre_on_reference(); });
  layout->addWidget(makeActionBar(this, {
    centre_both_,
    add_operation(QStringLiteral("Centre X"), 0, QStringLiteral("centre"),
      QStringLiteral("Match the box centres along the reference X axis."), QStringLiteral("alignCentreX")),
    add_operation(QStringLiteral("Centre Y"), 1, QStringLiteral("centre"),
      QStringLiteral("Match the box centres along the reference Y axis."), QStringLiteral("alignCentreY")),
    add_operation(QStringLiteral("Centre Z"), 2, QStringLiteral("centre"),
      QStringLiteral("Match the box centres along the reference Z axis."), QStringLiteral("alignCentreZ")),
  }));

  layout->addWidget(section(QStringLiteral("Align edges"), this));
  layout->addWidget(makeActionBar(this, {
    add_operation(QStringLiteral("Left"), 0, QStringLiteral("min"),
      QStringLiteral("Align left: the part's −X box face on the reference's −X box face."), QStringLiteral("alignLeft")),
    add_operation(QStringLiteral("Right"), 0, QStringLiteral("max"),
      QStringLiteral("Align right: the part's +X box face on the reference's +X box face."), QStringLiteral("alignRight")),
    add_operation(QStringLiteral("Front"), 1, QStringLiteral("min"),
      QStringLiteral("Align front: the part's −Y box face on the reference's −Y box face."), QStringLiteral("alignFront")),
    add_operation(QStringLiteral("Back"), 1, QStringLiteral("max"),
      QStringLiteral("Align back: the part's +Y box face on the reference's +Y box face."), QStringLiteral("alignBack")),
    add_operation(QStringLiteral("Bottom"), 2, QStringLiteral("min"),
      QStringLiteral("Align bottom: the part's −Z box face on the reference's −Z box face."), QStringLiteral("alignBottom")),
    add_operation(QStringLiteral("Top"), 2, QStringLiteral("max"),
      QStringLiteral("Align top: the part's +Z box face on the reference's +Z box face."), QStringLiteral("alignTop")),
  }));

  layout->addWidget(section(QStringLiteral("Stack"), this));
  mate_ = new QPushButton(QStringLiteral("Mate faces…"), this);
  mate_->setObjectName(QStringLiteral("alignMateFaces"));
  mate_->setToolTip(QStringLiteral("Flush on verified faces (datums), not boxes: opens the Mate tool for this part. "
    "Nothing from this panel is applied."));
  connect(mate_, &QPushButton::clicked, this, &AlignArrangePanel::mate_requested);
  layout->addWidget(makeActionBar(this, {
    add_operation(QStringLiteral("Place on top"), 2, QStringLiteral("on_top"),
      QStringLiteral("Rest the part's box bottom on the reference: on its verified upward surface datum when it has "
        "one (for example a workbench top), otherwise on its box top. Combine with Centre to place it centred."),
      QStringLiteral("alignPlaceOnTop")),
    mate_,
  }));

  layout->addWidget(section(QStringLiteral("Offset (mm, reference axes)"), this));
  auto * offsets = new QHBoxLayout();
  const char * prefixes[3] = {"X ", "Y ", "Z "};
  for (int k = 0; k < 3; ++k) {
    auto * spin = new QDoubleSpinBox(this);
    spin->setObjectName(QStringLiteral("alignOffset%1").arg(QLatin1Char("XYZ"[k])));
    spin->setPrefix(QString::fromLatin1(prefixes[k]));
    spin->setRange(-10000., 10000.);
    spin->setDecimals(1);
    spin->setSingleStep(1.);
    spin->setKeyboardTracking(false);
    spin->setAccelerated(true);
    spin->setMinimumWidth(80);
    connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this, k](double value) {
      if (!refreshing_ && controller_) controller_->set_offset_mm(static_cast<AlignArrangeController::Axis>(k), value);
    });
    offset_[k] = spin;
    offsets->addWidget(spin, 1);
  }
  layout->addLayout(offsets);

  layout->addWidget(section(QStringLiteral("Result"), this));
  once_ = new QRadioButton(QStringLiteral("Align once — changes the pose only"), this);
  once_->setObjectName(QStringLiteral("alignOnce"));
  once_->setToolTip(QStringLiteral("Moves the part now. It does not follow the reference later."));
  keep_ = new QRadioButton(QStringLiteral("Keep relative — follows the reference"), this);
  keep_->setObjectName(QStringLiteral("alignKeepRelative"));
  auto * group = new QButtonGroup(this);
  group->addButton(once_);
  group->addButton(keep_);
  once_->setChecked(true);
  connect(keep_, &QRadioButton::toggled, this, [this](bool on) {
    if (!refreshing_ && controller_) controller_->set_keep_relative(on);
  });
  layout->addWidget(once_);
  layout->addWidget(keep_);

  layout->addStretch(1);

  connect(reference_, qOverload<int>(&QComboBox::activated), this, [this](int index) {
    if (!refreshing_ && controller_ && index >= 0) controller_->set_reference(reference_->itemData(index).toString());
  });
  connect(apply_, &QPushButton::clicked, this, [this]() {
    QString error;
    if (controller_ && !controller_->apply(&error) && status_) status_->setText(error);
  });
  connect(clear_, &QPushButton::clicked, this, [this]() { if (controller_) controller_->clear_rules(); });
  connect(cancel_, &QPushButton::clicked, this, [this]() { if (controller_) controller_->cancel(); });
  if (controller_) connect(controller_, &AlignArrangeController::changed, this, &AlignArrangePanel::refresh);
  keepWrappedLabelHeights(this);
}

QToolButton * AlignArrangePanel::add_operation(const QString & text, int axis, const QString & rule,
  const QString & tooltip, const QString & object_name)
{
  auto * button = new QToolButton(this);
  button->setText(text);
  button->setObjectName(object_name);
  button->setCheckable(true);
  button->setToolTip(tooltip);
  button->setProperty("baseToolTip", tooltip);
  button->setAccessibleName(AlignArrangeController::rule_label(static_cast<AlignArrangeController::Axis>(axis), rule));
  button->setFocusPolicy(Qt::StrongFocus);
  connect(button, &QToolButton::clicked, this, [this, axis, rule]() {
    if (controller_) controller_->set_rule(static_cast<AlignArrangeController::Axis>(axis), rule);
  });
  operations_.append({button, axis, rule});
  return button;
}

void AlignArrangePanel::set_reference_candidates(const QList<QPair<QString, QString>> & candidates)
{
  const QSignalBlocker blocker(reference_);
  reference_->clear();
  reference_->addItem(QStringLiteral("Choose a reference part…"), QString());
  for (const auto & candidate : candidates) reference_->addItem(candidate.second, candidate.first);
  refresh();
}

void AlignArrangePanel::set_view_note(const QString & note)
{
  view_note_->setText(note.toHtmlEscaped());
  view_note_->setVisible(!note.isEmpty());
}

void AlignArrangePanel::refresh()
{
  if (!controller_) return;
  refreshing_ = true;
  const auto & c = *controller_;
  moving_->setText(QStringLiteral("<b>%1</b>").arg(c.display_name(c.moving()).toHtmlEscaped()));
  {
    const QSignalBlocker blocker(reference_);
    const int index = reference_->findData(c.reference());
    reference_->setCurrentIndex(index >= 0 ? index : 0);
  }
  const bool has_reference = !c.reference().isEmpty();
  // Disabled operations say why (Qt shows tooltips on disabled buttons).
  const auto explain = [has_reference](QAbstractButton * button) {
    const QString base = button->property("baseToolTip").toString();
    button->setEnabled(has_reference);
    button->setToolTip(has_reference ? base : QStringLiteral("Choose a reference part first. %1").arg(base));
  };
  for (auto & operation : operations_) {
    const auto axis = static_cast<AlignArrangeController::Axis>(operation.axis);
    explain(operation.button);
    operation.button->setChecked(c.rule(axis) == operation.rule);
  }
  explain(centre_both_);
  centre_both_->setChecked(c.rule(AlignArrangeController::Axis::X) == QStringLiteral("centre") &&
    c.rule(AlignArrangeController::Axis::Y) == QStringLiteral("centre"));
  for (int k = 0; k < 3; ++k) {
    const auto axis = static_cast<AlignArrangeController::Axis>(k);
    const bool used = !c.rule(axis).isEmpty();
    const QSignalBlocker blocker(offset_[k]);
    offset_[k]->setValue(c.offset_mm(axis));
    offset_[k]->setEnabled(used);
    offset_[k]->setToolTip(used
      ? QStringLiteral("Extra distance along the reference %1 axis after %2, in millimetres.")
          .arg(QLatin1Char("XYZ"[k])).arg(AlignArrangeController::rule_label(axis, c.rule(axis)))
      : QStringLiteral("Choose a %1 operation first: offsets apply only to aligned axes.").arg(QLatin1Char("XYZ"[k])));
  }
  const QString keep_blocker = c.keep_relative_blocker();
  keep_->setEnabled(keep_blocker.isEmpty());
  keep_->setToolTip(keep_blocker.isEmpty()
    ? QStringLiteral("Also mounts the part on the reference with a fixed offset (the Mount To relation), so it follows "
        "when the reference moves. Detach it later in Mount To.")
    : keep_blocker);
  (c.keep_relative() ? keep_ : once_)->setChecked(true);

  const auto & preview = c.preview();
  const auto report = preview.report;
  if (preview.ok) {
    QString text = preview.summary.toHtmlEscaped();
    if (!preview.followers.isEmpty()) {
      text += QStringLiteral("<br/>Also moves (mounted on it): %1.").arg(preview.followers.join(QStringLiteral(", ")).toHtmlEscaped());
    }
    preview_->setText(text);
    const auto sources = report.value(QStringLiteral("sources")).toObject();
    QStringList method{QStringLiteral("Bounding-box alignment in %1's axes — not a CAD face mate.")
      .arg(c.display_name(c.reference()).toHtmlEscaped())};
    method << QStringLiteral("Part: %1.").arg(sources.value(QStringLiteral("moving")).toString().toHtmlEscaped());
    method << QStringLiteral("Reference: %1.").arg(sources.value(QStringLiteral("reference")).toString().toHtmlEscaped());
    if (sources.contains(QStringLiteral("top"))) {
      method << QStringLiteral("Top: %1.").arg(sources.value(QStringLiteral("top")).toString().toHtmlEscaped());
    }
    for (const auto & note : report.value(QStringLiteral("notes")).toArray()) method << QStringLiteral("⚠ %1.").arg(note.toString().toHtmlEscaped());
    method_->setText(method.join(QStringLiteral("<br/>")));
  } else {
    preview_->setText(QStringLiteral("No preview yet: nothing has moved."));
    method_->clear();
  }
  method_->setVisible(!method_->text().isEmpty());
  status_->setText(c.status().toHtmlEscaped());
  apply_->setEnabled(preview.ok);
  apply_->setToolTip(preview.ok
    ? (c.keep_relative() ? QStringLiteral("Move the part and keep it relative to the reference (undoable).")
                         : QStringLiteral("Move the part to the ghost position (undoable)."))
    : (preview.error.isEmpty() ? QStringLiteral("Choose a reference and at least one operation first.") : preview.error));
  clear_->setEnabled(c.has_rules());
  mate_->setEnabled(c.active());
  refreshing_ = false;
}

}  // namespace workcell_builder
