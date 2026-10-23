#include "mate_tool_panel.hpp"

#include <QButtonGroup>
#include <QCheckBox>
#include <QMenu>
#include <QToolButton>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <QtMath>

#include "mate_tool_controller.hpp"
#include "workcell_builder_ui_utils.hpp"

namespace workcell_builder
{

MateToolPanel::MateToolPanel(MateToolController * controller, QWidget * parent)
: QFrame(parent), controller_(controller)
{
  setObjectName(QStringLiteral("sceneBuilderMatePanel"));
  setFrameShape(QFrame::NoFrame);
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setSpacing(6);

  // Like a CAD property manager: title and the Apply / Cancel actions first,
  // then the two picks and the preview, then the detailed fields.
  auto * header = new QHBoxLayout();
  auto * title = new QLabel(QStringLiteral("<b>Mate</b> — place a part by its faces"), this);
  title->setObjectName(QStringLiteral("mateTitle"));
  header->addWidget(title, 1);
  reset_ = new QPushButton(QStringLiteral("Reset"), this);
  reset_->setObjectName(QStringLiteral("mateReset"));
  reset_->setToolTip(QStringLiteral("Back to the proposal: keep the position on the face, close the gap, no tilt."));
  cancel_ = new QPushButton(QStringLiteral("Cancel"), this);
  cancel_->setObjectName(QStringLiteral("mateCancel"));
  cancel_->setToolTip(QStringLiteral("Leave the Mate tool without changing the scene (Esc)."));
  apply_ = new QPushButton(QStringLiteral("Apply"), this);
  apply_->setObjectName(QStringLiteral("mateApply"));
  header->addWidget(reset_);
  header->addWidget(cancel_);
  header->addWidget(apply_);
  layout->addLayout(header);
  intro_ = new QLabel(this);
  intro_->setWordWrap(true);
  intro_->setObjectName(QStringLiteral("mateIntro"));
  layout->addWidget(intro_);
  backend_note_ = new QLabel(this);
  backend_note_->setObjectName(QStringLiteral("mateBackendNote"));
  backend_note_->setWordWrap(true);
  layout->addWidget(backend_note_);
  status_ = new QLabel(this);
  status_->setObjectName(QStringLiteral("mateStatus"));
  status_->setWordWrap(true);
  status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(status_);

  child_ = build_row(QStringLiteral("1  Part to move"), QStringLiteral("mateChild"));
  parent_ = build_row(QStringLiteral("2  Part it sits on"), QStringLiteral("mateParent"));
  for (auto * row : {&child_, &parent_}) {
    layout->addWidget(row->title);
    auto * line = new QHBoxLayout();
    line->addWidget(row->value, 1);
    line->addWidget(row->replace);
    layout->addLayout(line);
    layout->addWidget(row->datums);
  }

  // Hidden faces (a bin bottom on a table): datum markers are always drawn on
  // top; Isolate and Look at a face expose the real face for a geometry click.
  auto * access = new QHBoxLayout();
  isolate_ = new QCheckBox(QStringLiteral("Isolate part"), this);
  isolate_->setObjectName(QStringLiteral("mateIsolate"));
  isolate_->setToolTip(QStringLiteral("Show only the part of the current step in the 3D view, so hidden faces can be reached."));
  face_view_ = new QToolButton(this);
  face_view_->setObjectName(QStringLiteral("mateFaceView"));
  face_view_->setText(QStringLiteral("Look at a face ▾"));
  face_view_->setPopupMode(QToolButton::InstantPopup);
  face_view_->setToolTip(QStringLiteral("Turn the 3D view to look straight at one of the part's datums from outside."));
  face_view_->setMenu(new QMenu(face_view_));
  access->addWidget(isolate_);
  access->addStretch(1);
  access->addWidget(face_view_);
  layout->addLayout(access);
  connect(isolate_, &QCheckBox::toggled, this, [this](bool on) { if (!refreshing_ && controller_) controller_->set_isolate(on); });
  connect(face_view_->menu(), &QMenu::triggered, this, [this](QAction * action) {
    QString error;
    if (controller_ && !controller_->view_datum(action->data().toString(), &error)) status_->setText(error);
  });

  auto * preview_title = new QLabel(QStringLiteral("<b>Preview</b>"), this);
  layout->addWidget(preview_title);
  preview_ = new QLabel(this);
  preview_->setObjectName(QStringLiteral("matePreview"));
  preview_->setWordWrap(true);
  preview_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  preview_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  layout->addWidget(preview_);

  auto * alignment_title = new QLabel(QStringLiteral("<b>Alignment</b>"), this);
  layout->addWidget(alignment_title);
  flush_ = new QRadioButton(QStringLiteral("Flush — faces touch, normals opposed"), this);
  flush_->setObjectName(QStringLiteral("mateFlush"));
  coincident_ = new QRadioButton(QStringLiteral("Coincident — datum frames line up"), this);
  coincident_->setObjectName(QStringLiteral("mateCoincident"));
  auto * group = new QButtonGroup(this);
  group->addButton(flush_); group->addButton(coincident_);
  layout->addWidget(flush_);
  layout->addWidget(coincident_);
  alignment_note_ = new QLabel(this);
  alignment_note_->setObjectName(QStringLiteral("mateAlignmentNote"));
  alignment_note_->setWordWrap(true);
  layout->addWidget(alignment_note_);
  connect(flush_, &QRadioButton::toggled, this, [this](bool on) {
    if (on && !refreshing_ && controller_) controller_->set_alignment(QStringLiteral("flush"));
  });
  connect(coincident_, &QRadioButton::toggled, this, [this](bool on) {
    if (on && !refreshing_ && controller_) controller_->set_alignment(QStringLiteral("coincident"));
  });

  auto * offset_title = new QLabel(QStringLiteral("<b>Position on the face</b><br/>Measured in the fixed face's frame; "
    "the proposal keeps where the part is now and closes the gap."), this);
  offset_title->setWordWrap(true);
  layout->addWidget(offset_title);
  auto * grid = new QGridLayout();
  grid->setHorizontalSpacing(6);
  const std::array<const char *, 6> names{{"mateOffsetX", "mateOffsetY", "mateOffsetGap",
    "mateOffsetRoll", "mateOffsetPitch", "mateOffsetYaw"}};
  const std::array<const char *, 6> labels{{"Along X", "Along Y", "Gap (normal)", "Tilt roll", "Tilt pitch", "Turn about normal"}};
  const std::array<int, 6> order{{0, 1, 2, 5, 3, 4}};
  for (int i = 0; i < 6; ++i) {
    const int k = order[i];
    auto * spin = new QDoubleSpinBox(this);
    spin->setObjectName(QString::fromLatin1(names[k]));
    spin->setRange(k < 3 ? -1000. : -360., k < 3 ? 1000. : 360.);
    spin->setDecimals(k < 3 ? 4 : 2);
    spin->setSingleStep(k < 3 ? 0.005 : 1.);
    spin->setSuffix(k < 3 ? QStringLiteral(" m") : QStringLiteral("°"));
    spin->setKeyboardTracking(false);
    spin->setMinimumWidth(96);
    offset_[k] = spin;
    auto * label = new QLabel(QString::fromLatin1(labels[k]), this);
    grid->addWidget(label, i / 2, (i % 2) * 2);
    grid->addWidget(spin, i / 2, (i % 2) * 2 + 1);
    connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
      if (!refreshing_ && controller_) controller_->set_offset(offset_from_fields());
    });
  }
  grid->setColumnStretch(1, 1);
  grid->setColumnStretch(3, 1);
  layout->addLayout(grid);

  manual_ = new QPushButton(QStringLiteral("Choose datums in Mount To…"), this);
  manual_->setObjectName(QStringLiteral("mateOpenMountDialog"));
  manual_->setToolTip(QStringLiteral("Open the Mount To / Relative Pose dialog to choose parent and datums from lists."));
  connect(manual_, &QPushButton::clicked, this, &MateToolPanel::manual_dialog_requested);
  layout->addWidget(manual_);
  layout->addStretch(1);
  connect(reset_, &QPushButton::clicked, this, [this]() { if (controller_) controller_->reset_offset(); });
  connect(cancel_, &QPushButton::clicked, this, [this]() { if (controller_) controller_->cancel(); });
  connect(apply_, &QPushButton::clicked, this, [this]() {
    QString error;
    if (controller_ && !controller_->apply(&error)) {
      preview_->setText(QStringLiteral("Cannot apply: %1").arg(error));
    }
  });
  if (controller_) connect(controller_, &MateToolController::changed, this, &MateToolPanel::refresh);
  setStyleSheet(studioQss(QStringLiteral(
    "QFrame#sceneBuilderMatePanel { background: @surface; border: 1px solid @accent; border-radius: @radiusL; }"
    "QLabel#mateStatus { color: @accent; font-weight: 600; }"
    "QLabel#mateBackendNote { color: @warning; background: @warningSoft; border-radius: @radiusM; padding: 4px 6px; }"
    "QPushButton#mateApply { background: @accent; color: @accentText; border: 1px solid @accent; border-radius: @radiusM; "
    "padding: 5px 14px; font-weight: 600; }"
    "QPushButton#mateApply:hover { background: @accentHover; }"
    "QPushButton#mateApply:disabled { background: @surfaceSunken; color: @textMuted; border-color: @border; font-weight: 400; }"
    "QLabel#matePreview { background: @surfaceSunken; border: 1px solid @border; border-radius: @radiusM; padding: 6px; }"
    "QLabel#mateChildValue, QLabel#mateParentValue { background: @surfaceSunken; border: 1px solid @border; "
    "border-radius: @radiusM; padding: 4px 6px; }")));
  refresh();
}

MateToolPanel::Row MateToolPanel::build_row(const QString & title, const QString & object_prefix)
{
  Row row;
  row.title = new QLabel(QStringLiteral("<b>%1</b>").arg(title.toHtmlEscaped()), this);
  row.value = new QLabel(this);
  row.value->setObjectName(object_prefix + QStringLiteral("Value"));
  row.value->setWordWrap(true);
  row.value->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  row.replace = new QPushButton(QStringLiteral("Replace"), this);
  row.replace->setObjectName(object_prefix + QStringLiteral("Replace"));
  row.datums = new QComboBox(this);
  row.datums->setObjectName(object_prefix + QStringLiteral("Datum"));
  row.datums->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  row.datums->setMinimumContentsLength(12);
  const bool child = object_prefix == QStringLiteral("mateChild");
  connect(row.replace, &QPushButton::clicked, this, [this, child]() {
    if (controller_) controller_->replace(child ? MateToolController::Role::Child : MateToolController::Role::Parent);
  });
  connect(row.datums, qOverload<int>(&QComboBox::activated), this, [this, child, combo = row.datums](int index) {
    if (refreshing_ || !controller_) return;
    const QString datum = combo->itemData(index).toString();
    const QString asset = combo->property("assetId").toString();
    if (datum.isEmpty() || asset.isEmpty()) return;
    QString error;
    if (!controller_->choose_datum(child ? MateToolController::Role::Child : MateToolController::Role::Parent, asset, datum, &error)) {
      status_->setText(error);
    }
  });
  return row;
}

void MateToolPanel::fill_datums(QComboBox * combo, const QString & asset_id, const QString & selected)
{
  const QSignalBlocker blocker(combo);
  combo->clear();
  combo->setProperty("assetId", asset_id);
  if (!controller_ || asset_id.isEmpty() || !controller_->frames().contains(asset_id)) {
    combo->addItem(QStringLiteral("Click a part first, then you can pick its datum here"), QString());
    combo->setEnabled(false);
    return;
  }
  combo->setEnabled(true);
  combo->addItem(QStringLiteral("Choose a datum of %1 manually…").arg(controller_->display_name(asset_id)), QString());
  for (const auto & datum : controller_->frames().value(asset_id).datums) {
    combo->addItem(QStringLiteral("%1 — %2").arg(datum.name, mate_datum_display(datum)), datum.name);
    combo->setItemData(combo->count() - 1, datum.source, Qt::ToolTipRole);
  }
  const int index = combo->findData(selected);
  combo->setCurrentIndex(index >= 0 ? index : 0);
}

std::array<double, 6> MateToolPanel::offset_from_fields() const
{
  std::array<double, 6> offset{};
  for (int k = 0; k < 6; ++k) offset[k] = k < 3 ? offset_[k]->value() : qDegreesToRadians(offset_[k]->value());
  return offset;
}

void MateToolPanel::refresh()
{
  if (!controller_) return;
  refreshing_ = true;
  const auto & tokens = studioTokens();
  status_->setText(controller_->status());
  intro_->setText(controller_->backend_note().isEmpty()
    ? QStringLiteral("Click a face of the part to move, then the face it should sit on. Mounting faces are marked in "
        "the 3D view; press Esc to cancel.")
    : QStringLiteral("Choose the part to move, then the part it sits on (Scene Hierarchy or 3D view), and pick their "
        "datums from the lists; press Esc to cancel."));
  backend_note_->setText(controller_->backend_note());
  backend_note_->setVisible(!controller_->backend_note().isEmpty());
  using Step = MateToolController::Step;
  const Step step = controller_->step();
  const auto describe = [this](const std::optional<MateToolController::Pick> & pick, bool waiting) {
    if (!pick) return waiting ? QStringLiteral("Waiting for your click…") : QStringLiteral("—");
    return QStringLiteral("%1 · %2 (%3)").arg(controller_->display_name(pick->asset_id), pick->datum_label, pick->how);
  };
  child_.value->setText(describe(controller_->child(), step == Step::PickChild));
  parent_.value->setText(describe(controller_->parent(), step == Step::PickParent));
  child_.replace->setEnabled(controller_->child().has_value());
  parent_.replace->setEnabled(controller_->parent().has_value());
  const QString child_asset = controller_->child() ? controller_->child()->asset_id
    : (step == Step::PickChild ? controller_->pending_asset() : QString());
  const QString parent_asset = controller_->parent() ? controller_->parent()->asset_id
    : (step == Step::PickParent ? controller_->pending_asset() : QString());
  fill_datums(child_.datums, child_asset, controller_->child() ? controller_->child()->datum : QString());
  fill_datums(parent_.datums, parent_asset, controller_->parent() ? controller_->parent()->datum : QString());

  const QString focus = controller_->focus_asset();
  const bool visual = controller_->backend_note().isEmpty();
  isolate_->setChecked(controller_->isolate());
  isolate_->setEnabled(visual && !focus.isEmpty());
  QStringList entries;
  if (controller_->frames().contains(focus)) {
    for (const auto & datum : controller_->frames().value(focus).datums) entries << datum.name + QStringLiteral("|") + mate_datum_display(datum);
  }
  const QString key = focus + QStringLiteral("#") + entries.join(QStringLiteral(";"));
  if (key != face_view_key_) {  // never delete actions of an unchanged (possibly open) menu
    face_view_key_ = key;
    face_view_->menu()->clear();
    for (const auto & entry : entries) {
      const QString name = entry.section(QLatin1Char('|'), 0, 0);
      auto * action = face_view_->menu()->addAction(QStringLiteral("%1 — %2").arg(name, entry.section(QLatin1Char('|'), 1)));
      action->setData(name);
    }
  }
  face_view_->setEnabled(visual && !face_view_->menu()->isEmpty());

  const bool ready = step == Step::Ready;
  QString flush_reason;
  const bool flush_ok = controller_->flush_allowed(&flush_reason);
  flush_->setEnabled(ready && flush_ok);
  coincident_->setEnabled(ready);
  flush_->setChecked(controller_->alignment() == QStringLiteral("flush"));
  coincident_->setChecked(controller_->alignment() != QStringLiteral("flush"));
  alignment_note_->setText(flush_ok ? QString() : flush_reason);
  alignment_note_->setVisible(!alignment_note_->text().isEmpty());

  const auto & preview = controller_->preview();
  for (int k = 0; k < 6; ++k) {
    const QSignalBlocker blocker(offset_[k]);
    offset_[k]->setEnabled(ready);
    offset_[k]->setValue(k < 3 ? preview.offset[k] : qRadiansToDegrees(preview.offset[k]));
  }
  if (!ready) {
    preview_->setStyleSheet(QString());
    preview_->setText(QStringLiteral("Pick both faces to see where the part will go. Nothing changes until Apply."));
  } else if (!preview.ok) {
    preview_->setStyleSheet(QStringLiteral("color:%1;").arg(tokens.danger));
    preview_->setText(QStringLiteral("Cannot apply: %1").arg(preview.error));
  } else {
    preview_->setStyleSheet(QStringLiteral("color:%1;").arg(tokens.text));
    QString text = preview.child_summary;
    text += QStringLiteral("\nStores: %1 %2 %3 · %4, offset X %5 · Y %6 · gap %7 m, turn %8°.")
      .arg(controller_->child()->datum,
        controller_->alignment() == QStringLiteral("flush") ? QStringLiteral("flush on") : QStringLiteral("coincident with"),
        controller_->display_name(controller_->parent()->asset_id), controller_->parent()->datum)
      .arg(preview.offset[0], 0, 'f', 4).arg(preview.offset[1], 0, 'f', 4).arg(preview.offset[2], 0, 'f', 4)
      .arg(qRadiansToDegrees(preview.offset[5]), 0, 'f', 1);
    if (!preview.followers.isEmpty()) text += QStringLiteral("\nFollows: ") + preview.followers.join(QStringLiteral(", "));
    text += QStringLiteral("\nValid — the ghost in the 3D view shows the result.");
    preview_->setText(text);
  }
  reset_->setEnabled(ready);
  apply_->setEnabled(ready && preview.ok);
  apply_->setToolTip(apply_->isEnabled() ? QStringLiteral("Store this mate (undoable).")
    : (ready ? QStringLiteral("Cannot apply: %1").arg(preview.error) : QStringLiteral("Pick both faces first.")));
  refreshing_ = false;
}

}  // namespace workcell_builder
