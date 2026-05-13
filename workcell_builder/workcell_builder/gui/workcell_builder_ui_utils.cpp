#include "workcell_builder_ui_utils.hpp"

#include <QAbstractButton>
#include <QDialogButtonBox>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QVariant>

namespace workcell_builder
{
void capDialogSize(QWidget * widget, int max_width, int max_height)
{
  if (!widget) {
    return;
  }
  widget->setMinimumSize(640, 420);
  widget->setMaximumSize(max_width, max_height);
  widget->resize(widget->size().boundedTo(QSize(max_width, max_height)));
}

void makeTextWidgetsWrap(QWidget * widget)
{
  if (!widget) {
    return;
  }

  const auto labels = widget->findChildren<QLabel *>();
  for (QLabel * label : labels) {
    label->setWordWrap(true);
    label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  }
}

void applyPrimarySecondaryButtonStyle(QWidget * widget)
{
  if (!widget) {
    return;
  }

  const auto button_boxes = widget->findChildren<QDialogButtonBox *>();
  for (QDialogButtonBox * box : button_boxes) {
    if (auto * ok = box->button(QDialogButtonBox::Ok)) {
      ok->setProperty("class", QVariant(QStringLiteral("primary_action")));
      ok->setDefault(true);
    }
    if (auto * save = box->button(QDialogButtonBox::Save)) {
      save->setProperty("class", QVariant(QStringLiteral("primary_action")));
      save->setDefault(true);
    }
  }

  const auto buttons = widget->findChildren<QPushButton *>();
  for (QPushButton * button : buttons) {
    const QString name = button->objectName().toLower();
    if (name.contains("generate") || name.contains("save") || name.contains("next")) {
      button->setProperty("class", QVariant(QStringLiteral("primary_action")));
      button->setDefault(true);
    }
  }

  widget->setStyleSheet(widget->styleSheet() +
    "\nQPushButton[class='primary_action'] {"
    "background-color: #1d5da8; color: white; font-weight: 600; padding: 4px 10px; }"
    "\nQPushButton[class='primary_action']:disabled {"
    "background-color: #8ea1b8; color: #f2f2f2; }");
}

void applyStatusLabelStyle(QLabel * label, StatusType status)
{
  if (!label) {
    return;
  }

  QString color = "#2f2f2f";
  if (status == StatusType::Info) color = "#1d5da8";
  if (status == StatusType::Warning) color = "#9a6700";
  if (status == StatusType::Error) color = "#b3261e";
  if (status == StatusType::Success) color = "#217a34";

  label->setWordWrap(true);
  label->setStyleSheet(QString("color: %1; font-weight: 500;").arg(color));
}

void applyCompactDialogDefaults(QWidget * widget)
{
  if (!widget) {
    return;
  }
  capDialogSize(widget);
  makeTextWidgetsWrap(widget);
  applyPrimarySecondaryButtonStyle(widget);
}
}  // namespace workcell_builder
