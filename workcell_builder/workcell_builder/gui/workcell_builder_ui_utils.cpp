#include "workcell_builder_ui_utils.hpp"

#include <QAbstractButton>
#include <QDialogButtonBox>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QVariant>

namespace workcell_builder
{
QString workcellStudioStyleSheet()
{
  return QStringLiteral(
    "QWidget { font-size: 13px; color: #17202a; }"
    "QMainWindow, QDialog { background-color: #f4f7fb; }"
    "QGroupBox { background-color: #ffffff; border: 1px solid #c7d3df; border-radius: 8px; margin-top: 10px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #4d5b6a; font-weight: 600; }"
    "QTabWidget::pane { border: 1px solid #c7d3df; background: #ffffff; }"
    "QTabBar::tab { background: #eef3f8; color: #4d5b6a; border: 1px solid #c7d3df; padding: 6px 12px; }"
    "QTabBar::tab:selected { background: #ffffff; color: #17202a; }"
    "QLineEdit, QComboBox, QSpinBox, QTextBrowser, QPlainTextEdit, QListWidget, QTreeWidget, QTableWidget { background-color: #ffffff; border: 1px solid #c7d3df; border-radius: 4px; }"
    "QLineEdit:focus, QComboBox:focus, QSpinBox:focus { border: 1px solid #1d5da8; }"
    "QHeaderView::section { background: #eef3f8; color: #17202a; border: 1px solid #c7d3df; padding: 4px; font-weight: 600; }"
    "QPushButton { background-color: #60758a; color: #ffffff; border: 1px solid #4d5b6a; border-radius: 5px; padding: 6px 12px; }"
    "QPushButton:hover { background-color: #4d647a; }"
    "QPushButton:pressed { background-color: #3f5366; }"
    "QPushButton:disabled { background-color: #d8e0e8; color: #6c7884; border: 1px solid #c7d3df; }"
    "QPushButton[class='primary_action'] { background-color: #1d5da8; border-color: #174a86; color: #ffffff; font-weight: 600; }"
    "QPushButton[class='secondary_action'] { background-color: #60758a; border-color: #4d647a; }"
    "QPushButton[class='safe_action'] { background-color: #217a34; border-color: #1a622a; }"
    "QPushButton[class='preview_action'] { background-color: #6f42c1; border-color: #57329a; }"
    "QPushButton[class='destructive_action'] { background-color: #fce8e7; color: #b3261e; border: 1px solid #b3261e; }"
    "QPushButton[class='disabled_placeholder'] { background-color: #d8e0e8; color: #6c7884; border: 1px dashed #6c7884; }"
    "QScrollArea, QAbstractScrollArea { background: #f4f7fb; border: 1px solid #c7d3df; }");
}

void applyWorkcellStudioTheme(QWidget * widget)
{
  if (widget) widget->setStyleSheet(workcellStudioStyleSheet());
}

void applyButtonRoleStyle(QPushButton * button, ButtonRole role)
{
  if (!button) return;
  switch (role) {
    case ButtonRole::primary_action: button->setProperty("class", "primary_action"); break;
    case ButtonRole::secondary_action: button->setProperty("class", "secondary_action"); break;
    case ButtonRole::safe_action: button->setProperty("class", "safe_action"); break;
    case ButtonRole::preview_action: button->setProperty("class", "preview_action"); break;
    case ButtonRole::destructive_action: button->setProperty("class", "destructive_action"); break;
    case ButtonRole::disabled_placeholder: button->setProperty("class", "disabled_placeholder"); break;
  }
}

void applyStatusBadgeStyle(QLabel * label, StatusType status)
{
  if (!label) return;
  QString bg = "#eef3f8"; QString fg = "#4d5b6a";
  if (status == StatusType::Success || status == StatusType::Ready) { bg = "#e8f5eb"; fg = "#217a34"; }
  else if (status == StatusType::Warning) { bg = "#fff3e0"; fg = "#b26a00"; }
  else if (status == StatusType::Error) { bg = "#fdecea"; fg = "#b3261e"; }
  else if (status == StatusType::PreviewOnly) { bg = "#f1ebff"; fg = "#6f42c1"; }
  else if (status == StatusType::FakeHardware || status == StatusType::Info || status == StatusType::LiveEpd || status == StatusType::NoRuntimeMotion) { bg = "#e7f0fb"; fg = "#2769b3"; }
  else if (status == StatusType::ScaffoldOnly) { bg = "#eef3f8"; fg = "#60758a"; }
  label->setStyleSheet(QString("background:%1; color:%2; border:1px solid #c7d3df; border-radius:10px; padding:2px 8px; font-weight:600;").arg(bg, fg));
}

void capDialogSize(QWidget * widget, int, int){ if(!widget) return; widget->setMinimumSize(640,420); widget->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX); }
void makeTextWidgetsWrap(QWidget * widget){ if(!widget) return; for (QLabel * label : widget->findChildren<QLabel *>()) { label->setWordWrap(true); label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);} }

void applyPrimarySecondaryButtonStyle(QWidget * widget)
{
  if (!widget) return;
  for (QDialogButtonBox * box : widget->findChildren<QDialogButtonBox *>()) {
    if (auto * ok = box->button(QDialogButtonBox::Ok)) { applyButtonRoleStyle(ok, ButtonRole::primary_action); ok->setDefault(true);}    
    if (auto * save = box->button(QDialogButtonBox::Save)) { applyButtonRoleStyle(save, ButtonRole::primary_action); save->setDefault(true);}    
  }
}

void applyStatusLabelStyle(QLabel * label, StatusType status){ applyStatusBadgeStyle(label, status); }

void applyCompactDialogDefaults(QWidget * widget)
{
  if (!widget) return;
  capDialogSize(widget);
  makeTextWidgetsWrap(widget);
  applyWorkcellStudioTheme(widget);
  applyPrimarySecondaryButtonStyle(widget);
}
}  // namespace workcell_builder
