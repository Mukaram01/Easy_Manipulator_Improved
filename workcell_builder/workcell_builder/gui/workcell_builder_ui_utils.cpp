#include "workcell_builder_ui_utils.hpp"

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAbstractScrollArea>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QEvent>
#include <QFrame>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHeaderView>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QListView>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScreen>
#include <QScrollArea>
#include <QSizePolicy>
#include <QTabBar>
#include <QTabWidget>
#include <QTableView>
#include <QTextEdit>
#include <QTreeView>
#include <QVariant>
#include <QWindow>

#include <algorithm>

namespace workcell_builder
{
namespace
{
QScreen * screenForWidget(QWidget * widget)
{
  if (widget && widget->windowHandle() && widget->windowHandle()->screen()) {
    return widget->windowHandle()->screen();
  }
  return QGuiApplication::primaryScreen();
}

QRect usableScreenGeometry(QWidget * widget)
{
  QScreen * screen = screenForWidget(widget);
  if (!screen) return QRect(0, 0, 1280, 720);

  QRect available = screen->availableGeometry();
  const int inset = std::min(16, std::min(available.width(), available.height()) / 20);
  return available.adjusted(inset, inset, -inset, -inset);
}

void keepWindowInsideScreen(QWidget * widget)
{
  if (!widget || widget->isFullScreen() || widget->isMaximized()) return;

  const QRect available = usableScreenGeometry(widget);
  if (!available.isValid()) return;

  const QSize bounded_window_size(
    std::min(widget->width(), available.width()),
    std::min(widget->height(), available.height()));

  const QSize current_minimum = widget->minimumSize();
  widget->setMinimumSize(
    std::min(current_minimum.width(), available.width()),
    std::min(current_minimum.height(), available.height()));

  if (bounded_window_size != widget->size()) {
    widget->resize(bounded_window_size);
  }

  const QRect frame = widget->frameGeometry();
  const int max_x = std::max(available.left(), available.right() - frame.width() + 1);
  const int max_y = std::max(available.top(), available.bottom() - frame.height() + 1);
  const int bounded_x = std::max(available.left(), std::min(frame.left(), max_x));
  const int bounded_y = std::max(available.top(), std::min(frame.top(), max_y));
  if (bounded_x != frame.left() || bounded_y != frame.top()) {
    widget->move(bounded_x, bounded_y);
  }
}

class ResponsiveWindowGuard : public QObject
{
public:
  explicit ResponsiveWindowGuard(QWidget * window)
  : QObject(window), window_(window)
  {
  }

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    if (watched == window_ && !adjusting_ &&
      (event->type() == QEvent::Show || event->type() == QEvent::Resize ||
      event->type() == QEvent::WindowStateChange))
    {
      adjusting_ = true;
      keepWindowInsideScreen(window_);
      adjusting_ = false;
    }
    return QObject::eventFilter(watched, event);
  }

private:
  QWidget * window_ = nullptr;
  bool adjusting_ = false;
};

void installResponsiveWindowGuard(QWidget * widget)
{
  if (!widget || widget->property("workcell_responsive_window_guard").toBool()) return;
  widget->installEventFilter(new ResponsiveWindowGuard(widget));
  widget->setProperty("workcell_responsive_window_guard", true);
}

void configureContainedWidgets(QWidget * widget)
{
  if (!widget) return;

  for (QLayout * layout : widget->findChildren<QLayout *>()) {
    const QMargins margins = layout->contentsMargins();
    layout->setContentsMargins(
      std::min(margins.left(), 12),
      std::min(margins.top(), 12),
      std::min(margins.right(), 12),
      std::min(margins.bottom(), 12));
    if (layout->spacing() > 10) layout->setSpacing(8);
  }

  for (QScrollArea * area : widget->findChildren<QScrollArea *>()) {
    area->setWidgetResizable(true);
    area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    area->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    area->setFrameShape(QFrame::NoFrame);
    area->setMinimumWidth(0);
    area->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  for (QAbstractItemView * view : widget->findChildren<QAbstractItemView *>()) {
    view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    view->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    view->setTextElideMode(Qt::ElideRight);
    view->setMinimumWidth(0);
    view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  for (QTableView * table : widget->findChildren<QTableView *>()) {
    if (table->horizontalHeader()) {
      table->horizontalHeader()->setStretchLastSection(true);
      table->horizontalHeader()->setMinimumSectionSize(44);
    }
    table->setWordWrap(true);
  }

  for (QTextEdit * editor : widget->findChildren<QTextEdit *>()) {
    editor->setLineWrapMode(QTextEdit::WidgetWidth);
    editor->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    editor->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    editor->setMinimumWidth(0);
  }

  for (QPlainTextEdit * editor : widget->findChildren<QPlainTextEdit *>()) {
    editor->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    editor->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    editor->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    editor->setMinimumWidth(0);
  }

  for (QTabWidget * tabs : widget->findChildren<QTabWidget *>()) {
    tabs->setUsesScrollButtons(true);
    tabs->setElideMode(Qt::ElideRight);
    tabs->setDocumentMode(true);
    tabs->setMinimumWidth(0);
    tabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    if (tabs->tabBar()) tabs->tabBar()->setExpanding(false);
  }

  for (QLineEdit * edit : widget->findChildren<QLineEdit *>()) {
    edit->setMinimumWidth(0);
    edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }
  for (QComboBox * combo : widget->findChildren<QComboBox *>()) {
    combo->setMinimumWidth(0);
    combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    combo->setMinimumContentsLength(8);
    combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }
  for (QAbstractSpinBox * spin : widget->findChildren<QAbstractSpinBox *>()) {
    spin->setMinimumWidth(0);
    spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }
  for (QPushButton * button : widget->findChildren<QPushButton *>()) {
    button->setMinimumWidth(0);
    button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  }
  for (QGroupBox * group : widget->findChildren<QGroupBox *>()) {
    group->setMinimumWidth(0);
    group->setSizePolicy(QSizePolicy::Expanding, group->sizePolicy().verticalPolicy());
  }
}
}  // namespace

QString workcellStudioStyleSheet()
{
  return QStringLiteral(
    "QWidget { font-size: 13px; color: #17202a; }"
    "QMainWindow, QDialog { background-color: #f4f7fb; }"
    "QGroupBox { background-color: #ffffff; border: 1px solid #c7d3df; border-radius: 8px; margin-top: 10px; padding-top: 5px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #4d5b6a; font-weight: 600; }"
    "QTabWidget::pane { border: 1px solid #c7d3df; background: #ffffff; border-radius: 6px; }"
    "QTabBar::tab { background: #eef3f8; color: #4d5b6a; border: 1px solid #c7d3df; padding: 7px 10px; }"
    "QTabBar::tab:selected { background: #ffffff; color: #17202a; border-bottom-color: #ffffff; }"
    "QTabBar::tab:hover { background: #e1eaf3; }"
    "QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTextBrowser, QTextEdit, QPlainTextEdit, QListWidget, QTreeWidget, QTableWidget { background-color: #ffffff; border: 1px solid #c7d3df; border-radius: 4px; selection-background-color: #d7edf7; selection-color: #17202a; }"
    "QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QListWidget:focus, QTreeWidget:focus, QTableWidget:focus { border: 2px solid #1d5da8; }"
    "QHeaderView::section { background: #eef3f8; color: #17202a; border: 0; border-right: 1px solid #c7d3df; border-bottom: 1px solid #c7d3df; padding: 5px; font-weight: 600; }"
    "QPushButton, QToolButton { min-height: 30px; background-color: #60758a; color: #ffffff; border: 1px solid #4d5b6a; border-radius: 5px; padding: 5px 10px; }"
    "QPushButton:hover, QToolButton:hover { background-color: #4d647a; }"
    "QPushButton:pressed, QToolButton:pressed { background-color: #3f5366; }"
    "QPushButton:disabled, QToolButton:disabled { background-color: #d8e0e8; color: #6c7884; border: 1px solid #c7d3df; }"
    "QPushButton[class='primary_action'] { background-color: #1d5da8; border-color: #174a86; color: #ffffff; font-weight: 600; }"
    "QPushButton[class='secondary_action'] { background-color: #60758a; border-color: #4d647a; }"
    "QPushButton[class='safe_action'] { background-color: #217a34; border-color: #1a622a; }"
    "QPushButton[class='preview_action'] { background-color: #6f42c1; border-color: #57329a; }"
    "QPushButton[class='destructive_action'] { background-color: #fce8e7; color: #b3261e; border: 1px solid #b3261e; }"
    "QPushButton[class='disabled_placeholder'] { background-color: #d8e0e8; color: #6c7884; border: 1px dashed #6c7884; }"
    "QScrollArea { background: transparent; border: 0; }"
    "QAbstractScrollArea { background: #ffffff; border: 1px solid #c7d3df; }"
    "QScrollBar:vertical { width: 10px; margin: 0; background: transparent; }"
    "QScrollBar::handle:vertical { min-height: 28px; background: #b7c4d1; border-radius: 5px; }"
    "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }"
    "QToolTip { background: #17202a; color: #ffffff; border: 0; padding: 5px; }");
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

void capDialogSize(QWidget * widget, int, int)
{
  if (!widget) return;

  const QRect available = usableScreenGeometry(widget);
  widget->setMinimumSize(
    std::min(640, available.width()),
    std::min(420, available.height()));
  widget->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
  installResponsiveWindowGuard(widget);
  keepWindowInsideScreen(widget);
}

void makeTextWidgetsWrap(QWidget * widget)
{
  if (!widget) return;
  for (QLabel * label : widget->findChildren<QLabel *>()) {
    label->setWordWrap(true);
    label->setMinimumWidth(0);
    label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  }
}

void applyPrimarySecondaryButtonStyle(QWidget * widget)
{
  if (!widget) return;
  for (QDialogButtonBox * box : widget->findChildren<QDialogButtonBox *>()) {
    if (auto * ok = box->button(QDialogButtonBox::Ok)) {
      applyButtonRoleStyle(ok, ButtonRole::primary_action);
      ok->setDefault(true);
    }
    if (auto * save = box->button(QDialogButtonBox::Save)) {
      applyButtonRoleStyle(save, ButtonRole::primary_action);
      save->setDefault(true);
    }
  }
}

void applyStatusLabelStyle(QLabel * label, StatusType status)
{
  applyStatusBadgeStyle(label, status);
}

void applyCompactDialogDefaults(QWidget * widget)
{
  if (!widget) return;
  capDialogSize(widget);
  makeTextWidgetsWrap(widget);
  configureContainedWidgets(widget);
  applyWorkcellStudioTheme(widget);
  applyPrimarySecondaryButtonStyle(widget);
}
}  // namespace workcell_builder
