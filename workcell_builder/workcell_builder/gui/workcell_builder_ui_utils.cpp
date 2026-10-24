#include "workcell_builder_ui_utils.hpp"

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAbstractScrollArea>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QColor>
#include <QCoreApplication>
#include <QHash>
#include <QPalette>
#include <QRegularExpression>
#include <QSettings>
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

namespace
{
StudioTokens lightTokens()
{
  StudioTokens t;
  t.bg = "#F5F7FA"; t.surface = "#FFFFFF"; t.surfaceRaised = "#FBFCFE"; t.surfaceSunken = "#EEF2F6";
  t.glass = "rgba(255,255,255,0.88)"; t.glassBorder = "rgba(16,43,78,0.10)";
  t.sidebar = "#0B2341"; t.sidebarRaised = "#12457A"; t.sidebarText = "#E2ECF7"; t.sidebarMuted = "#8FA6BF";
  t.border = "#DDE5EE"; t.borderStrong = "#C7D3DF"; t.text = "#172B3F"; t.textSecondary = "#5C6F82"; t.textMuted = "#8A99AA";
  t.accent = "#246FE5"; t.accentHover = "#0B4CC1"; t.accentSoft = "#EAF1FF"; t.accentText = "#FFFFFF"; t.selection = "#DCE8FF";
  t.success = "#147A49"; t.successSoft = "#ECF8F1"; t.warning = "#B45309"; t.warningSoft = "#FFF5E8";
  t.danger = "#BC2B34"; t.dangerSoft = "#FDEEEF"; t.info = "#1E5FB8"; t.infoSoft = "#EDF4FF";
  t.previewBg = "#121D28"; t.previewText = "#C7D3DF";
  t.radiusS = "6px"; t.radiusM = "9px"; t.radiusL = "12px";
  return t;
}

StudioTokens darkTokens()
{
  StudioTokens t;
  t.bg = "#111720"; t.surface = "#1A2230"; t.surfaceRaised = "#202A3A"; t.surfaceSunken = "#0E141C";
  t.glass = "rgba(26,34,48,0.90)"; t.glassBorder = "rgba(255,255,255,0.08)";
  t.sidebar = "#0B1626"; t.sidebarRaised = "#1B3760"; t.sidebarText = "#E2ECF7"; t.sidebarMuted = "#7F93AB";
  t.border = "#2A3647"; t.borderStrong = "#3B4A5E"; t.text = "#E6EDF5"; t.textSecondary = "#A9B6C6"; t.textMuted = "#7F8CA0";
  t.accent = "#4C8DFF"; t.accentHover = "#6AA2FF"; t.accentSoft = "#1B3159"; t.accentText = "#FFFFFF"; t.selection = "#27406B";
  t.success = "#3DD68C"; t.successSoft = "#12302A"; t.warning = "#F5B14B"; t.warningSoft = "#3A2A12";
  t.danger = "#FF7B7B"; t.dangerSoft = "#3A1A1E"; t.info = "#7FB3FF"; t.infoSoft = "#172A47";
  t.previewBg = "#0A1018"; t.previewText = "#C7D3DF";
  t.radiusS = "6px"; t.radiusM = "9px"; t.radiusL = "12px";
  return t;
}

QHash<QString, QString> tokenMap(const StudioTokens & t)
{
  return {
    {"bg", t.bg}, {"surface", t.surface}, {"surfaceRaised", t.surfaceRaised}, {"surfaceSunken", t.surfaceSunken},
    {"glass", t.glass}, {"glassBorder", t.glassBorder},
    {"sidebar", t.sidebar}, {"sidebarRaised", t.sidebarRaised}, {"sidebarText", t.sidebarText}, {"sidebarMuted", t.sidebarMuted},
    {"border", t.border}, {"borderStrong", t.borderStrong}, {"text", t.text}, {"textSecondary", t.textSecondary}, {"textMuted", t.textMuted},
    {"accent", t.accent}, {"accentHover", t.accentHover}, {"accentSoft", t.accentSoft}, {"accentText", t.accentText}, {"selection", t.selection},
    {"success", t.success}, {"successSoft", t.successSoft}, {"warning", t.warning}, {"warningSoft", t.warningSoft},
    {"danger", t.danger}, {"dangerSoft", t.dangerSoft}, {"info", t.info}, {"infoSoft", t.infoSoft},
    {"previewBg", t.previewBg}, {"previewText", t.previewText},
    {"radiusS", t.radiusS}, {"radiusM", t.radiusM}, {"radiusL", t.radiusL}};
}
}  // namespace

StudioAppearance studioAppearance()
{
  static int cached = -1;
  if (cached >= 0) return static_cast<StudioAppearance>(cached);
  QString mode = QString::fromLocal8Bit(qgetenv("WORKCELL_STUDIO_APPEARANCE")).trimmed().toLower();
  if (mode.isEmpty() && QCoreApplication::instance()) {
    mode = QSettings().value(QStringLiteral("appearance/mode"), QStringLiteral("system")).toString().trimmed().toLower();
  }
  bool dark = false;
  if (mode == QStringLiteral("dark")) dark = true;
  else if (mode == QStringLiteral("light")) dark = false;
  else if (QGuiApplication::instance()) dark = QGuiApplication::palette().color(QPalette::Window).lightness() < 128;
  cached = dark ? 1 : 0;
  return dark ? StudioAppearance::Dark : StudioAppearance::Light;
}

const StudioTokens & studioTokens()
{
  static const StudioTokens light = lightTokens();
  static const StudioTokens dark = darkTokens();
  return studioAppearance() == StudioAppearance::Dark ? dark : light;
}

QString studioQss(const QString & tmpl)
{
  static const QHash<QString, QString> values = tokenMap(studioTokens());
  static const QRegularExpression token(QStringLiteral("@([A-Za-z]+)"));
  QString out;
  out.reserve(tmpl.size() + 64);
  int last = 0;
  auto it = token.globalMatch(tmpl);
  while (it.hasNext()) {
    const auto m = it.next();
    const auto found = values.constFind(m.captured(1));
    if (found == values.constEnd()) continue;  // unknown names stay literal (e.g. "@media")
    out += tmpl.midRef(last, m.capturedStart() - last);
    out += found.value();
    last = m.capturedEnd();
  }
  out += tmpl.midRef(last);
  return out;
}

QString workcellStudioStyleSheet()
{
  // Application-wide base: typography, controls, surfaces. Page shells add
  // their own token-driven rules on top; nothing below names a raw colour.
  return studioQss(QStringLiteral(R"QSS(
    QWidget { font-size: 13px; color: @text; }
    QMainWindow, QDialog, QWizard { background-color: @bg; }
    QMenu { background: @surface; color: @text; border: 1px solid @border; border-radius: @radiusS; padding: 4px; }
    QMenu::item { padding: 6px 14px; border-radius: 4px; }
    QMenu::item:selected { background: @accentSoft; color: @text; }
    QGroupBox { background-color: @surface; border: 1px solid @border; border-radius: @radiusM; margin-top: 10px; padding-top: 6px; font-weight: 600; }
    QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: @textSecondary; font-weight: 600; }
    QGroupBox::indicator { width: 14px; height: 14px; }
    QTabWidget::pane { border: 1px solid @border; background: @surface; border-radius: @radiusM; top: -1px; }
    QTabBar::tab { background: transparent; color: @textSecondary; padding: 7px 12px; border: 0; border-bottom: 2px solid transparent; }
    QTabBar::tab:selected { color: @accent; border-bottom: 2px solid @accent; font-weight: 650; }
    QTabBar::tab:hover { color: @text; }
    QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTextBrowser, QTextEdit, QPlainTextEdit, QListWidget, QTreeWidget, QTableWidget, QListView, QTreeView, QTableView {
      background-color: @surface; color: @text; border: 1px solid @borderStrong; border-radius: @radiusS;
      selection-background-color: @selection; selection-color: @text; }
    QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { min-height: 28px; padding: 2px 8px; }
    QLineEdit[readOnly="true"] { background-color: @surfaceSunken; color: @textSecondary; border-style: dashed; }
    QLineEdit:disabled, QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled { background-color: @surfaceSunken; color: @textMuted; }
    QDoubleSpinBox#taskMinimumConfidence { qproperty-specialValueText: 'Unset'; }
    QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QListWidget:focus, QTreeWidget:focus, QTableWidget:focus { border: 2px solid @accent; }
    QComboBox::drop-down { border: 0; width: 22px; }
    QComboBox QAbstractItemView { background: @surface; color: @text; border: 1px solid @border; selection-background-color: @accentSoft; }
    QHeaderView::section { background: @surfaceRaised; color: @textSecondary; border: 0; border-bottom: 1px solid @border; padding: 6px 8px; font-weight: 650; }
    QPushButton, QToolButton { min-height: 30px; background-color: @surface; color: @text; border: 1px solid @borderStrong; border-radius: @radiusS; padding: 4px 12px; }
    QPushButton:hover, QToolButton:hover { background-color: @accentSoft; border-color: @accent; }
    QPushButton:pressed, QToolButton:pressed { background-color: @selection; }
    QPushButton:checked, QToolButton:checked { background-color: @accentSoft; border-color: @accent; color: @accent; font-weight: 650; }
    QPushButton:disabled, QToolButton:disabled { background-color: @surfaceSunken; color: @textMuted; border: 1px solid @border; }
    QPushButton:focus { border: 2px solid @accent; }
    QPushButton[class='primary_action'], QPushButton[role="primary"] { background-color: @accent; border-color: @accent; color: @accentText; font-weight: 650; }
    QPushButton[class='primary_action']:hover, QPushButton[role="primary"]:hover { background-color: @accentHover; border-color: @accentHover; }
    QPushButton[class='secondary_action'] { background-color: @surface; color: @text; }
    QPushButton[class='safe_action'] { background-color: @successSoft; color: @success; border-color: @success; font-weight: 650; }
    QPushButton[class='preview_action'] { background-color: @infoSoft; color: @info; border-color: @info; }
    QPushButton[class='destructive_action'] { background-color: @dangerSoft; color: @danger; border: 1px solid @danger; }
    QPushButton[class='disabled_placeholder'] { background-color: @surfaceSunken; color: @textMuted; border: 1px dashed @textMuted; }
    QCheckBox, QRadioButton { spacing: 6px; }
    QCheckBox::indicator, QGroupBox::indicator { width: 15px; height: 15px; border: 1px solid @borderStrong; border-radius: 4px; background: @surface; }
    QCheckBox::indicator:hover, QGroupBox::indicator:hover { border-color: @accent; }
    QCheckBox::indicator:checked, QGroupBox::indicator:checked { background: @accent; border-color: @accent; }
    QCheckBox::indicator:disabled, QGroupBox::indicator:disabled { background: @surfaceSunken; border-color: @border; }
    QScrollArea { background: transparent; border: 0; }
    QAbstractScrollArea { background: @surface; border: 1px solid @border; }
    QScrollBar:vertical { width: 10px; margin: 2px; background: transparent; }
    QScrollBar:horizontal { height: 10px; margin: 2px; background: transparent; }
    QScrollBar::handle:vertical, QScrollBar::handle:horizontal { min-height: 28px; min-width: 28px; background: @borderStrong; border-radius: 4px; }
    QScrollBar::handle:hover { background: @textMuted; }
    QScrollBar::add-line, QScrollBar::sub-line { height: 0; width: 0; }
    QSplitter::handle { background: @border; }
    QSplitter::handle:horizontal { width: 1px; }
    QSplitter::handle:vertical { height: 1px; }
    QStatusBar { background: @surface; color: @textSecondary; border-top: 1px solid @border; }
    QToolTip { background: @text; color: @bg; border: 0; padding: 6px 8px; border-radius: 4px; }
    QLabel#studioFieldHint { color: @textMuted; font-size: 11px; }
  )QSS"));
}

void applyStudioApplicationAppearance(QApplication * app)
{
  if (!app) return;
  const StudioTokens & t = studioTokens();
  // Native widgets (menus, message boxes, item views, the WebEngine host) follow
  // the palette; the stylesheet refines the rest.
  QPalette palette = app->palette();
  const auto set = [&palette](QPalette::ColorRole role, const QString & colour) {
    palette.setColor(QPalette::Active, role, QColor(colour));
    palette.setColor(QPalette::Inactive, role, QColor(colour));
  };
  set(QPalette::Window, t.bg); set(QPalette::WindowText, t.text);
  set(QPalette::Base, t.surface); set(QPalette::AlternateBase, t.surfaceRaised);
  set(QPalette::Text, t.text); set(QPalette::PlaceholderText, t.textMuted);
  set(QPalette::Button, t.surface); set(QPalette::ButtonText, t.text);
  set(QPalette::Highlight, t.accent); set(QPalette::HighlightedText, t.accentText);
  set(QPalette::ToolTipBase, t.text); set(QPalette::ToolTipText, t.bg);
  set(QPalette::Link, t.accent); set(QPalette::Mid, t.border); set(QPalette::Dark, t.borderStrong);
  palette.setColor(QPalette::Disabled, QPalette::Text, QColor(t.textMuted));
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(t.textMuted));
  palette.setColor(QPalette::Disabled, QPalette::WindowText, QColor(t.textMuted));
  app->setPalette(palette);
  app->setStyleSheet(workcellStudioStyleSheet());
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
  const StudioTokens & t = studioTokens();
  QString bg = t.surfaceSunken; QString fg = t.textSecondary;
  if (status == StatusType::Success || status == StatusType::Ready) { bg = t.successSoft; fg = t.success; }
  else if (status == StatusType::Warning) { bg = t.warningSoft; fg = t.warning; }
  else if (status == StatusType::Error) { bg = t.dangerSoft; fg = t.danger; }
  else if (status == StatusType::PreviewOnly) { bg = t.infoSoft; fg = t.info; }
  else if (status == StatusType::FakeHardware || status == StatusType::Info || status == StatusType::LiveEpd || status == StatusType::NoRuntimeMotion) { bg = t.accentSoft; fg = t.accent; }
  else if (status == StatusType::ScaffoldOnly) { bg = t.surfaceSunken; fg = t.textMuted; }
  label->setStyleSheet(QString("background:%1; color:%2; border:1px solid %3; border-radius:10px; padding:2px 8px; font-weight:600;").arg(bg, fg, t.border));
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
