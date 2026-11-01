#include "workcell_builder_ui_utils.hpp"

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAbstractScrollArea>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QColor>
#include <QCoreApplication>
#include <QHash>
#include <QPainter>
#include <QPainterPath>
#include <QPalette>
#include <QPixmap>
#include <QRegularExpression>
#include <QSettings>
#include <QCheckBox>
#include <QComboBox>
#include <QVBoxLayout>
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
#include <QToolButton>
#include <QTreeView>
#include <QVariant>
#include <QWindow>

#include <algorithm>
#include <cmath>

namespace workcell_builder
{
namespace
{
QScreen * screenForWidget(QWidget * widget)
{
  // The monitor the window actually overlaps most. The QWindow's associated screen lags
  // behind a move (and is the primary monitor until the window is first mapped), so
  // clamping into it dragged windows on a second or third monitor back to the primary.
  if (widget) {
    const QRect frame = widget->frameGeometry();
    QScreen * best = nullptr;
    qint64 best_area = 0;
    for (QScreen * screen : QGuiApplication::screens()) {
      const QRect overlap = screen->geometry().intersected(frame);
      const qint64 area = qint64(overlap.width()) * overlap.height();
      if (area > best_area) { best_area = area; best = screen; }
    }
    if (best) return best;
    if (widget->windowHandle() && widget->windowHandle()->screen()) return widget->windowHandle()->screen();
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

StudioDensity studioDensity()
{
  QString mode = QString::fromLocal8Bit(qgetenv("WORKCELL_STUDIO_DENSITY")).trimmed().toLower();
  if (mode.isEmpty() && QCoreApplication::instance())
    mode = QSettings().value(QStringLiteral("appearance/density"), QStringLiteral("comfortable")).toString().toLower();
  return mode == QStringLiteral("compact") ? StudioDensity::Compact : StudioDensity::Comfortable;
}

double studioTextScale()
{
  if (!QCoreApplication::instance()) return 1.0;
  const double scale = QSettings().value(QStringLiteral("appearance/text_scale"), 1.0).toDouble();
  return std::clamp(scale, 0.9, 1.5);
}

int studioFontPx(int base_px)
{
  return static_cast<int>(std::lround(base_px * studioTextScale()));
}

namespace
{
// Density-dependent control metrics (4 px grid). Comfortable is the established look.
QString densityQss()
{
  const bool compact = studioDensity() == StudioDensity::Compact;
  const int control = compact ? 26 : 30, field = compact ? 24 : 28;
  const int pad_v = compact ? 2 : 4, pad_h = compact ? 8 : 12, tab_v = compact ? 5 : 7;
  return QStringLiteral(
    "QWidget { font-size: %1px; }\n"
    "QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { min-height: %2px; }\n"
    "QPushButton, QToolButton { min-height: %3px; padding: %4px %5px; }\n"
    "QTabBar::tab { padding: %6px %5px; }\n"
    "QHeaderView::section { padding: %4px 8px; }\n")
    .arg(studioFontPx()).arg(field).arg(control).arg(pad_v).arg(pad_h).arg(tab_v);
}
}  // namespace

QString workcellStudioStyleSheet()
{
  // Application-wide base: typography, controls, surfaces. Page shells add
  // their own token-driven rules on top; nothing below names a raw colour.
  // Density/text-scale metrics are appended last so they win over the defaults.
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
    QTabBar::tab:selected { color: @accent; border-bottom: 2px solid @accent; }
    QTabBar::tab:hover { color: @text; }
    QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTextBrowser, QTextEdit, QPlainTextEdit, QListWidget, QTreeWidget, QTableWidget, QListView, QTreeView, QTableView {
      background-color: @surface; color: @text; border: 1px solid @borderStrong; border-radius: @radiusS;
      selection-background-color: @selection; selection-color: @text; }
    QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { min-height: 28px; padding: 2px 8px; }
    QLineEdit[readOnly="true"] { background-color: @surfaceSunken; color: @textSecondary; border-style: dashed; }
    QLineEdit:disabled, QComboBox:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled { background-color: @surfaceSunken; color: @textMuted; }
    QDoubleSpinBox#taskMinimumConfidence { qproperty-specialValueText: 'Unset'; }
    QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus { border: 2px solid @accent; }
    QListWidget:focus, QTreeWidget:focus, QTableWidget:focus, QListView:focus, QTreeView:focus, QTableView:focus { border: 1px solid @accent; }
    QComboBox::drop-down { border: 0; width: 22px; }
    QComboBox QAbstractItemView { background: @surface; color: @text; border: 1px solid @border; selection-background-color: @accentSoft; }
    QHeaderView::section { background: @surfaceRaised; color: @textSecondary; border: 0; border-bottom: 1px solid @border; padding: 6px 8px; font-weight: 650; }
    QPushButton, QToolButton { min-height: 30px; background-color: @surface; color: @text; border: 1px solid @borderStrong; border-radius: @radiusS; padding: 4px 12px; }
    QPushButton:hover, QToolButton:hover { background-color: @accentSoft; border-color: @accent; }
    QPushButton:pressed, QToolButton:pressed { background-color: @selection; }
    QPushButton:checked, QToolButton:checked { background-color: @accentSoft; border-color: @accent; color: @accent; }
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
    QAbstractButton:focus, QLineEdit:focus { outline: 0; }
    QScrollBar::handle:vertical:hover, QScrollBar::handle:horizontal:hover { background: @textMuted; }
    QSplitter::handle:hover { background: @accent; }
  )QSS")) + densityQss();
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

namespace
{
bool layoutContains(QLayout * layout, QWidget * widget)
{
  if (!layout) return false;
  for (int i = 0; i < layout->count(); ++i) {
    QLayoutItem * item = layout->itemAt(i);
    if (!item) continue;
    if (item->widget() == widget) return true;
    if (item->layout() && layoutContains(item->layout(), widget)) return true;
  }
  return false;
}
}  // namespace

int hideUnplacedChildWidgets(QWidget * root)
{
  if (!root) return 0;
  int hidden = 0;
  for (QWidget * widget : root->findChildren<QWidget *>()) {
    if (!widget->objectName().isEmpty() || widget->isWindow() || widget->isHidden()) continue;
    if (!qobject_cast<QAbstractButton *>(widget) && !qobject_cast<QLabel *>(widget)) continue;
    QWidget * parent = widget->parentWidget();
    if (!parent || !parent->layout() || layoutContains(parent->layout(), widget)) continue;
    widget->hide();
    ++hidden;
  }
  return hidden;
}

namespace
{
// Left-aligned wrapping layout (the Qt "flow layout" pattern) with height-for-width support.
class StudioFlowLayout : public QLayout
{
public:
  explicit StudioFlowLayout(QWidget * parent, int spacing) : QLayout(parent), spacing_(spacing) { setContentsMargins(0, 0, 0, 0); }
  ~StudioFlowLayout() override { while (QLayoutItem * item = takeAt(0)) delete item; }
  void addItem(QLayoutItem * item) override { items_.append(item); }
  int count() const override { return items_.size(); }
  QLayoutItem * itemAt(int index) const override { return items_.value(index); }
  QLayoutItem * takeAt(int index) override { return index >= 0 && index < items_.size() ? items_.takeAt(index) : nullptr; }
  Qt::Orientations expandingDirections() const override { return {}; }
  bool hasHeightForWidth() const override { return true; }
  int heightForWidth(int width) const override { return arrange(QRect(0, 0, width, 0), false); }
  void setGeometry(const QRect & rect) override { QLayout::setGeometry(rect); arrange(rect, true); }
  QSize sizeHint() const override { return minimumSize(); }
  QSize minimumSize() const override
  {
    QSize size;
    for (QLayoutItem * item : items_) size = size.expandedTo(item->minimumSize());
    const QMargins m = contentsMargins();
    return size + QSize(m.left() + m.right(), m.top() + m.bottom());
  }

private:
  int arrange(const QRect & rect, bool apply) const
  {
    const QMargins m = contentsMargins();
    const QRect area = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom());
    int x = area.x(), y = area.y(), row_height = 0;
    for (QLayoutItem * item : items_) {
      if (item->widget() && item->widget()->isHidden()) continue;
      const QSize hint = item->sizeHint();
      if (x > area.x() && x + hint.width() > area.right() + 1) { x = area.x(); y += row_height + spacing_; row_height = 0; }
      if (apply) item->setGeometry(QRect(QPoint(x, y), QSize(std::min(hint.width(), area.width()), hint.height())));
      x += hint.width() + spacing_;
      row_height = std::max(row_height, hint.height());
    }
    return y + row_height - rect.y() + m.bottom();
  }
  QList<QLayoutItem *> items_;
  int spacing_;
};
}  // namespace

QWidget * makeActionBar(QWidget * parent, const QList<QWidget *> & widgets)
{
  auto * bar = new QWidget(parent);
  bar->setObjectName(QStringLiteral("studioActionBar"));
  auto * flow = new StudioFlowLayout(bar, 8);
  for (QWidget * widget : widgets) {
    if (!widget) continue;
    widget->setParent(bar);
    widget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    flow->addWidget(widget);
  }
  QSizePolicy policy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  policy.setHeightForWidth(true);
  bar->setSizePolicy(policy);
  return bar;
}

void polishConsolePage(QWidget * page)
{
  auto * layout = page ? qobject_cast<QVBoxLayout *>(page->layout()) : nullptr;
  if (!layout || page->property("studioConsolePolished").toBool()) return;
  page->setProperty("studioConsolePolished", true);
  layout->setContentsMargins(24, 16, 24, 16);
  layout->setSpacing(12);
  QList<QWidget *> run;
  int run_start = -1;
  auto flush = [&]() {
    if (run.size() >= 1 && run_start >= 0) {
      QWidget * bar = makeActionBar(page, run);
      layout->insertWidget(run_start, bar);
    }
    run.clear();
    run_start = -1;
  };
  for (int i = 0; i < layout->count();) {
    QLayoutItem * item = layout->itemAt(i);
    QWidget * widget = item ? item->widget() : nullptr;
    if (auto * label = qobject_cast<QLabel *>(widget)) {
      label->setWordWrap(true);
      label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
      // Card padding as real margins: QLabel's word-wrap height ignores stylesheet padding, which cut
      // the last line of every card.
      if (label->objectName() == QLatin1String("studioCard")) label->setContentsMargins(14, 10, 14, 10);
    }
    if (widget && qobject_cast<QAbstractButton *>(widget) && !qobject_cast<QCheckBox *>(widget)) {
      if (run_start < 0) run_start = i;
      run.append(widget);
      delete layout->takeAt(i);
      continue;
    }
    if (run_start >= 0) { const int bar_index = run_start; flush(); i = bar_index + 2; continue; }  // bar, then this item
    ++i;
  }
  flush();
  // Tables and editors may grow; otherwise keep content at the top instead of spreading it out.
  bool grows = false;
  for (int i = 0; i < layout->count(); ++i) {
    QWidget * widget = layout->itemAt(i) ? layout->itemAt(i)->widget() : nullptr;
    // A fixed-height scroll view (studioFixedHeight) scrolls its own content and does not make the page grow.
    if (qobject_cast<QAbstractScrollArea *>(widget) && !widget->property("studioFixedHeight").toBool()) grows = true;
  }
  if (grows) return;
  layout->addStretch(1);
  // Text-and-action pages scroll vertically when the window is short (1280 x 720) instead of being
  // squeezed: the page's existing layout moves, unchanged, into a scrollable body.
  auto * body = new QWidget(page);
  body->setObjectName(QStringLiteral("studioConsoleBody"));
  auto * body_layout = new QVBoxLayout(body);
  body_layout->setContentsMargins(layout->contentsMargins());
  body_layout->setSpacing(layout->spacing());
  while (QLayoutItem * item = layout->takeAt(0)) {
    if (item->widget()) { body_layout->addWidget(item->widget()); delete item; }
    else body_layout->addItem(item);
  }
  layout->setContentsMargins(0, 0, 0, 0);
  auto * scroll = new QScrollArea(page);
  scroll->setObjectName(QStringLiteral("studioConsoleScroll"));
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setWidget(body);
  layout->addWidget(scroll);
  keepScrollContentHeight(scroll);
}

namespace
{
class WrappedLabelHeightKeeper : public QObject
{
public:
  explicit WrappedLabelHeightKeeper(QWidget * container) : QObject(container), container_(container) {}
  void refresh()
  {
    if (busy_) return;
    busy_ = true;
    for (QLabel * label : container_->findChildren<QLabel *>()) {
      if (!label->wordWrap() || !label->isVisible() || label->width() <= 0) continue;
      const int needed = label->heightForWidth(label->width());
      if (needed > 0 && label->minimumHeight() != needed) label->setMinimumHeight(needed);
    }
    busy_ = false;
  }

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    if (event->type() == QEvent::Resize || event->type() == QEvent::Show || event->type() == QEvent::LayoutRequest) {
      if (qobject_cast<QLabel *>(watched) || watched == container_) refresh();
    }
    return QObject::eventFilter(watched, event);
  }

private:
  QWidget * container_;
  bool busy_ = false;
};
}  // namespace

namespace
{
class ScrollContentHeightKeeper : public QObject
{
public:
  explicit ScrollContentHeightKeeper(QScrollArea * area) : QObject(area), area_(area) {}
  void refresh()
  {
    QWidget * content = area_->widget();
    if (busy_ || !content || !content->layout()) return;
    busy_ = true;
    const int width = area_->viewport()->width();
    const int need = content->layout()->hasHeightForWidth() ? content->layout()->totalHeightForWidth(width)
                                                             : content->layout()->totalMinimumSize().height();
    if (need > 0 && content->minimumHeight() != need) content->setMinimumHeight(need);
    busy_ = false;
  }

protected:
  bool eventFilter(QObject *, QEvent * event) override
  {
    if (event->type() == QEvent::Resize || event->type() == QEvent::LayoutRequest || event->type() == QEvent::Show) refresh();
    return false;
  }

private:
  QScrollArea * area_;
  bool busy_ = false;
};
}  // namespace

void keepScrollContentHeight(QScrollArea * area)
{
  if (!area || area->property("scrollContentHeight").toBool()) return;
  area->setProperty("scrollContentHeight", true);
  auto * keeper = new ScrollContentHeightKeeper(area);
  area->viewport()->installEventFilter(keeper);
  if (area->widget()) {
    area->widget()->installEventFilter(keeper);
    // Nested layouts change height without resizing the content widget; watch every child layout host.
    for (QWidget * child : area->widget()->findChildren<QWidget *>()) if (child->layout()) child->installEventFilter(keeper);
  }
}

void keepWrappedLabelHeights(QWidget * container)
{
  if (!container || container->property("wrappedLabelHeights").toBool()) return;
  container->setProperty("wrappedLabelHeights", true);
  auto * keeper = new WrappedLabelHeightKeeper(container);
  container->installEventFilter(keeper);
  for (QLabel * label : container->findChildren<QLabel *>()) if (label->wordWrap()) label->installEventFilter(keeper);
}

bool restoreWindowGeometryClamped(QWidget * window, const QString & key)
{
  if (!window) return false;
  const QByteArray saved = QSettings().value(key + QStringLiteral("/geometry")).toByteArray();
  if (saved.isEmpty() || !window->restoreGeometry(saved)) return false;
  // restoreGeometry() trusts the saved coordinates; a disconnected or rescaled monitor can leave the
  // window off-screen or larger than any screen. Clamp into the screen it overlaps most.
  QScreen * best = nullptr;
  int best_area = 0;
  const QRect frame = window->frameGeometry();
  for (QScreen * screen : QGuiApplication::screens()) {
    const QRect overlap = screen->availableGeometry().intersected(frame);
    if (overlap.width() * overlap.height() > best_area) { best_area = overlap.width() * overlap.height(); best = screen; }
  }
  if (!best) best = QGuiApplication::primaryScreen();
  if (!best) return true;
  const QRect available = best->availableGeometry();
  if (window->isMaximized() || window->isFullScreen()) return true;
  const QSize size(std::min(window->width(), available.width()), std::min(window->height(), available.height()));
  window->resize(size);
  const int x = std::clamp(window->x(), available.left(), std::max(available.left(), available.right() - window->frameGeometry().width() + 1));
  const int y = std::clamp(window->y(), available.top(), std::max(available.top(), available.bottom() - window->frameGeometry().height() + 1));
  window->move(x, y);
  return true;
}

void saveWindowGeometry(QWidget * window, const QString & key)
{
  if (window) QSettings().setValue(key + QStringLiteral("/geometry"), window->saveGeometry());
}

void seedSceneBuilderLayoutDefaults()
{
  QSettings settings;
  const auto seed = [&settings](const char * key, const QVariant & value) {
    if (!settings.contains(QLatin1String(key))) settings.setValue(QLatin1String(key), value);
  };
  seed("scene_builder/preferred_left_width", 260);
  seed("scene_builder/preferred_right_width", 470);
  seed("scene_builder/left_panel_visible", true);
  seed("scene_builder/right_panel_visible", true);
}

void resetSavedStudioLayout()
{
  QSettings settings;
  for (const QString & group : {QStringLiteral("window"), QStringLiteral("scene_builder")}) settings.remove(group);
}

namespace
{
class ResponsiveToolbarFilter final : public QObject
{
public:
  ResponsiveToolbarFilter(QWidget * host, QList<QAbstractButton *> primary, QList<QAbstractButton *> secondary,
                          QAbstractButton * save)
  : QObject(host), host_(host), primary_(std::move(primary)), secondary_(std::move(secondary)), save_(save) {}

  void apply()
  {
    if (!host_ || !host_->layout() || applying_) return;
    applying_ = true;
    // The host never shrinks below its own minimum, so measure the width its parent can offer.
    QWidget * parent = host_->parentWidget();
    const int available = parent ? parent->contentsRect().width() - 2 * std::max(0, host_->x()) : host_->width();
    int tier = 0;
    for (; tier < 2; ++tier) {
      set_tier(tier);
      if (host_->layout()->minimumSize().width() <= available) break;
    }
    if (tier == 2) set_tier(2);
    applying_ = false;
  }

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    if ((watched == host_ || watched == host_->parentWidget()) &&
        (event->type() == QEvent::Resize || event->type() == QEvent::Show)) apply();
    return QObject::eventFilter(watched, event);
  }

private:
  static void show_label(QAbstractButton * button, bool label)
  {
    if (!button) return;
    const QString full = button->property("fullText").toString();
    if (full.isEmpty()) return;
    button->setText(label ? full : QString());
    const bool segment = button->property("editModeSegment").toBool();
    QFont bold = button->font();
    bold.setBold(true);
    // label + 16 px icon + icon/text gap + side paddings (8+8 segment, 12+12 otherwise) + borders;
    // icon-only: a 34 px square target, a menu button also needs room for its arrow.
    const int icon_only = qobject_cast<QToolButton *>(button) ? 46 : 34;
    button->setMinimumWidth(label ? QFontMetrics(bold).horizontalAdvance(full) + button->iconSize().width() + (segment ? 30 : 38) : icon_only);
  }
  void set_tier(int tier)
  {
    for (auto * b : primary_) show_label(b, tier < 2);
    for (auto * b : secondary_) show_label(b, tier < 1);
    show_label(save_, true);
    host_->layout()->activate();
  }

  QWidget * host_;
  QList<QAbstractButton *> primary_, secondary_;
  QAbstractButton * save_;
  bool applying_ = false;
};
}  // namespace

void installResponsiveToolbar(QWidget * host, const QList<QAbstractButton *> & primary,
                              const QList<QAbstractButton *> & secondary, QAbstractButton * always_labelled)
{
  // (A property, not findChild<>: the filter has no Q_OBJECT, so findChild would match any QObject.)
  if (!host || host->property("responsiveToolbar").toBool()) return;
  host->setProperty("responsiveToolbar", true);
  auto * filter = new ResponsiveToolbarFilter(host, primary, secondary, always_labelled);
  host->installEventFilter(filter);
  if (host->parentWidget()) host->parentWidget()->installEventFilter(filter);
  filter->apply();
}

QIcon studioIcon(const QString & name, const QString & colour)
{
  const QColor ink(colour.isEmpty() ? studioTokens().text : colour);
  QIcon icon;
  for (const int scale : {1, 2}) {
    QPixmap pixmap(16 * scale, 16 * scale);
    pixmap.setDevicePixelRatio(scale);
    pixmap.fill(Qt::transparent);
    QPainter p(&pixmap);
    p.setRenderHint(QPainter::Antialiasing);
    QPen pen(ink, 1.6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    p.setPen(pen);
    QPainterPath path;
    if (name == QLatin1String("select")) {
      path.moveTo(4, 2.5); path.lineTo(4, 13); path.lineTo(7, 10); path.lineTo(9.5, 14); path.lineTo(11, 13.2);
      path.lineTo(8.6, 9.3); path.lineTo(12.5, 9.3); path.closeSubpath();
    } else if (name == QLatin1String("move")) {
      path.moveTo(8, 2); path.lineTo(8, 14); path.moveTo(2, 8); path.lineTo(14, 8);
      path.moveTo(6, 4); path.lineTo(8, 2); path.lineTo(10, 4);
      path.moveTo(6, 12); path.lineTo(8, 14); path.lineTo(10, 12);
      path.moveTo(4, 6); path.lineTo(2, 8); path.lineTo(4, 10);
      path.moveTo(12, 6); path.lineTo(14, 8); path.lineTo(12, 10);
    } else if (name == QLatin1String("rotate")) {
      path.arcMoveTo(2.5, 2.5, 11, 11, 60); path.arcTo(2.5, 2.5, 11, 11, 60, 270);
      path.moveTo(11.2, 1.8); path.lineTo(10.9, 4.6); path.lineTo(13.7, 4.9);
    } else if (name == QLatin1String("add")) {
      path.addRoundedRect(2.5, 2.5, 11, 11, 2.5, 2.5); path.moveTo(8, 5.5); path.lineTo(8, 10.5); path.moveTo(5.5, 8); path.lineTo(10.5, 8);
    } else if (name == QLatin1String("fit")) {
      for (const auto & c : {QPointF(2.5, 2.5), QPointF(13.5, 2.5), QPointF(13.5, 13.5), QPointF(2.5, 13.5)}) {
        const double sx = c.x() < 8 ? 1 : -1, sy = c.y() < 8 ? 1 : -1;
        path.moveTo(c + QPointF(0, 3.5 * sy)); path.lineTo(c); path.lineTo(c + QPointF(3.5 * sx, 0));
      }
      path.addEllipse(QPointF(8, 8), 2, 2);
    } else if (name == QLatin1String("mate")) {
      path.addRoundedRect(1.8, 5.5, 7.5, 5, 2.5, 2.5); path.addRoundedRect(6.7, 5.5, 7.5, 5, 2.5, 2.5);
    } else if (name == QLatin1String("undo") || name == QLatin1String("redo")) {
      const bool undo = name == QLatin1String("undo");
      QTransform flip; if (!undo) { flip.translate(16, 0); flip.scale(-1, 1); }
      QPainterPath arrow;
      arrow.moveTo(5.5, 3.5); arrow.lineTo(2.5, 6.5); arrow.lineTo(5.5, 9.5);
      arrow.moveTo(2.8, 6.5); arrow.lineTo(9.5, 6.5); arrow.cubicTo(15, 6.5, 15, 13.5, 9.5, 13.5); arrow.lineTo(6.5, 13.5);
      path = flip.map(arrow);
    } else if (name == QLatin1String("save")) {
      path.moveTo(2.5, 2.5); path.lineTo(11, 2.5); path.lineTo(13.5, 5); path.lineTo(13.5, 13.5); path.lineTo(2.5, 13.5); path.closeSubpath();
      path.addRect(5, 2.5, 5, 3.5); path.addRect(4.8, 9, 6.4, 4.5);
    } else if (name == QLatin1String("preview")) {
      path.addRoundedRect(1.8, 3, 12.4, 8.5, 1.5, 1.5); path.moveTo(5.5, 14); path.lineTo(10.5, 14);
      path.moveTo(6.8, 5.5); path.lineTo(9.8, 7.25); path.lineTo(6.8, 9); path.closeSubpath();
    } else if (name == QLatin1String("view")) {
      path.moveTo(1.5, 8); path.cubicTo(4.5, 3, 11.5, 3, 14.5, 8); path.cubicTo(11.5, 13, 4.5, 13, 1.5, 8);
      path.addEllipse(QPointF(8, 8), 2.2, 2.2);
    } else if (name == QLatin1String("more")) {
      p.setBrush(ink); p.setPen(Qt::NoPen);
      for (const double x : {3.5, 8.0, 12.5}) p.drawEllipse(QPointF(x, 8), 1.4, 1.4);
    } else if (name == QLatin1String("delete")) {
      path.moveTo(2.5, 4.5); path.lineTo(13.5, 4.5); path.moveTo(6, 4.5); path.lineTo(6.5, 2.5); path.lineTo(9.5, 2.5);
      path.lineTo(10, 4.5); path.moveTo(4, 4.5); path.lineTo(5, 13.5); path.lineTo(11, 13.5); path.lineTo(12, 4.5);
      path.moveTo(7, 7); path.lineTo(7, 11); path.moveTo(9, 7); path.lineTo(9, 11);
    } else if (name == QLatin1String("reset")) {
      path.arcMoveTo(2.5, 2.5, 11, 11, 100); path.arcTo(2.5, 2.5, 11, 11, 100, 300);
      path.moveTo(2.2, 1.8); path.lineTo(3.3, 5.2); path.lineTo(6.6, 4.2);
    }
    p.drawPath(path);
    p.end();
    icon.addPixmap(pixmap);
  }
  return icon;
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
