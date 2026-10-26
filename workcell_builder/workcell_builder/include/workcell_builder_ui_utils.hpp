#pragma once

#include <QWidget>
#include <QIcon>
#include <QLabel>
#include <QPushButton>
#include <QString>

class QApplication;
class QScrollArea;
class QAbstractButton;

#include "embedded_web_curated_add_bootstrap.hpp"

namespace workcell_builder
{
enum class StatusType
{
  Normal,
  Info,
  Warning,
  Error,
  Success,
  Ready,
  PreviewOnly,
  FakeHardware,
  LiveEpd,
  NoRuntimeMotion,
  ScaffoldOnly
};

enum class ButtonRole
{
  primary_action,
  secondary_action,
  safe_action,
  preview_action,
  destructive_action,
  disabled_placeholder
};


// Shared design tokens. Every studio stylesheet is written against these names
// ("@surface", "@accent", ...) and resolved through studioQss(), so light and
// dark appearance come from one place instead of scattered hex values.
enum class StudioAppearance { Light, Dark };

struct StudioTokens
{
  QString bg, surface, surfaceRaised, surfaceSunken, glass, glassBorder;
  QString sidebar, sidebarRaised, sidebarText, sidebarMuted;
  QString border, borderStrong, text, textSecondary, textMuted;
  QString accent, accentHover, accentSoft, accentText, selection;
  QString success, successSoft, warning, warningSoft, danger, dangerSoft, info, infoSoft;
  QString previewBg, previewText;
  QString radiusS, radiusM, radiusL;
};

// WORKCELL_STUDIO_APPEARANCE=light|dark|system (default system: follows the
// desktop palette). QSettings "appearance/mode" is honoured when the env is unset.
StudioAppearance studioAppearance();
const StudioTokens & studioTokens();
QString studioQss(const QString & tmpl);
QString workcellStudioStyleSheet();
void applyStudioApplicationAppearance(QApplication * app);
void applyWorkcellStudioTheme(QWidget * widget);
void applyStatusBadgeStyle(QLabel * label, StatusType status);
void applyButtonRoleStyle(QPushButton * button, ButtonRole role);
void applyCompactDialogDefaults(QWidget * widget);
void capDialogSize(QWidget * widget, int max_width = 1100, int max_height = 760);
void makeTextWidgetsWrap(QWidget * widget);
void applyStatusLabelStyle(QLabel * label, StatusType status);
void applyPrimarySecondaryButtonStyle(QWidget * widget);

// Interface density and text scale (QSettings "appearance/density" = comfortable|compact,
// "appearance/text_scale" = 1.0|1.125|1.25; env WORKCELL_STUDIO_DENSITY overrides density).
// Both shape the application stylesheet's control heights, paddings and base font size and can be
// re-applied live with applyStudioApplicationAppearance(). Appearance (light/dark) applies at start.
enum class StudioDensity { Comfortable, Compact };
StudioDensity studioDensity();
double studioTextScale();
// Base font size in px for the current text scale (13 px at 100 %).
int studioFontPx(int base_px = 13);

// Window geometry persisted under "<key>/geometry" and restored only if it still intersects an
// available screen (a monitor may have been removed or rescaled); the restored frame is clamped
// into that screen. Returns false when nothing usable was saved (callers then pick a default).
bool restoreWindowGeometryClamped(QWidget * window, const QString & key);
void saveWindowGeometry(QWidget * window, const QString & key);
// Hides unnamed buttons/labels that were created on a page with a layout but never added to it
// (state holders whose real control lives in a menu). Unplaced, they paint at the page's top-left
// under other widgets and still take keyboard focus. Named widgets (overlays positioned on purpose)
// and children of layout-less containers (stacks, splitters, scroll viewports) are left alone.
// Returns the number hidden.
int hideUnplacedChildWidgets(QWidget * root);
// Product panel defaults for a first run or after Reset Layout (left 260 / right 470 px, both panels
// shown). MainWindow re-reads these keys whenever a scene opens, so they are written only when absent.
void seedSceneBuilderLayoutDefaults();
// Clears saved window/panel arrangement keys (Reset Layout); appearance preferences are kept.
void resetSavedStudioLayout();

// Action bar: buttons keep their natural (label) width and wrap onto further rows when the page is
// narrow, instead of being squeezed into clipped labels or stretched into full-width bars.
// The widgets are reparented into the returned container (add it to the page layout).
QWidget * makeActionBar(QWidget * parent, const QList<QWidget *> & widgets);

// Console-style page (title, cards, actions in one QVBoxLayout): 24/16 px margins, 12 px rhythm,
// wrapped text, content aligned to the top, and every run of consecutive buttons gathered into one
// wrapping action bar. Widget identities and signal connections are unchanged.
void polishConsolePage(QWidget * page);

// Word-wrapped labels report their height for their preferred width; in a narrow scroll area that
// under-reserves lines and form rows overlap. This pins each visible wrapped label's minimum height to
// heightForWidth(actual width) whenever the container resizes or the label text changes.
void keepWrappedLabelHeights(QWidget * container);
// QScrollArea sizes a resizable widget from its sizeHint, not its height-for-width, so wrapped
// content is squeezed (rows overlap). This keeps the content's minimum height at the layout's
// height for the viewport width.
void keepScrollContentHeight(QScrollArea * area);

// Responsive toolbar: on every resize of the toolbar host (or its parent) choose the widest label tier
// that fits the width the parent offers: 0 = icon + label everywhere, 1 = ``secondary`` icon-only,
// 2 = everything icon-only except ``always_labelled``. Buttons need an icon and keep their full name in
// the "fullText" property, tooltip and accessible name. No timers; idempotent per host.
void installResponsiveToolbar(QWidget * host, const QList<QAbstractButton *> & primary,
                              const QList<QAbstractButton *> & secondary, QAbstractButton * always_labelled);

// Studio line icons, painted (no image assets): consistent 1.6 px strokes on a 16 px grid, rendered
// for 1x and 2x so they stay sharp on high-DPI screens. Names: select, move, rotate, add, fit, mate,
// undo, redo, save, preview, view, more, reset.
QIcon studioIcon(const QString & name, const QString & colour = QString());
}  // namespace workcell_builder
