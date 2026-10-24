#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QString>

class QApplication;

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
}  // namespace workcell_builder
