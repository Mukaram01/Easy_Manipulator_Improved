#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QString>

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

QString workcellStudioStyleSheet();
void applyWorkcellStudioTheme(QWidget * widget);
void applyStatusBadgeStyle(QLabel * label, StatusType status);
void applyButtonRoleStyle(QPushButton * button, ButtonRole role);
void applyCompactDialogDefaults(QWidget * widget);
void capDialogSize(QWidget * widget, int max_width = 1100, int max_height = 760);
void makeTextWidgetsWrap(QWidget * widget);
void applyStatusLabelStyle(QLabel * label, StatusType status);
void applyPrimarySecondaryButtonStyle(QWidget * widget);
}  // namespace workcell_builder

// Home-page screenshot polish is header-only so it can refine the existing dynamic
// shell without touching Scene Builder, generation, simulation, or safety logic.
#include "workcell_home_polish_v2.hpp"
