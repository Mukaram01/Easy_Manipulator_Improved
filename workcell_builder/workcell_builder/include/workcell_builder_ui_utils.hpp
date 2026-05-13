#pragma once

#include <QWidget>
#include <QLabel>

namespace workcell_builder
{
enum class StatusType
{
  Normal,
  Info,
  Warning,
  Error,
  Success
};

void applyCompactDialogDefaults(QWidget * widget);
void capDialogSize(QWidget * widget, int max_width = 1100, int max_height = 760);
void makeTextWidgetsWrap(QWidget * widget);
void applyStatusLabelStyle(QLabel * label, StatusType status);
void applyPrimarySecondaryButtonStyle(QWidget * widget);
}  // namespace workcell_builder
