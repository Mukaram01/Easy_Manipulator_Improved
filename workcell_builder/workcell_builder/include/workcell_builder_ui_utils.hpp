#pragma once

#include <QDialog>
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

void applyCompactDialogDefaults(QDialog * dialog);
void capDialogSize(QWidget * dialog, int max_width = 1100, int max_height = 760);
void makeTextWidgetsWrap(QWidget * dialog);
void applyStatusLabelStyle(QLabel * label, StatusType status);
void applyPrimarySecondaryButtonStyle(QWidget * dialog);
}  // namespace workcell_builder
