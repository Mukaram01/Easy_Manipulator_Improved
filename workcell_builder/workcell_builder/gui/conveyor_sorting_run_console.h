#pragma once

#include <QDialog>
#include <QProcess>
#include <QTimer>
#include <filesystem>

namespace Ui { class ConveyorSortingRunConsole; }

class ConveyorSortingRunConsole : public QDialog
{
  Q_OBJECT

public:
  explicit ConveyorSortingRunConsole(const std::filesystem::path & scene_path, const QString & scenario_name, QWidget * parent = nullptr);
  ~ConveyorSortingRunConsole() override;

  static bool isConveyorSortingScenario(const std::filesystem::path & scene_path);

private slots:
  void onRefresh();
  void onBuildScenario();
  void onLaunchPreview();
  void onStopPreview();
  void onRestartPreview();
  void onClearLog();
  void onOpenLogFolder();

  void onBuildOutput();
  void onLaunchOutput();
  void onBuildFinished(int exit_code, QProcess::ExitStatus status);
  void onLaunchFinished(int exit_code, QProcess::ExitStatus status);
  void onBuildError(QProcess::ProcessError error);
  void onLaunchError(QProcess::ProcessError error);

private:
  QString readStatusArtifact();
  QString resolveWorkspaceRoot() const;
  QString scenarioLaunchPath() const;
  QString buildCommand() const;
  QString launchCommand() const;
  bool safetyChecks(QString & reason) const;
  QString makeLogPath(const QString & prefix) const;
  void appendLog(const QString & text);
  void setBuildStatus(const QString & status);
  void setLaunchStatus(const QString & status);
  void setLastError(const QString & text);

  Ui::ConveyorSortingRunConsole * ui_;
  std::filesystem::path scene_path_;
  QString scenario_name_;
  QProcess * build_process_;
  QProcess * launch_process_;
  QFile * active_log_file_;
  QTimer status_timer_;
};
