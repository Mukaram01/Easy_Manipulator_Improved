#pragma once

#include <QDialog>
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
  void onCopyBuild();
  void onCopyLaunch();
  void onCopySample();
  void onOpenSceneFolder();
  void onOpenPreviewFolder();
  void onOpenRvizConfig();

private:
  QString buildRunCommandsText() const;
  QString readStatusArtifact() const;
  QString summarizeMissingFiles() const;
  QString safetyStateText(const QString & status_text) const;

  Ui::ConveyorSortingRunConsole * ui_;
  std::filesystem::path scene_path_;
  QString scenario_name_;
};
