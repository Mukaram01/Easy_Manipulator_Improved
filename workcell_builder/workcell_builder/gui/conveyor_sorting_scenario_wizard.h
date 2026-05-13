#pragma once

#include <QDialog>
#include <filesystem>

namespace Ui { class ConveyorSortingScenarioWizard; }

class ConveyorSortingScenarioWizard : public QDialog
{
  Q_OBJECT

public:
  explicit ConveyorSortingScenarioWizard(
    const std::filesystem::path & scenes_root,
    const std::filesystem::path & workspace_root,
    QWidget * parent = nullptr);
  ~ConveyorSortingScenarioWizard() override;

signals:
  void scenarioGenerated(const QString & scenarioName);

private slots:
  void onUseRecommendedLayout();
  void onResetLayout();
  void onResetDefaultRoutes();
  void onAddZone();
  void onRemoveZone();
  void onResetZones();
  void onValidateZones();
  void onAddRoute();
  void onRemoveRoute();
  void onValidateRouting();
  void onEpdModeChanged();
  void onGenerateScenario();
  void onGenerateYaml();
  void onGenerateFiles();
  void onRefreshStatus();
  void onCopyBuildCommand();
  void onCopyLaunchCommand();
  void onCopySampleEpdCommand();
  void onOpenRunConsole();

private:
  bool validatePoseField(const QString & text, const QString & fieldName, QStringList & errors) const;
  bool zoneExists(const QString & zoneName) const;
  QString generatedScenePath() const;
  void setGeneratedState(bool generated);
  void loadDefaults();
  void ensureZoneTableDefaults();
  void ensureRouteTableDefaults();
  void writeScenarioArtifacts(bool fullSet);
  void updateStatus(const QString & extra = QString());
  QString sceneName() const;

  Ui::ConveyorSortingScenarioWizard * ui_;
  std::filesystem::path scenes_root_;
  std::filesystem::path workspace_root_;
};
