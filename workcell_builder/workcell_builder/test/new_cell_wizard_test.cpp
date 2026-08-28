#include <gtest/gtest.h>

#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QListWidget>
#include <QMap>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLineEdit>

#include "gui/new_cell_wizard.h"

namespace {
QApplication * ensure_application()
{
  if (auto * app = qobject_cast<QApplication *>(QCoreApplication::instance())) return app;
  qputenv("QT_QPA_PLATFORM", "offscreen");
  static int argc = 1;
  static char app_name[] = "workcell_new_cell_wizard_test";
  static char * argv[] = {app_name, nullptr};
  static QApplication app(argc, argv);
  return &app;
}

QString repository_root()
{
#ifdef WORKCELL_BUILDER_REPO_ROOT
  return QStringLiteral(WORKCELL_BUILDER_REPO_ROOT);
#else
  return QStringLiteral(".");
#endif
}
}  // namespace

TEST(NewCellWizard, ScenePackageNameValidation)
{
  EXPECT_TRUE(NewCellWizard::is_valid_package_name("ur5_pick_place"));
  EXPECT_FALSE(NewCellWizard::is_valid_package_name("UR5_bad"));
  EXPECT_FALSE(NewCellWizard::is_valid_package_name("bad-name"));
  EXPECT_FALSE(NewCellWizard::is_valid_package_name("1bad"));
}

TEST(NewCellWizard, DefaultGripperRpy)
{
  EXPECT_EQ(NewCellWizard::default_gripper_rpy_text().toStdString(), "-1.5708, -1.5708, 0");
}

TEST(NewCellWizard, RecommendedLayoutDefaults)
{
  const auto items = NewCellWizard::recommended_environment_assets();
  EXPECT_TRUE(items.contains("workbench_01"));
  EXPECT_TRUE(items.contains("source_bin_01"));
  EXPECT_TRUE(items.contains("place_fixture_01"));
  EXPECT_TRUE(items.contains("camera_01"));
  EXPECT_TRUE(items.contains("safety_zone_01"));
}

TEST(NewCellWizard, UrDefaults)
{
  const QString family = "Universal Robots / UR";

  EXPECT_EQ(NewCellWizard::default_robot_base_link(family, "UR5").toStdString(), "base_link");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(family, "UR5").toStdString(), "ee_link");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(family, "UR5").toStdString(), "manipulator");

  EXPECT_EQ(NewCellWizard::default_robot_base_link(family, "UR3").toStdString(), "base_link");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(family, "UR3").toStdString(), "ee_link");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(family, "UR3").toStdString(), "manipulator");

  EXPECT_EQ(NewCellWizard::default_robot_base_link(family, "UR10").toStdString(), "base_link");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(family, "UR10").toStdString(), "ee_link");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(family, "UR10").toStdString(), "manipulator");
}

TEST(NewCellWizard, FrankaDefaults)
{
  const QString family = "Franka / Panda";

  EXPECT_EQ(NewCellWizard::default_robot_base_link(family, "Panda").toStdString(), "panda_link0");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(family, "Panda").toStdString(), "panda_hand");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(family, "Panda").toStdString(), "panda_arm");
}

TEST(NewCellWizard, PlaceholderDefaults)
{
  const QString family = "Delta";
  const QString model = "delta_placeholder";

  EXPECT_EQ(NewCellWizard::default_robot_base_link(family, model).toStdString(), "base_link");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(family, model).toStdString(), "tool0_placeholder");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(family, model).toStdString(), "preview_group");
}

TEST(NewCellWizard, RobotFamilyModelMappingsAreDeterministic)
{
  const QString ur_family = "Universal Robots / UR";
  const QString panda_family = "Franka / Panda";

  EXPECT_EQ(NewCellWizard::default_robot_base_link(ur_family, "UR10").toStdString(), "base_link");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(ur_family, "UR10").toStdString(), "ee_link");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(ur_family, "UR10").toStdString(), "manipulator");

  EXPECT_EQ(NewCellWizard::default_robot_base_link(panda_family, "FR3").toStdString(), "panda_link0");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(panda_family, "FR3").toStdString(), "panda_hand");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(panda_family, "FR3").toStdString(), "panda_arm");

  EXPECT_EQ(NewCellWizard::default_robot_tip_link("Custom / Placeholder", "my_placeholder_robot").toStdString(), "tool0_placeholder");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group("Custom / Placeholder", "my_placeholder_robot").toStdString(), "preview_group");
}

TEST(NewCellWizard, RobotiqDefaults)
{
  EXPECT_EQ(NewCellWizard::default_end_effector_attach_link("robotiq_85").toStdString(), "gripper_base_link");
  EXPECT_EQ(NewCellWizard::default_end_effector_tcp_link("robotiq_85").toStdString(), "ee_palm");
}

TEST(NewCellWizard, SuctionDefaults)
{
  EXPECT_EQ(NewCellWizard::default_end_effector_attach_link("single_suction").toStdString(), "tool_mount_link");
  EXPECT_EQ(NewCellWizard::default_end_effector_tcp_link("single_suction").toStdString(), "tcp_link");
  EXPECT_EQ(NewCellWizard::default_end_effector_type("single_suction").toStdString(), "suction");
}

TEST(NewCellWizard, ScaffoldReadinessClassification)
{
  EXPECT_EQ(NewCellWizard::default_end_effector_family_readiness("Custom / Placeholder").toStdString(), "SCAFFOLD");
  EXPECT_EQ(NewCellWizard::default_end_effector_family_readiness("Robotiq").toStdString(), "READY");
}

TEST(NewCellWizard, IndustrialScenarioCatalogDrivesCanonicalChoices)
{
  const auto choices = NewCellWizard::load_industrial_scenario_choices(
    repository_root() + "/catalog/scenarios/industrial_scenarios.yaml");
  QMap<QString, QString> labels_by_id;
  for (const auto & choice : choices) labels_by_id.insert(choice.id, choice.label);

  EXPECT_EQ(labels_by_id.value("static_table_pick_place"), "Pick & Place");
  EXPECT_EQ(labels_by_id.value("palletizing_depalletizing_light"), "Palletizing / Depalletizing");
  EXPECT_EQ(labels_by_id.value("multi_bin_sorting_cell"), "Sorting");
  EXPECT_EQ(labels_by_id.value("stacking"), "Stacking");
  EXPECT_EQ(labels_by_id.value("custom_blank"), "Custom / Blank");
  EXPECT_GE(choices.size(), 10);
}

TEST(NewCellWizard, ScenarioSelectionIsCanonicalAcrossTaskIntentAndReview)
{
  ensure_application();
  NewCellWizard wizard(repository_root());
  auto * scenario_combo = wizard.findChild<QComboBox *>("applicationScenarioCombo");
  auto * derived_task_scenario = wizard.findChild<QLabel *>("derivedTaskScenario");
  auto * steps = wizard.findChild<QListWidget *>("newCellWizardSteps");
  ASSERT_NE(scenario_combo, nullptr);
  ASSERT_NE(derived_task_scenario, nullptr);
  ASSERT_NE(steps, nullptr);
  EXPECT_EQ(wizard.selected_scenario_id(), "static_table_pick_place");
  EXPECT_TRUE(wizard.pick_place_configuration_available());

  ASSERT_TRUE(wizard.select_scenario_by_id("palletizing_depalletizing_light"));
  steps->setCurrentRow(4);
  steps->setCurrentRow(5);
  EXPECT_EQ(wizard.selected_scenario_id(), "palletizing_depalletizing_light");
  EXPECT_TRUE(derived_task_scenario->text().contains("palletizing_depalletizing_light"));
  EXPECT_TRUE(wizard.review_text().contains("Palletizing / Depalletizing"));
  EXPECT_TRUE(wizard.review_text().contains("palletizing_depalletizing_light"));
  EXPECT_FALSE(wizard.pick_place_configuration_available());

  ASSERT_TRUE(wizard.select_scenario_by_id("multi_bin_sorting_cell"));
  steps->setCurrentRow(0);
  steps->setCurrentRow(4);
  steps->setCurrentRow(5);
  EXPECT_EQ(wizard.selected_scenario_id(), "multi_bin_sorting_cell");
  EXPECT_TRUE(derived_task_scenario->text().contains("multi_bin_sorting_cell"));
  EXPECT_TRUE(wizard.review_text().contains("Sorting"));
  EXPECT_TRUE(wizard.review_text().contains("multi_bin_sorting_cell"));

  EXPECT_EQ(wizard.findChildren<QComboBox *>("taskFamilyCombo").size(), 0);
  EXPECT_EQ(scenario_combo->currentData().toString(), wizard.selected_scenario_id());
}

TEST(NewCellWizard, PickPlaceConfigurationRemainsAvailable)
{
  ensure_application();
  NewCellWizard wizard(repository_root());
  ASSERT_TRUE(wizard.select_scenario_by_id("static_table_pick_place"));
  EXPECT_TRUE(wizard.pick_place_configuration_available());
  EXPECT_TRUE(wizard.review_text().contains("Pick & Place"));
  EXPECT_TRUE(wizard.review_text().contains("static_table_pick_place"));
}

TEST(NewCellWizard, DynamicObjectSourcesDoNotRequireManualGeometry)
{
  ensure_application();
  NewCellWizard wizard(repository_root());
  ASSERT_TRUE(wizard.select_scenario_by_id("static_table_pick_place"));
  auto * dimension_x = wizard.findChild<QDoubleSpinBox *>("manualObjectDimensionX");
  auto * notice = wizard.findChild<QLabel *>("dynamicObjectNotice");
  ASSERT_NE(dimension_x, nullptr);
  ASSERT_NE(notice, nullptr);
  dimension_x->setValue(0.0);

  ASSERT_TRUE(wizard.select_object_source_by_id("live_epd_realsense"));
  EXPECT_TRUE(wizard.manual_object_geometry_valid());
  EXPECT_TRUE(notice->text().contains("Fixed manual dimensions are not required"));
  EXPECT_TRUE(wizard.review_text().contains("Live EPD / RealSense"));

  ASSERT_TRUE(wizard.select_object_source_by_id("recorded_perception"));
  EXPECT_TRUE(wizard.manual_object_geometry_valid());
  EXPECT_TRUE(wizard.review_text().contains("Recorded / replayed perception"));
}

TEST(NewCellWizard, ManualObjectSourceRequiresFallbackGeometry)
{
  ensure_application();
  NewCellWizard wizard(repository_root());
  ASSERT_TRUE(wizard.select_object_source_by_id("manual_simulated"));
  auto * manual_card = wizard.findChild<QGroupBox *>("manualObjectCard");
  auto * dimension_x = wizard.findChild<QDoubleSpinBox *>("manualObjectDimensionX");
  auto * object_id = wizard.findChild<QLineEdit *>("manualObjectId");
  ASSERT_NE(manual_card, nullptr);
  ASSERT_NE(dimension_x, nullptr);
  ASSERT_NE(object_id, nullptr);
  EXPECT_FALSE(manual_card->isHidden());
  EXPECT_TRUE(wizard.manual_object_geometry_valid());
  dimension_x->setValue(0.0);
  EXPECT_FALSE(wizard.manual_object_geometry_valid());
  dimension_x->setValue(0.05);
  object_id->clear();
  EXPECT_FALSE(wizard.manual_object_geometry_valid());
}
