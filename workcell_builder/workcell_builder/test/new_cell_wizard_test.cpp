#include <gtest/gtest.h>

#include <QApplication>
#include <QComboBox>
#include <QTableWidget>
#include <QLabel>
#include <QListWidget>
#include <QMap>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLineEdit>
#include <QFormLayout>
#include <QPushButton>

#include "gui/new_cell_wizard.h"

namespace {
QApplication * ensure_application()
{
  if (auto * app = qobject_cast<QApplication *>(QCoreApplication::instance())) return app;
  if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
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

TEST(NewCellWizard, RecommendedLayoutDefaults)
{
  const auto items = NewCellWizard::recommended_environment_assets();
  EXPECT_TRUE(items.contains("workbench_01"));
  EXPECT_TRUE(items.contains("source_bin_01"));
  EXPECT_TRUE(items.contains("place_fixture_01"));
  EXPECT_TRUE(items.contains("camera_01"));
  EXPECT_TRUE(items.contains("safety_zone_01"));
}

namespace {
QStringList combo_items(QComboBox * combo)
{
  QStringList items;
  for (int i = 0; i < combo->count(); ++i) items << combo->itemText(i);
  return items;
}
QComboBox * advanced(NewCellWizard & wizard, const QString & group, const QString & row)
{
  for (auto * box : wizard.findChildren<QGroupBox *>()) {
    if (box->title() != group) continue;
    auto * form = box->findChild<QFormLayout *>();
    for (int r = 0; form && r < form->rowCount(); ++r) {
      auto * label = qobject_cast<QLabel *>(form->itemAt(r, QFormLayout::LabelRole)->widget());
      if (label && label->text() == row) return qobject_cast<QComboBox *>(form->itemAt(r, QFormLayout::FieldRole)->widget());
    }
  }
  return nullptr;
}
}  // namespace

TEST(NewCellWizard, ReviewedPairIsTheDefaultAndItsFramesComeFromItsProfile)
{
  ensure_application();
  NewCellWizard wizard(repository_root());
  // The only reviewed New Cell profile (catalog/workcell_studio_profiles/ur5_2f_workbench.yaml) is the default.
  EXPECT_EQ(wizard.selected_robot_id(), "ur5");
  EXPECT_EQ(wizard.selected_tool_id(), "robotiq_2f_85");
  EXPECT_EQ(wizard.equipment_combination().value("status").toString(), "PLAN_CAPABLE");
  EXPECT_EQ(wizard.equipment_combination().value("profile").toString(), "ur5_2f_workbench");
  EXPECT_TRUE(wizard.equipment_blockers().isEmpty()) << wizard.equipment_blockers().join("\n").toStdString();
  EXPECT_EQ(advanced(wizard, "Advanced Robot Frames", "Robot base link")->currentText(), "base_link");
  EXPECT_EQ(advanced(wizard, "Advanced Robot Frames", "Robot end-effector/tip link")->currentText(), "tool0");
  EXPECT_EQ(advanced(wizard, "Advanced Robot Frames", "Planning group")->currentText(), "manipulator");
  EXPECT_EQ(advanced(wizard, "Advanced Tool Frames", "End-effector attach/base link")->currentText(), "gripper_base_link");
  EXPECT_EQ(advanced(wizard, "Advanced Tool Frames", "End-effector TCP/tool link")->currentText(), "ee_palm");
  EXPECT_EQ(advanced(wizard, "Advanced Tool Frames", "End-effector type")->currentText(), "finger");
  // The tool's catalog grasp strategies; AUTO only through the reviewed task authoring.
  const auto strategies = combo_items(wizard.findChild<QComboBox *>("graspStrategyCombo"));
  EXPECT_EQ(strategies.value(0), "auto");
  EXPECT_TRUE(strategies.contains("top_2f"));
  EXPECT_FALSE(strategies.contains("suction_top_basic"));
  // No embedded vendor list: models are the catalog's, families its robot families.
  const auto models = combo_items(wizard.findChild<QComboBox *>("newCellRobotModel"));
  EXPECT_TRUE(models.contains("Universal Robots UR5"));
  EXPECT_FALSE(models.contains("UR10"));
  EXPECT_TRUE(wizard.review_text().contains("PLAN_CAPABLE"));
  // Physical layout of the reviewed profile, with its test objects for sorting.
  ASSERT_TRUE(wizard.select_scenario_by_id("multi_bin_sorting_cell"));
  EXPECT_EQ(wizard.findChild<QTableWidget *>("testObjectsTable")->rowCount(), 3);
}

TEST(NewCellWizard, DiscoveredPairsAreClassifiedAndUnsupportedOnesFailClosed)
{
  ensure_application();
  NewCellWizard wizard(repository_root());
  auto * name = wizard.findChild<QLineEdit *>("newCellSceneName");
  ASSERT_NE(name, nullptr);
  name->setText("dynamic_wizard_unit_never_created");
  QPushButton * create = nullptr;
  for (auto * button : wizard.findChildren<QPushButton *>()) if (button->text() == "Create Cell") create = button;
  ASSERT_NE(create, nullptr);
  EXPECT_TRUE(create->isEnabled());

  // Installed and compatible, but no reviewed runtime binding: authoring only, with the tool's own frames.
  ASSERT_TRUE(wizard.select_equipment("ur5", "onrobot_airpick_style"));
  EXPECT_EQ(wizard.equipment_combination().value("status").toString(), "CONFIGURABLE");
  EXPECT_TRUE(wizard.equipment_blockers().isEmpty());
  EXPECT_TRUE(create->isEnabled());
  EXPECT_EQ(advanced(wizard, "Advanced Tool Frames", "End-effector TCP/tool link")->currentText(), "gripper_base_link");
  EXPECT_EQ(combo_items(wizard.findChild<QComboBox *>("graspStrategyCombo")), QStringList{"suction_top_basic"});
  EXPECT_TRUE(wizard.review_text().contains("Authoring only"));

  // Not compatible by its capability record: fails closed with the reason.
  ASSERT_TRUE(wizard.select_equipment("ur5", "suction_basic"));
  EXPECT_EQ(wizard.equipment_combination().value("status").toString(), "UNSUPPORTED");
  ASSERT_FALSE(wizard.equipment_blockers().isEmpty());
  EXPECT_TRUE(wizard.equipment_blockers().join(" ").contains("compatible_robot_families"));
  EXPECT_FALSE(create->isEnabled());

  // In the catalog but not installed here.
  ASSERT_TRUE(wizard.select_equipment("generic_scara_600", "robotiq_2f_85"));
  EXPECT_EQ(wizard.equipment_combination().value("status").toString(), "UNCOMMISSIONED");
  EXPECT_TRUE(wizard.equipment_blockers().join(" ").contains("not installed"));
  EXPECT_FALSE(create->isEnabled());

  // Advanced overrides are checked against the installed model and its record.
  ASSERT_TRUE(wizard.select_equipment("ur5", "robotiq_2f_85"));
  auto * tip = advanced(wizard, "Advanced Robot Frames", "Robot end-effector/tip link");
  tip->setCurrentText("flange_typo");
  EXPECT_TRUE(wizard.equipment_blockers().join(" ").contains("'flange_typo' is not declared by Universal Robots UR5"));
  EXPECT_FALSE(create->isEnabled());
  tip->setCurrentText("ee_link");
  EXPECT_TRUE(wizard.equipment_blockers().isEmpty());
  EXPECT_TRUE(create->isEnabled());
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

TEST(NewCellWizard, PhysicsMaterialsComeOnlyFromTheQualifiedCatalog)
{
  // The manual test object's physics material is chosen from qualified entries, never a free default.
  const auto materials = NewCellWizard::load_qualified_physics_materials(
    repository_root() + "/catalog/simulation/qualified_physics.yaml");
  EXPECT_EQ(materials, QStringList{"qualified_test_polymer"});
  EXPECT_TRUE(NewCellWizard::load_qualified_physics_materials("/nonexistent/qualified_physics.yaml").isEmpty());
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
  // Sorting is configured: the Pick & Place cycle per object, the reviewed two-bin layout and its test objects.
  EXPECT_TRUE(wizard.pick_place_configuration_available());
  auto *test_objects = wizard.findChild<QTableWidget *>("testObjectsTable");
  ASSERT_NE(test_objects, nullptr);
  EXPECT_EQ(test_objects->rowCount(), 3);
  EXPECT_TRUE(wizard.review_text().contains("READY"));

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

#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QStackedWidget>
#include <QScreen>
#include <QDir>
TEST(NewCellWizard, PagesScrollWithoutDisplacingNavigation) {
  auto * app = ensure_application();
  NewCellWizard wizard(repository_root());
  ASSERT_TRUE(wizard.select_scenario_by_id("static_table_pick_place"));
  ASSERT_TRUE(wizard.select_object_source_by_id("manual_simulated"));
  wizard.resize(1000, 600); wizard.show(); app->processEvents();
  EXPECT_LE(wizard.height(), 600);
  EXPECT_LE(wizard.frameGeometry().height(), wizard.screen()->availableGeometry().height());
  auto * steps = wizard.findChild<QListWidget *>("newCellWizardSteps"); ASSERT_NE(steps, nullptr);
  auto * pages = wizard.findChild<QStackedWidget *>(); ASSERT_NE(pages, nullptr);
  for (int page = 0; page < 6; ++page) {
    steps->setCurrentRow(page); app->processEvents();
    SCOPED_TRACE(page);
    EXPECT_LE(wizard.height(), 600);
    auto * scroll = qobject_cast<QScrollArea *>(pages->currentWidget()); ASSERT_NE(scroll, nullptr);
    const auto verify_actions = [&] {
      int actions = 0;
      for (auto * button : wizard.findChildren<QPushButton *>()) {
        if (!QStringList{"Back", "Next", "Create Cell", "Create and Open", "Cancel"}.contains(button->text())) continue;
        ++actions; EXPECT_TRUE(button->isVisible());
        const QRect bounds(button->mapTo(&wizard, QPoint()), button->size());
        EXPECT_TRUE(wizard.rect().contains(bounds));
        EXPECT_GT(bounds.top(), scroll->mapTo(&wizard, QPoint(0, scroll->height())).y());
      }
      EXPECT_EQ(actions, 5);
    };
    verify_actions();
    if (page == 4) { EXPECT_GT(scroll->verticalScrollBar()->maximum(), 0); EXPECT_EQ(scroll->horizontalScrollBar()->maximum(), 0); }
    scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum());
    app->processEvents(); verify_actions();
    EXPECT_EQ(wizard.selected_object_source_id(), "manual_simulated");
    const auto output = qEnvironmentVariable("WORKCELL_WIZARD_LAYOUT_EVIDENCE");
    if (!output.isEmpty()) { QDir().mkpath(output); EXPECT_TRUE(wizard.grab().save(output + QString("/page-%1.png").arg(page + 1))); }
  }
}
