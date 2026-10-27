// Align & Arrange controller against the real scripts/asset_mounting.py resolver
// and a disposable scene: previews are resolver dry runs, Apply sends exactly
// the previewed edit, Cancel and rejected requests change nothing.

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QTemporaryDir>

#include <cmath>
#include <memory>

#include "gui/align_arrange_controller.hpp"

using workcell_builder::AlignArrangeController;
using Axis = workcell_builder::AlignArrangeController::Axis;

namespace
{

class QtEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    static int argc = 1;
    static char name[] = "align_arrange_controller_test";
    static char * argv[] = {name, nullptr};
    if (!QCoreApplication::instance()) app_ = std::make_unique<QCoreApplication>(argc, argv);
  }
  void TearDown() override { app_.reset(); }

private:
  std::unique_ptr<QCoreApplication> app_;
};

const auto * const kEnvironment = ::testing::AddGlobalTestEnvironment(new QtEnvironment);

// A table (box, reviewed surface 20 mm above its origin), a block and a locked fixture.
const char * const kLayout = R"(schema_version: workcell_studio_layout/v1
items:
- {id: table, geometry_type: box, dimensions: [1.2, 0.8, 0.04], surface_z_m: 0.02}
- {id: block, geometry_type: box, dimensions: [0.2, 0.1, 0.1]}
- {id: fixture, geometry_type: box, dimensions: [0.1, 0.1, 0.1]}
- {id: zone, type: place_zone, geometry_type: box, dimensions: [0.2, 0.2, 0.1]}
)";

struct Fixture
{
  QTemporaryDir scene;
  QJsonArray items{
    QJsonObject{{"id", "table"}, {"pose_xyz", QJsonArray{1., 0., 0.7}}, {"pose_rpy", QJsonArray{0., 0., 0.}},
      {"mounting_yaml", "{surface_z_m: 0.02}"}},
    QJsonObject{{"id", "block"}, {"pose_xyz", QJsonArray{0.3, 0.3, 1.}}, {"pose_rpy", QJsonArray{0., 0., 0.}}},
  };
  QJsonArray committed;
  int resolver_calls{0};
  std::unique_ptr<AlignArrangeController> controller;

  QJsonObject resolve(QJsonObject request)
  {
    ++resolver_calls;
    request.insert("items", items);
    request.insert("scene_dir", scene.path());
    QProcess process;
    process.start("python3", {QDir(QStringLiteral(WORKCELL_REPO_ROOT)).filePath("scripts/asset_mounting.py")});
    process.waitForStarted(5000);
    process.write(QJsonDocument(request).toJson(QJsonDocument::Compact));
    process.closeWriteChannel();
    process.waitForFinished(20000);
    return QJsonDocument::fromJson(process.readAllStandardOutput()).object();
  }

  Fixture()
  {
    QDir(scene.path()).mkpath("layout");
    QFile layout(QDir(scene.path()).filePath("layout/workcell_studio_layout.yaml"));
    layout.open(QIODevice::WriteOnly);
    layout.write(kLayout);
    layout.close();
    AlignArrangeController::Hooks hooks;
    hooks.run_resolver = [this](const QJsonObject & request) { return resolve(request); };
    hooks.commit = [this](const QJsonObject & request) {
      const auto response = resolve(request);
      if (!response.value("ok").toBool()) return false;
      committed.append(request);
      items = response.value("result").toArray();
      return true;
    };
    hooks.current_items = [this]() { return items; };
    hooks.display_name = [](const QString & id) { return id == "table" ? QStringLiteral("Workbench") : QString(); };
    hooks.moving_blocker = [](const QString & id) {
      return id == "fixture" ? QStringLiteral("Fixture is locked. Unlock it before moving it.") : QString();
    };
    hooks.reference_blocker = [](const QString & id, QJsonObject * world) {
      if (id == "zone") return QStringLiteral("zone is a task zone, not a physical part.");
      if (id == "fixture" && world) *world = QJsonObject{{"xyz", QJsonArray{0.8, -0.2, 0.75}}, {"rpy", QJsonArray{0., 0., 0.}}};
      return QString();
    };
    controller = std::make_unique<AlignArrangeController>(hooks);
  }

  QJsonArray xyz(const QString & id) const
  {
    for (const auto & value : items) if (value.toObject().value("id").toString() == id) return value.toObject().value("pose_xyz").toArray();
    return {};
  }
};

}  // namespace

TEST(AlignArrangeController, CentreOnTopWithOffsetPreviewsThenAppliesExactly)
{
  Fixture f;
  QString error;
  ASSERT_TRUE(f.controller->begin("block", &error)) << error.toStdString();
  EXPECT_FALSE(f.controller->preview().ok);
  f.controller->set_reference("table");
  EXPECT_FALSE(f.controller->preview().ok);  // no operation yet: nothing planned
  EXPECT_TRUE(f.controller->status().contains("Centre, Align or Place on top"));
  f.controller->centre_on_reference();
  f.controller->set_rule(Axis::Z, "on_top");
  f.controller->set_offset_mm(Axis::X, 25.);
  const auto & preview = f.controller->preview();
  ASSERT_TRUE(preview.ok) << preview.error.toStdString();
  const auto target = preview.report.value("target_world").toObject().value("xyz").toArray();
  EXPECT_NEAR(target[0].toDouble(), 1.025, 1e-9);
  EXPECT_NEAR(target[1].toDouble(), 0., 1e-9);
  EXPECT_NEAR(target[2].toDouble(), 0.77, 1e-9);  // verified surface (0.72) + half the block
  EXPECT_TRUE(preview.summary.contains("Orientation unchanged")) << preview.summary.toStdString();
  EXPECT_TRUE(preview.summary.contains("+725.0")) << preview.summary.toStdString();
  EXPECT_TRUE(preview.report.value("sources").toObject().value("top").toString().contains("verified surface datum"));
  EXPECT_EQ(f.controller->overlay().value("ghost").toArray().size(), 1);
  EXPECT_TRUE(f.committed.isEmpty());  // previews never write
  EXPECT_NEAR(f.xyz("block")[0].toDouble(), 0.3, 1e-12);
  const QJsonObject previewed = preview.request;
  ASSERT_TRUE(f.controller->apply(&error)) << error.toStdString();
  ASSERT_EQ(f.committed.size(), 1);
  EXPECT_EQ(f.committed[0].toObject(), previewed);
  EXPECT_FALSE(previewed.contains("parent_object"));  // Align once: no relation
  EXPECT_NEAR(f.xyz("block")[0].toDouble(), 1.025, 1e-9);
  EXPECT_NEAR(f.xyz("block")[2].toDouble(), 0.77, 1e-9);
  EXPECT_FALSE(f.controller->active());
}

TEST(AlignArrangeController, RuleToggleKeepRelativeAndLockedReference)
{
  Fixture f;
  QString error;
  ASSERT_TRUE(f.controller->begin("block", &error));
  f.controller->set_reference("table");
  f.controller->set_rule(Axis::X, "min");
  EXPECT_EQ(f.controller->rule(Axis::X), "min");
  f.controller->set_rule(Axis::X, "min");  // the same rule again clears it
  EXPECT_TRUE(f.controller->rule(Axis::X).isEmpty());
  f.controller->set_rule(Axis::Y, "max");
  f.controller->set_keep_relative(true);
  ASSERT_TRUE(f.controller->preview().ok) << f.controller->preview().error.toStdString();
  EXPECT_EQ(f.controller->preview().request.value("parent_object").toString(), "table");
  // A locked reference: alignment works, keeping relative to it does not.
  f.controller->set_reference("fixture");
  EXPECT_FALSE(f.controller->keep_relative());
  EXPECT_FALSE(f.controller->keep_relative_blocker().isEmpty());
  f.controller->set_keep_relative(true);
  EXPECT_FALSE(f.controller->keep_relative());
  ASSERT_TRUE(f.controller->preview().ok) << f.controller->preview().error.toStdString();
  const auto target = f.controller->preview().report.value("target_world").toObject().value("xyz").toArray();
  EXPECT_NEAR(target[1].toDouble(), -0.2 + 0.05 - 0.05, 1e-9);  // back faces level along Y
}

TEST(AlignArrangeController, RejectionsAndCancelChangeNothing)
{
  Fixture f;
  QString error;
  EXPECT_FALSE(f.controller->begin(QString(), &error));
  EXPECT_TRUE(error.contains("Select the part to move"));
  EXPECT_FALSE(f.controller->begin("fixture", &error));
  EXPECT_TRUE(error.contains("locked"));
  ASSERT_TRUE(f.controller->begin("block", &error));
  f.controller->set_reference("block");
  EXPECT_TRUE(f.controller->reference().isEmpty());
  EXPECT_TRUE(f.controller->status().contains("part being moved"));
  f.controller->set_reference("zone");
  EXPECT_TRUE(f.controller->reference().isEmpty());
  EXPECT_TRUE(f.controller->status().contains("task zone"));
  EXPECT_FALSE(f.controller->apply(&error));
  f.controller->set_reference("table");
  f.controller->set_rule(Axis::X, "centre");
  f.controller->set_offset_mm(Axis::X, 20000.);  // beyond the 10 m offset limit
  EXPECT_FALSE(f.controller->preview().ok);
  EXPECT_TRUE(f.controller->preview().error.contains("offsets must be within")) << f.controller->preview().error.toStdString();
  EXPECT_FALSE(f.controller->apply(&error));
  f.controller->set_offset_mm(Axis::X, 0.);
  ASSERT_TRUE(f.controller->preview().ok);
  bool finished_applied = true;
  QObject::connect(f.controller.get(), &AlignArrangeController::finished, [&](bool applied, const QString &) { finished_applied = applied; });
  f.controller->cancel();
  EXPECT_FALSE(finished_applied);
  EXPECT_FALSE(f.controller->active());
  EXPECT_TRUE(f.committed.isEmpty());
  EXPECT_NEAR(f.xyz("block")[0].toDouble(), 0.3, 1e-12);
}
