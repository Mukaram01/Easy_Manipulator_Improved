// Arrangement controller against the real scripts/asset_mounting.py resolver and
// a disposable scene: one preview for every moving part, Apply commits exactly
// that request once, stale previews are refused, Cancel changes nothing.

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QTemporaryDir>

#include <memory>

#include "gui/arrange_controller.hpp"

using workcell_builder::ArrangeController;

namespace
{

class QtEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    static int argc = 1;
    static char name[] = "arrange_controller_test";
    static char * argv[] = {name, nullptr};
    if (!QCoreApplication::instance()) app_ = std::make_unique<QCoreApplication>(argc, argv);
  }
  void TearDown() override { app_.reset(); }

private:
  std::unique_ptr<QCoreApplication> app_;
};

const auto * const kEnvironment = ::testing::AddGlobalTestEnvironment(new QtEnvironment);

const char * const kLayout = R"(items:
- {id: table, geometry_type: box, dimensions: [1.2, 0.8, 0.04]}
- {id: a, geometry_type: box, dimensions: [0.1, 0.1, 0.1]}
- {id: b, geometry_type: box, dimensions: [0.1, 0.1, 0.1]}
- {id: fixture, geometry_type: box, dimensions: [0.1, 0.1, 0.1]}
)";

QJsonObject row(const QString & id, double x, double y, double z)
{
  return {{"id", id}, {"pose_xyz", QJsonArray{x, y, z}}, {"pose_rpy", QJsonArray{0., 0., 0.}}};
}

struct Fixture
{
  QTemporaryDir scene;
  QJsonArray items{row("table", 1., 0., 0.7), row("a", 0.6, 0.3, 1.), row("b", 0.9, -0.2, 1.)};
  ArrangeController::Selection selection;
  QJsonArray committed;
  std::unique_ptr<ArrangeController> controller;

  QJsonObject resolve(QJsonObject request)
  {
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
    ArrangeController::Hooks hooks;
    hooks.run_resolver = [this](const QJsonObject & request) { return resolve(request); };
    hooks.commit = [this](const QJsonObject & request) {
      const auto response = resolve(request);
      if (!response.value("ok").toBool()) return false;
      committed.append(request);
      items = response.value("result").toObject().value("rows").toArray();
      return true;
    };
    hooks.current_items = [this]() { return items; };
    hooks.display_name = [](const QString & id) { return id == "table" ? QStringLiteral("Workbench") : QString(); };
    hooks.selection = [this]() { return selection; };
    hooks.moving_blocker = [](const QString & id) { return id == "fixture" ? QStringLiteral("Fixture is locked.") : QString(); };
    hooks.reference_world = [](const QString & id) {
      return id == "fixture" ? QJsonObject{{"xyz", QJsonArray{0.5, 0., 0.75}}, {"rpy", QJsonArray{0., 0., 0.}}} : QJsonObject{};
    };
    controller = std::make_unique<ArrangeController>(hooks);
  }

  double y(const QString & id) const
  {
    for (const auto & value : items) if (value.toObject().value("id").toString() == id) return value.toObject().value("pose_xyz").toArray()[1].toDouble();
    return 0.;
  }
};

}  // namespace

TEST(ArrangeController, PreviewEveryMemberThenApplyOnceAsOneTransaction)
{
  Fixture f;
  f.selection = {{"table", "a", "b"}, "table"};
  QString error;
  ASSERT_TRUE(f.controller->start("align", QJsonObject{{"rules", QJsonObject{{"y", "max"}}}}, "Align back", &error))
    << error.toStdString();
  const auto & preview = f.controller->preview();
  ASSERT_TRUE(preview.ok) << preview.error.toStdString();
  EXPECT_EQ(preview.moved.size(), 2);
  EXPECT_EQ(f.controller->overlay().value("ghost").toArray().size(), 2);
  EXPECT_TRUE(preview.summary.contains("The reference Workbench stays put")) << preview.summary.toStdString();
  EXPECT_TRUE(f.committed.isEmpty());
  const QJsonObject planned = preview.request;
  ASSERT_TRUE(f.controller->apply(&error)) << error.toStdString();
  ASSERT_EQ(f.committed.size(), 1);  // one commit for both parts
  EXPECT_EQ(f.committed[0].toObject(), planned);
  EXPECT_NEAR(f.y("a"), 0.35, 1e-9);
  EXPECT_NEAR(f.y("b"), 0.35, 1e-9);
  EXPECT_FALSE(f.controller->active());
}

TEST(ArrangeController, AvailabilityStalenessAndCancel)
{
  Fixture f;
  f.selection = {{"table"}, "table"};
  EXPECT_TRUE(f.controller->availability("align").contains("Select two or more parts"));
  f.selection = {{"table", "a"}, "table"};
  EXPECT_TRUE(f.controller->availability("distribute", QJsonObject{{"mode", "equal"}}).contains("three or more"));
  EXPECT_TRUE(f.controller->availability("distribute", QJsonObject{{"mode", "gap"}}).isEmpty());
  f.selection = {{"table", "a", "fixture"}, "table"};
  EXPECT_TRUE(f.controller->availability("align").contains("Fixture is locked"));
  f.selection = {{"fixture", "a"}, "fixture"};  // a locked reference is fine for alignment
  EXPECT_TRUE(f.controller->availability("align").isEmpty());
  EXPECT_TRUE(f.controller->availability("mount").contains("locked"));
  QString error;
  f.selection = {{"table", "a", "b"}, "table"};
  ASSERT_TRUE(f.controller->start("distribute", QJsonObject{{"axis", "x"}, {"mode", "gap"}, {"gap_m", 0.02}}, "Gap", &error));
  ASSERT_TRUE(f.controller->preview().ok) << f.controller->preview().error.toStdString();
  // Something else edits the scene after the preview: Apply refuses and replans.
  f.items[1] = row("a", 0.61, 0.3, 1.);
  EXPECT_FALSE(f.controller->apply(&error));
  EXPECT_TRUE(error.contains("changed since the preview"));
  EXPECT_TRUE(f.committed.isEmpty());
  EXPECT_TRUE(f.controller->preview().ok);
  f.controller->cancel();
  EXPECT_FALSE(f.controller->active());
  EXPECT_TRUE(f.committed.isEmpty());
}
