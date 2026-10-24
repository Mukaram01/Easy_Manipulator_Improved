// Visual Mate controller against the real scripts/asset_mounting.py resolver:
// picks become datums by the shared policy, previews are resolver dry runs, and
// Apply sends exactly the previewed request. Nothing is written until Apply.

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>

#include <cmath>
#include <memory>

#include "gui/mate_tool_controller.hpp"

using workcell_builder::MatePickGeometry;
using workcell_builder::MateRawPick;
using workcell_builder::MateToolController;

namespace
{

class QtEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    static int argc = 1;
    static char name[] = "mate_tool_controller_test";
    static char * argv[] = {name, nullptr};
    if (!QCoreApplication::instance()) app_ = std::make_unique<QCoreApplication>(argc, argv);
  }
  // Before static destruction: Qt must not outlive its own globals.
  void TearDown() override { app_.reset(); }

private:
  std::unique_ptr<QCoreApplication> app_;
};

const auto * const kEnvironment = ::testing::AddGlobalTestEnvironment(new QtEnvironment);

// ur5_2f_test physical records (table support surface, standard bin, camera).
QJsonArray cell()
{
  const QJsonObject bin_mesh{{"scale", QJsonArray{0.001, 0.001, 0.001}}, {"rpy", QJsonArray{M_PI / 2, 0., M_PI / 2}},
    {"origin_offset", QJsonArray{-0.173899436593, 0., -0.1}}};
  return {
    QJsonObject{{"id", "table"}, {"pose_xyz", QJsonArray{0.71, -0.2, -0.0009}}, {"pose_rpy", QJsonArray{0., 0., 0.}},
      {"mounting_yaml", "{surface_z_m: 0.0}"}},
    QJsonObject{{"id", "bin"}, {"catalog_asset_id", "bin_standard"}, {"mesh", bin_mesh},
      {"pose_xyz", QJsonArray{0.45, 0.22, 0.1}}, {"pose_rpy", QJsonArray{0., 0., 0.3}}},
    QJsonObject{{"id", "camera"}, {"pose_xyz", QJsonArray{0.4, -0.36, 0.85}}, {"pose_rpy", QJsonArray{0., 1.57, 0.}}},
  };
}

struct Fixture
{
  QJsonArray items = cell();
  QJsonArray committed;
  std::unique_ptr<MateToolController> controller;

  QJsonObject resolve(QJsonObject request) const
  {
    request.insert("items", items);
    QProcess process;
    process.start("python3", {QDir(QStringLiteral(WORKCELL_REPO_ROOT)).filePath("scripts/asset_mounting.py")});
    process.waitForStarted(5000);
    process.write(QJsonDocument(request).toJson(QJsonDocument::Compact));
    process.closeWriteChannel();
    process.waitForFinished(10000);
    return QJsonDocument::fromJson(process.readAllStandardOutput()).object();
  }

  Fixture()
  {
    MateToolController::Hooks hooks;
    hooks.run_resolver = [this](const QJsonObject & request) { return resolve(request); };
    hooks.commit = [this](const QJsonObject & request) {
      const auto response = resolve(request);
      if (!response.value("ok").toBool()) return false;
      committed.append(request);
      items = response.value("result").toArray();
      return true;
    };
    hooks.current_items = [this]() { return items; };
    hooks.display_name = [](const QString & id) { return id == "bin" ? QStringLiteral("Destination Bin") : QString(); };
    controller = std::make_unique<MateToolController>(hooks);
  }

  static MateRawPick surface(const QString & asset, std::array<double, 3> p, std::array<double, 3> n, std::array<double, 3> ray)
  {
    MateRawPick pick;
    pick.backend = "test"; pick.asset_id = asset; pick.point_world = p; pick.normal_world = n; pick.ray_direction_world = ray;
    pick.geometry_source = "test mesh";
    return pick;
  }

  QJsonObject row(const QString & id) const
  {
    for (const auto & value : items) if (value.toObject().value("id").toString() == id) return value.toObject();
    return {};
  }
};

}  // namespace

TEST(MateToolController, BinBottomFlushOnTableTopSeatsWithoutCoordinates)
{
  Fixture f;
  QString error;
  ASSERT_TRUE(f.controller->begin("bin", &error)) << error.toStdString();
  EXPECT_EQ(f.controller->pending_asset(), "bin");
  // Seen from below: the bin bottom face.
  auto child = f.controller->pick(Fixture::surface("bin", {0.5, 0.25, 0.0}, {0., 0., 1.}, {0., 0., 1.}));
  ASSERT_TRUE(child.valid) << child.reason.toStdString();
  EXPECT_EQ(child.datum_name, "bottom");
  EXPECT_EQ(f.controller->step(), MateToolController::Step::PickParent);
  auto parent = f.controller->pick(Fixture::surface("table", {0.9, -0.1, 0.0}, {0., 0., 1.}, {0., 0., -1.}));
  ASSERT_TRUE(parent.valid) << parent.reason.toStdString();
  EXPECT_EQ(parent.datum_name, "surface");
  ASSERT_EQ(f.controller->step(), MateToolController::Step::Ready);
  EXPECT_EQ(f.controller->alignment(), "flush");
  const auto & preview = f.controller->preview();
  ASSERT_TRUE(preview.ok) << preview.error.toStdString();
  // Seated: in-plane position and yaw kept, gap and tilt removed.
  EXPECT_NEAR(preview.offset[2], 0., 1e-12);
  EXPECT_NEAR(preview.offset[3], 0., 1e-12);
  EXPECT_NEAR(preview.offset[4], 0., 1e-12);
  EXPECT_NEAR(preview.offset[5], 0.3, 1e-9);
  bool moved_bin = false;
  for (const auto & value : preview.moved) {
    const auto moved = value.toObject();
    if (moved.value("id").toString() != "bin") continue;
    moved_bin = true;
    EXPECT_NEAR(moved.value("xyz").toArray()[2].toDouble(), 0.0991, 1e-9);
    EXPECT_NEAR(moved.value("moved_m").toDouble(), 0.0009, 1e-9);
  }
  EXPECT_TRUE(moved_bin);
  EXPECT_TRUE(f.committed.isEmpty());  // preview never writes
  EXPECT_NEAR(f.row("bin").value("pose_xyz").toArray()[2].toDouble(), 0.1, 1e-12);
  const QJsonObject previewed = preview.request;
  ASSERT_TRUE(f.controller->apply(&error)) << error.toStdString();
  ASSERT_EQ(f.committed.size(), 1);
  EXPECT_EQ(f.committed[0].toObject(), previewed);  // Apply sends exactly what was previewed
  const auto stored = QJsonDocument::fromJson(f.row("bin").value("mounting_yaml").toString().toUtf8()).object();
  EXPECT_EQ(stored.value("parent_object").toString(), "table");
  EXPECT_EQ(stored.value("parent_link").toString(), "surface");
  EXPECT_EQ(stored.value("mate").toObject().value("alignment").toString(), "flush");
  EXPECT_NEAR(f.row("bin").value("pose_xyz").toArray()[2].toDouble(), 0.0991, 1e-9);
  EXPECT_FALSE(f.controller->active());
}

TEST(MateToolController, RejectedPicksAndRelationsChangeNothing)
{
  Fixture f;
  QString error;
  ASSERT_TRUE(f.controller->begin(QString(), &error));
  // A bin side wall is not a mounting surface; the bin becomes the pending part.
  const auto side = f.controller->pick(Fixture::surface("bin", {0.62, 0.22, 0.1}, {1., 0., 0.}, {-1., 0., 0.}));
  EXPECT_FALSE(side.valid);
  EXPECT_EQ(f.controller->pending_asset(), "bin");
  EXPECT_TRUE(f.controller->status().contains("Destination Bin"));
  EXPECT_FALSE(f.controller->child().has_value());
  // Manual datum, then self-mating is refused.
  ASSERT_TRUE(f.controller->choose_datum(MateToolController::Role::Child, "bin", "bottom", &error)) << error.toStdString();
  EXPECT_FALSE(f.controller->choose_datum(MateToolController::Role::Parent, "bin", "rim", &error));
  f.controller->pick(Fixture::surface("bin", {0.45, 0.22, 0.2}, {0., 0., 1.}, {0., 0., -1.}));
  EXPECT_FALSE(f.controller->parent().has_value());
  EXPECT_TRUE(f.controller->status().contains("cannot be mated to itself"));
  // Origin to plane: Flush is not allowed, the resolver rejects it with Apply blocked.
  ASSERT_TRUE(f.controller->choose_datum(MateToolController::Role::Child, "bin", "origin", &error));
  ASSERT_TRUE(f.controller->choose_datum(MateToolController::Role::Parent, "table", "surface", &error));
  EXPECT_EQ(f.controller->alignment(), "coincident");
  QString reason;
  EXPECT_FALSE(f.controller->flush_allowed(&reason));
  f.controller->set_alignment("flush");
  EXPECT_FALSE(f.controller->preview().ok);
  EXPECT_TRUE(f.controller->preview().error.contains("flush needs two plane datums"));
  EXPECT_FALSE(f.controller->apply(&error));
  EXPECT_TRUE(f.committed.isEmpty());
  f.controller->cancel();
  EXPECT_FALSE(f.controller->active());
}

TEST(MateToolController, CycleIsRejectedAndSceneChangesDropVanishedPicks)
{
  Fixture f;
  QString error;
  ASSERT_TRUE(f.controller->begin("bin", &error));
  ASSERT_TRUE(f.controller->choose_datum(MateToolController::Role::Child, "bin", "bottom", &error));
  ASSERT_TRUE(f.controller->choose_datum(MateToolController::Role::Parent, "table", "surface", &error));
  ASSERT_TRUE(f.controller->apply(&error)) << error.toStdString();
  // Table onto the bin rim: a cycle.
  ASSERT_TRUE(f.controller->begin("table", &error));
  ASSERT_TRUE(f.controller->choose_datum(MateToolController::Role::Child, "table", "surface", &error));
  ASSERT_TRUE(f.controller->choose_datum(MateToolController::Role::Parent, "bin", "rim", &error));
  EXPECT_FALSE(f.controller->preview().ok);
  EXPECT_TRUE(f.controller->preview().error.contains("cycle")) << f.controller->preview().error.toStdString();
  EXPECT_FALSE(f.controller->apply(&error));
  EXPECT_EQ(f.committed.size(), 1);
  // The bin disappears from the scene: its pick is dropped, not kept stale.
  QJsonArray without_bin;
  for (const auto & value : f.items) if (value.toObject().value("id").toString() != "bin") without_bin.append(value);
  f.items = without_bin;
  f.controller->scene_changed();
  EXPECT_FALSE(f.controller->parent().has_value());
  EXPECT_EQ(f.controller->step(), MateToolController::Step::PickParent);
}
