#include <gtest/gtest.h>

#include <QApplication>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QWidget>

#include "teardown_connections.hpp"

namespace
{
QApplication * ensure_app()
{
  if (QApplication::instance()) return qobject_cast<QApplication *>(QApplication::instance());
  static int argc = 0;
  static char * argv[] = {nullptr};
  static QApplication app(argc, argv);
  return &app;
}

bool owner_destructor_ran = false;
int callbacks_after_owner_destructor = 0;

// The MainWindow shape: a child scene with a selected item, wired to the owner.
class Owner : public QWidget
{
public:
  explicit Owner(bool retire) : retire_(retire)
  {
    auto * scene = new QGraphicsScene(this);
    auto * item = scene->addRect(0., 0., 1., 1.);
    item->setFlag(QGraphicsItem::ItemIsSelectable);
    item->setSelected(true);
    // Counts only: touching members here after ~Owner would itself be the bug.
    connect(scene, &QGraphicsScene::selectionChanged, this, []() {
      if (owner_destructor_ran) ++callbacks_after_owner_destructor;
    });
  }
  ~Owner() override
  {
    if (retire_) workcell_builder::retire_inbound_connections_from_descendants(this);
    owner_destructor_ran = true;
  }

private:
  bool retire_;
};
}  // namespace

TEST(TeardownConnections, ChildSceneCallsBackIntoDestroyedOwnerWithoutRetirement)
{
  ASSERT_NE(ensure_app(), nullptr);
  owner_destructor_ran = false;
  callbacks_after_owner_destructor = 0;
  delete new Owner(false);
  // Documents the Qt behaviour behind the shutdown heap corruption.
  EXPECT_GT(callbacks_after_owner_destructor, 0);
}

TEST(TeardownConnections, RetiredOwnerReceivesNoCallbacksFromDyingChildren)
{
  ASSERT_NE(ensure_app(), nullptr);
  owner_destructor_ran = false;
  callbacks_after_owner_destructor = 0;
  delete new Owner(true);
  EXPECT_EQ(callbacks_after_owner_destructor, 0);
}
