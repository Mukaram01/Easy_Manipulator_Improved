#include <gtest/gtest.h>

#include "scene_selection_set.hpp"

using workcell_builder::SceneSelectionSet;

TEST(SceneSelectionSet, FirstSelectedIsTheReferenceAndCtrlTogglesMembers)
{
  SceneSelectionSet set;
  set.select_only("table");
  EXPECT_EQ(set.size(), 1);
  EXPECT_EQ(set.reference(), "table");
  EXPECT_TRUE(set.toggle("bin"));
  EXPECT_TRUE(set.toggle("tray"));
  EXPECT_EQ(set.members(), (QStringList{"table", "bin", "tray"}));
  EXPECT_EQ(set.reference(), "table");
  EXPECT_EQ(set.others(), (QStringList{"bin", "tray"}));
  EXPECT_FALSE(set.toggle("bin"));  // Ctrl+click again removes it
  EXPECT_EQ(set.members(), (QStringList{"table", "tray"}));
}

TEST(SceneSelectionSet, ReferencePassesOnAndCanBeChosen)
{
  SceneSelectionSet set;
  set.select_only("table");
  set.toggle("bin");
  set.toggle("tray");
  EXPECT_TRUE(set.set_reference("tray"));
  EXPECT_FALSE(set.set_reference("camera"));  // not selected
  EXPECT_EQ(set.primary(), "tray");
  set.toggle("tray");  // removing the reference hands it to the earliest member
  EXPECT_EQ(set.reference(), "table");
  set.select_only("camera");  // a plain click replaces everything
  EXPECT_EQ(set.members(), (QStringList{"camera"}));
  EXPECT_EQ(set.reference(), "camera");
}

TEST(SceneSelectionSet, ShiftRangeAndPruneAfterSceneChanges)
{
  SceneSelectionSet set;
  set.select_only("bin");
  set.extend({"table", "bin", "tray", ""});
  EXPECT_EQ(set.members(), (QStringList{"bin", "table", "tray"}));
  EXPECT_EQ(set.reference(), "bin");
  EXPECT_FALSE(set.prune({"bin", "table", "tray"}));
  EXPECT_TRUE(set.prune({"table", "tray"}));  // the bin was deleted
  EXPECT_EQ(set.reference(), "table");
  EXPECT_TRUE(set.prune({}));
  EXPECT_TRUE(set.isEmpty());
  EXPECT_TRUE(set.reference().isEmpty());
}
