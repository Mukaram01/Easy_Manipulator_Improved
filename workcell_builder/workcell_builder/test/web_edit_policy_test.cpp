#include <gtest/gtest.h>

#include "gui/web_edit_policy.hpp"

using workcell_builder::WebEditDisposition;
using workcell_builder::WebEditFacts;
using workcell_builder::classify_web_edit;

namespace
{
WebEditFacts authored(const char * id)
{
  WebEditFacts f; f.item_id = id; f.canvas_found = true; f.source_layer = "editable_layout"; return f;
}
}  // namespace

TEST(WebEditPolicy, ValidAuthoredItemIsPersisted)
{
  QString reason;
  EXPECT_EQ(classify_web_edit(authored("target_bin_default"), &reason), WebEditDisposition::Persist);
  EXPECT_TRUE(reason.isEmpty());
}

TEST(WebEditPolicy, KnownDerivedRecordWithoutLayoutEntryIsSkippedNotFailed)
{
  WebEditFacts f; f.item_id = "default_drop_zone"; f.known_derived = true;
  QString reason;
  EXPECT_EQ(classify_web_edit(f, &reason), WebEditDisposition::SkipDerived);
  EXPECT_TRUE(reason.contains("default_drop_zone"));
}

TEST(WebEditPolicy, UnknownIdFailsClosedEvenIfItLooksDerivedElsewhere)
{
  WebEditFacts f; f.item_id = "ghost_item";
  QString reason;
  EXPECT_EQ(classify_web_edit(f, &reason), WebEditDisposition::Reject);
  EXPECT_TRUE(reason.contains("unknown to the authored layout"));
  // A derived record that DOES have a layout entry is never skipped: it is an authored item.
  WebEditFacts present = authored("place_zone_default"); present.known_derived = true;
  EXPECT_EQ(classify_web_edit(present, nullptr), WebEditDisposition::Persist);
}

TEST(WebEditPolicy, LockedItemIsRejectedWithActionableReason)
{
  WebEditFacts f = authored("support_surface_table"); f.locked = true;
  QString reason;
  EXPECT_EQ(classify_web_edit(f, &reason), WebEditDisposition::Reject);
  EXPECT_TRUE(reason.contains("locked"));
}

TEST(WebEditPolicy, DeletedAndNonEditableLayerItemsAreRejected)
{
  WebEditFacts deleted = authored("old_bin"); deleted.deleted = true;
  QString reason;
  EXPECT_EQ(classify_web_edit(deleted, &reason), WebEditDisposition::Reject);
  EXPECT_TRUE(reason.contains("deleted"));
  WebEditFacts generated = authored("ur5"); generated.source_layer = "locked_generated_urdf_visual";
  EXPECT_EQ(classify_web_edit(generated, &reason), WebEditDisposition::Reject);
  EXPECT_TRUE(reason.contains("outside editable_layout"));
  WebEditFacts empty; empty.canvas_found = true;
  EXPECT_EQ(classify_web_edit(empty, &reason), WebEditDisposition::Reject);
}
